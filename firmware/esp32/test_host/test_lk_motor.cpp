// Unit tests for src/lk_motor.cpp (LK MF6015 CAN protocol), built on the PC against a fake TWAI driver.
// Run: ./run_tests.sh
#include <cstdio>
#include <cmath>
#include <string>
#include "lk_motor.h"

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { ++checks; if (!(cond)) { ++fails; std::printf("  FAIL %s:%d  ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)
#define NEAR(a, b, tol) (std::fabs((double)(a) - (double)(b)) <= (tol))

static void reset() { fake::tx.clear(); fake::rx.clear(); fake::us = 1000000; fake::tx_queue_free = 1000;
  fake::installed = false; fake::started = false; fake::state = TWAI_STATE_STOPPED; fake::recoveries = fake::starts = 0; }

// LK reply [cmd, temp, iq lo/hi, dps lo/hi, enc lo/hi] from motor `id`
static void reply(uint32_t id, uint8_t cmd, int8_t temp, int16_t iq, int16_t dps, uint16_t enc, uint8_t dlc = 8) {
  twai_message_t m{}; m.identifier = id; m.data_length_code = dlc;
  const uint8_t d[8] = {cmd, (uint8_t)temp, (uint8_t)(iq & 0xFF), (uint8_t)((uint16_t)iq >> 8), (uint8_t)(dps & 0xFF),
                        (uint8_t)((uint16_t)dps >> 8), (uint8_t)(enc & 0xFF), (uint8_t)(enc >> 8)};
  memcpy(m.data, d, 8); fake::rx.push_back(m);
}
static int16_t i16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }

// spin one motor at `dps` (its own direction) with an encoder of `bits`, `n` replies 2 ms apart, starting at enc0
static uint16_t spin(LkMotors& mm, uint32_t id, int bits, float dps, int n, double& enc_f) {
  const double range = (double)(1 << bits);
  uint16_t e = 0;
  for (int i = 0; i < n; ++i) {
    enc_f += dps * 0.002 / 360.0 * range;
    double w = std::fmod(enc_f, range); if (w < 0) w += range;
    e = (uint16_t)((uint32_t)std::lround(w) % (uint32_t)range);
    reply(id, 0xA1, 30, 0, (int16_t)std::lround(dps), e);
    fake::us += 2000; mm.poll();
  }
  return e;
}

static void test_commands() {
  std::printf("commands\n");
  reset(); LkMotors m; CHECK(m.begin(), "begin");
  m.run();
  CHECK(fake::tx.size() == 2 && fake::tx[0].identifier == CAN_ID_LEFT && fake::tx[1].identifier == CAN_ID_RIGHT, "run -> 2 frames to 0x141/0x142");
  CHECK(fake::tx[0].data[0] == 0x88 && fake::tx[0].data_length_code == 8, "run = 0x88, DLC 8");
  fake::tx.clear(); m.off();
  CHECK(fake::tx.size() == 2 && fake::tx[0].data[0] == 0x80 && fake::tx[1].data[0] == 0x80, "off = 0x80 to both");
  fake::tx.clear(); m.request_state();
  CHECK(fake::tx.size() == 2 && fake::tx[0].data[0] == 0x9C, "request_state = 0x9C");
}

static void test_torque_packing() {
  std::printf("torque packing (0x280)\n");
  reset(); LkMotors m; m.begin();
  m.torque(0.2f, 0.2f);
  CHECK(fake::tx.size() == 1 && fake::tx[0].identifier == CAN_ID_MULTI, "one 0x280 frame");
  const int16_t a = i16(fake::tx[0].data), b = i16(fake::tx[0].data + 2);
  const int expect = (int)std::lround(0.2 / KT / IQ_FULL_A * IQ_MAX_CMD);   // 177
  CHECK(a == DIR_LEFT * expect, "left iq %d, expected %d (motor 1 in bytes 0-1)", a, DIR_LEFT * expect);
  CHECK(b == DIR_RIGHT * expect, "right iq %d, expected %d (motor 2 in bytes 2-3, mirrored)", b, DIR_RIGHT * expect);
  CHECK(i16(fake::tx[0].data + 4) == 0 && i16(fake::tx[0].data + 6) == 0, "motors 3/4 slots zero");
  fake::tx.clear(); m.torque(50.f, -50.f);
  CHECK(i16(fake::tx[0].data) == DIR_LEFT * IQ_MAX_CMD && i16(fake::tx[0].data + 2) == -DIR_RIGHT * IQ_MAX_CMD, "clamped to +-2048");
  fake::tx.clear(); m.torque(-0.0001f, 0.0001f);
  CHECK(i16(fake::tx[0].data) == 0 && i16(fake::tx[0].data + 2) == 0, "tiny torque rounds to 0");
}

static void test_reply_parsing() {
  std::printf("reply parsing\n");
  reset(); LkMotors m; m.begin();
  reply(CAN_ID_LEFT, 0xA1, 42, 1024, 360, 0); reply(CAN_ID_RIGHT, 0x9C, -5, -1024, -720, 0);
  m.poll();
  CHECK(m.left.seen && m.right.seen, "both seen");
  CHECK(m.left.temp == 42 && m.right.temp == -5, "temperature int8");
  CHECK(NEAR(m.left.current, DIR_LEFT * 8.25f, 1e-3), "left current %.3f A (1024 = 8.25 A)", m.left.current);
  CHECK(NEAR(m.right.current, DIR_RIGHT * -8.25f, 1e-3), "right current %.3f A, direction applied", m.right.current);
  CHECK(NEAR(m.left.speed, DIR_LEFT * 2 * PI, 1e-4), "left 360 dps = 2 pi rad/s (%.4f)", m.left.speed);
  CHECK(NEAR(m.right.speed, DIR_RIGHT * -4 * PI, 1e-4), "right -720 dps with direction (%.4f)", m.right.speed);
  CHECK(m.left.last_ms == millis(), "timestamped");
}

static void test_ignored_frames() {
  std::printf("frames that must not change the state\n");
  reset(); LkMotors m; m.begin();
  reply(CAN_ID_LEFT, 0xA1, 30, 100, 90, 1000); m.poll();
  const MotorState before = m.left;
  fake::us += 5000;
  reply(CAN_ID_LEFT, 0x88, 0, 0, 0, 0);            // motor-on echo (LK manual: reply = the command)
  reply(CAN_ID_LEFT, 0x80, 0, 0, 0, 0);            // motor-off echo
  reply(0x143, 0xA1, 0, 0, 0, 0);                   // other motor ID
  reply(CAN_ID_LEFT, 0xA1, 0, 0, 0, 0, 6);          // short frame
  m.poll();
  CHECK(m.left.angle == before.angle && m.left.speed == before.speed && m.left.last_ms == before.last_ms,
        "0x88/0x80 echoes, foreign IDs and short frames ignored");
}

// Spin with an 18-bit sensor whose count is reported through the 16-bit field: counts per turn is
// 262144, the field still wraps at 65536, so it wraps four times a turn. The wrap detector alone
// reads that as a 16-bit encoder and every angle comes out 4x too small, silently.
static void spin18(LkMotors& mm, uint32_t id, float dps, int n, double& enc_f) {
  const double per_turn = 262144.0;
  for (int i = 0; i < n; ++i) {
    enc_f += dps * 0.002 / 360.0 * per_turn;
    double w = std::fmod(enc_f, per_turn); if (w < 0) w += per_turn;
    const uint16_t e = (uint16_t)((uint32_t)std::lround(w) & 0xFFFFu);   // the low 16 bits only
    reply(id, 0xA1, 30, 0, (int16_t)std::lround(dps), e);
    fake::us += 2000; mm.poll();
  }
}

static void test_18bit(float dps, const char* what) {
  reset(); LkMotors m; m.begin();
  double enc = 1000.0;
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 1000); m.poll();
  for (int i = 0; i < 3000; ++i) spin18(m, CAN_ID_LEFT, dps, 1, enc);   // 6 s
  CHECK(m.enc_range() == 262144u, "%s: counts per turn %u (expected 262144)", what, m.enc_range());
  CHECK(m.enc_mod() == 65536u, "%s: field modulus %u (expected 65536)", what, m.enc_mod());
  CHECK(m.enc_turn_confirmed(), "%s: counts per turn confirmed against the reported speed", what);
  // The angle must track the truth, not a quarter of it. There is a one-off offset, as in the
  // 14/15/16-bit cases: motion before the lock is counted at the starting guess. Here the guess
  // (65536) UNDER-states counts per turn by 4x, so that motion is over-credited 4x and the offset is
  // bounded by 3 x the 90 deg learning window - not under it, as when the guess is too large.
  const float total = DIR_LEFT * dps * 0.002f * 3000 * PI / 180.f;
  CHECK(std::fabs(m.left.angle - total) < 3.f * PI / 2 + 0.05f,
        "%s: angle %.2f rad, true %.2f, offset %.2f (a 4x error would read %.2f)",
        what, m.left.angle, total, m.left.angle - total, total / 4);
  // and after the lock it must be exact: no drift, no scale error
  const float a0 = m.left.angle;
  for (int i = 0; i < 500; ++i) spin18(m, CAN_ID_LEFT, dps, 1, enc);
  const float after = DIR_LEFT * dps * 0.002f * 500 * PI / 180.f;
  CHECK(NEAR(m.left.angle - a0, after, 0.01), "%s: exact after detection: %.3f rad, expected %.3f",
        what, m.left.angle - a0, after);
}

static void test_unwrap(int bits, float dps, const char* what) {
  reset(); LkMotors m; m.begin();
  double enc = 1000.0;
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 1000); m.poll();
  int i = 0; float a_lock = 0; int i_lock = -1;
  for (; i < 3000; ++i) {                            // 6 s, one reply every 2 ms
    spin(m, CAN_ID_LEFT, bits, dps, 1, enc);
    if (i_lock < 0 && m.enc_range_known()) { i_lock = i; a_lock = m.left.angle; }
  }
  CHECK(m.enc_range_known() && m.enc_range() == (1u << bits), "%s: range %u detected (expected %u)", what, m.enc_range(), 1u << bits);
  const float after = DIR_LEFT * dps * 0.002f * (i - 1 - i_lock) * PI / 180.f;
  CHECK(i_lock >= 0 && NEAR(m.left.angle - a_lock, after, 0.01), "%s: exact after detection: %.3f rad, expected %.3f", what, m.left.angle - a_lock, after);
  const float total = DIR_LEFT * dps * 0.002f * i * PI / 180.f;
  CHECK(std::fabs(m.left.angle - total) < 0.75f * PI / 2 + 0.02f, "%s: one-off offset %.2f rad (bound: 3/4 of the 90 deg learning window)", what, m.left.angle - total);
}

static void test_ranges() {
  std::printf("encoder unwrap + range detection\n");
  for (int bits : {14, 15, 16}) {
    char w[40];
    std::snprintf(w, sizeof w, "%d-bit forward", bits); test_unwrap(bits, 720.f, w);
    std::snprintf(w, sizeof w, "%d-bit reverse", bits); test_unwrap(bits, -900.f, w);
  }
  // An 18-bit sensor through the 16-bit reply field. This is the MF7015 question: if its drive
  // reports this way, the wrap detector alone would lock 65536 and the robot would run on angles,
  // speeds and odometry that are all 4x too small, with nothing faulted.
  test_18bit(720.f, "18-bit through a 16-bit field, forward");
  test_18bit(-900.f, "18-bit through a 16-bit field, reverse");
}

static void test_ratio_before_wrap() {
  std::printf("range from the speed ratio, before any wrap\n");
  for (int bits : {14, 15, 16}) {
    reset(); LkMotors m; m.begin();
    double enc = 10.0;
    reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 10); m.poll();
    spin(m, CAN_ID_LEFT, bits, 400.f, 260, enc);     // 208 deg: no wrap yet (starts near 0)
    CHECK(m.enc_range_known() && m.enc_range() == (1u << bits), "%d-bit: %u", bits, m.enc_range());
    CHECK(m.enc_event == (bits == 16 ? 0 : (1 << bits)), "%d-bit: event %d", bits, m.enc_event);
  }
  reset(); LkMotors m; m.begin();
  double enc = 10.0;
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 10); m.poll();
  spin(m, CAN_ID_LEFT, 16, 30.f, 2000, enc);         // too slow to learn from
  CHECK(!m.enc_range_known(), "slow motion (30 dps) does not lock the range");
}

static void test_reply_gap() {
  std::printf("replies missing for a while (CAN glitch) while the wheel keeps turning\n");
  reset(); LkMotors m; m.begin();
  double enc = 1000.0;
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 1000); m.poll();
  spin(m, CAN_ID_LEFT, 16, 720.f, 400, enc);          // locks 65536 on the way
  CHECK(m.enc_range() == 65536, "locked 16-bit first (%u)", m.enc_range());
  const float a0 = m.left.angle;
  enc += 720.0 * 0.100 / 360.0 * 65536; fake::us += 100000;   // 100 ms without replies: 72 deg = 13107 counts
  spin(m, CAN_ID_LEFT, 16, 720.f, 50, enc);
  CHECK(m.enc_range() == 65536, "a gap must not change the detected range (now %u)", m.enc_range());
  const float exp = DIR_LEFT * (720.f * 0.100f + 720.f * 0.002f * 50) * PI / 180.f;
  CHECK(NEAR(m.left.angle - a0, exp, 0.05), "angle across the gap %.3f, expected %.3f", m.left.angle - a0, exp);
}

static void test_healthy() {
  std::printf("health / CAN timeout\n");
  reset(); LkMotors m; m.begin();
  CHECK(!m.healthy(millis()), "nothing seen -> unhealthy");
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 0); m.poll();
  CHECK(!m.healthy(millis()), "one motor only -> unhealthy");
  reply(CAN_ID_RIGHT, 0xA1, 30, 0, 0, 0); m.poll();
  CHECK(m.healthy(millis()), "both fresh -> healthy");
  CHECK(m.healthy(millis() - 1), "reply stamped after `now` (age -1 ms) -> healthy (was a wrap-around bug)");
  CHECK(m.healthy(millis() + CAN_TIMEOUT_MS - 1), "age %u ms -> healthy", (unsigned)CAN_TIMEOUT_MS - 1);
  CHECK(!m.healthy(millis() + CAN_TIMEOUT_MS), "age %u ms -> timeout", (unsigned)CAN_TIMEOUT_MS);
}

static void test_tx_full() {
  std::printf("full TX queue never blocks\n");
  reset(); LkMotors m; m.begin(); fake::tx_queue_free = 0;
  m.torque(1.f, 1.f); m.run(); m.off();
  CHECK(fake::tx.empty(), "frames dropped, call returned");
}

static void test_bus_off() {
  std::printf("CAN bus-off -> recovery -> restart\n");
  reset(); LkMotors m; m.begin();
  CHECK(fake::state == TWAI_STATE_RUNNING, "running after begin");
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 0); reply(CAN_ID_RIGHT, 0xA1, 30, 0, 0, 0); m.poll();
  CHECK(m.healthy(millis()), "healthy before");
  fake::state = TWAI_STATE_BUS_OFF;                        // e.g. a noise burst: TX error counter > 255
  fake::tx.clear(); m.torque(0.5f, 0.5f);
  CHECK(fake::tx.empty() && m.tx_drop_count == 1, "frame refused while bus-off, counted (%u)", m.tx_drop_count);
  fake::us += 1000; m.poll();
  CHECK(fake::recoveries == 1 && m.bus_off_count == 1 && m.can_event == 1, "recovery requested once, event 1");
  m.can_event = 0; m.torque(0.5f, 0.5f);
  CHECK(fake::tx.empty(), "nothing sent while recovering");
  fake::us += 1000; m.poll();                              // fake: recovery done -> STOPPED
  fake::us += 1000; m.poll();                              // driver restarts the controller
  CHECK(fake::state == TWAI_STATE_RUNNING && m.can_event == 2, "controller restarted, event 2 (state %d)", (int)fake::state);
  CHECK(m.recovered_after_ms >= 1 && m.recovered_after_ms <= 5, "outage %u ms", m.recovered_after_ms);
  CHECK(fake::recoveries == 1, "recovery not requested twice (%d)", fake::recoveries);
  fake::tx.clear(); m.torque(0.5f, 0.5f);
  CHECK(fake::tx.size() == 1, "sending again after recovery");
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 0); reply(CAN_ID_RIGHT, 0xA1, 30, 0, 0, 0); m.poll();
  CHECK(m.healthy(millis()), "healthy again once replies return");
  // a second bus-off is handled the same way
  fake::state = TWAI_STATE_BUS_OFF; m.poll(); m.poll(); m.poll(); m.poll();
  CHECK(m.bus_off_count == 2 && fake::state == TWAI_STATE_RUNNING, "second bus-off recovered (count %u)", m.bus_off_count);
}

static void test_bus_off_long() {
  std::printf("bus-off longer than the CAN timeout -> motor fault (disarm), then recovers\n");
  reset(); LkMotors m; m.begin();
  reply(CAN_ID_LEFT, 0xA1, 30, 0, 0, 0); reply(CAN_ID_RIGHT, 0xA1, 30, 0, 0, 0); m.poll();
  fake::state = TWAI_STATE_BUS_OFF; m.poll();
  fake::recover_polls = 1000000;                            // recovery stuck (bus shorted)
  for (int i = 0; i < 30; ++i) { fake::us += 2000; m.poll(); }
  CHECK(!m.healthy(millis()), "no replies for 60 ms -> unhealthy -> main disarms");
  CHECK(fake::recoveries == 1, "no recovery spam while stuck (%d)", fake::recoveries);
  fake::recover_polls = 1; m.poll(); m.poll();
  CHECK(fake::state == TWAI_STATE_RUNNING, "restarts when the bus clears");
}

int main() {
  test_commands(); test_torque_packing(); test_reply_parsing(); test_ignored_frames();
  test_ranges(); test_ratio_before_wrap(); test_reply_gap(); test_healthy(); test_tx_full(); test_bus_off(); test_bus_off_long();
  std::printf("\n%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
