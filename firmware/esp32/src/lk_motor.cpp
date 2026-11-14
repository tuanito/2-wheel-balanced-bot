#include "lk_motor.h"
#include <cmath>
#include "sim_hil.h"

bool LkMotors::begin() {
#ifdef SIM_HIL
  simhil::begin(); return true;
#endif
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8; g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

void LkMotors::send(uint32_t id, const uint8_t d[8]) {
#ifdef SIM_HIL
  simhil::can_tx(id, d); return;
#endif
  twai_message_t m = {};
  m.identifier = id; m.data_length_code = 8;
  memcpy(m.data, d, 8);
  if (recovering_) { ++tx_drop_count; return; }
  if (twai_transmit(&m, 0) != ESP_OK) ++tx_drop_count;   // never block the control loop
}

void LkMotors::run() {
  const uint8_t d[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
  send(CAN_ID_LEFT, d); send(CAN_ID_RIGHT, d);
}

void LkMotors::off() {
  const uint8_t d[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  send(CAN_ID_LEFT, d); send(CAN_ID_RIGHT, d);
}

static int16_t to_iq(float nm) {
  float iq = nm / KT / IQ_FULL_A * IQ_MAX_CMD;
  if (iq > IQ_MAX_CMD) iq = IQ_MAX_CMD;
  if (iq < -IQ_MAX_CMD) iq = -IQ_MAX_CMD;
  return (int16_t)lrintf(iq);
}

void LkMotors::torque(float left_nm, float right_nm) {
  const int16_t a = to_iq(DIR_LEFT * left_nm), b = to_iq(DIR_RIGHT * right_nm);
  // motor ID 1 = left, ID 2 = right
  const uint8_t d[8] = {(uint8_t)(a & 0xFF), (uint8_t)(a >> 8), (uint8_t)(b & 0xFF), (uint8_t)(b >> 8), 0, 0, 0, 0};
  send(CAN_ID_MULTI, d);
}

void LkMotors::handle(const twai_message_t& m) {
  MotorState* s = nullptr; int dir = 1;
  if (m.identifier == CAN_ID_LEFT) { s = &left; dir = DIR_LEFT; }
  else if (m.identifier == CAN_ID_RIGHT) { s = &right; dir = DIR_RIGHT; }
  if (!s || m.data_length_code < 8) return;
  const uint8_t c = m.data[0];
  // torque (0xA1) and state-2 (0x9C) replies carry [temp, iq, speed, encoder]; the replies to motor on/off (0x88/0x80)
  // only echo the command (LK manual), so they must not be parsed as state
  if (c != 0xA1 && c != 0x9C) return;
  s->temp = (int8_t)m.data[1];
  const int16_t iq = (int16_t)(m.data[2] | (m.data[3] << 8));
  const int16_t dps = (int16_t)(m.data[4] | (m.data[5] << 8));
  const uint16_t enc = (uint16_t)(m.data[6] | (m.data[7] << 8));
  // The LK manual allows a 14-, 15- or 16-bit encoder in this reply. Between two replies 2 ms apart the wheel turns at
  // most a few hundred counts, so a step larger than 8192 counts right after the previous reply is a wrap, and its size
  // tells the range. Before the first wrap, the ratio of encoder counts to the reply's own speed (dps) tells it too.
  // After a gap in the replies (CAN glitch) the step is unwrapped towards what the reported speed predicts instead.
  const uint32_t now_us = micros();
  if (s->seen) {
    int32_t d = (int32_t)enc - (int32_t)s->enc_prev;
    const float dt = (now_us - s->us_prev) * 1e-6f;
    const bool fresh = dt > 0 && dt < 0.005f;
    if (!mod_locked_ && fresh && (d > 8192 || d < -8192)) {
      const uint32_t a = (uint32_t)(d < 0 ? -d : d);
      enc_mod_ = a > 49152 ? 65536 : a > 24576 ? 32768 : 16384;         // |d| = modulus - (small motion)
      // A wrap measures the FIELD's modulus. For a 14/15/16-bit encoder that is also the counts per
      // turn, so assume it - but only provisionally: learn_range() keeps checking it against the
      // drive's own reported speed and corrects it if the sensor is finer than the field.
      mod_locked_ = true;
      if (!turn_confirmed_) {                                           // the ratio outranks the wrap
        enc_per_turn_ = enc_mod_;
        enc_locked_ = true; enc_event = (int)enc_per_turn_;
      }
    }
    const float CPT = (float)enc_per_turn_;                             // counts per mechanical turn
    const float MOD = (float)enc_mod_;                                  // where the reply field wraps
    const float pred = 0.5f * (dps + s->dps_prev) * dt / 360.f * CPT;   // counts expected from the reported speed
    d += (int32_t)lrintf((pred - d) / MOD) * (int32_t)enc_mod_;         // whole field wraps that put d closest to pred
    // Runs whether or not the modulus is locked. It is the only thing that can tell 65536 counts per
    // turn from an 18-bit sensor sending the low 16 bits of 262144, and that difference is a silent
    // 4x error in every angle, speed and odometry reading.
    if (fresh) learn_range(d, (float)dps, dt);
    s->raw += d * (2.f * PI / CPT);
  }
  s->dps_prev = dps;
  s->enc_prev = enc; s->us_prev = now_us; s->seen = true;
  s->angle = dir * s->raw;
  s->speed = dir * dps * (PI / 180.f);
  s->current = dir * iq * IQ_FULL_A / IQ_MAX_CMD;
  s->last_ms = millis();
}

void LkMotors::learn_range(int32_t d, float dps, float dt) {
  if (dt <= 0 || dt > 0.02f || std::fabs(dps) < 60.f) return;          // needs real motion
  sum_counts_ += (float)(d < 0 ? -d : d); sum_deg_ += std::fabs(dps) * dt;
  // A quarter turn to decide the first time (motion before that is counted at the starting guess: a
  // one-off offset, no jump). A full turn to OVERRIDE a value already in use, so a stretch of noise
  // or wheelspin cannot move it once the robot is running on it.
  const float need = turn_confirmed_ ? 360.f : 90.f;
  if (sum_deg_ < need) return;
  const float per_turn = sum_counts_ / sum_deg_ * 360.f;                // counts per mechanical turn
  // 262144 is an 18-bit sensor reporting the low 16 bits of its count: the field still wraps at
  // 65536, four times a turn, so only this ratio against the drive's own speed can see it.
  const uint32_t cand[4] = {16384, 32768, 65536, 262144};
  const float tol = turn_confirmed_ ? 0.08f : 0.15f;                    // stricter to override than to set
  for (uint32_t c : cand)
    if (std::fabs(per_turn / c - 1.f) < tol) {
      if (c != enc_per_turn_) enc_event = (int)c;
      enc_per_turn_ = c; enc_locked_ = true; turn_confirmed_ = true;
      // The reply field holds at most 16 bits. A turn that fits in it wraps once a turn, so the
      // modulus IS the counts per turn; a finer sensor wraps the field several times a turn and the
      // modulus stays 65536. A wrap we actually measured beats this inference.
      if (!mod_locked_) enc_mod_ = enc_per_turn_ < 65536u ? enc_per_turn_ : 65536u;
      sum_counts_ = sum_deg_ = 0;                                       // keep watching, from scratch
      return;
    }
  sum_counts_ = sum_deg_ = 0;                                           // inconsistent (slip, noise): start over
}

void LkMotors::poll() {
  twai_message_t m;
#ifdef SIM_HIL
  simhil::CanFrame f;
  simhil::pump();
  while (simhil::can_rx(f)) { m = {}; m.identifier = f.id; m.data_length_code = f.dlc; memcpy(m.data, f.d, 8); handle(m); }
  return;
#endif
  service(millis());
  while (twai_receive(&m, 0) == ESP_OK) handle(m);
}

void LkMotors::service(uint32_t now) {
#ifdef SIM_HIL
  (void)now; return;
#else
  now_ms_ = now;
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF && !recovering_) {
    // Bus-off: the controller has stopped sending. Ask for recovery (it needs 128 x 11 recessive bits, ~1.4 ms at
    // 1 Mbit/s, then the controller sits in STOPPED). Meanwhile nothing is sent; healthy() turns false after
    // CAN_TIMEOUT_MS without replies, which disarms the robot through the usual motor-fault path.
    recovering_ = true; bus_off_ms_ = now; ++bus_off_count; can_event = 1;
    twai_initiate_recovery();
  } else if (recovering_ && st.state == TWAI_STATE_STOPPED) {
    if (twai_start() == ESP_OK) { recovering_ = false; recovered_after_ms = now - bus_off_ms_; can_event = 2; }
  } else if (st.state == TWAI_STATE_STOPPED && !recovering_) {
    twai_start();                       // stopped for another reason (should not happen): start again
  }
#endif
}

void LkMotors::request_state() {
  const uint8_t d[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  send(CAN_ID_LEFT, d); send(CAN_ID_RIGHT, d);
}

bool LkMotors::healthy(uint32_t now) const {
  // signed age: a reply stamped after `now` was read (same cycle) has age <= 0, not 4 billion ms
  const int32_t al = (int32_t)(now - left.last_ms), ar = (int32_t)(now - right.last_ms);
  return left.seen && right.seen && al < (int32_t)CAN_TIMEOUT_MS && ar < (int32_t)CAN_TIMEOUT_MS;
}
