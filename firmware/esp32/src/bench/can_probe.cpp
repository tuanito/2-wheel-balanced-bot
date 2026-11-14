// CAN bench probe (env esp32s3_canprobe). Flash it instead of the robot firmware, open the serial monitor
// (pio device monitor -e esp32s3_canprobe), follow the prompts.
//
// 1. Transceiver self-test: TWAI in no-ACK mode sends frames to itself THROUGH the SN65HVD230 (needs only the CAN board,
//    no motor, no other node). 20/20 received = ESP32 <-> CAN board wiring OK. 0 = CTX/CRX swapped, no 3.3 V, bad joint.
// 2. Bus scan: 0x9C "read state 2" to motor IDs 1-4 every 200 ms, decoded replies + bus error counters.
// 3. Keys: 1/2 select motor ID, r = motor on (0x88), o = off (0x80), + / - = +-0.2 N·m for 0.5 s (wheels OFF the ground),
//    e = encoder range test (turn the wheel slowly by hand, one full turn), s = status, h = help.
#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"

static uint8_t sel = 1;

static bool install(twai_mode_t mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, mode);
  g.tx_queue_len = 16; g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  return twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
}
static void uninstall() { twai_stop(); twai_driver_uninstall(); }

static void status(const char* tag) {
  twai_status_info_t s; twai_get_status_info(&s);
  static const char* st[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
  Serial.printf("[%s] state %s  tx_err %u rx_err %u  tx_failed %u  bus_errors %u  arb_lost %u  rx_missed %u\n", tag,
                st[s.state & 3], s.tx_error_counter, s.rx_error_counter, s.tx_failed_count, s.bus_error_count, s.arb_lost_count, s.rx_missed_count);
}

static void send(uint32_t id, const uint8_t d[8]) {
  twai_message_t m = {}; m.identifier = id; m.data_length_code = 8; memcpy(m.data, d, 8);
  if (twai_transmit(&m, pdMS_TO_TICKS(5)) != ESP_OK) Serial.printf("  tx 0x%03X failed\n", (unsigned)id);
}

static void self_test() {
  Serial.println("\n== 1. transceiver self-test (no-ACK, self-reception, 1 Mbit/s) ==");
  if (!install(TWAI_MODE_NO_ACK)) { Serial.println("TWAI install failed"); return; }
  int ok = 0;
  for (int i = 0; i < 20; ++i) {
    twai_message_t m = {}; m.identifier = 0x7A0 + i; m.data_length_code = 8; m.flags = TWAI_MSG_FLAG_SELF;
    for (int k = 0; k < 8; ++k) m.data[k] = (uint8_t)(i * 13 + k);
    if (twai_transmit(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    twai_message_t r;
    if (twai_receive(&r, pdMS_TO_TICKS(20)) == ESP_OK && r.identifier == m.identifier && !memcmp(r.data, m.data, 8)) ++ok;
  }
  status("self-test");
  Serial.printf("self-test: %d/20 frames came back through the transceiver -> %s\n", ok,
                ok == 20 ? "ESP32 <-> CAN board OK" :
                ok == 0  ? "NOTHING: check CTX->GPIO%d, CRX->GPIO%d (not swapped), 3.3 V and GND on the CAN board" : "PARTIAL: loose joint or noise");
  uninstall();
}

static void decode(const twai_message_t& m) {
  const uint8_t c = m.data[0];
  if (c == 0x9C || c == 0xA1) {
    const int16_t iq = (int16_t)(m.data[2] | (m.data[3] << 8)), dps = (int16_t)(m.data[4] | (m.data[5] << 8));
    const uint16_t enc = (uint16_t)(m.data[6] | (m.data[7] << 8));
    Serial.printf("  motor %u: %s temp %d C  iq %.2f A  speed %d dps  enc %u\n", (unsigned)(m.identifier - 0x140), c == 0x9C ? "state" : "torque",
                  (int8_t)m.data[1], iq * IQ_FULL_A / IQ_MAX_CMD, dps, enc);
  } else {
    Serial.printf("  0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X\n", (unsigned)m.identifier, m.data[0], m.data[1], m.data[2], m.data[3],
                  m.data[4], m.data[5], m.data[6], m.data[7]);
  }
}

static void scan(bool print) {
  const uint8_t d[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  bool seen[5] = {};
  for (int id = 1; id <= 4; ++id) send(0x140 + id, d);
  const uint32_t t0 = millis();
  twai_message_t m;
  while (millis() - t0 < 20) if (twai_receive(&m, pdMS_TO_TICKS(2)) == ESP_OK) {
    if (m.identifier > 0x140 && m.identifier <= 0x144) seen[m.identifier - 0x140] = true;
    if (print) decode(m);
  }
  if (print) {
    Serial.printf("  answering IDs:"); for (int id = 1; id <= 4; ++id) if (seen[id]) Serial.printf(" %d", id);
    Serial.println(seen[1] || seen[2] || seen[3] || seen[4] ? "" : " none (motor powered? ID/baud set to 1/2 @ 1 Mbit/s? H/L swapped? 60 ohm?)");
  }
}

static void torque_test(float nm) {
  const int16_t iq = (int16_t)lrintf(nm / KT / IQ_FULL_A * IQ_MAX_CMD);
  Serial.printf("\nmotor %u: %+.2f N·m (iq %d) for 0.5 s -- wheel must be OFF the ground\n", sel, nm, iq);
  const uint8_t on[8] = {0x88, 0, 0, 0, 0, 0, 0, 0}, off[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t t[8] = {0xA1, 0, 0, 0, (uint8_t)(iq & 0xFF), (uint8_t)((uint16_t)iq >> 8), 0, 0};
  send(0x140 + sel, on);
  int32_t sum_dps = 0; int n = 0;
  for (uint32_t t0 = millis(); millis() - t0 < 500;) {
    send(0x140 + sel, t);
    twai_message_t m;
    while (twai_receive(&m, pdMS_TO_TICKS(1)) == ESP_OK)
      if (m.identifier == 0x140u + sel && m.data[0] == 0xA1) { sum_dps += (int16_t)(m.data[4] | (m.data[5] << 8)); ++n; }
    delay(2);
  }
  send(0x140 + sel, off);
  const int avg = n ? sum_dps / n : 0;
  Serial.printf("  %d replies, mean speed %d dps (motor's own sign)\n", n, avg);
  Serial.printf("  -> look at the wheel: did it turn so the ROBOT would roll FORWARD? If yes, set %s = %+d in config.h,"
                " otherwise %+d. (Positive command gave %s speed.)\n", sel == 1 ? "DIR_LEFT" : "DIR_RIGHT", nm > 0 ? 1 : -1,
                nm > 0 ? -1 : 1, avg > 0 ? "positive" : avg < 0 ? "negative" : "no");
}

static void encoder_test() {
  Serial.printf("\nmotor %u encoder: turn the wheel slowly by hand, about one full turn, within 10 s...\n", sel);
  const uint8_t d[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  uint16_t lo = 65535, hi = 0; int32_t biggest = 0; uint16_t prev = 0; bool first = true;
  int wraps = 0;                       // how many times the reply FIELD wrapped during the turn
  for (uint32_t t0 = millis(); millis() - t0 < 10000;) {
    send(0x140 + sel, d);
    twai_message_t m;
    while (twai_receive(&m, pdMS_TO_TICKS(2)) == ESP_OK)
      if (m.identifier == 0x140u + sel && m.data[0] == 0x9C) {
        const uint16_t e = (uint16_t)(m.data[6] | (m.data[7] << 8));
        lo = std::min(lo, e); hi = std::max(hi, e);
        if (!first) {
          const int32_t j = abs((int32_t)e - prev);
          biggest = std::max(biggest, j);
          if (j > 8192) ++wraps;       // a jump that big in a hand turn is the field rolling over
        }
        prev = e; first = false;
      }
    delay(5);
  }
  // The field's modulus and the counts per MECHANICAL TURN are not the same number. They match for
  // a 14/15/16-bit encoder, whose whole turn fits in the reply field and so wraps it once. A finer
  // sensor reporting its low 16 bits wraps the field several times a turn - 4x for 18 bits - and
  // reading the modulus as the counts per turn makes every angle, speed and odometry figure come
  // out by that factor, with nothing faulted. So count the wraps, do not assume one.
  const uint32_t field = hi > 32767 ? 65536 : hi > 16383 ? 32768 : 16384;
  Serial.printf("  encoder min %u max %u, largest jump %ld, %d field wrap(s) in that turn\n",
                lo, hi, (long)biggest, wraps);
  Serial.printf("  -> reply field modulus %u\n", (unsigned)field);
  if (wraps <= 1) {
    Serial.printf("  -> %u counts per turn (the turn fits in the field: an ordinary %d-bit encoder)\n",
                  (unsigned)field, field == 65536 ? 16 : field == 32768 ? 15 : 14);
    if (biggest <= 8192) Serial.println("     no wrap seen: turn a full revolution to be sure");
  } else {
    Serial.printf("  -> about %u counts per turn: the sensor is FINER than the 16-bit field and wraps\n"
                  "     it %d times a turn. 4 wraps of 65536 = 262144 = an 18-bit sensor. The firmware\n"
                  "     must use this, not the modulus, or every reading is %dx too small.\n",
                  (unsigned)field * (unsigned)wraps, wraps, wraps);
  }
  Serial.println("  (the robot firmware detects this by itself from its own speed reports; this is a cross-check)");
}

static void help() {
  Serial.println("\nkeys: 1/2 motor ID | r on | o off | + / - torque test | e encoder test | s status + scan | h help");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== CAN bench probe: ESP32-S3 TWAI + SN65HVD230 + LK motors ===");
  Serial.printf("pins: CTX GPIO%d, CRX GPIO%d, 1 Mbit/s\n", PIN_CAN_TX, PIN_CAN_RX);
  self_test();
  Serial.println("\n== 2. bus scan (normal mode) ==");
  if (!install(TWAI_MODE_NORMAL)) { Serial.println("TWAI install failed"); return; }
  scan(true); status("scan");
  help();
}

void loop() {
  static uint32_t last = 0;
  if (Serial.available()) {
    const char k = Serial.read();
    if (k == '1' || k == '2') { sel = k - '0'; Serial.printf("motor %u selected\n", sel); }
    else if (k == 'r') { const uint8_t d[8] = {0x88}; send(0x140 + sel, d); Serial.println("on"); }
    else if (k == 'o') { const uint8_t d[8] = {0x80}; send(0x140 + sel, d); Serial.println("off"); }
    else if (k == '+') torque_test(+0.2f);
    else if (k == '-') torque_test(-0.2f);
    else if (k == 'e') encoder_test();
    else if (k == 's') { scan(true); status("bus"); }
    else if (k == 'h') help();
  }
  twai_status_info_t s; twai_get_status_info(&s);
  if (s.state == TWAI_STATE_BUS_OFF) { Serial.println("BUS OFF -> recovering (check wiring/termination)"); twai_initiate_recovery(); delay(100); twai_start(); }
  if (millis() - last > 2000) { last = millis(); scan(false); }
}
