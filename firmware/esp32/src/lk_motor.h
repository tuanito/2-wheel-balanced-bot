// LK (LKMTECH) motor driver over CAN (ESP32 TWAI), torque mode.
// Protocol (LK CAN V2.x): ID 0x140 + motor id. 0x88 = motor run, 0x80 = motor off, 0xA1 = torque command
// (iq int16, ±2048 ≙ ±16.5 A). Multi-motor torque command: ID 0x280, 4 × int16 iq. Every motor answers with
// [cmd, temp int8, iq int16, speed int16 (dps), encoder uint16].
#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"

struct MotorState {
  float angle = 0;       // rad, multi-turn, robot-forward positive (relative to the body)
  float speed = 0;       // rad/s, robot-forward positive
  float current = 0;     // A
  int8_t temp = 0;       // °C
  uint32_t last_ms = 0;  // last reply
  bool seen = false;
  uint16_t enc_prev = 0;
  uint32_t us_prev = 0;
  int16_t dps_prev = 0;
  float raw = 0;         // rad, multi-turn, in the motor's own direction
};

class LkMotors {
 public:
  bool begin();
  void run();                                        // 0x88 to both
  void off();                                        // 0x80 to both (free-wheel)
  void torque(float left_nm, float right_nm);        // one 0x280 frame
  void poll();                                       // read all pending replies
  void request_state();                              // 0x9C (read state 2) to both: keeps speeds/encoders flowing while off
  bool healthy(uint32_t now_ms) const;
  // CAN controller health: a bus-off (too many TX errors: noise burst, loose plug, motors unpowered while sending)
  // stops the TWAI controller for good unless the driver restarts it. service() does that and is called by poll().
  void service(uint32_t now_ms);
  uint32_t bus_off_count = 0;        // bus-off events since boot
  uint32_t tx_drop_count = 0;        // frames the TX queue refused (bus not acknowledging or busy)
  uint32_t can_event = 0;            // 1 = bus-off, recovery started; 2 = recovered and restarted (main reports, then clears)
  uint32_t recovered_after_ms = 0;   // duration of the last bus-off
  MotorState left, right;
  // Counts per MECHANICAL TURN: what converts counts to radians. 16384 / 32768 / 65536 / 262144,
  // detected at run time. NOT the same thing as the reply field's wrap modulus below - they differ
  // when a higher-resolution sensor reports its low 16 bits, which is the case this split exists for.
  uint32_t enc_range() const { return enc_per_turn_; }
  // The modulus the 16-bit reply field wraps at: what the unwrap arithmetic needs.
  uint32_t enc_mod() const { return enc_mod_; }
  bool enc_range_known() const { return enc_locked_; }
  // True once the counts-per-turn has been CONFIRMED against the drive's own reported speed, rather
  // than merely assumed equal to the wrap modulus. An 18-bit sensor truncated to the 16-bit reply
  // field wraps 4x per turn, so the wrap alone would say 65536 and every angle, speed and odometry
  // reading would come out 4x too small with nothing faulted.
  bool enc_turn_confirmed() const { return turn_confirmed_; }
  int enc_event = 0;                                  // set when the range gets locked or corrected (main reports it)
 private:
  uint32_t enc_mod_ = (uint32_t)ENC_COUNTS;        // reply field wrap modulus
  uint32_t enc_per_turn_ = (uint32_t)ENC_COUNTS;   // counts per mechanical turn
  bool mod_locked_ = false;                        // a wrap has measured the modulus directly
  bool enc_locked_ = false;                        // counts per turn is known (wrap-implied or confirmed)
  bool turn_confirmed_ = false;                    // ... and confirmed against the reported speed
  float sum_counts_ = 0, sum_deg_ = 0;                // for the speed-ratio detection
  void learn_range(int32_t d, float dps, float dt);
  void send(uint32_t id, const uint8_t d[8]);
  void handle(const twai_message_t& m);
  bool recovering_ = false;
  uint32_t bus_off_ms_ = 0;
  uint32_t now_ms_ = 0;
};
