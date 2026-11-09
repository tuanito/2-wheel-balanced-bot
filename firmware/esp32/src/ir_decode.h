// IR beacon decoding, plain C++ (unit-tested on the PC, used by ir_beacon.h on the ESP32).
//
// The dock's beacon (dock/beacon_attiny85.ino) sends 38 kHz bursts, one code per LED group, round-robin:
//   'A' all round : 600 us burst  (4 LEDs, one per column side: "the dock is that way")
//   'C' centre    : 1200 us burst (1 LED in a tube, about ±6°: "you are on the dock's axis")
// each followed by >= 2.4 ms of silence. A TSOP38238 pulls its output LOW for the length of a burst (±~150 us).
// Per receiver we count the bursts of each code over a window; >= IR_MIN_HITS in a window = heard.
#pragma once
#include <stdint.h>

namespace ir {

constexpr uint32_t A_MIN = 380, A_MAX = 900;      // us: 'A' bursts read 450-750 on a TSOP38238
constexpr uint32_t C_MIN = 950, C_MAX = 1600;     // us: 'C' 1050-1350
constexpr int IR_MIN_HITS = 3;                    // per 100 ms window (the beacon sends ~15 of each)

enum Code : uint8_t { NONE = 0, ALL_ROUND = 1, CENTRE = 2 };

inline Code classify(uint32_t low_us) {
  if (low_us >= A_MIN && low_us <= A_MAX) return ALL_ROUND;
  if (low_us >= C_MIN && low_us <= C_MAX) return CENTRE;
  return NONE;                                     // noise, remote controls, sunlight flicker
}

struct Receiver {
  uint32_t fall_us = 0;
  bool low = false;
  uint16_t hits_a = 0, hits_c = 0;
  // one call per edge of the receiver output (level after the edge, time in us)
  void edge(bool level_high, uint32_t t_us) {
    if (!level_high) { fall_us = t_us; low = true; return; }
    if (!low) return;
    low = false;
    const Code c = classify(t_us - fall_us);
    if (c == ALL_ROUND) ++hits_a;
    else if (c == CENTRE) ++hits_c;
  }
};

// 4 receivers -> 8 bits: bit 2i = receiver i heard 'A', bit 2i+1 = heard 'C'. i: 0 front-left, 1 front-right,
// 2 back-left, 3 back-right. Call once per window; it resets the counters.
inline uint8_t window_bits(Receiver* r, int n = 4) {
  uint8_t b = 0;
  for (int i = 0; i < n; ++i) {
    if (r[i].hits_a >= IR_MIN_HITS) b |= 1u << (2 * i);
    if (r[i].hits_c >= IR_MIN_HITS) b |= 1u << (2 * i + 1);
    r[i].hits_a = r[i].hits_c = 0;
  }
  return b;
}

}  // namespace ir
