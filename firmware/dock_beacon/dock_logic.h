// Dock rod switch: the charge rods are only live while the robot is on them.
// Pure logic (no AVR code), so it is unit-tested on the PC: firmware/dock_beacon/test_dock_logic.cpp.
//
// Sense circuit (switch OFF): +25.2 V -> R_SENSE 100k -> rod+ ; rod+ -> 220k -> ADC -> 47k -> GND.
//   nothing on the rods : rod+ = 18.3 V  (ADC ~660)
//   the robot           : its dock-detect divider (100k + 12k = 112k, before its diode) is the only load:
//                         rod+ = 11.1 V (ADC ~400). Window 310..480 = a load of about 65..180 kOhm.
//   coin / key / short  : rod+ ~0 V (ADC < 100): stays OFF.
// Switch ON: rod+ = charger output, the robot's battery + a diode drop: 18.5..25.2 V (ADC 660..910).
//   Below 15 V (ADC 540) for 3 reads in a row = short or overload -> OFF, 5 s pause; 3 faults in a minute -> latched OFF
//   until the dock is unplugged. Every 2 s it switches OFF for 30 ms and checks the robot is still there (the robot's
//   charge detect ignores dropouts shorter than 0.15 s), so the rods go dead within 2 s of the robot leaving.
#pragma once
#include <stdint.h>

namespace dock {

constexpr uint16_t ADC_ROBOT_LO = 310, ADC_ROBOT_HI = 480;   // robot's 112 k divider, with tolerances
constexpr uint16_t ADC_ON_MIN   = 540;                       // 15 V on the rods while ON
constexpr uint32_t PRESENT_MS   = 500;                       // robot seen this long before switching ON
constexpr uint32_t RECHECK_MS   = 2000, PROBE_OFF_MS = 30;   // presence re-check while ON
constexpr uint32_t FAULT_PAUSE_MS = 5000, FAULT_WINDOW_MS = 60000;
constexpr uint8_t  LOW_READS = 3, MAX_FAULTS = 3;

enum State : uint8_t { SENSE, ON, PROBE, FAULT, LATCHED };

inline bool robot_load(uint16_t adc) { return adc >= ADC_ROBOT_LO && adc <= ADC_ROBOT_HI; }

struct Switch {
  State st = SENSE;
  uint32_t t_state = 0, t_seen = 0, t_fault0 = 0;
  bool seen = false;
  uint8_t lows = 0, faults = 0;

  bool on() const { return st == ON; }                 // drive the high-side switch

  void go(State s, uint32_t now) { st = s; t_state = now; }

  // call every few ms with the rod+ ADC reading (taken with the switch in its current state)
  void step(uint32_t now, uint16_t adc) {
    switch (st) {
      case SENSE:
        if (robot_load(adc)) {
          if (!seen) { seen = true; t_seen = now; }
          if (now - t_seen >= PRESENT_MS) { seen = false; lows = 0; go(ON, now); }
        } else seen = false;
        break;
      case ON:
        if (adc < ADC_ON_MIN) {
          if (++lows >= LOW_READS) fault(now);
        } else lows = 0;
        if (st == ON && now - t_state >= RECHECK_MS) go(PROBE, now);
        break;
      case PROBE:                                       // switch is OFF: is the robot's divider still there?
        if (now - t_state >= PROBE_OFF_MS) {
          if (robot_load(adc)) { lows = 0; go(ON, now); }
          else { seen = false; go(SENSE, now); }
        }
        break;
      case FAULT:
        if (now - t_state >= FAULT_PAUSE_MS) { seen = false; go(SENSE, now); }
        break;
      case LATCHED: break;                              // stays off until power is removed
    }
  }

  void fault(uint32_t now) {
    if (faults == 0 || now - t_fault0 > FAULT_WINDOW_MS) { faults = 0; t_fault0 = now; }
    ++faults; lows = 0;
    go(faults >= MAX_FAULTS ? LATCHED : FAULT, now);
  }
};

}  // namespace dock
