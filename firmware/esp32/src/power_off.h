// Automatic power-off when the battery is flat (protects the LiPo from over-discharge).
// The power switch is a Pololu Big Pushbutton Power Switch HP (#2813); its OFF input switches the robot off on a
// high pulse. The push button on top always works by itself (wired straight to the switch board).
// Pure logic, host-tested in test_host/test_power_off.cpp.
//   battery below PWR_CUTOFF_V (3.3 V/cell) for PWR_CUTOFF_MS, not charging, motors disarmed:
//     -> pending (telemetry flag 32: the Pi shuts down cleanly) -> PWR_GRACE_MS later: OFF pulses (200 ms every 2 s)
//   recovers above PWR_CUTOFF_V + 0.6 V, or a charger is connected, or it is armed -> cancelled.
#pragma once
#include <stdint.h>

constexpr float    PWR_CUTOFF_V  = 19.8f;     // 6 x 3.3 V
constexpr uint32_t PWR_CUTOFF_MS = 30000;
constexpr uint32_t PWR_GRACE_MS  = 20000;     // time for the Pi to halt
constexpr uint32_t PWR_PULSE_MS  = 200, PWR_REPULSE_MS = 2000;

struct PowerOff {
  uint32_t below_since = 0, pending_since = 0;
  bool pending = false, below = false;

  // -> true while the OFF pin must be driven high
  bool step(uint32_t now, float vbat, bool charging, bool armed) {
    if (charging || armed || vbat > PWR_CUTOFF_V + 0.6f) { below = false; pending = false; return false; }
    if (vbat < PWR_CUTOFF_V) {
      if (!below) { below = true; below_since = now; }
      if (!pending && now - below_since >= PWR_CUTOFF_MS) { pending = true; pending_since = now; }
    } else if (!pending) below = false;                      // in the hysteresis band: restart the count
    if (!pending) return false;
    uint32_t t = now - pending_since;
    if (t < PWR_GRACE_MS) return false;
    return (t - PWR_GRACE_MS) % PWR_REPULSE_MS < PWR_PULSE_MS;
  }
};
