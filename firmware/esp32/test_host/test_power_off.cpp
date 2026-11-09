// Host test of the flat-battery power-off logic (src/power_off.h).
#include <cstdio>
#include "../src/power_off.h"

static int checks = 0, fails = 0;
#define CHECK(c, m) do { ++checks; if (!(c)) { ++fails; printf("FAIL: %s\n", m); } } while (0)

static bool run(PowerOff &p, uint32_t &t, uint32_t ms, float v, bool chg, bool armed, uint32_t *first_high = nullptr) {
  bool any = false;
  for (uint32_t k = 0; k < ms; k += 10, t += 10) {
    bool h = p.step(t, v, chg, armed);
    if (h && !any && first_high) *first_high = t;
    any |= h;
  }
  return any;
}

int main() {
  { PowerOff p; uint32_t t = 0;
    CHECK(!run(p, t, 600000, 22.5f, false, false), "healthy battery: never off"); }
  { PowerOff p; uint32_t t = 0, fh = 0;
    CHECK(run(p, t, 60000, 19.5f, false, false, &fh), "flat battery: switches off");
    CHECK(fh >= PWR_CUTOFF_MS + PWR_GRACE_MS && fh < PWR_CUTOFF_MS + PWR_GRACE_MS + 50, "after 30 s + 20 s grace");
    CHECK(p.pending, "pending flag for the Pi"); }
  { PowerOff p; uint32_t t = 0;
    CHECK(!run(p, t, 120000, 19.5f, true, false), "charging: never off"); }
  { PowerOff p; uint32_t t = 0;
    CHECK(!run(p, t, 120000, 19.5f, false, true), "armed (balancing): never cut, it disarms first"); }
  { PowerOff p; uint32_t t = 0;
    run(p, t, 35000, 19.5f, false, false);
    CHECK(p.pending, "pending after 35 s");
    CHECK(!run(p, t, 60000, 20.6f, false, false) && !p.pending, "recovers above 20.4 V: cancelled"); }
  { PowerOff p; uint32_t t = 0;   // brief dips (motor load) don't add up
    bool off = false;
    for (int i = 0; i < 40; ++i) { off |= run(p, t, 5000, 19.5f, false, false); off |= run(p, t, 1000, 20.5f, false, false); }
    CHECK(!off, "dips of 5 s separated by recovery: no power-off"); }
  { PowerOff p; uint32_t t = 0; uint32_t highs = 0;
    run(p, t, PWR_CUTOFF_MS + PWR_GRACE_MS, 19.5f, false, false);
    for (int i = 0; i < 1000; ++i, t += 10) if (p.step(t, 19.5f, false, false)) highs += 10;
    CHECK(highs >= 900 && highs <= 1100, "OFF pulses 200 ms every 2 s"); }
  printf("%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
