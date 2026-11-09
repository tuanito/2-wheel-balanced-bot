// PC test of the dock's rod switch (dock_logic.h) against an electrical model of the sense circuit.
// g++ -std=c++17 -O1 test_dock_logic.cpp -o /tmp/t && /tmp/t
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "dock_logic.h"

static int fails = 0, checks = 0;
#define CHECK(c, msg) do { ++checks; if (!(c)) { ++fails; printf("FAIL: %s\n", msg); } } while (0)

constexpr double VCH = 25.2, RS = 100e3, RA = 220e3, RB = 47e3;
// rod+ voltage: OFF = through the 100 k sense resistor into (load || 267 k); ON = charger, unless a short drags it down
static uint16_t adc(double load, bool on, double vbat_robot = 23.0) {
  double rdiv = RA + RB, v;
  if (on) {
    if (load < 50) v = 0.3;                                   // dead short: the charger current-limits, ~0 V
    else if (load > 50e3 && load < 200e3 && vbat_robot > 0) v = std::fmin(VCH, vbat_robot + 0.5);   // charging the robot
    else v = VCH;
  } else {
    double rl = (load <= 0 || load > 1e12) ? rdiv : 1.0 / (1.0 / load + 1.0 / rdiv);
    v = VCH * rl / (RS + rl);
  }
  double a = v * RB / rdiv / 5.0 * 1023.0;
  return (uint16_t)std::fmax(0, std::fmin(1023, std::lround(a)));
}

struct World { double load = 1e13; bool robot = false; };

static void run(dock::Switch &s, World &w, uint32_t &t, uint32_t ms, bool *ever_on = nullptr, uint32_t *t_off = nullptr) {
  for (uint32_t k = 0; k < ms; k += 3, t += 3) {
    s.step(t, adc(w.load, s.on()));
    if (ever_on && s.on()) *ever_on = true;
    if (t_off && !s.on() && *t_off == 0) *t_off = t;
  }
}

int main() {
  printf("ADC: open %u, robot %u, robot -10%% %u, robot +10%% %u, coin %u, ON charging %u (low batt 18 V: %u)\n",
         adc(1e13, false), adc(112e3, false), adc(100e3, false), adc(124e3, false), adc(0.01, false),
         adc(112e3, true, 23.0), adc(112e3, true, 18.0));
  {   // nothing on the rods: never live
    dock::Switch s; World w; uint32_t t = 0; bool on = false;
    run(s, w, t, 20000, &on);
    CHECK(!on, "no robot: rods stay dead");
  }
  {   // coin across the rods: never live
    dock::Switch s; World w; w.load = 0.01; uint32_t t = 0; bool on = false;
    run(s, w, t, 20000, &on);
    CHECK(!on, "coin/key short: rods stay dead");
  }
  {   // 10 k (wet hand, a resistor, the wrong device): stays dead
    dock::Switch s; World w; w.load = 10e3; uint32_t t = 0; bool on = false;
    run(s, w, t, 20000, &on);
    CHECK(!on, "10 k load: rods stay dead");
  }
  {   // robot docks: live after ~0.5 s, stays live while charging, through re-checks
    dock::Switch s; World w; uint32_t t = 0;
    run(s, w, t, 1000);
    w.load = 112e3; uint32_t t0 = t;
    while (!s.on() && t - t0 < 3000) run(s, w, t, 3);
    CHECK(s.on() && t - t0 >= 500 && t - t0 < 700, "robot: rods live 0.5 s after it arrives");
    uint32_t on_ms = 0;
    for (int i = 0; i < 10000; ++i, t += 3) { s.step(t, adc(w.load, s.on())); if (s.on()) on_ms += 3; }
    CHECK(on_ms > 29000, "robot: live > 97 % of the time while charging (2 s re-check, 30 ms off)");
    // robot leaves (undocks): dead within the re-check period
    w.load = 1e13; uint32_t tl = t, t_off = 0;
    run(s, w, t, 3000, nullptr, &t_off);
    CHECK(!s.on() && t_off > 0 && t_off - tl <= 2100, "robot leaves: rods dead within 2.1 s");
    bool on = false; run(s, w, t, 10000, &on);
    CHECK(!on, "after the robot left: stays dead");
  }
  {   // short while charging (coin dropped between rod and pad): off within 10 ms, retries, latches after 3 faults
    dock::Switch s; World w; w.load = 112e3; uint32_t t = 0;
    run(s, w, t, 1000);
    CHECK(s.on(), "robot charging");
    w.load = 0.01; uint32_t ts = t;
    while (s.on() && t - ts < 1000) run(s, w, t, 3);
    CHECK(!s.on() && t - ts <= 12, "short while live: off within 12 ms");
    CHECK(s.st == dock::FAULT, "short: fault state");
    bool on = false; run(s, w, t, 60000, &on);
    CHECK(!on, "short stays: never live again (sense sees ~0 V, not the robot)");
    // robot back, but the short keeps re-appearing each time it switches on: latched after 3 faults
    dock::Switch s2; World w2; w2.load = 112e3; uint32_t t2 = 0; int faults_seen = 0;
    for (int i = 0; i < 40000; ++i, t2 += 3) {
      bool on2 = s2.on();
      uint16_t a = on2 ? adc(0.01, true) : adc(112e3, false);          // shorts only when live
      s2.step(t2, a);
      if (s2.st == dock::FAULT && !on2) {}
      if (s2.st == dock::LATCHED) break;
    }
    faults_seen = s2.faults;
    CHECK(s2.st == dock::LATCHED && faults_seen == 3, "repeated short: latched off after 3 faults");
    bool on3 = false; for (int i = 0; i < 20000; ++i, t2 += 3) { s2.step(t2, adc(112e3, false)); if (s2.on()) on3 = true; }
    CHECK(!on3, "latched: stays off until unplugged");
  }
  {   // deeply discharged robot (18 V): still charges (above the 15 V short threshold)
    dock::Switch s; World w; w.load = 112e3; uint32_t t = 0; uint32_t on_ms = 0;
    for (int i = 0; i < 5000; ++i, t += 3) { s.step(t, adc(w.load, s.on(), 18.0)); if (s.on()) on_ms += 3; }
    CHECK(on_ms > 13000, "flat robot battery (18 V): charges");
  }
  {   // robot's divider resistors ±10 %: still recognised
    for (double r : {100e3, 124e3}) {
      dock::Switch s; World w; w.load = r; uint32_t t = 0; bool on = false;
      run(s, w, t, 2000, &on);
      CHECK(on, "robot divider at ±10 %: recognised");
    }
  }
  printf("%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
