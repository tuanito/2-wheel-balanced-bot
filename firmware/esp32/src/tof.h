// VL53L5CX: 8 x 8 zones, 45 x 45 deg, on its own I2C bus. The alternative to the HC-SR04.
//
// What this turns 64 ranges into, for the controller:
//   obstacle_m - the nearest return that is NOT the floor, i.e. something standing in the way.
//   drop_m     - how far ahead the floor stops. A sonar cannot produce this at all: its cone reports
//                one range with no bearing, so "the floor echo is gone" says nothing about where.
//                Each zone here has a known elevation, so the range the floor SHOULD return in that
//                zone is h / sin(tilt + elevation + pitch) - exact geometry, no calibration - and the
//                first row whose return is much longer than that is looking past the edge.
//
// The ST ULD API (STSW-IMG023) is not vendored here: it is ~90 kB of ST-licensed C. Add it through
// platformio (lib_deps = stm32duino/VL53L5CX) or drop the ULD sources into lib/vl53l5cx/, then build
// with -D FRONT_SENSOR=FRONT_TOF -D HAS_VL53L5CX=1. Without it this file still compiles and reports
// nothing, so a board with no sensor fitted behaves exactly as before.
#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

#if HAS_VL53L5CX
#include <vl53l5cx_api.h>
#endif

class Tof {
 public:
  static constexpr int N = 8;                       // zones per side
  static constexpr float FLOOR_TOL = 0.080f;        // m: longer than predicted floor = past the edge

  bool begin() {
#if HAS_VL53L5CX
    pinMode(PIN_TOF_LPN, OUTPUT);
    digitalWrite(PIN_TOF_LPN, LOW);                 // held in reset while the bus comes up
    delay(10);
    digitalWrite(PIN_TOF_LPN, HIGH);
    cfg_.platform.address = 0x52;
    cfg_.platform.sda = PIN_TOF_SDA;
    cfg_.platform.scl = PIN_TOF_SCL;
    cfg_.platform.freq_hz = 1000000;                // its own bus: the IMU's is capped at 400 kHz
    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&cfg_, &alive) || !alive) return false;
    if (vl53l5cx_init(&cfg_)) return false;         // ~84 kB of firmware over I2C: ~1.4 s at 1 MHz
    vl53l5cx_set_resolution(&cfg_, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&cfg_, 15);   // 8x8 is capped at 15 Hz; 4x4 would allow 60
    vl53l5cx_set_ranging_mode(&cfg_, VL53L5CX_RANGING_MODE_CONTINUOUS);
    return vl53l5cx_start_ranging(&cfg_) == 0;
#else
    return false;
#endif
  }

  // Call from the slow loop. `pitch` is the body pitch in rad, nose-down positive, from the IMU:
  // the array is bolted to the body, so every zone's elevation moves with it.
  void update(float pitch) {
#if HAS_VL53L5CX
    uint8_t ready = 0;
    if (vl53l5cx_check_data_ready(&cfg_, &ready) || !ready) return;
    if (vl53l5cx_get_ranging_data(&cfg_, &res_)) return;
    float obstacle = -1, drop = -1;
    for (int row = 0; row < N; ++row) {
      const float el = radians(TOF_TILT_DEG) + (row - (N - 1) / 2.0f) * radians(TOF_FOV_DEG) / N;
      const float a = el + pitch;
      const float floor_m = (a > radians(3.0f)) ? TOF_HEIGHT_M / sinf(a) : 1e9f;
      for (int col = 0; col < N; ++col) {
        const int i = row * N + col;
        if ((res_.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i] != 5) &&
            (res_.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i] != 9)) continue;   // 5/9 = valid
        const float r = res_.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i] / 1000.0f;
        if (r < 0.02f || r > 4.0f) continue;
        if (r < floor_m - FLOOR_TOL) {              // nearer than the floor: something is there
          if (obstacle < 0 || r < obstacle) obstacle = r;
        } else if (r > floor_m + FLOOR_TOL) {       // further than the floor: the floor ran out
          const float horiz = r * cosf(a);          // where that ray would have met floor level
          const float edge = floor_m * cosf(a);
          const float d = fminf(edge, horiz);
          if (drop < 0 || d < drop) drop = d;
        }
      }
    }
    obstacle_ = obstacle;
    drop_ = drop;
    fresh_ = true;
#else
    (void)pitch;
#endif
  }

  float obstacle_m() const { return obstacle_; }    // m, < 0 = nothing
  float drop_m() const { return drop_; }            // m, < 0 = no edge seen
  bool fresh() const { return fresh_; }

 private:
  float obstacle_ = -1, drop_ = -1;
  bool fresh_ = false;
#if HAS_VL53L5CX
  VL53L5CX_Configuration cfg_{};
  VL53L5CX_ResultsData res_{};
#endif
};
