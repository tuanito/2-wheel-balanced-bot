// MPU6050 (GY-521) at 1 kHz internal rate, ±2000 dps (stand-up spins at up to ~20 rad/s), ±8 g.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

struct ImuSample { float ax, ay, az, gx, gy, gz; bool ok; };

#include "sim_hil.h"

class Imu {
 public:
#if defined(SIM_HIL) || defined(BENCH_CAN)
  bool begin() { simhil::begin(); return true; }
  ImuSample read() {                       // IMU sample from the physics host (already in m/s2, rad/s, chip axes)
    ImuSample s{}; float v[6];
    s.ok = simhil::step(micros(), v);
    const float a[3] = {v[0], v[1], v[2]}, g[3] = {v[3], v[4], v[5]};
    s.ax = IMU_MAP_AX(a); s.ay = IMU_MAP_AY(a); s.az = IMU_MAP_AZ(a);
    s.gx = IMU_MAP_GX(g); s.gy = IMU_MAP_GY(g); s.gz = IMU_MAP_GZ(g);
    return s;
  }
#else
  bool begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    if (!wr(0x6B, 0x80)) return false;   // reset
    delay(100);
    wr(0x6B, 0x01);                       // clock = gyro X PLL
    wr(0x1A, 0x02);                       // DLPF 94 Hz accel / 98 Hz gyro, 1 kHz
    wr(0x19, 0x00);                       // sample rate divider 0 -> 1 kHz
    wr(0x1B, 0x18);                       // gyro ±2000 dps
    wr(0x1C, 0x10);                       // accel ±8 g
    return rd8(0x75) == 0x68 || rd8(0x75) == 0x70 || rd8(0x75) == 0x98;   // WHO_AM_I of MPU6050 clones
  }
  ImuSample read() {
    ImuSample s{}; uint8_t b[14];
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)MPU_ADDR, 14) != 14) { s.ok = false; return s; }
    for (int i = 0; i < 14; ++i) b[i] = Wire.read();
    auto w = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
    const float A = 9.80665f / 4096.f, G = (2000.f / 32768.f) * (PI / 180.f);
    const float a[3] = {w(0) * A, w(2) * A, w(4) * A};
    const float g[3] = {w(8) * G, w(10) * G, w(12) * G};
    s.ax = IMU_MAP_AX(a); s.ay = IMU_MAP_AY(a); s.az = IMU_MAP_AZ(a);
    s.gx = IMU_MAP_GX(g); s.gy = IMU_MAP_GY(g); s.gz = IMU_MAP_GZ(g);
    s.ok = true;
    return s;
  }
 private:
  bool wr(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }
  uint8_t rd8(uint8_t r) { Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.endTransmission(false); Wire.requestFrom((int)MPU_ADDR, 1); return Wire.read(); }
#endif
};
