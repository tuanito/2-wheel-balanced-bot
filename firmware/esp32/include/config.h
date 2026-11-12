// Board wiring and robot constants. Check every line marked VERIFY on the real hardware.
#pragma once
#include <Arduino.h>

// ---------------- pins (ESP32-S3-DevKitC-1) ----------------
constexpr int PIN_CAN_TX     = 4;    // -> SN65HVD230 CTX
constexpr int PIN_CAN_RX     = 5;    // <- SN65HVD230 CRX
constexpr int PIN_I2C_SDA    = 8;    // IMU
constexpr int PIN_I2C_SCL    = 9;
constexpr int PIN_PI_TX      = 17;   // -> Pi GPIO15 (RXD)
constexpr int PIN_PI_RX      = 18;   // <- Pi GPIO14 (TXD)
constexpr int PIN_SONAR_TRIG = 10;   // only with FRONT_SENSOR == FRONT_SONAR
constexpr int PIN_SONAR_ECHO = 11;   // through the 1k/2k divider (HC-SR04 echo is 5 V)

// ---------------- front sensor: one or the other, never both (they share the aperture) ----------
#define FRONT_SONAR 0
#define FRONT_TOF   1
#ifndef FRONT_SENSOR
#define FRONT_SENSOR FRONT_SONAR     // -D FRONT_SENSOR=FRONT_TOF to build for the VL53L5CX
#endif

// VL53L5CX (8 x 8 zones, 45 x 45 deg), on the front panel above the camera, looking down.
// Its OWN I2C bus, not the IMU's: the MPU6050 caps that bus at 400 kHz, and a full 8 x 8 frame is
// ~3.4 kB, which is ~51 kB/s at 15 Hz and does not fit in 400 kHz. On a second controller at 1 MHz,
// with the unused outputs compiled out (~450 B/frame), it costs ~7 kB/s. The S3 has two I2C
// peripherals and 8 MB of flash against the ULD driver's ~90 kB, so no board change is needed.
// NOT GPIO 33-37: on the N8R8 module those are the octal PSRAM bus and are not brought out for use.
// Not 0/3/45/46 (strapping), not 19/20 (USB), not 26-32 (flash), not 48 (on-board RGB LED).
constexpr int PIN_TOF_SDA    = 42;
constexpr int PIN_TOF_SCL    = 41;
constexpr int PIN_TOF_LPN    = 2;    // low = held in reset; released once at boot
constexpr int PIN_TOF_INT    = 47;   // data-ready, active low, self-clearing
constexpr float TOF_TILT_DEG = 30.0f;  // below horizontal. sim/tof_sensor.py --measure picked this:
                                       // best answer rate at a 250 mm standoff while still seeing
                                       // floor 1.47 m ahead, so it also does the obstacle job.
constexpr float TOF_HEIGHT_M = 0.194f; // transducer height above the ground (panel z 140 + wheel r)
constexpr float TOF_FOV_DEG  = 45.0f;
constexpr int PIN_ARM_BTN    = 12;   // to GND, internal pull-up
constexpr int PIN_LED_R      = 38;
constexpr int PIN_LED_G      = 39;
constexpr int PIN_LED_B      = 40;
constexpr int PIN_VBAT       = 1;    // ADC1: battery 100k/12k divider
constexpr int PIN_DOCK_SENSE = 6;    // ADC1: charge pad + (before the diode) 100k/12k divider
constexpr int PIN_USB_SENSE  = 7;    // ADC1: IP2368 VBUS 56k/10k divider
constexpr int PIN_IR_FL       = 13;   // IR beacon receivers (TSOP38238, active low): front-left
constexpr int PIN_IR_FR       = 14;   //   front-right
constexpr int PIN_IR_BL       = 15;   //   back-left
constexpr int PIN_IR_BR       = 16;   //   back-right
constexpr int PIN_PWR_OFF      = 21;   // -> power switch OFF input (Pololu #2813): high pulse switches the robot off

// ---------------- dividers ----------------
constexpr float VBAT_DIV      = (100.f + 12.f) / 12.f;   // calibrate once against a multimeter (VERIFY)
constexpr float DOCK_DIV      = (100.f + 12.f) / 12.f;
constexpr float USB_DIV       = (56.f + 10.f) / 10.f;
constexpr float VBAT_OVER     = 26.8f;   // braking energy with nowhere to go (BMS blocking charge): motors off
constexpr float VBAT_LOW      = 20.4f;   // 3.4 V/cell under load: disarm after 5 s below this
constexpr float CHARGE_DETECT = 15.f;    // V at the pad / USB VBUS that means "charger connected"

// ---------------- motors (LK MF6015-V2, CAN 1 Mbps) ----------------
constexpr uint32_t CAN_ID_LEFT  = 0x141;   // motor ID 1 (set with LK's PC tool or the ID switch)
constexpr uint32_t CAN_ID_RIGHT = 0x142;   // motor ID 2
constexpr uint32_t CAN_ID_MULTI = 0x280;   // multi-motor torque command (motors 1-4 in one frame)
// The motors face opposite ways. +1/-1 so that "positive torque" drives the robot forward on both sides. VERIFY on the
// bench: wheels off the ground, send +0.2 N·m to each, both must turn so the robot would roll forward.
constexpr int DIR_LEFT  = +1;
constexpr int DIR_RIGHT = -1;
constexpr float KT          = 0.14f;      // N·m/A (15T winding)
constexpr float IQ_FULL_A   = 16.5f;      // iq command 2048 = 16.5 A
constexpr int   IQ_MAX_CMD  = 2048;
constexpr float ENC_COUNTS  = 65536.f;    // starting guess only: the firmware detects 14/15/16-bit at run time (boot log says which)
constexpr uint32_t CAN_TIMEOUT_MS = 20;   // no reply for this long -> motors off, FAULT

// ---------------- IMU (MPU6050 / GY-521) ----------------
constexpr uint8_t MPU_ADDR = 0x68;
// Map the sensor axes to the robot frame (X forward, Y left, Z up). Default: chip X forward, Y left, Z up.
// VERIFY at rest, upright: the boot log must print acc ≈ (0, 0, +9.8); tilt forward -> ax becomes negative... see README.
#define IMU_MAP_AX(a) ( (a)[0])
#define IMU_MAP_AY(a) ( (a)[1])
#define IMU_MAP_AZ(a) ( (a)[2])
#define IMU_MAP_GX(g) ( (g)[0])
#define IMU_MAP_GY(g) ( (g)[1])
#define IMU_MAP_GZ(g) ( (g)[2])

// ---------------- timing ----------------
constexpr uint32_t CONTROL_PERIOD_US = 2000;   // 500 Hz: must match Params::DT in gains.hpp
constexpr uint32_t TELEM_PERIOD_MS   = 20;     // 50 Hz to the Pi
constexpr uint32_t CMD_TIMEOUT_MS    = 300;    // no command from the Pi -> stop driving (keep balancing)
constexpr uint32_t PI_BAUD           = 921600;
