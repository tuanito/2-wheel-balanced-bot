// ESP32 <-> Raspberry Pi UART protocol (921600 8N1). Mirrored in pi/robot/link.py: keep both in sync.
// Frame: 0xA5 0x5A | type u8 | len u8 | payload[len] | crc8 (poly 0x07 over type, len, payload)
// All multi-byte fields little-endian, floats IEEE-754 single.
#pragma once
#include <stdint.h>

namespace pilink {
constexpr uint8_t SYNC0 = 0xA5, SYNC1 = 0x5A;

enum : uint8_t {
  MSG_CMD   = 0x01,   // Pi -> ESP32, >= 10 Hz while driving
  MSG_TELEM = 0x81,   // ESP32 -> Pi, 50 Hz
  MSG_EVENT = 0x82,   // ESP32 -> Pi, text line (boot info, faults)
};

enum : uint8_t {        // CmdMsg.flags
  F_ARM          = 1 << 0,   // request armed (edge-triggered on the ESP32: 0->1 arms)
  F_DISARM       = 1 << 1,   // request disarm (wins over F_ARM)
  F_AUTO_STANDUP = 1 << 2,   // stand up by itself when lying
  F_OBST_STOP    = 1 << 3,   // sonar may block forward driving
};

#pragma pack(push, 1)
struct CmdMsg {         // 16 bytes
  uint8_t mode;         // 0 velocity, 1 turn-to (absolute heading), 2 turn-by (relative, new seq), 3 backflip (new seq),
                        // 4 dock (velocity + lean back onto the dock on contact / new seq), 5 undock (new seq)
  uint8_t flags;
  uint16_t seq;
  float v;              // m/s
  float yaw_rate;       // rad/s
  float angle;          // rad
};
struct TelemMsg {       // 57 bytes
  uint32_t t_ms;
  uint8_t state;        // bot::State
  int8_t side;          // +1 face-down, -1 back-down
  uint8_t flags;        // bit0 armed, bit1 charging, bit2 sonar limiting, bit3 battery low, bit4 motor fault
  uint8_t tries;
  float pitch, roll, yaw;   // rad
  float x, y, v;            // m, m, m/s (odometry)
  float vbat;               // V
  float sonar;              // m, <0 none
  float tl, tr;             // N·m
  float standup_adj;        // learned stand-up energy correction
  float motor_temp;         // °C, max of both
  uint8_t ir;               // IR beacon, last 100 ms: bit 2i = receiver i heard 'all round', 2i+1 = 'centre';
                            // i: 0 front-left, 1 front-right, 2 back-left, 3 back-right
};
#pragma pack(pop)
static_assert(sizeof(CmdMsg) == 16, "CmdMsg layout");
static_assert(sizeof(TelemMsg) == 57, "TelemMsg layout");

inline uint8_t crc8(const uint8_t* d, int n, uint8_t crc = 0) {
  for (int i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
}  // namespace pilink
