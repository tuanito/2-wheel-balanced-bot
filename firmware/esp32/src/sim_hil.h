// Hardware-in-the-loop shim. -DSIM_HIL (env esp32s3_sim, QEMU): IMU + CAN over UART0.
// -DBENCH_CAN (env esp32s3_bench, real ESP32 + real CAN on the bench): only the IMU comes from the PC over UART0;
// the Pi's command frames share that port and are handed to `on_cmd`.
// The firmware runs unchanged in Espressif QEMU; the parts QEMU does not emulate are replaced here:
//   - CAN (TWAI): frames go over UART0 to the physics host (virtual LK motors); console text shares the port
//   - IMU (I2C):  each read sends a step request (with the firmware's own clock) and waits for the host's sample
// The host advances the MuJoCo physics by exactly the time the firmware measured between two requests,
// so the controller, its timing, the CAN protocol handling and the Pi link all run as on the robot.
#pragma once
#if defined(SIM_HIL) || defined(BENCH_CAN)
#include <Arduino.h>
#include "link_proto.h"
#include "driver/uart.h"
#include <algorithm>

namespace simhil {
constexpr uart_port_t PORT = UART_NUM_0;   // shared with the console (Serial); the host separates frames from text
enum : uint8_t { T_STEP = 'S', T_CAN = 'C', T_IMU = 'I' };

struct CanFrame { uint32_t id; uint8_t dlc; uint8_t d[8]; };
inline void (*on_cmd)(const uint8_t* payload, uint8_t len) = nullptr;   // Pi command frames arriving on this port

inline uint8_t crc(const uint8_t* p, int n) { uint8_t c = 0; while (n--) c = (uint8_t)((c << 1 | c >> 7) ^ *p++); return c; }

inline void begin() {
  static bool done = false;
  if (done) return;
  if (!uart_is_driver_installed(PORT)) uart_driver_install(PORT, 4096, 4096, 0, nullptr, 0);   // Serial.begin() normally did it
  uart_set_rx_full_threshold(PORT, 1);   // hand every byte over at once (lock-step latency)
  uart_set_rx_timeout(PORT, 1);
  done = true;
}

inline void send(uint8_t type, const void* p, uint8_t len) {
  uint8_t b[4 + 64 + 1] = {0xA5, 0x5A, type, len};
  memcpy(b + 4, p, len);
  b[4 + len] = crc(b + 2, 2 + len);
  uart_write_bytes(PORT, (const char*)b, 5 + len);
}

// received CAN frames, drained by LkMotors::poll()
inline CanFrame rxq[32];
inline volatile int rx_head = 0, rx_tail = 0;

inline bool read_exact(uint8_t* p, int n) {
  int got = 0;
  while (got < n) {
    const int r = uart_read_bytes(PORT, p + got, n - got, pdMS_TO_TICKS(5000));
    if (r <= 0) return false;
    got += r;
  }
  return true;
}

// Send a step request and block until the host's IMU sample arrives; CAN frames received meanwhile are queued.
inline bool step(uint32_t t_us, float imu[6]) {
  send(T_STEP, &t_us, 4);
  for (;;) {
    uint8_t h[4];
    if (!read_exact(h, 1)) return false;
    if (h[0] != 0xA5) continue;
    if (!read_exact(h + 1, 3) || h[1] != 0x5A) continue;
    uint8_t p[64 + 1];
    if (h[3] > 64 || !read_exact(p, h[3] + 1)) continue;
    uint8_t chk[66]; chk[0] = h[2]; chk[1] = h[3]; memcpy(chk + 2, p, h[3]);
    if (h[2] == pilink::MSG_CMD) {                       // Pi link frame (its own CRC)
      if (pilink::crc8(chk, 2 + h[3]) == p[h[3]] && on_cmd) on_cmd(p, h[3]);
      continue;
    }
    if (crc(chk, 2 + h[3]) != p[h[3]]) continue;
    if (h[2] == T_CAN && h[3] == sizeof(CanFrame)) {
      const int nx = (rx_head + 1) % 32;
      if (nx != rx_tail) { memcpy(&rxq[rx_head], p, sizeof(CanFrame)); rx_head = nx; }
    } else if (h[2] == T_IMU && h[3] == 24) {
      memcpy(imu, p, 24);
      return true;
    }
  }
}

// Non-blocking: parse whatever bytes already arrived (motor replies to the last command), as the TWAI RX FIFO would hold them.
inline void pump() {
  static uint8_t buf[256]; static int n = 0;
  size_t avail = 0; uart_get_buffered_data_len(PORT, &avail);
  while (avail > 0) {
    const int r = uart_read_bytes(PORT, buf + n, (int)std::min<size_t>(avail, sizeof(buf) - n), 0);
    if (r <= 0) break;
    n += r; avail -= r;
    for (;;) {
      int i = 0; while (i + 1 < n && !(buf[i] == 0xA5 && buf[i + 1] == 0x5A)) ++i;
      if (i) { memmove(buf, buf + i, n - i); n -= i; }
      if (n < 4) break;
      const int need = 4 + buf[3] + 1;
      if (buf[3] > 64) { memmove(buf, buf + 2, n - 2); n -= 2; continue; }
      if (n < need) break;
      if (crc(buf + 2, 2 + buf[3]) == buf[need - 1] && buf[2] == T_CAN && buf[3] == sizeof(CanFrame)) {
        const int nx = (rx_head + 1) % 32;
        if (nx != rx_tail) { memcpy(&rxq[rx_head], buf + 4, sizeof(CanFrame)); rx_head = nx; }
      }
      memmove(buf, buf + need, n - need); n -= need;
    }
    if (n == (int)sizeof(buf)) n = 0;
  }
}

inline void can_tx(uint32_t id, const uint8_t d[8]) {
  CanFrame f{}; f.id = id; f.dlc = 8; memcpy(f.d, d, 8);
  send(T_CAN, &f, sizeof f);
}

inline bool can_rx(CanFrame& f) {
  if (rx_tail == rx_head) return false;
  f = rxq[rx_tail]; rx_tail = (rx_tail + 1) % 32;
  return true;
}
}  // namespace simhil
#endif
