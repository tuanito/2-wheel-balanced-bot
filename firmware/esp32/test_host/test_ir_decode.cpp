// PC test of the IR beacon decoder (src/ir_decode.h).
#include <cstdio>
#include "ir_decode.h"
static int fails = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++fails; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)
static void burst(ir::Receiver& r, uint32_t& t, uint32_t low_us, uint32_t gap_us) {
  r.edge(false, t); t += low_us; r.edge(true, t); t += gap_us;
}
int main() {
  CHECK(ir::classify(600) == ir::ALL_ROUND); CHECK(ir::classify(1200) == ir::CENTRE);
  CHECK(ir::classify(200) == ir::NONE); CHECK(ir::classify(920) == ir::NONE); CHECK(ir::classify(2500) == ir::NONE);
  ir::Receiver rx[4]; uint32_t t = 1000;
  for (int k = 0; k < 15; ++k) { burst(rx[2], t, 640, 2400); burst(rx[2], t, 1180, 2400); }   // back-left: both codes
  for (int k = 0; k < 15; ++k) burst(rx[1], t, 560, 2400);                                      // front-right: all round only
  for (int k = 0; k < 2; ++k) burst(rx[0], t, 600, 2400);                                       // front-left: too few
  for (int k = 0; k < 20; ++k) burst(rx[3], t, 9000, 4500);                                     // back-right: a TV remote's header
  const uint8_t b = ir::window_bits(rx);
  CHECK(b == ((1u << 4) | (1u << 5) | (1u << 2)));
  CHECK(ir::window_bits(rx) == 0);                   // counters reset each window
  rx[0].edge(true, t);                               // a rising edge without a fall is ignored
  CHECK(rx[0].hits_a == 0 && rx[0].hits_c == 0);
  std::printf("%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
