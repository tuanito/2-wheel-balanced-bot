// Compiles the zone-array driver the way a board with no sensor library would: the guard must leave a
// class that still builds and reports nothing.
#include "tof.h"
#include <cstdio>
static int checks = 0, failed = 0;
static void ck(bool c, const char* what) { ++checks; if (!c) { ++failed; printf("FAIL: %s\n", what); } }

int main() {
  Tof t;
  // Without the ST ULD library linked in, the driver must still build and must report "nothing" -
  // never a stale or default distance. A board with no sensor fitted has to behave exactly as before.
  ck(!t.begin(), "begin() fails when no sensor library is linked");
  t.update(0.0f);
  ck(t.obstacle_m() < 0, "obstacle_m is negative (unknown), not 0");
  ck(t.drop_m() < 0, "drop_m is negative (unknown): the fence must not arm on a missing sensor");
  ck(!t.fresh(), "no frame is reported fresh");
  ck(Tof::N == 8, "8 x 8 zones");
  printf("%d checks, %d failed\n", checks, failed);
  return failed ? 1 : 0;
}
