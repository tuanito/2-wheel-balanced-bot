#!/bin/bash
# Build and run the PC unit tests of the CAN motor code (needs g++ with C++17).
set -e
cd "$(dirname "$0")"
g++ -std=gnu++17 -O1 -Wall -Wno-unused-function -I fake -I ../include -I ../src test_lk_motor.cpp ../src/lk_motor.cpp -o /tmp/test_lk_motor
/tmp/test_lk_motor
g++ -std=gnu++17 -O1 -Wall -I ../src test_ir_decode.cpp -o /tmp/test_ir_decode
/tmp/test_ir_decode
g++ -std=gnu++17 -O1 -Wall -I ../src test_power_off.cpp -o /tmp/test_power_off
/tmp/test_power_off
g++ -std=gnu++17 -O1 -Wall -I fake -I ../include -I ../src test_tof.cpp -o /tmp/test_tof
/tmp/test_tof
g++ -std=gnu++17 -O1 -Wall ../../dock_beacon/test_dock_logic.cpp -o /tmp/test_dock_logic
/tmp/test_dock_logic

# The motor choice is made in two files and they must agree. Compile the pair four ways: the two
# matched combinations must build, the two crossed ones must NOT. A static_assert nobody trips is a
# comment, so this trips it on purpose.
echo "motor profile: config.h and core/motor.hpp agree"
mm() {  # <label> <expect: ok|fail> <defines...>
  local label="$1" expect="$2"; shift 2
  if g++ -std=gnu++17 -O1 -Wall -I fake -I ../include -I ../../../core "$@" \
         test_motor_match.cpp -o /tmp/test_motor_match 2>/dev/null; then got=ok; else got=fail; fi
  if [ "$got" = "$expect" ]; then echo "  ok   $label ($expect)"; else
    echo "  FAIL $label: expected $expect, got $got" >&2; exit 1; fi
}
mm "mf6015 drive + mf6015 controller" ok
mm "mf7015 drive + mf7015 controller" ok -D FITTED_MOTOR=MOTOR_MF7015 -D BOT_MOTOR_MF7015
mm "mf7015 drive + mf6015 controller" fail -D FITTED_MOTOR=MOTOR_MF7015
mm "mf6015 drive + mf7015 controller" fail -D BOT_MOTOR_MF7015
echo "  4 checks, 0 failed"
