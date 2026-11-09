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
g++ -std=gnu++17 -O1 -Wall ../../dock_beacon/test_dock_logic.cpp -o /tmp/test_dock_logic
/tmp/test_dock_logic
