#!/usr/bin/env bash
# Build the simulation-validated controller as the shared library the simulator loads.
# macOS -> sim/libbot.dylib, everything else -> sim/libbot.so (sim/botsim.py picks the right one).
# Override the compiler with CXX=... if you need to.
set -euo pipefail
cd "$(dirname "$0")/.."
case "$(uname -s)" in
  Darwin) CXX="${CXX:-clang++}"; OUT="sim/libbot.dylib" ;;
  *)      CXX="${CXX:-g++}";     OUT="sim/libbot.so"    ;;
esac
# The motor is a compile-time choice (core/motor.hpp): torque limit, speed ceiling, rotor inertia.
# ROBOT_MOTOR selects it here exactly as it does for the CAD and sim/motor_profile.py.
MOTOR="${ROBOT_MOTOR:-mf6015}"
case "$MOTOR" in
  mf6015) DEF="" ;;
  mf7015) DEF="-DBOT_MOTOR_MF7015" ;;
  *) echo "unknown ROBOT_MOTOR=$MOTOR (expected mf6015 or mf7015)" >&2; exit 2 ;;
esac
"$CXX" -O2 -std=c++17 $DEF -shared -fPIC core/controller.cpp core/sim_api.cpp -o "$OUT"
echo "built $OUT for $MOTOR  ($CXX $("$CXX" -dumpversion 2>/dev/null || echo '?'))"
