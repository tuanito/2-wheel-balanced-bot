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
"$CXX" -O2 -std=c++17 -shared -fPIC core/controller.cpp core/sim_api.cpp -o "$OUT"
echo "built $OUT  ($CXX $("$CXX" -dumpversion 2>/dev/null || echo '?'))"
