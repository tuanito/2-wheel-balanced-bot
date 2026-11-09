#!/usr/bin/env bash
# Regenerate everything the simulator is built from, then run the simulation suite (README "Build and run").
#   robot_c.xml + robot_c_arena.xml   from the CAD mass properties (sim/cad_mass.json)
#   core/gains.hpp                    LQR gains from the model's mass properties
#   sim/libbot.{so,dylib}             the real C++ controller
#   sim/results/                      the 30-robot report
#
#   sim/rebuild.sh            # the lot
#   sim/rebuild.sh 60         # ... with 60 randomised robots instead of the default 30
#   sim/rebuild.sh --no-gains # leave core/gains.hpp alone (it is committed and tuned)
#
# Note: the committed sim/results/report.json was produced with the default 30. Passing a
# different count changes the numbers, so the report will no longer match what is committed.
set -euo pipefail
cd "$(dirname "$0")/.."
PY="$PWD/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"

GAINS=1; N=""
for a in "$@"; do
  case "$a" in
    --no-gains) GAINS=0 ;;
    ''|*[!0-9]*) echo "unknown option: $a (expected a robot count or --no-gains)" >&2; exit 2 ;;
    *) N="$a" ;;
  esac
done
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

step "MuJoCo models from the CAD mass properties"
(cd sim && "$PY" build_model.py && "$PY" build_model.py --arena)

if [ "$GAINS" = 1 ]; then
  step "LQR gains -> core/gains.hpp"
  (cd sim && "$PY" design_lqr.py 0.06 0.6 10)
fi

step "controller library"
core/build_lib.sh

step "simulation suite${N:+ ($N robots)}"
(cd sim && "$PY" run_tests.py ${N:+"$N"})

printf '\nWrote sim/results/. Videos and the rest:  cd sim && python make_video.py && python nav_sim.py 150 --video\n'
