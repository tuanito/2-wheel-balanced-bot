#!/usr/bin/env bash
# Every check that can run on a development machine, in one command.
#
#   ./check.sh                 # all of it (~1.5 minutes)
#   ./check.sh elec pi         # only these sections
#   sections: firmware | elec | pi | sim
#
# Several of these regenerate committed reports. This script snapshots those files
# first, tells you if the fresh run disagrees with what is committed, and puts the
# originals back, so running a check never leaves the working tree dirty.
set -uo pipefail
cd "$(dirname "$0")"
ROOT="$PWD"
PY="$ROOT/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"

SECTIONS=("$@"); [ ${#SECTIONS[@]} -eq 0 ] && SECTIONS=(firmware elec pi sim)
want() { for s in "${SECTIONS[@]}"; do [ "$s" = "$1" ] && return 0; done; return 1; }

PASS=(); FAIL=(); SKIP=(); DRIFT=()
SNAP="$(mktemp -d)"; trap 'rm -rf "$SNAP"' EXIT

# Generated files that the checks below overwrite.
ARTIFACTS=(elec/erc_report.json elec/robot.net
           sim/robot_c.xml sim/robot_c_arena.xml
           sim/results/report.json sim/results/mission_log.npy sim/results/mission_marks.json)

save_artifacts() {
  for f in "${ARTIFACTS[@]}"; do
    [ -f "$f" ] || continue
    mkdir -p "$SNAP/$(dirname "$f")"; cp -p "$f" "$SNAP/$f"
  done
}
restore_artifacts() {                       # report drift, then put the originals back
  for f in "${ARTIFACTS[@]}"; do
    [ -f "$SNAP/$f" ] || continue
    cmp -s "$SNAP/$f" "$f" || DRIFT+=("$f")
    cp -p "$SNAP/$f" "$f"
  done
}

run() {                                     # run <name> <command...>
  local name="$1"; shift
  printf '\n\033[1m== %s\033[0m\n' "$name"
  if "$@"; then PASS+=("$name"); else FAIL+=("$name"); fi
}

save_artifacts

if want firmware; then
  if command -v g++ >/dev/null; then
    run "firmware: host unit tests" firmware/esp32/test_host/run_tests.sh
  else
    SKIP+=("firmware: host unit tests (no g++)")
  fi
fi

if want elec; then
  run "elec: electrical rules check" bash -c "cd '$ROOT/elec' && '$PY' erc.py | tail -3"
  # test_erc.py calls erc.main(), which overwrites erc_report.json with the last
  # mutation case: re-run erc.py afterwards so the report matches the real wiring.
  run "elec: 8 wiring mutation tests" bash -c "cd '$ROOT/elec' && '$PY' test_erc.py && '$PY' erc.py >/dev/null"
  if [ -f cad_src/case/wiring_data.json ] || [ -f elec/wiring_data.json ]; then
    run "elec: CAN load and ground offset" bash -c "cd '$ROOT/elec' && '$PY' can_ground_check.py | tail -3"
  else
    SKIP+=("elec: CAN load and ground offset (needs wiring_data.json: cad_src/rebuild.sh)")
  fi
fi

if want pi; then
  if "$PY" -c "import pytest, aiohttp" 2>/dev/null; then
    run "pi: software test suite" bash -c "cd '$ROOT/pi' && '$PY' -m pytest -q tests"
  else
    SKIP+=("pi: software test suite (pip install 'pytest>=8' pytest-aiohttp)")
  fi
fi

if want sim; then
  run "sim: build the controller library" core/build_lib.sh
  if "$PY" -c "import mujoco" 2>/dev/null; then
    run "sim: regenerate the MuJoCo models" bash -c "cd '$ROOT/sim' && '$PY' build_model.py && '$PY' build_model.py --arena"
    run "sim: 30 randomised robots" bash -c "cd '$ROOT/sim' && '$PY' run_tests.py >/dev/null && '$PY' -c \"
import json; r = json.load(open('results/report.json'))
for k in ('stand-up from face-down', 'stand-up from back-down'): print(f'  {k}: {r[k][\\\"success\\\"]}')
print('  balance push recovery:', r['balance 10 s (randomised)']['push_recovered'])\""
  else
    SKIP+=("sim: model and simulation tests (no mujoco: ml/setup_mac.sh)")
  fi
fi

restore_artifacts

printf '\n\033[1m== summary\033[0m\n'
for s in ${PASS[@]+"${PASS[@]}"}; do printf '  \033[32mpass\033[0m  %s\n' "$s"; done
for s in ${SKIP[@]+"${SKIP[@]}"}; do printf '  \033[33mskip\033[0m  %s\n' "$s"; done
for s in ${FAIL[@]+"${FAIL[@]}"}; do printf '  \033[31mFAIL\033[0m  %s\n' "$s"; done
if [ ${#DRIFT[@]} -gt 0 ]; then
  printf '\n  \033[33mregenerated output differs from the committed copy:\033[0m\n'
  for f in "${DRIFT[@]}"; do printf '    %s\n' "$f"; done
  printf '  (the committed files were left untouched. Small last-digit differences are\n'
  printf '   normal across platforms; a changed success count is not.)\n'
fi
[ ${#FAIL[@]} -eq 0 ] || { printf '\n%d section(s) failed\n' "${#FAIL[@]}"; exit 1; }
printf '\nall good\n'
