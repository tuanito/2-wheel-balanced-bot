#!/usr/bin/env bash
# The whole learned stand-up loop (ml/README.md), in order:
#   baseline -> train -> evaluate -> export to C++ -> REBUILD THE LIBRARY -> evaluate what the robot runs.
#
#   ml/run_training.sh                                   # 2,000,000 steps
#   ml/run_training.sh --steps 500000
#   ml/run_training.sh --resume ml/runs/standup/latest.zip
#   ml/run_training.sh --steps 50000 --no-baseline       # quick smoke run
#
# The rebuild before the last step is the one that is easy to forget by hand: without it
# `evaluate.py --compiled` silently measures the PREVIOUS policy and looks like a real result.
#
# Writes: ml/runs/standup/ (checkpoints, best.zip, TensorBoard logs) and core/standup_policy.hpp.
# Progress while it trains:  tensorboard --logdir ml/runs
set -euo pipefail
cd "$(dirname "$0")/.."
PY="$PWD/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"

STEPS=2000000
BASELINE=1
RESUME=()
while [ $# -gt 0 ]; do
  case "$1" in
    --steps)       STEPS="$2"; shift 2 ;;
    --resume)      RESUME=(--resume "$2"); shift 2 ;;
    --no-baseline) BASELINE=0; shift ;;
    -h|--help)     sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

BEST=ml/runs/standup/best.zip
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

if [ "$BASELINE" = 1 ]; then
  step "baseline: the hand-tuned stand-up"
  "$PY" ml/evaluate.py
fi

step "training $STEPS steps"
"$PY" ml/train.py --steps "$STEPS" ${RESUME[@]+"${RESUME[@]}"}

# train.py only writes best.zip once EvalCallback has fired (every ~100k steps), although it
# always prints that it did. Short runs have latest.zip only.
if [ ! -f "$BEST" ]; then
  BEST=ml/runs/standup/latest.zip
  [ -f "$BEST" ] || { echo "training produced no model in ml/runs/standup/" >&2; exit 1; }
  echo "note: no best.zip yet (too few steps for an evaluation) - using $BEST"
fi

step "learned vs hand-tuned, policy run from Python"
"$PY" ml/evaluate.py "$BEST"

step "exporting the policy to core/standup_policy.hpp"
"$PY" ml/export_policy.py "$BEST"

step "rebuilding the controller library with the exported policy"
core/build_lib.sh

step "the policy inside the C++ controller: what the robot will actually run"
"$PY" ml/evaluate.py --compiled

cat <<'NEXT'

Done. core/standup_policy.hpp now holds the trained policy.
If --compiled does not clearly beat the hand-tuned stand-up, keep standup_mode = 0:
  git checkout -- core/standup_policy.hpp && core/build_lib.sh
To put it on the robot, see "On the robot" in ml/README.md.
NEXT
