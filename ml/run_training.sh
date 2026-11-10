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

# Which checkpoint? EvalCallback writes best.zip by eval REWARD, and that metric is far too noisy
# here to rank anything (sigma ~2.5 on a mean of ~8). One run's "best" was a 100k-step checkpoint
# that scored 26/60 compiled while the same run's 2M-step policy scored 171/180 - and it was
# exported and reported as the result. So score the candidates on the gate metric itself.
score_of() {               # -> "<successes out of 60>" for the policy currently in the library
  # sed, not awk: BSD awk (macOS) has no 3-argument match(), so a gawk-ism would silently
  # score every candidate 0 here and the choice below would be made on nothing.
  "$PY" ml/evaluate.py --compiled \
    | sed -nE 's#^compiled.*face-down ([0-9]+)/30.*back-down ([0-9]+)/30.*#\1 \2#p' \
    | { read -r a b; echo $((a + b)); }
}
BESTN=-1; WINNER=""; SEEN=""          # SEEN must exist before the case below: set -u is on
for CAND in "$BEST" ml/runs/standup/latest.zip; do
  [ -f "$CAND" ] || continue
  case " $SEEN " in *" $CAND "*) continue ;; esac     # --steps < ~100k makes BEST == latest.zip
  SEEN="$SEEN $CAND"
  step "scoring $(basename "$CAND") as the C++ controller runs it"
  "$PY" ml/export_policy.py "$CAND" >/dev/null
  core/build_lib.sh >/dev/null
  N="$(score_of)" || N=""
  [ -n "$N" ] || { echo "  $CAND: could not be scored (evaluate.py printed no 'compiled' line)" >&2; continue; }
  echo "  $CAND: $N/60"
  if [ "$N" -gt "$BESTN" ]; then BESTN="$N"; WINNER="$CAND"; fi
done
[ -n "$WINNER" ] || { echo "no checkpoint could be scored" >&2; exit 1; }

step "exporting the winner ($(basename "$WINNER"), $BESTN/60) to core/standup_policy.hpp"
"$PY" ml/export_policy.py "$WINNER"
core/build_lib.sh
firmware/esp32/sync_core.sh

step "learned vs hand-tuned, policy run from Python"
"$PY" ml/evaluate.py "$WINNER"

step "the policy inside the C++ controller: what the robot will actually run"
"$PY" ml/evaluate.py --compiled
echo
echo "If the two blocks above disagree by more than a couple of robots, the policy is sitting on a"
echo "knife edge: the C++ and torch forward passes differ by ~1e-7 and that is enough to flip it."
echo "Only the --compiled number is the gate."

cat <<'NEXT'

Done. core/standup_policy.hpp now holds the trained policy.
If --compiled does not clearly beat the hand-tuned stand-up, keep standup_mode = 0:
  git checkout -- core/standup_policy.hpp && core/build_lib.sh
To put it on the robot, see "On the robot" in ml/README.md.
NEXT
