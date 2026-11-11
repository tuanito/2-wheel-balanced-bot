#!/usr/bin/env bash
# The whole stair-nod loop, in order. Mirrors ml/run_training.sh for the stand-up.
#
#   ml/run_stairs_training.sh [--steps 800000]
#
# The gate is ml/evaluate_stairs.py on held-out seeds (20000+), never the eval reward in TensorBoard:
# the per-episode reward here is nearly bimodal (the riser comes out or it does not), which is exactly
# the signal that made best.zip useless for the stand-up. So every candidate is scored, and the winner
# is reported - if none of them beats the scripted baseline, the baseline is what the robot should use.
set -eu
cd "$(dirname "$0")/.."
PY=".venv/bin/python"
[ -x "$PY" ] || PY="python3"
STEPS=800000
[ "${1:-}" = "--steps" ] && STEPS="$2"

echo "== the library the environment runs (the controller is in the loop, so this must be current)"
core/build_lib.sh

echo
echo "== baseline: the hand-scripted nod, on the held-out seeds"
"$PY" ml/evaluate_stairs.py --trials 30 --sweep

echo
echo "== train ($STEPS steps)"
"$PY" ml/train_stairs.py --steps "$STEPS"

echo
echo "== score every candidate on the gate, not on best.zip"
BESTN="-1"; WINNER=""
for CAND in ml/runs/stairs/best.zip ml/runs/stairs/latest.zip ml/runs/stairs/ckpt/*.zip; do
  [ -f "$CAND" ] || continue
  N="$("$PY" ml/evaluate_stairs.py "$CAND" --trials 30 | sed -nE 's/^  randomised stairs *([0-9.]+).*/\1/p')"
  [ -n "$N" ] || continue
  echo "  $CAND -> success $N"
  # integer compare in shell: scale the rate by 100
  NI="$(printf '%.0f' "$(echo "$N * 100" | bc -l)")"
  if [ "$NI" -gt "$BESTN" ]; then BESTN="$NI"; WINNER="$CAND"; fi
done
echo
echo "== winner: ${WINNER:-none} (success ${BESTN}%)"
[ -n "$WINNER" ] && "$PY" ml/evaluate_stairs.py "$WINNER" --trials 40 --sweep
