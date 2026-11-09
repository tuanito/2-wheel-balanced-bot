#!/usr/bin/env bash
# One-time setup on macOS (Apple Silicon or Intel): Python venv with MuJoCo + PPO, and the controller library.
# Needs: Xcode command-line tools (xcode-select --install) and Python 3.10+ (python.org or `brew install python`).
#
#   ml/setup_mac.sh          # simulator + reinforcement learning + the tools ./check.sh needs
#   ml/setup_mac.sh --cad    # ... and CadQuery + the bought-part models, for cad_src/rebuild.sh
set -euo pipefail
cd "$(dirname "$0")/.."
CAD=0
for a in "$@"; do
  case "$a" in
    --cad) CAD=1 ;;
    *) echo "unknown option: $a (try --cad)" >&2; exit 2 ;;
  esac
done
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r ml/requirements.txt
# the test tools ./check.sh needs for the Pi test suite (pi/requirements.txt declares them)
pip install "pytest>=8" "pytest-aiohttp>=1.0"
if [ "$CAD" = 1 ]; then
  pip install cadquery                      # large: pulls in OCP, a few hundred MB
  models/fetch_models.sh                    # bought-part STEP models for the fit check
fi
# the same C++ controller the robot runs, as a library for the simulator
core/build_lib.sh
python -c "import sys; sys.path.insert(0, 'ml'); from standup_env import StandupEnv; e = StandupEnv(); o, _ = e.reset(seed=0); print('env OK, first observation:', o)"
echo
echo "Ready. Next:  source .venv/bin/activate && ./check.sh"
echo "              ml/run_training.sh --steps 2000000"
if [ "$CAD" = 1 ]; then
  echo "CAD: cad_src/rebuild.sh  (also needs models/bought/mf6015.step, see models/fetch_models.sh)"
fi
