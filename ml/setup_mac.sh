#!/usr/bin/env bash
# One-time setup on macOS (Apple Silicon or Intel): Python venv with MuJoCo + PPO, and the controller library.
# Needs: Xcode command-line tools (xcode-select --install) and Python 3.10+ (python.org or `brew install python`).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r ml/requirements.txt
# the same C++ controller the robot runs, as a macOS library for the simulator
clang++ -O2 -std=c++17 -shared -fPIC core/controller.cpp core/sim_api.cpp -o sim/libbot.dylib
python -c "import sys; sys.path.insert(0, 'ml'); from standup_env import StandupEnv; e = StandupEnv(); o, _ = e.reset(seed=0); print('env OK, first observation:', o)"
echo
echo "Ready. Next:  source .venv/bin/activate && python ml/train.py --steps 2000000"
