# Learned stand-up (reinforcement learning)

Goal: replace the hand-tuned stand-up kick (lying → upright) with a small neural network trained in the
simulator, because the taller V1.1 body made the hand-tuned one fail ~15 % of the time (the kick reaches the
motors' speed limit before the body is up). Balancing stays with the LQR controller; only the push from lying
until the catch (|pitch| < 20°) is learned. The hand-tuned stand-up stays as the fallback.

## How it fits together

| Piece | What it does |
|---|---|
| `core/controller.cpp` | `standup_mode`: 0 = hand-tuned (default), 1 = neural policy, 2 = torque from outside (training). The policy's inputs come from `Controller::standup_obs()`, the same function in the sim and on the robot. |
| `core/standup_policy.hpp` | The policy as plain C++ arrays (generated). The placeholder in git has no policy, so mode 1 falls back to 0. |
| `ml/standup_env.py` | Gymnasium environment around `sim/botsim.py` (MuJoCo + the real C++ controller). Every episode is a randomised robot: mass, centre of mass, inertia, grip, battery, torque gain, CAN delay, IMU bias. |
| `ml/train.py` | PPO (stable-baselines3), one simulator per CPU core. |
| `ml/evaluate.py` | Hand-tuned vs learned on the 60 test robots of `sim/run_tests.py`. |
| `ml/export_policy.py` | Trained model → `core/standup_policy.hpp` (checks C++ math = PyTorch). |

Policy: tanh MLP 6 → 64 → 64 → 1, one action every 10 ms = push for both motors (fraction of 2 × 2.3 N·m).
Inputs (side-normalised): pitch, pitch rate, wheel speed, time, battery voltage, last action.
Reward: progress towards upright, small costs for fast wheels and big pushes, +10 if the normal controller
still holds it upright 1 s after the catch (minus distance travelled), −5 for falling over / running away.

## On the Mac

```bash
git clone https://github.com/tuanito/2-wheel-balanced-bot && cd 2-wheel-balanced-bot
ml/setup_mac.sh                                # venv + MuJoCo + PPO, builds sim/libbot.dylib
source .venv/bin/activate
python ml/evaluate.py                          # baseline: the hand-tuned stand-up
python ml/train.py --steps 2000000             # uses all cores but one; progress: tensorboard --logdir ml/runs
python ml/evaluate.py ml/runs/standup/best.zip # learned vs hand-tuned
python ml/export_policy.py ml/runs/standup/best.zip
clang++ -O2 -std=c++17 -shared -fPIC core/controller.cpp core/sim_api.cpp -o sim/libbot.dylib
python ml/evaluate.py --compiled               # the exported C++ policy inside the controller (what the robot runs)
```

Training time on an M-series Mac is not measured yet; a first useful policy should take tens of minutes to
about an hour. If the learned policy doesn't clearly beat the hand-tuned one on `--compiled`, keep mode 0.

## On the robot

1. Copy `core/controller.*` and `core/standup_policy.hpp` into `firmware/esp32/lib/botcore/` and build (`pio run`).
2. Set `standup_mode = 1` on the controller (e.g. in `main.cpp` after creating it). Mode 0 stays one line away.
3. First tests on a soft floor, robot tethered: a learned policy can do things the hand-tuned one never did.

Checked here (Linux, Nov 2026): environment passes gymnasium's checker, a 30 k-step smoke training runs end to end,
the export matches PyTorch to 1e-7, the controller compiles in the sim and in the ESP32 firmware, and mode 0
(default) behaves exactly as before.
