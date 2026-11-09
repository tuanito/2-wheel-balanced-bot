"""Compare the learned stand-up with the hand-tuned one on the same 60 randomised robots as sim/run_tests.py
(seeds 3000-3029, face-down and back-down), with the full controller in charge (5 s each, retries allowed).

  python ml/evaluate.py                              # the hand-tuned stand-up only
  python ml/evaluate.py ml/runs/standup/best.zip     # + the trained model, run from Python
After export_policy.py + rebuilding the library, `--compiled` runs the policy inside the C++ controller (mode 1),
which is exactly what the robot will run.
"""
import sys, math, pathlib
import numpy as np
from multiprocessing import Pool

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim")); sys.path.insert(0, str(ROOT / "ml"))
from botsim import BotSim, LIB, CTRL_DT  # noqa: E402


def trial(args):
    # the model path travels in the task, not in a global: multiprocessing uses spawn on
    # macOS, where a worker re-imports this module and would not see a module-level MODEL.
    seed, side, mode, model_path = args
    model = None
    if mode == "model":
        from stable_baselines3 import PPO
        model = PPO.load(model_path, device="cpu")
    s = BotSim(seed=seed, randomize=True)
    s.place(pitch=1.8 * side)
    LIB.bot_standup(s.ctl, {"hand": 0, "model": 2, "compiled": 1}[mode], 0.0)
    obs = (__import__("ctypes").c_float * 6)()
    tick = 0
    for _ in range(int(5.0 / CTRL_DT)):
        if model is not None and s.out.state == 2 and tick % 5 == 0:
            LIB.bot_standup_obs(s.ctl, obs)
            a, _ = model.predict(np.array(obs, dtype=np.float32), deterministic=True)
            LIB.bot_standup(s.ctl, 2, float(np.clip(a[0], -1, 1)) * 4.6)
        tick = tick + 1 if s.out.state == 2 else 0
        s.step()
    L = np.array(s.log); s.close()
    st = L[:, 1].astype(int)
    ok = st[-1] == 4 and np.abs(L[-500:, 2]).max() < math.radians(12)
    up = np.where(st == 4)[0]
    return ok, (L[up[0], 0] if len(up) else np.nan), float(np.hypot(L[-1, 9] - L[0, 9], L[-1, 10] - L[0, 10]))


def run(mode, model_path=None):
    with Pool() as p:
        R = p.map(trial, [(3000 + i, side, mode, model_path) for i in range(30) for side in (1, -1)])
    ok = [r[0] for r in R]
    print(f"{mode:9s} face-down {sum(ok[0::2])}/30  back-down {sum(ok[1::2])}/30  "
          f"median time to upright {np.nanmedian([r[1] for r in R]):.2f} s  median travel {np.median([r[2] for r in R]):.2f} m")


if __name__ == "__main__":
    run("hand")
    if "--compiled" in sys.argv: run("compiled")
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if args:
        run("model", args[0])
