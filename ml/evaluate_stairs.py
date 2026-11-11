"""Score a nod-timing policy, or the scripted baseline, on held-out staircases.

  python ml/evaluate_stairs.py                              # the scripted baseline
  python ml/evaluate_stairs.py ml/runs/stairs/latest.zip     # a trained policy
  python ml/evaluate_stairs.py ml/runs/stairs/latest.zip --sweep    # per-riser breakdown

The gate is this script, not the eval reward in TensorBoard: the per-episode reward is nearly bimodal
(the riser comes out or it does not), which is exactly the kind of signal that made `best.zip` useless
for the stand-up. Seeds 20000+ are never used in training.

A success means all three at once: the riser measured to within 20 mm, the robot still upright, and at
least 100 mm of landing left between the wheels and the drop at the worst moment of the episode.
"""
import argparse, math, pathlib, sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from stairs_env import StairsLookEnv, RISE_LO, RISE_HI          # noqa: E402
from stairs_baseline import Scripted, score                      # noqa: E402


class Learned:
    """The trained policy, acting deterministically (the mean action), exactly as the robot would."""

    def __init__(self, path):
        from stable_baselines3 import PPO
        self.model = PPO.load(path, device="cpu")

    def reset(self):
        pass

    def act(self, obs):
        return self.model.predict(np.asarray(obs, dtype=np.float32), deterministic=True)[0]


def line(tag, s):
    print(f"  {tag:<22} {s['success']:5.2f}  {s['measured']:5.2f}  {s['err']:6.0f} +-{s['err_sd']:4.0f} mm  "
          f"{s['d_peak']:6.0f} mm  {s['margin']:6.0f} mm {s['margin_min']:6.0f} mm  {s['fell']:5.2f} {s['off']:5.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", nargs="?", default=None)
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--sweep", action="store_true", help="break the result down by riser height")
    ap.add_argument("--seed", type=int, default=20_000)
    a = ap.parse_args()
    env = StairsLookEnv()
    pol = Learned(a.model) if a.model else Scripted()
    tag = pathlib.Path(a.model).name if a.model else "scripted baseline"
    print(f"\n\033[1m{tag}: {a.trials} held-out episodes (seeds {a.seed}+)\033[0m")
    print(f"  {'':<22} {'ok':>5}  {'meas':>5}  {'riser error':>13}  {'d at peak':>9}  "
          f"{'margin':>9} {'worst':>9}  {'fell':>5} {'off':>5}")
    line("randomised stairs", score(env, pol, a.trials, seed0=a.seed))
    if a.sweep:
        for rise in (0.08, 0.10, 0.13, 0.15, 0.17):
            line(f"riser {rise*1000:.0f} mm", score(env, pol, max(10, a.trials // 2),
                                                    seed0=a.seed + 1000, rise=rise, tread=0.30))
    env.close()


if __name__ == "__main__":
    main()
