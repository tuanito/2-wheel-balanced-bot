"""Score a step-down policy on held-out staircases. THIS is the gate, not the training reward.

  python ml/evaluate_descend.py                                   # the scripted recipe
  python ml/evaluate_descend.py ml/runs/descend/bc.zip
  python ml/evaluate_descend.py --all                             # every checkpoint of the run

Seeds 40000+ are never used in training or in the warm start. `--all` exists because best.zip is
not to be trusted: the stand-up lost a whole run to that, and the per-episode reward here is just as
bimodal (it lands or it does not), so every candidate gets scored and the winner is the winner.
"""
import argparse, pathlib, sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from descend_env import DescendEnv, V_MAX, STEP_LO, STEP_HI  # noqa: E402
from warmstart_descend import recipe                          # noqa: E402

RUN = pathlib.Path(__file__).parent / "runs" / "descend"


class Learned:
    def __init__(self, path):
        from stable_baselines3 import PPO
        self.m = PPO.load(path, device="cpu")

    def reset(self):
        pass

    def act(self, obs):
        return self.m.predict(np.asarray(obs, dtype=np.float32), deterministic=True)[0]


class Scripted:
    def reset(self):
        self.st = {}

    def act(self, obs):
        return recipe(obs, self.st)


def score(pol, trials=60, seed0=40_000, rise=None):
    env = DescendEnv(seed=seed0)
    rows = []
    for k in range(trials):
        obs, _ = env.reset(seed=seed0 + k, options=dict(rise=rise) if rise else None)
        pol.reset()
        while True:
            obs, r, done, _, info = env.step(pol.act(obs))
            if done:
                break
        rows.append(info)
    env.close()
    m = lambda k: float(np.mean([x[k] for x in rows]))
    peaks = [x["peak_deg"] for x in rows if x["crossed"]]
    return dict(success=m("success"), crossed=m("crossed"), fell=m("fell"), ran_on=m("ran_on"),
                no_step=m("no_step"), peak=float(np.mean(peaks)) if peaks else float("nan"), n=trials)


def line(tag, s):
    print(f"  {tag:<34} ok={s['success']:5.2f}  stepped={s['crossed']:5.2f}  fell={s['fell']:5.2f}  "
          f"ran on={s['ran_on']:5.2f}  never={s['no_step']:5.2f}  peak={s['peak']:5.1f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", nargs="?", default=None)
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    print(f"\n\033[1mheld-out staircases (seeds 40000+), {a.trials} each\033[0m")
    if a.all:
        cands = sorted(RUN.glob("ckpt/*.zip"), key=lambda p: int(p.stem.split("_")[1])) + \
                [p for p in (RUN / "latest.zip", RUN / "bc.zip") if p.exists()]
        best = (None, -1)
        line("scripted recipe", score(Scripted(), a.trials))
        for c in cands:
            s = score(Learned(c), a.trials)
            line(c.name, s)
            if s["success"] > best[1]:
                best = (c, s["success"])
        print(f"\n  winner: {best[0].name if best[0] else '--'} at {best[1]:.2f}")
        return
    pol = Learned(a.model) if a.model else Scripted()
    line(pathlib.Path(a.model).name if a.model else "scripted recipe", score(pol, a.trials))
    if a.sweep:
        for rise in (0.06, 0.08, 0.10, 0.13, 0.15, 0.17):
            line(f"  riser {rise*1000:.0f} mm", score(pol, max(20, a.trials // 3), rise=rise))


if __name__ == "__main__":
    main()
