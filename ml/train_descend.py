"""Train the step-down policy with PPO, one simulator per core.

  python ml/train_descend.py --steps 3000000 --envs 12
  python ml/train_descend.py --steps 1000000 --resume ml/runs/descend/latest.zip

The hand-written recipe it has to beat is in sim/stairs_descend.py: roll at a fixed speed, commit to
step_mode a fixed distance before the edge, hold a fixed attitude. Three constants, tuned on a grid,
scoring 57% in this environment. The opening is that those constants cannot depend on the step - and
the robot knows the riser to about 13 mm before it moves, because the array measured it.

What to watch, in this order (all under env/ in TensorBoard):
  crossed_rate   did it step at all. If this collapses to 0 the reward is wrong again: refusing to
                 step is always the safe choice, and NO_STEP exists to make it the wrong one.
  success_rate   landed, settled, and stayed settled
  peak_deg       how far the body went over. The recipe runs at 8-12 deg; a tumble is 46.
  fell_rate / ran_on_rate   the two ways it loses: on its face, or across the tread and off the next

Do NOT judge a run by best.zip - see the note in CLAUDE.md, it cost a whole run on the stand-up.
ml/evaluate_descend.py scores every checkpoint on held-out staircases, and that is the gate.
"""
import argparse, collections, os, pathlib, shutil, sys
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from descend_env import DescendEnv, RISE_HI  # noqa: E402

OUT = pathlib.Path(__file__).parent / "runs" / "descend"
KEYS = ("success", "fell", "ran_on", "no_step", "crossed", "peak_deg", "rise_mm", "t_end",
        "committed", "end_pitch_deg")


class Curriculum(BaseCallback):
    """Raise the tallest riser once the policy can handle the ones it is getting.

    Ratchets on SUCCESS, not on crossing: a gate that advances on merely stepping would hand the
    policy taller and taller steps it cannot land, which is how the stand-up's curriculum silently
    stalled for 2 M steps. It never goes back down, and it stops at the real distribution.
    """

    def __init__(self, stats, start=0.07, step=0.015, up_at=0.55, window=300, every=5000):
        super().__init__()
        self.stats, self.hi, self.step, self.up_at = stats, start, step, up_at
        self.window, self.every = window, every

    def _on_step(self):
        if self.n_calls % self.every or self.hi >= RISE_HI - 1e-6:
            return True
        d = self.stats.buf["success"]
        if len(d) >= self.window and float(np.mean(d)) > self.up_at:
            self.hi = min(RISE_HI, self.hi + self.step)
            self.training_env.env_method("set_rise_hi", self.hi)
            d.clear()
            print(f"  curriculum -> risers up to {self.hi*1000:.0f} mm")
        self.logger.record("env/curriculum_rise_hi_mm", self.hi * 1000)
        return True


class EnvStats(BaseCallback):
    """Rolling means of every info key, so a bad reward shows up in minutes rather than at the end."""

    def __init__(self, window=200):
        super().__init__()
        self.buf = {k: collections.deque(maxlen=window) for k in KEYS}

    def _on_step(self):
        for info in self.locals.get("infos", []):
            if "success" not in info:
                continue
            for k in KEYS:
                if k in info:
                    self.buf[k].append(float(info[k]))
        if self.n_calls % 500 == 0:
            for k, d in self.buf.items():
                if d:
                    name = f"env/{k}" + ("_rate" if k in ("success", "fell", "ran_on", "no_step",
                                                          "crossed", "committed") else "")
                    self.logger.record(name, float(np.mean(d)))
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=3_000_000)
    ap.add_argument("--envs", type=int, default=12)
    ap.add_argument("--resume", default=None)
    ap.add_argument("--curriculum", action="store_true",
                    help="start on small risers and ratchet up; pointless from a warm start, which "
                         "already steps on the full distribution")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    ck = OUT / "ckpt"
    if not a.resume and ck.exists():
        # checkpoints are named by step count, so a shorter run leaves older, longer-numbered files
        # behind and the scorer happily picks one from a different reward. Clear them.
        shutil.rmtree(ck)
    ck.mkdir(parents=True, exist_ok=True)

    env = make_vec_env(DescendEnv, n_envs=a.envs, vec_env_cls=SubprocVecEnv,
                       env_kwargs=dict(curriculum=a.curriculum))
    if a.resume:
        model = PPO.load(a.resume, env=env, device="cpu")
        print(f"resumed from {a.resume}")
    else:
        model = PPO("MlpPolicy", env, verbose=1, device="cpu", tensorboard_log=str(OUT),
                    n_steps=256, batch_size=512, gamma=0.99, gae_lambda=0.95,
                    learning_rate=3e-4, ent_coef=0.004, clip_range=0.2,
                    policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]), log_std_init=0.0))
    stats = EnvStats()
    cbs = [stats] + ([Curriculum(stats)] if a.curriculum else []) + [CheckpointCallback(save_freq=max(1, 100_000 // a.envs),
                                          save_path=str(ck), name_prefix="descend")]
    model.learn(total_timesteps=a.steps, callback=cbs, progress_bar=False,
                reset_num_timesteps=not a.resume)
    model.save(str(OUT / "latest.zip"))
    print(f"saved {OUT / 'latest.zip'}")


if __name__ == "__main__":
    main()
