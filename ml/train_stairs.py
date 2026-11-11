"""Train the nod-timing policy with PPO (stable-baselines3), one simulator per CPU core.

  python ml/train_stairs.py --steps 600000
  python ml/train_stairs.py --steps 600000 --resume ml/runs/stairs/latest.zip

Writes ml/runs/stairs/: checkpoints, latest.zip, best.zip (by the periodic evaluation) and TensorBoard
logs (`tensorboard --logdir ml/runs`). Then: python ml/evaluate_stairs.py ml/runs/stairs/latest.zip.

best.zip is NOT to be trusted on its own - see the note in CLAUDE.md. The eval reward here is just as
noisy as the stand-up's was (the riser either comes out or it does not, so the per-episode reward is
nearly bimodal between 0 and 10), so ml/evaluate_stairs.py scores every candidate on held-out
staircases and that is the gate.

What to watch, in this order (all under env/ in TensorBoard):
  measured_rate   did the cloud contain the first tread at all - the sensing
  d_peak_mm       where the pitch peaked: the whole game is to land this in 400-480 mm
  rise_err_mm     the actual measurement error
  margin_mm       how close to the drop it ended up, and off_edge_rate / fell_rate
A run where measured_rate climbs but margin_mm sinks is learning to measure by falling downstairs.
"""
import argparse, collections, os, pathlib, sys
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from stairs_env import StairsLookEnv  # noqa: E402

OUT = pathlib.Path(__file__).parent / "runs" / "stairs"
KEYS = ("success", "measured", "rise_err_mm", "margin_mm", "d_peak_mm", "peak_deg",
        "d_best_mm", "peak_best_deg", "fell", "off_edge", "no_nod", "n_pts", "conf", "nods", "t_end")


class EnvStats(BaseCallback):
    """Rolling means of what the environment is actually doing (NaNs skipped: a missed riser has no error)."""

    def __init__(self, n=200):
        super().__init__()
        self.q = {k: collections.deque(maxlen=n) for k in KEYS}

    def _on_step(self):
        for i in self.locals.get("infos", ()):
            for k in KEYS:
                v = i.get(k)
                if v is not None and not (isinstance(v, float) and np.isnan(v)):
                    self.q[k].append(float(v))
        for k, d in self.q.items():
            if d:
                name = k if k.endswith(("_mm", "_deg", "_pts")) or k in ("conf", "nods", "t_end") else k + "_rate"
                self.logger.record(f"env/{name}", sum(d) / len(d))
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=600_000, help="policy steps (50 ms each) in total")
    ap.add_argument("--envs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--resume", type=str, default=None)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    env = make_vec_env(StairsLookEnv, n_envs=a.envs, seed=a.seed,
                       vec_env_cls=SubprocVecEnv if a.envs > 1 else None)
    eval_env = make_vec_env(StairsLookEnv, n_envs=1, seed=10_000)
    if a.resume:
        model = PPO.load(a.resume, env=env, device="cpu")
    else:
        model = PPO("MlpPolicy", env, device="cpu", seed=a.seed, verbose=1, tensorboard_log=str(OUT / "tb"),
                    n_steps=256, batch_size=512, n_epochs=10, gamma=0.99, gae_lambda=0.95,
                    learning_rate=3e-4, ent_coef=0.01, clip_range=0.2,
                    policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]), log_std_init=0.0))
        # gamma 0.99: an episode is ~30-100 steps of approach and the reward is all at the end, so the
        # horizon has to cover the whole approach (0.99^100 = 0.37) without being so long that the
        # per-step time cost disappears. n_steps 256 x envs still gives whole episodes per update.
    cbs = [EnvStats(),
           CheckpointCallback(save_freq=max(1, 100_000 // a.envs), save_path=str(OUT / "ckpt"),
                              name_prefix="stairs"),
           EvalCallback(eval_env, n_eval_episodes=30, eval_freq=max(1, 50_000 // a.envs),
                        deterministic=True, best_model_save_path=str(OUT), log_path=str(OUT / "eval"))]
    model.learn(total_timesteps=a.steps, callback=cbs, reset_num_timesteps=a.resume is None)
    model.save(OUT / "latest")
    if (OUT / "best_model.zip").exists():
        os.replace(OUT / "best_model.zip", OUT / "best.zip")
    print("saved", OUT / "latest.zip")


if __name__ == "__main__":
    main()
