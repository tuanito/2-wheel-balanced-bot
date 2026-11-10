"""Train the stand-up policy with PPO (stable-baselines3), one simulator per CPU core.

  python ml/train.py --steps 2000000            # on a Mac M-series: ~30-60 min (a guess until measured)
  python ml/train.py --steps 2000000 --resume ml/runs/standup/latest.zip

Writes ml/runs/standup/: checkpoints, latest.zip, best.zip (by the periodic evaluation) and TensorBoard logs
(`tensorboard --logdir ml/runs`). Then: python ml/evaluate.py, python ml/export_policy.py.
"""
import argparse, collections, os, pathlib, sys
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from standup_env import StandupEnv  # noqa: E402

OUT = pathlib.Path(__file__).parent / "runs" / "standup"


class EnvStats(BaseCallback):
    """Rolling means of what the environment is actually doing, logged to TensorBoard.

    Two 2 M-step runs failed for reasons that were invisible in the PPO scalars: the first never reached the
    catch, the second reached it far too fast. Both had to be diagnosed afterwards from a separate script.
    These three numbers would have shown either failure within the first 100 k steps.
    """

    def __init__(self, n=200):
        super().__init__()
        self.succ = collections.deque(maxlen=n)
        self.rate = collections.deque(maxlen=n)
        self.pitch = collections.deque(maxlen=n)
        self.v = collections.deque(maxlen=n)
        self.curric = collections.deque(maxlen=n)

    def _on_step(self):
        for i in self.locals.get("infos", ()):
            if "success" in i: self.succ.append(float(i["success"]))
            if "catch_rate" in i: self.rate.append(i["catch_rate"])
            if "catch_v" in i: self.v.append(i["catch_v"])
            if "curric" in i: self.curric.append(float(i["curric"]))
            if "start_pitch" in i: self.pitch.append(i["start_pitch"])
        if self.succ: self.logger.record("env/success_rate", sum(self.succ) / len(self.succ))
        if self.rate: self.logger.record("env/catch_rate_rad_s", sum(self.rate) / len(self.rate))
        if self.v: self.logger.record("env/catch_v_m_s", sum(self.v) / len(self.v))
        if self.curric: self.logger.record("env/curric_rate", sum(self.curric) / len(self.curric))
        if self.pitch: self.logger.record("env/start_pitch", sum(self.pitch) / len(self.pitch))
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000, help="policy steps (10 ms each) in total")
    ap.add_argument("--envs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--resume", type=str, default=None)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    env = make_vec_env(StandupEnv, n_envs=a.envs, seed=a.seed, vec_env_cls=SubprocVecEnv if a.envs > 1 else None)
    # evaluation is always the real task: no curriculum, so the eval curve stays comparable across runs
    eval_env = make_vec_env(StandupEnv, n_envs=1, seed=10_000, env_kwargs=dict(curriculum=False))
    if a.resume:
        model = PPO.load(a.resume, env=env, device="cpu")
    else:
        model = PPO("MlpPolicy", env, device="cpu", seed=a.seed, verbose=1, tensorboard_log=str(OUT / "tb"),
                    n_steps=512, batch_size=512, n_epochs=10, gamma=0.995, gae_lambda=0.95,
                    learning_rate=3e-4, ent_coef=0.01, clip_range=0.2,
                    policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]), log_std_init=0.0))
        # ent_coef and log_std_init 0.0: with ent_coef 0 the first run's action noise annealed from std 0.60
        # to 0.15 by 1 M steps, the reward stopped improving at 250 k and never recovered. 0.005 was not
        # enough either (0.97 -> 0.17 over the second run), hence 0.01.
        # batch_size 512 divides n_steps * n_envs exactly, so no truncated minibatch each update.
        # small tanh MLP on purpose: it is exported to C++ and runs on the ESP32-S3 every 10 ms
    cbs = [EnvStats(),
           CheckpointCallback(save_freq=max(1, 200_000 // a.envs), save_path=str(OUT / "ckpt"), name_prefix="standup"),
           EvalCallback(eval_env, n_eval_episodes=40, eval_freq=max(1, 100_000 // a.envs), deterministic=True,
                        best_model_save_path=str(OUT), log_path=str(OUT / "eval"))]
    model.learn(total_timesteps=a.steps, callback=cbs, reset_num_timesteps=a.resume is None)
    model.save(OUT / "latest")
    os.replace(OUT / "best_model.zip", OUT / "best.zip") if (OUT / "best_model.zip").exists() else None
    print("saved", OUT / "latest.zip", "and", OUT / "best.zip")


if __name__ == "__main__":
    main()
