"""Warm-start the step-down policy by imitating the hand-written recipe, then let PPO improve it.

Starting PPO from noise does not work here and it is worth being precise about why. The recipe is
three decisions that have to coincide: roll at a moderate speed, commit within a couple of
centimetres of an edge, and stop asking for speed. Miss any one and the robot tumbles. A random
policy crosses the edge readily - that part is easy - and lands essentially never, so every early
trajectory ends the same way and there is no gradient towards the one combination that works.

So this clones the recipe first. It is a few hundred episodes of supervised learning on (observation,
action) pairs, which costs minutes and puts the policy somewhere PPO can actually improve from. The
recipe scores about 57% in this environment; anything the training does afterwards is measured
against that, not against zero.

  python ml/warmstart_descend.py --episodes 400
  python ml/train_descend.py --resume ml/runs/descend/bc.zip --steps 3000000
"""
import argparse, pathlib, sys
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from descend_env import DescendEnv, V_MAX, STEP_LO, STEP_HI  # noqa: E402

OUT = pathlib.Path(__file__).parent / "runs" / "descend"


def recipe(obs, state, v_roll=0.20, arm=0.020, hold=0.0):
    """The tuned recipe as an action: roll, commit just before the edge, then stop asking for speed.

    Reads only what the robot has - the array's distance estimate - so the demonstrations are
    reachable by the policy rather than being privileged.
    """
    d = float(obs[0]) * 0.5
    if d < 0.0:
        state["past"] = True
    v = 0.0 if state.get("past") else v_roll
    return np.array([2.0 * v / V_MAX - 1.0,
                     1.0 if d < arm else -1.0,
                     2.0 * (hold - STEP_LO) / (STEP_HI - STEP_LO) - 1.0], dtype=np.float32)


def collect(episodes, seed0=7000, jitter=True):
    """Demonstrations, with the recipe's own constants jittered a little.

    Cloning one exact trajectory teaches the policy a single reflex; jittering the speed and the
    commit distance across episodes shows it the shape of the decision instead, which is what we
    want it to keep when PPO starts moving it.
    """
    env = DescendEnv(seed=seed0)
    X, Y, ok = [], [], 0
    rng = np.random.default_rng(seed0)
    for k in range(episodes):
        obs, _ = env.reset(seed=seed0 + k)
        v = float(rng.uniform(0.15, 0.26)) if jitter else 0.20
        arm = float(rng.uniform(0.012, 0.030)) if jitter else 0.020
        hold = float(rng.uniform(-0.04, 0.02)) if jitter else 0.0
        st, ep_x, ep_y = {}, [], []
        while True:
            a = recipe(obs, st, v, arm, hold)
            ep_x.append(np.asarray(obs, dtype=np.float32)); ep_y.append(a)
            obs, r, done, _, info = env.step(a)
            if done:
                break
        if info["success"]:                      # clone what WORKED, not what merely happened
            X += ep_x; Y += ep_y; ok += 1
        if (k + 1) % 50 == 0:
            print(f"  {k+1}/{episodes} episodes, {ok} usable ({ok/(k+1):.0%}), {len(X)} samples")
    env.close()
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32), ok / max(1, episodes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=400)
    ap.add_argument("--epochs", type=int, default=60)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    X, Y, rate = collect(a.episodes)
    print(f"collected {len(X)} samples from {rate:.0%} successful demonstrations")
    if len(X) < 500:
        raise SystemExit("too few successful demonstrations to clone - check the recipe constants")

    env = make_vec_env(DescendEnv, n_envs=1, env_kwargs=dict(curriculum=False))
    model = PPO("MlpPolicy", env, device="cpu", n_steps=256, batch_size=512,
                policy_kwargs=dict(net_arch=dict(pi=[64, 64], vf=[64, 64]), log_std_init=0.0))
    pol = model.policy
    opt = torch.optim.Adam(pol.parameters(), lr=1e-3)
    xt, yt = torch.as_tensor(X), torch.as_tensor(Y)
    n = len(xt)
    for ep in range(a.epochs):
        perm = torch.randperm(n)
        tot = 0.0
        for i in range(0, n, 256):
            idx = perm[i:i + 256]
            feat = pol.extract_features(xt[idx])
            if isinstance(feat, tuple):
                feat = feat[0]
            mean = pol.action_net(pol.mlp_extractor.forward_actor(feat))
            loss = torch.nn.functional.mse_loss(mean, yt[idx])
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss) * len(idx)
        if ep % 10 == 0 or ep == a.epochs - 1:
            print(f"  epoch {ep:3d}  mse {tot/n:.4f}")
    model.save(str(OUT / "bc.zip"))
    print(f"saved {OUT / 'bc.zip'}")


if __name__ == "__main__":
    main()
