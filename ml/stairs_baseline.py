"""The hand-scripted nod, and the grid that tuned it: the baseline the learned policy has to beat.

Four numbers: approach at V_APP, brake one stopping distance before D_DIVE, command -V_REV, and flick
to PITCH once the robot is really reversing. Why that shape - and why a flick rather than a held lean -
is measured in sim/stairs_probe.py and quoted at the top of ml/stairs_env.py. The trigger is on SPEED,
not on distance: a flick entered while still rolling forwards does not just travel further, it diverges
(the body is still pitched forward when the balance law takes it back, the catch saturates and the
wheels run away past the 114 rad/s guard). Reversing first is not an optimisation, it is the manoeuvre.

Everything runs through the environment itself, so the baseline and the policy are scored by exactly
the same simulator, noise, estimator and reward. The script only ever uses one of its three nods: it
has no way to decide whether a reading was any good, which is one of the things the policy can learn.

  python ml/stairs_baseline.py --grid            # search the recipe (fixed 100 x 300 mm steps)
  python ml/stairs_baseline.py --trials 20       # score the chosen recipe on randomised staircases
"""
import argparse, math, pathlib, sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from stairs_env import (StairsLookEnv, HOLD, CTRL_DT, V_MAX, NOD_LO, NOD_HI,   # noqa: E402
                        SWING_END_LO, SWING_END_HI)

BEST = dict(v_app=0.30, d_dive=0.40, v_rev=0.40, pitch=0.75)
BRAKE_EFF = 0.9   # m/s^2 actually achieved when braking: max_accel limits the speed REFERENCE,
REACT = 0.10      # s, and the lean that delivers it takes about this long to build up.


class Scripted:
    """Three phases: approach, turn around, flick. The depth is turned back into the env's action."""

    def __init__(self, **p):
        self.p = dict(BEST, **p)
        self.reset()

    def reset(self):
        self.phase, self.done = 0, False

    def act(self, obs):
        p = self.p
        a2 = 2.0 * (p["pitch"] - NOD_LO) / (NOD_HI - NOD_LO) - 1.0
        d_est = float(obs[0]) * 1.5
        if self.done:                                        # one nod only: coast, pulling back
            return np.array([-p["v_rev"] / V_MAX, -1.0, a2])
        if self.phase == 0:                                  # approach, braking one stopping distance early
            if d_est > p["d_dive"] + p["v_app"] * REACT + p["v_app"] ** 2 / (2 * BRAKE_EFF):
                return np.array([p["v_app"] / V_MAX, -1.0, a2])
            self.phase = 1
        if self.phase == 1:                                  # turn around; nod only once really reversing
            if float(obs[1]) * V_MAX > -0.8 * p["v_rev"]:
                return np.array([-p["v_rev"] / V_MAX, -1.0, a2])
            self.phase = 2
        self.done = True
        return np.array([-p["v_rev"] / V_MAX, +1.0, a2])     # flick, still commanding reverse


CREEP = dict(v_app=0.35, v_creep=0.12, creep_from=0.80, standoff=0.48, depth=0.95)


class Creep:
    """Creep in slowly, stop at a standoff, then swing. The second recipe, for StairsLookEnv(swing=True).

    Only three numbers, because the swing removes the one the flick recipe spends most of its tuning on:
    a flick has to be entered reversing at the right speed or it costs a metre of travel, and a swing
    does not care - it leans back for as long as it leans forward.
    """

    def __init__(self, **p):
        self.p = dict(CREEP, **p)
        self.reset()

    def reset(self):
        self.done = False

    def act(self, obs):
        p = self.p
        a2 = 2.0 * (p["depth"] - SWING_END_LO) / (SWING_END_HI - SWING_END_LO) - 1.0
        d_est, v = float(obs[0]) * 1.5, float(obs[1]) * V_MAX
        # fast until the last stretch, then creep: a slow final approach is what keeps the stop
        # accurate, and the standoff has to be right to about 50 mm for the swing to see anything
        speed = p["v_app"] if d_est > p["creep_from"] else p["v_creep"]
        if self.done or d_est > p["standoff"] + speed * REACT + speed ** 2 / (2 * BRAKE_EFF):
            return np.array([(0.0 if self.done else speed) / V_MAX, -1.0, a2])
        if v > 0.04:                                         # stop first: look from standing still
            return np.array([0.0, -1.0, a2])
        self.done = True
        return np.array([0.0, +1.0, a2])


# --------------------------------------------------------------------------- stop, wind, look, judge
# What it costs to take the look from a standstill is measured in sim/stairs_windup.py. The short
# version: a one-sided flick is a forward acceleration, and from a dead stop at 0.75 rad it carries the
# robot +899 mm towards the drop. Reversing for 0.9 s first - still starting from a standstill, still
# one forward look - reaches -0.47 m/s, and the same flick then costs +62 +- 86 mm with every robot
# still upright. The wind-up is what makes "stop, settle, look once" affordable at an angle that sees.
# each retry closes in AND looks deeper: a different standoff changes which part of the tread the nose
# occludes, and a deeper look is what a tall riser needs - the two failures the first look leaves
# every retry is the SAME cheap look from a different standoff. Going deeper was tried and is a trap:
# 0.75 rad costs +62 mm of forward travel and 0.90 rad costs +504 mm (sim/stairs_windup.py), so a deep
# retry spends exactly the margin it is trying to earn.
PROBE = dict(v_app=0.35, v_creep=0.15, stand=(0.40, 0.34, 0.30), pitch=(0.75, 0.75, 0.78),
             wind=0.90, v_wind=0.50, settle_pitch=0.035, settle_rate=0.5, settle_max=1.2, conf_ok=0.45)


class Probe:
    """Creep to a standoff, stand still and straight, wind back, look once, then judge the look.

    The judging is the point. `stairs_map.reconstruct` refuses to answer when the surface it found
    cannot be the first tread - too few returns on it, or it starts behind the edge, or too far past
    it - so "no answer" is a state the recipe can act on rather than a number it has to trust. On a
    refusal it moves one standoff closer and looks again: a different viewpoint changes which part of
    the tread is occluded by the nose, and that is exactly what a second look is for.

    Needs StairsLookEnv(max_nods=3, t_max=..., estimator="map"); with the single-shot estimator it
    still runs, but it has nothing to judge and simply takes all three looks.
    """

    def __init__(self, env=None, **p):
        self.p = dict(PROBE, **p)
        self.env = env
        self.reset()

    def reset(self):
        self.phase, self.look, self.wind_t, self.done = "approach", 0, 0.0, False
        self.settle_t = 0.0

    def _conf(self):
        """What the reconstruction made of the looks so far - computed on board, from its own cloud."""
        return 0.0 if self.env is None else float(getattr(self.env, "best_conf", 0.0))

    def act(self, obs):
        p = self.p
        i = min(self.look, len(p["stand"]) - 1)
        stand, depth = p["stand"][i], p["pitch"][i]
        a2 = 2.0 * (min(depth, NOD_HI) - NOD_LO) / (NOD_HI - NOD_LO) - 1.0
        d_est, v, pitch, prate = float(obs[0]) * 1.5, float(obs[1]) * V_MAX, float(obs[2]), float(obs[3]) * 10.0
        if self.done:
            return np.array([0.0, -1.0, a2])
        if self.phase == "approach":
            speed = p["v_app"] if d_est > stand + 0.30 else p["v_creep"]
            if d_est > stand + speed * REACT + speed ** 2 / (2 * BRAKE_EFF):
                return np.array([speed / V_MAX, -1.0, a2])
            self.phase = "settle"
        if self.phase == "settle":
            # straight and still, so the look starts from a state that is the same every time. A real
            # robot cannot wait for ever either: some robots carry a steady pitch offset they never
            # settle out of, so the wait is capped and the look is taken anyway.
            self.settle_t += HOLD * CTRL_DT
            if (abs(v) > 0.03 or abs(pitch) > p["settle_pitch"] or abs(prate) > p["settle_rate"]) \
                    and self.settle_t < p["settle_max"]:
                return np.array([0.0, -1.0, a2])
            self.phase, self.wind_t, self.settle_t = "wind", 0.0, 0.0
        if self.phase == "wind":
            self.wind_t += HOLD * CTRL_DT
            if self.wind_t < p["wind"]:
                return np.array([-p["v_wind"] / V_MAX, -1.0, a2])
            self.phase = "look"
        if self.phase == "look":
            self.look += 1
            self.phase = "judge"
            return np.array([-p["v_wind"] / V_MAX, +1.0, a2])     # flick, still commanding reverse
        # judge: a good enough reconstruction ends it, otherwise close in and look again
        if self._conf() >= p["conf_ok"] or self.look >= len(p["stand"]):
            self.done = True
            if self.env is not None:
                self.env.stop_requested = True      # finished: do not stand still until the clock ends
            return np.array([0.0, -1.0, a2])
        self.phase = "approach"
        return np.array([0.0, -1.0, a2])


def run(env, pol, **opts):
    obs, _ = env.reset(options=opts)
    pol.reset()
    while True:
        obs, r, done, _, info = env.step(pol.act(obs))
        if done:
            return r, info


def score(env, pol, trials, seed0=9000, rise=None, tread=None, d0=None):
    rows = []
    for k in range(trials):
        env.reset(seed=seed0 + k)
        o = {k2: v for k2, v in (("rise", rise), ("tread", tread), ("d0", d0)) if v}
        rows.append(run(env, pol, **o))
    info = [i for _, i in rows]
    errs = [i["rise_err_mm"] for i in info if i["measured"]]
    marg = [i["margin_mm"] for i in info]
    dpk = [i["d_peak_mm"] for i in info if not math.isnan(i["d_peak_mm"])]
    m = lambda k: float(np.mean([i[k] for i in info]))
    return dict(reward=float(np.mean([r for r, _ in rows])), n=trials,
                success=m("success"), measured=m("measured"), fell=m("fell"), off=m("off_edge"),
                nods=m("nods"), err=float(np.mean(errs)) if errs else float("nan"),
                err_sd=float(np.std(errs)) if errs else float("nan"),
                margin=float(np.mean(marg)), margin_min=float(np.min(marg)),
                d_peak=float(np.mean(dpk)) if dpk else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--rise", type=float, default=None)
    ap.add_argument("--tread", type=float, default=None)
    a = ap.parse_args()
    env = StairsLookEnv()
    if a.grid:
        print(f"\n\033[1mGrid: scripted flick on 100 x 300 mm steps, {a.trials} randomised robots each\033[0m")
        print(f"{'d_dive':>7} {'v_rev':>6} {'v_app':>7} {'pitch':>6} | {'reward':>7} {'meas':>5} "
              f"{'riser err':>14} {'d at peak':>10} {'margin':>8} {'min':>7} {'fell':>5} {'off':>5} {'ok':>5}")
        best = None
        for d_dive in (0.20, 0.28, 0.36):
            for v_rev in (0.30, 0.45, 0.60):
                for v_app in (0.25, 0.40):
                    for pitch in (0.70, 0.80, 0.90, 1.00):
                        pol = Scripted(d_dive=d_dive, v_rev=v_rev, v_app=v_app, pitch=pitch)
                        s = score(env, pol, a.trials, rise=0.10, tread=0.30, d0=1.2)
                        print(f"{d_dive:7.2f} {v_rev:6.2f} {v_app:7.2f} {pitch:6.2f} | {s['reward']:7.2f} "
                              f"{s['measured']:5.2f} {s['err']:8.0f} +-{s['err_sd']:3.0f} mm "
                              f"{s['d_peak']:7.0f} mm {s['margin']:5.0f} mm {s['margin_min']:4.0f} mm "
                              f"{s['fell']:5.2f} {s['off']:5.2f} {s['success']:5.2f}")
                        if best is None or s["reward"] > best[0]["reward"]:
                            best = (s, dict(d_dive=d_dive, v_rev=v_rev, v_app=v_app, pitch=pitch))
        print(f"\n  best: {best[1]}  reward {best[0]['reward']:.2f}, measured {best[0]['measured']:.2f}, "
              f"riser error {best[0]['err']:.0f} mm, success {best[0]['success']:.2f}")
    else:
        s = score(env, Scripted(), a.trials, rise=a.rise, tread=a.tread)
        print(f"\n\033[1mScripted baseline {BEST}\033[0m")
        print(f"  {s['n']} episodes: reward {s['reward']:+.2f}, measured {s['measured']:.2f}, "
              f"riser error {s['err']:.0f} +- {s['err_sd']:.0f} mm, d at peak pitch {s['d_peak']:.0f} mm,\n"
              f"  edge margin {s['margin']:.0f} mm (min {s['margin_min']:.0f}), fell {s['fell']:.2f}, "
              f"off the edge {s['off']:.2f}, success {s['success']:.2f}")
    env.close()


if __name__ == "__main__":
    main()
