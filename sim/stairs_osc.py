"""Creep up to the edge, then OSCILLATE instead of flicking once: does it buy a closer look?

The single flick is one-sided, so gravity acts forward for the whole nod and the robot ends up 0.1-1.1 m
down range (sim/stairs_probe.py part C2). That forward travel is what keeps it from looking from close
in, and close in is where the first tread is visible at all: part C1 has the band at 41-50 deg from
200 mm and nothing at all beyond 550 mm.

A symmetric swing does not have that problem - it leans back for as long as it leans forward, so the
net travel is close to nothing (measured: 0.650 m standoff to 0.638 m across six cycles). What that
buys is safety, not closeness, and the distinction is the result of this script:

  * creeping to 150 mm is SAFE and USELESS. The robot gets there without trouble and the swing is
    stable (6/6 upright at amplitude 0.5), but it puts 0-3 returns on the first tread and reads the
    riser 469 mm wrong. Closer is worse, not better: the step nose subtends a larger angle the nearer
    you are, so the pitch needed to see past it rises - 41-50 deg at 200 mm, roughly 60-70 at 150 - and
    above about 0.8 rad of amplitude the swing loses the robot. The limit is the lidar being 231 mm up,
    not the controller.
  * the swing's real gain is margin. From 480-650 mm it leaves 350-540 mm of landing against the
    flick's 204 mm, stays upright in 25 of 25 randomised robots, and on a known staircase it is far
    better: 6/6 successes at 5 mm of error on a 100 mm riser.
  * across UNKNOWN staircases it still loses to the flick, 0.23 against 0.37, because the amplitude it
    can safely reach caps the angle, and a 170 mm riser needs an angle it cannot get to from a standoff
    it can survive. Its failures are blind rather than dangerous, which is the better way to fail.

  python sim/stairs_osc.py                 # the sweep
  python sim/stairs_osc.py --trials 8      # more robots per row
"""
import argparse, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "ml"))
from botsim import LIB, CTRL_DT, quat_pitch                    # noqa: E402
from stairs_env import StairsLookEnv, EDGE, LOOK_MIN           # noqa: E402
from stairs_probe import estimate                              # noqa: E402

BALANCE = 4


def episode(rise, tread, standoff, amp, hz, cycles, v_app, seed, d0=0.85, settle=1.2, amp_end=0.0):
    """Creep in at v_app, stop at `standoff`, swing, and measure. Returns what actually happened."""
    env = StairsLookEnv()
    env.reset(seed=seed)
    env.reset(options=dict(rise=rise, tread=tread, d0=d0))
    sim = env.sim
    sim.cmd.mode = 0
    x_far = float(sim.d.qpos[0])
    # creep in: aim the stop one braking distance early, the same lead the scripted flick uses
    lead = v_app * 0.10 + v_app ** 2 / (2 * 0.9)
    sim.cmd.v = v_app
    for _ in range(int(6.0 / CTRL_DT)):
        o = sim.step()
        x_far = max(x_far, float(sim.d.qpos[0]))
        if o.state != BALANCE:
            env.close()
            return dict(fell=True, phase="creep")
        if env.d_est() <= standoff + lead:
            break
    sim.cmd.v = 0.0
    for _ in range(int(1.2 / CTRL_DT)):                        # let it come to rest before looking
        o = sim.step()
        x_far = max(x_far, float(sim.d.qpos[0]))
        if o.state != BALANCE:
            env.close()
            return dict(fell=True, phase="stopping")
    d_look = EDGE - float(sim.d.qpos[0])
    LIB.bot_look_osc(sim.ctl, amp, hz, cycles, amp_end)
    LIB.bot_look(sim.ctl, 1, amp, 0.0)
    pts, peak, fell, phase = [], 0.0, False, "ok"
    for _ in range(int((cycles / hz + settle) / CTRL_DT)):
        o = sim.step()
        x_far = max(x_far, float(sim.d.qpos[0]))
        if o.state != BALANCE:
            fell = True
            phase = "swing"
            break
        p = float(o.pitch)
        peak = max(peak, p)
        if p > LOOK_MIN:
            c = env._cloud(p - quat_pitch(sim.d.qpos[3:7]))
            if len(c):
                pts.append(c)
    cloud = np.concatenate(pts) if pts else np.zeros((0, 3))
    est = estimate(cloud, env.landing_z) or {}
    seen = 0
    if len(cloud):
        on = ((np.abs(cloud[:, 2] - (env.landing_z - rise)) < 0.02)
              & (cloud[:, 0] > EDGE) & (cloud[:, 0] < EDGE + tread))
        seen = int(on.sum())
    env.close()
    return dict(fell=fell, phase=phase, d_look=d_look, peak=math.degrees(peak), seen=seen,
                rise_est=est.get("rise"), margin=(EDGE - x_far) * 1000,
                err=None if est.get("rise") is None else abs(est["rise"] - rise) * 1000)


def row(label, trials, **kw):
    rs = [episode(seed=4200 + k, **kw) for k in range(trials)]
    up = [r for r in rs if not r["fell"]]
    errs = [r["err"] for r in up if r["err"] is not None]
    good = [r for r in up if r["err"] is not None and r["err"] <= 20 and r["margin"] >= 100]
    f = lambda k: np.mean([r[k] for r in up]) if up else float("nan")
    print(f"  {label:<34} {f('d_look'):5.3f} m {f('peak'):5.1f}deg {f('seen'):7.0f} "
          f"{(np.mean(errs) if errs else float('nan')):7.0f} mm {f('margin'):7.0f} mm "
          f"{len(up)}/{len(rs)} up  {len(good)}/{len(rs)} ok")
    return len(good) / len(rs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--rise", type=float, default=0.10)
    ap.add_argument("--tread", type=float, default=0.30)
    a = ap.parse_args()
    base = dict(rise=a.rise, tread=a.tread, v_app=0.12, trials=a.trials)
    print(f"\n\033[1mCreep in and swing: {a.rise*1000:.0f} mm riser, {a.tread*1000:.0f} mm tread, "
          f"{a.trials} randomised robots per row\033[0m")
    print(f"  {'':<34} {'look at':>7} {'peak':>8} {'on tread':>8} {'riser err':>10} "
          f"{'landing left':>12}  {'upright':>9} {'success':>9}")
    print("  -- how close can it look from (amplitude 0.5 rad, 1.5 Hz, 2 cycles) --")
    for so in (0.12, 0.15, 0.20, 0.28, 0.38, 0.50):
        row(f"stop at {so*1000:.0f} mm", amp=0.5, hz=1.5, cycles=2, standoff=so, **base)
    print("  -- amplitude, from 150 mm (above 0.8 rad the swing itself loses the robot) --")
    for amp in (0.35, 0.45, 0.55, 0.65):
        row(f"amplitude {amp:.2f} rad", amp=amp, hz=1.5, cycles=2, standoff=0.15, **base)
    print("  -- amplitude, from 280 mm --")
    for amp in (0.45, 0.55, 0.65, 0.75):
        row(f"amplitude {amp:.2f} rad", amp=amp, hz=1.5, cycles=2, standoff=0.28, **base)
    print("  -- how many swings, and how fast (0.5 rad, from 150 mm) --")
    for hz, cyc in ((1.0, 1), (1.0, 3), (1.5, 1), (1.5, 3), (2.2, 3)):
        row(f"{cyc} cycle(s) at {hz:.1f} Hz", amp=0.5, hz=hz, cycles=cyc, standoff=0.15, **base)


if __name__ == "__main__":
    main()
