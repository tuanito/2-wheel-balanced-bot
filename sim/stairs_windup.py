"""What it costs to take the look from a standstill, and what a wind-up buys back.

The scripted recipe flicks while already reversing, which is efficient but sloppy: the entry speed is
whatever the approach happened to leave, so the same flick lands at a different place every time. The
obvious alternative is to stop, settle upright, and then look once - except that a one-sided flick is a
forward acceleration, and with no reverse momentum to cancel it the robot keeps the speed it gains.

This measures three entries over randomised robots, from the moment the robot is settled and still:

  plain     settled at 0 deg, then flick                      (what "stop, then look once" costs)
  wind-up   settled, reverse command for W s, then flick      (a short local run-up, still from a stop)
  pullback  the above, plus a reverse command held through the recovery

What matters is `forward` - how far past the settled standoff the robot ever gets - because that is
what eats the margin to the drop, and `peak`, because the angle is what decides whether it sees
anything at all.

  python sim/stairs_windup.py              # the full table
  python sim/stairs_windup.py --trials 8
"""
import argparse, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "ml"))
from botsim import BotSim, LIB, CTRL_DT                      # noqa: E402
from stairs_probe import model_xml                           # noqa: E402

SETTLE_MAX = 3.0            # s to wait for "settled": it always takes well under this


def settle(sim):
    """Stand still at 0 deg: the state the look is supposed to start from."""
    sim.cmd.v = 0.0
    for _ in range(int(SETTLE_MAX / CTRL_DT)):
        o = sim.step()
        if o.state != 4:
            return None
        if abs(o.pitch) < 0.01 and abs(o.pitch_rate) < 0.20 and abs(o.v) < 0.02:
            return o
    return o


def episode(tgt, wind, pullback, seed, gain=0.5):
    """One look from a standstill. Returns (peak deg, forward mm, back mm, upright)."""
    sim = BotSim(xml_str=model_xml(0.10, 0.30, 6, edge=4.0), seed=seed, randomize=True)
    sim.place(pitch=0.0, x=2.2)
    sim.hold(0.6)
    sim.cmd.mode = 0
    if settle(sim) is None:
        sim.close()
        return None
    x_stand = float(sim.d.qpos[0])                 # the standoff the look is taken from
    if wind > 0:
        sim.cmd.v = -0.35
        for _ in range(int(wind / CTRL_DT)):
            if sim.step().state != 4:
                sim.close()
                return None
    LIB.bot_look_gains(sim.ctl, 4.0, 2.0, gain)
    LIB.bot_look(sim.ctl, 1, tgt, 0.0)
    sim.cmd.v = -0.5 if pullback else 0.0
    v_entry = float(sim.out.v)                     # the speed the flick is actually entered at
    peak, x_far, x_near, bad = 0.0, x_stand, x_stand, False
    for k in range(int(3.0 / CTRL_DT)):
        o = sim.step()
        if o.state != 4:
            bad = True
            break
        peak = max(peak, float(o.pitch))
        x = float(sim.d.qpos[0])
        x_far, x_near = max(x_far, x), min(x_near, x)
        if pullback and k > int(1.6 / CTRL_DT):
            sim.cmd.v = 0.0                        # stop pulling back once the nod is over
    sim.close()
    return (math.degrees(peak), (x_far - x_stand) * 1000, (x_stand - x_near) * 1000,
            0.0 if bad else 1.0, v_entry)


def table(trials):
    print(f"\n\033[1mThe look taken from a standstill: {trials} randomised robots each\033[0m")
    print("  'forward' is how far past the settled standoff it ever gets - the margin it spends.")
    print(f"\n  {'entry':<22} {'target':>7} {'v at flick':>11} {'peak':>10} {'forward':>14} "
          f"{'back first':>12} {'upright':>8}")
    rows = []
    for name, wind, pull in (("stop, then flick", 0.0, False),
                             ("stop, wind 0.35 s", 0.35, False),
                             ("stop, wind 0.60 s", 0.60, False),
                             ("stop, wind 0.90 s", 0.90, False),
                             ("stop, wind 0.90 s + pull", 0.90, True)):
        for tgt in (0.75, 0.90, 1.00):
            out = [episode(tgt, wind, pull, 700 + k) for k in range(trials)]
            a = np.array([o for o in out if o is not None])
            if not len(a):
                print(f"  {name:<22} {tgt:7.2f}   lost every robot")
                continue
            print(f"  {name:<22} {tgt:7.2f} {a[:, 4].mean():+8.2f} m/s {a[:, 0].mean():5.1f}+-{a[:, 0].std():2.0f}d "
                  f"{a[:, 1].mean():+8.0f}+-{a[:, 1].std():3.0f} mm {a[:, 2].mean():9.0f} mm "
                  f"{a[:, 3].mean():7.0%}")
            rows.append((name, tgt, a[:, 0].mean(), a[:, 1].mean(), a[:, 2].mean(), a[:, 3].mean()))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=6)
    a = ap.parse_args()
    rows = table(a.trials)
    best = min((r for r in rows if r[2] > 30 and r[5] == 1.0), key=lambda r: r[3], default=None)
    if best:
        print(f"\n  cheapest entry that still reaches 30 deg on every robot: {best[0]} at {best[1]:.2f} rad, "
              f"{best[3]:+.0f} mm forward")


if __name__ == "__main__":
    main()
