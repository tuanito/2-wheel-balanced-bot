"""How much of the LQR pitch gain the look-down nod can use, measured on DELAY-2 robots.

The nod holds a pitch target by running a PD on pitch alone, at `look_gain` times the LQR pitch
gains. At full gain that is a ~130 rad/s rate loop, and 4 ms of CAN latency takes it unstable: the
pitch rings, the wheels get pumped and the robot is lost. The history is in CLAUDE.md - on the
nominal robot a full-gain nod worked, and over randomised robots it lost 7 in 10, every failure a
robot that drew two control periods of latency. `BotSim.randomize` draws delay 1 or 2 with equal
probability, so a sweep over random seeds dilutes the very robots that decide the answer. This
script pins `delay = 2` instead.

It also reports the peak wheel speed relative to the body, because that is what the runaway guard
watches (0.95 * motor::W_MAX) and the guard DOES apply during the nod - look_mode runs inside
BALANCE. On the MF6015 that threshold is 114 rad/s; on the MF7015 it is 75, so a gain that was safe
on one motor is not automatically safe on the other. That is why look_gain cannot be carried over.

    python look_gain_sweep.py                 # the sweep at the committed 0.6 rad target
    ROBOT_MOTOR=mf7015 python look_gain_sweep.py --trials 20
"""
import argparse
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from botsim import BotSim, LIB, CTRL_DT, LIB_MOTOR      # noqa: E402
import motor_profile as MP                              # noqa: E402

def episode(gain, tgt, seed, delay, dur, settle_s=1.2, run_s=2.5):
    """One nod. Returns (upright, peak pitch deg, peak |wheel_rel| rad/s, drift mm)."""
    s = BotSim(seed=seed, randomize=True)
    s.delay = delay                      # pin it: this is the whole point of the script
    s.place(0)
    s.hold()
    s.run(settle_s)
    if s.out.state != 4:
        s.close()
        return None
    x0 = float(s.d.qpos[0])
    LIB.bot_look_gains(s.ctl, 4.0, 2.0, gain)
    LIB.bot_look(s.ctl, 1, tgt, dur)
    peak, w_peak, bad = 0.0, 0.0, False
    for _ in range(int(run_s / CTRL_DT)):
        o = s.step()
        inp = s.sense()
        peak = max(peak, abs(float(o.pitch)))
        w_peak = max(w_peak, 0.5 * abs(inp.wl + inp.wr))
        if o.state != 4:
            bad = True
            break
    drift = abs(float(s.d.qpos[0]) - x0) * 1000.0
    s.close()
    return (0 if bad else 1, math.degrees(peak), w_peak, drift)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--target", type=float, default=0.6, help="rad, the nod's pitch target")
    ap.add_argument("--time", type=float, default=0.4, help="s, look_time")
    ap.add_argument("--gains", type=float, nargs="*",
                    default=[0.3, 0.4, 0.5, 0.6, 0.8, 1.0])
    a = ap.parse_args()

    guard = 0.95 * LIB.bot_motor_w_max()
    print(f"\n\033[1mLook-down nod on delay-2 robots: {a.trials} each, target "
          f"{a.target:.2f} rad ({math.degrees(a.target):.0f} deg) for {a.time:.2f} s\033[0m")
    print(f"  motor {LIB_MOTOR}: runaway guard at 0.95 x {LIB.bot_motor_w_max():.0f} = "
          f"{guard:.0f} rad/s, and it applies during the nod (look_mode runs inside BALANCE)")
    print(f"\n  {'look_gain':>9} {'upright':>9} {'peak pitch':>12} {'peak wheel':>12} "
          f"{'vs guard':>10} {'drift':>9}")
    for gain in a.gains:
        rows = [r for r in (episode(gain, a.target, 900 + i, 2, a.time) for i in range(a.trials))
                if r is not None]
        if not rows:
            print(f"  {gain:>9.2f}   no robot even settled")
            continue
        up = sum(r[0] for r in rows)
        pk = np.median([r[1] for r in rows])
        wp = np.max([r[2] for r in rows])
        dr = np.median([r[3] for r in rows])
        flag = "OVER" if wp > guard else f"{wp / guard * 100:.0f}%"
        print(f"  {gain:>9.2f} {up:>6}/{len(rows):<3} {pk:>10.1f} deg {wp:>9.0f} rad/s "
              f"{flag:>10} {dr:>7.0f} mm")
    print("\n  'peak wheel' is the MAXIMUM over the trials, not the median: one robot touching the"
          "\n  guard is a lost robot. Pick the largest gain that stays clear of it at full depth.")


if __name__ == "__main__":
    main()
