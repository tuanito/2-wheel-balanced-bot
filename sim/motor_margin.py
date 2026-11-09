import math, numpy as np
from multiprocessing import Pool
import motor_compare as mc
from botsim import BotSim, LIB
def run(a):
    name, seed, side = a; tmax = mc.setup(name)
    s = BotSim(seed=seed, randomize=True); mc.apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    LIB.bot_tune(s.ctl, min(1.6, tmax), 0.6, 30.0, 0.35)
    s.place(1.8 * side); s.run(4.0)
    L = np.array(s.log); st = L[:, 1].astype(int)
    up = np.where(st == 4)[0]; lift = np.where(st == 2)[0]
    wmax = max(abs(s.m.opt.timestep) * 0, 0)
    return (L[up[0], 0] - L[lift[0], 0]) if len(up) else np.nan, np.abs(L[:, 5]).max(), np.hypot(L[-1, 9] - L[0, 9], L[-1, 10] - L[0, 10])
def push(a):
    name, seed, imp = a; tmax = mc.setup(name)
    s = BotSim(seed=seed, randomize=True); mc.apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    s.place(0); s.hold(); s.run(1.0)
    for _ in range(50): s.step(push=np.array([imp / 0.1, 0, 0]))
    s.run(3.0); L = np.array(s.log)
    return bool((L[500:, 1] == 4).all())
def accel(a):
    name, seed = a; tmax = mc.setup(name)
    s = BotSim(seed=seed, randomize=True); mc.apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    LIB.bot_limits(s.ctl, 3.0, 1.5, 1.2, 3.0)
    s.place(0); s.hold(); s.run(1.0); s.run(4.0, v=2.5); L = np.array(s.log)
    ok = bool((L[500:, 1] == 4).all())
    return ok, float(np.median(L[-500:, 5]))
if __name__ == "__main__":
    with Pool(2) as p:
        for name in mc.MOTORS:
            r = p.map(run, [(name, 100 + i, sd) for i in range(10) for sd in (1, -1)])
            pu = {imp: sum(p.map(push, [(name, 300 + i, imp) for i in range(10)])) for imp in (1.5, 2.0, 2.5)}
            ac = p.map(accel, [(name, 500 + i) for i in range(6)])
            print(f"{name:26s} stand-up {np.nanmedian([x[0] for x in r]):.2f}s, peak speed {np.median([x[1] for x in r]):.1f} m/s, travel {np.median([x[2] for x in r]):.2f} m | "
                  f"pushes survived 1.5/2.0/2.5 N·s: {pu[1.5]}/10 {pu[2.0]}/10 {pu[2.5]}/10 | top-speed test (2.5 m/s cmd): {sum(a[0] for a in ac)}/6 ok, reached {np.median([a[1] for a in ac]):.2f} m/s", flush=True)
