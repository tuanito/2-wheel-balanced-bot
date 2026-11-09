"""Backflip through the real controller (CMD_BACKFLIP): success = back to BALANCE after a full turn, without falling."""
import math, sys, itertools, numpy as np, ctypes as C, mujoco
from multiprocessing import Pool
from botsim import BotSim, LIB, STATES, quat_pitch
LIB.bot_flip.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
def trial(a):
    seed, *fp = a
    s = BotSim(seed=seed, randomize=seed > 0)
    if fp: LIB.bot_flip(s.ctl, *fp)
    s.place(0); s.hold(); s.run(1.0)
    s.cmd.mode, s.cmd.seq = 3, 1
    lid, floor = s.m.geom("lidar").id, s.m.geom("floor").id
    seen, lid_hit, p = [], 0.0, []
    for k in range(int(6.0 / 0.002)):
        o = s.step(); p.append(quat_pitch(s.d.qpos[3:7]))
        if not seen or seen[-1] != STATES[o.state]: seen.append(STATES[o.state])
        for i in range(s.d.ncon):
            c = s.d.contact[i]
            if lid in (c.geom1, c.geom2) and floor in (c.geom1, c.geom2):
                f = np.zeros(6); mujoco.mj_contactForce(s.m, s.d, i, f); lid_hit = max(lid_hit, f[0])
        if "FLIP_AIR" in seen and o.state == 4 and len(seen) > 0 and seen[-1] == "BALANCE": break
    s.cmd.mode = 0; s.run(3.0)
    rot = math.degrees(np.unwrap(p)[-1] - p[0])
    clean = seen[-2:] == ["FLIP_AIR", "BALANCE"] and s.out.state == 4 and "FALLEN" not in seen
    return clean, s.out.state == 4, rot, lid_hit, seen
if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 16
    with Pool(2) as pool:
        res = pool.map(trial, [(i,) for i in range(N)])
    print(f"clean backflip (full turn, caught on the wheels): {sum(r[0] for r in res)}/{N}; balancing at the end (incl. via stand-up): {sum(r[1] for r in res)}/{N}; "
          f"lidar floor hits: {sum(r[3] > 1 for r in res)}/{N}, max {max(r[3] for r in res):.0f} N")
    for r in res[:6]: print(r[0], round(r[2]), round(r[3]), r[4])

def grid_search(N=10):
    import itertools
    with Pool(2) as pool:
        for sp, bt, at, ca in itertools.product([4.5, 4.9, 5.3], [0.02, 0.04, 0.07], [0.3, 0.6, 1.0], [0.6, 0.9]):
            res = pool.map(trial, [(i + 100, sp, bt, at, ca) for i in range(N)])
            print(f"speed {sp} brake {bt} air {at} catch {ca}: clean {sum(r[0] for r in res)}/{N}, end balancing {sum(r[1] for r in res)}/{N}, lidar hits {sum(r[3] > 1 for r in res)}", flush=True)
