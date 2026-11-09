"""KENBOT-style backflip through the controller: balancing -> lie on back -> kick -> fly -> catch."""
import sys, itertools, numpy as np, ctypes as C, mujoco, math
from multiprocessing import Pool
from botsim import BotSim, STATES, CTRL_DT, LIB
LIB.bot_flip.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
def run(a):
    seed, lie, kt, at, ca = a
    s = BotSim(seed=seed, randomize=seed > 0); LIB.bot_flip(s.ctl, lie, kt, at, ca)
    s.place(0); s.hold(); seen = []
    lid, fl = s.m.geom("lidar").id, s.m.geom("floor").id; lidF = 0.0
    for k in range(int(7.0 / CTRL_DT)):
        if k == int(1.0 / CTRL_DT): s.cmd.mode, s.cmd.seq = 3, 1
        o = s.step(); st = STATES[o.state]
        if not seen or seen[-1] != st: seen.append(st)
        if "FLIP_AIR" in seen and st == "BALANCE": s.cmd.mode = 0
        for i in range(s.d.ncon):
            c = s.d.contact[i]
            if lid in (c.geom1, c.geom2) and fl in (c.geom1, c.geom2):
                f = np.zeros(6); mujoco.mj_contactForce(s.m, s.d, i, f); lidF = max(lidF, f[0])
    clean = seen == ["BALANCE", "FLIP_RUN", "FLIP_BRAKE", "FLIP_AIR", "BALANCE"]
    return clean, s.out.state == 4, lidF, seen
if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    with Pool(2) as p:
        for lie, kt, at, ca in itertools.product([0.6], [0.03, 0.06, 0.12], [0.5, 1.0, 1.6], [0.6, 0.9]):
            r = p.map(run, [(i, lie, kt, at, ca) for i in range(N)])
            print(f"kick {kt}s air {at} catch {ca}: clean {sum(x[0] for x in r)}/{N}, upright at end {sum(x[1] for x in r)}/{N}, lidar hits {sum(x[2] > 1 for x in r)} (max {max(x[2] for x in r):.0f} N)", flush=True)
        print(r[0][3])
