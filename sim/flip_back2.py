"""From lying on the back: very hard stand-up kick, then brake hard as it swings up -> launch into a flip?"""
import math, itertools, numpy as np, mujoco
from multiprocessing import Pool
from botsim import BotSim, quat_pitch
def trial(a):
    T1, thb, T2, after, seed = a
    s = BotSim(seed=seed, randomize=seed > 0); s.place(-1.8)
    m, d = s.m, s.d
    wl, wr = m.jnt_dofadr[m.joint("left_hinge").id], m.jnt_dofadr[m.joint("right_hinge").id]
    fl = m.geom("floor").id; ty = {m.geom("tyre_left").id, m.geom("tyre_right").id}; lid = m.geom("lidar").id
    p = [quat_pitch(d.qpos[3:7])]; zmax = 0; lidF = 0; tb = None; air = 0; landed = None
    for k in range(int(1.6 / m.opt.timestep)):
        t = k * m.opt.timestep
        pu = np.unwrap(p)[-1] if len(p) < 3 else pu + math.remainder(p[-1] - p[-2], 2 * math.pi)
        if t < T1: u = -2.3                                   # stand-up kick (lying on the back: push the wheels backwards)
        elif tb is None and pu > math.radians(thb): tb = t; u = 2.3
        elif tb is not None and t < tb + T2: u = 2.3          # hard brake: the body's momentum launches the robot
        else: u = after
        d.ctrl[:] = [s.motor(u, d.qvel[wl]), s.motor(u, d.qvel[wr])]
        mujoco.mj_step(m, d)
        p.append(quat_pitch(d.qpos[3:7])); zmax = max(zmax, d.qpos[2])
        touch = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if fl in (c.geom1, c.geom2):
                o = c.geom2 if c.geom1 == fl else c.geom1; touch.add(o)
                if o == lid:
                    f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f); lidF = max(lidF, f[0])
        if not touch: air += m.opt.timestep
        elif air > 0.15 and landed is None and touch <= ty: landed = math.degrees(pu)
    return T1, thb, T2, after, math.degrees(pu), zmax, air, lidF, landed
if __name__ == "__main__":
    grid = list(itertools.product([0.08, 0.15], [-40, -10, 20, 60], [0.03, 0.08], [0.0, 2.3, -2.3], [0]))
    with Pool(2) as pool:
        for r in pool.map(trial, grid):
            T1, thb, T2, after, rot, z, air, lidF, landed = r
            print(f"kick {T1:.2f}s brake at {thb:+4d}° for {T2:.2f}s then {after:+.1f}: ends at {rot:+7.1f}°, airborne {air:.2f}s, axle up {1000*z:4.0f} mm, lidar {lidF:4.0f} N, wheels landed at {landed}")
