"""KENBOT style: start lying on the back, kick hard (using the back as a lever), fly, land on the wheels?"""
import math, itertools, numpy as np, mujoco
from multiprocessing import Pool
from botsim import BotSim, quat_pitch
def trial(a):
    sign, T, after, seed = a
    s = BotSim(seed=seed, randomize=seed > 0); s.place(-1.8)
    m, d = s.m, s.d
    wl, wr = m.jnt_dofadr[m.joint("left_hinge").id], m.jnt_dofadr[m.joint("right_hinge").id]
    fl = m.geom("floor").id; ty = {m.geom("tyre_left").id, m.geom("tyre_right").id}; lid = m.geom("lidar").id
    p = [quat_pitch(d.qpos[3:7])]; zmax = 0; air = 0; lidF = 0; land = None
    for k in range(int(1.5 / m.opt.timestep)):
        t = k * m.opt.timestep
        u = sign * 2.3 if t < T else after * sign
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
        if land is None and air > 0.05 and touch and touch <= ty:
            land = (math.degrees(np.unwrap(p)[-1]), d.qvel[0], d.qvel[4])
    return sign, T, after, math.degrees(np.unwrap(p)[-1] - p[0]), zmax, air, lidF, land
if __name__ == "__main__":
    grid = list(itertools.product([1, -1], [0.05, 0.1, 0.15, 0.25, 0.4], [0.0, 1.0], [0]))
    with Pool(2) as pool:
        for r in pool.map(trial, grid):
            sign, T, after, rot, z, air, lidF, land = r
            ld = f"first wheel landing at {land[0]:.0f}° vx {land[1]:+.1f} m/s pitch-rate {land[2]:+.1f}" if land else "no wheel landing"
            print(f"kick {'+' if sign>0 else '-'} {T:.2f}s then {after:.1f}: total rotation {rot:+7.1f}°, airborne {air:.2f}s, axle up {1000*z:.0f} mm, lidar {lidF:.0f} N; {ld}")
