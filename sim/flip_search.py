"""Search for a full airborne backflip (KENBOT style): run up backwards, brake, fly a full turn, land on the wheels."""
import math, itertools, numpy as np, mujoco, ctypes as C
from multiprocessing import Pool
from botsim import BotSim, LIB, quat_pitch, CTRL_DT
def trial(args):
    v, tb, tq, after, seed = args
    s = BotSim(seed=seed, randomize=seed > 0); s.place(0); s.hold(); s.run(0.5)
    LIB.bot_limits(s.ctl, v, 2.0, 1.2, 3.0)
    s.run(4.0, v=-v)
    v0 = s.out.v
    m, d = s.m, s.d
    lid, floor = m.geom("lidar").id, m.geom("floor").id
    tyres = {m.geom("tyre_left").id, m.geom("tyre_right").id}
    wl, wr = m.jnt_dofadr[m.joint("left_hinge").id], m.jnt_dofadr[m.joint("right_hinge").id]
    p = [quat_pitch(d.qpos[3:7])]; zmax = 0; lid_hit = 0; body_hit = 0; air = 0
    for k in range(int(1.5 / m.opt.timestep)):
        t = k * m.opt.timestep
        u = tq if t < tb else after
        d.ctrl[:] = [s.motor(u, d.qvel[wl]), s.motor(u, d.qvel[wr])]
        mujoco.mj_step(m, d)
        p.append(quat_pitch(d.qpos[3:7])); zmax = max(zmax, d.qpos[2])
        touching = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if floor in (c.geom1, c.geom2):
                o = c.geom2 if c.geom1 == floor else c.geom1; touching.add(o)
                f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f)
                if o == lid: lid_hit = max(lid_hit, f[0])
                elif o not in tyres: body_hit = max(body_hit, f[0])
        if not touching: air += m.opt.timestep
        pu = np.unwrap(p)[-1]
        if pu < -math.radians(330) and touching and touching <= tyres: break   # back on its wheels after a full turn
    pu = math.degrees(np.unwrap(p)[-1])
    return dict(v=v, v0=v0, tb=tb, tq=tq, after=after, rot=pu, zmax=zmax, air=air, lid=lid_hit, body=body_hit)
def sweep(grid):
    with Pool(2) as p: return p.map(trial, grid)
if __name__ == "__main__":
    grid = list(itertools.product([3.9, 4.3, 4.7, 5.1, 5.5], [0.03, 0.06, 0.1, 0.2], [2.3], [0.0, 1.0, -1.0], [0]))
    with Pool(2) as p:
        res = p.map(trial, grid)
    for r in sorted(res, key=lambda r: -abs(r["rot"])):
        print(f"run-up {r['v']:.1f} (real {r['v0']:+.2f}) brake {r['tb']:.2f}s then {r['after']:+.1f} N·m: rotation {r['rot']:7.1f}°, "
              f"airborne {r['air']:.2f}s, axle up to {1000*r['zmax']:.0f} mm, lidar hit {r['lid']:.0f} N, body hit {r['body']:.0f} N")
