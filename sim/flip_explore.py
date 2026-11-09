"""Can the robot tumble over backwards (a 'backflip' over its own top) and land? Drive backwards, then brake hard."""
import math, numpy as np, mujoco, itertools
from botsim import BotSim, quat_pitch, CTRL_DT
def trial(v, t_brake, tq=2.3, seed=0):
    s = BotSim(seed=seed); s.place(0); s.hold(); s.run(1.0)
    LIB = __import__("botsim").LIB; LIB.bot_limits(s.ctl, 4.0, 2.0, 1.2, 3.0)
    s.run(4.0, v=-v)
    v_real = s.out.v
    m, d = s.m, s.d
    lid = m.geom("lidar").id; floor = m.geom("floor").id
    pitch = [quat_pitch(d.qpos[3:7])]; lid_hit = 0.0; maxF = 0.0
    wl, wr = m.jnt_dofadr[m.joint("left_hinge").id], m.jnt_dofadr[m.joint("right_hinge").id]
    steps = int(2.5 / m.opt.timestep)
    for k in range(steps):
        t = k * m.opt.timestep
        u = tq if t < t_brake else 0.0        # positive torque = brake backward motion -> body pitches back
        d.ctrl[:] = [s.motor(u, d.qvel[wl]), s.motor(u, d.qvel[wr])]
        mujoco.mj_step(m, d)
        pitch.append(quat_pitch(d.qpos[3:7]))
        for i in range(d.ncon):
            c = d.contact[i]
            if floor in (c.geom1, c.geom2):
                f = np.zeros(6); mujoco.mj_contactForce(m, d, i, f)
                other = c.geom2 if c.geom1 == floor else c.geom1
                if other == lid: lid_hit = max(lid_hit, f[0])
                maxF = max(maxF, f[0])
    p = np.unwrap(pitch)
    return v_real, math.degrees(p.min()), math.degrees(p[-1]), lid_hit, maxF, d.qpos[2]
for v, tb in itertools.product([1.0, 1.5, 2.0, 2.5, 3.0], [0.15, 0.3, 0.6]):
    vr, pmin, pend, lid, F, z = trial(v, tb)
    print(f"v {v:.1f} (real {vr:+.2f}) brake {tb:.2f}s -> most backward {pmin:7.1f}°, ends at {pend:7.1f}°, lidar floor force {lid:6.1f} N, max impact {F:6.1f} N")
