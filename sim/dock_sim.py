"""Docking simulation: the real C++ controller (libbot.so) + the real Pi docking code (pi/robot/dock.py) + the dock.

  python3 dock_sim.py            -> sweep of start offsets, undock test, results/dock_report.json + dock_sweep.png

The dock (case/body_case.py build_dock, docked frame) is in MuJoCo as collision boxes: face slab (with its 1 mm
pads), column, floor plate and the two wings. The robot gets the 10 mm contact plate. Not geometry, but forces:
  - the 2 sprung brass rods: funnel walls (45° cone, obround entrance stretched down), screw-head contact with a
    0.15 N/mm spring + 0.2 N preload, 4 mm travel, then solid
  - (no magnets any more: the wheels sit in the floor wave in front of the dock, its crest stops them rolling out)
  - charge sense = both rods on their screw heads
Lidar: 360 rays (LD19 ~ 450), sigma 4 mm noise, 1 mm steps, points under 6 cm masked (as ld19.py), 10 Hz.
"""
import json, math, pathlib, sys, re, os, ctypes
import numpy as np, mujoco
from multiprocessing import Pool
from botsim import BotSim, STATES, CTRL_DT, LIB, HERE
sys.path.insert(0, str(HERE.parent / "pi"))
from robot.dock import fit_dock, DockPilot, X_APPROACH, X_DOCKED, ir_bearing

OUT = HERE / "results"; OUT.mkdir(exist_ok=True)
MM = 0.001
LEAN = math.radians(8.0)
X_FACE = -46.0 * MM                  # dock face (robot frame, docked) = X_BACK - COVER_T - CONTACT_T - DOCK_GAP
PADS = [(-25.0 * MM, 111.0 * MM), (25.0 * MM, 111.0 * MM)]
MAGS = []                            # no magnets any more: the floor wave holds the wheels
PLATE_X = -45.0 * MM                 # plate face (robot frame)
HEAD_X = PLATE_X + 8.0 * MM          # screw head at the bottom of the funnel
ROD_TIP = X_FACE + 10.5 * MM         # rod tip (dock frame)
R_ENT, R_HEAD, F_DEPTH, F_DROP = 11.0 * MM, 4.95 * MM, 8.0 * MM, 8.0 * MM   # M5 head Ø9.3 at the bottom
K_ROD, PRELOAD, TRAVEL = 150.0, 0.2, 4.0 * MM     # N/m, N, m (soft spring)
K_WALL = 3000.0                                    # N/m, plastic funnel wall (penalty)
ROD_FRICTION = 0.15                                # N, brass rod sliding in its printed Ø4.3 bore
MAG_F0 = 1.8                          # N per pair at 2 mm (N42 Ø10x2 pair)
N_RAYS, LID_Z = 360, 0.177
TAPE_Y, TAPE_Z = (0.012, 0.037), (0.104, 0.245)   # body_case.DOCK_TAPE / DOCK_TAPE_Z (m, docked robot frame)
WHEEL_R_SIM = 0.0543
FLOOR_T = 1.0 * MM                   # trough top above the floor (TPU insert's thinnest part; body_case.DOCK_FLOOR)
SADDLE_REAR = (1.2, -34.0, -22.0, -14.0)   # body_case.SADDLE_REAR / _FRONT / _LEAD / _INSERT (mm)
SADDLE_FRONT = (1.8, 5.5, 19.5, 99.5)
SADDLE_LEAD = 20.0
SADDLE_TILT = 0.3
SADDLE_INSERT = (-16.0, 44.0)
MU_TPU, MU_PETG = 1.1, 0.6           # ribbed TPU insert / printed PETG frame, against the rubber tyre


def wave_profile(x):
    """Top of the wheel track (mm above the floor), x in mm from the docked axle (= body_case.dock_wave_profile)."""
    def cb(x, a, b, h0, h1):
        t = min(1.0, max(0.0, (x - a) / (b - a)))
        return h0 + (h1 - h0) * (1 - math.cos(math.pi * t)) / 2
    hr, r0, r1, r2 = SADDLE_REAR
    hf, f0, f1, f2 = SADDLE_FRONT
    t = FLOOR_T / MM
    if x > f2: return max(0.0, t * (1 - (x - f2) / SADDLE_LEAD))
    z = 0.0 if x < r0 else cb(x, r0, r1, 0, hr) if x < r1 else cb(x, r1, r2, hr, SADDLE_TILT) if x < r2 else \
        SADDLE_TILT * (f0 - x) / (f0 - r2) if x < f0 else cb(x, f0, f1, 0, hf) if x < f1 else cb(x, f1, f2, hf, 0)
    return z + t


def quat_y(a): return np.array([math.cos(a / 2), 0, math.sin(a / 2), 0])


def build_xml():
    s = (HERE / "robot_c.xml").read_text()
    # contact plate on the back cover
    plate = '<geom name="cplate" type="box" pos="-0.040 0 0.111" size="0.005 0.052 0.021" mass="0.030" friction="0.35 0.005 0.0001" rgba="0.2 0.2 0.2 1"/>'
    rays = "".join(f'<site name="ray{i}" pos="0 0 {LID_Z}" zaxis="{math.cos(2*math.pi*i/N_RAYS):.5f} {math.sin(2*math.pi*i/N_RAYS):.5f} 0" size="0.001"/>'
                   for i in range(N_RAYS))
    s = s.replace('<site name="imu"', plate + rays + '<site name="imu"', 1)
    sens = "".join(f'<rangefinder name="lidar{i}" site="ray{i}" cutoff="6"/>' for i in range(N_RAYS))
    s = s.replace("<sensor>", "<sensor>" + sens, 1)
    # dock, in the docked robot frame: rotated -LEAN about y around the axle (axle at world z = 0, floor at -R)
    q = quat_y(-LEAN)
    xf = X_FACE + 0.001                                  # its pads/skins: the plate rests 1 mm off the slab
    g = []
    def box(name, x0, x1, y0, y1, z0, z1, extra=""):
        g.append(f'<geom name="{name}" type="box" pos="{(x0+x1)/2:.5f} {(y0+y1)/2:.5f} {(z0+z1)/2:.5f}" '
                 f'size="{(x1-x0)/2:.5f} {(y1-y0)/2:.5f} {(z1-z0)/2:.5f}" ' + ('' if 'rgba' in extra else 'rgba="0.85 0.8 0.7 1" ') + f'{extra}/>')
    box("dface", xf - 0.007, xf, -0.088, 0.088, 0.072, 0.140, 'friction="0.35 0.005 0.0001"')   # PETG on PETG
    xb, zs, step = X_FACE - 0.0055, 0.072, 0.032          # column: upper part behind the slab, lower part set back
    box("dcol", xb - 0.034, xb, -0.040, 0.040, zs, 0.265)
    box("dshelf", xb - 0.034 - step, xb, -0.040, 0.040, zs, zs + 0.012)
    box("dlow", xb - 0.034 - step, xb - step, -0.040, 0.040, -0.060, zs)
    vis = 'contype="0" conaffinity="0" rgba="0.92 0.94 0.97 1"'          # reflective tape (drawing only, flush)
    for sg in (1, -1):
        y0_, y1_ = sorted((sg * TAPE_Y[0], sg * TAPE_Y[1]))
        box(f"tape{sg}", xb - 0.0002, xb + 0.0002, y0_, y1_, TAPE_Z[0], TAPE_Z[1], vis)
    g.append('<geom name="irled" type="sphere" size="0.004" pos="%.4f 0 0.252" contype="0" conaffinity="0" rgba="0.8 0.1 0.1 1"/>' % (xb + 0.001))
    wi, wo, wl, wt = 0.093, 0.120, 0.040, 0.003
    L = math.hypot(wl, wo - wi); th = math.atan2(wo - wi, wl)
    for sgn, nm in ((1, "wingL"), (-1, "wingR")):
        mx, my = X_FACE + wl / 2, sgn * (wi + wo) / 2
        nx, ny = -math.sin(th), math.cos(th)                # outward normal of the left wing
        cx, cy = mx + nx * wt / 2 * 1, my + sgn * ny * wt / 2
        g.append(f'<geom name="{nm}" type="box" pos="{cx:.5f} {cy:.5f} 0.110" size="{L/2:.5f} {wt/2:.5f} 0.028" '
                 f'euler="0 0 {math.degrees(sgn*th):.4f}" rgba="0.85 0.8 0.7 1"/>')
    dock = f'<body name="dock" pos="0 0 {FLOOR_T:.4f}" quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}">' + "".join(g) + "</body>"
    base = '<geom name="dbase" type="box" pos="-0.147 0 -0.0523" size="0.085 0.088 0.002" rgba="0.85 0.8 0.7 1"/>'
    # the wheel tracks: thin boxes whose tops follow the saddle profile (1 mm segments); TPU friction on the insert
    hf_asset = ""
    th = 0.004
    for sgn in (1, -1):
        # segment ends: 1 mm steps on the curved parts, one piece on the straight ones (facet edges catch the tyre)
        r, f = SADDLE_REAR, SADDLE_FRONT
        brk = [-62.0, r[1]] + [r[1] + i for i in range(1, int(r[3] - r[1]))] + [r[3], f[1]] + \
              [f[1] + i for i in range(1, int(f[3] - f[1]))] + [f[3], f[3] + SADDLE_LEAD]
        for xa, xb in zip(brk[:-1], brk[1:]):
            xa, xb = xa * MM, xb * MM
            za, zb = wave_profile(xa * 1000) * MM, wave_profile(xb * 1000) * MM
            if max(za, zb) > 1e-5:
                ang = math.atan2(zb - za, xb - xa)
                cx, cz = (xa + xb) / 2 + math.sin(ang) * th / 2, -WHEEL_R_SIM + (za + zb) / 2 - math.cos(ang) * th / 2
                mu = MU_TPU if SADDLE_INSERT[0] * MM <= (xa + xb) / 2 <= SADDLE_INSERT[1] * MM else MU_PETG
                hl = math.hypot(xb - xa, zb - za) / 2 + 0.00005
                base += (f'<geom type="box" pos="{cx:.5f} {sgn * 0.0815:.4f} {cz:.5f}" size="{hl:.5f} 0.0265 {th / 2}" '
                         f'euler="0 {math.degrees(-ang):.4f} 0" friction="{mu} 0.005 0.0001" rgba="0.8 0.75 0.65 1"/>')
    s = s.replace("<worldbody>", "<worldbody>" + dock + base, 1)
    s = s.replace("<asset>", "<asset>" + hf_asset, 1)
    # MuJoCo friction is soft (slow creep under a steady side load): a few no-slip iterations stop that, as real friction would
    s = s.replace('integrator="implicitfast"', 'integrator="implicitfast" noslip_iterations="4"', 1)
    p = HERE / "robot_c_dock.xml"; p.write_text(s)
    return p.name


class DockSim(BotSim):
    def __init__(self, seed=0, randomize=True, xml=None):
        super().__init__(xml=xml or "robot_c_dock.xml", seed=seed, randomize=randomize)
        self.dock_R = np.zeros(9); mujoco.mju_quat2Mat(self.dock_R, quat_y(-LEAN)); self.dock_R = self.dock_R.reshape(3, 3)
        self.charge = False; self.contact = False; self.sw_t = None; self.sw_on_t = None
        self.rod_c = [0.0, 0.0]; self.mag_f = 0.0; self.rod_prev = [0.0, 0.0]
        self.lid_rng = np.random.default_rng(seed + 1000)
        self.base_int = self.lid_rng.uniform(60, 160, N_RAYS)          # walls / furniture: plain surfaces
        # friction is the larger of the two geoms' values in MuJoCo: make the tyre low so the surface decides
        self.floor_mu = float(os.environ.get("FLOOR_MU", "0")) or (self.rng.uniform(0.4, 1.0) if randomize else 0.9)
        for side in ("left", "right"): self.m.geom_friction[self.m.geom(f"tyre_{side}").id][0] = 0.3
        self.m.geom_friction[self.m.geom("floor").id][0] = self.floor_mu
        LIB.bot_dock_hold.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        hold = float(os.environ.get("DOCK_HOLD", "-1"))          # N·m per motor; -1 = firmware default
        if hold >= 0: LIB.bot_dock_hold(self.ctl, 6.0, 0.3, hold)

    def charging(self): return self.charge

    def _forces(self):
        """Rods, funnels, magnets -> one wrench on the chassis (world frame) + the charge sense."""
        d, ch = self.d, self.ch
        Rb = d.xmat[ch].reshape(3, 3); pb = d.xpos[ch]
        Rd = self.dock_R
        F = np.zeros(3); T = np.zeros(3)
        def apply(f, p):
            nonlocal F, T
            F += f; T += np.cross(p - pb, f)
        heads = 0
        for i, (y, z) in enumerate(PADS):
            tip_w = Rd @ np.array([ROD_TIP, y, z]) + np.array([0, 0, FLOOR_T])   # rod tip, world (dock body FLOOR_T up)
            tip = Rb.T @ (tip_w - pb)                         # ... in the robot frame
            p = tip[0] - PLATE_X                              # depth into the plate (>0: inside)
            lat = np.array([tip[1] - y, tip[2] - z]); dlat = np.linalg.norm(lat)
            self.rod_c[i] = 0.0
            if p <= 0 or p > 0.02: continue                   # not at the plate (e.g. the robot faces the dock)
            if abs(tip[1]) > 0.052 or not (0.090 < tip[2] < 0.132): continue    # not in front of the plate at all
            u = lat / (dlat + 1e-9)
            # funnel radius at this depth (entrance stretched downwards)
            frac = min(p / F_DEPTH, 1.0)
            r_up = R_ENT + (R_HEAD - R_ENT) * frac
            r_dn = r_up + F_DROP * (1 - frac)
            r = r_dn if lat[1] < 0 else r_up
            if p > F_DEPTH + 2 * MM or dlat > R_ENT + F_DROP + 2 * MM:
                # outside the funnel mouth: the rod tip presses on the plate face (spring rod)
                if dlat > R_ENT + F_DROP:
                    c = min(p, TRAVEL); f = K_ROD * c + PRELOAD + (K_WALL * (p - TRAVEL) if p > TRAVEL else 0)
                    apply(Rb @ np.array([f, 0, 0]), Rb @ np.array([PLATE_X, tip[1], tip[2]]) + pb)
                continue
            if dlat > r:                                      # on the 45° funnel wall: centres the plate
                # the rod is sprung: the 45° wall pushes it back into the dock rather than through the plastic,
                # so the wall force is capped by the rod spring (fully compressed: 1.6 N axial -> 2.3 N normal)
                fn = min(K_WALL * (dlat - r), 1.414 * (K_ROD * TRAVEL + PRELOAD))
                f = fn * 0.7071 * np.array([1.0, u[0], u[1]])
                apply(Rb @ f, tip_w)
            if tip[0] > HEAD_X and dlat < R_HEAD:            # on the screw head: spring rod compressed
                c = tip[0] - HEAD_X
                f = K_ROD * min(c, TRAVEL) + PRELOAD + (K_WALL * 5 * (c - TRAVEL) if c > TRAVEL else 0)
                dc = (c - self.rod_prev[i]) / self.m.opt.timestep          # rod sliding in its printed bore: ~0.15 N friction
                f += ROD_FRICTION * math.tanh(dc / 0.002)
                self.rod_prev[i] = c
                apply(Rb @ np.array([f, 0, 0]), tip_w)
                self.rod_c[i] = c
                if dlat < 4.6 * MM: heads += 1                      # rod tip on the Ø9.3 head
        self.mag_f = 0.0
        for (y, z) in MAGS:
            m_w = Rb @ np.array([PLATE_X + 1 * MM, y, z]) + pb       # robot magnet, 1 mm under the plate face
            m_d = Rd.T @ (m_w - np.array([0, 0, FLOOR_T]))            # ... in the dock frame
            gap = (m_d[0] - (X_FACE + 0.001 - 1 * MM))                # to the dock magnet 1 mm under its face
            lat = math.hypot(m_d[1] - y, m_d[2] - z)
            if gap <= 0 or gap > 0.03 or lat > 10 * MM: continue
            f = MAG_F0 * (2 * MM / max(gap, 2 * MM)) ** 2 * (1 - lat / (10 * MM))
            self.mag_f += f
            apply(Rd @ np.array([-f, 0, 0]), m_w)
        self.contact = heads == 2
        return F, T

    def step(self, push=None):
        LIB.bot_set_cmd(self.ctl, mujoco_byref(self.cmd))
        LIB.bot_step(self.ctl, mujoco_byref(self.sense()), mujoco_byref(self.out))
        self.queue.append((self.out.tl, self.out.tr))
        tl, tr = self.queue[-1 - self.delay]
        self.queue = self.queue[-4:]
        for _ in range(int(round(CTRL_DT / self.m.opt.timestep))):
            wl = self.d.qvel[self.m.jnt_dofadr[self.m.joint("left_hinge").id]]
            wr = self.d.qvel[self.m.jnt_dofadr[self.m.joint("right_hinge").id]]
            self.d.ctrl[:] = [self.motor(tl, wl), self.motor(tr, wr)]
            F, T = self._forces()
            self.d.xfrc_applied[self.ch, :3] = F; self.d.xfrc_applied[self.ch, 3:] = T
            mujoco.mj_step(self.m, self.d)
        self.d.xfrc_applied[self.ch, :] = 0
        self.t += CTRL_DT
        self._rod_switch()
        o = self.out
        self.log.append((self.t, o.state, self._pitch(), o.pitch, o.yaw, o.v, o.v_ref, o.tl, o.tr,
                         self.d.qpos[0], self.d.qpos[1], self.true_yaw(), int(self.charge), self.mag_f))
        return o

    def _rod_switch(self):
        """The dock's rod switch (firmware/dock_beacon/dock_logic.h): rods live 0.5 s after the robot's detect divider
        appears on them, off for 30 ms every 2 s to re-check, dead as soon as the contacts open."""
        if not self.contact: self.sw_t = self.sw_on_t = None; self.charge = False; return
        if self.sw_t is None: self.sw_t = self.t
        if self.sw_on_t is None:
            if self.t - self.sw_t >= 0.5: self.sw_on_t = self.t
            self.charge = False; return
        self.charge = ((self.t - self.sw_on_t) % 2.03) < 2.0

    def _pitch(self):
        from botsim import quat_pitch
        return quat_pitch(self.d.qpos[3:7])

    def scan(self, with_intensity=False):
        r0 = self.lidar().copy()
        r = np.where(r0 > 0, r0 + self.lid_rng.normal(0, 0.004, r0.shape), 0.0)
        r = np.round(r, 3)
        r[r < 0.06] = 0.0                                    # ld19.py self mask
        a = np.arange(N_RAYS) * 2 * math.pi / N_RAYS
        if not with_intensity: return a, r
        # intensity: the retro-reflective strips on the column face read ~250, everything else 60-160
        if not hasattr(self, "ray_sid"):
            self.ray_sid = np.array([self.m.site(f"ray{i}").id for i in range(N_RAYS)])
        pos = self.d.site_xpos[self.ray_sid]; zax = self.d.site_xmat[self.ray_sid][:, [2, 5, 8]]
        hit = pos + zax * r0[:, None]
        pd = (hit - np.array([0, 0, FLOOR_T])) @ self.dock_R                  # world -> dock frame (row vectors)
        on_face = np.abs(pd[:, 0] - (X_FACE - 0.0055)) < 0.003
        on_tape = on_face & (np.abs(pd[:, 1]) > TAPE_Y[0]) & (np.abs(pd[:, 1]) < TAPE_Y[1]) & \
                  (pd[:, 2] > TAPE_Z[0]) & (pd[:, 2] < TAPE_Z[1])
        it = np.where(on_tape, self.lid_rng.normal(245, 6, N_RAYS), self.base_int + self.lid_rng.normal(0, 12, N_RAYS))
        it = np.clip(np.where(r > 0, it, 0), 0, 255)
        return a, r, it


# IR beacon model (dock_beacon.ino + 4 x TSOP38238 in hoods). Receiver (x, y, axis) robot frame (body_case.IR_RX);
# beacon LEDs at DOCK_IR_Z on the column; 'C' = centre tube, ±6° about the dock axis.
IR_RX = [(0.015, 0.054, 45), (0.015, -0.054, -45), (-0.015, 0.054, 135), (-0.015, -0.054, -135)]   # body_case.IR_RX
IR_Z = LID_Z - 0.030                  # hood tops on the top skin
IR_FOV = math.radians(55)             # hood half-angle (TSOP38238: -50 % at ±45°, hood cuts the rest)
IR_RANGE = (5.0, 6.0)                 # m, all-round / centre beam (TSAL6200 at 100 mA pulses, typical rooms)
IR_C_HALF = math.radians(6)
IR_OMNI = (X_FACE - 0.0055 - 0.017, 0.0, 0.252)   # column centre, DOCK_IR_Z (dock frame)
IR_CTR = (X_FACE + 0.016, -0.020, 0.252)          # centre tube mouth


def _ir_bits(sim):
    if not hasattr(sim, "_ir_w"):
        off = np.array([0, 0, FLOOR_T])
        sim._ir_w = (sim.dock_R @ np.array(IR_OMNI) + off, sim.dock_R @ np.array(IR_CTR) + off, sim.dock_R @ np.array([1.0, 0, 0]))
        sim._ir_rng = np.random.default_rng(sim.seed_ + 77 if hasattr(sim, "seed_") else 77)
    omni, ctr, ax = sim._ir_w
    R = sim.d.xmat[sim.ch].reshape(3, 3); P = sim.d.xpos[sim.ch]
    yaw = math.atan2(R[1, 0], R[0, 0])
    bits = 0; gid = np.zeros(1, np.int32)
    for i, (x, y, a) in enumerate(IR_RX):
        p = P + R @ np.array([x, y, IR_Z])
        for k, (src, rng_max) in enumerate(((omni, IR_RANGE[0]), (ctr, IR_RANGE[1]))):
            d = src - p; dist = float(np.linalg.norm(d))
            if dist > rng_max * sim._ir_rng.uniform(0.8, 1.0): continue
            rel = (math.atan2(d[1], d[0]) - yaw - math.radians(a) + math.pi) % (2 * math.pi) - math.pi
            if abs(rel) > IR_FOV: continue
            if k == 1:                                             # centre beam: only on the axis
                c = float(-d @ ax) / dist
                if c < math.cos(IR_C_HALF): continue
            hit = mujoco.mj_ray(sim.m, sim.d, p, d / dist, None, 1, sim.ch, gid)
            if 0 <= hit < dist - 0.045: continue                   # blocked (the column's own skin is allowed)
            if sim._ir_rng.random() < 0.05: continue               # a missed 100 ms window
            bits |= 1 << (2 * i + k)
    return bits


def home_trial(args):
    """IR homing from metres away: start (seed, x, y, heading) in the docked frame, open floor. Docks with the same
    pilot as dock_trial (find -> home -> lineup -> reverse)."""
    seed, x0, y0, h0 = args
    s = DockSim(seed=seed, randomize=True); s.seed_ = seed
    s.place(x=x0, y=y0, yaw=h0); s.hold(0.5); s.run(2.0)
    pilot = DockPilot(); n = 0; t_start = s.t; phases = []
    res = dict(seed=seed, x0=round(x0, 2), y0=round(y0, 2), h0=round(math.degrees(h0), 1), ok=False, fell=False)
    track = []
    while s.t < t_start + 150.0:
        if n % 50 == 0:
            scan = s.scan(with_intensity=True); o = s.out
            mode, v, w, lean = pilot.step(s.t, scan, (o.x, o.y, o.yaw), STATES[o.state], s.charge, o.pitch, _ir_bits(s))
            s.cmd.mode = MODE[mode]; s.cmd.v = v; s.cmd.yaw_rate = w
            if lean: s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF
            if not phases or phases[-1] != pilot.phase: phases.append(pilot.phase)
            track.append((float(s.d.qpos[0]), float(s.d.qpos[1])))
            if mode in ("done", "failed"): break
        o = s.step(); n += 1
        if STATES[o.state] in ("FALLEN", "LYING", "FAULT"): res["fell"] = True; break
    res.update(ok=pilot.phase == "done", time=round(s.t - t_start, 1), tries=pilot.tries, msg=pilot.msg,
               phases="-".join(phases), track=track[::5])
    s.close()
    return res


def main_home():
    build_xml()
    rng = np.random.default_rng(11)
    grid = []
    while len(grid) < 24:                                     # 1-4 m from the dock, all round (behind it too)
        d, ph = rng.uniform(1.0, 4.0), rng.uniform(-math.pi, math.pi)
        grid.append((100 + len(grid), d * math.cos(ph) - X_DOCKED, d * math.sin(ph), rng.uniform(-math.pi, math.pi)))
    with Pool() as p: R = p.map(home_trial, grid)
    ok = [r for r in R if r["ok"]]
    print(f"homed + docked {len(ok)}/{len(R)}")
    for r in R:
        print(f"  start {r['x0']:+.2f} {r['y0']:+.2f} m h {r['h0']:+6.1f}° ok={r['ok']!s:5} {r['time']} s tries={r['tries']} "
              f"{r['phases']} {r['msg']}")
    (OUT / "home_report.json").write_text(json.dumps(dict(n=len(R), docked=len(ok), trials=[{k: v for k, v in r.items() if k != "track"} for r in R]),
                                                     indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o)))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.add_patch(plt.Rectangle((-0.04, -X_DOCKED - 0.04), 0.08, 0.04, color="0.4"))
        for r in R:
            tr = np.array(r["track"]); c = "tab:green" if r["ok"] else "tab:red"
            if len(tr): ax.plot(tr[:, 1], tr[:, 0], c=c, lw=0.8); ax.scatter(tr[0, 1], tr[0, 0], c=c, s=12)
        ax.plot([0, 0], [-X_DOCKED, 6], ls=":", c="0.5", lw=0.8)
        ax.set_aspect("equal"); ax.set_xlabel("y (m)"); ax.set_ylabel("x (m), dock face at the grey block")
        ax.set_title(f"IR homing from 1-4 m: {len(ok)}/{len(R)} docked")
        fig.tight_layout(); fig.savefig(OUT / "home_sweep.png", dpi=110)
    except Exception as e: print("plot:", e)


def mujoco_byref(x):
    import ctypes; return ctypes.byref(x)


MODE = {"velocity": 0, "dock": 4, "done": 0, "failed": 0}


def dock_trial(args):
    """args = (seed, x, y, heading): start pose of the axle in the docked frame (dock face at x = -0.074 from the docked
    axle, facing +x), anywhere within ~50 cm of the dock, any heading."""
    seed, x0, y0, h0 = args
    s = DockSim(seed=seed, randomize=True)
    s.place(x=x0, y=y0, yaw=h0)
    s.hold(0.5); s.run(2.0)                                  # stood still, settled
    pilot = DockPilot()
    res = dict(seed=seed, x0=round(x0, 3), y0=round(y0, 3), h0=round(math.degrees(h0), 1), floor_mu=round(s.floor_mu, 2),
               ok=False, tries=1, t=None, max_rod_c=0.0, land_rate=0.0, fell=False)
    n_ctrl = 0; t_end = 70.0; t_start = s.t
    while s.t < t_end:
        if n_ctrl % 50 == 0:                                  # 10 Hz scan + pilot
            scan = s.scan(with_intensity=True)
            o = s.out
            mode, v, w, lean = pilot.step(s.t, scan, (o.x, o.y, o.yaw), STATES[o.state], s.charge, o.pitch)
            s.cmd.mode = MODE[mode]; s.cmd.v = v; s.cmd.yaw_rate = w
            if lean: s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF
            if mode in ("done", "failed"): break
        o = s.step(); n_ctrl += 1
        st = STATES[o.state]
        if st == "DOCK_LAND": res["land_rate"] = max(res["land_rate"], abs(o.pitch_rate))
        if st in ("FALLEN", "LYING", "FAULT"): res["fell"] = True; break
        res["max_rod_c"] = max(res["max_rod_c"], *s.rod_c)
    res["time"] = round(s.t - t_start, 1)
    res["tries"] = pilot.tries; res["msg"] = pilot.msg
    if pilot.phase == "done":
        # hold docked 2 s: must stay charging with motors off
        t0 = s.t; stay = True
        s.run(0.5)                                            # the plate settles (contacts may bounce for a moment)
        while s.t < t0 + 2.5:
            o = s.step()
            if not s.contact or STATES[o.state] != "CHARGING": stay = False
        res["ok"] = stay; res["t"] = round(t0, 2)
        res["pitch_docked"] = round(math.degrees(s._pitch()), 2)
        res["mag_f"] = round(s.mag_f, 2); res["rod_c_mm"] = [round(c * 1000, 2) for c in s.rod_c]
        if stay:                                              # undock and drive away
            s.cmd.mode = 5; s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF; s.cmd.v = 0; s.cmd.yaw_rate = 0
            s.run(1.0)
            s.cmd.mode = 0; s.cmd.v = 0.15
            s.run(2.5)
            s.cmd.v = 0; s.run(1.0)
            res["undock_ok"] = bool(STATES[s.out.state] == "BALANCE" and s.d.qpos[0] > 0.2 and not s.charge)
            res["undock_x"] = round(float(s.d.qpos[0]), 3)
    res["log"] = np.array(s.log)[::10].tolist() if (seed < 0 or os.environ.get("KEEPLOG")) else None
    s.close()
    return res


def main():
    build_xml()
    rng = np.random.default_rng(7)
    grid = []
    while len(grid) < 40:                                     # anywhere within 15-50 cm of the dock face, any heading
        d, ph = rng.uniform(0.15, 0.50), math.radians(rng.uniform(-65, 65))
        x, y = d * math.cos(ph) - X_DOCKED, d * math.sin(ph)
        if x < 0.10: continue
        grid.append((len(grid), x, y, rng.uniform(-math.pi, math.pi)))
    with Pool() as p: R = p.map(dock_trial, grid)
    ok = [r for r in R if r["ok"]]
    und = [r for r in ok if r.get("undock_ok")]
    print(f"docked {len(ok)}/{len(R)}   undocked {len(und)}/{len(ok)}")
    for r in R:
        print(f"  start x {r['x0']*1000:+4.0f} y {r['y0']*1000:+4.0f} mm, heading {r['h0']:+6.1f}°  ok={r['ok']!s:5} tries={r['tries']} "
              f"time={r['time']} s pitch={r.get('pitch_docked')} rods={r.get('rod_c_mm')} undock={r.get('undock_ok')} "
              f"fell={r['fell']} {r['msg']}")
    rep = dict(n=len(R), docked=len(ok), undocked=len(und), trials=[{k: v for k, v in r.items() if k != "log"} for r in R])
    (OUT / "dock_report.json").write_text(json.dumps(rep, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o)))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.add_patch(plt.Rectangle((-40, -X_DOCKED * 1000 - 40), 80, 40, color="0.7"))
        for r in R:
            c = "tab:green" if r["ok"] else "tab:red"; h = math.radians(r["h0"])
            ax.scatter(r["y0"] * 1000, r["x0"] * 1000, c=c, marker="o" if r["tries"] == 1 else "s")
            ax.plot([r["y0"] * 1000, r["y0"] * 1000 + 40 * math.sin(h)], [r["x0"] * 1000, r["x0"] * 1000 + 40 * math.cos(h)], c=c, lw=1)
        ax.set_aspect("equal"); ax.set_xlabel("start y (mm)"); ax.set_ylabel("start x from the docked axle (mm)")
        ax.set_title(f"Docking: {len(ok)}/{len(R)} docked (square = needed a retry)")
        fig.tight_layout(); fig.savefig(OUT / "dock_sweep.png", dpi=110)
    except Exception as e: print("plot:", e)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "home":
        main_home()
    elif len(sys.argv) > 1 and sys.argv[1] == "homeone":
        build_xml()
        r = home_trial((int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), math.radians(float(sys.argv[5]))))
        print({k: v for k, v in r.items() if k != "track"})
    elif len(sys.argv) > 1 and sys.argv[1] == "one":
        build_xml()
        r = dock_trial((int(sys.argv[2]) if len(sys.argv) > 2 else 0, float(sys.argv[3]) if len(sys.argv) > 3 else 0.0,
                        math.radians(float(sys.argv[4])) if len(sys.argv) > 4 else 0.0, X_APPROACH))
        print({k: v for k, v in r.items() if k != "log"})
    else:
        main()
