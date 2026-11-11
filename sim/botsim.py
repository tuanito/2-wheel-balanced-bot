"""MuJoCo plant + the real C++ controller (libbot.so, built from core/controller.cpp).

The controller sees only what the robot sees: IMU (with bias/noise/mount tilt), wheel encoder angles and
speeds (relative to the body), sonar, battery voltage. Motors are modelled as the LK drive: torque command
-> current limited to 16.5 A, back-EMF limited by the battery voltage, one control period of CAN latency.
"""
import ctypes as C, pathlib, math, os, numpy as np, mujoco

HERE = pathlib.Path(__file__).parent
LIB = C.CDLL(str(HERE / ("libbot.dylib" if (HERE / "libbot.dylib").exists() and os.uname().sysname == "Darwin" else "libbot.so")))   # macOS: built by ml/setup_mac.sh
# IMU full scale: MPU6050 = +-2000 dps, +-16 g (env GYRO_DPS / ACC_G to try a wide-range IMU such as the ICM-20649)
GYRO_MAX = math.radians(float(os.environ.get("GYRO_DPS", "2000"))); ACC_MAX = 9.81 * float(os.environ.get("ACC_G", "16"))


class Inputs(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("dt", "gx", "gy", "gz", "ax", "ay", "az", "ql", "qr", "wl", "wr", "sonar_m", "vbat")] + [("charging", C.c_uint8)]


class Command(C.Structure):
    _fields_ = [("mode", C.c_uint8), ("v", C.c_float), ("yaw_rate", C.c_float), ("angle", C.c_float), ("seq", C.c_uint16),
                ("arm", C.c_uint8), ("auto_standup", C.c_uint8), ("obstacle_stop", C.c_uint8)]


class Outputs(C.Structure):
    _fields_ = [("tl", C.c_float), ("tr", C.c_float), ("state", C.c_uint8), ("side", C.c_int8)] + \
               [(n, C.c_float) for n in ("pitch", "pitch_rate", "roll", "yaw", "x", "y", "v", "v_ref", "yaw_ref")] + \
               [("obstacle", C.c_uint8), ("standup_tries", C.c_uint8), ("standup_energy_adj", C.c_float), ("dbg", C.c_float * 6), ("motors_on", C.c_uint8)]


LIB.bot_new.restype = C.c_void_p
LIB.bot_free.argtypes = [C.c_void_p]
LIB.bot_set_cmd.argtypes = [C.c_void_p, C.POINTER(Command)]
LIB.bot_step.argtypes = [C.c_void_p, C.POINTER(Inputs), C.POINTER(Outputs)]

STATES = ["DISARMED", "LYING", "LIFT", "COAST", "BALANCE", "FALLEN", "CHARGING", "FAULT", "FLIP_RUN", "FLIP_BRAKE", "FLIP_AIR", "DOCK_LAND", "UNDOCK"]
CTRL_DT = 0.002
KT, KE, R_LINE, I_MAX = 0.14, 1 / (53 * 2 * math.pi / 60), 0.89 + 0.10, 16.5


def quat_pitch(q):
    """Body pitch about its own wheel axle (independent of heading): angle of the body z-axis from vertical,
    positive towards the body's forward direction."""
    Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, q); Rm = Rm.reshape(3, 3)
    yb, zb = Rm[:, 1], Rm[:, 2]
    fwd = np.cross(yb, [0.0, 0.0, 1.0]); fwd /= np.linalg.norm(fwd) + 1e-12
    return math.atan2(float(zb @ fwd), float(zb[2]))


class BotSim:
    def __init__(self, xml="robot_c.xml", seed=0, randomize=False):
        self.rng = np.random.default_rng(seed)
        self.m = mujoco.MjModel.from_xml_path(str(HERE / xml))
        self.d = mujoco.MjData(self.m)
        m, rng = self.m, self.rng
        self.ch = m.body("chassis").id
        self.sid = {n: m.sensor_adr[m.sensor(n).id] for n in ("gyro", "accel", "ql", "qr", "wl", "wr")}
        self.has_sonar = any(m.sensor(i).name == "sonar" for i in range(m.nsensor))
        self.lidar_ids = [i for i in range(m.nsensor) if m.sensor(i).name.startswith("lidar")]
        # --- domain randomisation (what we don't know exactly about the real robot) ---
        self.vbat, self.tq_gain, self.delay = 22.2, 1.0, 1
        self.gyro_bias = np.zeros(3); self.acc_bias = np.zeros(3); self.mount = np.eye(3)
        if randomize:
            m.body_mass[self.ch] *= rng.uniform(0.92, 1.08)
            m.body_ipos[self.ch] += [rng.uniform(-0.004, 0.004), 0, rng.uniform(-0.006, 0.008)]
            m.body_inertia[self.ch] *= rng.uniform(0.85, 1.2)
            mu = rng.uniform(0.5, 1.2)
            for side in ("left", "right"): m.geom_friction[m.geom(f"tyre_{side}").id][0] = mu
            self.vbat = rng.uniform(21.0, 25.0)
            self.tq_gain = rng.uniform(0.9, 1.1)
            self.delay = int(rng.integers(1, 3))
            self.gyro_bias = rng.normal(0, 0.01, 3)
            self.acc_bias = rng.normal(0, 0.1, 3)
            a = math.radians(rng.uniform(-1.5, 1.5))           # IMU not perfectly flat on the tray
            self.mount = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
        self.ctl = LIB.bot_new()
        self.cmd = Command(mode=0, v=0, yaw_rate=0, angle=0, seq=0, arm=1, auto_standup=1, obstacle_stop=1)
        self.out = Outputs()
        self.queue = [(0.0, 0.0)] * 3
        self.t = 0.0
        self.log = []

    def charging(self):
        """Charge-sense input (override in the dock sim)."""
        return False

    def close(self):
        LIB.bot_free(self.ctl)

    def place(self, pitch=0.0, x=0.0, y=0.0, yaw=0.0, settle=True):
        d = self.d
        mujoco.mj_resetData(self.m, d)
        qy = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        qp = np.array([math.cos(pitch / 2), 0, math.sin(pitch / 2), 0])
        q = np.zeros(4); mujoco.mju_mulQuat(q, qy, qp)
        d.qpos[:3] = [x, y, 0.0 if abs(pitch) < 0.5 else 0.02]
        d.qpos[3:7] = q
        mujoco.mj_forward(self.m, d)
        if settle and abs(pitch) > 0.5:
            for _ in range(800): mujoco.mj_step(self.m, d)
        if settle and abs(pitch) <= 0.5:
            for _ in range(20): mujoco.mj_step(self.m, d)

    def hold(self, seconds=0.4):
        """Hold the robot still (a hand holding it) while the controller boots and its filter settles."""
        q, v = self.d.qpos.copy(), self.d.qvel.copy()
        for _ in range(int(round(seconds / CTRL_DT))):
            self.step()
            self.d.qpos[:] = q; self.d.qvel[:] = 0; mujoco.mj_forward(self.m, self.d)

    def motor(self, tau, w):
        tau *= self.tq_gain
        i = max(-I_MAX, min(I_MAX, tau / KT))
        i_hi = (self.vbat - KE * w) / R_LINE
        i_lo = (-self.vbat - KE * w) / R_LINE
        return KT * max(i_lo, min(i_hi, i))

    def sense(self):
        s, sd = self.d.sensordata, self.sid
        g = self.mount @ s[sd["gyro"]:sd["gyro"] + 3] + self.gyro_bias
        a = self.mount @ s[sd["accel"]:sd["accel"] + 3] + self.acc_bias
        g = np.clip(g, -GYRO_MAX, GYRO_MAX); a = np.clip(a, -ACC_MAX, ACC_MAX)      # IMU full-scale range
        q = (s[sd["ql"]], s[sd["qr"]])
        w = (s[sd["wl"]], s[sd["wr"]])
        enc = lambda x: round(x * 65536 / (2 * math.pi)) * 2 * math.pi / 65536   # 16-bit encoder in the CAN reply
        wq = lambda x: round(math.degrees(x)) * math.pi / 180                   # speed reply in whole dps
        sonar = -1.0
        if self.has_sonar:
            r = self.d.sensordata[self.m.sensor("sonar").adr[0]]
            sonar = r if 0.02 < r < 4.0 else -1.0
        return Inputs(dt=CTRL_DT, gx=g[0], gy=g[1], gz=g[2], ax=a[0], ay=a[1], az=a[2],
                      ql=enc(q[0]), qr=enc(q[1]), wl=wq(w[0]), wr=wq(w[1]), sonar_m=sonar, vbat=self.vbat, charging=int(self.charging()))

    def lidar(self):
        """Ranges (m) of the simulated lidar, -1 = no return, ray i at angle 2*pi*i/N in the robot frame."""
        return np.array([self.d.sensordata[self.m.sensor_adr[i]] for i in self.lidar_ids])

    def step(self, push=None):
        LIB.bot_set_cmd(self.ctl, C.byref(self.cmd))
        LIB.bot_step(self.ctl, C.byref(self.sense()), C.byref(self.out))
        self.queue.append((self.out.tl, self.out.tr))
        tl, tr = self.queue[-1 - self.delay]
        self.queue = self.queue[-4:]
        for _ in range(int(round(CTRL_DT / self.m.opt.timestep))):
            wl = self.d.qvel[self.m.jnt_dofadr[self.m.joint("left_hinge").id]]
            wr = self.d.qvel[self.m.jnt_dofadr[self.m.joint("right_hinge").id]]
            self.d.ctrl[:] = [self.motor(tl, wl), self.motor(tr, wr)]
            if push is not None: self.d.xfrc_applied[self.ch, :3] = push
            mujoco.mj_step(self.m, self.d)
        self.d.xfrc_applied[self.ch, :] = 0
        self.t += CTRL_DT
        o = self.out
        self.log.append((self.t, o.state, quat_pitch(self.d.qpos[3:7]), o.pitch, o.yaw, o.v, o.v_ref, o.tl, o.tr,
                         self.d.qpos[0], self.d.qpos[1], self.true_yaw(), o.x, o.y))
        return o

    def true_yaw(self):
        """Heading of the body's forward direction in the horizontal plane (also valid while lying)."""
        Rm = np.zeros(9); mujoco.mju_quat2Mat(Rm, self.d.qpos[3:7]); Rm = Rm.reshape(3, 3)
        fwd = np.cross(Rm[:, 1], [0.0, 0.0, 1.0])
        return math.atan2(fwd[1], fwd[0])

    def run(self, seconds, **cmd):
        for k, v in cmd.items(): setattr(self.cmd, k, v)
        for _ in range(int(round(seconds / CTRL_DT))): self.step()
        return self.out

LIB.bot_tune.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
LIB.bot_lift_mode.argtypes = [C.c_void_p, C.c_int, C.c_float, C.c_float]
LIB.bot_look.argtypes = [C.c_void_p, C.c_int, C.c_float, C.c_float]   # look-down nod: on, pitch target, seconds
LIB.bot_limits.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
LIB.bot_recover.argtypes = [C.c_void_p, C.c_float, C.c_float]
LIB.bot_dock.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
LIB.bot_standup.argtypes = [C.c_void_p, C.c_int, C.c_float]
LIB.bot_standup_obs.argtypes = [C.c_void_p, C.POINTER(C.c_float)]
