"""Virtual LK motors on the MuJoCo robot, shared by the QEMU test (hil/run_hil.py) and the real-CAN bench
(bench/bench_hil.py). LK CAN protocol V2: 0x88 on / 0x80 off (reply = echo of the command), 0x9C read state 2 and
0x280 multi-motor torque (replies [cmd, temp, iq, speed dps, encoder] from 0x141/0x142).

WHICH motor it plays comes from `sim/motor_profile.py`, i.e. from `cad_src/case/motors.py`, selected by
`ROBOT_MOTOR` exactly as the CAD and the plant are. It used to hard-code the MF6015's drive constants, which
made this a THIRD copy of Kt and the current limit, and a HIL run against the mf7015 firmware would have been
measured on a drive that answers like the small motor.

    ROBOT_MOTOR=mf7015 ENC_BITS=16 python3 run_hil.py drive      # the big motor, 16-bit encoder reply
"""
import math, os, struct, sys, pathlib
import numpy as np, mujoco

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from botsim import BotSim, CTRL_DT, KT, I_MAX, quat_pitch      # noqa: E402,F401  (plant, motor model, IMU model)
import motor_profile as MP                                     # noqa: E402  (the same record the CAD holds)

# The sensor's resolution and the width of the field it is reported through are two different numbers, and
# the profile carries both: both LK motors here are 18-bit sensors whose CAN reply field is 16 bits, so the
# field wraps four times per turn. That is the default BECAUSE it is what the datasheets say the motors do;
# ENC_BITS=16 asks for a drive that pre-scales its reply to the field instead, which is what the bench has to
# settle. See lk_motor.cpp's enc_mod_ vs enc_per_turn_.
ENC_BITS = int(os.environ.get("ENC_BITS", MP.M["encoder_bits"]))
ENC_RANGE = 1 << ENC_BITS                                       # counts per TURN of the sensor
ENC_FIELD = 1 << MP.M["reply_bits"]                             # how wide the reply's encoder field is
ID_L, ID_R, ID_MULTI = 0x141, 0x142, 0x280
DIR = {ID_L: +1, ID_R: -1}                                      # must match config.h DIR_LEFT/DIR_RIGHT
# IQ_MAX is the PROTOCOL's full-scale torque command (LK V2: +-2048), the same on every LK drive. IQ_FULL is
# the current that full scale means, a property of the DRIVE - config.h's IQ_FULL_A, which the firmware
# announces at boot and run_hil.py checks against this.
IQ_FULL, IQ_MAX = MP.I_MAX, 2048


def crc(p):
    c = 0
    for b in p: c = (((c << 1) | (c >> 7)) & 0xFF) ^ b
    return c


def frame(t, payload):
    body = bytes([t, len(payload)]) + payload
    return b"\xA5\x5A" + body + bytes([crc(body)])




class Plant(BotSim):
    """MuJoCo robot driven by external motor commands (the firmware), with the LK drive model of botsim."""
    def __init__(self, seed, randomize=True):
        super().__init__(seed=seed, randomize=randomize)
        self.on = {ID_L: False, ID_R: False}
        self.iq_cmd = {ID_L: 0, ID_R: 0}
        self.t_fw = None; self.rem = 0.0
        self.push = None; self.push_until = 0.0
        self.hold = None
        self.trace = []

    def joint_w(self, side):
        return self.d.qvel[self.m.jnt_dofadr[self.m.joint(f"{side}_hinge").id]]

    def advance(self, dt):
        n = int((dt + self.rem) / self.m.opt.timestep + 1e-9)
        self.rem = dt + self.rem - n * self.m.opt.timestep
        for _ in range(n):
            tq = []
            for mid, side in ((ID_L, "left"), (ID_R, "right")):
                tau = DIR[mid] * self.iq_cmd[mid] * IQ_FULL / IQ_MAX * KT if self.on[mid] else 0.0
                tq.append(self.motor(tau, self.joint_w(side)))
            self.d.ctrl[:] = tq
            if self.push is not None and self.t < self.push_until: self.d.xfrc_applied[self.ch, :3] = self.push
            else: self.d.xfrc_applied[self.ch, :3] = 0
            mujoco.mj_step(self.m, self.d)
            if self.hold is not None:
                self.d.qpos[:] = self.hold; self.d.qvel[:] = 0; mujoco.mj_forward(self.m, self.d)
            self.t += self.m.opt.timestep
        self.trace.append((self.t, quat_pitch(self.d.qpos[3:7]), float(self.d.qpos[0]), float(self.d.qpos[1]),
                           self.on[ID_L], self.iq_cmd[ID_L], self.iq_cmd[ID_R]))

    def imu(self):
        s = self.sense()
        return struct.pack("<6f", s.ax, s.ay, s.az, s.gx, s.gy, s.gz)

    def reply(self, mid, cmd):
        """LK V2 reply: [cmd, temp, iq, speed dps, encoder] (0xA1 / 0x9C)."""
        s, sd = self.d.sensordata, self.sid
        q = s[sd["ql"]] if mid == ID_L else s[sd["qr"]]           # wheel angle relative to the body, robot-forward +
        w = s[sd["wl"]] if mid == ID_L else s[sd["wr"]]
        raw = DIR[mid] * q
        enc = int(round((raw / (2 * math.pi)) * ENC_RANGE)) % ENC_RANGE
        dps = int(round(math.degrees(DIR[mid] * w)))
        iq = int(self.iq_cmd[mid] if self.on[mid] else 0)
        # The field is 16 bits whatever the sensor is. A 14/15/16-bit encoder's whole turn fits in it; an
        # 18-bit one does not, and the motor sends the low 16 bits - so the field wraps FOUR TIMES per turn.
        # That is the case lk_motor.cpp's enc_mod_ / enc_per_turn_ split exists for, and ENC_BITS=18 here is
        # how it gets exercised end to end. Masking is a no-op for ENC_BITS <= 16.
        return struct.pack("<BbhhH", cmd, 30, iq, max(-32768, min(32767, dps)), enc % ENC_FIELD)


class VirtualLK:
    """Answers LK frames for motors 1 (0x141) and 2 (0x142) from the plant's state. handle() returns the reply frames."""
    def __init__(self, plant):
        self.p = plant
        self.mute_until = -1.0          # plant time until which the motors are silent (CAN unplugged)
        self.rx = 0; self.tx = 0

    def handle(self, cid, d):
        P = self.p; d = bytes(d)
        self.rx += 1
        if P.t < self.mute_until or len(d) < 8: return []
        cmd = d[0]; out = []
        if cid == ID_MULTI:
            a, b = struct.unpack("<hh", d[:4])
            P.iq_cmd[ID_L], P.iq_cmd[ID_R] = max(-IQ_MAX, min(IQ_MAX, a)), max(-IQ_MAX, min(IQ_MAX, b))
            out = [(mid, P.reply(mid, 0xA1)) for mid in (ID_L, ID_R)]
        elif cid in (ID_L, ID_R):
            if cmd == 0x88: P.on[cid] = True
            if cmd == 0x80: P.on[cid] = False; P.iq_cmd[cid] = 0
            if cmd in (0x80, 0x88): out = [(cid, d)]                     # LK manual: reply = the command itself
            elif cmd == 0x9C: out = [(cid, P.reply(cid, 0x9C))]
            elif cmd == 0xA1:                                              # single-motor torque (probe tool)
                P.iq_cmd[cid] = max(-IQ_MAX, min(IQ_MAX, struct.unpack("<h", d[4:6])[0])); P.on[cid] = True
                out = [(cid, P.reply(cid, 0xA1))]
        self.tx += len(out)
        return out
