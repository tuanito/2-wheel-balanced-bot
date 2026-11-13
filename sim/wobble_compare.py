"""Wobble at standstill + push recovery for cheaper motors (MF5015-V2 10T, Damiao DM-H6215) vs the MF6015.
Same robot, same firmware controller (libbot.so). Motor = datasheet Kt/back-EMF/resistance/torque cap/mass/inertia.
H6215 has a Hall encoder: 14 pole pairs x 6 Hall states = 84 counts/rev (4.3 deg). Two speed models:
  'hall-ok'  : drive reports a well-filtered speed (1 dps steps, 5 ms lag)  -> optimistic
  'hall-raw' : speed = count difference over a 20 ms window             -> pessimistic"""
import math, sys, numpy as np
from multiprocessing import Pool
import botsim, motor_compare as mc
from botsim import BotSim, LIB, Inputs
mc.MOTORS["DM-H6215"] = (0.716, 13.3, 2.5, 2.0, 0.360, 600e-7)   # Kt from 320 rpm @ 24 V; inertia estimated
CASES = [("MF6015-V2 15T (current)", "abs16"), ("MF5015-V2 10T", "abs16"),
         ("DM-H6215", "hall-ok"), ("DM-H6215", "hall-raw"),
         # the bigger 70 mm frame, as a replacement rather than a saving (see README)
         ("MF7015-V2 10T", "abs16"), ("MF7015-V2 23T", "abs16"), ("MF7015-V2 45T", "abs16")]

class Sim(BotSim):
    def __init__(self, enc_mode, **kw):
        super().__init__(**kw); self.enc_mode = enc_mode; self.hist = []
    def sense(self):
        i = super().sense()
        if self.enc_mode == "abs16": return i
        s, sd = self.d.sensordata, self.sid
        q = (s[sd["ql"]], s[sd["qr"]]); step = 2 * math.pi / 84
        cq = [math.floor(x / step) * step for x in q]
        self.hist.append((cq, (s[sd["wl"]], s[sd["wr"]])))
        if self.enc_mode == "hall-ok":
            lag = self.hist[max(0, len(self.hist) - 6)][1]
            w = [round(math.degrees(x)) * math.pi / 180 for x in lag]
        else:
            n = 20; old = self.hist[max(0, len(self.hist) - 1 - n)][0]
            w = [(cq[k] - old[k]) / (n * botsim.CTRL_DT) for k in (0, 1)]
        i.ql, i.qr, i.wl, i.wr = cq[0], cq[1], w[0], w[1]
        return i

def make(name, enc, seed):
    tmax = mc.setup(name)
    s = Sim(enc, seed=seed, randomize=True); mc.apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    return s

def wobble(a):
    name, enc, seed = a; s = make(name, enc, seed)
    s.place(0); s.hold(); s.run(2.0); n0 = len(s.log); s.run(10.0)
    L = np.array(s.log[n0:]); ok = bool((L[:, 1] == 4).all())
    pitch = np.degrees(L[:, 2]); x = L[:, 9] - L[0, 9]
    tq = 0.5 * (L[:, 7] + L[:, 8]); dtq = np.diff(tq)
    return ok, pitch.std(), np.ptp(pitch), np.ptp(x) * 1000, np.sqrt((dtq ** 2).mean())

def push(a):
    name, enc, seed, imp = a; s = make(name, enc, seed)
    s.place(0); s.hold(); s.run(1.0)
    for _ in range(50): s.step(push=np.array([imp / 0.1, 0, 0]))
    s.run(3.0); L = np.array(s.log)
    return bool((L[500:, 1] == 4).all())

def drive(a):
    name, enc, seed = a; s = make(name, enc, seed)
    s.place(0); s.hold(); s.run(1.0)
    s.run(3.0, v=1.0); s.run(3.0, v=0.0); s.run(3.0, mode=2, angle=math.radians(180), seq=1); s.run(2.0)
    return bool((np.array(s.log)[200:, 1] == 4).all())

if __name__ == "__main__":
    N = 10; IMPS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
    with Pool(4) as p:
        for name, enc in CASES:
            w = p.map(wobble, [(name, enc, 100 + i) for i in range(N)])
            pu = {imp: sum(p.map(push, [(name, enc, 300 + i, imp) for i in range(N)])) for imp in IMPS}
            d = p.map(drive, [(name, enc, 700 + i) for i in range(N)])
            med = lambda k: np.median([x[k] for x in w])
            print(f"{name} [{enc}] | standing 10 s: {sum(x[0] for x in w)}/{N} stayed up, pitch rms {med(1):.3f} deg, "
                  f"pitch p-p {med(2):.2f} deg, wheel creep p-p {med(3):.1f} mm, torque jitter {med(4)*1000:.1f} mN·m/step | "
                  f"pushes " + " ".join(f"{k}:{v}/{N}" for k, v in pu.items()) + f" N·s | drive/stop/turn {sum(d)}/{N}", flush=True)
