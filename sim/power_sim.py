"""Battery sag and regenerative braking, with the robot simulation in the loop.
Battery = open-circuit voltage + series resistance (cells + BMS FETs + fuse + XT60s + wires). Each motor's drive is
treated as a lossless buck converter: bus current = phase current x (back-EMF + I*R) / Vbus, negative when braking
(energy flows back to the battery). 5 V rail load = 1 A from the bus (~20 W). Step-by-step the bus voltage the motors
and the 5 V regulator see is OCV - R * I_bus.
Also: what the bus voltage would do if the BMS blocked charge current (cells at the over-voltage limit): the braking
energy then only goes into the capacitors on the 6S rail."""
import math, json, numpy as np
from multiprocessing import Pool
import botsim
from botsim import BotSim, LIB

I_5V_BUS = 1.0            # A drawn by the 5 V regulator at ~20 V (4.66 A worst case x 5 V / 90 % / 22 V)
C_BUS = 470e-6 + 2 * 220e-6   # bulk cap + an estimate for each MF6015 drive's input capacitors (not documented)

class PowerSim(BotSim):
    def __init__(self, ocv, r_tot, **kw):
        super().__init__(**kw)
        self.ocv, self.r_tot, self.vbat = ocv, r_tot, ocv
        self.i_acc = 0.0; self.n_acc = 0; self.trace = []
    def motor(self, tau, w):
        tau *= self.tq_gain
        i = max(-botsim.I_MAX, min(botsim.I_MAX, tau / botsim.KT))
        i_hi = (self.vbat - botsim.KE * w) / botsim.R_LINE
        i_lo = (-self.vbat - botsim.KE * w) / botsim.R_LINE
        i = max(i_lo, min(i_hi, i))
        v_ph = botsim.KE * w + i * botsim.R_LINE          # voltage the drive has to make
        self.i_acc += i * v_ph / max(self.vbat, 1.0); self.n_acc += 1
        return botsim.KT * i
    def step(self, push=None):
        self.i_acc = 0.0; self.n_acc = 0
        o = super().step(push)
        n_sub = max(1, self.n_acc // 2)
        i_bus = self.i_acc / n_sub + I_5V_BUS              # both motors, averaged over the control period
        self.vbat = self.ocv - self.r_tot * i_bus
        self.trace.append((self.t, i_bus, self.vbat))
        return o

def standup(a):
    ocv, r, seed, side = a
    s = PowerSim(ocv, r, seed=seed, randomize=True); s.ocv, s.vbat = ocv, ocv   # randomize must not override the battery
    s.place(1.8 * side); s.run(6.0)
    L = np.array(s.log); T = np.array(s.trace)
    ok = bool(L[-1, 1] == 4)
    st = L[:, 1].astype(int); tries = int((np.diff((st == 2).astype(int)) == 1).sum())
    i = T[:, 1]; v = T[:, 2]
    above30 = (i > 30).sum() * botsim.CTRL_DT
    # regen: energy and peak current flowing back
    regen_i = -i.min() if i.min() < 0 else 0.0
    e_regen = float(-(np.clip(i, None, 0) * v).sum() * botsim.CTRL_DT)
    return ok, tries, float(i.max()), float(v.min()), float(above30), regen_i, e_regen, float(v.max())

def drive(a):
    """Drive 1 m/s, hard stop, turns, a push: worst normal-use braking."""
    ocv, r, seed = a
    s = PowerSim(ocv, r, seed=seed, randomize=True); s.ocv, s.vbat = ocv, ocv
    s.place(0); s.hold(); s.run(1.0)
    s.run(3.0, v=1.0); s.run(3.0, v=0.0); s.run(3.0, mode=2, angle=math.radians(180), seq=1)
    for _ in range(50): s.step(push=np.array([15.0, 0, 0]))
    s.run(3.0)
    T = np.array(s.trace); i = T[:, 1]; v = T[:, 2]
    ok = bool((np.array(s.log)[200:, 1] == 4).all())
    return ok, float(i.max()), float(-i.min()), float(-(np.clip(i, None, 0) * v).sum() * botsim.CTRL_DT), float(v.min()), float(v.max())

def blocked_rise(e_regen_J, v0=25.2, c=C_BUS):
    """Bus voltage if this braking energy can only go into the capacitors (BMS blocking charge)."""
    return math.sqrt(v0 ** 2 + 2 * e_regen_J / c)

if __name__ == "__main__":
    OCV = {"full 25.2 V": 25.2, "half 22.8 V": 22.8, "low 21.0 V (3.5 V/cell)": 21.0, "empty 19.8 V (3.3 V/cell)": 19.8}
    R = {"new 35 mΩ": 0.035, "worn/cold 75 mΩ": 0.075, "bad 120 mΩ": 0.12}
    out = {"standup": {}, "drive": {}}
    N = 8
    with Pool(2) as p:
        for on, ocv in OCV.items():
            for rn, r in R.items():
                res = p.map(standup, [(ocv, r, 4000 + i, sd) for i in range(N) for sd in (1, -1)])
                ok = sum(x[0] for x in res); first = sum(1 for x in res if x[0] and x[1] <= 1)
                row = dict(success=f"{ok}/{2*N}", first_try=f"{first}/{2*N}", peak_bus_A=round(max(x[2] for x in res), 1),
                           min_bus_V=round(min(x[3] for x in res), 2), s_above_30A=round(max(x[4] for x in res), 3),
                           peak_regen_A=round(max(x[5] for x in res), 1), regen_J=round(max(x[6] for x in res), 2),
                           max_bus_V=round(max(x[7] for x in res), 2))
                out["standup"][f"{on}, {rn}"] = row
                print("stand-up", on, rn, row, flush=True)
        for on in ("full 25.2 V", "empty 19.8 V (3.3 V/cell)"):
            res = p.map(drive, [(OCV[on], 0.075, 6000 + i) for i in range(N)])
            row = dict(ok=f"{sum(x[0] for x in res)}/{N}", peak_bus_A=round(max(x[1] for x in res), 1), peak_regen_A=round(max(x[2] for x in res), 1),
                       regen_J=round(max(x[3] for x in res), 2), min_bus_V=round(min(x[4] for x in res), 2), max_bus_V=round(max(x[5] for x in res), 2))
            out["drive"][on] = row; print("drive", on, row, flush=True)
    worst_e = max(max(r["regen_J"] for r in out["standup"].values()), max(r["regen_J"] for r in out["drive"].values()))
    out["bms_blocks_charge"] = {"C_bus_uF": round(C_BUS * 1e6), "worst_regen_J_per_manoeuvre": worst_e,
                               "bus_V_if_all_into_caps": round(blocked_rise(worst_e), 1),
                               "J_to_reach_32V": round(0.5 * C_BUS * (32 ** 2 - 25.2 ** 2), 3)}
    print(json.dumps(out["bms_blocks_charge"]))
    json.dump(out, open("results/power_report.json", "w"), indent=1, ensure_ascii=False)
