import sys, itertools, numpy as np, ctypes as C
from multiprocessing import Pool
from botsim import BotSim, STATES, CTRL_DT, LIB
LIB.bot_flip.argtypes = [C.c_void_p, C.c_float, C.c_float, C.c_float, C.c_float]
LIB.bot_flip_gain.argtypes = [C.c_void_p, C.c_float, C.c_float]
def run(a):
    seed, sp, bt, at, ca, kp, kd = a
    s = BotSim(seed=seed, randomize=seed > 0); LIB.bot_flip(s.ctl, sp, bt, at, ca); LIB.bot_flip_gain(s.ctl, kp, kd)
    s.place(0); s.hold(); seen = []; t_bal = None
    for k in range(int(8.0 / CTRL_DT)):
        if k == int(1.0 / CTRL_DT): s.cmd.mode, s.cmd.seq = 3, 1
        o = s.step(); st = STATES[o.state]
        if not seen or seen[-1] != st: seen.append(st)
        if "FLIP_AIR" in seen and st == "BALANCE": s.cmd.mode = 0
    clean = len(seen) >= 5 and seen[:5] == ["BALANCE", "FLIP_RUN", "FLIP_BRAKE", "FLIP_AIR", "BALANCE"] and len(seen) == 5
    return clean, s.out.state == 4
if __name__ == "__main__":
    N = 12
    with Pool(2) as p:
        for sp, bt, at, ca, kp, kd in itertools.product([4.9, 4.5], [0.03], [2.3], [0.6], [1.5, 3.0, 6.0], [0.1, 0.18, 0.3]):
            r = p.map(run, [(i, sp, bt, at, ca, kp, kd) for i in range(N)])
            print(f"speed {sp} brake {bt} air {at} catch {ca} kp {kp} kd {kd}: clean {sum(x[0] for x in r)}/{N}, upright at end {sum(x[1] for x in r)}/{N}", flush=True)
