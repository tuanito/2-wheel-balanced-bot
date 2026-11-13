"""Same robot, same controller, different LK motors (datasheet values). Stand-up from both sides + drive/turn."""
import sys, math, numpy as np, ctypes as C
from multiprocessing import Pool
import botsim
from botsim import BotSim, LIB
LIB.bot_torque_limit.argtypes = [C.c_void_p, C.c_float]
MOTORS = {  # name: (Kt N·m/A, speed const rpm/V, line resistance ohm, max torque N·m, motor mass kg, rotor inertia kg m2)
    "MF6015-V2 15T (current)": (0.14, 53.0, 0.89, 2.31, 0.251, 515e-7),       # drive limit 16.5 A
    "MF5015-V2 10T":           (0.08, 97.0, 0.96, 0.70, 0.175, 202e-7),
    "MF4015-V2 20T":           (0.11, 46.9, 6.0, 0.65, 0.116, 81e-7),
    # MF7015-V2 (= MyActuator RMD-L-7015), the larger 70 mm frame. The 23T is the published variant
    # (Dings Motion / MyActuator L-series parameter sheet): 0.28 N.m/A, 34 rpm/V, 1.14 ohm, 933 g.cm2,
    # 360 g, 15 A instantaneous. The 10T and 45T are the same stator rewound, so Kt ~ turns, R ~ turns^2,
    # Kv ~ 1/turns - which reproduces the one published 10T table (79 rpm/V, 0.12 N.m/A) to 1 %, so it is
    # arithmetic rather than a guess. Torque cap = 15 A x Kt clamped to the published peak, the same way
    # the MF6015 row is its drive limit (16.5 A x 0.14 = 2.31) and not its paper 5.5 N.m.
    "MF7015-V2 10T":           (0.122, 78.0, 0.215, 1.83, 0.360, 933e-7),
    "MF7015-V2 23T":           (0.280, 34.0, 1.14, 3.80, 0.360, 933e-7),
    "MF7015-V2 45T":           (0.548, 17.4, 4.36, 8.22, 0.360, 933e-7),
}
def apply_mass(s, name):
    """Lighter motor: 40 % of its mass turns with the wheel (rotor), 60 % stays in the body (stator + drive)."""
    m_tot, j = MOTORS[name][4], MOTORS[name][5]
    m, ch = s.m, s.ch
    m.body_mass[ch] -= 2 * (0.150 - 0.6 * m_tot)
    for side in ("left", "right"):
        b = m.body(f"wheel_{side}").id
        m.body_mass[b] -= 0.101 - 0.4 * m_tot
        m.dof_armature[m.joint(f"{side}_hinge").dofadr[0]] = j
def setup(name):
    kt, kv, r, tmax = MOTORS[name][:4]
    botsim.KT, botsim.KE, botsim.R_LINE, botsim.I_MAX = kt, 1 / (kv * 2 * math.pi / 60), r + 0.1, tmax / kt
    return tmax
def stand(a):
    name, seed, side = a; tmax = setup(name)
    s = BotSim(seed=seed, randomize=True); apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    LIB.bot_tune(s.ctl, min(1.6, tmax), 0.6, 30.0, 0.35)
    s.place(1.8 * side); s.run(6.0)
    L = np.array(s.log); st = L[:, 1].astype(int)
    first = (np.diff((st == 2).astype(int)) == 1).sum() <= 1
    return bool(L[-1, 1] == 4), bool(L[-1, 1] == 4 and first)
def drive(a):
    name, seed = a; tmax = setup(name)
    s = BotSim(seed=seed, randomize=True); apply_mass(s, name); LIB.bot_torque_limit(s.ctl, tmax)
    s.place(0); s.hold(); s.run(1.0)
    s.run(3.0, v=1.0); s.run(3.0, v=0.0)
    s.run(3.0, mode=2, angle=math.radians(180), seq=1)
    for _ in range(50): s.step(push=np.array([10.0, 0, 0]))
    s.run(3.0)
    L = np.array(s.log)
    return bool((L[200:, 1] == 4).all())
if __name__ == "__main__":
    N = 20
    with Pool(2) as p:
        for name in MOTORS:
            r = p.map(stand, [(name, 8000 + i, sd) for i in range(N) for sd in (1, -1)])
            d = p.map(drive, [(name, 9000 + i) for i in range(N)])
            print(f"{name:26s} stand-up {sum(x[0] for x in r)}/{2*N} (first try {sum(x[1] for x in r)})  drive/stop/turn/push without falling {sum(d)}/{N}", flush=True)
