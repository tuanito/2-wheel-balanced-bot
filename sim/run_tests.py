"""Validation of the shape-C robot + the real controller code (core/controller.cpp) in MuJoCo.

  python3 run_tests.py            -> prints the report, writes results/report.json and plots
Every run uses domain randomisation unless stated: mass ±8 %, CoM ±4 mm (x) / -6..+8 mm (z), inertia -15/+20 %,
tyre friction 0.5-1.2, battery 21-25 V, motor torque ±10 %, 2-4 ms actuation delay, IMU bias + 1.5° mount tilt.
"""
import json, math, pathlib, sys, numpy as np
from multiprocessing import Pool
from botsim import BotSim, STATES, CTRL_DT

OUT = pathlib.Path(__file__).parent / "results"; OUT.mkdir(exist_ok=True)
DEG = math.degrees


def arr(s): return np.array(s.log)


def standup_trial(args):
    seed, side = args
    s = BotSim(seed=seed, randomize=True)
    s.place(pitch=1.8 * side)
    detected = None
    for _ in range(int(5.0 / CTRL_DT)):
        o = s.step()
        if detected is None and o.state == 2: detected = o.side   # side it decided on when it started the lift
    L = arr(s); s.close()
    st = L[:, 1].astype(int)
    up = np.where(st == 4)[0]
    ok = st[-1] == 4 and np.abs(L[-500:, 2]).max() < math.radians(12)
    lift = np.where(st == 2)[0]
    t_up = (L[up[0], 0] - L[lift[0], 0]) if len(up) and len(lift) else float("nan")
    tries = int(np.sum(np.diff((st == 2).astype(int)) == 1))
    return dict(seed=seed, side=side, detected=int(detected or 0), ok=bool(ok), t_up=t_up, tries=tries,
                vmax=float(np.abs(L[:, 5]).max()), travel=float(np.hypot(L[-1, 9] - L[0, 9], L[-1, 10] - L[0, 10])))


def balance_trial(seed):
    s = BotSim(seed=seed, randomize=True)
    s.place(0.0); s.hold()
    s.run(10.0)
    L = arr(s)
    drift = float(np.hypot(L[-1, 9], L[-1, 10]))
    pitch_rms = float(np.degrees(np.std(L[2500:, 2])))
    # push: 10 N for 0.1 s on the body (1 N·s, a firm shove), forwards then backwards
    res = []
    for f in (10.0, -10.0):
        for _ in range(50): s.step(push=np.array([f, 0, 0]))
        s.run(4.0)
        L = arr(s)
        fell = bool((L[-2050:, 1] != 4).any())
        res.append(dict(ok=bool(L[-1, 1] == 4) and not fell, max_pitch=float(DEG(np.abs(L[-2050:, 2]).max())),
                        displaced=float(np.abs(L[-2050:, 9] - L[-2051, 9]).max())))
    s.close()
    return dict(seed=seed, drift_10s=drift, pitch_rms=pitch_rms, pushes=res)


def mission(seed=None, video=False):
    """Back-down start -> stand up -> stand still -> accelerate/cruise/stop -> turns -> arc -> stand still."""
    s = BotSim(seed=seed or 0, randomize=seed is not None)
    s.place(pitch=-1.8)
    marks = []
    def phase(name, secs, **cmd):
        t0 = s.t; s.run(secs, **cmd); marks.append((name, t0, s.t))
    phase("stand up (lying on its back)", 4.0)
    phase("stand still", 3.0, mode=0, v=0.0, yaw_rate=0.0)
    phase("accelerate to 1.0 m/s and cruise", 3.0, v=1.0)
    phase("stop", 3.0, v=0.0)
    phase("turn +90°", 3.0, mode=2, angle=math.radians(90), seq=1)
    phase("turn +180°", 4.0, mode=2, angle=math.radians(180), seq=2)
    phase("turn -135°", 3.5, mode=2, angle=math.radians(-135), seq=3)
    phase("turn to heading 30°", 3.5, mode=1, angle=math.radians(30))
    phase("reverse 0.4 m/s", 3.0, mode=0, v=-0.4, yaw_rate=0.0)
    phase("arc: 0.5 m/s, 1 rad/s", 3.0, mode=0, v=0.5, yaw_rate=1.0)
    phase("stop and stand still", 4.0, mode=0, v=0.0, yaw_rate=0.0)
    L = arr(s)
    t = L[:, 0]
    def sl(name):
        for n, a, b in marks:
            if n == name: return (t > a) & (t <= b)
    yaw0 = L[sl("stand still"), 11][-1]
    rep = {}
    m = sl("stand still")
    rep["stand still: drift (mm)"] = 1000 * float(np.hypot(*(L[m][-1, 9:11] - L[m][0, 9:11])))
    rep["stand still: pitch wobble (deg, std)"] = float(DEG(np.std(L[m, 2])))
    m = sl("accelerate to 1.0 m/s and cruise")
    vt = np.gradient(L[:, 9], t) * np.cos(L[:, 11]) + np.gradient(L[:, 10], t) * np.sin(L[:, 11])
    rep["cruise: true speed over last 1 s (m/s)"] = float(np.mean(vt[m][-500:]))
    rep["cruise: max pitch (deg)"] = float(DEG(np.abs(L[m, 2]).max()))
    m = sl("stop")
    x_stop_cmd = L[m][0, 9:11]
    rep["stop: distance after the stop command (m)"] = float(np.hypot(*(L[m][-1, 9:11] - x_stop_cmd)))
    rep["stop: residual speed after 3 s (m/s)"] = float(abs(vt[m][-1]))
    targets = [("turn +90°", 90), ("turn +180°", 270), ("turn -135°", 135), ("turn to heading 30°", None)]
    for name, rel in targets:
        m = sl(name)
        yaw_end = L[m][-1, 11]
        if rel is None:   # absolute heading 30° in the robot's own heading frame (yaw estimate = 0 at power-up)
            m0 = sl("stand still"); offset = L[m0][-1, 11] - L[m0][-1, 4]       # true - estimated heading
            err = DEG(math.remainder(yaw_end - (offset + math.radians(30)), 2 * math.pi))
        else:
            err = DEG(math.remainder(yaw_end - (yaw0 + math.radians(rel)), 2 * math.pi))
        rep[f"{name}: heading error (deg)"] = float(err)
        rep[f"{name}: position drift while turning (mm)"] = 1000 * float(np.hypot(*(L[m][-1, 9:11] - L[m][0, 9:11])))
    m = sl("stop and stand still")
    rep["final: still balancing"] = bool(L[-1, 1] == 4)
    rep["final: drift in the last 2 s (mm)"] = 1000 * float(np.hypot(*(L[-1, 9:11] - L[-1000, 9:11])))
    rep["whole mission: max |pitch| after stand-up (deg)"] = float(DEG(np.abs(L[t > 4.0, 2]).max()))
    rep["odometry error at the end (mm)"] = 1000 * float(np.hypot(L[-1, 12] - (L[-1, 9] - L[0, 9]), L[-1, 13] - (L[-1, 10] - L[0, 10])))
    s.close()
    return rep, marks, L


def fall_recovery(seed=7):
    s = BotSim(seed=seed, randomize=True)
    s.place(0.0); s.hold(); s.run(1.0)
    for _ in range(50): s.step(push=np.array([40.0, 0, 0]))   # big shove (2 N·s): it falls over
    s.run(5.0)
    L = arr(s); st = L[:, 1].astype(int)
    seq = [STATES[st[i]] for i in range(len(st)) if i == 0 or st[i] != st[i - 1]]
    s.close()
    return dict(states=seq, back_up=bool(st[-1] == 4))


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    with Pool(2) as pool:
        su = pool.map(standup_trial, [(3000 + i, side) for i in range(N) for side in (1, -1)])
        bal = pool.map(balance_trial, range(4000, 4000 + max(6, N // 3)))
    report = {}
    for side, name in ((1, "face-down"), (-1, "back-down")):
        r = [x for x in su if x["side"] == side]
        report[f"stand-up from {name}"] = dict(
            success=f"{sum(x['ok'] for x in r)}/{len(r)}",
            side_detected_correctly=f"{sum(x['detected'] == side for x in r)}/{len(r)}",
            first_try=f"{sum(x['ok'] and x['tries'] == 1 for x in r)}/{len(r)}",
            median_time_s=round(float(np.nanmedian([x['t_up'] for x in r])), 3),
            max_time_s=round(float(np.nanmax([x['t_up'] for x in r])), 3),
            peak_speed_m_s=round(float(np.median([x['vmax'] for x in r])), 2),
            median_travel_m=round(float(np.median([x['travel'] for x in r])), 2),
            max_travel_m=round(float(np.max([x['travel'] for x in r])), 2))
    report["balance 10 s (randomised)"] = dict(
        runs=len(bal), median_drift_mm=round(1000 * float(np.median([b['drift_10s'] for b in bal])), 1),
        max_drift_mm=round(1000 * float(np.max([b['drift_10s'] for b in bal])), 1),
        pitch_std_deg=round(float(np.median([b['pitch_rms'] for b in bal])), 3),
        push_recovered=f"{sum(p['ok'] for b in bal for p in b['pushes'])}/{2 * len(bal)}",
        push_max_pitch_deg=round(float(max(p['max_pitch'] for b in bal for p in b['pushes'])), 1))
    rep, marks, L = mission(None)
    report["mission (nominal robot)"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rep.items()}
    rep2, _, _ = mission(seed=4242)
    report["mission (randomised robot)"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rep2.items()}
    report["knocked over -> gets up again"] = fall_recovery()
    json.dump(report, open(OUT / "report.json", "w"), indent=1)
    np.save(OUT / "mission_log.npy", L); json.dump(marks, open(OUT / "mission_marks.json", "w"))
    print(json.dumps(report, indent=1, ensure_ascii=False))
