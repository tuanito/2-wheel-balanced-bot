"""Docking: dock column line fit from a lidar scan, the reverse-in pilot, the Brain's dock / undock commands."""
import math, struct, sys, pathlib, json, time, asyncio
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi"))
from robot import dock as D
from robot import link as L


def synth_scan(X, Y, psi, n=450, noise=0.003, seed=0):
    """LD19-like scan of the dock column (80 mm face + 34 mm sides) seen from robot pose (X, Y, psi) in the dock frame."""
    rng = np.random.default_rng(seed)
    segs = [((0, -0.04), (0, 0.04)), ((0, 0.04), (-0.034, 0.04)), ((0, -0.04), (-0.034, -0.04)),
            ((-0.5, -2), (-0.5, 2))]                      # + a wall 0.5 m behind the dock
    a = np.arange(n) * 2 * math.pi / n
    r = np.zeros(n)
    o = np.array([X, Y])
    for i, ai in enumerate(a):
        d = np.array([math.cos(ai + psi), math.sin(ai + psi)]); best = 0.0
        for p0, p1 in segs:
            p0, p1 = np.array(p0), np.array(p1); e = p1 - p0
            M = np.array([d, -e]).T
            if abs(np.linalg.det(M)) < 1e-12: continue
            t, u = np.linalg.solve(M, p0 - o)
            if t > 0 and 0 <= u <= 1 and (best == 0 or t < best): best = t
        r[i] = best + rng.normal(0, noise) if best > 0 else 0.0
    return a, r


@pytest.mark.parametrize("X,Y,psi", [(0.6, 0.0, 0.0), (0.4, 0.03, 0.15), (0.25, -0.05, -0.2), (0.12, 0.01, 0.05), (0.08, 0.0, 0.0)])
def test_fit_dock_recovers_the_pose(X, Y, psi):
    a, r = synth_scan(X, Y, psi)
    f = D.fit_dock(a, r)
    assert f is not None
    assert f.X == pytest.approx(X, abs=0.006)
    # lateral offset = range x heading error of the 80 mm face fit: loose far out, tight close in (the pilot filters it)
    assert f.Y == pytest.approx(Y, abs=0.006 + 0.12 * X)
    assert f.psi == pytest.approx(psi, abs=math.radians(5))


def test_fit_dock_ignores_a_plain_wall():
    a = np.arange(450) * 2 * math.pi / 450
    r = np.where(np.cos(a) < -0.2, 0.4 / np.maximum(-np.cos(a), 1e-3), 0.0)     # flat wall 0.4 m behind, nothing else
    r[r > 3] = 0
    assert D.fit_dock(a, r) is None


def tape_scan(X, Y, psi, n=450, noise=0.003, seed=0):
    """synth_scan + LD19-like intensity: the two tape strips on the face read ~245, the rest ~100."""
    a, r = synth_scan(X, Y, psi, n, noise, seed)
    th = a + psi
    hx, hy = X + r * np.cos(th), Y + r * np.sin(th)
    on = (r > 0) & (np.abs(hx) < 0.002) & (np.abs(hy) > 0.012) & (np.abs(hy) < 0.037)
    return a, r, np.where(on, 245.0, 100.0)


def test_fit_dock_tape_finds_the_strips_and_ignores_a_wall():
    a, r, it = tape_scan(0.3, 0.02, 0.1)
    f = D.fit_dock_tape(a, r, it)
    assert f is not None and f.tape
    assert f.X == pytest.approx(0.3, abs=0.006) and f.Y == pytest.approx(0.02, abs=0.012)
    a, r, it = tape_scan(0.3, 0.02, 0.1)
    assert D.fit_dock_tape(a, r, np.full_like(it, 100.0)) is None     # same shape, no tape: not the dock
    a2, r2, it2 = tape_scan(0.35, 0.0, math.pi)                       # facing the dock: found all round
    assert D.fit_dock_tape(a2, r2, it2, behind=False) is not None
    assert D.fit_dock_tape(a2, r2, it2, behind=True) is None


def run_pilot(X, Y, psi, contact=True, t_max=150.0, seed=0):
    """Kinematic world: odometry frame = dock frame. Returns (pilot, pose at the lean or None, n leans)."""
    p = D.DockPilot(); t = 0.0; leans = []; state = "BALANCE"; t_lean = None
    rng = np.random.default_rng(seed)
    while t < t_max:
        scan = tape_scan(X, Y, psi, seed=int(rng.integers(1e9))) if X > 0.02 else None
        m, v, w, lean = p.step(t, scan, (X, Y, psi), state, state == "CHARGING", 0.0)
        if m in ("done", "failed"): break
        if lean:
            leans.append((X, Y, psi)); t_lean = t; state = "DOCK_LAND"
        if state == "DOCK_LAND" and t - t_lean > 1.0:
            state = "CHARGING" if contact else "BALANCE"
        if state == "BALANCE":
            psi = (psi + w * 0.1 + math.pi) % (2 * math.pi) - math.pi
            X += v * 0.1 * math.cos(psi); Y += v * 0.1 * math.sin(psi)
        t += 0.1
    return p, leans


@pytest.mark.parametrize("X,Y,psi", [(0.30, 0.20, 2.0), (0.20, -0.12, 0.0), (0.50, 0.0, 3.1), (0.30, 0.25, -1.5), (0.45, -0.3, 0.8)])
def test_pilot_finds_lines_up_and_reverses_in_from_anywhere_close(X, Y, psi):
    p, leans = run_pilot(X, Y, psi)
    assert p.phase == "done", p.msg
    Xl, Yl, psil = leans[0]
    assert Xl == pytest.approx(D.X_LEAN, abs=0.01)
    assert abs(Yl) < 0.006 and abs(psil) < math.radians(3)


def test_pilot_retries_then_gives_up():
    p, leans = run_pilot(0.4, 0.05, 0.3, contact=False, t_max=400)
    assert p.phase == "failed" and "no contact" in p.msg and len(leans) == D.MAX_TRIES


def test_pilot_gives_up_when_there_is_no_dock():
    p = D.DockPilot(); t = 0.0
    a = np.arange(450) * 2 * math.pi / 450
    while t < 30 and p.phase == "find":
        p.step(t, (a, np.full(450, 2.0), np.full(450, 90.0)), (0, 0, 0), "BALANCE", False); t += 0.1
    assert p.phase == "failed" and "not found" in p.msg


def test_estimate_puts_the_lean_distance_on_the_axle_not_the_lidar():
    """Leaning forward 5° moves the lidar 14 mm away from the dock: the estimate is for the axle."""
    e = D.DockEstimate()
    e.update(D.DockFix(X=0.20, Y=0.0, psi=0.0, n=20, rms=0.002, tape=True), (0, 0, 0), pitch=math.radians(5))
    X, Y, psi = e.robot((0, 0, 0))
    assert X == pytest.approx(0.20 - D.LIDAR_H * math.sin(math.radians(5)), abs=1e-6)


def test_approach_pose_and_dock_file(tmp_path):
    ax, ay, ah = D.approach_pose((1.0, 2.0, math.pi / 2))
    assert ax == pytest.approx(1.0) and ay == pytest.approx(2.0 + D.X_APPROACH - D.X_DOCKED) and ah == pytest.approx(math.pi / 2)
    D.save_dock(tmp_path / "dock.json", (1.0, 2.0, 0.5))
    assert D.load_dock(tmp_path / "dock.json") == (1.0, 2.0, 0.5)
    assert D.load_dock(tmp_path / "missing.json") is None


def test_controller_states_match_the_pi_list():
    """The Pi's state names must follow the C++ State enum (the ESP32 sends the number)."""
    src = (ROOT / "core/controller.hpp").read_text()
    import re
    enum = dict((int(v), k[3:]) for k, v in re.findall(r"\b(ST_\w+)\s*=\s*(\d+)", src))
    assert [enum[i] for i in range(len(enum))] == L.STATES
    modes = dict((int(v), k) for k, v in re.findall(r"\b(CMD_\w+)\s*=\s*(\d+)", src))
    assert modes[L.MODE_DOCK] == "CMD_DOCK" and modes[L.MODE_UNDOCK] == "CMD_UNDOCK"


async def test_brain_undock_sends_undock_mode_with_a_new_seq(tmp_path):
    from robot.brain import Brain
    from test_robot import FakeSerial
    esp = FakeSerial()
    brain = Brain(dict(esp32_port=esp, lidar=False, eyes=False, data_dir=str(tmp_path), log_scans=False))
    esp.inject(L.frame(L.MSG_TELEM, struct.pack(L.TELEM_FMT, 1, 6, 0, 1 | 2, 0, *([0.0] * 12), 0)))   # CHARGING, armed
    await asyncio.sleep(0.15)
    assert brain.state()["charging"]
    seq0 = brain.link.seq
    esp.tx.clear(); brain.command("undock"); await asyncio.sleep(0.15)
    cmds = [struct.unpack(L.CMD_FMT, p) for t, p in L.Parser().feed(bytes(esp.tx)) if t == L.MSG_CMD]
    assert cmds and cmds[-1][0] == L.MODE_UNDOCK and cmds[-1][2] == (seq0 + 1) & 0xFFFF
    brain.command("dock")                                       # already on the charger
    assert brain.mode == "manual"


@pytest.mark.asyncio
async def test_brain_halts_when_the_esp32_announces_power_off(tmp_path):
    """Telemetry flag 32 (flat battery, power cut in 20 s): the brain logs it and starts a clean halt (disabled here)."""
    from robot.brain import Brain
    from test_robot import FakeSerial
    esp = FakeSerial()
    brain = Brain(dict(esp32_port=esp, lidar=False, eyes=False, data_dir=str(tmp_path), log_scans=False, halt_on_power_off=False))
    esp.inject(L.frame(L.MSG_TELEM, struct.pack(L.TELEM_FMT, 1, 4, 0, 32, 0, *([0.0] * 12), 0)))
    await asyncio.sleep(0.15)
    assert brain.link.telem.power_off_pending
    assert any("shutting down" in e for e in brain.state()["events"])


def test_ir_bearing_from_the_four_receivers():
    deg = lambda bits: round(math.degrees(D.ir_bearing(bits)[0]))
    assert deg(0b00000101) == 0            # FL + FR 'A': straight ahead
    assert deg(0b00000001) == 45 and deg(0b00010001) == 90 and deg(0b01010000) == 180
    assert D.ir_bearing(0) == (None, False)
    assert D.ir_bearing(0b01000001)[0] is None          # FL + BR alone: opposite, no bearing
    assert D.ir_bearing(0b00001010)[1] is True          # a 'C' bit: on the dock axis


def kin_ir(X, Y, psi):
    """Ideal receivers (±55° hoods, 5 m) for a kinematic robot at (X, Y, psi) in the dock frame (beacon at the origin)."""
    b = math.atan2(-Y, -X) - psi; bits = 0
    for i, a in enumerate(D.IR_AX):
        if abs(D._wrap(b - a)) < math.radians(55) and math.hypot(X, Y) < 5: bits |= 1 << (2 * i)
    return bits


@pytest.mark.parametrize("X,Y,psi", [(2.5, 1.0, 0.0), (3.0, -1.5, 2.5), (1.6, 0.0, 1.0)])
def test_pilot_homes_on_the_beacon_then_docks(X, Y, psi):
    """Far out (no tape within 1.2 m): heads for the beacon, then lines up and reverses in as from close by."""
    p = D.DockPilot(); t = 0.0; state = "BALANCE"; t_lean = None; phases = []
    rng = np.random.default_rng(1)
    while t < 200:
        scan = tape_scan(X, Y, psi, seed=int(rng.integers(1e9)))
        m, v, w, lean = p.step(t, scan, (X, Y, psi), state, state == "CHARGING", 0.0, kin_ir(X, Y, psi))
        if not phases or phases[-1] != p.phase: phases.append(p.phase)
        if m in ("done", "failed"): break
        if lean: t_lean = t; state = "DOCK_LAND"
        if state == "DOCK_LAND" and t - t_lean > 1.0: state = "CHARGING"
        if state == "BALANCE":
            psi = D._wrap(psi + w * 0.1); X += v * 0.1 * math.cos(psi); Y += v * 0.1 * math.sin(psi)
        t += 0.1
    assert p.phase == "done", (p.msg, phases)
    assert "home" in phases
