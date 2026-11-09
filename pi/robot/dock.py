"""Autonomous docking onto the vertical charger.

The dock is a column (80 mm wide at lidar height) standing on a floor plate. The robot parks 0.45 m in front of it,
facing away, then reverses slowly. Each lidar scan: find the column's flat face behind the robot, fit a line ->
the robot's pose in the dock frame (distance X out from the face, lateral offset Y, heading error psi). Steer so Y
and psi go to zero; at the lean distance tell the ESP32 to lean back (CMD_DOCK with a new seq). The ESP32 lands
the plate on the dock face; the rods touch the contacts -> CHARGING. No contact -> it stands up again: drive out and
retry (3 tries). The first successful dock saves the dock's map pose (data/dock.json), so 'dock' later can plan to
the approach point from anywhere on the map.

Geometry (case/body_case.py): column front plane at lidar height is 74 mm behind the lidar when docked; upright the
plate touches the dock face 14 mm before that (X = 60 mm) and the rods are in the funnels from X = 68 mm.
The robot cannot slide sideways, so the lidar guidance has to bring it in within ~±5 mm (sim: it does); the funnels
take the rest by turning it slightly, the wings only stop a bad approach from hitting the rods.
"""
import json, math, pathlib
from dataclasses import dataclass
import numpy as np

COL_HALF_W = 0.040        # m, column half width (DOCK_TOWER)
X_DOCKED = 0.074          # m, lidar -> column face when docked
X_LEAN = 0.093            # m: lean back from here: the axle is on the floor ridge's crest (sim: leaning from behind it
                          # meant rolling down into the dock uncontrolled)
X_APPROACH = 0.45         # m: the line-up point on the dock axis, where the straight reverse starts
SEARCH_R = 1.2            # m: look for the column this close behind the robot
V_FAR, V_NEAR = -0.07, -0.04   # m/s
K_Y, K_PSI = 6.0, 2.2     # psi_des = K_Y * Y (rad per m), yaw rate = K_PSI * (psi_des - psi)
PSI_MAX = math.radians(20)
MAX_TRIES = 3
LINEUP_MAX = 35.0          # s: a line-up that hasn't converged by then counts as a failed try
LIDAR_H = 0.177            # m, lidar above the axle: leaning by p moves it 0.177·sin(p) along the dock axis


@dataclass
class DockFix:
    X: float              # m, lidar -> column face, along the face normal
    Y: float              # m, robot lateral offset in the dock frame (+ = robot is to the dock's left)
    psi: float            # rad, robot heading relative to the dock axis (0 = facing straight away from it)
    n: int                # points on the face
    rms: float            # m, line fit residual
    tape: bool = False    # found by its retro-reflective strips (an ID: nothing else in the room looks like it)


def _fit_line(p):
    c = p.mean(axis=0)
    u, s, vt = np.linalg.svd(p - c, full_matrices=False)
    d = vt[0]                                  # direction along the face
    n = np.array([-d[1], d[0]])                # normal
    res = (p - c) @ n
    return c, d, n, res


TAPE_W, TAPE_PITCH = 0.025, 0.049   # m: the two retro-reflective strips on the column face, centre to centre
TAPE_MIN = 200                       # LD19 intensity of the tape (paint/walls read ~60-160 close up; calibrate: config)


def _pose_from_face(c, cen_t):
    """Robot pose in the dock frame from the face points c (robot frame) and the face centre cen_t."""
    cen, d, n, res = _fit_line(c)
    if n @ cen > 0: n = -n                                # normal from the face towards the robot (lidar at 0)
    t = np.array([-n[1], n[0]])
    rr = -cen_t
    return float(rr @ n), float(rr @ t), float(math.atan2(-n[1], n[0])), float(np.sqrt(np.mean(res ** 2)))


def fit_dock_tape(angles, ranges, intensity, behind=True, tape_min=TAPE_MIN, max_r=SEARCH_R):
    """The dock by its tape: two bright strips 49 mm apart (centre to centre), 25 mm wide, in line. Their midpoint is
    the face centre (no edge guessing); the column face around them gives the angle. behind=False: look all round."""
    a = np.asarray(angles, float); r = np.asarray(ranges, float); it = np.asarray(intensity, float)
    ok = (r > 0.02) & (r < max_r)
    P = np.c_[r * np.cos(a), r * np.sin(a)]
    if behind: ok &= P[:, 0] < -0.02
    bright = ok & (it >= tape_min)
    idx = np.where(bright)[0]
    if len(idx) < 2: return None
    # strips = runs of bright points (split where neighbours are > 15 mm apart), in angle order
    o = idx[np.argsort(np.mod(a[idx], 2 * np.pi))]
    runs, cur = [], [o[0]]
    for i0, i1 in zip(o[:-1], o[1:]):
        if np.hypot(*(P[i1] - P[i0])) > 0.015: runs.append(cur); cur = []
        cur.append(i1)
    runs.append(cur)
    strips = []
    for rn in runs:
        q = P[rn]
        w = np.hypot(*(q[-1] - q[0])) if len(q) > 1 else 0.0
        if w < 0.045: strips.append((q.mean(axis=0), w, len(q)))     # one strip (or part of it, at a grazing angle)
    best = None
    for i in range(len(strips)):
        for j in range(i + 1, len(strips)):
            ci, cj = strips[i][0], strips[j][0]
            dd = np.hypot(*(ci - cj))
            if abs(dd - TAPE_PITCH) < 0.012 and abs(np.hypot(*ci) - np.hypot(*cj)) < 0.05:
                m = 0.5 * (ci + cj); dist = np.hypot(*m)
                if best is None or dist < best[0]: best = (dist, m)
    if best is None: return None
    m = best[1]
    face = P[ok & (np.hypot(P[:, 0] - m[0], P[:, 1] - m[1]) < 0.042)]   # the 80 mm face, not its sides
    if len(face) < 3: return None
    for thr in (0.008, 0.005):
        cen, d, n, res = _fit_line(face)
        keep = np.abs(res) < thr
        if keep.sum() < 3: break
        face = face[keep]
    X, Y, psi, rms = _pose_from_face(face, m)
    return DockFix(X=X, Y=Y, psi=psi, n=int(len(face)), rms=rms, tape=True)


def fit_dock(angles, ranges, prev: "DockFix" = None, intensity=None, require_tape=False):
    """Find the dock column behind the robot in one scan (robot frame, x forward, y left; angle 0 = forward).
    With intensity (LD19 gives it per point): by its tape first; otherwise (or without tape) by its shape.
    Returns DockFix or None. prev (last fix) narrows the shape search so a nearby wall is not mistaken for the dock."""
    if intensity is not None:
        f = fit_dock_tape(angles, ranges, intensity)
        if f is not None or require_tape: return f
    a = np.asarray(angles, float); r = np.asarray(ranges, float)
    ok = (r > 0.02) & (r < SEARCH_R)
    pts = np.c_[r * np.cos(a), r * np.sin(a)][ok]
    pts = pts[pts[:, 0] < -0.02]                         # behind the robot
    if prev is not None:                                 # expected face centre from the last fix
        cx = -prev.X * math.cos(prev.psi) - prev.Y * math.sin(prev.psi)
        cy = prev.X * math.sin(prev.psi) - prev.Y * math.cos(prev.psi)
        pts = pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) < 0.12]
    if len(pts) < 4: return None
    # cluster: the nearest blob behind (gaps > 4 cm split clusters, scan order = angle order)
    ang = np.arctan2(pts[:, 1], pts[:, 0]); o = np.argsort(np.mod(ang, 2 * np.pi)); pts = pts[o]
    gaps = np.r_[0, np.hypot(*np.diff(pts, axis=0).T)] > 0.04
    labels = np.cumsum(gaps)
    best = None
    for k in np.unique(labels):
        c = pts[labels == k]
        if len(c) < 4: continue
        width = np.hypot(*(c[-1] - c[0]))
        if not 0.04 < width < 0.14: continue             # the column face (+ maybe one side) is 80-115 mm across
        dist = np.hypot(*c.mean(axis=0))
        if best is None or dist < best[0]: best = (dist, c)
    if best is None: return None
    c = best[1]
    # robust line: drop the column's side face / outliers (2 passes)
    for thr in (None, 0.008, 0.005):
        cen, d, n, res = _fit_line(c)
        keep = np.abs(res) < (max(0.006, 2.0 * np.std(res)) if thr is None else thr)
        if keep.sum() < 4: break
        c = c[keep]
    cen, d, n, res = _fit_line(c)
    if n @ cen > 0: n = -n                                # normal from the face towards the robot (lidar at 0)
    t = np.array([-n[1], n[0]])
    # the face centre: project the end points on the face line, the centre is midway when both edges are seen,
    # otherwise one column half-width in from the visible edge that is farther from the robot's axis
    s = (c - cen) @ t
    s0, s1 = s.min(), s.max()
    if s1 - s0 > 2 * COL_HALF_W - 0.02: mid = 0.5 * (s0 + s1)
    elif abs(s0) > abs(s1): mid = s0 + COL_HALF_W
    else: mid = s1 - COL_HALF_W
    face_c = cen + mid * t
    rr = -face_c                                          # robot (lidar) relative to the face centre
    return DockFix(X=float(rr @ n), Y=float(rr @ t), psi=float(math.atan2(-n[1], n[0])), n=int(len(c)),
                   rms=float(np.sqrt(np.mean(res ** 2))))


def _wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


class DockEstimate:
    """The dock's pose in the odometry frame, from lidar fixes. The dock doesn't move, so every fix is a measurement
    of the same thing: average them (close fixes count much more: the face-angle error grows with range), and between
    fixes the robot's own odometry carries the estimate (smooth, no lidar noise in the steering)."""

    def __init__(self):
        self.p = None; self.th = None; self.n = 0

    def update(self, fix, odom, pitch=0.0):
        xo, yo, tho = odom
        psi = fix.psi
        Xa = fix.X - LIDAR_H * math.sin(pitch) * math.cos(psi)       # axle, not lidar (it sits 16 cm higher)
        Ya = fix.Y - LIDAR_H * math.sin(pitch) * math.sin(psi)
        th = _wrap(tho - psi)                                         # dock x axis in the odom frame
        c, s_ = math.cos(th), math.sin(th)
        p = np.array([xo - (c * Xa - s_ * Ya), yo - (s_ * Xa + c * Ya)])
        w = (0.8 if fix.tape else 0.5) / (1.0 + (max(Xa, 0.05) / 0.22) ** 2)
        if self.p is None: self.p, self.th = p, th
        else:
            self.p = self.p + w * (p - self.p)
            self.th = _wrap(self.th + w * _wrap(th - self.th))
        self.n += 1

    def robot(self, odom):
        """-> (X, Y, psi) of the robot (axle) in the dock frame."""
        xo, yo, tho = odom
        c, s_ = math.cos(self.th), math.sin(self.th)
        d = np.array([xo, yo]) - self.p
        return float(c * d[0] + s_ * d[1]), float(-s_ * d[0] + c * d[1]), _wrap(tho - self.th)


# ---------------- IR beacon (dock_beacon.ino): coarse bearing from metres away ----------------
IR_AX = [math.radians(a) for a in (45, -45, 135, -135)]    # FL, FR, BL, BR receiver axes (body_case.IR_RX)
HOME_STOP = 0.45          # m: something this close ahead while homing and no tape yet = the dock's back or side: orbit
ORBIT_R = 0.50            # m, orbit radius around the column until the tape on its face shows up
V_HOME = 0.25             # m/s
HOME_TIMEOUT = 90.0       # s


def ir_bearing(bits):
    """telemetry ir byte -> (bearing rad robot frame or None, on_axis). bit 2i = 'A' (all-round) heard by receiver i,
    bit 2i+1 = 'C' (centre beam, ±6° about the dock axis). Two facing-opposite receivers alone = no bearing."""
    if not bits: return None, False
    vx = vy = 0.0
    for i, a in enumerate(IR_AX):
        if bits >> (2 * i) & 3: vx += math.cos(a); vy += math.sin(a)
    on_axis = bool(bits & 0xAA)
    if math.hypot(vx, vy) < 0.3: return None, on_axis
    return math.atan2(vy, vx), on_axis


def _nearest(a, r, b, half=math.radians(45)):
    """(bearing, range) of the closest lidar return within ±half of bearing b, or None."""
    d = np.abs((a - b + math.pi) % (2 * math.pi) - math.pi)
    m = (d < half) & (r > 0.06)
    if not m.any(): return None
    i = np.argmin(np.where(m, r, 99.0))
    return _wrap(float(a[i])), float(r[i])


class DockPilot:
    """The last metres to the dock. Call step() every lidar scan (10 Hz).
      find    turn on the spot until the dock's tape shows up (all round, within SEARCH_R), or the IR beacon is heard
      home    IR heard, tape not yet seen: turn towards the beacon and drive to it (the lidar's closest return near
              the IR bearing refines it); something HOME_STOP ahead without tape = the dock from behind or the side:
              orbit it at ORBIT_R until the tape on its face shows up
      lineup  get onto the dock's axis X_APPROACH out, facing away: back off if too close to turn, turn towards the
              point, drive to it, turn to face away (lidar re-fixes all the way, odometry in between)
      reverse back in, steering onto the axis; lean back on the ridge crest (lean=True: bump seq)
      lean    the ESP32 lands it; CHARGING -> done. Stood up again -> retry (lineup again), 3 tries.
    step() -> (mode, v, yaw_rate, lean); mode 'dock' (CMD_DOCK), 'velocity', 'done' or 'failed'."""

    def __init__(self):
        self.phase, self.tries, self.t_phase = "find", 1, 0.0
        self.est = DockEstimate()
        self.last_fix_t = None
        self.sub = None
        self.msg = ""
        self.t_home, self.ir_b, self.ir_t = 0.0, 0.0, -99.0
        self.t_lineup = None
        self._home_out = ("velocity", 0.0, 0.0, False)

    def _go(self, ph, t, sub=None):
        self.phase, self.t_phase, self.sub = ph, t, sub

    def step(self, t, scan, odom, state, charging, pitch=0.0, ir=0):
        """scan = (angles, ranges, intensity) robot frame or None; odom = (x, y, yaw) from the ESP32;
        pitch (rad, + forward) moves the lidar 16 cm x sin(pitch) relative to the axle; ir = telemetry ir byte."""
        if self.phase in ("done", "failed"): return self.phase, 0.0, 0.0, False
        if state == "CHARGING" and charging:
            self._go("done", t); self.msg = f"docked (try {self.tries})"; return "done", 0.0, 0.0, False
        if scan is not None and self.phase in ("find", "home", "lineup", "reverse"):
            a, r, it = scan
            f = fit_dock_tape(a, r, it, behind=(self.phase == "reverse")) if it is not None else None
            if f is None and self.phase == "reverse" and it is None: f = fit_dock(a, r)
            if f is not None and f.rms < 0.01:
                self.est.update(f, odom, pitch); self.last_fix_t = t
        if self.phase in ("find", "home") and self.est.n >= 2:
            self._go("lineup", t, "check"); self.msg = "dock seen: lining up"
        if self.phase == "find":
            b, _ = ir_bearing(ir)
            if b is not None:
                self._go("home", t, "turn"); self.t_home = t; self.msg = "beacon heard: homing"
            elif t - self.t_phase > 16.0:
                self._go("failed", t); self.msg = "dock not found"; return "failed", 0.0, 0.0, False
            else: return "velocity", 0.0, 0.5, False
        if self.phase == "home":
            return self._home(t, scan, ir)
        if self.est.n == 0: return "velocity", 0.0, 0.0, False
        X, Y, psi = self.est.robot(odom)
        if self.phase == "lineup":
            return self._lineup(t, X, Y, psi, state)
        if self.phase == "reverse":
            if self.last_fix_t is None or t - self.last_fix_t > 2.0:
                return self._retry(t, "dock not seen")
            if X <= X_LEAN:
                self._go("lean", t); return "dock", 0.0, 0.0, True
            if abs(Y) > 0.010 and X < 0.11:                       # will not make the funnels: out and retry
                return self._retry(t, f"off centre {Y*1000:.0f} mm")
            psi_des = max(-PSI_MAX, min(PSI_MAX, K_Y * Y)) if X > 0.10 else 0.0
            w = K_PSI * _wrap(psi_des - psi)
            v = V_FAR if X > 0.18 else V_NEAR
            return "dock", v, max(-0.5, min(0.5, w)), False
        if self.phase == "lean":
            if state == "BALANCE" and t - self.t_phase > 0.5: return self._retry(t, "no contact")
            if t - self.t_phase > 4.0: return self._retry(t, "timeout")
            return "dock", 0.0, 0.0, False
        return "velocity", 0.0, 0.0, False

    def _home(self, t, scan, ir):
        if t - self.t_home > HOME_TIMEOUT:
            self._go("failed", t); self.msg = "homing timeout"; return "failed", 0.0, 0.0, False
        b, _ = ir_bearing(ir)
        if b is not None: self.ir_b, self.ir_t = b, t
        elif t - getattr(self, "ir_t", -99) > 3.0:              # beacon lost: back to turning and listening
            self._go("find", t); return "velocity", 0.0, 0.5, False
        if scan is None: return self._home_out
        b = self.ir_b
        a, r = scan[0], scan[1]
        near = _nearest(a, r, b) if self.sub != "orbit" else _nearest(a, r, b, math.radians(70))
        if near is not None and near[1] < 1.5: b = near[0]      # the column itself: a precise bearing
        if self.sub == "orbit" or (near is not None and near[1] < HOME_STOP and abs(_wrap(near[0])) < math.radians(30)):
            # dock on the left at ORBIT_R: hold it at +90°, steer in/out on its range (the lidar does it all)
            self.sub = "orbit"
            if near is None: out = ("velocity", 0.0, 0.5, False)
            else:
                bb, rr = near
                if abs(_wrap(bb - math.pi / 2)) > math.radians(50): out = ("velocity", 0.0, 0.6 if _wrap(bb - math.pi / 2) > 0 else -0.6, False)
                else: out = ("velocity", 0.12, max(-0.8, min(0.8, 1.5 * _wrap(bb - math.pi / 2) + 2.0 * (rr - ORBIT_R) + 0.12 / ORBIT_R)), False)
        elif abs(b) > math.radians(20):
            out = ("velocity", 0.0, 0.7 if b > 0 else -0.7, False)
        else:
            out = ("velocity", V_HOME, max(-0.6, min(0.6, 1.5 * b)), False)
        self._home_out = out
        return out

    def _lineup(self, t, X, Y, psi, state):
        if self.sub in ("check", "settle") and getattr(self, "t_lineup", None) is None: self.t_lineup = t
        if t - (self.t_lineup if self.t_lineup is not None else t) > LINEUP_MAX:     # circling the point: count a try
            self.t_lineup = None; return self._retry(t, "line-up took too long")
        if state != "BALANCE": self.t_phase = t; return "velocity", 0.0, 0.0, False
        if t - self.t_phase < 0.8 and self.sub == "settle": return "velocity", 0.0, 0.0, False
        XA = X_APPROACH
        turn = lambda err: max(-0.8, min(0.8, 2.0 * err))
        if self.sub in ("check", "settle"):
            if abs(Y) < 0.020 and XA - 0.12 < X < XA + 0.25 and abs(psi) < math.radians(8):
                self.t_lineup = None; self._go("reverse", t); return "dock", 0.0, 0.0, False
            self.sub = "clear" if X < 0.26 else "aim"
        if self.sub == "clear":                     # too close to the dock to turn on the spot: move straight out
            if X >= 0.28: self.sub = "aim"
            else:
                v = 0.10 if math.cos(psi) >= 0 else -0.10
                return "velocity", v, 0.0, False
        dx, dy = XA - X, -Y
        if self.sub == "aim":                       # face the approach point
            beta = math.atan2(dy, dx)
            err = _wrap(beta - psi)
            if math.hypot(dx, dy) < 0.03: self.sub = "face_away"
            elif abs(err) < math.radians(4): self.sub = "drive"
            else: return "velocity", 0.0, turn(err), False
        if self.sub == "drive":                     # drive to it (forward), heading held on the point
            dist = math.hypot(dx, dy)
            beta = math.atan2(dy, dx)
            err = _wrap(beta - psi)
            if dist < 0.012 or abs(err) > math.radians(60): self.sub = "face_away"
            else: return "velocity", min(0.15, 0.6 * dist + 0.03), turn(err), False
        if self.sub == "face_away":                 # turn to face away from the dock, then check again
            err = _wrap(0.0 - psi)
            if abs(err) < math.radians(2):
                tl = self.t_lineup; self._go("lineup", t, "settle"); self.t_lineup = tl; return "velocity", 0.0, 0.0, False
            return "velocity", 0.0, turn(err), False
        return "velocity", 0.0, 0.0, False

    def _retry(self, t, why):
        if self.tries >= MAX_TRIES:
            self._go("failed", t); self.msg = f"docking failed: {why}"; return "failed", 0.0, 0.0, False
        self.tries += 1; self.msg = f"retry {self.tries}: {why}"
        self._go("lineup", t, "settle")
        return "velocity", 0.0, 0.0, False


def approach_pose(dock_pose):
    """Map pose (x, y, heading) of the approach point: X_APPROACH out from the face, facing away from the dock."""
    x, y, h = dock_pose                          # robot pose when it was docked (heading = facing away)
    return (x + (X_APPROACH - X_DOCKED) * math.cos(h), y + (X_APPROACH - X_DOCKED) * math.sin(h), h)


def save_dock(path, pose):
    pathlib.Path(path).write_text(json.dumps({"pose": [float(p) for p in pose]}))


def load_dock(path):
    try: return tuple(json.loads(pathlib.Path(path).read_text())["pose"])
    except Exception: return None
