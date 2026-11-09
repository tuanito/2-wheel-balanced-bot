"""2D occupancy-grid mapping from lidar scans + wheel/gyro odometry, with a small correlative scan matcher
that corrects odometry drift (wheel slip, stand-ups). Also logs every raw scan to disk."""
import math, time, pathlib, threading
import numpy as np

L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.85, -4.0, 4.0


def compose(a, b):
    """pose a ∘ pose b (x, y, theta)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class GridMap:
    def __init__(self, size_m=24.0, res=0.05):
        self.res = res
        self.n = int(round(size_m / res))
        self.origin = -size_m / 2          # world coordinate of cell (0, 0) corner, same for x and y
        self.L = np.zeros((self.n, self.n), np.float32)   # [iy, ix]
        self.lock = threading.Lock()

    def to_cell(self, xy):
        return np.floor((xy - self.origin) / self.res).astype(np.int32)

    def inside(self, c):
        return (c[..., 0] >= 0) & (c[..., 0] < self.n) & (c[..., 1] >= 0) & (c[..., 1] < self.n)

    def update(self, pose, pts_robot, max_range=8.0):
        """pts_robot: (N,2) lidar hits in the robot frame. Free space along each ray, occupied at the end."""
        if len(pts_robot) == 0: return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        R = np.array([[c, -s], [s, c]])
        hits = pts_robot @ R.T + np.array(pose[:2])
        start = np.array(pose[:2])
        d = np.linalg.norm(hits - start, axis=1)
        steps = np.maximum((d / (self.res * 0.7)).astype(int), 1)
        smax = int(steps.max())
        t = np.arange(smax)[None, :] / steps[:, None]                 # (N, smax), only t < 1 is free space
        valid = t < 1.0
        free = start[None, None, :] + t[..., None] * (hits - start)[:, None, :]
        fc = self.to_cell(free[valid])
        hc = self.to_cell(hits[d < max_range])
        with self.lock:
            fc = fc[self.inside(fc)]
            if len(fc):
                lin = np.unique(fc[:, 1] * self.n + fc[:, 0])
                self.L.ravel()[lin] += L_FREE
            hc = hc[self.inside(hc)]
            if len(hc):
                lin = np.unique(hc[:, 1] * self.n + hc[:, 0])
                self.L.ravel()[lin] += L_OCC - L_FREE           # undo the free update on the hit cell, then occupy
            np.clip(self.L, L_MIN, L_MAX, out=self.L)

    def score(self, pose, pts_robot):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        w = pts_robot @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])
        cell = self.to_cell(w)
        ok = self.inside(cell)
        return float(self.L[cell[ok, 1], cell[ok, 0]].clip(0, None).sum())

    def occupied_world(self, thresh=1.0):
        iy, ix = np.nonzero(self.L > thresh)
        return np.stack([self.origin + (ix + 0.5) * self.res, self.origin + (iy + 0.5) * self.res], 1)

    def save(self, folder, trajectory=None):
        folder = pathlib.Path(folder); folder.mkdir(parents=True, exist_ok=True)
        with self.lock: L = self.L.copy()
        np.savez_compressed(folder / "map.npz", logodds=L, res=self.res, origin=self.origin)
        img = (255 * (1 - 1 / (1 + np.exp(-L)))).astype(np.uint8)          # white free, black occupied, grey unknown
        rgb = np.stack([img] * 3, -1)[::-1].copy()                          # north up
        if trajectory is not None and len(trajectory):
            c = self.to_cell(np.asarray(trajectory)[:, :2])
            c = c[self.inside(c)]
            rgb[self.n - 1 - c[:, 1], c[:, 0]] = (220, 40, 40)
        try:
            from PIL import Image
            Image.fromarray(rgb).save(folder / "map.png")
        except ImportError:
            pass
        return folder / "map.png"


class ScanMatcher:
    """Correlative scan matching: try poses on a small grid around the odometry guess, keep the one whose lidar
    hits land on the most strongly occupied cells. Vectorised over translations (fine for a Pi Zero 2 at 10 Hz)."""
    def __init__(self, grid: GridMap, xy=0.0, xy_step=0.025, th=math.radians(2.5), th_step=math.radians(0.5)):
        # xy=0: correct heading only. In sim, wheel odometry is good for distance; matching translation too made it worse.
        self.g = grid
        k, j = int(round(xy / xy_step)), int(round(th / th_step))
        d = np.arange(-k, k + 1) * xy_step
        self.offsets = np.array([(dx, dy) for dx in d for dy in d])          # (M, 2), contains (0, 0)
        self.zero = int(np.argmin(np.abs(self.offsets).sum(1)))
        self.dth = np.arange(-j, j + 1) * th_step

    def match(self, pose, pts):
        """Best pose near `pose`. Returns (pose, relative score gain over the guess)."""
        if len(pts) < 30: return pose, 0.0
        pts = pts[:: max(1, len(pts) // 200)]
        L = self.g.L.clip(0, None); n, res, org = self.g.n, self.g.res, self.g.origin
        pen = 4.0 * np.linalg.norm(self.offsets, axis=1)                    # mild preference for the odometry guess
        best, best_s, base = pose, -1e9, None
        for dt in self.dth:
            th = pose[2] + dt
            c, s = math.cos(th), math.sin(th)
            w = pts @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])        # (P, 2)
            cells = np.floor((w[None, :, :] + self.offsets[:, None, :] - org) / res).astype(np.int32)   # (M, P, 2)
            ok = (cells >= 0).all(-1) & (cells < n).all(-1)
            cx, cy = np.clip(cells[..., 0], 0, n - 1), np.clip(cells[..., 1], 0, n - 1)
            sc = (L[cy, cx] * ok).sum(1) - pen - 3.0 * abs(math.degrees(dt))
            if dt == 0.0: base = float(sc[self.zero])
            i = int(np.argmax(sc))
            if sc[i] > best_s: best_s, best = float(sc[i]), (pose[0] + self.offsets[i, 0], pose[1] + self.offsets[i, 1], th)
        gain = (best_s - base) / max(base, 1.0) if base is not None else 0.0
        return best, gain


class ScanLog:
    """Stores every scan (time, corrected pose, angles, ranges) in compressed chunks: data/scans/scans_<t>.npz"""
    def __init__(self, folder, chunk=300):
        self.folder = pathlib.Path(folder); self.folder.mkdir(parents=True, exist_ok=True)
        self.chunk, self.items = chunk, []

    def add(self, t, pose, angles, ranges):
        self.items.append((t, pose, angles.astype(np.float32), ranges.astype(np.float32)))
        if len(self.items) >= self.chunk: self.flush()

    def flush(self):
        if not self.items: return
        t = np.array([i[0] for i in self.items]); pose = np.array([i[1] for i in self.items])
        n = np.array([len(i[2]) for i in self.items]); idx = np.concatenate([[0], np.cumsum(n)])
        np.savez_compressed(self.folder / f"scans_{int(t[0])}.npz", t=t, pose=pose, index=idx,
                            angles=np.concatenate([i[2] for i in self.items]), ranges=np.concatenate([i[3] for i in self.items]))
        self.items = []


class Mapper:
    """Feed it (odometry pose, scan). Keeps a drift correction so pose = correction ∘ odometry."""
    def __init__(self, folder="data", size_m=24.0, res=0.05, match=True, log_scans=True):
        self.grid = GridMap(size_m, res)
        self.matcher = ScanMatcher(self.grid)
        self.corr = (0.0, 0.0, 0.0)
        self.pose = (0.0, 0.0, 0.0)
        self.match = match
        self.log = ScanLog(pathlib.Path(folder) / "scans") if log_scans else None
        self.folder = folder
        self.trajectory = []
        self.n = 0
        self._last_odom = None

    def update(self, odom, scan, moving_ok=True):
        guess = compose(self.corr, odom)
        pts = scan.points()
        if self.match and self.n > 5 and moving_ok:
            best, gain = self.matcher.match(guess, pts)
            if gain > 0.02:                                            # clearly better than odometry alone
                # move half-way: smooths out noise in single scans, still removes drift within a second
                best = (0.5 * (guess[0] + best[0]), 0.5 * (guess[1] + best[1]), guess[2] + 0.5 * (best[2] - guess[2]))
                self.corr = compose(best, inverse(odom))
                guess = best
        self.pose = guess
        if moving_ok:
            self.grid.update(guess, pts)
        if self.log: self.log.add(scan.t, guess, scan.angles, scan.ranges)
        self.trajectory.append(guess)
        self.n += 1
        return guess

    # ---------------- V2: no lidar, map with the front sonar ----------------
    SONAR_X = 0.031          # sonar face ahead of the axle, m
    SONAR_HALF = math.radians(7.5)   # HC-SR04 beam half-angle
    SONAR_MAX = 2.0          # m: readings beyond this (or no echo) only clear space

    def update_sonar(self, odom, r, moving_ok=True):
        """One sonar reading (m, <= 0 = no echo). Marks the beam free up to the echo and the echo arc occupied.
        Odometry only (no scan matching: one beam has nothing to match)."""
        guess = compose(self.corr, odom)
        self.pose = guess
        if moving_ok:
            a = np.linspace(-self.SONAR_HALF, self.SONAR_HALF, 7)
            hit = 0.03 < r < self.SONAR_MAX
            rr = r if hit else self.SONAR_MAX
            pts = np.c_[self.SONAR_X + rr * np.cos(a), rr * np.sin(a)]
            src = compose(guess, (self.SONAR_X, 0.0, 0.0))
            self.grid.update(src, pts - [self.SONAR_X, 0.0], max_range=8.0 if hit else rr - 0.01)
        self.trajectory.append(guess)
        self.n += 1
        return guess

    def nearby_obstacles(self, radius=1.5, thresh=1.0):
        """Occupied map cells within radius, in the robot frame: the avoider's memory of what the sonar saw."""
        pts = self.grid.occupied_world(thresh)
        if len(pts) == 0: return np.zeros((0, 2))
        d = pts - np.array(self.pose[:2])
        d = d[np.einsum("ij,ij->i", d, d) < radius * radius]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        return d @ np.array([[c, -s], [s, c]])

    def save(self):
        if self.log: self.log.flush()
        return self.grid.save(self.folder, self.trajectory)
