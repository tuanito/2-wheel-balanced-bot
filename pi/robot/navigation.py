"""Obstacle avoidance (dynamic-window style) and autonomous exploration.

The ESP32 already refuses to drive forward into whatever the sonar sees. This layer looks all around with the lidar
(+ the sonar point), and bends or slows the commanded (v, yaw_rate) so the predicted path keeps a safe clearance.
"""
import math
import numpy as np

ROBOT_RADIUS = 0.115      # m: half the outer wheel span (208 mm) + a little
MARGIN = 0.05             # m: extra clearance we want to keep
V_MAX, W_MAX = 0.8, 1.2   # m/s, rad/s that the planner may ask for
DECEL = 0.5               # m/s^2 the robot can comfortably brake with (low centre of mass: it must lean back)


def arc_points(v, w, horizon=1.6, dt=0.1):
    t = np.arange(dt, horizon + 1e-9, dt)
    if abs(w) < 1e-3:
        return np.stack([v * t, np.zeros_like(t)], 1)
    th = w * t
    return np.stack([v / w * np.sin(th), v / w * (1 - np.cos(th))], 1)


def clearance_along(path, obstacles):
    """Distance from each path point to the nearest obstacle point (both robot frame)."""
    if len(obstacles) == 0: return np.full(len(path), 10.0)
    d = np.linalg.norm(path[:, None, :] - obstacles[None, :, :], axis=2)
    return d.min(1)


class Avoider:
    def __init__(self, v_max=V_MAX, w_max=W_MAX):
        self.v_max, self.w_max = v_max, w_max
        self.vs = np.linspace(-0.25, v_max, 12)
        self.ws = np.linspace(-w_max, w_max, 15)

    def safe_command(self, v_des, w_des, obstacles, sonar=-1.0):
        """obstacles: (N,2) points in the robot frame (lidar). Returns (v, w, blocked)."""
        obs = obstacles
        if sonar is not None and sonar > 0.02:
            obs = np.vstack([obs, [[sonar + 0.03, 0.0]]]) if len(obs) else np.array([[sonar + 0.03, 0.0]])
        if len(obs):
            obs = obs[np.linalg.norm(obs, axis=1) < 2.5]
        if len(obs) == 0: return v_des, w_des, False
        stop_r = ROBOT_RADIUS + 0.01
        best, best_cost = (0.0, 0.0), 1e9
        cands = [(v_des, w_des)] + [(v, w) for v in self.vs for w in self.ws]
        for v, w in cands:
            if v * v_des < 0 and abs(v_des) > 0.05: continue           # never reverse direction on its own
            path = arc_points(v, w)
            clr = clearance_along(path, obs)
            # the part of the path we would cover before we could stop must stay clear
            brake_t = abs(v) / DECEL + 0.3
            n = max(1, min(len(path), int(math.ceil(brake_t / 0.1))))
            if clr[:n].min() < stop_r: continue
            c_near = clr[:n].min()
            cost = 2.0 * abs(v - v_des) + 0.8 * abs(w - w_des) + 0.3 * max(0.0, ROBOT_RADIUS + MARGIN + 0.2 - c_near)
            if cost < best_cost: best_cost, best = cost, (v, w)
        blocked = best_cost >= 1e9
        if blocked:                                                     # nothing safe: turn on the spot towards open space
            ang = np.arctan2(obs[:, 1], obs[:, 0])
            left = (ang > 0).sum() < (ang < 0).sum()
            return 0.0, (0.6 if left else -0.6), True
        return best[0], best[1], False


class Explorer:
    """Wander: drive towards open, preferably unexplored, space; re-pick the direction regularly."""
    def __init__(self, speed=0.35, rng=None):
        self.speed = speed
        self.goal_heading = None
        self.t_pick = 0.0
        self.rng = rng or np.random.default_rng()

    def command(self, t, pose, scan_pts, grid=None):
        if self.goal_heading is None or t - self.t_pick > 4.0:
            self.goal_heading = self._pick(pose, scan_pts, grid)
            self.t_pick = t
        err = math.atan2(math.sin(self.goal_heading - pose[2]), math.cos(self.goal_heading - pose[2]))
        w = max(-1.0, min(1.0, 1.5 * err))
        v = self.speed * max(0.0, math.cos(err)) ** 2
        return v, w

    def _pick(self, pose, pts, grid):
        angs = np.radians(np.arange(-180, 180, 10))
        if len(pts):
            pa = np.arctan2(pts[:, 1], pts[:, 0]); pr = np.linalg.norm(pts, axis=1)
        score = []
        for a in angs:
            free = 4.0
            if len(pts):
                sel = np.abs(np.arctan2(np.sin(pa - a), np.cos(pa - a))) < math.radians(15)
                if sel.any(): free = min(4.0, pr[sel].min())
            unknown = 0.0
            if grid is not None:                                       # how much unknown map lies 1-3 m that way
                wa = pose[2] + a
                probe = np.array([[pose[0] + r * math.cos(wa), pose[1] + r * math.sin(wa)] for r in (1.0, 1.5, 2.0, 2.5, 3.0)])
                c = grid.to_cell(probe); ok = grid.inside(c)
                unknown = float(np.mean(np.abs(grid.L[c[ok, 1], c[ok, 0]]) < 0.3)) if ok.any() else 0.0
            turn_pen = 0.15 * abs(a)
            score.append(free + 1.5 * unknown - turn_pen + self.rng.uniform(0, 0.3))
        return pose[2] + angs[int(np.argmax(score))]


# ---------------------------------------------------------------------------------------------------------------
class Planner:
    """Breadth-first search on a coarse copy of the occupancy grid, obstacles inflated by the robot radius.
    Used for frontier exploration (go where the map is still unknown) and for 'go to this point' from the web app."""
    def __init__(self, cell=0.10, inflate=0.20):
        self.cell, self.inflate = cell, inflate

    def coarse(self, grid):
        f = int(round(self.cell / grid.res))
        n = grid.n // f
        L = grid.L[:n * f, :n * f].reshape(n, f, n, f)
        occ = (L > 0.6).any(axis=(1, 3))
        free = (L < -0.3).mean(axis=(1, 3)) > 0.5
        k = int(math.ceil(self.inflate / self.cell))
        blocked = occ.copy()
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx * dx + dy * dy <= k * k:
                    blocked |= np.roll(np.roll(occ, dy, 0), dx, 1)
        return free & ~blocked, ~free & ~occ, n

    def search(self, grid, start_xy, goal_xy=None, min_dist=0.6, banned=()):
        """BFS from the robot. goal_xy=None: nearest frontier. Returns a list of world (x, y) waypoints or None."""
        from collections import deque
        ok, unknown, n = self.coarse(grid)
        to_c = lambda p: (int((p[0] - grid.origin) / self.cell), int((p[1] - grid.origin) / self.cell))
        to_w = lambda c: (grid.origin + (c[0] + 0.5) * self.cell, grid.origin + (c[1] + 0.5) * self.cell)
        s = to_c(start_xy)
        if not (0 <= s[0] < n and 0 <= s[1] < n): return None
        # the robot's own cell (and around it) may look blocked right next to a wall: let it start anyway
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                if 0 <= s[1] + dy < n and 0 <= s[0] + dx < n and not unknown[s[1] + dy, s[0] + dx]: ok[s[1] + dy, s[0] + dx] = True
        g = to_c(goal_xy) if goal_xy is not None else None
        parent = {s: None}; q = deque([s]); found = None
        while q:
            c = q.popleft()
            if g is not None and abs(c[0] - g[0]) <= 1 and abs(c[1] - g[1]) <= 1: found = c; break
            if g is None:
                d = math.hypot((c[0] - s[0]) * self.cell, (c[1] - s[1]) * self.cell)
                if d >= min_dist and all(math.hypot(c[0] - b[0], c[1] - b[1]) > 5 for b in banned):
                    x, y = c
                    if unknown[max(0, y - 1):y + 2, max(0, x - 1):x + 2].any(): found = c; break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nb = (c[0] + dx, c[1] + dy)
                if 0 <= nb[0] < n and 0 <= nb[1] < n and nb not in parent and ok[nb[1], nb[0]]:
                    parent[nb] = c; q.append(nb)
        if found is None: return None
        path = []
        while found is not None: path.append(to_w(found)); found = parent[found]
        return path[::-1]


class PathFollower:
    def __init__(self, speed=0.4, lookahead=0.45):
        self.speed, self.look = speed, lookahead
        self.path = None

    def command(self, pose):
        """-> (v, w, done)"""
        if not self.path: return 0.0, 0.0, True
        p = np.array(self.path); d = np.linalg.norm(p - pose[:2], axis=1)
        i = int(np.argmin(d))
        self.path = self.path[i:]
        if len(self.path) <= 2 and d[-1] < 0.15: self.path = None; return 0.0, 0.0, True
        j = next((k for k in range(len(self.path)) if math.hypot(self.path[k][0] - pose[0], self.path[k][1] - pose[1]) > self.look), len(self.path) - 1)
        tx, ty = self.path[j]
        err = math.atan2(ty - pose[1], tx - pose[0]) - pose[2]
        err = math.atan2(math.sin(err), math.cos(err))
        w = max(-1.0, min(1.0, 2.0 * err))
        v = self.speed * max(0.0, math.cos(err)) ** 3
        return v, w, False


class FrontierExplorer:
    """Autonomous mapping: repeatedly drive to the nearest boundary between known-free and unknown space."""
    def __init__(self, speed=0.4):
        self.planner, self.follow = Planner(), PathFollower(speed)
        self.t_plan, self.goal, self.banned, self.done = -1e9, None, [], False
        self.last_progress = (0.0, None)

    def command(self, t, pose, grid):
        stuck = self.last_progress[1] is not None and t - self.last_progress[0] > 6.0 and \
            math.hypot(pose[0] - self.last_progress[1][0], pose[1] - self.last_progress[1][1]) < 0.15
        if self.last_progress[1] is None or t - self.last_progress[0] > 6.0: self.last_progress = (t, pose[:2])
        if stuck and self.goal is not None:
            self.banned.append(self._cell(grid, self.goal)); self.follow.path = None
        if self.follow.path is None or t - self.t_plan > 3.0:
            path = self.planner.search(grid, pose[:2], banned=self.banned)
            self.t_plan = t
            if path is None:
                self.done = True; return 0.0, 0.4, True                   # nothing left to explore: spin slowly
            self.follow.path, self.goal = path, path[-1]
        v, w, arrived = self.follow.command(pose)
        if arrived: self.follow.path = None
        return v, w, False

    def _cell(self, grid, xy):
        return (int((xy[0] - grid.origin) / self.planner.cell), int((xy[1] - grid.origin) / self.planner.cell))
