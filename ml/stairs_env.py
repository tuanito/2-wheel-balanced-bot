"""Gymnasium environment: learn WHEN to nod, so the lidar can measure the first step of a staircase.

Why this is a learning problem and not a geometry problem
---------------------------------------------------------
`sim/stairs_probe.py` measured the geometry kinematically (the robot placed at a pitch, nothing
balancing). The first tread is visible from a narrow diagonal band of (standoff, pitch), and the riser
comes out of it at 101 +- 1 mm against a true 100:

      standoff   800   700   600   500   450   400   350   300   250  mm
      pitch       --    --    --    26  26-32 29-35 32-41 35-47 38-50 deg

Nothing at all beyond ~550 mm: to see past the step nose the scan plane has to drop below the landing
at the edge, tan(theta) > h_lidar / D, and the ray that clears the nose from 800 mm already overflies
the whole first tread and lands two steps down. That is why `sim/stairs_look.py`, flying the real
controller, got the EDGE right (1.170 +- 55 mm against 1.200) and the riser wrong (88 +- 183 mm): not a
sensing failure, a timing failure. Its reverse run-up was 1.5 s long and left the robot 640-1015 mm out
when the pitch peaked, in the blind regime.

And the timing is not free. A two-wheel balancer cannot hold a pitch standing still - leaning forward
IS accelerating forward - so the lean has to be a nod, and gravity acting over the time the body is
tilted adds up. `sim/stairs_probe.py` part C2 measures what one costs, over randomised robots, with
the real controller flying (a flick: look_time 0, so the depth is the target alone):

    entry speed   target   peak     travel to peak   travel in total   upright
      +0.38 m/s    0.80    34.5 deg     +109 mm         +1332 mm          0%
       0.00 m/s    0.80    34.5 deg       +9 mm         +1043 mm        100%
      -0.27 m/s    0.80    34.6 deg      -92 mm          +441 mm        100%
      -0.53 m/s    0.80    34.7 deg     -169 mm           +98 mm        100%
      -0.53 m/s    0.60    23.9 deg     -142 mm             +0 mm       100%
      -0.53 m/s    1.00    46.2 deg     -185 mm          +880 mm        100%

Three things in that table, all of them constraints on the policy: entering the nod REVERSING is what
pays for it (the nod's own acceleration cancels the reverse speed and the robot ends up where it
started), entering it rolling FORWARDS loses the robot outright, and the depth has to be matched to
the standoff because deep nods cost travel while shallow ones see nothing. So the policy has to arrive
at a particular standoff with a particular speed and pick a depth to match, with the whole manoeuvre
fitting in the landing it has left. Nothing about that is a lookup table.

What it is up against: a hand-scripted version of exactly that recipe (ml/stairs_baseline.py, tuned by
grid search) measures the riser to 3 +- 5 mm on the 100 x 300 mm staircase it was tuned on, and 40% on
randomised ones - because one fixed depth cannot cover 80 to 170 mm of riser. It gets 0/10 at 170 mm.

The episode
-----------
The robot starts upright and balancing, 0.70-1.40 m from the edge, on a staircase whose riser
(80-170 mm) and tread (240-320 mm) are drawn fresh every episode, with the usual randomised robot
(mass, CoM, inertia, tyre grip, battery, torque gain, CAN delay, IMU bias and mount tilt).

It already knows roughly where the edge is: the probe showed the EDGE is recoverable from any standoff
(-3 mm at 800 mm), so a reconnaissance nod from far back is not simulated - the environment hands the
policy that estimate with the measured noise (EDGE_SD) instead, and the policy tracks it with wheel
odometry. The riser is what nobody can get from far away, and it is the only thing rewarded.

Every 50 ms the policy chooses a wheel speed, and whether to nod now and how deep. It gets MAX_NODS
nods, each costing NOD_COST, and keeps whichever one produced the most confident reading - most returns
on the surface it measured the riser from, which is a number the robot has. The reward for the
measurement is paid as the improvement in that best reading, so the nod that found it gets the credit.

Observation (8 floats, all of them things the real robot has):
  d_est / 1.5   distance to the edge: the reconnaissance estimate minus wheel odometry
  v / V_MAX     odometry speed, + = towards the edge
  pitch         rad, the controller's own estimate
  0.1 * pitch rate
  t / T_MAX     time used
  nods / MAX_NODS
  confidence    min(returns on the measured surface / 400, 1): 0 until something has been measured
  last action   the speed command of the previous step (one CAN period of latency)
Action (3 floats in [-1, 1]):
  v command = V_MAX * a0 m/s; nod now if a1 > 0; nod depth = NOD_LO..NOD_HI rad from a2.
  The speed command is NOT zeroed for the nod: look mode ignores it while the nod runs and the balance
  controller picks it up on the way out, so one action covers both entering and pulling back.
"""
import math, pathlib, sys
import numpy as np
import gymnasium as gym

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from botsim import BotSim, LIB, CTRL_DT, quat_pitch       # noqa: E402
from stairs_probe import estimate, model_xml, RANGE_NOISE  # noqa: E402

BALANCE = 4
N_RAYS = 120
EDGE = 1.2                # m: where the landing ends, in world x
STEPS = 6
HOLD = 25                 # control ticks per policy step = 50 ms (at 0.5 m/s, 25 mm of approach)
T_MAX = 5.5               # s of episode time, nods included (a nod costs NOD_RUN of it)
MAX_NODS = 3
NOD_RUN = 1.0             # s the nod owns: the lidar is sampled over it, then control goes back
SETTLE = 1.0              # s at the end, still counted against the edge margin
NOD_LO, NOD_HI = 0.55, 1.00   # rad: pitch target of the flick (look_time 0), peaks at 21-46 deg
V_MAX = 0.6               # m/s the speed command spans
EDGE_SD = 0.020           # m: noise on the reconnaissance edge estimate (sim/stairs_probe.py)
LOOK_MIN = math.radians(12)    # rad: below this the scan plane is too flat to see the floor at all
RISE_LO, RISE_HI = 0.08, 0.17
TREAD_LO, TREAD_HI = 0.24, 0.32
D_LO, D_HI = 0.70, 1.30   # m: starting standoff
MEAS = 10.0               # reward for a perfect riser
ERR_OK = 0.020            # m: riser error that earns exp(-1) of MEAS
BROAD = 2.0               # a second, wide basin, so a 50 mm error still has a gradient to follow
ERR_BROAD = 0.080
ERR_SUCCESS = 0.020       # m: what counts as a measurement in the reported success rate
NOD_COST = 0.25           # per nod: nodding is never free, so three blind nods are worse than one good one
TIME_COST = 0.01          # per policy step
NO_NOD = -2.0             # ran out of time without ever looking
FELL = -5.0               # went over
OFF_EDGE = -10.0          # a wheel over the drop
EDGE_HARD = 0.06          # m: wheel radius is 54 mm, so this is "a wheel over the edge"
EDGE_SOFT = 0.20          # m: margin below which a graded penalty starts
EDGE_SOFT_W = 4.0
MARGIN_OK = 0.10          # m: margin required for a reported success
CONF_FULL = 400.0         # returns on the measured surface that count as full confidence in the obs


def meas_score(err):
    """Two basins: 20 mm to decide the measurement is good, 80 mm so a near miss still points uphill."""
    return MEAS * math.exp(-(err / ERR_OK) ** 2) + BROAD * math.exp(-(err / ERR_BROAD) ** 2)


class StairsLookEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, rise=None, tread=None, d0=None, randomize=True):
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (8,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.sim = None
        self.fixed = dict(rise=rise, tread=tread, d0=d0)
        self.randomize = randomize

    # ---------------------------------------------------------------- the cloud
    def _prep_sensors(self):
        m = self.sim.m
        self._sites = np.array([m.site(f"ray{i}").id for i in range(N_RAYS)])
        self._adr = np.array([m.sensor(f"lidar{i}").adr[0] for i in range(N_RAYS)])

    def _cloud(self, dpitch):
        """World points the fan struck this tick, with LD19 range noise and the controller's attitude error.

        Built from MuJoCo's own site frames (no sign conventions to get wrong), then rotated about the
        lidar by `dpitch`, the difference between the pitch the controller believes and the true one. On
        the robot that rotation is the only way the cloud can be built at all - the attitude comes from
        the IMU - so leaving it out would make the estimate look better than it can be.
        """
        d = self.sim.d
        r = d.sensordata[self._adr]
        ok = (r > 0.02) & (r < 12.0)
        if not ok.any():
            return np.zeros((0, 3))
        idx = self._sites[ok]
        org = d.site_xpos[idx]
        zax = d.site_xmat[idx].reshape(-1, 3, 3)[:, :, 2]
        rr = r[ok] + self.rng.normal(0, RANGE_NOISE, int(ok.sum()))
        p = org + rr[:, None] * zax
        if dpitch:                                  # rotate the cloud about the lidar, in the x-z plane
            c, s = math.cos(dpitch), math.sin(dpitch)
            q = p - org
            p = org + np.stack([c * q[:, 0] + s * q[:, 2], q[:, 1], -s * q[:, 0] + c * q[:, 2]], axis=1)
        x = float(d.qpos[0])
        keep = (p[:, 0] > x + 0.05) & (np.abs(p[:, 1]) < 0.25)
        return p[keep]

    # ---------------------------------------------------------------- gym api
    def _obs(self):
        o = self.sim.out
        d_est = self.d0_est - (float(o.x) - self.x_odom0)
        return np.array([d_est / 1.5, float(o.v) / V_MAX, float(o.pitch), 0.1 * float(o.pitch_rate),
                         self.t / T_MAX, self.nods / MAX_NODS,
                         min(self.best_n / CONF_FULL, 1.0), self.a_last], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.sim is not None:
            self.sim.close()
        self.rng = np.random.default_rng(int(self.np_random.integers(0, 2**31 - 1)))
        o = options or {}
        self.rise = o.get("rise") or self.fixed["rise"] or float(self.rng.uniform(RISE_LO, RISE_HI))
        self.tread = o.get("tread") or self.fixed["tread"] or float(self.rng.uniform(TREAD_LO, TREAD_HI))
        d0 = o.get("d0") or self.fixed["d0"] or float(self.rng.uniform(D_LO, D_HI))
        sim_seed = int(self.rng.integers(0, 2**31 - 1))
        self.sim = BotSim(xml_str=model_xml(self.rise, self.tread, STEPS, EDGE),
                          seed=sim_seed, randomize=self.randomize)
        self._prep_sensors()
        self.landing_z = float(self.sim.m.geom("landing").pos[2] + self.sim.m.geom("landing").size[2])
        self.sim.place(pitch=0.0, x=EDGE - d0)
        self.sim.hold(0.6)                           # held still while the controller's filter settles
        self.sim.cmd.mode = 0                        # CMD_VELOCITY
        self.x_odom0 = float(self.sim.out.x)
        self.d0_est = d0 + float(self.rng.normal(0, EDGE_SD))
        self.t, self.a_last, self.x_far = 0.0, 0.0, float(self.sim.d.qpos[0])
        self.nods, self.best_n, self.best_rise, self.best_score = 0, 0.0, None, 0.0
        self.peak, self.d_peak, self.n_pts = 0.0, float("nan"), 0
        self.best_peak, self.best_dpeak, self.nod_pitch = 0.0, float("nan"), 0.0
        return self._obs(), {}

    def _track(self):
        self.x_far = max(self.x_far, float(self.sim.d.qpos[0]))

    def _run(self, ticks, collect=False):
        """Step the simulator, tracking the worst approach to the edge; True if it is still balancing."""
        pts = []
        for _ in range(ticks):
            o = self.sim.step()
            self._track()
            if o.state != BALANCE:
                self.cloud_add(pts)
                return False
            if collect:
                p = float(o.pitch)
                if p > self.peak:
                    self.peak, self.d_peak = p, EDGE - float(self.sim.d.qpos[0])
                if p > LOOK_MIN:
                    c = self._cloud(p - quat_pitch(self.sim.d.qpos[3:7]))
                    if len(c):
                        pts.append(c)
        self.cloud_add(pts)
        return True

    def cloud_add(self, pts):
        self._pts = np.concatenate(pts) if pts else np.zeros((0, 3))
        self.n_pts += len(self._pts)

    def _end(self, r, fell=False, off=False, no_nod=False):
        margin = EDGE - self.x_far
        off = off or margin < EDGE_HARD          # a fall past the edge is both, and should report as both
        if fell:
            r += FELL
        elif off:
            r += OFF_EDGE
        else:
            if no_nod:
                r += NO_NOD
            if margin < EDGE_SOFT:
                r -= EDGE_SOFT_W * (EDGE_SOFT - margin) / (EDGE_SOFT - EDGE_HARD)
        err = None if self.best_rise is None else abs(self.best_rise - self.rise)
        ok = (not fell) and (not off) and err is not None and err <= ERR_SUCCESS and margin >= MARGIN_OK
        info = {"success": float(ok), "measured": float(err is not None),
                "rise_err_mm": 1000.0 * err if err is not None else float("nan"),
                "rise_est_mm": 1000.0 * self.best_rise if self.best_rise is not None else float("nan"),
                "rise_true_mm": 1000.0 * self.rise, "margin_mm": 1000.0 * margin,
                # the LAST nod's geometry, measured or not: when nothing comes out, where the pitch
                # peaked is the first thing to look at
                "d_peak_mm": 1000.0 * self.d_peak, "peak_deg": math.degrees(self.peak),
                "d_best_mm": 1000.0 * self.best_dpeak, "peak_best_deg": math.degrees(self.best_peak),
                "n_pts": self.n_pts, "conf": self.best_n, "nods": self.nods,
                "fell": float(fell), "off_edge": float(off), "no_nod": float(no_nod), "t_end": self.t}
        return self._obs(), r, True, False, info

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self.sim.cmd.v = float(V_MAX * a[0])
        self.a_last = float(a[0])
        nod = a[1] > 0.0 and self.nods < MAX_NODS
        r = -TIME_COST
        if nod:
            tgt = float(NOD_LO + 0.5 * (a[2] + 1.0) * (NOD_HI - NOD_LO))
            LIB.bot_look(self.sim.ctl, 1, tgt, 0.0)          # flick: no hold, the target slews straight back
            self.nods += 1
            r -= NOD_COST
            self.nod_pitch = tgt
            self.peak, self.d_peak = 0.0, float("nan")       # per nod: see d_peak_mm in the info dict
            alive = self._run(int(NOD_RUN / CTRL_DT), collect=True)
            self.t += NOD_RUN
            est = estimate(self._pts, self.landing_z) or {}
            if est.get("rise") is not None and est["n_first"] > self.best_n:
                # Keep the most confident reading, not the luckiest: n_first is how many returns landed on
                # the surface the riser came off, which the robot can count without knowing the answer.
                self.best_n, self.best_rise = float(est["n_first"]), float(est["rise"])
                self.best_peak, self.best_dpeak = self.peak, self.d_peak
                s = meas_score(abs(self.best_rise - self.rise))
                r += s - self.best_score                     # paid as the improvement: telescopes
                self.best_score = s
        else:
            alive = self._run(HOLD)
            self.t += HOLD * CTRL_DT
        if not alive:
            return self._end(r, fell=True)
        if EDGE - self.x_far < EDGE_HARD:
            return self._end(r, off=True)
        if self.t >= T_MAX or self.nods >= MAX_NODS:
            if not self._run(int(SETTLE / CTRL_DT)):
                return self._end(r, fell=True)
            return self._end(r, off=EDGE - self.x_far < EDGE_HARD, no_nod=self.nods == 0)
        return self._obs(), r, False, False, {}

    def close(self):
        if self.sim is not None:
            self.sim.close()
            self.sim = None
