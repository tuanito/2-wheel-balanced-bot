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
nods (one - see the note on the constant), costing NOD_COST, and keeps whichever produced the most
confident reading: most returns on the surface it measured the riser from, which is a number the robot
has. The reward is paid as the improvement in that best reading, so the nod that found it gets credit.
A speed command that would drive at the drop from inside D_FENCE is refused - see that constant.

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
  v command = V_MAX * a0 m/s; nod now if a1 > 0 (and only inside D_ARM); depth NOD_LO..NOD_HI from a2.
  The speed command is NOT zeroed for the nod: look mode ignores it while the nod runs and the balance
  controller picks it up on the way out, so one action covers both entering and pulling back.
"""
import collections, math, pathlib, sys
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
T_MAX = 4.0               # s of episode time, nods included (a nod costs NOD_RUN of it):
# enough to drive 1.3 m in at 0.5 m/s and still flick, and no more - idle steps are only a
# chance to wander, and they were costing two thirds of the simulator's time.
MAX_NODS = 1
# One nod per episode, and that is a learnability decision with a number behind it. A nod is a large
# irreversible lurch towards the drop, so the nod budget IS the travel budget. From 550-750 mm out, with
# a random policy: driving about without ever nodding goes over the edge in 3% of episodes (the fence
# does its job), one random-depth nod in 30%, two in 78%, three in 97%. At three nods nearly every
# episode ended in the terminal -10 and there was no gradient left to learn from; at one, 75% of
# episodes still produce a reading and a random policy already succeeds 12% of the time. The cost is
# real - probing from far out, approaching and probing again is the natural way to handle a riser the
# robot cannot guess - and that is worth revisiting by warm-starting from a trained policy rather than
# by asking exploration to find it.
NOD_RUN = 1.0             # s the nod owns: the lidar is sampled over it, then control goes back
SETTLE = 1.0              # s at the end, still counted against the edge margin
NOD_LO, NOD_HI = 0.50, 0.85   # rad: flick target (look_time 0), peaks at 19-37 deg. Measured
# against 0.45-0.75: the same off-edge rate but a reading in 75% of random episodes, not 60%.
V_MAX = 0.6               # m/s the speed command spans
D_ARM = 0.72              # m: outside this, a nod is simply not taken.
# The second shield, and the one that makes the task learnable at all. Reaching a standoff where the
# first tread is visible means holding "do not nod yet" for up to 15 consecutive steps, and a Gaussian
# policy draws that sequence about 3 times in 100 000 - so the good trajectory is never sampled, and
# every run so far kept the habit it learned close in and nodded on the first step from wherever it
# started (ep_len 1.05, nodding at 550-600 mm, success 0.07). Arming the nod only inside 800 mm forces
# the approach to happen and leaves the policy the decision that was asked for: where inside 250-800 mm
# to look, how fast to arrive, and how deep to flick. The number is where the measurements put it, and it had
# to be moved once: at 800 mm the policy nodded the instant the gate armed and stopped there, which is
# outside the visible band (seen 10 returns, measured 0.3, success 0.07) - the gate had moved the
# exploration problem rather than solved it, because waiting any longer than forced still costs 8
# consecutive unlikely draws. 720 mm is inside the band part C1 measures and at the standoff the
# ceiling table likes best (0.66-0.74 m scores 0.50, the best available to one nod), so nodding as
# soon as it arms is now roughly right and what is left to learn is the depth and the arrival speed.
# This is worth being plain about: between this gate and the 250 mm driving fence, the window is
# engineered and the policy is choosing within it. The multi-nod version - probe, read the confidence,
# move, refine - is where a policy would genuinely beat a script, and it is disabled for learnability.
D_FENCE = 0.25            # m: inside this, a command to drive TOWARDS the drop is refused.
# A shield, not a hint, and one the real robot wants whether or not anything is learned: it knows where
# the edge is (it is in the observation), so refusing forward speed inside 250 mm of it is a one-line
# auditable rule. It is here because without it undirected exploration is simply fatal - a random policy
# from 650 mm out ends up over the edge in 75-100% of episodes whatever the action ranges, and a terminal
# -10 on nearly every episode is not a gradient. The fence cannot save the robot from a nod that is too
# deep to afford (momentum is not a command), so sizing the nod is still entirely the policy's problem.
EDGE_SD = 0.020           # m: noise on the reconnaissance edge estimate (sim/stairs_probe.py)
LOOK_MIN = math.radians(12)    # rad: below this the scan plane is too flat to see the floor at all
RISE_LO, RISE_HI = 0.08, 0.17
TREAD_LO, TREAD_HI = 0.24, 0.32
D_LO, D_HI = 0.70, 1.30   # m: starting standoff, the real task
# The only curriculum is a STATIC one: training starts the episode anywhere from D_EASY_LO, while
# evaluation starts no nearer than D_LO. Those extra 150 mm of close starts are where the flick itself
# is learnable - from 650 mm at rest one shallow flick (depth 0.60, peak 23 deg) measures the riser to
# 5 mm with 327 mm of margin left, on 6 of 6 randomised robots - and the far starts are what teach the
# approach, so both gradients are present in every batch.
#
# A ratcheting curriculum was tried first and is worth not repeating. It stalls, for a reason that is
# structural rather than a bad threshold: it can only advance on the thing being learned, and with one
# nod per episode the best FIXED standoff and depth scores 0.50 over risers of 80-170 mm, so a rate of
# clean readings much above 0.2 is not available to ratchet on. It sat at 0.78 m for 300 k steps while
# the evaluation starts at 0.70-1.30 m, and the policy scored 0.00 on held-out seeds: it had learned to
# nod immediately, which is right at 650 mm and useless at 1018 mm. Lowering the threshold would only
# have advanced the start distance past what the policy could do. A distribution that always contains
# both ends cannot stall that way.
D_EASY_LO = 0.55          # m: training starts can be this close; evaluation starts no nearer than D_LO
ERR_CURRIC = 0.040        # m: riser error that counts as a clean reading for the ratchet
MEAS = 10.0               # reward for a perfect riser
ERR_OK = 0.020            # m: riser error that earns exp(-1) of MEAS
BROAD = 2.0               # a second, wide basin, so a 50 mm error still has a gradient to follow
ERR_BROAD = 0.080
ERR_SUCCESS = 0.020       # m: what counts as a measurement in the reported success rate
APPROACH_W = 2.0          # bonus for ending up standing off about D_IDEAL from the drop (see below)
D_IDEAL, D_IDEAL_SD = 0.30, 0.35   # one-sided: full credit for anything closer than D_IDEAL
# Why the environment pays for a standoff at all. Without it the first 40 k steps went exactly where the
# reward pointed: a policy that reverses away from the edge and nods blindly loses nothing (-0.8) where
# one that nods while driving at the drop loses everything (-10), so "retreat and nod at nothing" is a
# local optimum - and from 1.3 m back the first tread is never visible, so the +12 for measuring it can
# never be discovered from there. The bonus is a measured property of the SENSOR, not of the episode's
# secret riser: part C1 says the first tread is only visible from 300-500 mm, whatever the riser, so
# "get within about 300 mm" is as much as the reward is allowed to give away. One-sided: being closer
# than D_IDEAL is not penalised here, the edge terms do that. Capped at +2 against the +12 for the
# measurement, so it is a stepping stone and cannot be the destination.
NOD_COST = 0.25           # per nod: nodding is never free, so three blind nods are worse than one good one
TIME_COST = 0.01          # per policy step
NO_NOD = -10.0            # ran out of time without ever looking
FELL = -20.0
OFF_EDGE = -20.0          # a wheel over the drop
# These four numbers have to be ordered, and getting the order wrong is how the last two runs failed.
# What a policy can earn for a good look is MEAS + BROAD + SEEN_W + APPROACH_W = 16, so any penalty
# below that is something to spend rather than avoid:
#   * at NO_NOD -2 against OFF_EDGE -10, never looking was RATIONAL - a nod with an unlearned depth
#     goes over the edge 30% of the time, so looking was worth -3 against a certain -2. The 300 k
#     policy worked that out and stopped nodding at all: not one nod in 12 deterministic episodes.
#   * at NO_NOD -8 against OFF_EDGE -8, "measure well, then fall downstairs" was worth +12 +2 -8 = +6,
#     better than anything safe. That policy measured the riser to 44 +- 48 mm, against the scripted
#     baseline's 205 +- 286, and drove off the edge in 30 of 30 held-out episodes.
# So the drop has to cost more than the best possible reading is worth, and not looking has to cost
# more than a safe bad look. The ordering is now: safe and accurate +16, safe but inaccurate ~0,
# looked and fell -6, never looked -10.
SEEN_W = 2.0              # dense credit for actually pointing the scan at the first tread (see below)
SEEN_FULL = 60.0          # returns on it that count as full credit (measured: a good nod lands 54-101)
# Why this exists: with only the error basins, a nod that measures a surface two steps down scores zero
# and so does a nod that misses everything, so there is no gradient from "wrong by 469 mm" towards
# "right". This term is computed from the TRUE staircase - shaping may use what the policy may not - and
# it rises smoothly as the sweep crosses the visibility band, which is exactly the skill being learned.

EDGE_HARD = 0.06          # m: wheel radius is 54 mm, so this is "a wheel over the edge"
EDGE_SOFT = 0.20          # m: margin below which a graded penalty starts
EDGE_SOFT_W = 4.0
MARGIN_OK = 0.10          # m: margin required for a reported success
CONF_FULL = 400.0         # returns on the measured surface that count as full confidence in the obs


assert FELL < NO_NOD < 0 and abs(FELL) > MEAS + BROAD + SEEN_W + APPROACH_W, (
    "ordering of the penalties: the drop has to cost more than the best possible reading is worth, "
    "and never looking has to cost more than a safe bad look. Two runs were lost to getting this wrong.")


def meas_score(err):
    """Two basins: 20 mm to decide the measurement is good, 80 mm so a near miss still points uphill."""
    return MEAS * math.exp(-(err / ERR_OK) ** 2) + BROAD * math.exp(-(err / ERR_BROAD) ** 2)


class StairsLookEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, rise=None, tread=None, d0=None, randomize=True, curriculum=False):
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (8,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.sim = None
        self.fixed = dict(rise=rise, tread=tread, d0=d0)
        self.randomize = randomize
        # each worker carries its own curriculum: no plumbing through SubprocVecEnv. Default OFF, so
        # evaluation and the baseline always see the real task unless training asks for it.
        self.curriculum = curriculum

    def _d_range(self):
        return (D_EASY_LO if self.curriculum else D_LO), D_HI

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
    def d_est(self):
        """Distance to the edge as the ROBOT has it: the reconnaissance estimate minus wheel odometry."""
        return self.d0_est - (float(self.sim.out.x) - self.x_odom0)

    def _obs(self):
        o = self.sim.out
        d_est = self.d_est()
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
        d0 = o.get("d0") or self.fixed["d0"] or float(self.rng.uniform(*self._d_range()))
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
        self.d0_true = d0
        self.t, self.a_last, self.x_far = 0.0, 0.0, float(self.sim.d.qpos[0])
        self.nods, self.best_n, self.best_rise, self.best_score = 0, 0.0, None, 0.0
        self.peak, self.d_peak, self.n_pts = 0.0, float("nan"), 0
        self.best_peak, self.best_dpeak, self.nod_pitch = 0.0, float("nan"), 0.0
        self.n_seen, self.seen_paid = 0, 0.0
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
            r += APPROACH_W * math.exp(-(max(0.0, margin - D_IDEAL) / D_IDEAL_SD) ** 2)
            if margin < EDGE_SOFT:
                r -= EDGE_SOFT_W * (EDGE_SOFT - margin) / (EDGE_SOFT - EDGE_HARD)
        err = None if self.best_rise is None else abs(self.best_rise - self.rise)
        ok = (not fell) and (not off) and err is not None and err <= ERR_SUCCESS and margin >= MARGIN_OK
        curric = (not fell) and (not off) and err is not None and err <= ERR_CURRIC and margin >= MARGIN_OK
        info = {"success": float(ok), "measured": float(err is not None),
                "rise_err_mm": 1000.0 * err if err is not None else float("nan"),
                "rise_est_mm": 1000.0 * self.best_rise if self.best_rise is not None else float("nan"),
                "rise_true_mm": 1000.0 * self.rise, "margin_mm": 1000.0 * margin,
                # the LAST nod's geometry, measured or not: when nothing comes out, where the pitch
                # peaked is the first thing to look at
                "d_peak_mm": 1000.0 * self.d_peak, "peak_deg": math.degrees(self.peak),
                "d_best_mm": 1000.0 * self.best_dpeak, "peak_best_deg": math.degrees(self.best_peak),
                "n_pts": self.n_pts, "conf": self.best_n, "nods": self.nods,
                "fell": float(fell), "off_edge": float(off), "no_nod": float(no_nod), "t_end": self.t,
                "curric": float(curric), "start_d0": self.d0_true, "n_seen": self.n_seen}
        return self._obs(), r, True, False, info

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        v_cmd = float(V_MAX * a[0])
        if self.d_est() < D_FENCE:
            v_cmd = min(v_cmd, 0.0)          # the fence: no driving at the drop from inside D_FENCE
        self.sim.cmd.v = v_cmd
        self.a_last = float(a[0])
        nod = a[1] > 0.0 and self.nods < MAX_NODS and self.d_est() <= D_ARM
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
            c = self._pts
            if len(c):
                on = ((np.abs(c[:, 2] - (self.landing_z - self.rise)) < 0.02)
                      & (c[:, 0] > EDGE) & (c[:, 0] < EDGE + self.tread))
                self.n_seen = max(self.n_seen, int(on.sum()))
            r += SEEN_W * min(self.n_seen / SEEN_FULL, 1.0) - self.seen_paid
            self.seen_paid = SEEN_W * min(self.n_seen / SEEN_FULL, 1.0)
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
