"""Gymnasium environment: learn to take one step DOWN, which the hand-written recipe does 60-75% of.

The scripted version is in sim/stairs_descend.py and it works like this: roll at a chosen speed,
and a chosen distance before the edge commit to `step_mode` - hold attitude, speed loop off - then
hand back once the body has settled. Three numbers, tuned by hand on a grid: roll speed, arming
distance, held pitch. It reaches about 65% pooled over risers 60-170 mm.

The three numbers are a poor fit for the problem, and that is the opening. A 60 mm lip and a 170 mm
step want different speeds and different commitment points, and the robot KNOWS which it faces - the
zone array measures the riser to about 13 mm before it ever moves. A fixed recipe cannot use that.
A policy can.

What the policy controls each 20 ms:
  a[0]  wheel speed command, 0 .. V_MAX
  a[1]  commit: > 0 arms step_mode (latching, one-way - a step is not something to change your mind
        about halfway down)
  a[2]  the attitude to hold across the lip, STEP_LO .. STEP_HI rad

What it sees (9 floats): distance to the edge and the measured riser - both as the array would give
them, with noise, not the truth - pitch, pitch rate, wheel speed, whether it has committed, whether
the wheels are unloaded, time, and the last speed command.

Reward design, with the three mistakes from the nod's reward already paid for:

  * never stepping has to cost more than a bad step, or refusing is correct arithmetic and the
    policy learns to stand at the edge for ever. NO_STEP < -(FELL * worst-case rate) is the test.
  * the landing bonus is SCALED by how gently it lands, never flat. The nod's run-2 policy slammed
    through a flat catch window at 16.9 rad/s because the bonus did not care.
  * every term telescopes: the policy is paid the improvement, not the state, so nothing can be
    farmed by sitting still.
"""
import math, pathlib, sys
import numpy as np
import gymnasium as gym

HERE = pathlib.Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "sim"))
from botsim import BotSim, CTRL_DT                                  # noqa: E402
from stairs_probe import model_xml                                  # noqa: E402

EDGE = 1.2
HOLD = 10                  # sim ticks per policy step: 20 ms, fast enough to place the commitment
T_MAX = 6.0
RISE_LO, RISE_HI = 0.06, 0.17
TREAD_LO, TREAD_HI = 0.24, 0.32
D0_LO, D0_HI = 0.30, 0.55
V_MAX = 0.45
STEP_LO, STEP_HI = -0.12, 0.06       # rad to hold across the lip (negative = heels first)
EDGE_SD = 0.013            # m: the zone array knows the edge this well (sim/tof_sensor.py)
RISE_SD = 0.013            # m: and the riser about as well

LAND_OK = 0.80             # rad: peak pitch above this is a tumble, not a step
SETTLE_HOLD = 0.6                 # s the settled state must hold before it counts as a step
SETTLE_P, SETTLE_V = 0.20, 0.12   # it has to STOP on the tread, not merely be upright while rolling
                                  # across it: at 0.35 m/s a policy can "land" and coast straight off
                                  # the next edge, and the first version of this reward paid for that
SUCCESS = 14.0             # paid only on a landing that is actually a landing, and scaled by it
GENTLE = 0.45              # rad: scale of the "how gently" term
NO_STEP = -26.0            # never crossing the edge. WORSE than falling, which inverts the nod's
                           # ordering on purpose: for the nod, going over the edge was catastrophic
                           # and declining to look was merely useless, so falling had to cost more.
                           # Here the robot's whole job is to get down, so a robot that stands at the
                           # top for ever has failed completely while a scruffy step is progress. At
                           # -12 it learned exactly that - crossed_rate 0.01 after 800k steps.
CROSS = 3.0                # paid ONCE, at the moment the edge is crossed, win or lose
FELL = -20.0
PROGRESS_W = 2.0           # telescoping shaping towards the edge
STAB_W = 5.0               # telescoping shaping towards "upright and stopped", AFTER the edge only.
                           # Without it every exploratory step ends in a tumble or a run-on and the
                           # two score alike, so there is no gradient towards landing at all: a
                           # random policy crossed 20 times out of 20 and landed none.
TIME_COST = 0.02

# The ordering is not enough on its own: what matters is the EXPECTED value of stepping while the
# policy is still bad at it. With a crossing bonus paid win or lose, stepping beats refusing once
# p(success) > (NO_STEP + FELL + CROSS) / (SUCCESS + CROSS - FELL - CROSS), about 15% here - which a
# curriculum of small risers reaches almost immediately. Without it the first run learned, correctly,
# never to step at all: crossed_rate sat at 0 for 129k steps.
# E[step] = p*(SUCCESS + CROSS) + (1-p)*(FELL + CROSS) must beat NO_STEP
_p_break = (NO_STEP - CROSS - FELL) / (SUCCESS - FELL)
assert NO_STEP < FELL < 0 and _p_break <= 0.0, (
    f"stepping has to be worth trying from the very first episode: with these numbers it needs "
    f"p(success) > {_p_break:.0%}, and exploration will never get there.")


class DescendEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, rise=None, tread=None, randomize=True, curriculum=False):
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (9,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.sim = None
        self.fixed = dict(rise=rise, tread=tread)
        self.randomize = randomize
        self.curriculum = curriculum
        self.rise_hi = 0.07 if curriculum else RISE_HI      # ratchets up, see train_descend.py
        self.rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------ helpers
    def set_rise_hi(self, v):
        """The curriculum: start on lips the robot can already survive, so stepping pays early."""
        self.rise_hi = float(v)
        return self.rise_hi

    def d_true(self):
        return EDGE - float(self.sim.d.qpos[0])

    def d_est(self):
        return self.d0_est - (float(self.sim.out.x) - self.x_odom0)

    def _obs(self):
        o = self.sim.out
        return np.array([self.d_est() / 0.5, self.rise_est / 0.17, o.pitch, 0.1 * o.pitch_rate,
                         o.v / V_MAX, 1.0 if self.committed else 0.0, float(o.airborne),
                         self.t / T_MAX, self.a_last], dtype=np.float32)

    def _run(self, ticks):
        for _ in range(ticks):
            o = self.sim.step()
            if self.crossed:
                self.peak = max(self.peak, abs(float(o.pitch)))
            if self.d_true() <= 0 and not self.crossed:
                self.crossed, self.t_cross = True, self.t
                self.cross_pay = CROSS
            if int(o.state) not in (3, 4):           # COAST or BALANCE; anything else is a loss
                return False
        return True

    # ------------------------------------------------------------------ gym api
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        o = options or {}
        lo, hi = RISE_LO, (o.get("rise_hi") or self.rise_hi)
        self.rise = o.get("rise") or self.fixed["rise"] or float(self.rng.uniform(lo, hi))
        self.tread = o.get("tread") or self.fixed["tread"] or float(self.rng.uniform(TREAD_LO, TREAD_HI))
        d0 = o.get("d0") or float(self.rng.uniform(D0_LO, D0_HI))
        if self.sim is not None:
            self.sim.close()
        sim_seed = int(self.rng.integers(0, 2 ** 31 - 1))
        self.sim = BotSim(xml_str=model_xml(self.rise, self.tread, 8, EDGE),
                          seed=sim_seed, randomize=self.randomize)
        yaw0 = float(self.rng.normal(0, math.radians(1.5)))   # it never arrives perfectly square
        self.sim.place(pitch=0.0, x=EDGE - d0, yaw=yaw0)
        self.sim.hold(0.6)
        self.sim.cmd.mode = 0
        self.sim.cmd.obstacle_stop = 0
        self.sim.cmd.drop_stop = 0                   # the fence is exactly what this is learning past
        self.sim.cmd.step_mode = 0
        self.sim.cmd.step_max = 1.6
        self.x_odom0 = float(self.sim.out.x)
        # what the array hands over: an edge and a riser, each with its own error
        self.d0_est = d0 + float(self.rng.normal(0, EDGE_SD))
        self.rise_est = self.rise + float(self.rng.normal(0, RISE_SD))
        self.t, self.a_last = 0.0, 0.0
        self.committed, self.crossed, self.t_cross = False, False, None
        self.cross_pay = 0.0
        self.best_stab = 0.0
        self.settle_t = 0.0
        self.peak, self.d_best = 0.0, d0
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        v_cmd = float(0.5 * (a[0] + 1.0) * V_MAX)
        self.a_last = float(a[0])
        if not self.committed and a[1] > 0.0:
            # one-way: committing is a decision about the next 200 ms, not a knob to wobble
            self.committed = True
            self.sim.cmd.step_pitch = float(STEP_LO + 0.5 * (a[2] + 1.0) * (STEP_HI - STEP_LO))
            self.sim.cmd.step_mode = 1
        self.sim.cmd.v = v_cmd
        r = -TIME_COST
        # telescoping progress: paid for getting closer to the edge, once, never for being near it
        d = max(0.0, self.d_true())
        if d < self.d_best:
            r += PROGRESS_W * (self.d_best - d) / 0.5
            self.d_best = d
        alive = self._run(HOLD)
        r += self.cross_pay            # paid once; the episode cannot farm it
        self.cross_pay = 0.0
        self.t += HOLD * CTRL_DT
        if self.crossed:
            o2 = self.sim.out
            stab = (math.exp(-(abs(float(o2.pitch)) / 0.30) ** 2)
                    * math.exp(-(abs(float(o2.v)) / 0.40) ** 2))
            if stab > self.best_stab:           # telescoping: paid the improvement, once
                r += STAB_W * (stab - self.best_stab)
                self.best_stab = stab
        if not alive:
            return self._end(r, fell=True)
        o = self.sim.out
        # off the far end of the tread is not a landing, it is the next step taken by accident
        past = self.d_true()                      # negative once over the edge
        if -past > self.tread - 0.04:
            return self._end(r, ran_on=True)
        # Settled has to HOLD. Ending the episode the instant the robot is briefly upright and slow
        # scores a robot that settles and then topples as a clean step, and it put the scripted
        # baseline at 100% in here against 65-90% in sim/stairs_descend.py - the environment was
        # easier than the task, which is the one thing a training environment must never be.
        settled = (self.crossed and self.t - (self.t_cross or 0) > 0.9
                   and abs(float(o.pitch)) < SETTLE_P and abs(float(o.v)) < SETTLE_V
                   and int(o.state) == 4)
        self.settle_t = self.settle_t + HOLD * CTRL_DT if settled else 0.0
        if self.settle_t >= SETTLE_HOLD:
            return self._end(r, landed=True)
        if self.t >= T_MAX:
            return self._end(r)
        return self._obs(), r, False, False, self._info()

    def _info(self, **kw):
        o = self.sim.out
        d = dict(success=0.0, fell=0.0, no_step=0.0, ran_on=0.0, crossed=float(self.crossed),
                 peak_deg=math.degrees(self.peak), committed=float(self.committed),
                 rise_mm=1000.0 * self.rise, t_end=self.t,
                 end_pitch_deg=math.degrees(float(o.pitch)), end_v=float(o.v))
        d.update(kw)
        return d

    def _end(self, r, fell=False, landed=False, ran_on=False):
        ok = False
        if fell:
            r += FELL
        elif not self.crossed:
            r += NO_STEP
        elif landed:
            # scaled by how gently it went over, never flat: a landing that peaked at 60 degrees is
            # a tumble the robot happened to survive, and must not pay like a step
            gentle = math.exp(-(self.peak / GENTLE) ** 2)
            r += SUCCESS * gentle
            ok = self.peak < LAND_OK
        elif ran_on:
            r += 0.5 * FELL                      # rolled off the far end of the tread
        else:
            r += 0.5 * FELL                      # crossed, never settled: still on the floor somewhere
        info = self._info(success=float(ok), fell=float(fell), no_step=float(not self.crossed),
                          ran_on=float(ran_on))
        return self._obs(), r, True, False, info

    def close(self):
        if self.sim is not None:
            self.sim.close()
            self.sim = None
