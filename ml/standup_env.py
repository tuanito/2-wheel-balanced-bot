"""Gymnasium environment: learn the stand-up (lying -> upright) with the real controller in the loop.

The robot lies on its face or back (random side), the C++ controller detects the side, waits until it is still and
starts the push (state LIFT). From there the policy sets the push every 10 ms (`standup_mode = 2`: torque from
outside) until the controller's normal catch takes over (|pitch| < 20°) or it gives up. After the catch the episode
keeps running 1 s under the normal balance controller: success = still balancing, upright, without running off.

Every episode uses a randomised robot (mass, centre of mass, inertia, tyre grip, battery voltage, torque gain, CAN
delay, IMU bias and mount): the same randomisation as sim/run_tests.py, so the policy doesn't overfit one robot.

Curriculum: the first run (2 M steps, ent_coef 0) never found the success bonus - it half-rose, ran out the
controller's 2 s timeout and collected the shaping, and once the exploration noise had annealed away it could no
longer discover anything better. So training starts nearer upright and each worker walks its own start pitch out
to the real 1.8 rad as it succeeds. Evaluation always uses the real task: StandupEnv(curriculum=False).

What the ratchet is gated on matters, and the third run got it wrong: it advanced only on a full success
(caught first try AND still balancing a second later), which sat around 12%, never reached UP_OK, and so left
every worker pinned at PITCH_START for all 2 M steps. The curriculum silently collapsed to "always train at
1.2 rad". It is now gated on a catchable handover - a success, or reaching the catch with `gentle` >= GENTLE_OK
(pitch rate <= ~6.7 rad/s). That is the thing the reward is pushing for, it is reached early in training, and
it gets harder as the start pitch grows, so the ratchet has signal to climb on and self-limits. The strict
success rate is still reported separately (env/success_rate vs env/curric_rate).

The catch is a handover, not a finish line. The second run (flat CATCH_ONLY = +2.0) reached the catch window on
every episode and lost every one of them: measured in ml/evaluate.py's loop it arrived at |pitch rate| 16.9
rad/s median against the hand-tuned stand-up's 3.66. Both swing up equally hard (peak ~14-17 rad/s in the
150 ms before the catch); the hand-tuned one *decelerates into the window*, the learned one is still at its
peak when it crosses. Nothing in the reward mentioned the rate, so the fastest route into the window won and
the gradient from 17 rad/s towards 4 was exactly zero. CATCH_ONLY is now scaled by exp(-(rate/RATE_OK)^2):
~0.8 at the hand-tuned arrival, ~0.01 at a slam, monotone in between. The rate is reported in the info dict
so it shows up in TensorBoard (env/catch_rate_rad_s) instead of having to be measured afterwards.

That rate is differentiated from the simulator's body quaternion, captured at the catch tick. The gyro
itself is faithful (corr 1.000 with the truth over the swing, one control tick of lag), so o.pitch_rate
would have done - but only if read at the right moment: BotSim.step() returns BotSim.out itself, one ctypes
struct the C++ refills in place every tick. Read below the post-catch check loop it reported 5.5 rad/s for a
handover that really happened at 16.9, because by then the struct described the robot a second later.

Observation (6 floats, side-normalised, computed by the controller itself: Controller::standup_obs):
  pitch, 0.1·pitch rate, wheel speed / max, time / timeout, (vbat - 23)/2, last action
Action: 1 float in [-1, 1] = push for both motors as a fraction of 2 × torque_limit (4.6 N·m).
"""
import collections, sys, math, pathlib, ctypes as C
import numpy as np
import gymnasium as gym

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from botsim import BotSim, LIB, CTRL_DT, quat_pitch  # noqa: E402

LIFT, BALANCE = 2, 4
HOLD = 5                      # control ticks per policy step (= STANDUP_HOLD in core/standup_policy.hpp)
NOBS = 6
CHECK_AFTER_CATCH = 1.0       # s of normal balancing that must follow a catch
SHAPE = 5.0                   # weight of the progress-towards-upright shaping
CATCH_ONLY = 2.0              # reached the catch but the balance controller lost it (scaled by how gently)
RATE_OK = 8.0                 # rad/s: pitch rate at the catch that still earns most of CATCH_ONLY
FALLEN_PENALTY = -2.0         # went over, ran away or gave up (was -5: too timid to explore)
PITCH_FULL = 1.8              # rad: lying flat, the real task
PITCH_START = 1.2             # rad: where the curriculum begins
PITCH_STEP = 0.05             # rad moved per adjustment
WINDOW = 20                   # episodes the curriculum signal is measured over
UP_OK, UP_BAD = 0.6, 0.3      # rate of that signal that makes the task harder / easier
GENTLE_OK = 0.5               # `gentle` (i.e. rate <= 6.7 rad/s) that counts as a catchable handover


class StandupEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, fixed_side=None, curriculum=True):
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (NOBS,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
        self.sim = None
        self.fixed_side = fixed_side
        self._obs = (C.c_float * NOBS)()
        # each worker carries its own curriculum: no plumbing through SubprocVecEnv
        self.curriculum = curriculum
        self.pitch = PITCH_START if curriculum else PITCH_FULL
        self.recent = collections.deque(maxlen=WINDOW)

    def _advance_curriculum(self, good):
        """Walk the start pitch towards PITCH_FULL while handovers are catchable, back off while they are not."""
        if not self.curriculum:
            return
        self.recent.append(bool(good))
        if len(self.recent) < WINDOW:
            return
        rate = sum(self.recent) / len(self.recent)
        if rate >= UP_OK and self.pitch < PITCH_FULL:
            self.pitch = min(PITCH_FULL, self.pitch + PITCH_STEP); self.recent.clear()
        elif rate <= UP_BAD and self.pitch > PITCH_START:
            self.pitch = max(PITCH_START, self.pitch - PITCH_STEP); self.recent.clear()

    def _obs_now(self):
        LIB.bot_standup_obs(self.sim.ctl, self._obs)
        return np.array(self._obs, dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.sim is not None: self.sim.close()
        sim_seed = int(self.np_random.integers(0, 2**31 - 1)) if (options or {}).get("sim_seed") is None else options["sim_seed"]
        side = (options or {}).get("side") or self.fixed_side or (1 if self.np_random.random() < 0.5 else -1)
        pitch = (options or {}).get("pitch") or self.pitch
        self.sim = BotSim(seed=sim_seed, randomize=True)
        self.sim.place(pitch=pitch * side)
        self.up0 = math.cos(pitch)
        LIB.bot_standup(self.sim.ctl, 2, 0.0)
        for _ in range(int(1.5 / CTRL_DT)):          # lying: the controller waits until it is still, then starts the push
            if self.sim.step().state == LIFT: break
        self.x0 = float(self.sim.d.qpos[0]); self.y0 = float(self.sim.d.qpos[1])
        self.prev_up = math.cos(self.sim.out.pitch)
        self.true_pitch = quat_pitch(self.sim.d.qpos[3:7])
        return self._obs_now(), {}

    def step(self, action):
        a = float(np.clip(action[0], -1.0, 1.0))
        LIB.bot_standup(self.sim.ctl, 2, a * 2 * 2.3)
        for _ in range(HOLD):
            prev_true = self.true_pitch
            o = self.sim.step()
            self.true_pitch = quat_pitch(self.sim.d.qpos[3:7])
            if o.state != LIFT: break
        true_rate = abs(self.true_pitch - prev_true) / CTRL_DT
        true_v = abs(float(o.v))                 # copy it NOW: see the note on `o` below
        obs = self._obs_now()
        # Shaping: progress towards upright, small costs for spinning the wheels fast and for big pushes.
        # It telescopes to SHAPE * (cos pitch_end - cos pitch_start), so SHAPE sets how much an episode can earn
        # by rising at all. At 2.0 that was +2.45 against a +10 all-or-nothing bonus the policy never reached;
        # 5.0 makes partial progress worth chasing without letting it beat an actual stand-up.
        up = math.cos(o.pitch)
        r = SHAPE * (up - self.prev_up) - 0.002 * a * a - 0.01 * float(obs[2]) ** 2
        self.prev_up = up
        if o.state == LIFT:
            return obs, r, False, False, {}
        if o.state == BALANCE:                       # caught: does the normal controller keep it?
            # `o` is BotSim.out itself, one ctypes struct the C++ refills in place on every step, not a
            # snapshot. Anything about the handover has to be copied before the check loop below steps the
            # sim on: reading o.pitch_rate after it reported 5.5 rad/s where the catch really saw 16.9.
            ok = True
            for _ in range(int(CHECK_AFTER_CATCH / CTRL_DT)):
                if self.sim.step().state != BALANCE: ok = False; break
            travel = math.hypot(self.sim.d.qpos[0] - self.x0, self.sim.d.qpos[1] - self.y0)
            ok = ok and abs(self.sim.out.pitch) < math.radians(12)
            rate = true_rate                        # how fast the body really was turning when the catch fired
            # Reaching the catch is most of the job, so credit it even when the balance controller drops it -
            # but only in proportion to how catchable the handover was. A slam through the window earns nothing.
            gentle = math.exp(-(rate / RATE_OK) ** 2)
            r += (10.0 - 2.0 * travel) if ok else CATCH_ONLY * gentle
            # Ratchet on a catchable handover, not on a full success: see the note at the top of the file.
            curric = bool(ok or gentle >= GENTLE_OK)
            self._advance_curriculum(curric)
            return obs, r, True, False, {"success": ok, "travel": travel, "catch_rate": rate,
                                         "catch_v": true_v, "start_pitch": self.pitch, "curric": curric}
        self._advance_curriculum(False)
        # went over, ran away or gave up
        return obs, r + FALLEN_PENALTY, True, False, {"success": False, "start_pitch": self.pitch,
                                                      "curric": False}

    def close(self):
        if self.sim is not None: self.sim.close(); self.sim = None
