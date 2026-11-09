"""Gymnasium environment: learn the stand-up (lying -> upright) with the real controller in the loop.

The robot lies on its face or back (random side), the C++ controller detects the side, waits until it is still and
starts the push (state LIFT). From there the policy sets the push every 10 ms (`standup_mode = 2`: torque from
outside) until the controller's normal catch takes over (|pitch| < 20°) or it gives up. After the catch the episode
keeps running 1 s under the normal balance controller: success = still balancing, upright, without running off.

Every episode uses a randomised robot (mass, centre of mass, inertia, tyre grip, battery voltage, torque gain, CAN
delay, IMU bias and mount): the same randomisation as sim/run_tests.py, so the policy doesn't overfit one robot.

Observation (6 floats, side-normalised, computed by the controller itself: Controller::standup_obs):
  pitch, 0.1·pitch rate, wheel speed / max, time / timeout, (vbat - 23)/2, last action
Action: 1 float in [-1, 1] = push for both motors as a fraction of 2 × torque_limit (4.6 N·m).
"""
import sys, math, pathlib, ctypes as C
import numpy as np
import gymnasium as gym

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from botsim import BotSim, LIB, CTRL_DT  # noqa: E402

LIFT, BALANCE = 2, 4
HOLD = 5                      # control ticks per policy step (= STANDUP_HOLD in core/standup_policy.hpp)
NOBS = 6
CHECK_AFTER_CATCH = 1.0       # s of normal balancing that must follow a catch


class StandupEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, fixed_side=None):
        self.observation_space = gym.spaces.Box(-5.0, 5.0, (NOBS,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)
        self.sim = None
        self.fixed_side = fixed_side
        self._obs = (C.c_float * NOBS)()

    def _obs_now(self):
        LIB.bot_standup_obs(self.sim.ctl, self._obs)
        return np.array(self._obs, dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.sim is not None: self.sim.close()
        sim_seed = int(self.np_random.integers(0, 2**31 - 1)) if (options or {}).get("sim_seed") is None else options["sim_seed"]
        side = (options or {}).get("side") or self.fixed_side or (1 if self.np_random.random() < 0.5 else -1)
        self.sim = BotSim(seed=sim_seed, randomize=True)
        self.sim.place(pitch=1.8 * side)
        LIB.bot_standup(self.sim.ctl, 2, 0.0)
        for _ in range(int(1.5 / CTRL_DT)):          # lying: the controller waits until it is still, then starts the push
            if self.sim.step().state == LIFT: break
        self.x0 = float(self.sim.d.qpos[0]); self.y0 = float(self.sim.d.qpos[1])
        self.prev_up = math.cos(self.sim.out.pitch)
        return self._obs_now(), {}

    def step(self, action):
        a = float(np.clip(action[0], -1.0, 1.0))
        LIB.bot_standup(self.sim.ctl, 2, a * 2 * 2.3)
        for _ in range(HOLD):
            o = self.sim.step()
            if o.state != LIFT: break
        obs = self._obs_now()
        # shaping: progress towards upright, small costs for spinning the wheels fast and for big pushes
        up = math.cos(o.pitch)
        r = 2.0 * (up - self.prev_up) - 0.002 * a * a - 0.01 * float(obs[2]) ** 2
        self.prev_up = up
        if o.state == LIFT:
            return obs, r, False, False, {}
        if o.state == BALANCE:                       # caught: does the normal controller keep it?
            ok = True
            for _ in range(int(CHECK_AFTER_CATCH / CTRL_DT)):
                if self.sim.step().state != BALANCE: ok = False; break
            travel = math.hypot(self.sim.d.qpos[0] - self.x0, self.sim.d.qpos[1] - self.y0)
            ok = ok and abs(self.sim.out.pitch) < math.radians(12)
            r += (10.0 - 2.0 * travel) if ok else -3.0
            return obs, r, True, False, {"success": ok, "travel": travel}
        return obs, r - 5.0, True, False, {"success": False}       # FALLEN: went over, ran away or gave up

    def close(self):
        if self.sim is not None: self.sim.close(); self.sim = None
