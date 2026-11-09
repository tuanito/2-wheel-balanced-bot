"""The Pi-side robot: ties together the ESP32 link, lidar (optional: V2 has none and maps with the sonar), mapping,
obstacle avoidance, exploration and the eyes.
Runs a 20 Hz decision loop in a thread; the web server (server.py) only sets intents and reads state."""
import math, re, threading, time, pathlib
import numpy as np
from .link import Link, MODE_VELOCITY, MODE_TURN_BY, MODE_TURN_TO, MODE_FLIP, MODE_DOCK, MODE_UNDOCK
from . import dock as dockmod
from .ld19 import LD19, level_scan
from .mapper import Mapper
from .navigation import Avoider, FrontierExplorer, Planner, PathFollower

LEVEL_MAX = 0.09      # rad: only add scans to the map while the body (and the lidar plane) tilts less than ~5°
SONAR_LEVEL_MAX = 0.06  # rad: same for sonar readings (V2)


class Brain:
    def __init__(self, cfg):
        self.cfg = cfg
        self.data = pathlib.Path(cfg.get("data_dir", "data")); self.data.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.link = Link(cfg.get("esp32_port", "/dev/serial0"), on_event=self._event)
        self.lidar = None
        if cfg.get("lidar", True):
            self.lidar = LD19(cfg.get("lidar_port", "/dev/ttyUSB0"), offset_deg=cfg.get("lidar_offset_deg", 0.0),
                              clockwise=cfg.get("lidar_clockwise", True))
        self.mapper = self._new_mapper()
        self.avoider, self.explorer = Avoider(), FrontierExplorer(speed=cfg.get("explore_speed", 0.4))
        self.planner, self.follower = Planner(), PathFollower(speed=cfg.get("goto_speed", 0.4))
        self.eyes = None
        if cfg.get("eyes", True):
            from .eyes import Eyes
            self.eyes = Eyes()
        from .headlight import Headlight
        self.headlight = Headlight(cfg.get("headlight_pwmchip", "/sys/class/pwm/pwmchip0"))
        # intents (set by the web app)
        self.mode = "manual"                     # manual | explore | goto | turn
        self.joy = (0.0, 0.0); self.joy_t = 0.0
        self.arm_req = self.disarm_req = False
        self.auto_standup, self.avoid_on = True, True
        self.turn_angle, self.turn_t0 = 0.0, 0.0
        self.goto_xy = None
        self.agent_cmd, self.agent_until, self.flip_t0 = (0.0, 0.0), 0.0, 0.0
        self.voice = None                        # VoiceAgent, attached by __main__ when voice is enabled
        self.vision = None                       # Vision (AI Camera detections + face ID), attached by __main__
        self.tracker = None                      # follow mode: the person being followed
        self.cmd = (0.0, 0.0)
        self.dock_pose = dockmod.load_dock(self.data / "dock.json")   # map pose of the robot when docked
        self.pilot, self.dock_t0, self.dock_out = None, 0.0, (MODE_VELOCITY, 0.0, 0.0)
        self._auto_dock_t = 0.0
        self._docked_t = None                    # when the last docking finished (re-dock if the contacts open soon after)
        self.scan_pts = np.zeros((0, 2))
        self._last_scan_t = 0.0
        self._last_save = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    def _event(self, txt):
        self.events = (self.events + [time.strftime("%H:%M:%S ") + txt])[-30:]
        print("[esp32]", txt)

    # ---------------- intents ----------------
    def joystick(self, v, w):
        self.joy, self.joy_t = (float(v), float(w)), time.time()
        if abs(v) > 0.02 or abs(w) > 0.05: self.mode = "manual"

    def command(self, name, **kw):
        if name == "arm": self.arm_req = True
        elif name == "disarm": self.disarm_req = True; self.mode = "manual"
        elif name == "stop": self.mode = "manual"; self.joy = (0.0, 0.0)
        elif name == "explore": self.mode = "explore"; self.explorer = FrontierExplorer(speed=self.cfg.get("explore_speed", 0.4))
        elif name == "turn": self.mode, self.turn_angle, self.turn_t0 = "turn", math.radians(float(kw.get("angle", 90))), time.time(); self.link.seq += 1
        elif name == "flip": self.mode, self.flip_t0 = "flip", time.time(); self.link.seq += 1
        elif name == "goto":
            self.goto_xy = (float(kw["x"]), float(kw["y"]))
            path = self.planner.search(self.mapper.grid, self.mapper.pose[:2], goal_xy=self.goto_xy)
            if path: self.follower.path, self.mode = path, "goto"
            else: self._event("no path to that point (unknown or blocked)")
        elif name == "auto_standup": self.auto_standup = bool(kw.get("on", True))
        elif name == "avoid": self.avoid_on = bool(kw.get("on", True))
        elif name == "light":
            on = bool(kw.get("on", False))
            self.headlight.set(float(kw.get("level", 1.0)) if on else 0.0)      # the COB discs
            if self.eyes: self.eyes.light = on                                  # and the eyes' white night light
        elif name == "follow":
            from .vision import Tracker
            self.tracker, self.mode = Tracker(name=kw.get("name") or None), "follow"
        elif name == "dock": self._start_dock()
        elif name == "undock":
            self._docked_t = None
            self.mode, self.dock_t0 = "undock", time.time(); self.link.seq += 1
        elif name == "save_map": self.save()
        elif name == "clear_map": self.mapper = self._new_mapper()

    # ---------------- docking ----------------
    def _start_dock(self):
        """Known dock and far from it: plan a path to its approach point first. Then the pilot: turn and listen for
        the IR beacon / look for the tape, home on the beacon, line up on the axis, reverse in (the first dock saves its
        pose for next time)."""
        self.pilot = None
        if self.link.telem.charging: self._event("already on the charger"); self.mode = "manual"; return
        if self.dock_pose is not None:
            ax, ay, ah = dockmod.approach_pose(self.dock_pose)
            px, py = self.mapper.pose[:2]
            if math.hypot(ax - px, ay - py) > 0.6:
                path = self.planner.search(self.mapper.grid, (px, py), goal_xy=(ax, ay))
                if path:
                    self.follower.path, self.mode = path, "dock_goto"; return
                self._event("no path to the dock: homing on its beacon from here")
        self.mode, self.pilot, self.dock_t0 = "dock", dockmod.DockPilot(), time.time()
        self.dock_out = (MODE_VELOCITY, 0.0, 0.0)

    def _dock_tick(self, now, tel, on_wheels, pose, new_scan, lv):
        """-> (mode, v, w, angle) for the docking modes."""
        if self.mode == "dock_goto":
            if not on_wheels: return MODE_VELOCITY, 0.0, 0.0, 0.0
            v, w, done = self.follower.command(pose)
            if done: self.mode, self.pilot = "dock", dockmod.DockPilot()
            return MODE_VELOCITY, v, w, 0.0
        if self.mode == "dock":
            if new_scan:
                scan = (lv.angles, lv.ranges, lv.intensity) if lv is not None else None
                m, v, w, lean = self.pilot.step(now, scan, (tel.x, tel.y, tel.yaw), tel.state_name, tel.charging,
                                                tel.pitch, getattr(tel, "ir", 0))
                if lean: self.link.seq += 1
                self.dock_out = (MODE_DOCK if m == "dock" else MODE_VELOCITY, v, w)
                if m == "done":
                    self.dock_pose = tuple(float(p) for p in pose)
                    dockmod.save_dock(self.data / "dock.json", self.dock_pose)
                    self._event(self.pilot.msg); self.mode = "manual"; self._docked_t = now
                elif m == "failed":
                    self._event(self.pilot.msg); self.mode = "manual"
                elif self.pilot.msg and self.pilot.msg != getattr(self, "_last_dock_msg", ""):
                    self._event(self.pilot.msg)
                self._last_dock_msg = self.pilot.msg
            m, v, w = self.dock_out
            return m, v, w, 0.0
        if self.mode == "undock":                            # stand up off the dock, then drive 0.3 m out
            t = now - self.dock_t0
            if tel.state_name in ("CHARGING", "UNDOCK") and t < 3.0: return MODE_UNDOCK, 0.0, 0.0, 0.0
            if tel.state_name == "BALANCE" and t < 5.0:
                return MODE_VELOCITY, (0.15 if t > 2.0 else 0.0), 0.0, 0.0
            self.mode = "manual"
        return MODE_VELOCITY, 0.0, 0.0, 0.0

    def _new_mapper(self):
        return Mapper(folder=str(self.data), match=self.lidar is not None,
                      log_scans=self.cfg.get("log_scans", True) and self.lidar is not None)

    def agent_drive(self, v, w, seconds):
        """Velocity command from the voice agent for a limited time (obstacle avoidance still applies)."""
        self.agent_cmd, self.agent_until = (float(v), float(w)), time.time() + float(seconds)
        self.mode = "agent" if seconds > 0 else "manual"

    def set_eyes(self, shape, color=None, seconds=5.0):
        if not self.eyes: return
        c = None
        if color and re.fullmatch(r"#?[0-9a-fA-F]{6}", color):
            h = color.lstrip("#"); c = tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))
        self.eyes.override(shape, c, seconds)

    def save(self):
        self._last_save = time.time()
        return self.mapper.save()

    # ---------------- 20 Hz loop ----------------
    def _loop(self):
        while True:
            t0 = time.time()
            try:
                self._tick(t0)
            except Exception as e:          # never let the loop die: stop driving and report
                self.cmd = (0.0, 0.0); self._event(f"brain error: {e!r}")
            time.sleep(max(0.0, 0.05 - (time.time() - t0)))

    def _tick(self, now):
        tel = self.link.telem
        fresh = now - tel.rx_time < 0.3
        odom = (tel.x, tel.y, tel.yaw)
        on_wheels = fresh and tel.state_name == "BALANCE"
        scan = self.lidar.latest if self.lidar else None
        new_scan, lv = False, None
        if self.lidar is None:
            # V2: no lidar. Map with the sonar while nearly upright (a leaning robot points the beam at the floor)
            if fresh and tel.rx_time != self._last_scan_t:
                self._last_scan_t = tel.rx_time
                self.mapper.update_sonar(odom, tel.sonar, moving_ok=on_wheels and abs(tel.pitch) < SONAR_LEVEL_MAX)
                self.scan_pts = self.mapper.nearby_obstacles()
        elif scan is not None and scan.t != self._last_scan_t:
            self._last_scan_t = scan.t
            lv = level_scan(scan, tel.pitch, tel.roll); new_scan = True
            level = abs(tel.pitch) < LEVEL_MAX and abs(tel.roll) < LEVEL_MAX
            self.mapper.update(odom, lv, moving_ok=on_wheels and level)
            self.scan_pts = lv.points()
        pose = self.mapper.pose
        mode = MODE_VELOCITY; angle = 0.0
        v = w = 0.0
        if self.mode == "manual":
            if now - self.joy_t < 0.5: v, w = self.joy          # joystick watchdog
        elif self.mode == "explore" and on_wheels:
            v, w, _ = self.explorer.command(now, pose, self.mapper.grid)
        elif self.mode == "goto" and on_wheels:
            v, w, done = self.follower.command(pose)
            if done: self.mode = "manual"; self._event("arrived")
        elif self.mode == "follow" and on_wheels:
            from .vision import follow_cmd
            dets = self.vision.snapshot() if self.vision else []
            d = self.tracker.update(dets, now) if self.tracker else None
            v, w = follow_cmd(d.box if d else None)
            if d is None and self.tracker and not self.tracker.locked:
                w = 0.6                                                    # nobody in view: turn slowly to look for them
        elif self.mode == "agent":
            if now < self.agent_until: v, w = self.agent_cmd
            else: self.mode = "manual"
        elif self.mode == "flip":
            mode = MODE_FLIP
            if now - self.flip_t0 > 5.0: self.mode = "manual"
        elif self.mode == "turn":
            mode, angle = MODE_TURN_BY, self.turn_angle
            if now - self.turn_t0 > 2.0 + abs(self.turn_angle) / 1.0: self.mode = "manual"
        elif self.mode.startswith("dock") or self.mode == "undock":
            mode, v, w, angle = self._dock_tick(now, tel, on_wheels, pose, new_scan, lv)
        # contacts opened again soon after docking (a rod on the edge of its contact): stand-up happened, dock again
        if (self._docked_t is not None and self.mode == "manual" and fresh and not tel.charging
                and tel.state_name == "BALANCE" and now - self._docked_t < 60):
            self._docked_t = None; self._event("charge contacts opened: docking again"); self._start_dock()
        # flat battery: the ESP32 cuts the power in 20 s -> halt the Pi cleanly first (SD card safety)
        if fresh and tel.power_off_pending and not getattr(self, "_halting", False):
            self._halting = True; self._event("battery empty: shutting down")
            if self.cfg.get("halt_on_power_off", True):
                import subprocess
                try: subprocess.Popen(["sudo", "-n", "shutdown", "-h", "now"])
                except Exception as e: self._event(f"halt failed: {e}")
        # low battery: go and charge by itself (once; only when it knows where the dock is)
        if (tel.battery_low and fresh and self.dock_pose is not None and self.cfg.get("auto_dock", True)
                and self.mode in ("manual", "explore") and not tel.charging and now - self._auto_dock_t > 120):
            self._auto_dock_t = now; self._event("battery low: going to the charger"); self._start_dock()
        if self.avoid_on and mode == MODE_VELOCITY and (self.mode not in ("dock", "undock") or
                                                              self.pilot is not None and self.pilot.phase == "home") and (abs(v) > 0.01 or abs(w) > 0.01):
            v, w, _ = self.avoider.safe_command(v, w, self.scan_pts, tel.sonar)
        self.cmd = (v, w)
        self.link.send(v=v, yaw_rate=w, mode=mode, angle=angle, arm=self.arm_req, disarm=self.disarm_req,
                       auto_standup=self.auto_standup, obstacle_stop=True)
        if self.arm_req and tel.armed: self.arm_req = False
        if self.disarm_req and not tel.armed: self.disarm_req = False
        if self.eyes: self.eyes.set_from_telemetry(tel, w)
        if now - self._last_save > 15.0: self.save()

    def state(self):
        t = self.link.telem
        return dict(state=t.state_name, side=t.side, armed=t.armed, charging=t.charging, obstacle=t.obstacle,
                    battery_low=t.battery_low, motor_fault=t.motor_fault, tries=t.tries,
                    pitch=round(math.degrees(t.pitch), 1), roll=round(math.degrees(t.roll), 1),
                    yaw=round(math.degrees(t.yaw), 1), v=round(t.v, 2), vbat=round(t.vbat, 2), sonar=round(t.sonar, 2),
                    motor_temp=t.motor_temp, link_ok=time.time() - t.rx_time < 0.5,
                    pose=[round(p, 3) for p in self.mapper.pose], mode=self.mode, cmd=[round(c, 2) for c in self.cmd],
                    auto_standup=self.auto_standup, avoid=self.avoid_on, events=self.events[-5:],
                    voice=self.voice.status() if self.voice else None,
                    seen=[d.as_dict() for d in self.vision.snapshot()] if self.vision else [],
                    following=(self.tracker.name or "nearest person") if self.mode == "follow" and self.tracker else None,
                    dock_known=self.dock_pose is not None)
