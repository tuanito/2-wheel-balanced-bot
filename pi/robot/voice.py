"""Talk to the robot: microphones -> speech-to-text -> Claude (with tools that move the robot) -> spoken reply.

  "Hey robot, drive forward a bit and turn left"  -> Claude calls drive(...) then turn(...), then answers.
  "Hey robot, what do you see?"                    -> Claude calls look() (camera JPEG) and describes it.
  "Stop!"                                          -> handled locally at once (no network round trip).

Wake phrase or push-to-talk (button in the phone app). The API key comes from the environment
(ANTHROPIC_API_KEY, e.g. /etc/robot.env loaded by robot.service), never from config.json.
Claude only gets these tools, with hard limits enforced here; obstacle avoidance and the ESP32's
sonar stop stay active whatever it asks.
"""
import base64, math, os, re, threading, time

SYSTEM = """You are the mind of a small two-wheeled self-balancing robot (27 cm wide, 17 cm tall) with LED eyes, a camera,
a front sonar, a microphone pair and a speaker. A person is talking to you; their words come from speech recognition,
so expect small transcription errors. Act on clear requests with the tools, then answer in one or two short spoken
sentences (no lists, no markdown, no emoji) in the language the person used. If a request is unclear or unsafe,
ask a short question instead of moving. Speeds: walking pace is 0.3 m/s; never exceed 0.6 m/s. Positive turn angles
are to the robot's left. The robot balances by itself; if it has fallen it stands up automatically. Map coordinates
are in metres from where the robot was switched on (x forward at start, y to the left)."""

TOOLS = [
    {"name": "drive", "description": "Drive for a limited time. Blocks until done. Obstacle avoidance stays on.",
     "input_schema": {"type": "object", "properties": {
         "speed_mps": {"type": "number", "description": "forward speed, -0.4 to 0.6 m/s (negative = backwards)"},
         "turn_rate_dps": {"type": "number", "description": "turning rate while driving, -70 to 70 deg/s (positive = left)"},
         "seconds": {"type": "number", "description": "duration, 0.2 to 6 s"}}, "required": ["speed_mps", "seconds"]}},
    {"name": "turn", "description": "Turn on the spot by an angle. Blocks until done.",
     "input_schema": {"type": "object", "properties": {"degrees": {"type": "number", "description": "-360..360, positive = left"}},
                      "required": ["degrees"]}},
    {"name": "stop", "description": "Stop moving and stand still (cancels exploring / going somewhere).",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "go_to", "description": "Plan a path on the map and drive to a point (returns at once; check status).",
     "input_schema": {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}}, "required": ["x", "y"]}},
    {"name": "explore", "description": "Start exploring and mapping the room on its own.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "flip", "description": "Do the flip trick (lies on its back, flips over, gets up). Needs 1 m of free floor. Takes ~3 s.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "set_eyes", "description": "Change the LED eyes.",
     "input_schema": {"type": "object", "properties": {
         "shape": {"type": "string", "enum": ["open", "happy", "half", "closed", "x"]},
         "color": {"type": "string", "description": "hex colour like #00aaff"},
         "seconds": {"type": "number", "description": "how long before going back to normal (default 5)"}}, "required": ["shape"]}},
    {"name": "headlight", "description": "Turn the eyes into a white headlight (on/off).",
     "input_schema": {"type": "object", "properties": {"on": {"type": "boolean"}}, "required": ["on"]}},
    {"name": "follow", "description": "Follow a person with the camera (keeps ~1.5 m away, avoids obstacles). Give a name to follow that enrolled person only; omit it for the nearest person. Stop with the stop tool.",
     "input_schema": {"type": "object", "properties": {"name": {"type": "string"}}}},
    {"name": "who_is_there", "description": "What the AI camera sees right now: people (with names when recognised) and cats, with their position in the image.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "look", "description": "Take a photo with the front camera and see it.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "dock", "description": "Go to the charging dock and charge (backs into it on its own; returns at once, check status: state CHARGING when docked).",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "undock", "description": "Leave the charging dock: stand up off it and drive 0.4 m forward.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "status", "description": "Battery, state, speed, position on the map, distance to the obstacle ahead.",
     "input_schema": {"type": "object", "properties": {}}},
]

STOP_WORDS = re.compile(r"^\W*(stop|halt|stopp|arrête|arrete|basta|freeze)\W*$", re.I)


def clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


class VoiceAgent:
    def __init__(self, cfg, brain, camera=None, audio=None, stt=None, tts=None, client=None):
        self.cfg, self.brain, self.camera = cfg, brain, camera
        self.wake = [w.strip().lower() for w in cfg.get("wake_phrase", "hey robot|robot").split("|") if w.strip()]
        self.model = cfg.get("claude_model", "claude-haiku-5-5")
        self.face_speaker = cfg.get("face_speaker", True)
        self.push_to_talk_only = cfg.get("push_to_talk_only", False)
        self.state, self.heard, self.reply, self.error = "idle", "", "", ""
        self.history, self.last_t = [], 0.0
        self.armed_until = 0.0                    # next utterance is a command (after the wake word or the app button)
        self.abort = threading.Event()
        self.busy = threading.Lock()
        self.client = client
        if self.client is None:
            try:
                import anthropic
                if os.environ.get("ANTHROPIC_API_KEY"): self.client = anthropic.Anthropic()
                else: self.error = "ANTHROPIC_API_KEY not set"
            except Exception as e:
                self.error = f"anthropic SDK missing: {e}"
        if stt is None or tts is None:
            from .speech import STT, TTS
            stt = stt or STT(cfg); tts = tts or TTS(cfg)
        self.stt, self.tts = stt, tts
        if audio is None:
            from .audio import Audio
            audio = Audio(device=cfg.get("audio_device"), gain_db=cfg.get("mic_gain_db", 0.0), on_utterance=self.on_utterance,
                          spacing=cfg.get("mic_spacing_m", 0.124))
        else:
            audio.on_utterance = self.on_utterance
        self.audio = audio
        self.audio.listening = not self.push_to_talk_only

    # ------------------------------------------------------------------ listening
    def push_to_talk(self):
        """App button: the next thing said is a command (no wake word needed)."""
        self.armed_until = time.time() + 8.0
        self.audio.listening = True
        self.state = "listening"

    def on_utterance(self, pcm, bearing=0.0):
        text = self.stt(pcm)
        if not text: return
        now = time.time()
        if STOP_WORDS.match(text):                         # local fast path, also interrupts a running action
            self.abort.set(); self.brain.command("stop"); self.heard = text; self.say("Stopping."); return
        low = text.lower()
        cmd = None
        if now < self.armed_until:
            cmd = text
        else:
            for w in self.wake:
                m = re.match(r"^\W*" + re.escape(w) + r"\b[\s,.!?]*", low)
                if m: cmd = text[m.end():].strip(); break
            if cmd is None: return                         # not for us
            if self.face_speaker and abs(bearing) > 20: self.brain.command("turn", angle=bearing)
            if not cmd:
                self.armed_until = now + 8.0; self.state = "listening"; self.say("Yes?"); return
        self.armed_until = 0.0
        if self.push_to_talk_only: self.audio.listening = False
        self.heard = text
        threading.Thread(target=self.handle_text, args=(cmd,), daemon=True).start()

    # ------------------------------------------------------------------ thinking
    def handle_text(self, text):
        """One request: Claude, tools, spoken answer. Returns the reply text."""
        if not self.busy.acquire(blocking=False):
            self.say("One moment, I'm still busy."); return ""
        try:
            self.abort.clear(); self.state = "thinking"; self.error = ""
            if time.time() - self.last_t > 300: self.history = []          # new conversation after 5 min
            self.last_t = time.time()
            if self.client is None:
                self.reply = "I can't reach Claude: " + (self.error or "no client")
                self.say(self.reply); return self.reply
            self.history.append({"role": "user", "content": text})
            reply = ""
            for _ in range(8):                                             # tool-use loop
                r = self.client.messages.create(model=self.model, max_tokens=600, system=SYSTEM, tools=TOOLS,
                                                messages=self.history)
                self.history.append({"role": "assistant", "content": r.content})
                uses = [b for b in r.content if getattr(b, "type", "") == "tool_use"]
                reply = " ".join(b.text for b in r.content if getattr(b, "type", "") == "text").strip()
                if r.stop_reason != "tool_use" or not uses: break
                self.state = "acting"
                results = []
                for u in uses:
                    try: out = self.run_tool(u.name, dict(u.input or {}))
                    except Exception as e: out = f"error: {e}"
                    results.append({"type": "tool_result", "tool_use_id": u.id, "content": out})
                self.history.append({"role": "user", "content": results})
            self.history = self._trim(self.history)
            self.reply = reply
            self.say(reply)
            return reply
        except Exception as e:
            self.error = str(e); self.reply = "Sorry, I couldn't reach my brain."
            self.say(self.reply); return self.reply
        finally:
            self.state = "idle"; self.busy.release()

    @staticmethod
    def _trim(h, keep=12):
        """Keep the last turns, starting at a plain user message (never orphan a tool_result)."""
        if len(h) <= keep: return h
        h = h[-keep:]
        while h and not (h[0]["role"] == "user" and isinstance(h[0]["content"], str)): h = h[1:]
        return h

    def say(self, text):
        if not text: return
        prev = self.state; self.state = "speaking"
        x, fs = self.tts(text)
        if len(x): self.audio.play(x, fs)
        self.state = prev if prev != "speaking" else "idle"

    # ------------------------------------------------------------------ tools
    def _wait(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            if self.abort.is_set(): return False
            time.sleep(0.05)
        return True

    def run_tool(self, name, a):
        b = self.brain
        if name == "drive":
            v = clamp(a.get("speed_mps", 0), -0.4, 0.6)
            w = math.radians(clamp(a.get("turn_rate_dps", 0), -70, 70))
            secs = clamp(a.get("seconds", 1), 0.2, 6.0)
            b.agent_drive(v, w, secs)
            done = self._wait(secs)
            b.agent_drive(0.0, 0.0, 0.0)
            s = b.state()
            return f"{'done' if done else 'interrupted'}; sonar {s['sonar']} m{'; obstacle stopped me' if s['obstacle'] else ''}"
        if name == "turn":
            deg = clamp(a.get("degrees", 0), -360, 360)
            b.command("turn", angle=deg)
            self._wait(1.0 + abs(deg) / 60.0)
            return f"turned, heading now {b.state()['yaw']} deg"
        if name == "stop":
            b.command("stop"); return "stopped"
        if name == "go_to":
            b.command("goto", x=float(a["x"]), y=float(a["y"]))
            return "on my way" if b.mode == "goto" else "no path to that point"
        if name == "explore":
            b.command("explore"); return "exploring"
        if name == "flip":
            b.command("flip"); self._wait(4.0); return f"flip done, state {b.state()['state']}"
        if name == "set_eyes":
            b.set_eyes(a.get("shape", "open"), a.get("color"), clamp(a.get("seconds", 5), 0.5, 60)); return "ok"
        if name == "headlight":
            b.command("light", on=bool(a.get("on"))); return "ok"
        if name == "follow":
            b.command("follow", name=a.get("name")); return f"following {a.get('name') or 'the nearest person'}"
        if name == "who_is_there":
            seen = b.vision.snapshot(1.0) if b.vision else []
            if not seen: return "nobody (no person or cat in view)"
            def where(box):
                cx = 0.5 * (box[0] + box[2]); return "left" if cx < 0.38 else "right" if cx > 0.62 else "ahead"
            return "; ".join(f"{d.name or ('a ' + d.label if d.label == 'cat' else 'an unknown person')} {where(d.box)}"
                             f"{'' if d.label == 'cat' else f', box height {d.box[3] - d.box[1]:.2f}'}" for d in seen)
        if name == "look":
            frame = None
            if self.camera is not None and getattr(self.camera, "output", None) is not None:
                frame, _ = self.camera.output.wait(-1, timeout=1.0)
            if not frame: return "camera not available"
            return [{"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                  "data": base64.b64encode(frame).decode()}}]
        if name == "dock":
            b.command("dock")
            return ("already charging" if b.state()["charging"] else
                    "going to the dock" if b.mode.startswith("dock") else "can't dock now")
        if name == "undock":
            if not b.state()["charging"]: return "not on the dock"
            b.command("undock"); self._wait(5.0); return f"undocked, state {b.state()['state']}"
        if name == "status":
            s = b.state()
            keys = ("state", "vbat", "charging", "v", "pose", "sonar", "obstacle", "mode", "pitch", "yaw", "dock_known")
            return ", ".join(f"{k}={s.get(k)}" for k in keys)
        return f"unknown tool {name}"

    def status(self):
        return dict(state=self.state, heard=self.heard, reply=self.reply, error=self.error,
                    level=round(self.audio.level_db, 1), bearing=round(self.audio.bearing))
