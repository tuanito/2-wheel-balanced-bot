"""Voice: microphone processing on synthetic audio, wake phrase, stop fast-path, and the Claude tool loop with a fake
API client (no network, no sound card)."""
import math, struct, sys, time, types, pathlib, threading
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi"))
from robot import audio as A
from robot.voice import VoiceAgent


def speechlike(n, fs, rng):
    """Noise shaped into syllables (4 Hz envelope) in the 300-3000 Hz band."""
    x = rng.standard_normal(n)
    X = np.fft.rfft(x); f = np.fft.rfftfreq(n, 1 / fs); X[(f < 300) | (f > 3000)] = 0
    x = np.fft.irfft(X, n)
    t = np.arange(n) / fs
    return x * (0.6 + 0.4 * np.sin(2 * np.pi * 4 * t)) / np.std(x)


def test_direction_of_a_talker_on_the_left():
    rng = np.random.default_rng(1)
    fs = A.FS; n = 4096
    s = speechlike(n + 10, fs, rng)
    true_deg = 40.0
    tau = A.MIC_SPACING * math.sin(math.radians(true_deg)) / A.C_SOUND     # left mic hears it first
    d = tau * fs                                                            # ~1.6 samples
    k = np.fft.rfftfreq(n + 10, 1 / fs)
    right = np.fft.irfft(np.fft.rfft(s) * np.exp(-2j * np.pi * k * tau), n + 10)
    left = s
    est = A.direction_deg(A.gcc_phat(left[:n], right[:n]))
    assert abs(est - true_deg) < 8, est


def test_beam_adds_signal_and_averages_noise():
    rng = np.random.default_rng(2)
    n = 16000; s = speechlike(n, A.FS, rng) * 0.1
    nl, nr = rng.standard_normal(n) * 0.05, rng.standard_normal(n) * 0.05
    out = A.beam(s + nl, s + nr, 0.0)
    snr_in = 10 * math.log10(np.var(s) / np.var(nl))
    snr_out = 10 * math.log10(np.var(s) / np.var(out - s))
    assert snr_out > snr_in + 2.0                                           # ~ +3 dB for 2 mics


def test_speech_gate_finds_one_utterance_at_two_metres():
    """Speech at 2 m is ~ -66 dBFS; room noise ~ -80 dBFS. The gate must cut out exactly one utterance."""
    rng = np.random.default_rng(3)
    fs = A.FS
    noise = lambda n: rng.standard_normal(n) * 10 ** (-80 / 20)
    speech = speechlike(int(1.5 * fs), fs, rng) * 10 ** (-66 / 20)
    sig = np.concatenate([noise(2 * fs), speech + noise(len(speech)), noise(2 * fs)])
    g = A.SpeechGate()
    outs = [u for i in range(0, len(sig), 1600) if (u := g.feed(sig[i:i + 1600])) is not None]
    assert len(outs) == 1
    assert 1.3 < len(outs[0]) / fs < 2.8


def test_frontend_48k_stereo_end_to_end():
    rng = np.random.default_rng(4)
    n = 4800 * 4
    s = speechlike(n, A.FS_HW, rng) * 0.01
    fe = A.Frontend()
    for i in range(0, n, 4800):
        mono, b = fe(np.c_[s[i:i + 4800], s[i:i + 4800]])
    assert len(mono) == 1600 and abs(b) < 10


# ------------------------------------------------------------------ agent with fakes
class FakeAudio:
    def __init__(self): self.listening, self.level_db, self.bearing, self.on_utterance, self.played = True, -60.0, 0.0, None, []
    def play(self, x, fs): self.played.append(len(x))


class FakeBrain:
    def __init__(self):
        self.calls, self.mode = [], "manual"
    def command(self, name, **kw): self.calls.append((name, kw)); self.mode = "goto" if name == "goto" else self.mode
    def agent_drive(self, v, w, s): self.calls.append(("drive", (round(v, 3), round(w, 3), s)))
    def set_eyes(self, *a): self.calls.append(("eyes", a))
    def state(self): return dict(state="BALANCE", vbat=24.1, charging=False, v=0.0, pose=[0, 0, 0], sonar=1.2,
                                 obstacle=False, mode=self.mode, pitch=0.1, yaw=0.0)


class Block(types.SimpleNamespace): pass


class FakeClient:
    """Plays a scripted conversation: first asks for tools, then answers."""
    def __init__(self, script):
        self.script, self.requests = list(script), []
        self.messages = self
    def create(self, **kw):
        self.requests.append(kw)
        return self.script.pop(0)


def tool(name, inp, i): return Block(type="tool_use", name=name, input=inp, id=f"t{i}")
def text(t): return Block(type="text", text=t)


class FakeCam:
    class output:
        @staticmethod
        def wait(c, timeout=1.0): return b"\xff\xd8fakejpeg\xff\xd9", 1


def make_agent(script, **cfg):
    spoken = []
    tts = lambda t: (spoken.append(t) or np.zeros(160, np.float32), 16000)
    client = FakeClient(script)
    ag = VoiceAgent(dict(wake_phrase="hey robot|robot", **cfg), FakeBrain(), FakeCam(), audio=FakeAudio(),
                    stt=lambda pcm: pcm, tts=tts, client=client)
    ag._wait = lambda s: True                                              # don't really wait in tests
    return ag, client, spoken


def test_claude_drives_looks_and_answers():
    script = [Block(stop_reason="tool_use", content=[text("Okay."), tool("drive", {"speed_mps": 2.0, "turn_rate_dps": 30, "seconds": 9}, 1),
                                                     tool("look", {}, 2)]),
              Block(stop_reason="end_turn", content=[text("I drove forward and I see a fake picture.")])]
    ag, client, spoken = make_agent(script)
    reply = ag.handle_text("drive forward and tell me what you see")
    assert reply.startswith("I drove forward")
    drive = [c for c in ag.brain.calls if c[0] == "drive"][0][1]
    assert drive[0] == 0.6 and abs(drive[1] - math.radians(30)) < 1e-3 and drive[2] == 6.0     # clamped
    msgs = client.requests[1]["messages"]
    results = next(m["content"] for m in msgs if m["role"] == "user" and isinstance(m["content"], list))
    assert results[0]["type"] == "tool_result" and "done" in results[0]["content"]
    assert results[1]["content"][0]["type"] == "image"                                         # camera went to Claude
    assert spoken[-1] == reply
    assert client.requests[0]["model"] == "claude-haiku-5-5" and client.requests[0]["tools"]


def test_wake_phrase_and_stop_fast_path():
    script = [Block(stop_reason="end_turn", content=[text("Turning left.")])]
    ag, client, spoken = make_agent(script, face_speaker=True)
    ag.on_utterance("what a nice day", bearing=0)          # not for the robot
    assert client.requests == [] and ag.brain.calls == []
    ag.on_utterance("Hey robot, turn left", bearing=45)
    for _ in range(50):
        if client.requests: break
        time.sleep(0.01)
    time.sleep(0.05)
    assert client.requests[0]["messages"][0]["content"] == "turn left"
    assert ("turn", {"angle": 45}) in ag.brain.calls       # turned toward the talker
    ag.on_utterance("Stop!", bearing=0)
    assert ag.brain.calls[-1] == ("stop", {}) and spoken[-1] == "Stopping."


def test_wake_word_alone_then_command():
    script = [Block(stop_reason="end_turn", content=[text("Hello!")])]
    ag, client, spoken = make_agent(script, face_speaker=False)
    ag.on_utterance("robot", bearing=0)
    assert spoken[-1] == "Yes?" and client.requests == []
    ag.on_utterance("say hello", bearing=0)
    for _ in range(50):
        if client.requests: break
        time.sleep(0.01)
    assert client.requests[0]["messages"][0]["content"] == "say hello"


def test_history_trim_never_starts_with_tool_result():
    h = [{"role": "user", "content": "a"}, {"role": "assistant", "content": [1]}, {"role": "user", "content": [{"type": "tool_result"}]},
         {"role": "assistant", "content": [2]}] * 5
    t = VoiceAgent._trim(h, keep=7)
    assert t[0]["role"] == "user" and isinstance(t[0]["content"], str)


def test_flip_command_reaches_the_esp32(tmp_path):
    from robot import link as L
    from robot.brain import Brain
    from test_robot import FakeSerial
    esp = FakeSerial()
    brain = Brain(dict(esp32_port=esp, lidar=False, eyes=False, data_dir=str(tmp_path), log_scans=False))
    payload = struct.pack(L.TELEM_FMT, 1, 4, 0, 1, 0, *([0.0] * 12), 0)
    esp.inject(L.frame(L.MSG_TELEM, payload))
    time.sleep(0.1); esp.tx.clear()
    brain.command("flip"); time.sleep(0.2)
    cmds = [struct.unpack(L.CMD_FMT, p) for t, p in L.Parser().feed(bytes(esp.tx)) if t == L.MSG_CMD]
    assert cmds and cmds[-1][0] == L.MODE_FLIP
