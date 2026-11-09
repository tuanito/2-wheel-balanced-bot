"""Speech-to-text and text-to-speech backends (all optional, chosen in config.json "voice").

STT
  "vosk"  offline on the Pi Zero 2 W (small models, ~50 MB, English/German/French... ). Good enough for commands.
  "http"  POST the utterance as WAV to an OpenAI-compatible /v1/audio/transcriptions endpoint, e.g. faster-whisper
          running on the Jetson Nano on your LAN (much better accuracy, ~0.5-1 s for a short command).
TTS
  "piper"  natural neural voices, runs on the Pi Zero 2 W (use an x_low/low voice for speed).
  "espeak" espeak-ng: robotic but instant, no model download.
"""
import io, json, shutil, subprocess, wave
import numpy as np
from .audio import FS, to_wav_bytes


class STT:
    def __init__(self, cfg):
        self.kind = cfg.get("stt", "vosk")
        self.lang = cfg.get("language", "en")
        self.url = cfg.get("stt_url", "")
        self._model = None
        if self.kind == "vosk":
            try:
                import vosk
                vosk.SetLogLevel(-1)
                self._model = vosk.Model(cfg.get("vosk_model", "models/vosk-model-small-en-us-0.15"))
                self._vosk = vosk
            except Exception as e:
                print("vosk unavailable:", e)

    def __call__(self, pcm16k):
        if self.kind == "http" and self.url:
            import urllib.request, uuid
            boundary = uuid.uuid4().hex
            body = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-1\r\n"
                    f"--{boundary}\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\n{self.lang}\r\n"
                    f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"u.wav\"\r\n"
                    f"Content-Type: audio/wav\r\n\r\n").encode() + to_wav_bytes(pcm16k) + f"\r\n--{boundary}--\r\n".encode()
            req = urllib.request.Request(self.url, data=body, headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
            with urllib.request.urlopen(req, timeout=15) as r:
                return json.loads(r.read()).get("text", "").strip()
        if self._model is not None:
            rec = self._vosk.KaldiRecognizer(self._model, FS)
            rec.AcceptWaveform((np.clip(pcm16k, -1, 1) * 32767).astype("<i2").tobytes())
            return json.loads(rec.FinalResult()).get("text", "").strip()
        return ""


class TTS:
    def __init__(self, cfg):
        self.kind = cfg.get("tts", "piper")
        self.piper_model = cfg.get("piper_model", "models/en_US-lessac-low.onnx")
        self.lang = cfg.get("language", "en")
        if self.kind == "piper" and not shutil.which("piper"):
            print("piper not found, using espeak-ng"); self.kind = "espeak"

    def __call__(self, text):
        """-> (mono float32 samples, sample rate)."""
        if not text: return np.zeros(0, np.float32), FS
        try:
            if self.kind == "piper":
                p = subprocess.run(["piper", "--model", self.piper_model, "--output_raw"], input=text.encode(),
                                   capture_output=True, timeout=30, check=True)
                cfgp = self.piper_model + ".json"
                try: fs = json.load(open(cfgp))["audio"]["sample_rate"]
                except Exception: fs = 16000
                return np.frombuffer(p.stdout, "<i2").astype(np.float32) / 32768, fs
            p = subprocess.run(["espeak-ng", "-v", self.lang, "-s", "160", "--stdout", text], capture_output=True,
                               timeout=30, check=True)
            with wave.open(io.BytesIO(p.stdout)) as w:
                fs = w.getframerate(); x = np.frombuffer(w.readframes(w.getnframes()), "<i2").astype(np.float32) / 32768
            return x, fs
        except Exception as e:
            print("tts failed:", e)
            return np.zeros(0, np.float32), FS
