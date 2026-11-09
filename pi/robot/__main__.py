"""Start everything:  python3 -m robot [--config config.json]"""
import argparse, json, pathlib
from aiohttp import web
from .brain import Brain
from .camera import Camera
from .server import make_app

DEFAULT = {
    "esp32_port": "/dev/serial0",     # Pi UART (GPIO14/15), enable it with raspi-config, disable the serial console
    "lidar": True,                    # V1 has the LD19 on top (False = map from the front sonar, as in V2)
    "lidar_port": "/dev/ttyUSB0",     # LD19 kit adapter board on the micro-USB OTG port
    "lidar_offset_deg": 0.0,          # robot-frame angle of the lidar's 0° mark: calibrate with a wall straight ahead
    "lidar_clockwise": True,
    "data_dir": "data",               # map.png, map.npz, scans/*.npz
    "log_scans": True,
    "explore_speed": 0.4,
    "goto_speed": 0.4,
    "eyes": True,
    "camera_size": [800, 600],
    "camera_fps": 20,
    "ai_camera": True,                # Raspberry Pi AI Camera: person / cat detection on the sensor
    "ai_model": "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk",
    "http_port": 8080,
    "voice": {                        # microphones + speaker + Claude (needs ANTHROPIC_API_KEY in the environment)
        "enabled": True,
        "wake_phrase": "hey robot|robot",   # alternatives separated by |
        "push_to_talk_only": False,         # True: only listen after the Talk button in the app
        "language": "en",                   # speech recognition / synthesis language
        "stt": "vosk",                      # "vosk" (offline) or "http" (Whisper server, e.g. on the Jetson)
        "vosk_model": "/home/pi/models/vosk-model-small-en-us-0.15",
        "stt_url": "",                      # e.g. "http://jetson.local:8000/v1/audio/transcriptions"
        "tts": "piper",                     # "piper" or "espeak"
        "piper_model": "/home/pi/models/en_US-lessac-low.onnx",
        "claude_model": "claude-haiku-5-5",
        "mic_gain_db": 0.0,
        "mic_spacing_m": 0.124,             # V1.1 front panel (V2: 0.086)
        "face_speaker": True,               # turn toward whoever says the wake phrase
        "audio_device": None,               # None = default ALSA card (the I2S voice card)
    },
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    a = ap.parse_args()
    cfg = dict(DEFAULT)
    p = pathlib.Path(a.config)
    if p.exists(): cfg.update(json.loads(p.read_text()))
    brain = Brain(cfg)
    cam = Camera(tuple(cfg["camera_size"]), cfg["camera_fps"], ai_model=cfg["ai_model"] if cfg.get("ai_camera") else None)
    from .vision import Vision
    brain.vision = Vision(cam, brain.data / "people")
    vcfg = dict(DEFAULT["voice"]); vcfg.update(cfg.get("voice") or {})
    if vcfg.get("enabled"):
        from .voice import VoiceAgent
        brain.voice = VoiceAgent(vcfg, brain, cam)
    web.run_app(make_app(brain, cam), port=cfg["http_port"])


if __name__ == "__main__":
    main()
