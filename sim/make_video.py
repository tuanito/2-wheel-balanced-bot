"""Renders the mission (stand up from the back, stand still, drive, stop, turns, arc) + a fall/re-stand, and plots."""
import os, math, json, pathlib, subprocess
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
from botsim import BotSim, STATES, CTRL_DT

OUT = pathlib.Path(__file__).parent / "results"; OUT.mkdir(exist_ok=True)
W, H, FPS = 960, 540, 30


def script(s):
    yield "Lying on its back -> detects side, stands up", 3.0, dict()
    yield "Stand still", 2.0, dict(mode=0, v=0.0, yaw_rate=0.0)
    yield "Accelerate to 1 m/s", 3.0, dict(v=1.0)
    yield "Stop", 3.0, dict(v=0.0)
    yield "Turn on the spot +90°", 3.0, dict(mode=2, angle=math.radians(90), seq=1)
    yield "Turn +180°", 4.0, dict(mode=2, angle=math.radians(180), seq=2)
    yield "Turn to heading 30°", 3.5, dict(mode=1, angle=math.radians(30))
    yield "Arc: 0.5 m/s, 1 rad/s", 3.0, dict(mode=0, v=0.5, yaw_rate=1.0)
    yield "Stop, stand still", 2.0, dict(v=0.0, yaw_rate=0.0)
    yield "Knocked over (2 N·s shove)", 0.1, dict(push=40.0)
    yield "Falls on its face -> stands up again", 4.0, dict()


def main():
    s = BotSim(seed=0)
    s.place(pitch=-1.8)
    r = mujoco.Renderer(s.m, H, W)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 0.75, -16, 140
    font = ImageFont.load_default(size=26); small = ImageFont.load_default(size=20)
    frames_dir = OUT / "frames"; frames_dir.mkdir(exist_ok=True)
    for f in frames_dir.glob("*.png"): f.unlink()
    n = 0; steps_per_frame = int(round(1 / FPS / CTRL_DT)); k = 0
    look = np.array([0.0, 0.0, 0.0])
    for title, secs, cmd in script(s):
        push = cmd.pop("push", None)
        for key, v in cmd.items(): setattr(s.cmd, key, v)
        for _ in range(int(round(secs / CTRL_DT))):
            o = s.step(push=np.array([push, 0, 0]) if push else None)
            k += 1
            if k % steps_per_frame: continue
            look = 0.92 * look + 0.08 * np.array([s.d.qpos[0], s.d.qpos[1], 0.0])
            cam.lookat[:] = look + [0, 0, 0.02]
            r.update_scene(s.d, cam)
            im = Image.fromarray(r.render()); dr = ImageDraw.Draw(im)
            dr.rectangle([0, 0, W, 74], fill=(255, 255, 255))
            dr.text((16, 8), title, fill=(20, 20, 20), font=font)
            dr.text((16, 44), f"state {STATES[o.state]}   pitch {math.degrees(o.pitch):+6.1f}°   speed {o.v:+5.2f} m/s   "
                    f"heading {math.degrees(math.remainder(o.yaw, 2 * math.pi)):+6.1f}°   t {s.t:5.1f} s", fill=(70, 70, 70), font=small)
            im.save(frames_dir / f"{n:05d}.png"); n += 1
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(frames_dir / "%05d.png"),
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "26", "-pix_fmt", "yuv420p", str(OUT / "robot_c_mission.mp4")], check=True)
    for f in frames_dir.glob("*.png"): f.unlink()
    frames_dir.rmdir()
    L = np.array(s.log)
    np.save(OUT / "video_log.npy", L)
    print("frames", n)
    plot(L)


def plot(L):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = L[:, 0]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, np.degrees(L[:, 2]), lw=1, label="true pitch"); ax[0].plot(t, np.degrees(L[:, 3]), lw=1, ls="--", label="estimated")
    ax[0].set_ylabel("pitch (°)"); ax[0].legend(loc="upper right"); ax[0].set_ylim(-110, 110)
    ax[1].plot(t, L[:, 5], lw=1, label="speed"); ax[1].plot(t, L[:, 6], lw=1, label="speed reference"); ax[1].set_ylabel("m/s"); ax[1].legend(loc="upper right")
    ax[2].plot(t, np.degrees(L[:, 4]), lw=1, label="heading (estimated)"); ax[2].set_ylabel("heading (°)"); ax[2].legend(loc="upper right")
    ax[3].plot(t, L[:, 7], lw=0.8, label="left"); ax[3].plot(t, L[:, 8], lw=0.8, label="right"); ax[3].set_ylabel("torque (N·m)"); ax[3].legend(loc="upper right")
    ax[3].set_xlabel("time (s)")
    for a in ax: a.grid(alpha=0.3)
    fig.suptitle("Shape-C robot in MuJoCo with the firmware controller (core/controller.cpp)")
    fig.tight_layout(); fig.savefig(OUT / "robot_c_mission.png", dpi=110)


if __name__ == "__main__":
    main()
