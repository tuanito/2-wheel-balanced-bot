"""Five stand-up clips, rendered with whichever motor the library and the plant are built for.

The overlay carries the two numbers this manoeuvre turned out to be about: the wheel speed RELATIVE
TO THE BODY, which is what the motor's back-EMF and the runaway guard both see, and that guard's
threshold. The kick saturates the motor on purpose - median 79% of the no-load ceiling on the
MF6015 and 95% on the MF7015 - which is why the guard had to be restricted to BALANCE. Watch the
bar: it is the whole story of this phase.

    ROBOT_MOTOR=mf7015 python standup_video.py                 # five clips into sim/results/
    ROBOT_MOTOR=mf7015 python standup_video.py --only 3        # just the third

Needs the library and the MuJoCo model built for the same motor; botsim.py refuses otherwise.
ffmpeg comes from imageio-ffmpeg if there is no system one (there is none on the owner's Mac).
"""
import argparse
import math
import os
import pathlib
import shutil
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np                                      # noqa: E402
import mujoco                                           # noqa: E402
from PIL import Image, ImageDraw                        # noqa: E402
from PIL import ImageFont                               # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from botsim import BotSim, LIB, STATES, CTRL_DT, LIB_MOTOR   # noqa: E402
import motor_profile as MP                                   # noqa: E402

OUT = HERE / "results"
W, H, FPS = 960, 540, 30
FACE_DOWN, BACK_DOWN = math.radians(100), math.radians(-97)


def ffmpeg():
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


# (file suffix, caption, how to set the robot up, how long to film)
CLIPS = [
    ("1-facedown", "Face-down to upright. Nominal robot.",
     dict(pitch=FACE_DOWN, seed=0, randomize=False, delay=None, push=None), 5.0),
    ("2-backdown", "Back-down to upright. Nominal robot.",
     dict(pitch=BACK_DOWN, seed=0, randomize=False, delay=None, push=None), 5.0),
    ("3-facedown-delay2", "Face-down to upright. Randomised robot, 2 control periods of CAN latency.",
     dict(pitch=FACE_DOWN, seed=3007, randomize=True, delay=2, push=None), 5.0),
    ("4-backdown-random", "Back-down to upright. Randomised robot (mass, IMU bias, torque gain).",
     dict(pitch=BACK_DOWN, seed=3019, randomize=True, delay=None, push=None), 5.0),
    ("5-knocked-over", "Balancing, shoved over at 2 N·s, stands up again.",
     dict(pitch=0.0, seed=3011, randomize=True, delay=None, push=2.0), 8.0),
]


def render_clip(tag, caption, setup, secs, guard, outdir):
    s = BotSim(seed=setup["seed"], randomize=setup["randomize"])
    if setup["delay"] is not None:
        s.delay = setup["delay"]
    s.place(pitch=setup["pitch"])
    s.hold(0.4)

    r = mujoco.Renderer(s.m, H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 0.95, -14, 135
    big = ImageFont.load_default(size=25)
    small = ImageFont.load_default(size=19)
    tiny = ImageFont.load_default(size=16)

    frames = outdir / f"frames_{tag}"
    if frames.exists():
        shutil.rmtree(frames)
    frames.mkdir(parents=True)

    look = np.array([float(s.d.qpos[0]), float(s.d.qpos[1]), 0.0])
    steps_per_frame = int(round(1 / FPS / CTRL_DT))
    n = k = 0
    pushed = setup["push"] is None
    w_peak = 0.0
    note = ""
    for i in range(int(secs / CTRL_DT)):
        push = None
        if not pushed and i > int(2.0 / CTRL_DT):       # let it settle, then shove
            push = np.array([setup["push"] / 0.1, 0.0, 0.0])
            if i > int(2.1 / CTRL_DT):
                pushed, push = True, None
        if setup["push"] is not None and i == int(0.4 / CTRL_DT):
            s.cmd.arm = 1
        if setup["push"] is None and i == int(0.2 / CTRL_DT):
            s.cmd.arm = 1
        o = s.step(push=push)
        inp = s.sense()
        w_rel = 0.5 * abs(inp.wl + inp.wr)
        w_peak = max(w_peak, w_rel)
        k += 1
        if k % steps_per_frame:
            continue
        look = 0.90 * look + 0.10 * np.array([float(s.d.qpos[0]), float(s.d.qpos[1]), 0.0])
        cam.lookat[:] = look + [0, 0, 0.04]
        r.update_scene(s.d, cam)
        im = Image.fromarray(r.render())
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, W, 92], fill=(255, 255, 255))
        d.text((16, 7), caption, fill=(20, 20, 20), font=big)
        d.text((16, 40), f"{MP.NAME}, torque limit {LIB.bot_motor_tau_max():.1f} N·m", fill=(70, 70, 70), font=small)
        d.text((16, 64), f"state {STATES[o.state]}    pitch {math.degrees(o.pitch):+6.1f}°    "
                         f"speed {o.v:+5.2f} m/s    t {s.t:4.1f} s", fill=(70, 70, 70), font=small)
        # The bar, bottom right on its own panel: wheel speed relative to the body against the
        # runaway guard. This is the number the stand-up turns on - the kick saturates the motor on
        # purpose, which is why the guard had to be restricted to BALANCE.
        bw, bh = 320, 18
        bx, by = W - bw - 20, H - 54
        d.rectangle([bx - 12, by - 26, W - 8, H - 8], fill=(255, 255, 255))
        full = max(guard * 1.15, w_peak * 1.05)
        d.text((bx, by - 22), "wheel speed relative to the body", fill=(70, 70, 70), font=tiny)
        d.rectangle([bx, by, bx + bw, by + bh], outline=(170, 170, 170))
        frac = min(w_rel / full, 1.0)
        over = w_rel > guard
        d.rectangle([bx + 1, by + 1, bx + 1 + int((bw - 2) * frac), by + bh - 1],
                    fill=(200, 60, 60) if over else (60, 130, 210))
        gx = bx + int(bw * guard / full)
        d.line([gx, by - 3, gx, by + bh + 3], fill=(200, 60, 60), width=2)
        d.text((bx, by + bh + 3), f"{w_rel:5.1f} rad/s    peak {w_peak:5.1f}    guard {guard:.0f}",
               fill=(200, 60, 60) if over else (70, 70, 70), font=tiny)
        if o.state == 4 and abs(o.pitch) < 0.15 and not note:
            note = f"caught at t {s.t:.2f} s"
        if note:
            d.text((16, H - 32), note, fill=(30, 120, 60), font=small)
        im.save(frames / f"{n:05d}.png")
        n += 1
    s.close()

    mp4 = outdir / f"standup_{LIB_MOTOR}_{tag}.mp4"
    subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-framerate", str(FPS),
                    "-i", str(frames / "%05d.png"), "-c:v", "libx264", "-preset", "veryfast",
                    "-crf", "25", "-pix_fmt", "yuv420p", str(mp4)], check=True)
    # a poster frame, for the site's <video poster=...>
    jpg = mp4.with_suffix(".jpg")
    mid = sorted(frames.glob("*.png"))[int(n * 0.62)]
    Image.open(mid).convert("RGB").save(jpg, quality=86)
    shutil.rmtree(frames)
    return mp4, jpg, n, w_peak


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, help="render just this clip (1-5)")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    outdir = pathlib.Path(a.out)
    outdir.mkdir(parents=True, exist_ok=True)
    guard = 0.95 * LIB.bot_motor_w_max()
    print(f"library and plant: {LIB_MOTOR} / {MP.PROFILE}   torque limit "
          f"{LIB.bot_motor_tau_max():.1f} N.m   runaway guard {guard:.0f} rad/s")
    for i, (tag, caption, setup, secs) in enumerate(CLIPS, 1):
        if a.only and i != a.only:
            continue
        mp4, jpg, n, w_peak = render_clip(tag, caption, setup, secs, guard, outdir)
        print(f"  {i}. {mp4.name}  {n} frames, {mp4.stat().st_size/1e6:.1f} MB, "
              f"peak wheel {w_peak:.1f} rad/s ({w_peak/guard*100:.0f}% of the guard)")


if __name__ == "__main__":
    main()
