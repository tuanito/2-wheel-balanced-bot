"""Ten clips of the robot taking a step down - the ones that work and the ones that do not.

Nothing here is staged. Each clip drives the robot off a real edge at a chosen speed, stops asking
for speed the moment the axle crosses, and then lets it land. The caption is written from the run's
own telemetry afterwards, and the overlay shows the three numbers that decide the outcome:

  |a|       the accelerometer magnitude. ~9.8 standing, ~0 in free fall, a spike on touchdown. This
            is the whole basis of the catch: the IMU can tell the robot it is falling, 6 ms in.
  AIRBORNE  the controller's own flag, not a guess from outside: while it is lit, the balance law is
            not servoing the wheels, only damping them.
  wheel v   why the catch has to exist. Unloaded wheels run from 0.5 to 5.8 m/s in 50 ms, and the
            reaction throws the body to 90 degrees before the drop is even over.

  python sim/descend_video.py            # all ten, into site/files/
  python sim/descend_video.py --only 3   # one, while fiddling with the camera

Needs an ffmpeg: uses imageio_ffmpeg's bundled binary if installed, else one on PATH.
"""
import argparse, json, math, os, pathlib, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "ml"))
from botsim import BotSim, CTRL_DT                        # noqa: E402
from stairs_probe import model_xml                        # noqa: E402
from stairs_video import ffmpeg                           # noqa: E402

OUT = ROOT / "site" / "files"
W, H, FPS = 960, 540, 30
EDGE = 1.2
INK, GREY, GOOD, BAD, HOT = (24, 26, 30), (110, 116, 124), (22, 120, 64), (176, 40, 40), (200, 90, 10)

# (slug, caption, riser, approach speed, catch on?, wanted outcome)
SCENES = [
    ("step-040-ok", "40 mm lip at 0.15 m/s: about the biggest step it can actually take",
     0.04, 0.15, True, "ok"),
    ("step-060-ok", "60 mm step at 0.15 m/s", 0.06, 0.15, True, "ok"),
    ("step-080-ok", "80 mm step at 0.15 m/s", 0.08, 0.15, True, "ok"),
    ("step-100-fail", "100 mm step at 0.25 m/s: nothing in the set survives this height",
     0.10, 0.25, True, "fail"),
    ("step-130-fail", "130 mm step at 0.25 m/s", 0.13, 0.25, True, "fail"),
    ("step-170-fail", "170 mm step at 0.25 m/s, the tallest riser in the set", 0.17, 0.25, True, "fail"),
    ("step-100-fast", "100 mm step at 0.45 m/s: too fast to land on", 0.10, 0.45, True, "fail"),
    ("nocatch-040", "40 mm lip, free-fall catch OFF: even a kerb loses it", 0.04, 0.15, False, "fail"),
    ("nocatch-100", "100 mm step, catch OFF: the wheels run away in mid-air", 0.10, 0.25, False, "fail"),
    ("catch-100", "100 mm step, catch ON: the wheels stay put, the landing still beats it",
     0.10, 0.15, True, None),
]


def run(rise, v_app, catch, seed, d0=0.45, post=1.8, collect=None):
    """One step down. `collect(sim, o, over, t)` is called every tick when rendering."""
    # the camera has to sit outside the stairwell, so the walls are made see-through. Alpha is a
    # rendering property only - MuJoCo's rangefinders ignore it - so nothing measured changes.
    xml = model_xml(rise, 0.30, 8, EDGE).replace('rgba="0.55 0.5 0.45 1"/>',
                                                 'rgba="0.55 0.5 0.45 0.10"/>')
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - d0)
    sim.hold(0.6)
    sim.cmd.mode = 0
    sim.cmd.obstacle_stop = 0
    sim.cmd.drop_stop = 0                      # the fence exists to prevent exactly this
    sim.cmd.fall_catch = 1 if catch else 0
    sim.cmd.v = v_app
    over, t_over, peak, air_ticks = False, 0, 0.0, 0
    n = int((d0 / max(0.05, v_app) + 3.0) / CTRL_DT)
    for i in range(n):
        o = sim.step()
        x = float(sim.d.qpos[0])
        if x > EDGE and not over:
            over, t_over = True, i
            sim.cmd.v = 0.0                    # one step only
        if over:
            peak = max(peak, abs(float(o.pitch)))
            air_ticks += int(o.airborne)
        if collect is not None:
            collect(sim, o, over, i)
        if over and i - t_over > int(post / CTRL_DT):
            break
    o = sim.out
    # peak pitch too: auto-stand-up means a tumble can still end upright, and that is not a step
    ok = (over and int(o.state) == 4 and abs(float(o.pitch)) < 0.25 and abs(float(o.v)) < 0.35
          and peak < 0.80)
    info = dict(ok=bool(ok), over=bool(over), rise_mm=rise * 1000, v_app=v_app, catch=bool(catch),
                peak_deg=math.degrees(peak), end_pitch_deg=math.degrees(float(o.pitch)),
                end_v=float(o.v), state=int(o.state), air_ms=air_ticks * CTRL_DT * 1000,
                past_mm=(float(sim.d.qpos[0]) - EDGE) * 1000)
    sim.close()
    return info


def verdict(i):
    if not i["over"]:
        return False, "FAILED - never reached the edge"
    if i["ok"]:
        return True, (f"SUCCESS - landed and balancing, peak pitch {i['peak_deg']:.0f} deg, "
                      f"{i['air_ms']:.0f} ms airborne, settled {i['past_mm']:.0f} mm past the edge")
    return False, (f"FAILED - peak pitch {i['peak_deg']:.0f} deg, ended at "
                   f"{i['end_pitch_deg']:+.0f} deg doing {i['end_v']:+.2f} m/s "
                   f"({i['air_ms']:.0f} ms airborne)")


def find_seed(sc, tries=80):
    slug, cap, rise, v, catch, want = sc
    for k in range(tries):
        seed = 4000 + 13 * k
        i = run(rise, v, True if catch == "any" else catch, seed)
        if want is None or (want == "ok") == bool(i["ok"]):
            return seed, i
    return None, None


def render(sc, seed, fonts):
    slug, cap, rise, v, catch, want = sc
    font, small, tiny = fonts
    frames, stride = [], int(round(1 / FPS / CTRL_DT))
    state = {}

    def collect(sim, o, over, i):
        if i % stride:
            return
        if "rend" not in state:
            state["rend"] = mujoco.Renderer(sim.m, H, W)
            state["opt"] = mujoco.MjvOption()
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.distance, cam.elevation, cam.azimuth = 1.50, -12, 116
            state["cam"] = cam
            state["look"] = np.array([float(sim.d.qpos[0]) + 0.14, 0.0, -0.04])
        x = float(sim.d.qpos[0])
        state["look"][:] = 0.88 * state["look"] + 0.12 * np.array([x + 0.14, 0.0, -0.06])
        state["cam"].lookat[:] = state["look"]
        state["rend"].update_scene(sim.d, state["cam"], state["opt"])
        im = Image.fromarray(state["rend"].render())
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, W, 92], fill=(255, 255, 255))
        dr.text((18, 10), cap, fill=INK, font=font)
        s, sd = sim.d.sensordata, sim.sid
        a = float(np.linalg.norm(s[sd["accel"]:sd["accel"] + 3]))
        dr.text((18, 48), f"|a| {a:5.1f} m/s2    pitch {math.degrees(o.pitch):+6.1f} deg    "
                          f"wheel v {o.v:+5.2f} m/s    {EDGE - x:+.3f} m to the edge",
                fill=GREY, font=small)
        if o.airborne:
            dr.rectangle([18, 70, 150, 88], fill=HOT)
            dr.text((26, 71), "AIRBORNE", fill=(255, 255, 255), font=tiny)
            dr.text((160, 71), "balance law held off; wheels damped only", fill=HOT, font=tiny)
        elif not catch:
            dr.text((18, 71), "catch OFF: the balance law keeps servoing in mid-air", fill=BAD, font=tiny)
        frames.append(np.asarray(im))

    info = run(rise, v, True if catch == "any" else catch, seed, collect=collect)
    return frames, info


def write(slug, frames, info, small):
    ok, line = verdict(info)
    tail = []
    im = Image.fromarray(frames[-1].copy())
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, H - 56, W, H], fill=(255, 255, 255))
    dr.text((18, H - 46), line, fill=GOOD if ok else BAD, font=small)
    tail = [np.asarray(im)] * int(FPS * 1.8)
    seq = frames + tail
    raw = OUT / f".{slug}.raw"
    with open(raw, "wb") as fh:
        for f in seq:
            fh.write(np.ascontiguousarray(f).tobytes())
    mp4 = OUT / f"descend_{slug}.mp4"
    subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", f"{W}x{H}", "-framerate", str(FPS), "-i", str(raw),
                    "-c:v", "libx264", "-preset", "slow", "-crf", "30", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(mp4)], check=True)
    raw.unlink()
    Image.fromarray(frames[len(frames) * 3 // 5]).save(OUT / f"descend_{slug}.jpg", quality=82)
    row = dict(info)                       # info already carries `ok`; do not pass it twice
    row.update(slug=slug, file=mp4.name, ok=ok, line=line, kb=mp4.stat().st_size // 1024,
               seconds=round(len(seq) / FPS, 1))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    fonts = (ImageFont.load_default(size=27), ImageFont.load_default(size=19),
             ImageFont.load_default(size=16))
    rows = []
    for i, sc in enumerate(SCENES):
        if a.only is not None and a.only != i:
            continue
        seed, _ = find_seed(sc)
        if seed is None:
            print(f"{i} {sc[0]}: no seed in 26 gave a {sc[5]!r} outcome - skipped rather than relabelled")
            continue
        frames, info = render(sc, seed, fonts)
        row = write(sc[0], frames, info, fonts[1])
        row["caption"], row["seed"] = sc[1], seed
        rows.append(row)
        print(f"{i} {row['file']:<28} {row['seconds']:4.1f}s {row['kb']:5d} kB  seed {seed}  {row['line']}")
    if a.only is None and rows:
        (OUT / "descend_clips.json").write_text(json.dumps(rows, indent=1))
        n = sum(1 for r in rows if r["ok"])
        print(f"\n{n} of {len(rows)} landed it; wrote {OUT / 'descend_clips.json'}")


if __name__ == "__main__":
    main()
