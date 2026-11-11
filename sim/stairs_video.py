"""Ten episodes of the stair-measuring nod, rendered: what works, and every way it fails.

The point of the videos is that the failures are not subtle once you can see them. The lidar fan is
drawn, so you can watch the scan plane tip over and either catch the first tread or overfly it and land
two steps down - which is the difference between a 2 mm reading and a 500 mm one.

Every clip runs the real C++ controller through ml/stairs_env.py, so the simulator, the LD19 noise, the
attitude error and the estimator are the ones the numbers in ml/evaluate_stairs.py come from. Nothing
is staged: each caption is written from the episode's own info dict after it has run, so a clip labelled
a success really did measure the riser to within 20 mm with the wheels still on the landing.

  python sim/stairs_video.py                 # all ten, into site/files/
  python sim/stairs_video.py --only 3        # just one, while fiddling with the camera

Needs an ffmpeg: uses imageio_ffmpeg's bundled binary if it is installed, else one on PATH.
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
from botsim import CTRL_DT                                           # noqa: E402
from stairs_env import (StairsLookEnv, EDGE, NOD_LO, NOD_HI, V_MAX, D_ARM, D_FENCE,   # noqa: E402
                        MARGIN_OK, SWING_END_LO, SWING_END_HI)
from stairs_baseline import Scripted, Creep                         # noqa: E402
import stairs_env as ENV                                             # noqa: E402

_model_xml = ENV.model_xml


def see_through_walls(*a, **k):
    """The stairwell walls are opaque and the camera has to sit outside them. Alpha is a rendering
    property only - MuJoCo's rangefinders do not consult it - so the lidar still sees exactly what it
    saw in every number quoted here."""
    return _model_xml(*a, **k).replace('rgba="0.55 0.5 0.45 1"/>', 'rgba="0.55 0.5 0.45 0.10"/>')


ENV.model_xml = see_through_walls

OUT = ROOT / "site" / "files"
W, H, FPS = 960, 540, 30
INK, GREY, GOOD, BAD = (24, 26, 30), (110, 116, 124), (22, 120, 64), (176, 40, 40)


def ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return "ffmpeg"


# (slug, caption, riser m, tread m, start standoff m, overrides, wanted outcome). The overrides are the
# recipe's own four numbers, so a "too deep" clip is the same script with one number changed.
#
# `want` is honest about what these clips are: the scripted recipe succeeds on 37% of held-out
# staircases, so a clip that shows it working is a SELECTED episode. The environment is asked for seeds
# until the outcome matches, the seed goes in the JSON, and every caption is written from the episode's
# own info dict afterwards. The unbiased rate is the one in ml/evaluate_stairs.py, not the clip tally.
SCENES = [
    ("works-100", "The tuned recipe, 100 mm riser", 0.10, 0.30, 0.95, {}, "ok"),
    ("works-130", "130 mm riser, where the visibility band is widest", 0.13, 0.30, 1.05, {}, "ok"),
    ("works-080", "80 mm riser: a shallow step, visible from further out", 0.08, 0.30, 0.90, {}, "ok"),
    ("works-150", "150 mm riser", 0.15, 0.30, 1.00, {}, "ok"),
    ("tall-170", "170 mm riser, where the band narrows to one 3 degree cell per standoff",
     0.17, 0.30, 1.00, {}, "fail"),
    ("too-deep", "The deepest flick the action range allows, 0.85 rad",
     0.10, 0.30, 0.95, dict(pitch=0.95), "fail"),
    ("too-shallow", "A flick of 0.52 rad, below the angle that clears the step nose",
     0.10, 0.30, 0.95, dict(pitch=0.52), "fail"),
    ("too-far", "Flicking the moment the 720 mm arming gate allows it, instead of closing in",
     0.10, 0.30, 1.10, dict(d_dive=0.80, v_rev=0.10), "fail"),
    ("rolling-in", "Flicking while still rolling forwards, instead of reversing into it",
     0.10, 0.30, 0.95, dict(v_rev=-0.35), "fail"),
    ("narrow-tread", "A 240 mm tread: less room beyond the edge for the ray to land on",
     0.13, 0.24, 1.00, {}, "ok"),
    ("swing-480", "Creeping in and oscillating from 480 mm, the amplitude growing each swing",
     0.10, 0.30, 1.00, dict(swing=True, standoff=0.48, depth=0.95), "clean"),
    ("swing-150", "The same oscillation taken from 150 mm, as close as the wheels can safely get",
     0.10, 0.30, 1.00, dict(swing=True, standoff=0.15, depth=0.95), "fail"),
]


def policy(over):
    """Either recipe, selected by the scene: the one-sided flick, or the creep-and-oscillate."""
    kw = {k: v for k, v in over.items() if k != "swing"}
    return Creep(**kw) if over.get("swing") else Scripted(**kw)


def dry(scene, seed):
    """Run the episode with no rendering, to find a seed whose outcome is the one being illustrated."""
    slug, caption, rise, tread, d0, over, want = scene
    env = StairsLookEnv(swing=bool(over.get("swing")))
    env.reset(seed=seed)
    obs, _ = env.reset(options=dict(rise=rise, tread=tread, d0=d0))
    pol = policy(over)
    pol.reset()
    while True:
        obs, r, done, _, info = env.step(pol.act(obs))
        if done:
            break
    env.close()
    return info


def find_seed(scene, tries=40):
    """`want`: "ok" any success, "clean" a success whose reading really came off the first tread,
    "fail" anything that did not work. A clip is never relabelled to match - if no seed gives the
    outcome the scene is meant to illustrate, the scene is dropped instead."""
    want = scene[6]
    for k in range(tries):
        seed = 31_000 + 7 * k
        ok, line = verdict(dry(scene, seed))
        hit = {"ok": ok, "clean": ok and "LUCKY" not in line, "fail": not ok}[want]
        if hit:
            return seed, k + 1
    return None, tries


def render(scene, seed, font, small, tiny):
    slug, caption, rise, tread, d0, over, want = scene
    env = StairsLookEnv(swing=bool(over.get("swing")))
    env.reset(seed=seed)
    obs, _ = env.reset(options=dict(rise=rise, tread=tread, d0=d0))
    pol = policy(over)
    pol.reset()
    sim = env.sim
    rend = mujoco.Renderer(sim.m, H, W)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 1     # draw the fan: it is the whole story
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 1.55, -13, 116
    frames, stride = [], int(round(1 / FPS / CTRL_DT))
    look = np.array([float(sim.d.qpos[0]) + 0.16, 0.0, 0.05])
    state = dict(k=0, nod=None)
    orig = sim.step

    def step(push=None):
        o = orig(push)
        state["k"] += 1
        if state["k"] % stride:
            return o
        x = float(sim.d.qpos[0])
        look[:] = 0.90 * look + 0.10 * np.array([x + 0.16, 0.0, 0.05])
        cam.lookat[:] = look
        rend.update_scene(sim.d, cam, opt)
        im = Image.fromarray(rend.render())
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, W, 92], fill=(255, 255, 255))
        dr.text((18, 10), caption, fill=INK, font=font)
        d_true = EDGE - x
        dr.text((18, 50), f"riser {rise*1000:.0f} mm   tread {tread*1000:.0f} mm   "
                          f"standoff {d_true*1000:+.0f} mm   pitch {math.degrees(o.pitch):+5.1f}deg   "
                          f"speed {o.v:+5.2f} m/s   t {env.t + state['k']*CTRL_DT:4.1f} s",
                fill=GREY, font=small)
        if state["nod"] is not None:
            word = "swing to" if env.swing else "flick:"
            dr.text((18, 72), f"{word} {state['nod']:.2f} rad, started at {state['nod_d']*1000:.0f} mm",
                    fill=GREY, font=tiny)
        frames.append(np.asarray(im))
        return o

    sim.step = step
    while True:
        a = pol.act(obs)
        if a[1] > 0 and state["nod"] is None and env.d_est() <= D_ARM:
            lo, hi = (SWING_END_LO, SWING_END_HI) if env.swing else (NOD_LO, NOD_HI)
            state["nod"] = float(lo + 0.5 * (float(np.clip(a[2], -1, 1)) + 1) * (hi - lo))
            state["nod_d"] = EDGE - float(sim.d.qpos[0])
        obs, r, done, _, info = env.step(a)
        if done:
            break
    env.close()
    return frames, info, state


SEEN_REAL = 20      # returns on the true first tread below which a right answer is not a measurement


def verdict(info):
    """What actually happened, read off the episode rather than assumed.

    `seen` is how many returns landed on the real first tread. It is reported because the success test
    is on the NUMBER, and the number can come out right without the scan ever reaching the tread - the
    estimator takes the first surface below the landing, and on some staircases something else sits at
    roughly the right depth. Two of these ten clips are right that way, and saying so is the point.
    """
    seen = info.get("n_seen", 0)
    if info["fell"] or info["off_edge"]:
        where = "over the edge and down the stairs" if info["off_edge"] else "fell over"
        return False, f"FAILED - {where}"
    if not info["measured"]:
        return False, f"FAILED - no reading at all ({seen} returns on the first tread)"
    err = info["rise_err_mm"]
    if info["success"] and seen < SEEN_REAL:
        return True, (f"SUCCESS, BUT LUCKY - {info['rise_est_mm']:.0f} mm against "
                      f"{info['rise_true_mm']:.0f} ({err:+.0f} mm) off only {seen} returns on the tread: "
                      f"the right number from the wrong surface")
    if info["success"]:
        return True, (f"SUCCESS - riser {info['rise_est_mm']:.0f} mm against {info['rise_true_mm']:.0f} "
                      f"({err:+.0f} mm) off {seen} returns, {info['margin_mm']:.0f} mm of landing left")
    if err > 40:
        return False, (f"FAILED - read {info['rise_est_mm']:.0f} mm against {info['rise_true_mm']:.0f} "
                       f"({err:+.0f} mm) off {seen} returns: it measured a surface further down")
    return False, (f"FAILED - riser within {err:.0f} mm but only {info['margin_mm']:.0f} mm of "
                   f"landing left, inside the {MARGIN_OK*1000:.0f} mm a success needs")


def write(slug, frames, info, state, font, small):
    ok, line = verdict(info)
    tail = []
    for f in frames[-1:]:
        im = Image.fromarray(f.copy())
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, H - 64, W, H], fill=(255, 255, 255))
        dr.text((18, H - 54), line, fill=GOOD if ok else BAD, font=small)
        tail += [np.asarray(im)] * int(FPS * 1.6)
    seq = frames + tail
    raw = OUT / f".{slug}.raw"
    with open(raw, "wb") as fh:
        for f in seq:
            fh.write(np.ascontiguousarray(f).tobytes())
    mp4 = OUT / f"stairs_{slug}.mp4"
    subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", f"{W}x{H}", "-framerate", str(FPS), "-i", str(raw),
                    "-c:v", "libx264", "-preset", "slow", "-crf", "30", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(mp4)], check=True)
    raw.unlink()
    Image.fromarray(frames[len(frames) * 2 // 3]).save(OUT / f"stairs_{slug}.jpg", quality=82)
    return dict(slug=slug, file=mp4.name, ok=ok, line=line, kb=mp4.stat().st_size // 1024,
                seconds=round(len(seq) / FPS, 1),
                nod_target=state["nod"], nod_at_mm=round(state.get("nod_d", float("nan")) * 1000)
                if state["nod"] is not None else None,
                **{k: (round(v, 1) if isinstance(v, float) else v) for k, v in info.items()})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    font = ImageFont.load_default(size=27)
    small = ImageFont.load_default(size=19)
    tiny = ImageFont.load_default(size=17)
    rows = []
    for i, scene in enumerate(SCENES):
        if a.only is not None and a.only != i:
            continue
        seed, tried = find_seed(scene)
        if seed is None:
            print(f"{i} {scene[0]}: no seed in 40 gave a {scene[6]!r} outcome - the clip would be a lie, skipped")
            continue
        frames, info, state = render(scene, seed, font, small, tiny)
        row = write(scene[0], frames, info, state, font, small)
        row["caption"] = scene[1]
        row["seed"] = seed
        row["seeds_tried"] = tried
        rows.append(row)
        print(f"{i} {row['file']:<34} {row['seconds']:4.1f}s {row['kb']:5d} kB  seed {seed} "
              f"(tried {tried})  {row['line']}")
    if a.only is None:
        (OUT / "stairs_clips.json").write_text(json.dumps(rows, indent=1))
        n = sum(1 for r in rows if r["ok"])
        print(f"\n{n} of {len(rows)} succeeded; wrote {OUT / 'stairs_clips.json'}")


if __name__ == "__main__":
    main()
