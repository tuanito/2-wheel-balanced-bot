"""The two ways of taking the look, side by side on the same staircase, ten times.

The flick (reverse away from the edge, then one deep one-sided dive) and the swing (creep in, stop,
then oscillate the pitch target with the amplitude growing each cycle) score 0.390 and 0.290 over 200
held-out staircases. These clips are where that difference comes from, and unlike sim/stairs_video.py
**no episode here is selected**: the seeds are the first ten of the evaluation set, 20000..20009, the
staircase is whatever the environment drew for that seed, and both recipes get exactly the same one -
same riser, same tread, same start distance, same robot, same CAN latency, same sensor noise. Whatever
each one did is what the clip shows, so the tally across the ten is an unbiased (if small) sample of
the same number ml/evaluate_stairs.py reports.

  python sim/stairs_ab_video.py            # ten pairs into site/files/
  python sim/stairs_ab_video.py --only 0   # one pair, while fiddling with the camera

Panels are rendered one at a time into raw files under the scratch dir and composed by streaming, so
peak memory is one panel's worth of frames, not twenty.

Needs an ffmpeg: uses imageio_ffmpeg's bundled binary if installed, else one on PATH.
"""
import argparse, json, math, os, pathlib, shutil, subprocess, sys, tempfile, textwrap
os.environ.setdefault("MUJOCO_GL", "glfw")
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "ml"))
from botsim import CTRL_DT                                              # noqa: E402
# importing stairs_video is what makes the stairwell walls see-through (rendering only: MuJoCo's
# rangefinders do not consult rgba, so the lidar sees what it saw in every number quoted here)
from stairs_video import ffmpeg, verdict                                # noqa: E402
from stairs_env import (StairsLookEnv, EDGE, NOD_LO, NOD_HI, D_ARM,     # noqa: E402
                        SWING_END_LO, SWING_END_HI)
from stairs_baseline import Scripted, Creep                             # noqa: E402

OUT = ROOT / "site" / "files"
PW, PH, FPS, GAP = 720, 440, 30, 8                  # panel size; composed frame is 2*PW+GAP wide
TAIL = 2.2                                          # s of held last frame with the verdict on it
INK, GREY, GOOD, BAD = (24, 26, 30), (110, 116, 124), (22, 120, 64), (176, 40, 40)
SEEDS = list(range(20_000, 20_010))                 # the first ten of the held-out evaluation set

LOOKS = [("flick", False, "FLICK  -  reverse away, then one deep dive", Scripted),
         ("swing", True, "SWING  -  creep in, stop, then oscillate", Creep)]


def panel(look, seed, raw_path, fonts):
    """Render one recipe on the staircase `seed` draws, streaming frames to `raw_path`.

    Returns (frame count, info, nod state, staircase). The same seed gives the same staircase to both
    recipes because the environment draws riser, tread, start distance and robot in that order from a
    freshly seeded RNG; the staircase is returned so the caller can assert the pair really matched.
    """
    name, swing, title, Pol = look
    font, small, tiny = fonts
    env = StairsLookEnv(swing=swing)
    env.reset(seed=seed)
    obs, _ = env.reset()
    stair = (round(env.rise, 4), round(env.tread, 4), round(env.d0_true, 4))
    pol = Pol()
    pol.reset()
    sim = env.sim
    rend = mujoco.Renderer(sim.m, PH, PW)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 1       # draw the fan: it is the whole story
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 1.55, -13, 116
    look_at = np.array([float(sim.d.qpos[0]) + 0.16, 0.0, 0.05])
    state = dict(k=0, n=0, nod=None)
    orig, stride = sim.step, int(round(1 / FPS / CTRL_DT))
    fh = open(raw_path, "wb")

    def step(push=None):
        o = orig(push)
        state["k"] += 1
        if state["k"] % stride:
            return o
        x = float(sim.d.qpos[0])
        look_at[:] = 0.90 * look_at + 0.10 * np.array([x + 0.16, 0.0, 0.05])
        cam.lookat[:] = look_at
        rend.update_scene(sim.d, cam, opt)
        im = Image.fromarray(rend.render())
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, PW, 86], fill=(255, 255, 255))
        dr.text((16, 8), title, fill=INK, font=font)
        dr.text((16, 40), f"standoff {(EDGE - x)*1000:+.0f} mm   "
                          f"pitch {math.degrees(o.pitch):+5.1f}deg   speed {o.v:+5.2f} m/s   "
                          f"t {state['k']*CTRL_DT:4.1f} s", fill=GREY, font=small)
        if state["nod"] is not None:
            word = "swing to" if swing else "flick to"
            dr.text((16, 62), f"{word} {state['nod']:.2f} rad, started at {state['nod_d']*1000:.0f} mm",
                    fill=GREY, font=tiny)
        fh.write(np.ascontiguousarray(np.asarray(im)).tobytes())
        state["n"] += 1
        return o

    sim.step = step
    while True:
        a = pol.act(obs)
        if a[1] > 0 and state["nod"] is None and env.d_est() <= D_ARM:
            lo, hi = (SWING_END_LO, SWING_END_HI) if swing else (NOD_LO, NOD_HI)
            state["nod"] = float(lo + 0.5 * (float(np.clip(a[2], -1, 1)) + 1) * (hi - lo))
            state["nod_d"] = EDGE - float(sim.d.qpos[0])
        obs, r, done, _, info = env.step(a)
        if done:
            break
    fh.close()
    env.close()
    return state["n"], info, state, stair


def banded(frame, info, small):
    """The last frame with that episode's verdict written across the bottom of it."""
    ok, line = verdict(info)
    im = Image.fromarray(frame.copy())
    dr = ImageDraw.Draw(im)
    wrapped = textwrap.wrap(line, width=62)[:3]
    h = 16 + 20 * len(wrapped)
    dr.rectangle([0, PH - h, PW, PH], fill=(255, 255, 255))
    for i, s in enumerate(wrapped):
        dr.text((16, PH - h + 8 + 20 * i), s, fill=GOOD if ok else BAD, font=small)
    return np.asarray(im), ok, line


def compose(raws, counts, lasts, mp4, small):
    """Stream the two panels into one clip: pad the shorter with its last frame so they end together,
    then hold both verdicts for TAIL seconds."""
    n = max(counts)
    mm = [np.memmap(p, dtype=np.uint8, mode="r", shape=(c, PH, PW, 3)) for p, c in zip(raws, counts)]
    bands = [banded(l, i, small)[0] for l, i in lasts]
    div = np.full((PH, GAP, 3), 220, dtype=np.uint8)
    pipe = subprocess.Popen([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", f"{2*PW+GAP}x{PH}", "-framerate", str(FPS), "-i", "-",
                             "-c:v", "libx264", "-preset", "slow", "-crf", "30", "-pix_fmt", "yuv420p",
                             "-movflags", "+faststart", str(mp4)], stdin=subprocess.PIPE)
    mid = None
    for k in range(n + int(FPS * TAIL)):
        row = []
        for j in (0, 1):
            if k >= n:
                row.append(bands[j])
            else:
                row.append(mm[j][min(k, counts[j] - 1)])
        f = np.hstack([row[0], div, row[1]])
        if k == (2 * n) // 3:
            mid = f.copy()
        pipe.stdin.write(np.ascontiguousarray(f).tobytes())
    pipe.stdin.close()
    if pipe.wait():
        raise RuntimeError("ffmpeg failed")
    del mm
    return mid, n + int(FPS * TAIL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    fonts = (ImageFont.load_default(size=24), ImageFont.load_default(size=17),
             ImageFont.load_default(size=15))
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="stairs_ab_"))
    rows = []
    try:
        for i, seed in enumerate(SEEDS):
            if a.only is not None and a.only != i:
                continue
            raws, counts, lasts, infos, stairs, nods = [], [], [], [], [], []
            for look in LOOKS:
                raw = tmp / f"{look[0]}_{seed}.raw"
                n, info, state, stair = panel(look, seed, raw, fonts)
                mm = np.memmap(raw, dtype=np.uint8, mode="r", shape=(n, PH, PW, 3))
                lasts.append((np.array(mm[-1]), info))
                del mm
                raws.append(raw), counts.append(n), infos.append(info), stairs.append(stair), nods.append(state)
            assert stairs[0] == stairs[1], f"seed {seed} gave different staircases: {stairs}"
            slug = f"ab{i:02d}"
            mp4 = OUT / f"stairs_{slug}.mp4"
            mid, frames = compose(raws, counts, lasts, mp4, fonts[1])
            Image.fromarray(mid).save(OUT / f"stairs_{slug}.jpg", quality=80)
            for p in raws:
                p.unlink()
            row = dict(slug=slug, file=mp4.name, seed=seed, kb=mp4.stat().st_size // 1024,
                       seconds=round(frames / FPS, 1),
                       rise_mm=round(1000 * stairs[0][0]), tread_mm=round(1000 * stairs[0][1]),
                       start_mm=round(1000 * stairs[0][2]))
            for (name, _, _, _), info, state in zip(LOOKS, infos, nods):
                ok, line = verdict(info)
                row[name] = dict(
                    ok=ok, line=line, rise_est_mm=None if info["rise_est_mm"] != info["rise_est_mm"]
                    else round(info["rise_est_mm"]), err_mm=None if info["rise_err_mm"] != info["rise_err_mm"]
                    else round(info["rise_err_mm"]), n_seen=info["n_seen"],
                    margin_mm=round(info["margin_mm"]), peak_deg=round(info["peak_deg"], 1),
                    fell=bool(info["fell"]), off_edge=bool(info["off_edge"]),
                    nod_target=None if state["nod"] is None else round(state["nod"], 2),
                    nod_at_mm=None if state["nod"] is None else round(1000 * state["nod_d"]))
            rows.append(row)
            print(f"{i} {row['file']:<22} riser {row['rise_mm']:3d} tread {row['tread_mm']:3d} "
                  f"from {row['start_mm']:4d} mm  {row['kb']:5d} kB\n"
                  f"    flick  {row['flick']['line']}\n    swing  {row['swing']['line']}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if a.only is None:
        (OUT / "stairs_ab.json").write_text(json.dumps(rows, indent=1))
        f = sum(1 for r in rows if r["flick"]["ok"])
        s = sum(1 for r in rows if r["swing"]["ok"])
        print(f"\nflick {f}/{len(rows)}, swing {s}/{len(rows)} on the same ten staircases; "
              f"wrote {OUT / 'stairs_ab.json'}")


if __name__ == "__main__":
    main()
