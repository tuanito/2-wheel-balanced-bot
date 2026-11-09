"""Video: the robot finds the charger and docks from 5 starts round it (real C++ controller + real Pi DockPilot,
IR beacon model, tape-intensity lidar). Writes results/dock_5starts.mp4 and results/dock_5starts.png (top view)."""
import os, math, pathlib, subprocess, sys
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
import dock_sim as D
from botsim import STATES, CTRL_DT

OUT = D.OUT
W, H, FPS = 960, 540, 30
OPT = mujoco.MjvOption(); OPT.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0
STARTS = [   # (title, x, y, heading deg) in the docked frame: dock face at x = -0.074, its axis along +x
    ("35 cm in front, facing the dock", 0.30, 0.04, 175),
    ("40 cm off to the side, sideways", 0.12, 0.40, -90),
    ("1.6 m out at an angle, facing away", 1.15, -1.10, -40),
    ("2 m behind the dock", -2.0, 0.40, 60),
    ("3 m to the side, IR beacon only", 0.40, 3.00, 0),
]
SPEED = {"find": 4, "home": 4, "lineup": 3, "reverse": 1, "lean": 1, "done": 1, "failed": 1}


def run(idx, title, x0, y0, h0, r, cam, frames_dir, n0, font, small, track):
    s = D.DockSim(seed=200 + idx, randomize=True); s.seed_ = 200 + idx
    s.place(x=x0, y=y0, yaw=math.radians(h0)); s.hold(0.5); s.run(2.0)
    pilot = D.DockPilot(); n = n0; k = 0; t0 = s.t; ir = 0; done_t = None
    look = np.array([(x0 - 0.074) / 2, y0 / 2, 0.0])
    while s.t < t0 + 150:
        if k % 50 == 0:
            scan = s.scan(with_intensity=True); o = s.out; ir = D._ir_bits(s)
            if done_t is None:
                mode, v, w, lean = pilot.step(s.t, scan, (o.x, o.y, o.yaw), STATES[o.state], s.charge, o.pitch, ir)
                s.cmd.mode = D.MODE[mode]; s.cmd.v = v; s.cmd.yaw_rate = w
                if lean: s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF
                if mode in ("done", "failed"): done_t = s.t
            track.append((float(s.d.qpos[0]), float(s.d.qpos[1])))
        o = s.step(); k += 1
        if done_t is not None and s.t > done_t + 3.0: break
        stride = int(round(SPEED.get(pilot.phase, 1) / FPS / CTRL_DT))
        if k % stride: continue
        rx, ry = s.d.qpos[0], s.d.qpos[1]
        sep = math.hypot(rx + 0.074, ry)
        want = np.array([0.6 * rx + 0.4 * -0.074, 0.6 * ry, 0.05])
        look = 0.9 * look + 0.1 * want
        cam.lookat[:] = look
        cam.distance = 0.9 * cam.distance + 0.1 * (0.55 + 0.75 * sep)
        r.update_scene(s.d, cam, OPT)
        im = Image.fromarray(r.render()); dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, W, 78], fill=(255, 255, 255))
        dr.text((16, 6), f"Start {idx + 1}/5: {title}", fill=(20, 20, 20), font=font)
        heard = "".join(n_ if ir >> (2 * i) & 1 else "·" for i, n_ in enumerate(("FL ", "FR ", "BL ", "BR ")))
        on_axis = "  on axis" if ir & 0xAA else ""
        sp = SPEED.get(pilot.phase, 1)
        dr.text((16, 44), f"{pilot.phase:8s} {STATES[o.state]:10s} pitch {math.degrees(o.pitch):+5.1f}°  IR {heard}{on_axis}"
                f"  {'charging' if s.charge else ''}  t {s.t - t0:5.1f} s  {'x' + str(sp) if sp > 1 else ''}",
                fill=(70, 70, 70), font=small)
        if done_t is not None:
            msg = "DOCKED - CHARGING, motors off" if pilot.phase == "done" else pilot.msg
            dr.rectangle([W // 2 - 230, H - 70, W // 2 + 230, H - 24], fill=(30, 140, 60) if pilot.phase == "done" else (180, 50, 40))
            dr.text((W // 2 - 210, H - 62), msg, fill=(255, 255, 255), font=font)
        im.save(frames_dir / f"{n:05d}.png"); n += 1
    res = dict(title=title, ok=pilot.phase == "done", time=round((done_t or s.t) - t0, 1), tries=pilot.tries, msg=pilot.msg)
    # 1 s title card gap
    s.close()
    return n, res


def main():
    D.build_xml()
    s0 = D.DockSim(seed=0)
    r = mujoco.Renderer(s0.m, H, W)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 1.2, -28, 215
    s0.close()
    font = ImageFont.load_default(size=26); small = ImageFont.load_default(size=19)
    frames_dir = OUT / "dock_frames"; frames_dir.mkdir(exist_ok=True)
    for f in frames_dir.glob("*.png"): f.unlink()
    n = 0; results = []; tracks = []
    sel = [int(a) for a in sys.argv[1:]] or range(len(STARTS))
    for i in sel:
        title, x, y, h = STARTS[i]
        tr = []
        n, res = run(i, title, x, y, h, r, cam, frames_dir, n, font, small, tr)
        results.append(res); tracks.append((title, x, y, h, tr, res["ok"]))
        print(res, n, flush=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(frames_dir / "%05d.png"),
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "26", "-pix_fmt", "yuv420p", str(OUT / "dock_5starts.mp4")], check=True)
    for f in frames_dir.glob("*.png"): f.unlink()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.add_patch(plt.Rectangle((-0.04, -0.074 - 0.04), 0.08, 0.04, color="0.3"))
    for k, (title, x, y, h, tr, ok) in enumerate(tracks):
        tr = np.array(tr)
        ax.plot(tr[:, 1], tr[:, 0], lw=1.5, label=f"{k + 1}: {title} ({'docked' if ok else 'missed'})")
        ax.annotate(str(k + 1), (y, x), fontsize=12, weight="bold")
        ax.arrow(y, x, 0.15 * math.sin(math.radians(h)), 0.15 * math.cos(math.radians(h)), head_width=0.05, color="0.4")
    ax.plot([0, 0], [-0.074, 1.5], ls=":", c="0.5")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("y (m)"); ax.set_ylabel("x (m), dock (grey) at the origin")
    ax.legend(fontsize=8, loc="lower left"); ax.set_title("Docking from 5 starts (sim, real controller + Pi pilot)")
    fig.tight_layout(); fig.savefig(OUT / "dock_5starts.png", dpi=110)


if __name__ == "__main__":
    main()
