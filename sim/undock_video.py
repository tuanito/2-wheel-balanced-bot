"""Video: 2 departures from the dock (slippery and grippy floor). The robot docks first (not shown), sits charging
with its motors off, then UNDOCK: wheels roll back under it until upright, balance, drive out over the saddle lip,
turn and go. Writes results/undock_2runs.mp4."""
import os, math, subprocess
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
import dock_sim as D
from botsim import STATES, CTRL_DT

OUT = D.OUT
W, H, FPS = 960, 540, 30
OPT = mujoco.MjvOption(); OPT.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0
RUNS = [("Departure 1: slippery floor (friction 0.4)", 0.4, 304), ("Departure 2: grippy floor (friction 1.0)", 1.0, 302)]


def main():
    D.build_xml()
    font = ImageFont.load_default(size=26); small = ImageFont.load_default(size=19)
    fd = OUT / "undock_frames"; fd.mkdir(exist_ok=True)
    for f in fd.glob("*.png"): f.unlink()
    n = 0
    for title, mu, seed in RUNS:
        os.environ["FLOOR_MU"] = str(mu)
        s = D.DockSim(seed=seed, randomize=True); s.seed_ = seed
        s.place(x=0.30, y=0.0, yaw=0.0); s.hold(0.5); s.run(2.0)
        p = D.DockPilot(); k = 0
        while s.t < 60:                                         # dock (not filmed)
            if k % 50 == 0:
                o = s.out
                m, v, w, lean = p.step(s.t, s.scan(with_intensity=True), (o.x, o.y, o.yaw), STATES[o.state], s.charge, o.pitch, D._ir_bits(s))
                s.cmd.mode = D.MODE[m]; s.cmd.v = v; s.cmd.yaw_rate = w
                if lean: s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF
                if m in ("done", "failed"): break
            s.step(); k += 1
        print(title, "docked:", p.phase, flush=True)
        r = mujoco.Renderer(s.m, H, W)
        cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.elevation, cam.azimuth = 0.9, -14, 125
        look = np.array([0.0, 0.0, 0.03])
        script = [("Docked, charging, motors off", 1.5, dict(mode=0, v=0.0, yaw_rate=0.0)),
                  ("UNDOCK: wheels roll back under the body", 1.5, dict(mode=5, v=0.0, yaw_rate=0.0, seq=1)),
                  ("Balancing: drive out over the saddle lip", 3.0, dict(mode=0, v=0.15, yaw_rate=0.0)),
                  ("Turn and drive off", 2.0, dict(mode=0, v=0.25, yaw_rate=1.2)),
                  ("Turn and drive off", 2.0, dict(mode=0, v=0.3, yaw_rate=0.0))]
        stride = int(round(1 / FPS / CTRL_DT)); k = 0; t0 = s.t
        for cap, secs, cmd in script:
            for key, v in cmd.items():
                if key == "seq": s.cmd.seq = (s.cmd.seq + 1) & 0xFFFF
                else: setattr(s.cmd, key, v)
            for _ in range(int(round(secs / CTRL_DT))):
                o = s.step(); k += 1
                if k % stride: continue
                look = 0.9 * look + 0.1 * np.array([0.5 * s.d.qpos[0], 0.5 * s.d.qpos[1], 0.06])
                cam.lookat[:] = look
                cam.distance = 0.9 * cam.distance + 0.1 * (0.8 + 0.6 * math.hypot(s.d.qpos[0], s.d.qpos[1]))
                r.update_scene(s.d, cam, OPT)
                im = Image.fromarray(r.render()); dr = ImageDraw.Draw(im)
                dr.rectangle([0, 0, W, 78], fill=(255, 255, 255))
                dr.text((16, 6), title, fill=(20, 20, 20), font=font)
                dr.text((16, 44), f"{cap}   |  {STATES[o.state]:9s} pitch {math.degrees(o.pitch):+5.1f}°  speed {o.v:+.2f} m/s  "
                        f"{'charging' if s.charge else ''}  t {s.t - t0:4.1f} s", fill=(70, 70, 70), font=small)
                im.save(fd / f"{n:05d}.png"); n += 1
        print(title, "end state", STATES[s.out.state], "x", round(float(s.d.qpos[0]), 2), flush=True)
        r.close(); s.close()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(fd / "%05d.png"),
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "26", "-pix_fmt", "yuv420p", str(OUT / "undock_2runs.mp4")], check=True)
    for f in fd.glob("*.png"): f.unlink()


if __name__ == "__main__":
    main()
