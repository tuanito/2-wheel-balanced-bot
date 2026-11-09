"""Backflip via the controller's CMD_BACKFLIP, rendered (slow motion during the flip)."""
import os, math, pathlib, subprocess
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
from botsim import BotSim, STATES, CTRL_DT, quat_pitch
OUT = pathlib.Path(__file__).parent / "results"
s = BotSim(seed=0); s.place(0); s.hold()
r = mujoco.Renderer(s.m, 540, 960); cam = mujoco.MjvCamera()
cam.distance, cam.elevation, cam.azimuth = 1.3, -6, 90
font = ImageFont.load_default(size=26); small = ImageFont.load_default(size=20)
fr = pathlib.Path("/tmp/flip2"); fr.mkdir(exist_ok=True)
for f in fr.glob("*.png"): f.unlink()
n = 0; look = np.zeros(3)
TITLES = {"BALANCE": "Balancing", "FLIP_RUN": "Run-up backwards to 4.9 m/s", "FLIP_BRAKE": "Hard brake (30 ms)",
          "FLIP_AIR": "Over the top… (slow motion)", "FALLEN": "Missed: fell", "LYING": "Lying", "LIFT": "Standing up", "COAST": "Standing up"}
done_flip = False
for k in range(int(9.0 / CTRL_DT)):
    if k == int(1.0 / CTRL_DT): s.cmd.mode, s.cmd.seq = 3, 1
    o = s.step()
    st = STATES[o.state]
    if st == "FLIP_AIR": done_flip = True
    if done_flip and st == "BALANCE": s.cmd.mode = 0
    slow = st in ("FLIP_BRAKE", "FLIP_AIR")
    if k % (4 if slow else 17): continue
    look = 0.4 * look + 0.6 * np.array([s.d.qpos[0], s.d.qpos[1], 0.05])
    cam.lookat[:] = look
    r.update_scene(s.d, cam); im = Image.fromarray(r.render()); d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 960, 74], fill=(255, 255, 255))
    title = TITLES.get(st, st) if not (done_flip and st == "BALANCE") else "Caught on its wheels: balancing again"
    d.text((16, 8), title, fill=(20, 20, 20), font=font)
    d.text((16, 44), f"pitch {math.degrees(quat_pitch(s.d.qpos[3:7])):+7.1f}°   speed {o.v:+5.2f} m/s   state {st}", fill=(70, 70, 70), font=small)
    im.save(fr / f"{n:05d}.png"); n += 1
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i", str(fr / "%05d.png"), "-c:v", "libx264", "-preset", "veryfast",
                "-crf", "26", "-pix_fmt", "yuv420p", str(OUT / "backflip_full.mp4")], check=True)
print(n, STATES[s.out.state])
