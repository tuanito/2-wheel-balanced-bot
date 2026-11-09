"""Backward tumble ('backflip' over its own top) + stand-up, rendered."""
import os, math, pathlib, subprocess
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco, ctypes as C
from PIL import Image, ImageDraw, ImageFont
from botsim import BotSim, STATES, CTRL_DT, LIB, quat_pitch
OUT = pathlib.Path(__file__).parent / "results"
s = BotSim(seed=0); s.place(0); s.hold()
LIB.bot_limits(s.ctl, 4.0, 2.0, 1.2, 3.0)
r = mujoco.Renderer(s.m, 540, 960); cam = mujoco.MjvCamera()
cam.distance, cam.elevation, cam.azimuth = 1.1, -10, 90
font = ImageFont.load_default(size=26); small = ImageFont.load_default(size=20)
frames = pathlib.Path("/tmp/flipframes"); frames.mkdir(exist_ok=True)
for f in frames.glob("*.png"): f.unlink()
n = [0]; look = np.zeros(3); lid = s.m.geom("lidar").id; floor = s.m.geom("floor").id
def shot(title, slow=1):
    global look
    look = 0.4 * look + 0.6 * np.array([s.d.qpos[0], s.d.qpos[1], 0.03])
    cam.lookat[:] = look
    r.update_scene(s.d, cam); im = Image.fromarray(r.render()); dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, 960, 74], fill=(255, 255, 255))
    dr.text((16, 8), title, fill=(20, 20, 20), font=font)
    dr.text((16, 44), f"pitch {math.degrees(quat_pitch(s.d.qpos[3:7])):+7.1f}°   speed {s.out.v:+5.2f} m/s   state {STATES[s.out.state]}" + ("   (slow motion)" if slow > 1 else ""),
            fill=(70, 70, 70), font=small)
    for _ in range(slow): im.save(frames / f"{n[0]:05d}.png"); n[0] += 1
def run(title, secs, every=17, slow=1, raw_torque=None, **cmd):
    for k, v in cmd.items(): setattr(s.cmd, k, v)
    for i in range(int(round(secs / CTRL_DT))):
        if raw_torque is None: s.step()
        else:                               # controller keeps estimating, but we drive the motors directly
            LIB.bot_set_cmd(s.ctl, C.byref(s.cmd)); LIB.bot_step(s.ctl, C.byref(s.sense()), C.byref(s.out))
            s.t += CTRL_DT; s.log.append((s.t, s.out.state, 0,0,0,0,0,0,0,0,0,0,0,0))
            for _ in range(2):
                wl = s.d.qvel[s.m.jnt_dofadr[s.m.joint("left_hinge").id]]; wr = s.d.qvel[s.m.jnt_dofadr[s.m.joint("right_hinge").id]]
                s.d.ctrl[:] = [s.motor(raw_torque, wl), s.motor(raw_torque, wr)]; mujoco.mj_step(s.m, s.d)
        if i % every == 0: shot(title, slow)
run("Balancing", 0.8)
run("Backflip: run up backwards to ~3.9 m/s", 2.6, v=-4.0)
run("Brake hard: the body swings back over its head", 0.12, every=4, slow=2, raw_torque=2.3)
run("Tumbles over its top (the lidar takes the hit)", 0.9, every=4, slow=2, raw_torque=0.0, v=0.0)
run("Landed on its face -> stands up", 3.5, v=0.0)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i", str(frames / "%05d.png"), "-c:v", "libx264", "-preset", "veryfast",
                "-crf", "26", "-pix_fmt", "yuv420p", str(OUT / "backflip.mp4")], check=True)
L = np.array(s.log); st = L[:, 1].astype(int)
print([(round(L[i,0],2), STATES[st[i]]) for i in range(len(st)) if i == 0 or st[i] != st[i-1]])
print("frames", n[0], "final state", STATES[s.out.state], "pitch", math.degrees(quat_pitch(s.d.qpos[3:7])))
