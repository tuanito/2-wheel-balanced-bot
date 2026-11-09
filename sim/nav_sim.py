"""Closed-loop test of the Pi navigation stack (pi/robot: mapper, scan matcher, avoider, explorer) on the simulated
robot in a 8 x 8 m room with obstacles. The balancing/standing is the C++ firmware controller as everywhere else.

  python3 nav_sim.py [seconds]   -> results/nav_map.png, results/nav_report.json, results/nav.mp4 (optional --video)
"""
import sys, math, json, pathlib, os
os.environ.setdefault("MUJOCO_GL", "osmesa")
import numpy as np, mujoco
sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "pi"))
from robot.mapper import Mapper
from robot.navigation import Avoider, FrontierExplorer
from robot.ld19 import Scan, level_scan
from botsim import BotSim, STATES, CTRL_DT

OUT = pathlib.Path(__file__).parent / "results"; OUT.mkdir(exist_ok=True)


def run(seconds=150.0, seed=3, video=False):
    s = BotSim("robot_c_arena.xml", seed=seed, randomize=True)
    s.place(pitch=-1.8, x=-2.8, y=-2.8, yaw=0.3)            # starts lying on its back in a corner
    rng = np.random.default_rng(seed)
    mapper = Mapper(folder=str(OUT / "nav_data"), match=True, log_scans=True)
    avoid, explore = Avoider(), FrontierExplorer(speed=0.45)
    n_rays = len(s.lidar_ids)
    ray_ang = 2 * np.pi * np.arange(n_rays) / n_rays
    obst_ids = {s.m.geom(i).id for i in range(s.m.ngeom) if s.m.geom(i).name.startswith(("obst", "leg"))}
    robot_geoms = {i for i in range(s.m.ngeom) if s.m.geom_bodyid[i] in (s.ch, s.m.body("wheel_left").id, s.m.body("wheel_right").id)}
    collisions, min_clear, log, blocked_n = 0, 9.0, [], 0
    frames = []
    if video:
        r = mujoco.Renderer(s.m, 540, 960); cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth, cam.lookat[:] = 9.5, -62, 90, [0, -0.6, 0]
    steps_per_scan = int(round(0.1 / CTRL_DT))
    v_cmd = w_cmd = 0.0
    for k in range(int(seconds / CTRL_DT)):
        o = s.step()
        for i in range(s.d.ncon):
            c = s.d.contact[i]
            if (c.geom1 in obst_ids and c.geom2 in robot_geoms) or (c.geom2 in obst_ids and c.geom1 in robot_geoms):
                collisions += 1
        if k % steps_per_scan: continue
        # ---- what the Pi does at 10 Hz ----
        rng_m = s.lidar()
        rng_m = np.where(rng_m > 0, rng_m + rng.normal(0, 0.01, n_rays), 0.0)    # LD19: ~±1 cm
        scan = level_scan(Scan(s.t, ray_ang.copy(), rng_m, np.full(n_rays, 200.0)), o.pitch, o.roll)
        odom = (o.x, o.y, o.yaw)
        on_wheels = o.state == 4
        level = abs(o.pitch - 0.06) < 0.07          # 0.06 rad: this robot's normal standing lean (CoM is 2 mm behind the axle)
        pose = mapper.update(odom, scan, moving_ok=on_wheels and level)
        pts = scan.points()
        if on_wheels:
            v_des, w_des, _ = explore.command(s.t, pose, mapper.grid)
            son = s.d.sensordata[s.m.sensor("sonar").adr[0]]
            v_cmd, w_cmd, blocked = avoid.safe_command(v_des, w_des, pts, son if 0.02 < son < 4.0 else -1.0)
            blocked_n += blocked
        else:
            v_cmd = w_cmd = 0.0
        s.cmd.mode, s.cmd.v, s.cmd.yaw_rate = 0, v_cmd, w_cmd
        log.append((s.t, *pose, s.d.qpos[0], s.d.qpos[1], s.true_yaw(), o.state, v_cmd, w_cmd, o.x, o.y, o.yaw))
        if video and k % (steps_per_scan) == 0:
            r.update_scene(s.d, cam); frames.append(r.render().copy())
    L = np.array(log)
    # pose error: compare mapper pose (map frame, starting at the odometry origin) with the truth, after aligning the start
    t0 = np.argmax(L[:, 7] == 4)
    p0_est, p0_true = L[t0, 1:4], L[t0, 4:7]
    dth = p0_true[2] - p0_est[2]
    R = np.array([[math.cos(dth), -math.sin(dth)], [math.sin(dth), math.cos(dth)]])
    est_in_true = (L[t0:, 1:3] - p0_est[:2]) @ R.T + p0_true[:2]
    err = np.linalg.norm(est_in_true - L[t0:, 4:6], axis=1)
    odo_in_true = (L[t0:, 10:12] - L[t0, 10:12]) @ np.array([[math.cos(p0_true[2] - L[t0, 12]), -math.sin(p0_true[2] - L[t0, 12])], [math.sin(p0_true[2] - L[t0, 12]), math.cos(p0_true[2] - L[t0, 12])]]).T + p0_true[:2]
    odo_err = np.linalg.norm(odo_in_true - L[t0:, 4:6], axis=1)
    head_err = np.degrees(np.angle(np.exp(1j * ((L[t0:, 3] - p0_est[2]) - (L[t0:, 6] - p0_true[2])))))
    odo_head_err = np.degrees(np.angle(np.exp(1j * ((L[t0:, 12] - L[t0, 12]) - (L[t0:, 6] - p0_true[2])))))
    mapper.save()
    report = dict(seconds=seconds, collisions_contact_steps=int(collisions),
                  distance_driven_m=float(np.sum(np.linalg.norm(np.diff(L[t0:, 4:6], axis=0), axis=1))),
                  pose_error_final_m=float(err[-1]), pose_error_max_m=float(err.max()), odometry_only_error_final_m=float(odo_err[-1]),
                  heading_error_final_deg=float(head_err[-1]), odometry_heading_error_final_deg=float(odo_head_err[-1]),
                  time_balancing_pct=float(100 * np.mean(L[:, 7] == 4)), avoid_blocked_ticks=int(blocked_n),
                  mapped_occupied_cells=int((mapper.grid.L > 1).sum()), mapped_free_cells=int((mapper.grid.L < -1).sum()))
    json.dump(report, open(OUT / "nav_report.json", "w"), indent=1)
    plot(mapper, L, t0, est_in_true, s)
    if video: write_video(frames)
    return report


def plot(mapper, L, t0, est_in_true, s):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    g = mapper.grid
    fig, ax = plt.subplots(1, 2, figsize=(13, 6.3))
    ext = [g.origin, g.origin + g.n * g.res] * 2
    ax[0].imshow(1 / (1 + np.exp(g.L)), origin="lower", cmap="gray", extent=ext, vmin=0, vmax=1)
    ax[0].plot(L[t0:, 1], L[t0:, 2], "r-", lw=1)
    ax[0].set_title("Map built by the Pi code (robot's own frame)\nblack = obstacle, white = free, red = path")
    ax[0].set_xlim(-1, 7.5); ax[0].set_ylim(-1.5, 7.5)
    # truth
    for i in range(s.m.ngeom):
        n = s.m.geom(i).name
        if n.startswith(("obst", "leg")):
            p, sz = s.m.geom_pos[i], s.m.geom_size[i]
            if s.m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX:
                ax[1].add_patch(plt.Rectangle((p[0] - sz[0], p[1] - sz[1]), 2 * sz[0], 2 * sz[1], color="0.45"))
            else:
                ax[1].add_patch(plt.Circle(p[:2], sz[0], color="saddlebrown"))
    ax[1].plot(L[t0:, 4], L[t0:, 5], "b-", lw=1, label="true path")
    ax[1].plot(est_in_true[:, 0], est_in_true[:, 1], "r--", lw=0.8, label="robot's estimate")
    ax[1].plot(L[0, 4], L[0, 5], "ko", label="start (lying on its back)")
    ax[1].set_xlim(-4.2, 4.2); ax[1].set_ylim(-4.2, 4.2); ax[1].set_aspect("equal"); ax[1].legend(loc="upper right", fontsize=8)
    ax[1].set_title("Ground truth room (brown dot: 18 cm post, below the lidar)")
    ax[0].set_aspect("equal")
    fig.tight_layout(); fig.savefig(OUT / "nav_map.png", dpi=100)


def write_video(frames):
    import subprocess, tempfile
    d = pathlib.Path(tempfile.mkdtemp())
    from PIL import Image
    for i, f in enumerate(frames): Image.fromarray(f).save(d / f"{i:05d}.png")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "40", "-i", str(d / "%05d.png"), "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "28", "-pix_fmt", "yuv420p", str(OUT / "nav.mp4")], check=True)


if __name__ == "__main__":
    secs = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else 150.0
    print(json.dumps(run(secs, video="--video" in sys.argv), indent=1))
