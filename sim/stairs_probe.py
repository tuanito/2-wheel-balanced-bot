"""Can the robot measure a staircase by nodding, using only the LD19 lidar and the sonar?

Two separate questions, measured separately on purpose:

  A. GEOMETRY - if the body is pitched forward by theta, does the tilted scan plane recover the
     riser and tread? Run kinematically (the robot is placed at a pitch, nothing is balancing),
     so this measures the sensing and the estimator, not the controller.
  B. DYNAMICS - a two-wheel balancer cannot hold a pitch while standing still: leaning forward IS
     driving forward. So the lean has to be a nod, and the cost of a nod is how far it travels
     while doing it. That number decides how far back it is safe to look.
  C. The two together, as the two tables a controller actually needs: WHERE the first tread is
     visible from (standoff x pitch) and WHAT a nod of a given depth costs in travel, entered at a
     given speed. Everything ml/stairs_env.py claims about the manoeuvre comes from these two.

  python sim/stairs_probe.py                      # all three, on 100 mm x 300 mm steps
  python sim/stairs_probe.py --part c             # one part only
  python sim/stairs_probe.py --rise 0.17 --tread 0.25
"""
import math, pathlib, subprocess, sys
import numpy as np
import mujoco

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
import build_model                                           # noqa: E402
from botsim import BotSim, LIB, CTRL_DT                      # noqa: E402

H_LIDAR = 0.231          # scan plane above the floor, upright (pi/robot/ld19.py LIDAR_HEIGHT)
RANGE_NOISE = 0.010      # m, LD19, the same figure nav_sim.py uses


def opt(name, default):
    return type(default)(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


def model_xml(rise, tread, steps=6, edge=1.2):
    """The staircase model as a string, for MjModel.from_xml_string: no temporary file, no subprocess.

    meshdir has to be made absolute - from_xml_string resolves relative paths against the working
    directory, not against any file. 30 ms to compile, which is nothing beside an episode.
    """
    s = build_model.xml(stairs=dict(rise=rise, tread=tread, steps=steps, edge=edge))
    return s.replace('meshdir="."', f'meshdir="{HERE}"')


def build(rise, tread, steps, edge):
    out = HERE / "robot_c_stairs.xml"
    subprocess.run([sys.executable, str(HERE / "build_model.py"), "--stairs", "--rise", str(rise),
                    "--tread", str(tread), "--steps", str(steps), "--edge", str(edge)],
                   check=True, capture_output=True)
    return out


def ray_hit(sim, name, rng):
    """World point a rangefinder struck, from MuJoCo's own site frame: no sign conventions to get wrong."""
    sid = sim.m.site(name).id
    o = sim.d.site_xpos[sid]
    z = sim.d.site_xmat[sid].reshape(3, 3)[:, 2]            # rangefinder looks along the site's +z
    return o + rng * z


def nod_cloud(sim, x_robot, sweep, rng, n_rays=120, half_y=0.25):
    """Sweep the body pitch and project EVERY ray of the fan: a point cloud of the floor ahead.

    The fan is a circle in the body's own plane, so when the body pitches the forward ray looks
    steepest down and the side rays stay shallower. That spread is useful: one nod samples a patch
    rather than a single line, which is what makes the first tread resolvable at all.
    """
    adr = [sim.m.sensor(f"lidar{i}").adr[0] for i in range(n_rays)]
    pts = []
    for th in sweep:
        sim.place(pitch=th, x=x_robot, settle=False)
        mujoco.mj_forward(sim.m, sim.d)
        for i in range(n_rays):
            r = float(sim.d.sensordata[adr[i]])
            if not (0.02 < r < 12.0):
                continue
            p = ray_hit(sim, f"ray{i}", r + rng.normal(0, RANGE_NOISE))
            if p[0] > x_robot + 0.05 and abs(p[1]) < half_y:     # ahead of the robot, off the walls
                pts.append((p[0], p[1], p[2]))
    return np.array(pts) if pts else np.zeros((0, 3))


def estimate(pts, landing_z, tol=0.02, min_pts=8):
    """Edge, first riser and visible tread depth, by grouping the cloud into level surfaces by height.

    Clustering on HEIGHT, not on x: beyond the edge the hits jump outwards, so the cloud is not
    monotonic in x and sorting by x mixes the landing with whatever lies two steps down.
    """
    if len(pts) < min_pts:
        return None
    z = pts[:, 2]
    surfaces = []
    for zc in np.arange(z.min(), z.max() + tol, tol):
        m = np.abs(z - zc) <= tol / 2
        if m.sum() >= min_pts:
            surfaces.append(dict(z=float(np.median(z[m])), n=int(m.sum()),
                                 x0=float(pts[m, 0].min()), x1=float(pts[m, 0].max())))
    # merge surfaces that are really the same level
    merged = []
    for s in sorted(surfaces, key=lambda s: -s["z"]):
        if merged and abs(merged[-1]["z"] - s["z"]) < tol:
            m = merged[-1]
            m["x0"] = min(m["x0"], s["x0"]); m["x1"] = max(m["x1"], s["x1"]); m["n"] += s["n"]
        else:
            merged.append(dict(s))
    land = next((s for s in merged if abs(s["z"] - landing_z) < 0.02), None)
    if land is None or len(merged) < 2:
        return dict(surfaces=merged, edge=None, rise=None, tread=None)
    below = [s for s in merged if s["z"] < land["z"] - 0.02]
    if not below:
        return dict(surfaces=merged, edge=land["x1"], rise=None, tread=None)
    first = below[0]
    # n_first is the confidence the robot itself can use to choose between two nods' estimates: how many
    # returns landed on the surface the riser was measured from. No privileged information in it.
    return dict(surfaces=merged, edge=land["x1"], rise=land["z"] - first["z"],
                tread=first["x1"] - first["x0"], n_surfaces=len(merged), n_first=first["n"])


def part_a(rise, tread, steps, edge, standoff=0.35, trials=8):
    xml = build(rise, tread, steps, edge)
    sweep = np.radians(np.arange(8, 50.5, 0.5))
    rows = []
    for k in range(trials):
        rng = np.random.default_rng(4000 + k)
        sim = BotSim(xml=xml.name, seed=4000 + k, randomize=False)
        x_robot = edge - standoff
        landing_z = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
        pts = nod_cloud(sim, x_robot, sweep, rng)
        est = estimate(pts, landing_z)
        sim.close()
        if est and est.get("rise") is not None and est.get("edge") is not None:
            rows.append((standoff, est["edge"], est["rise"], est["tread"]))
    if not rows:
        return None
    A = np.array(rows, dtype=float)
    return dict(standoff=standoff, n=len(rows), trials=trials,
                edge_err=float(np.mean(A[:, 1] - edge)), edge_sd=float(np.std(A[:, 1])),
                rise=float(np.mean(A[:, 2])), rise_sd=float(np.std(A[:, 2])),
                tread=float(np.mean(A[:, 3])), tread_sd=float(np.std(A[:, 3])))


def part_b(rise, tread, steps, edge):
    print(f"\n\033[1mB. dynamics: what a nod costs in forward travel\033[0m")
    xml = build(rise, tread, steps, edge)
    print(f"   {'commanded v':>12} {'peak lean':>10} {'travel during the nod':>23} {'returns upright':>16}")
    for v in (0.15, 0.25, 0.35, 0.50):
        sim = BotSim(xml=xml.name, seed=7, randomize=False)
        sim.place(pitch=0.0, x=edge - 1.6)                   # well back from the edge
        sim.hold(0.6)
        x0 = float(sim.d.qpos[0])
        peak, pitches = 0.0, []
        sim.cmd.mode = 0; sim.cmd.v = v                      # CMD_VELOCITY = 0: accelerate -> it leans
        for _ in range(int(0.30 / CTRL_DT)):
            o = sim.step(); peak = max(peak, abs(o.pitch)); pitches.append(o.pitch)
        sim.cmd.v = 0.0                                      # stop: it leans back and settles
        for _ in range(int(1.2 / CTRL_DT)):
            o = sim.step(); peak = max(peak, abs(o.pitch))
        travel = float(sim.d.qpos[0]) - x0
        upright = abs(float(sim.out.pitch)) < math.radians(3)
        print(f"   {v:9.2f} m/s {math.degrees(peak):9.1f}deg {travel*1000:20.0f} mm {str(upright):>16}")
        sim.close()


def part_c_window(rise, tread, steps, edge, n_rays=120):
    """Where is the first tread visible FROM? Points landing on it, per (standoff, body pitch).

    Kinematic on purpose: this is the sensor geometry, with no controller in it. The band is diagonal
    because to see past the step nose the scan plane has to drop below the landing at the edge
    (tan theta > h_lidar / standoff), while to land ON the first tread rather than two steps down it
    must not drop too far. Beyond ~550 mm the two conditions no longer overlap at all - which is the
    whole reason a nod has to be timed rather than just taken from a safe distance.
    """
    print(f"\n\033[1mC1. where the first tread is visible from: returns on it, {n_rays} rays, one placement\033[0m")
    print(f"    truth: riser {rise*1000:.0f} mm, tread {tread*1000:.0f} mm\n")
    sim = BotSim(xml_str=model_xml(rise, tread, steps, edge), seed=0, randomize=False)
    land = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
    adr = [sim.m.sensor(f"lidar{i}").adr[0] for i in range(n_rays)]
    ths = list(range(14, 51, 3))
    print("    standoff " + "".join(f"{t:5d}" for t in ths) + "   deg")
    for D in (0.80, 0.70, 0.60, 0.50, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20):
        row = []
        for thd in ths:
            sim.place(pitch=math.radians(thd), x=edge - D, settle=False)
            mujoco.mj_forward(sim.m, sim.d)
            n = 0
            for i in range(n_rays):
                r = float(sim.d.sensordata[adr[i]])
                if not (0.02 < r < 12.0):
                    continue
                p = ray_hit(sim, f"ray{i}", r)
                if abs(p[1]) < 0.25 and edge < p[0] < edge + tread and abs(p[2] - (land - rise)) < 0.02:
                    n += 1
            row.append(n)
        print(f"    {D*1000:5.0f} mm " + "".join(f"{n:5d}" for n in row))
    sim.close()


def part_c_cost(rise, tread, steps, trials=5):
    """What a nod costs, with the real controller flying: a FLICK (look_time 0, depth set by the target).

    Measured on a 3 m landing with nothing to fall off, over randomised robots, so the numbers are the
    manoeuvre's own cost and its spread. Two things dominate: the depth - gravity acting over the time
    the body is tilted is what adds the forward speed - and the speed it is entered at, because the
    nod's own acceleration cancels a reverse entry and the robot can end up back where it started.
    Entered rolling forwards the same flick costs several times the travel and starts losing robots.
    """
    print(f"\n\033[1mC2. what a flick costs: peak pitch, travel and survival, {trials} randomised robots\033[0m")
    xml = model_xml(rise, tread, steps, edge=4.0)
    print(f"    {'entry v':>9} {'target':>7} {'peak':>8} {'travel to peak':>15} {'travel in total':>17} "
          f"{'t > 26 deg':>11} {'upright':>8}")
    for vc in (0.30, 0.0, -0.30, -0.50):
        for tgt in (0.60, 0.80, 1.00):
            rows = []
            for k in range(trials):
                sim = BotSim(xml_str=xml, seed=600 + k, randomize=True)
                sim.place(pitch=0.0, x=2.2)      # 1.2 m of landing behind it: the reverse entry needs room
                sim.hold(0.6)
                sim.cmd.mode = 0
                sim.cmd.v = vc
                for _ in range(int(2.0 / CTRL_DT)):
                    sim.step()
                x0, v0 = float(sim.d.qpos[0]), float(sim.out.v)
                LIB.bot_look(sim.ctl, 1, tgt, 0.0)
                sim.cmd.v = -0.5                             # pull back on the way out
                peak, xpk, xfar, bad, n = 0.0, x0, x0, False, 0
                for _ in range(int(2.3 / CTRL_DT)):
                    o = sim.step()
                    if o.state != 4:
                        bad = True
                        break
                    if o.pitch > peak:
                        peak, xpk = float(o.pitch), float(sim.d.qpos[0])
                    if o.pitch > math.radians(26):
                        n += 1
                    xfar = max(xfar, float(sim.d.qpos[0]))
                rows.append((v0, math.degrees(peak), (xpk - x0) * 1000, (xfar - x0) * 1000,
                             n * CTRL_DT, 0.0 if bad else 1.0))
                sim.close()
            a = np.array(rows)
            print(f"    {a[:, 0].mean():+7.2f} m/s {tgt:7.2f} {a[:, 1].mean():5.1f}+-{a[:, 1].std():2.0f}d "
                  f"{a[:, 2].mean():+9.0f} mm    {a[:, 3].mean():+9.0f} +-{a[:, 3].std():3.0f} mm "
                  f"{a[:, 4].mean():10.3f}s {a[:, 5].mean():7.0%}")


if __name__ == "__main__":
    rise, tread = opt("--rise", 0.10), opt("--tread", 0.30)
    steps, edge = opt("--steps", 6), opt("--edge", 1.2)
    part = opt("--part", "abc")
    if "a" in part:
        print(f"\n\033[1mA. geometry: nod 8 deg -> 50 deg, all 120 rays projected\033[0m")
        print(f"   truth: riser {rise*1000:.0f} mm, tread {tread*1000:.0f} mm, edge at x = {edge} m\n")
        print(f"   {'standoff':>9} {'usable':>7} {'edge error':>12} {'riser':>16} {'visible tread':>16}")
        for so in (0.80, 0.55, 0.40, 0.30, 0.20):
            r = part_a(rise, tread, steps, edge, standoff=so)
            if r is None:
                print(f"   {so*1000:6.0f} mm {0:4d}/8  {'first tread never visible':>48}")
                continue
            print(f"   {so*1000:6.0f} mm {r['n']:4d}/{r['trials']}  {r['edge_err']*1000:+8.0f} mm "
                  f"{r['rise']*1000:8.0f} +-{r['rise_sd']*1000:3.0f} mm "
                  f"{r['tread']*1000:9.0f} +-{r['tread_sd']*1000:3.0f} mm")
    if "b" in part:
        part_b(rise, tread, steps, edge)
    if "c" in part:
        part_c_window(rise, tread, steps, edge)
        part_c_cost(rise, tread, steps)
