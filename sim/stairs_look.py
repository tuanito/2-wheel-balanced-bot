"""End to end: reverse, nod, and measure the staircase from the lidar while the robot is balancing.

Nothing is placed by hand here - the real controller is flying the robot throughout, the lidar is
sampled every control tick with LD19 noise, and the riser is estimated from that cloud alone.

This is the script that showed the problem: it gets the EDGE (1.170 +- 55 mm against 1.200) but not the
riser, because its fixed 1.5 s reverse run-up leaves the robot 640-1015 mm out when the pitch peaks, and
from there the step nose hides the whole first tread. The timing is what `ml/stairs_env.py` learns;
`sim/stairs_probe.py --part c` is where the two tables that explain it are measured.

  python sim/stairs_look.py                       # 100 mm x 300 mm steps
  python sim/stairs_look.py --rise 0.17 --tread 0.25
"""
import math, pathlib, subprocess, sys
import numpy as np
import mujoco

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
from botsim import BotSim, LIB, CTRL_DT              # noqa: E402
from stairs_probe import ray_hit, estimate, RANGE_NOISE, opt   # noqa: E402

N_RAYS, EDGE = 120, 1.2


def look_and_measure(rise, tread, steps, seed, standoff=0.35, runup=1.5, tgt=0.60, dur=0.35):
    subprocess.run([sys.executable, str(HERE / "build_model.py"), "--stairs", "--rise", str(rise),
                    "--tread", str(tread), "--steps", str(steps), "--edge", str(EDGE)],
                   check=True, capture_output=True)
    rng = np.random.default_rng(seed)
    sim = BotSim(xml="robot_c_stairs.xml", seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - standoff); sim.hold(0.8)
    landing_z = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
    adr = [sim.m.sensor(f"lidar{i}").adr[0] for i in range(N_RAYS)]

    sim.cmd.mode = 0; sim.cmd.v = -1.5                       # reverse run-up, away from the edge
    for _ in range(int(runup / CTRL_DT)):
        sim.step()
    LIB.bot_look(sim.ctl, 1, tgt, dur)                       # nod
    pts, peak, x_far = [], 0.0, float(sim.d.qpos[0])
    for _ in range(int(1.6 / CTRL_DT)):
        o = sim.step()
        peak = max(peak, o.pitch)
        x_far = max(x_far, float(sim.d.qpos[0]))
        if o.pitch > math.radians(12):                       # only tilted enough to see the floor
            for i in range(N_RAYS):
                r = float(sim.d.sensordata[adr[i]])
                if not (0.02 < r < 12.0):
                    continue
                p = ray_hit(sim, f"ray{i}", r + rng.normal(0, RANGE_NOISE))
                if p[0] > float(sim.d.qpos[0]) + 0.05 and abs(p[1]) < 0.25:
                    pts.append((p[0], p[1], p[2]))
        if o.state != 4:
            break
    sim.cmd.v = 0.0
    for _ in range(int(1.5 / CTRL_DT)):
        o = sim.step(); x_far = max(x_far, float(sim.d.qpos[0]))
    est = estimate(np.array(pts) if pts else np.zeros((0, 3)), landing_z)
    ok = sim.out.state == 4
    sim.close()
    return dict(n=len(pts), peak=math.degrees(peak), margin=(EDGE - x_far) * 1000,
                est=est, upright=ok)


if __name__ == "__main__":
    rise, tread, steps = opt("--rise", 0.10), opt("--tread", 0.30), opt("--steps", 6)
    print(f"\n\033[1mNod and measure: {rise*1000:.0f} mm riser x {tread*1000:.0f} mm tread, "
          f"edge at {EDGE} m, 10 randomised robots\033[0m")
    print(f"{'seed':>5} {'points':>7} {'peak':>8} {'edge margin':>12} {'edge x':>8} "
          f"{'riser':>9} {'tread seen':>11} {'upright':>8}")
    rs, es = [], []
    for k in range(10):
        r = look_and_measure(rise, tread, steps, 5100 + k)
        e = r["est"] or {}
        rr, ee = e.get("rise"), e.get("edge")
        if rr is not None:
            rs.append(rr)
        if ee is not None:
            es.append(ee)
        print(f"{5100+k:5d} {r['n']:7d} {r['peak']:7.1f}deg {r['margin']:+9.0f} mm "
              f"{(f'{ee:.3f} m' if ee else '--'):>8} {(f'{rr*1000:.0f} mm' if rr else '--'):>9} "
              f"{(f'{e.get(chr(116)+chr(114)+chr(101)+chr(97)+chr(100))*1000:.0f} mm' if e.get('tread') else '--'):>11} "
              f"{str(r['upright']):>8}")
    if rs:
        print(f"\n  riser : {np.mean(rs)*1000:.0f} +- {np.std(rs)*1000:.0f} mm over {len(rs)}/10 nods "
              f"(true {rise*1000:.0f})   error {(np.mean(rs)-rise)*1000:+.0f} mm")
    if es:
        print(f"  edge  : {np.mean(es):.3f} +- {np.std(es)*1000:.0f} mm (true {EDGE:.3f})")
