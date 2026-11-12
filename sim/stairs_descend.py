"""Going DOWN: how much of the staircase the array can map at once, and what a step costs to take.

Two questions, in the order they have to be answered.

**A. How far ahead can it see?** The occlusion condition for the k-th tread - clear the nose of step
k-1, land before the nose of step k - reduces to D <= h * tread / rise for every k, with no k in it.
A staircase is self-similar, so a sensor that can see the first tread can geometrically see all of
them; what actually limits the map is angular resolution (8 rows over 45 degrees, so the far treads
fall into fewer and fewer zones) and range. That is a measurement, not an argument, so this measures
it: how many distinct treads come back with enough zones to fit a surface, and how wrong each one is.

**B. What does one step cost?** A 2-wheel balancer going down a 170 mm step is in free fall for about
190 ms with nothing under the wheels. Whether it survives, and at what speed it has to arrive, is a
dynamics question the controller answers - so it is asked here directly, by driving off the edge.

The answer to A decides the shape of the descent: map several steps and commit to a continuous
descent, or take one, stop, re-measure, take the next.

  python sim/stairs_descend.py --map        # A: treads mapped per frame, by standoff and staircase
  python sim/stairs_descend.py --step       # B: survival of a single step, by riser and approach speed
"""
import argparse, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "ml"))
from botsim import BotSim, LIB, CTRL_DT                        # noqa: E402
from stairs_probe import model_xml                             # noqa: E402
from stairs_map import surfaces                                # noqa: E402
from tof_sensor import xml_with_tof, Tof, TOF_Z, TOF_TILT, EDGE  # noqa: E402

MIN_ZONES = 4           # zones on a surface below which it is not a tread, it is a few stray returns


def map_depth(tilt, rise, tread, standoff, seed=0, height=TOF_Z, settle=0.8, frames=6):
    """Stand still and pool a few frames: which treads come back, and how wrong is each one?

    Pooling frames is free here - the robot is stationary and the array runs at 15 Hz - and it is
    what the real thing would do, so the far treads get the benefit of several looks.
    """
    xml, n = xml_with_tof(math.radians(tilt), rise, tread, steps=8, edge=EDGE, height=height)
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - standoff)
    sim.hold(0.4)
    sim.cmd.mode = 0
    sim.cmd.v = 0.0
    for _ in range(int(settle / CTRL_DT)):
        if sim.step().state != 4:
            sim.close()
            return None
    tof = Tof(sim, n, np.random.default_rng(seed))
    pts = []
    for _ in range(frames):
        for _ in range(int(0.067 / CTRL_DT)):          # one frame period at 15 Hz
            sim.step()
        c = tof.cloud()
        if len(c):
            pts.append(c)
    landing_z = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
    sim.close()
    if not pts:
        return dict(found=0, errs=[])
    cloud = np.concatenate(pts)
    surf = surfaces(cloud, min_pts=MIN_ZONES)
    # which step is each surface? step k sits k risers below the landing
    errs, seen = [], []
    for k in range(1, 7):
        want = landing_z - k * rise
        s = min(surf, key=lambda s: abs(s["z"] - want), default=None)
        if s is None or abs(s["z"] - want) > max(0.025, 0.35 * rise) or s["n"] < MIN_ZONES:
            continue
        seen.append(k)
        errs.append((k, (s["z"] - want) * 1000, s["n"]))
    return dict(found=len(seen), deepest=max(seen) if seen else 0, errs=errs, n_surf=len(surf))


def part_a(trials=4):
    print(f"\n\033[1mA. Treads mapped from one standing position "
          f"(array {TOF_Z*1000+54.3:.0f} mm up, {TOF_TILT:.0f} deg down, 6 frames pooled)\033[0m")
    print("   'deepest' is the furthest tread that came back with at least "
          f"{MIN_ZONES} zones on it.\n")
    print(f"   {'staircase':>18} {'standoff':>9} {'treads mapped':>14} {'deepest':>8}   per-tread error")
    for rise, tread in ((0.08, 0.30), (0.10, 0.30), (0.13, 0.28), (0.17, 0.26)):
        for D in (0.25, 0.30):
            rows = [map_depth(TOF_TILT, rise, tread, D, seed=600 + k) for k in range(trials)]
            rows = [r for r in rows if r]
            if not rows:
                continue
            found = np.mean([r["found"] for r in rows])
            deep = np.mean([r["deepest"] for r in rows])
            per = {}
            for r in rows:
                for k, e, n in r["errs"]:
                    per.setdefault(k, []).append(abs(e))
            txt = "  ".join(f"#{k}:{np.mean(v):.0f}mm" for k, v in sorted(per.items()))
            print(f"   {rise*1000:4.0f} x {tread*1000:3.0f} mm {D*1000:8.0f} mm {found:11.1f} "
                  f"{deep:8.1f}   {txt}")


def take_step(rise, tread, v_app, seed, brake=0.0, brake_at=0.06, steps=8, t_max=None, d0=0.6):
    """Drive off the edge at `v_app` and see what happens.

    `brake` commands a reverse speed once the axle is within `brake_at` of the edge. That is not
    braking - the robot is already committed and momentum carries it over - it is how a balancer
    leans BACK: the balance law tips the body away from the direction it is asked to accelerate. The
    body then arrives on the lower tread pitched back, so the landing impulse rotates it towards
    vertical instead of over the front. It is the same thing a person does stepping off a kerb.
    """
    xml = model_xml(rise, tread, steps, EDGE)
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    # the clock has to cover the approach, or a slow run simply never reaches the edge and reports
    # as a failure to survive a step it never took
    t_max = t_max or (d0 / max(0.05, abs(v_app)) + 3.0)
    post = int(1.6 / CTRL_DT)          # how long after the edge the step is judged
    sim.place(pitch=0.0, x=EDGE - d0)
    sim.hold(0.6)
    sim.cmd.mode = 0
    sim.cmd.obstacle_stop = 0
    sim.cmd.drop_stop = 0                      # the fence exists to stop exactly this: turn it off
    sim.cmd.v = v_app
    landing_z = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
    over, t_over = False, 0
    z_min, states, pitch_max = 0.0, set(), 0.0
    # Deliberately NOT failing on a state change: the wheels leave the ground, so the controller is
    # entitled to pass through LIFT or COAST on the way down. What matters is where it ends up. An
    # earlier version returned at the first non-BALANCE tick and reported 0% survival everywhere,
    # which was the test failing, not the robot.
    for i in range(int(t_max / CTRL_DT)):
        if brake and not over and (EDGE - float(sim.d.qpos[0])) < brake_at:
            sim.cmd.v = -brake                      # lean back: the body tips away from the demand
        o = sim.step()
        x = float(sim.d.qpos[0])
        if x > EDGE and not over:
            over, t_over = True, i
            sim.cmd.v = 0.0      # ONE step: stop asking for speed the moment the edge is crossed.
            # Without this the robot keeps its forward command and simply drives off the next step,
            # and the next - which is a test of descending a whole flight, not of taking one step,
            # and it reported even a 20 mm lip as unsurvivable.
        if over:
            states.add(int(o.state))
            pitch_max = max(pitch_max, abs(float(o.pitch)))
        z_min = min(z_min, float(sim.d.qpos[2]))
        # Judge 1.6 s after the edge, not at the end of the clock. Left running, the robot coasts
        # across the tread and goes off the NEXT edge, so a shallow step scored worse than a deep one
        # purely because it rolled further.
        if over and i - t_over >= post:
            break
    o = sim.out
    x, pitch, v = float(sim.d.qpos[0]), float(o.pitch), float(o.v)
    # upright, balancing, over the edge, and not still careering: that is a step taken
    # peak pitch matters as much as the end state: with auto_standup on, a robot that tumbles and
    # then picks itself up ends "balancing" and would otherwise score as having taken the step.
    ok = (over and int(o.state) == 4 and abs(pitch) < 0.25 and abs(v) < 0.35
          and pitch_max < 0.80)
    drop = landing_z - z_min
    sim.close()
    return dict(ok=float(ok), past=x - EDGE, pitch=pitch, v=v, state=int(o.state),
                drop=drop, peak_pitch=pitch_max, states=states)


def part_b2(trials=6):
    print(f"\n\033[1mB2. The same step, leaning back over the edge\033[0m")
    print("   A reverse command inside 60 mm of the edge: the body tips back, so it lands on its")
    print("   heels rather than its nose. Approach 0.30 m/s.\n")
    print(f"   {'riser':>7} " + "  ".join(f"{b:>5.2f} back" for b in (0.0, 0.2, 0.4, 0.6)))
    for rise in (0.08, 0.10, 0.13, 0.15, 0.17):
        cells = []
        for b in (0.0, 0.2, 0.4, 0.6):
            out = [take_step(rise, 0.30, 0.30, 700 + k, brake=b) for k in range(trials)]
            cells.append(f"{np.mean([o['ok'] for o in out]):6.0%}    ")
        print(f"   {rise*1000:4.0f} mm  {''.join(cells)}")


def part_b(trials=6):
    print(f"\n\033[1mB. Taking one step down: does the robot survive the drop?\033[0m")
    print("   Driving straight off the edge with the drop fence disabled, over randomised robots.\n")
    print(f"   {'riser':>7} " + "  ".join(f"{v:>5.2f} m/s" for v in (0.15, 0.30, 0.45)))
    for rise in (0.08, 0.10, 0.13, 0.15, 0.17):
        cells = []
        for v in (0.15, 0.30, 0.45):
            out = [take_step(rise, 0.30, v, 700 + k) for k in range(trials)]
            cells.append(f"{np.mean([o['ok'] for o in out]):6.0%}   ")
            if v == 0.30:
                det = out[0]
                tail = (f"  (one run: ends {det['past']*1000:+.0f} mm past the edge, pitch "
                        f"{math.degrees(det['pitch']):+.0f} deg, peak {math.degrees(det['peak_pitch']):.0f} deg, "
                        f"state {det['state']})")
        print(f"   {rise*1000:4.0f} mm  {''.join(cells)}{tail}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", action="store_true")
    ap.add_argument("--step", action="store_true")
    ap.add_argument("--trials", type=int, default=4)
    a = ap.parse_args()
    if a.map or not (a.map or a.step):
        part_a(a.trials)
    if a.step:
        part_b(max(4, a.trials))
        part_b2(max(4, a.trials))


if __name__ == "__main__":
    main()
