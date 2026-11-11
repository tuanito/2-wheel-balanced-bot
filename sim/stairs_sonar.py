"""Would tilting the sonar down help see the edge? Measured, with the beam modelled as a cone.

The sim models the HC-SR04 as a single ray along +x, which flatters it badly: a real one is a wide
beam - roughly +-15 deg - that reports the range of the NEAREST thing anywhere inside it, with no
bearing at all. A pencil ray tilted at the floor would read a clean floor distance; a cone tilted at
the floor reads whatever its lowest edge touches first.

This replaces the single ray with a cone of rays and takes the minimum, which is the right first-order
model of a time-of-flight sonar, then measures the two things that decide whether the idea is worth
the hardware change:

  1. **Where the floor echo dies.** Driving at the edge, the floor fills the cone until the edge
     enters it; past that the nearest return is a tread far below, or nothing. That transition is an
     edge detector that owes nothing to odometry - which is what the margin failures need.
  2. **What body pitch does to it.** The robot is a balancing robot: the body swings several degrees
     just holding station, and the sonar is bolted to the body. A tilt that reads the floor at 320 mm
     at 0 deg reads something else entirely at -6 deg, and that is the whole question.

  python sim/stairs_sonar.py                 # both tables, tilts 0 to 20 deg
  python sim/stairs_sonar.py --tilt 10       # one tilt, verbose trace
"""
import argparse, math, pathlib, sys
import numpy as np
import mujoco

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "ml"))
from botsim import BotSim, CTRL_DT                       # noqa: E402
from stairs_probe import model_xml                       # noqa: E402

SONAR_Z = 0.080         # body frame, on the axle: 134 mm above ground with the 54.3 mm wheels
SONAR_X = 0.032
HALF = math.radians(15.0)       # HC-SR04 beam half-angle, the usual -6 dB figure
RINGS = (0.0, 0.5, 1.0)         # fractions of the half-angle to place rays at
PER_RING = 8
EDGE = 1.2


def cone_dirs(tilt):
    """Unit directions filling a cone of half-angle HALF about an axis pitched `tilt` below +x."""
    out = []
    for f in RINGS:
        phi = HALF * f
        n = 1 if f == 0 else PER_RING
        for k in range(n):
            psi = 2 * math.pi * k / n
            # cone about +x, then pitch down by `tilt` about y
            v = np.array([math.cos(phi), math.sin(phi) * math.cos(psi), math.sin(phi) * math.sin(psi)])
            c, s = math.cos(tilt), math.sin(tilt)
            out.append(np.array([c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]]))
    return out


def xml_with_cone(tilt, rise, tread, steps=6, edge=EDGE):
    """The stairs model with the single sonar ray replaced by a cone of them."""
    xml = model_xml(rise, tread, steps, edge)
    dirs = cone_dirs(tilt)
    sites = "".join(f'<site name="son{i}" pos="{SONAR_X} 0 {SONAR_Z}" '
                    f'zaxis="{d[0]:.6f} {d[1]:.6f} {d[2]:.6f}" size="0.002"/>' for i, d in enumerate(dirs))
    sens = "".join(f'<rangefinder name="son{i}" site="son{i}" cutoff="4"/>' for i in range(len(dirs)))
    old_site = '<site name="sonar" pos="0.032 0 0.080" zaxis="1 0 0" size="0.003"/>'
    old_sens = '<rangefinder name="sonar" site="sonar" cutoff="4"/>'
    assert old_site in xml and old_sens in xml, "the sonar site moved: update this script"
    return xml.replace(old_site, sites).replace(old_sens, sens), len(dirs)


class Cone:
    def __init__(self, sim, n):
        self.sim = sim
        self.adr = np.array([sim.m.sensor(f"son{i}").adr[0] for i in range(n)])

    def read(self):
        """What a sonar would report: the nearest echo in the cone, or None for no echo."""
        r = self.sim.d.sensordata[self.adr]
        ok = (r > 0.02) & (r < 4.0)
        return float(r[ok].min()) if ok.any() else None


def approach(tilt, rise, tread, seed, v=0.15, d0=1.0):
    """Creep at the edge, logging the cone's reading against the true distance to the drop."""
    xml, n = xml_with_cone(math.radians(tilt), rise, tread)
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - d0)
    sim.hold(0.6)
    sim.cmd.mode = 0
    sim.cmd.v = v
    cone = Cone(sim, n)
    rows = []
    for _ in range(int(12.0 / CTRL_DT)):
        o = sim.step()
        if o.state != 4:
            break
        d = EDGE - float(sim.d.qpos[0])
        rows.append((d, cone.read() or float("nan"), float(o.pitch)))
        if d < 0.12:
            break
    sim.close()
    return np.array(rows)


def lost(rows, jump=0.15):
    """Distance to the edge at which the floor echo is lost: it vanishes, or jumps by `jump` metres."""
    r = rows[:, 1]
    for i in range(3, len(rows)):
        prev = r[max(0, i - 3):i]
        if not np.isfinite(prev).any():
            continue
        base = np.nanmedian(prev)
        if not np.isfinite(r[i]) or (r[i] - base) > jump:
            return rows[i, 0]
    return float("nan")


def pitch_sensitivity(tilt, rise, tread, seed=900):
    """The cone's reading on open landing, with the body held at a range of pitches."""
    xml, n = xml_with_cone(math.radians(tilt), rise, tread)
    out = []
    for p in (-0.12, -0.06, 0.0, 0.06, 0.12):
        sim = BotSim(xml_str=xml, seed=seed, randomize=False)
        sim.place(pitch=p, x=EDGE - 0.9)
        cone = Cone(sim, n)
        mujoco.mj_forward(sim.m, sim.d)
        out.append((math.degrees(p), cone.read()))
        sim.close()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", type=float, default=None)
    ap.add_argument("--trials", type=int, default=4)
    a = ap.parse_args()
    tilts = [a.tilt] if a.tilt is not None else [0, 5, 10, 15, 20]
    print(f"\n\033[1m1. Where the floor echo dies, approaching the edge "
          f"({a.trials} randomised robots, 100 mm riser)\033[0m")
    print("   'floor at rest' is what the cone reads on open landing: anything closer than that is")
    print("   clutter the obstacle stop would have to ignore.\n")
    print(f"   {'tilt':>5} {'floor at rest':>15} {'edge seen at':>22} {'spread':>9}")
    for t in tilts:
        rest = pitch_sensitivity(t, 0.10, 0.30)
        r0 = next((v for p, v in rest if abs(p) < 1e-6), None)
        ds = [lost(approach(t, 0.10, 0.30, 800 + k)) for k in range(a.trials)]
        ds = np.array([d for d in ds if np.isfinite(d)])
        if len(ds):
            print(f"   {t:4.0f}d {(r0 * 1000 if r0 else float('nan')):12.0f} mm "
                  f"{ds.mean() * 1000:16.0f} mm {ds.std() * 1000:7.0f} mm")
        else:
            print(f"   {t:4.0f}d {(r0 * 1000 if r0 else float('nan')):12.0f} mm        never lost it")
    print(f"\n\033[1m2. What body pitch does to the same reading (robot still, 900 mm from the edge)\033[0m")
    print(f"   {'tilt':>5} " + " ".join(f"{p:+6.0f}d" for p in (-7, -3, 0, 3, 7)))
    for t in tilts:
        vals = pitch_sensitivity(t, 0.10, 0.30)
        cells = " ".join(f"{(v * 1000):6.0f}" if v else "   --- " for _, v in vals)
        print(f"   {t:4.0f}d {cells}   mm")
    print("\n   (a reading that moves hundreds of mm with a few degrees of sway is not a range to a")
    print("    surface, it is the cone grazing the floor at a different place)")


if __name__ == "__main__":
    main()
