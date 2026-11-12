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

Known residual: k is calibrated on a constant 0.15 m/s creep, while the Probe recipe approaches at
0.35 and slows late. That difference leaves a **+16 +- 14 mm** bias in the recipe - the robot believes
it is 16 mm further from the drop than it is, which is the unsafe direction, though the spread is still
tighter than the +3 +- 20 mm odometry it replaces. Calibrating against the recipe's own approach
profile would remove it; the lesson from the last two bugs is that the calibration has to match the
conditions it runs in, not just the detection rule.

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
SONAR_Z_GND = 0.1343    # the same height measured from the ground, which is what the geometry needs
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


# Measured by `--calibrate` with FloorWatch itself, 12 approaches per tilt over risers 80-170 mm at
# the real ping cadence and with the real noise. The event fires at k * r_floor(pitch), and k's SPREAD
# is the accuracy the robot gets:
#   tilt   k              event      residual
#    0d    0.974 +-0.041  462 mm     +-19 mm
#    5d    0.957 +-0.038  352 mm     +-13 mm
#   10d    0.934 +-0.027  282 mm     +- 8 mm
#   15d    0.900 +-0.026  232 mm     +- 6 mm   <- the pick: measured end to end at -4 +- 12 mm, and
#   20d    0.865 +-0.023  196 mm     +- 5 mm      the standoff it sets is where the look works
K_EVENT = {0: 0.974, 5: 0.957, 10: 0.934, 15: 0.900, 20: 0.865}


def event_distance(pitch, tilt):
    """Distance to the edge at the instant the floor echo dies, from the robot's own IMU pitch."""
    k = K_EVENT.get(int(round(math.degrees(tilt))), 1.0)
    return k * r_floor(pitch, tilt)



PING_DT = 0.050         # s between pings: the env's control-step cadence, and about an HC-SR04's limit
CONFIRM = 3             # pings the floor must stay lost before the event counts
PITCH_BAND = 0.05       # rad: the beam's floor range goes as 1/sin(tilt + 15 + pitch), so a few degrees
                        # of lean moves it tens of mm. At 0.12 rad the event fired during braking and
                        # biased the distance +55 mm - towards the drop. Level and steady, or not at all.
RATE_BAND = 0.60        # rad/s: likewise, a body still rotating is not a calibrated range finder


class FloorWatch:
    """The edge event, defined ONCE and shared by the calibration and the environment.

    Both have to use the same rule or the calibration is meaningless: the factor below is the ratio
    between where this detector fires and the floor range predicted at that moment, so a detector
    that fires two pings later produces a different factor. Measuring with one rule and running with
    another put a +80 mm bias into the distance estimate - in the dangerous direction.
    """

    def __init__(self, tilt, k=None, h=SONAR_Z_GND):
        self.tilt, self.k, self.h = tilt, k, h
        self.on_floor, self.lost_n, self.fired, self.at = False, 0, False, float("nan")
        self.long_n = 0

    def update(self, r, pitch, pitch_rate=0.0, lost_tol=0.080):
        exp = r_floor(pitch, self.tilt, self.h)
        on = (r is not None and math.isfinite(exp) and abs(r - exp) < lost_tol)
        self.lost_n = 0 if on else self.lost_n + 1
        # A dropped ping and a vanished floor both read as "no echo", and 2% of pings drop. An edge
        # also produces POSITIVE evidence: an echo from the stairs below, much longer than the floor
        # should be. Requiring at least one of those among the confirming pings is what separates the
        # two - without it, two dropped pings in a row fired the event and put a -239 mm error into
        # the distance in one episode out of eighty.
        self.long_n = 0 if on else self.long_n + (1 if (r is not None and r > exp + lost_tol) else 0)
        fire = (self.on_floor and self.lost_n >= CONFIRM and self.long_n >= 1
                and abs(pitch) < PITCH_BAND and abs(pitch_rate) < RATE_BAND
                and math.isfinite(exp) and not self.fired)
        if fire:
            k = K_EVENT.get(int(round(math.degrees(self.tilt))), 1.0) if self.k is None else self.k
            self.fired, self.at = True, k * exp
        if on:
            self.on_floor = True
        return fire


def r_floor(pitch, tilt, h=SONAR_Z_GND):
    """Range the cone should read on flat floor, from the robot's own IMU pitch.

    The nearest point of the beam on the floor is where its lowest edge lands, so the range is
    h / sin(tilt + HALF + pitch): exact geometry, no calibration. A reading longer than this means the
    floor the beam was grazing is no longer there - which is the edge, seen without odometry.
    Positive `pitch` is nose-down in this convention, matching the controller's sign.
    """
    a = tilt + HALF + pitch
    return h / math.sin(a) if a > math.radians(3.0) else float("inf")


def xml_with_cone(tilt, rise, tread, steps=6, edge=EDGE, height=None):
    """The stairs model with the single sonar ray replaced by a cone of them.

    `height` moves the transducer in the body frame. It matters as much as the tilt: the event fires
    at h/sin(tilt+15) and its sensitivity to body pitch goes as h*cos(a)/sin(a)^2, so a HIGH sensor
    steeply tilted gets a far event AND a well-conditioned one, which a low sensor cannot.
    """
    xml = model_xml(rise, tread, steps, edge)
    z = SONAR_Z if height is None else height - 0.0543
    dirs = cone_dirs(tilt)
    sites = "".join(f'<site name="son{i}" pos="{SONAR_X} 0 {z}" '
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


def approach(tilt, rise, tread, seed, v=0.15, d0=1.0, noise=0.005, drop=0.02, rng=None):
    """Creep at the edge, logging the cone's reading against the true distance to the drop."""
    xml, n = xml_with_cone(math.radians(tilt), rise, tread)
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - d0)
    sim.hold(0.6)
    sim.cmd.mode = 0
    sim.cmd.v = v
    cone = Cone(sim, n)
    rng = rng or np.random.default_rng(seed)
    rows, every = [], int(round(PING_DT / CTRL_DT))
    for i in range(int(12.0 / CTRL_DT)):
        o = sim.step()
        if o.state != 4:
            break
        if i % every:
            continue
        d = EDGE - float(sim.d.qpos[0])
        r = cone.read()
        if r is not None and rng.random() >= drop:
            r += float(rng.normal(0, noise))
        else:
            r = None
        rows.append((d, float("nan") if r is None else r, float(o.pitch), float(o.pitch_rate)))
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


def calibrate(tilt, trials=8, rises=(0.08, 0.10, 0.13, 0.15, 0.17)):
    """How far from the edge the echo actually dies, as a fraction of the predicted floor range.

    The beam does not lose the floor the instant its lowest ray clears the edge - the rest of the cone
    is still on floor - so the event comes in closer than r_floor(). The ratio is what a real robot
    would calibrate once against a known edge, and what matters is its SPREAD: that is the accuracy of
    every distance the sonar then hands to the fence.
    """
    ks, ds = [], []
    for k in range(trials):
        rise = rises[k % len(rises)]
        rows = approach(tilt, rise, 0.30, 900 + k)
        w = FloorWatch(math.radians(tilt), k=1.0)
        for d, r, pitch, prate in rows:
            if w.update(None if not np.isfinite(r) else r, pitch, prate):
                exp = r_floor(pitch, math.radians(tilt))      # w.at with k = 1
                if np.isfinite(exp) and exp > 0:
                    ks.append(d / exp)      # the factor that makes the prediction land on the truth
                    ds.append(d)
                break
    return (np.array(ks), np.array(ds))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", type=float, default=None)
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--trials", type=int, default=4)
    a = ap.parse_args()
    tilts = [a.tilt] if a.tilt is not None else [0, 5, 10, 15, 20]
    if a.calibrate:
        print(f"\n\033[1mCalibrating the edge event: {a.trials} approaches per tilt, risers 80-170 mm\033[0m")
        print(f"   {'tilt':>5} {'event / predicted floor':>26} {'event distance':>18} {'spread':>9}")
        for t in tilts:
            ks, ds = calibrate(t, a.trials)
            if not len(ks):
                print(f"   {t:4.0f}d   never lost the floor")
                continue
            print(f"   {t:4.0f}d {ks.mean():16.3f} +-{ks.std():5.3f} {ds.mean()*1000:13.0f} mm "
                  f"{ds.std()*1000:6.0f} mm")
        return
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
