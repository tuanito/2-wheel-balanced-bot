"""A VL53L5CX as the robot would actually have it: 8 x 8 zones, 45 x 45 degrees, bearing resolved.

Why this sensor changes the problem. Every number in the stair work so far is limited by one thing:
a ray can only reach the first tread if it clears the step nose AND lands before the next one, which
needs tan(a) between (h + rise)/(D + tread) and h/D. Those are compatible only when

    D  <  h * tread / rise

and the landing occludes everything below it short of the edge, so the vertical riser face is never
visible from behind the edge at any angle. The limit is the sensor HEIGHT, and it is why a 170 mm
riser is unreachable beyond 408 mm even from the lidar at 231 mm - and why mounting a time-of-flight
sensor low, as I first suggested, would have been the worst possible place for it: at 80 mm the same
riser is only reachable inside 141 mm, which is past the point of no return.

What the VL53L5CX adds is not reach, it is SIMULTANEITY. The lidar has one scan plane, so it has to
pitch the whole robot to put that plane inside the band - which costs travel towards the drop, needs
a wind-up, and is why the recipe takes three looks to answer a third of the time. A 45 degree vertical
field of view spans every angle in the band at once, from a standstill, with no nod at all.

  python sim/tof_sensor.py --band          # zones on the tread, by tilt and standoff
  python sim/tof_sensor.py --tilt 33       # one tilt, per-riser
"""
import argparse, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "ml"))
from botsim import BotSim, CTRL_DT                       # noqa: E402
from stairs_probe import model_xml                       # noqa: E402

N = 8                           # 8 x 8 zones
FOV = math.radians(45.0)        # 45 x 45 degrees of detection volume
ZONE = FOV / N                  # 5.625 deg per zone
TOF_X = 0.032                   # body frame: on the front panel face
TOF_Z = 0.140                   # body frame -> 194 mm above ground, the top of the front panel
TOF_TILT = 30.0                 # degrees below horizontal. Measured in --measure: best answer rate
                                # (27/30 at a 250 mm standoff against 26 at 35 deg and 23 at 40) while
                                # still seeing floor 1.47 m ahead, so it can do the sonar's job too
R_MIN, R_MAX = 0.02, 4.00       # datasheet range per zone
NEAR_MM, NEAR_ERR = 200.0, 0.015    # +-15 mm below 200 mm (2.7 sigma, continuous mode)
FAR_ERR = 0.05                  # +-5 % beyond, in the dark; 8-11 % in 5 klux ambient
EDGE = 1.2


SUB = ((0.0, 0.0), (-0.35, -0.35), (0.35, -0.35), (-0.35, 0.35), (0.35, 0.35))


def zone_dirs(tilt, n=N, fov=FOV, sub=SUB):
    """Rays per zone, in the body frame, for a sensor pitched `tilt` below +x.

    A zone is not a ray: it is a 5.6 degree cone, and the part reports the NEAREST target in it. That
    matters most exactly where this robot looks - a zone straddling the step nose sees landing and
    tread at once and latches the near one. Modelling a zone as its centre ray alone would quietly
    remove every mixed pixel from the problem, so each zone gets `sub` rays across its own footprint
    and the reading is their minimum.
    """
    out = []
    for r in range(n):
        for c in range(n):
            for (du, dv) in sub:
                el = tilt + (r - (n - 1) / 2 + dv) * (fov / n)
                az = (c - (n - 1) / 2 + du) * (fov / n)
                v = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)])
                out.append(v / np.linalg.norm(v))
    return out


def xml_with_tof(tilt, rise, tread, steps=6, edge=EDGE, height=TOF_Z, base=None):
    """The stairs model with 64 rangefinders standing in for the zone array."""
    xml = base if base is not None else model_xml(rise, tread, steps, edge)
    dirs = zone_dirs(tilt)
    sites = "".join(f'<site name="tof{i}" pos="{TOF_X} 0 {height}" '
                    f'zaxis="{d[0]:.6f} {d[1]:.6f} {d[2]:.6f}" size="0.002"/>' for i, d in enumerate(dirs))
    sens = "".join(f'<rangefinder name="tof{i}" site="tof{i}" cutoff="{R_MAX}"/>' for i in range(len(dirs)))
    old_site = '<site name="sonar" pos="0.032 0 0.080" zaxis="1 0 0" size="0.003"/>'
    old_sens = '<rangefinder name="sonar" site="sonar" cutoff="4"/>'
    assert old_site in xml and old_sens in xml, "the sonar site moved: update this script"
    return xml.replace(old_site, sites).replace(old_sens, sens), len(dirs)


class Tof:
    """Reads the array and turns it into what the sensor actually hands over: a cloud with bearings.

    The per-zone range noise follows the datasheet - +-15 mm below 200 mm, +-5 % beyond, both 2.7
    sigma in the dark - and zones with no return inside 4 m are dropped rather than reported as 4 m,
    which is what target status does on the real part.
    """

    def __init__(self, sim, n_rays, rng=None, sub=len(SUB)):
        self.sim, self.n, self.sub = sim, n_rays, sub
        self.adr = np.array([sim.m.sensor(f"tof{i}").adr[0] for i in range(n_rays)])
        self.sid = np.array([sim.m.site(f"tof{i}").id for i in range(n_rays)])
        self.rng = rng or np.random.default_rng(0)

    def frame(self):
        """The 8 x 8 range image, in metres, NaN where the zone had no valid target.

        Each zone is the nearest of its sub-rays, which is what the part reports; the noise is applied
        once per zone, not per ray, because it is one measurement.
        """
        r = np.array(self.sim.d.sensordata[self.adr], dtype=float)
        r[(r <= R_MIN) | (r >= R_MAX)] = np.nan
        if self.sub > 1:
            g = r.reshape(-1, self.sub)
            any_hit = np.isfinite(g).any(axis=1)          # a zone with no sub-ray hit has no target
            z = np.full(len(g), np.nan)
            z[any_hit] = np.nanmin(g[any_hit], axis=1)
        else:
            z = r
        sd = np.where(z < NEAR_MM / 1000.0, NEAR_ERR, FAR_ERR * z) / 2.7
        z = z + self.rng.normal(0, 1, len(z)) * sd
        return z.reshape(N, N)

    def cloud(self, frame=None):
        """World points, one per valid zone, placed along the zone's CENTRE ray.

        The bearing is known per zone, which is the whole point: a sonar hands back one number with no
        direction and cannot measure a height difference. The centre ray is the right direction to use
        because that is all the real part reports - the range came from somewhere in the cone, and the
        robot has to assume the middle of it.
        """
        r = (self.frame() if frame is None else frame).reshape(-1)
        ok = np.isfinite(r)
        if not ok.any():
            return np.zeros((0, 3))
        d = self.sim.d
        centre = self.sid.reshape(-1, self.sub)[:, 0]     # sub-ray 0 is the zone centre
        org = d.site_xpos[centre[ok]]
        zax = d.site_xmat[centre[ok]].reshape(-1, 3, 3)[:, :, 2]
        return org + r[ok][:, None] * zax


def stand_and_look(tilt, rise, tread, standoff, seed=0, height=TOF_Z, settle=0.8):
    """Drive nothing: place the robot at a standoff, let it balance, take one frame."""
    xml, n = xml_with_tof(tilt, rise, tread, height=height)
    sim = BotSim(xml_str=xml, seed=seed, randomize=True)
    sim.place(pitch=0.0, x=EDGE - standoff)
    sim.hold(0.4)
    sim.cmd.mode = 0
    sim.cmd.v = 0.0
    for _ in range(int(settle / CTRL_DT)):
        if sim.step().state != 4:
            sim.close()
            return None, None, None
    tof = Tof(sim, n, np.random.default_rng(seed))
    fr = tof.frame()
    pts = tof.cloud(fr)
    landing_z = float(sim.m.geom("landing").pos[2] + sim.m.geom("landing").size[2])
    on = ((np.abs(pts[:, 2] - (landing_z - rise)) < 0.02) & (pts[:, 0] > EDGE)
          & (pts[:, 0] < EDGE + tread)) if len(pts) else np.zeros(0, bool)
    sim.close()
    return fr, pts, dict(n_on_tread=int(on.sum()), landing_z=landing_z, n_valid=int(np.isfinite(fr).sum()))


def band(tilts, standoffs, rises, tread=0.30, trials=2):
    print(f"\n\033[1mZones landing on the first tread, 8 x 8 array at {TOF_Z*1000+54.3:.0f} mm above ground"
          f"\033[0m")
    print("   one frame, robot standing still, no nod. '--' means the tread was not reachable at all.\n")
    head = "  ".join(f"{r*1000:3.0f}mm" for r in rises)
    print(f"   {'tilt':>5} {'standoff':>9}   {head}")
    for t in tilts:
        for D in standoffs:
            cells = []
            for rise in rises:
                got = []
                for k in range(trials):
                    _, _, info = stand_and_look(math.radians(t), rise, tread, D, seed=500 + k)
                    got.append(info["n_on_tread"] if info else 0)
                m = int(np.mean(got))
                cells.append(f"{m:5d} " if m else "   -- ")
            print(f"   {t:4.0f}d {D*1000:8.0f} mm  {' '.join(cells)}")


# The reconstruction thresholds are the lidar's: it pours hundreds of returns onto a tread, so it can
# demand 80 of them. The array puts 5 to 18 zones there and that is plenty, because every one carries a
# bearing. Same rejection rules, counts scaled to the sensor.
TOF_MAP = dict(min_pts=3, min_tread_pts=4, min_span=0.020, max_gap=0.300, min_gap=-0.010)


def measure(tilt, standoffs, trials=40, seed0=20_000, height=TOF_Z):
    """One frame, standing still, through the same reconstruction the lidar work uses."""
    from stairs_map import reconstruct
    print(f"\n\033[1mRiser from a single frame, no nod: {trials} randomised staircases per standoff"
          f"\033[0m")
    print(f"   array {height*1000+54.3:.0f} mm above ground, {math.degrees(tilt):.0f} deg down\n")
    print(f"   {'standoff':>9} {'answered':>10} {'right (<=20mm)':>16} {'riser error':>18}")
    rows = []
    for D in standoffs:
        errs, ans = [], 0
        for k in range(trials):
            rng = np.random.default_rng(seed0 + k)
            rise = float(rng.uniform(0.08, 0.17))
            tread = float(rng.uniform(0.24, 0.32))
            fr, pts, info = stand_and_look(tilt, rise, tread, D, seed=seed0 + k, height=height)
            if info is None:
                continue
            r = reconstruct(pts, info["landing_z"], **TOF_MAP)
            if r["rise"] is not None:
                ans += 1
                errs.append(abs(r["rise"] - rise) * 1000)
        e = np.array(errs)
        ok = int((e <= 20).sum()) if len(e) else 0
        rows.append((D, ans, ok, e))
        print(f"   {D*1000:6.0f} mm {ans:7d}/{trials} {ok:12d}/{trials} "
              f"{(e.mean() if len(e) else float('nan')):11.1f} +-{(e.std() if len(e) else 0):5.1f} mm")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--band", action="store_true")
    ap.add_argument("--measure", action="store_true")
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--tilt", type=float, default=None)
    a = ap.parse_args()
    rises = (0.08, 0.10, 0.13, 0.15, 0.17)
    if a.measure:
        for t in (20.0, 25.0, 30.0):
            measure(math.radians(t), (0.25, 0.30, 0.35), trials=a.trials)
    elif a.band:
        band((20, 25, 30, 33, 36, 40), (0.25, 0.30, 0.35, 0.40), rises)
    elif a.tilt is not None:
        band((a.tilt,), (0.20, 0.25, 0.30, 0.35, 0.40, 0.50), rises, trials=3)
    else:
        band((33,), (0.30,), rises)


if __name__ == "__main__":
    main()
