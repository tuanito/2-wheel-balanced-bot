"""Parametric MF6015 -> 12 mm hex RC wheel adapter (CadQuery).

Print orientation: hex end DOWN on the bed. The 45-degree cone under the flange
means no supports are needed, and the motor-facing flange and centering spigot
come out as clean top surfaces.

Run:  python3 wheel_adapter.py   -> writes STEP + STL next to this script.
"""
import cadquery as cq
import math, pathlib

# ---------------- motor interface (from LKMTECH MF6015 v2 drawing + STEP) ------
BORE_D        = 12.7   # motor centering bore, H7 (1/2")
SPIGOT_D      = 12.6   # printed spigot: tune 12.55-12.70 for a snug slide fit
SPIGOT_L      = 4.0    # bore is ~17 mm deep, 4 mm is plenty for centering
PCD           = 25.0   # 4 x M3 bolt circle
M3_CLEAR      = 3.4
M3_HEAD_CB    = 6.0    # counterbore for M3 socket head (5.5 mm head)

# ---------------- adapter body -------------------------------------------------
FLANGE_D      = 36.0
FLANGE_T      = 5.5    # plate under screw heads. M3x8 -> 2.5 mm into motor
NECK_D        = 18.0
NECK_L        = 6.0    # standoff so the wheel clears the motor - set per wheel
HEX_AF        = 11.95  # 12 mm hex across flats, minus a print tolerance
HEX_L         = 7.0    # match the depth of your wheel's hex pocket (it must bottom out)
M4_INSERT_D   = 5.6    # hole for M4 heat-set insert (check your insert's spec)
M4_INSERT_L   = 9.0
CHAMFER       = 0.6
BRIDGE_LAYER  = 0.2    # layer height used by the two-layer bridge at the screw-head seats

CONE_H = (FLANGE_D - NECK_D) / 2          # 45 degrees -> support-free


def build():
    # z = 0 is the motor mating face; +z goes into the motor, -z toward the wheel.
    spigot = (cq.Workplane("XY").circle(SPIGOT_D / 2).extrude(SPIGOT_L)
              .faces(">Z").edges().chamfer(0.8))
    flange = cq.Workplane("XY").circle(FLANGE_D / 2).extrude(-FLANGE_T)
    cone = (cq.Workplane("XY").workplane(offset=-FLANGE_T)
            .circle(FLANGE_D / 2).workplane(offset=-CONE_H)
            .circle(NECK_D / 2).loft())
    z0 = -FLANGE_T - CONE_H
    neck = cq.Workplane("XY").workplane(offset=z0).circle(NECK_D / 2).extrude(-NECK_L)
    z1 = z0 - NECK_L
    hexp = (cq.Workplane("XY").workplane(offset=z1)
            .polygon(6, HEX_AF / math.cos(math.radians(30))).extrude(-HEX_L)
            .faces("<Z").edges().chamfer(CHAMFER))
    body = spigot.union(flange).union(cone).union(neck).union(hexp)

    zb = z1 - HEX_L
    # M4 heat-set insert from the hex end, with a 45 deg pointed end (prints without a bridge, hex-down)
    body = body.cut(cq.Workplane("XY").workplane(offset=zb)
                    .circle(M4_INSERT_D / 2).extrude(M4_INSERT_L))
    body = body.cut(cq.Solid.makeCone(M4_INSERT_D / 2, 0.01, M4_INSERT_D / 2,
                                      cq.Vector(0, 0, zb + M4_INSERT_L), cq.Vector(0, 0, 1)))
    # 4 x M3 clearance holes + counterbores opening on the wheel side. The head seat is printed with the
    # two-layer bridge trick (hex-down): layer 1 leaves a slot as wide as the hole, layer 2 a square, then the round
    # hole: clean bridges, no sacrificial membrane to drill out, no support.
    pts_a = [(a, PCD / 2 * math.cos(math.radians(a)), PCD / 2 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
    pts = [(x, y) for _, x, y in pts_a]
    seat = -FLANGE_T
    body = body.cut(cq.Workplane("XY").workplane(offset=seat + 2 * BRIDGE_LAYER)
                    .pushPoints(pts).circle(M3_CLEAR / 2).extrude(FLANGE_T + 1))
    for a, x, y in pts_a:
        slot = (cq.Workplane("XY").workplane(offset=seat).center(x, y)
                .rect(M3_HEAD_CB + 0.4, M3_CLEAR).extrude(BRIDGE_LAYER).rotate((x, y, 0), (x, y, 1), a))
        sq = (cq.Workplane("XY").workplane(offset=seat + BRIDGE_LAYER).center(x, y)
              .rect(M3_CLEAR, M3_CLEAR).extrude(BRIDGE_LAYER + 0.01).rotate((x, y, 0), (x, y, 1), a))
        body = body.cut(slot.intersect(cq.Workplane("XY").workplane(offset=seat).center(x, y).circle(M3_HEAD_CB / 2).extrude(BRIDGE_LAYER))).cut(sq)
    body = body.cut(cq.Workplane("XY").workplane(offset=seat)
                    .pushPoints(pts).circle(M3_HEAD_CB / 2).extrude(-(CONE_H + 1)))
    # outer edge chamfer on the mating flange
    return body


if __name__ == "__main__":
    out = pathlib.Path(__file__).parent
    part = build()
    cq.exporters.export(part, str(out / "mf6015_hex12_adapter.step"))
    cq.exporters.export(part, str(out / "mf6015_hex12_adapter.stl"), tolerance=0.02, angularTolerance=0.1)
    bb = part.val().BoundingBox()
    print("length %.1f mm, volume %.1f cm3, PETG mass ~%.0f g"
          % (bb.zlen, part.val().Volume() / 1000, part.val().Volume() / 1000 * 1.27))
