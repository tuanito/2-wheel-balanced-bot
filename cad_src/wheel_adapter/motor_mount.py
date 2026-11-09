"""Parametric MF6015 motor-mount wall section (CadQuery).

The motor's fixed back face bolts to this wall: 4 x M2.5 on a 20 x 20 mm square
(8 mm deep threads in the motor). A shallow pocket centers the Ø49 back section.
A slot over the connector notch lets the plugs go in radially and the cables pass
through into the body.

Use it either as a standalone plate (4 corner M3 holes -> heat-set inserts in
the chassis) or as a reference: import the STEP into your chassis CAD and
boolean it into the side wall.

Coordinates match the LKMTECH STEP: motor axis = X, back face at x = 29,
connector notch toward +Z.

Run:  python3 motor_mount.py
"""
import cadquery as cq
import pathlib

BACK_FACE_X  = 29.0    # motor back face in the STEP's frame
BOSS_D       = 49.0    # back section, Ø49 -0.05
POCKET_D     = 49.4    # tune 49.2-49.6 for a snug, centered fit
POCKET_DEPTH = 2.0
WALL_T       = 7.0     # total wall thickness -> 5 mm under the screw heads
PLATE_W      = 76.0    # square plate size (standalone version)
CORNER_R     = 6.0
M25_CLEAR    = 2.9
M25_HEAD_CB  = 5.0     # counterbore Ø for M2.5 socket head (4.5 mm head)
M25_CB_DEPTH = 0.0     # 0 = heads sit on the inner face; >0 to sink them
SQUARE       = 20.0    # motor bolt square
SLOT_W       = 22.0    # connector notch is ~17.4 wide; plugs need a bit more
SLOT_Z0      = 12.0    # slot starts this far above the axis (connector base)
SLOT_Z1      = 30.0
M3_CLEAR     = 3.4     # corner holes to bolt the plate to the chassis
M3_INSET     = 7.0


def build():
    x_out = BACK_FACE_X - POCKET_DEPTH            # face toward the motor
    # plate: sketch in YZ, extrude +X (away from the motor)
    wp = cq.Workplane("YZ", origin=(x_out, 0, 0))
    plate = wp.rect(PLATE_W, PLATE_W).extrude(WALL_T).edges("|X").fillet(CORNER_R)
    # centering pocket
    plate = plate.cut(cq.Workplane("YZ", origin=(x_out, 0, 0)).circle(POCKET_D / 2).extrude(POCKET_DEPTH))
    # connector slot through the wall
    zc = (SLOT_Z0 + SLOT_Z1) / 2
    plate = plate.cut(cq.Workplane("YZ", origin=(x_out - 1, 0, 0)).center(0, zc)
                      .rect(SLOT_W, SLOT_Z1 - SLOT_Z0).extrude(WALL_T + 2)
                      .edges("|X").fillet(2.0))
    # motor screws (inserted from inside the body)
    h = SQUARE / 2
    pts = [(-h, -h), (h, -h), (h, h), (-h, h)]
    plate = plate.cut(cq.Workplane("YZ", origin=(x_out - 1, 0, 0)).pushPoints(pts)
                      .circle(M25_CLEAR / 2).extrude(WALL_T + 2))
    if M25_CB_DEPTH > 0:
        plate = plate.cut(cq.Workplane("YZ", origin=(x_out + WALL_T - M25_CB_DEPTH, 0, 0))
                          .pushPoints(pts).circle(M25_HEAD_CB / 2).extrude(M25_CB_DEPTH + 1))
    # chassis mounting holes
    c = PLATE_W / 2 - M3_INSET
    plate = plate.cut(cq.Workplane("YZ", origin=(x_out - 1, 0, 0))
                      .pushPoints([(-c, -c), (c, -c), (c, c), (-c, c)])
                      .circle(M3_CLEAR / 2).extrude(WALL_T + 2))
    return plate


if __name__ == "__main__":
    out = pathlib.Path(__file__).parent
    part = build()
    cq.exporters.export(part, str(out / "mf6015_motor_mount.step"))
    cq.exporters.export(part, str(out / "mf6015_motor_mount.stl"), tolerance=0.02, angularTolerance=0.1)
    under_head = WALL_T - POCKET_DEPTH - M25_CB_DEPTH
    print("plate %.0f x %.0f x %.0f mm, ~%.0f g PETG; material under M2.5 heads %.1f mm "
          "-> M2.5 x %d screws (thread engagement %.1f of 8 mm)"
          % (PLATE_W, PLATE_W, WALL_T, part.val().Volume() / 1000 * 1.27, under_head,
             int(under_head + 5), 5.0))
