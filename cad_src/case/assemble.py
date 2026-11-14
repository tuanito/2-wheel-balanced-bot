"""Full robot assembly: case + motors + adapters + wheel proxies + placeholder components.
Checks interference, ground contact when lying flat, centre of mass; renders views with MuJoCo.
"""
import sys, math, pathlib, os
import robot_paths as RP; RP.add_sources()
import functools

import cadquery as cq
import body_case as bc
import wheel_adapter as wa
from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.TopoDS import TopoDS

OUT = RP.ASSEMBLY; OUT.mkdir(parents=True, exist_ok=True)

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0)).val()

# ---------------- motor (sewn into solids so boolean checks work) ----------------
m = cq.importers.importStep(str(RP.motor_step(bc.MOTOR))).val()
sew = BRepBuilderAPI_Sewing(0.01); sew.Add(m.wrapped); sew.Perform()
msol = []
for s in cq.Shape.cast(sew.SewedShape()).Shells():
    so = cq.Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(s.wrapped)).Solid()); so.fix()
    if abs(so.Volume()) > 1: msol.append(so)
motor = cq.Compound.makeCompound(msol)
# Where the motor sits in its OWN STEP's frame, and which way its axis runs. These are not
# transferable between motors: the MF6015's STEP has the rear face at x = +29 with the axis running
# +x from the output face, while the L-7015's has the rear at x = -33.9 with the axis running -x. Get
# the sign wrong and the motor is placed inside out, crossing the centreline, with no error.
_AX = bc._M["step_axis"]                   # +1 (mf6015) or -1 (mf7015)
_BACK = abs(bc._M["step_back_face"])       # 29.0 (mf6015) or 33.9 (mf7015)
_FACE = bc._M["step_back_face"] - _AX * bc.MOTOR_LEN   # output face x: mf6015 -5.0, mf7015 0.0
adapter = wa.build().val().rotate((0, 0, 0), (0, 1, 0), 90 * _AX).translate((_FACE, 0, 0))  # motor frame
yb = bc.MOTOR_BACK_Y                       # motor back face position
def place(shape, side):
    # rear face lands at |y| = yb with the can extending outboard, for either axis direction
    if side > 0:
        return shape.rotate((0, 0, 0), (0, 0, 1), -90 * _AX).translate((0, yb + _BACK, 0))
    return shape.rotate((0, 0, 0), (0, 0, 1), 90 * _AX).translate((0, -yb - _BACK, 0))

wheel_y0 = bc.WHEEL_IN_Y        # wheel inner face (set NECK_L in wheel_adapter.py to match)
def wheel(side):
    w = cq.Workplane("XZ", origin=(0, 0, 0)).circle(bc.WHEEL_R).circle(26).extrude(-45).val()
    return w.translate((0, wheel_y0, 0)) if side > 0 else w.translate((0, -wheel_y0 - 45, 0))

body, cover, front, shelf = bc.build_body().val(), bc.build_cover().val(), bc.build_front().val(), bc.build_shelf().val()
cap = bc.build_contact_plate().val()
tray = bc.build_tray().val()
ret_l, ret_r = bc.build_eye_retainer(1).val(), bc.build_eye_retainer(-1).val()
def plate_at(y_face, side):   # carrier plate against the wall bosses, plane XZ
    p = bc.build_carrier_plate().val().rotate((0, 0, 0), (1, 0, 0), 90)   # now in XZ, thickness along -Y
    p = p.translate((3.0, 0, 92.0))
    return p.translate((0, y_face, 0)) if side < 0 else p.mirror("XZ").translate((0, y_face, 0))
plate_l = plate_at(bc.Y_IN - bc.WALL_BOSS_L, 1)
ring = bc.build_speaker_ring().val()                      # right wall: speaker ring (+ amp tab) instead of a carrier plate
micc_l, micc_r = bc.build_mic_clamp(1).val(), bc.build_mic_clamp(-1).val()
lret_l, lret_r = bc.build_light_retainer(1).val(), bc.build_light_retainer(-1).val()

# ---------------- placeholder components (mass g, shape) ----------------
L = bc
lid_z = bc.Z_TOP
Z0 = bc.SHELF_Z + bc.SHELF_T
eye = box(bc.X_IN_F - 1.1, bc.X_IN_F + 2.2, bc.EYE_Y - 13, bc.EYE_Y + 13, bc.EYE_Z - 13, bc.EYE_Z + 13)
# motor plugs (from the LK drawing: a 6-pin and a 3-pin header side by side in a 15 mm bay on top of the Ø49 boss,
# plugs inserted downward). Envelope: plug bodies up to z 29, leads bend up and inward through the wall slot.
def motor_plugs(s):
    # shape comes from motors.plug_boxes: ONE definition, shared with real_assembly.py
    bs = [box(*b) for b in bc._motors.plug_boxes(bc.MOTOR, s, bc.MOTOR_BACK_Y, with_leads=True)]
    return functools.reduce(lambda a, b: a.fuse(b), bs)
comps = {
 "motor plugs + leads (left)": (6, motor_plugs(1)),
 "motor plugs + leads (right)": (6, motor_plugs(-1)),
 "battery 6S 1400":   (222, box(bc.BATT_X[0], bc.BATT_X[1], -35, 43, Z0, Z0 + bc.BATT_H)),
 "regulator D24V50F5 (tray)": (3, box(6.35, 26.65, -8.9, 8.9, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4 + 8.8)),
 "PDB + fuse (tray, under clamp)": (25, box(-28, -12, -6.5, 6.5, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 9)),
 "IMU GY-521 (tray, near the axle)": (2, box(-10, 6, -10.5, 10.5, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4 + 3)),
 "sonar HC-SR04":     (9, box(bc.X_IN_F - bc.SONAR_POST - 3.5, bc.X_IN_F - bc.SONAR_POST, -22.5, 22.5, bc.SONAR_Z - 10, bc.SONAR_Z + 10).fuse(
        cq.Workplane("YZ", origin=(bc.X_IN_F - bc.SONAR_POST, 0, 0)).pushPoints([(-13, bc.SONAR_Z), (13, bc.SONAR_Z)]).circle(8).extrude(12).val())),
 "Pi Zero 2 W + heatsink": (16, box(bc.X_BACK + 9, bc.X_BACK + 16, bc.PI_CENTER[0] - 32.5, bc.PI_CENTER[0] + 32.5, bc.PI_CENTER[1] - 15, bc.PI_CENTER[1] + 15)),
 "Camera Module 3":   (4, box(bc.X_IN_F - bc.STANDOFF_F - 3.4, bc.X_IN_F - bc.STANDOFF_F, -12.5, 12.5, bc.CAM_HOLE_Z - 18.5, bc.CAM_HOLE_Z + 8.3)),
 "perfboard + ESP32 + CAN board": (35, box(bc.X_BACK + 9, bc.X_BACK + 29, bc.PB_CENTER[0] - 35, bc.PB_CENTER[0] + 35, bc.PB_CENTER[1] - 25, bc.PB_CENTER[1] + 25)),
 "arm button":        (3, cq.Workplane("XY").center(bc.BUTTON[0], bc.BUTTON[1]).circle(6).extrude(15).val().translate((0, 0, bc.Z_IN_TOP - 15))),
 "power switch board (Pololu 2813)": (3, box(*bc.pwr_sw_box())),
 "power button":      (3, cq.Workplane("XY").center(bc.POWER_BTN[0], bc.POWER_BTN[1]).circle(6).extrude(15).val().translate((0, 0, bc.Z_IN_TOP - 15))),
 "LED eyes (2 x 8x8 WS2812B-2020)": (8, eye.fuse(eye.mirror("XZ"))),
 "lidar LD19":        (47, box(-19.3, 19.3, -19.3, 19.3, lid_z, lid_z + 34.8)),
 "lidar adapter board (on carrier plate)": (5, box(-8, 10, 44, 56, Z0 + 7, Z0 + 13)),
 "brass contacts (M5 countersunk, 8 mm down the funnels)": (4, cq.Workplane("YZ", origin=(bc.X_BACK - bc.COVER_T - bc.CONTACT_T + bc.FUNNEL[1], 0, 0))
        .pushPoints(list(bc.PADS.values())).circle(bc.PAD_HEAD / 2).extrude(0.2).val()),
 "USB-C PD charger IP2368 (left carrier plate)": (20, box(-13.5, 21.5, 67, 77, 71, 108)),
 "USB-C panel socket (left side wall)": (6, box(bc.USBC_SIDE[0] - 11, bc.USBC_SIDE[0] + 11, bc.Y_IN - 14, bc.Y_IN, bc.USBC_SIDE[1] - 4, bc.USBC_SIDE[1] + 4)),
 "charge diode + fuse (inline in the charge lead)": (5, box(-10, 8, -52, -46, Z0 + 2, Z0 + 9)),
 "I2S amp MAX98357A (on the speaker ring tab)": (2, box(-11, 8, bc.SPK_Y0 + bc.SPK_FLANGE + bc.RING_T, bc.SPK_Y0 + bc.SPK_FLANGE + bc.RING_T + 9.5, 105, 123)),
 "speaker 40 mm 4 ohm (right wall)": (27, cq.Workplane("XZ", origin=(0, bc.SPK_Y0, 0)).center(*bc.SPK_XZ).circle(bc.SPK_D / 2).extrude(-bc.SPK_FLANGE).val()
        .fuse(cq.Workplane("XZ", origin=(0, bc.SPK_Y0 + bc.SPK_FLANGE, 0)).center(*bc.SPK_XZ).circle(bc.SPK_MAG_D / 2).extrude(-14).val())),
 "mic ICS-43434 L (front panel)": (1, box(bc.X_IN_F + bc.MIC_POCKET - 1.5, bc.X_IN_F + bc.MIC_POCKET - 0.05, bc.MIC_YZ[0] - 0.76 - 6.35, bc.MIC_YZ[0] - 0.76 + 6.35, bc.MIC_YZ[1] - 8.5, bc.MIC_YZ[1] + 8.5)
        .fuse(box(bc.X_IN_F + bc.MIC_POCKET - 2.6, bc.X_IN_F + bc.MIC_POCKET - 1.6, bc.MIC_YZ[0] - 2, bc.MIC_YZ[0] + 2, bc.MIC_YZ[1] - 2, bc.MIC_YZ[1] + 2))),
 "mic ICS-43434 R (front panel)": (1, box(bc.X_IN_F + bc.MIC_POCKET - 1.5, bc.X_IN_F + bc.MIC_POCKET - 0.05, -bc.MIC_YZ[0] - 0.76 - 6.35, -bc.MIC_YZ[0] - 0.76 + 6.35, bc.MIC_YZ[1] - 8.5, bc.MIC_YZ[1] + 8.5)
        .fuse(box(bc.X_IN_F + bc.MIC_POCKET - 2.6, bc.X_IN_F + bc.MIC_POCKET - 1.6, -bc.MIC_YZ[0] - 2, -bc.MIC_YZ[0] + 2, bc.MIC_YZ[1] - 2, bc.MIC_YZ[1] + 2))),
 "6S BMS (upright, on the back of the battery)": (20, box(bc.BATT_X[0] - 4.0, bc.BATT_X[0] - 0.5, -44, 44, Z0, Z0 + 33.5)),
}
wires_g = 40   # leads, CAN, USB, connectors (smeared around z~40)

def vol(a, b):
    try: return a.intersect(b).Volume()
    except Exception: return float("nan")

print("== interference (mm3, 0 = clear) ==")
parts_case = {"body": body, "cover": cover, "front": front, "shelf": shelf, "cap": cap, "tray": tray, "ret_l": ret_l, "ret_r": ret_r, "plate_l": plate_l, "ring": ring, "micc_l": micc_l, "micc_r": micc_r, "lret_l": lret_l, "lret_r": lret_r}
for side in (1, -1):
    mo = place(motor, side); ad = place(adapter, side); wh = wheel(side)
    parts_case[f"motor{side:+d}"] = mo
    print(f"motor {side:+d} vs body: {vol(mo, body):.2f}   adapter vs body: {vol(ad, body):.2f}   wheel vs body: {vol(wh, body):.2f}   wheel vs motor: {vol(wh, mo):.2f}")
for name, (g, shp) in comps.items():
    hits = {k: vol(shp, v) for k, v in parts_case.items()}
    bad = {k: round(v, 1) for k, v in hits.items() if v > 0.5}
    print(f"{name:40s} {'OK' if not bad else bad}")
names = list(comps)
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        v = vol(comps[names[i]][1], comps[names[j]][1])
        if v > 0.5: print(f"  CLASH {names[i]} <-> {names[j]}: {v:.0f} mm3")

# ---------------- centre of mass ----------------
def centroid(shape):
    c = shape.Center(); return c.x, c.y, c.z
rho = 1.27e-3 * 0.75   # g/mm3 PETG at typical infill
items = [("body (printed)", body.Volume() * rho, centroid(body)),
         ("cover (printed)", cover.Volume() * rho, centroid(cover)),
         ("front panel (printed)", front.Volume() * rho, centroid(front)),
         ("battery shelf (printed)", shelf.Volume() * rho, centroid(shelf)),
         ("contact plate (printed)", cap.Volume() * rho, centroid(cap)),
         ("tray + plates + retainers (printed)", (tray.Volume() + ret_l.Volume() + ret_r.Volume() + plate_l.Volume()) * rho, (0, 0, 20)),
         ("speaker ring + mic clamps (printed)", (ring.Volume() + micc_l.Volume() + micc_r.Volume()) * rho, (0, -60, 95)),
         ("headlight retainers (printed)", (lret_l.Volume() + lret_r.Volume()) * rho, (26, 0, bc.LIGHT_Z))]
for name, (g, shp) in comps.items(): items.append((name, g, centroid(shp)))
items += [("wires/connectors", wires_g, (0, 0, 30)),
          ("2 motors", 502, (0, 0, 0)), ("2 wheels", 220, (0, 0, 0)), ("2 adapters + hardware", 45, (0, 0, 0))]
M = sum(g for _, g, _ in items)
cx = sum(g * c[0] for _, g, c in items) / M; cz = sum(g * c[2] for _, g, c in items) / M
body_items = [it for it in items if it[0] not in ("2 wheels", "2 adapters + hardware")]
Mb = sum(g for _, g, _ in body_items)
bz = sum(g * c[2] for _, g, c in body_items) / Mb
print(f"\n== mass ==\n total {M:.0f} g; CoM of whole robot: x {cx:+.1f} mm, z {cz:+.1f} mm above the axle")
print(f" pendulum part (everything but wheels/adapters): {Mb:.0f} g, CoM {bz:.1f} mm above the axle")
for n, g, c in sorted(items, key=lambda t: -t[1]): print(f"   {n:40s} {g:6.0f} g  at z {c[2]:6.1f}")

# ---------------- lying flat: which point touches the ground first ----------------
prof = [("body top-front edge", bc.X_FRONT, bc.Z_TOP), ("body bottom-front edge", bc.X_FRONT, bc.Z_BOT),
        ("lidar top-front", 19.3, lid_z + 34.8), ("lidar top-back", -19.3, lid_z + 34.8),
        ("lidar back tab", bc.LIDAR_SINGLE_X - 3.5, lid_z + 3),
        ("body top-back (cover)", bc.X_BACK - bc.COVER_T, bc.Z_TOP), ("body bottom-back", bc.X_BACK - bc.COVER_T, bc.Z_BOT),
        ("contact plate top", bc.X_BACK - bc.COVER_T - bc.CONTACT_T, bc.CONTACT_PLATE[3] - 0.5)]
R = bc.WHEEL_R
def lie_angle(x, z, direction):
    # tilt t (rad) toward +x (direction=+1) or -x: point height = -x*sin t*dir + z*cos t ; touches when = -R
    best = None
    for k in range(1, 18000):
        t = k * math.pi / 18000
        h = -direction * x * math.sin(t) + z * math.cos(t)
        if h <= -R: best = t; break
    return best
print("\n== lying flat (wheel radius %.1f) ==" % R)
for d, label in ((1, "face down (front)"), (-1, "back down")):
    res = [(lie_angle(x, z, d), n) for n, x, z in prof]
    res = [(t, n) for t, n in res if t]
    t, n = min(res)
    print(f" {label}: rests at {math.degrees(t):.0f} deg from upright on '{n}'")
    for tt, nn in sorted(res)[:3]: print(f"     {nn:25s} would touch at {math.degrees(tt):.0f} deg")

# ---------------- export meshes for rendering ----------------
mesh_parts = {"body": (body, "0.24 0.48 0.79 1"), "cover": (cover, "0.30 0.55 0.85 1"), "front": (front, "0.30 0.55 0.85 1"), "shelf": (shelf, "0.36 0.62 0.90 1"), "cap": (cap, "0.30 0.55 0.85 1"), "tray": (tray, "0.95 0.6 0.2 1"),
              "ret_l": (ret_l, "0.95 0.6 0.2 1"), "ret_r": (ret_r, "0.95 0.6 0.2 1"), "plate_l": (plate_l, "0.95 0.6 0.2 1"), "ring": (ring, "0.95 0.6 0.2 1"),
              "micc_l": (micc_l, "0.95 0.6 0.2 1"), "micc_r": (micc_r, "0.95 0.6 0.2 1"), "lret_l": (lret_l, "0.95 0.6 0.2 1"), "lret_r": (lret_r, "0.95 0.6 0.2 1")}
for side in (1, -1):
    mesh_parts[f"motor{side:+d}"] = (place(motor, side), "0.62 0.65 0.69 1")
    mesh_parts[f"adapter{side:+d}"] = (place(adapter, side), "0.88 0.54 0.18 1")
    mesh_parts[f"wheel{side:+d}"] = (wheel(side), "0.12 0.12 0.13 1")
colors = ["0.85 0.75 0.2 1", "0.4 0.4 0.45 1", "0.55 0.3 0.7 1", "0.25 0.25 0.3 1", "0.75 0.25 0.2 1", "0.75 0.25 0.2 1", "0.3 0.7 0.3 1", "0.8 0.3 0.3 1", "0.2 0.6 0.6 1", "0.15 0.5 0.2 1",
          "0.5 0.5 0.9 1", "0.2 0.4 0.2 1", "0.9 0.2 0.2 1", "0.9 0.85 0.1 1", "0.15 0.15 0.15 1", "0.4 0.4 0.4 1",
          "0.72 0.45 0.2 1", "0.4 0.6 0.3 1", "0.3 0.5 0.8 1", "0.6 0.3 0.6 1", "0.5 0.5 0.5 1"]
for (name, (g, shp)), c in zip(comps.items(), colors):
    mesh_parts["c_" + "".join(ch for ch in name.split("(")[0] if ch.isalnum())[:24]] = (shp, c)
for k, (shp, c) in mesh_parts.items():
    cq.exporters.export(shp, str(OUT / f"{k}.stl"), tolerance=0.2, angularTolerance=0.3)
cq.exporters.export(cq.Compound.makeCompound([s for s, _ in mesh_parts.values()]), str(OUT / "robot_assembly.step"))
open(OUT / "parts.txt", "w").write("\n".join(f"{k}|{c}" for k, (s, c) in mesh_parts.items()))
print("\nexported", len(mesh_parts), "meshes")

# ---------------- sim data (robot/sim: cad_mass.json + meshes) ----------------
import json
SIM = RP.SIM; (SIM / "meshes").mkdir(parents=True, exist_ok=True)
printed = {"body": body, "cover": cover, "front": front, "shelf": shelf, "cap": cap, "tray": tray, "ret_l": ret_l, "ret_r": ret_r,
           "plate_l": plate_l, "ring": ring, "micc_l": micc_l, "micc_r": micc_r, "lret_l": lret_l, "lret_r": lret_r}
pm = {}
for k, v in printed.items():
    cq.exporters.export(v, str(SIM / "meshes" / f"{k}.stl"), tolerance=0.1, angularTolerance=0.2); pm[k] = v.Volume() * rho
cj = {}
for name, (g, shp) in comps.items():
    bb = shp.BoundingBox(); cj[name] = {"mass": g, "lo": [bb.xmin, bb.ymin, bb.zmin], "hi": [bb.xmax, bb.ymax, bb.zmax]}
# z_top / x_in_f / motor_len: build_model.py had these as literals (lid_z 0.152, the sonar site at
# x 0.032), which are MF6015 panel numbers - on a deeper, taller body the lidar scan plane and the
# front sensor would have sat in the wrong place with nothing to say so.
json.dump({"version": "V1.1", "printed": pm, "comps": cj, "R": bc.WHEEL_R, "wheel_y": [bc.WHEEL_IN_Y, bc.WHEEL_OUT_Y],
           "motor_back_y": bc.MOTOR_BACK_Y, "motor": bc.MOTOR, "z_top": bc.Z_TOP, "x_in_f": bc.X_IN_F,
           "motor_len": bc.MOTOR_LEN, "sonar_z": bc.SONAR_Z}, open(SIM / "cad_mass.json", "w"), indent=1)
print("sim data written")
