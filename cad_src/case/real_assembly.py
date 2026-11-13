"""Fit check with the real bought-part models. Writes real_fit_report.json."""
import sys, json, itertools
import robot_paths as RP; RP.add_sources()
import cadquery as cq
exec(open(RP.source('assemble.py')).read().split("# ---------------- placeholder components")[0])   # printed parts + motors
import real_parts as rp
from real_parts import box as B

P = rp.real_parts()
Z0 = bc.SHELF_Z + bc.SHELF_T
LIST = {   # no published model: envelope from the listed size, placed where the design puts it
 "Battery Tattu R-Line 6S 1400 (listed 78 x 39 x 42)": (B(bc.BATT_X[0], bc.BATT_X[1], -35, 43, Z0, Z0 + bc.BATT_H), 222),
 "6S BMS board (listed 88 x 34 x 3.5)": (B(bc.BATT_X[0] - 4.0, bc.BATT_X[0] - 0.5, -44, 44, Z0, Z0 + 33.5), 20),
 "IP2368 USB-C PD board (est. 35 x 47 x 10)": (B(-13.5, 21.5, 67, 77, 71, 108), 20),
 "90° micro-USB OTG plug (cable to the LD19 adapter)": (B(-24.7, -16.7, bc.PI_CENTER[0] + 8.9 - 4.5, bc.PI_CENTER[0] + 8.9 + 4.5, bc.PI_CENTER[1] - 23, bc.PI_CENTER[1] - 15), 2),
 "LED eye panels 2 x (listed 26 x 26 x 3)": (cq.Compound.makeCompound([B(bc.X_IN_F - 1.1, bc.X_IN_F + 2.2, s * bc.EYE_Y - 13, s * bc.EYE_Y + 13, bc.EYE_Z - 13, bc.EYE_Z + 13) for s in (1, -1)]), 8),
 "GY-521 IMU (listed 21 x 16 x 3)": (B(-10, 6, -10.5, 10.5, ztr := bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4, ztr + 3), 2),
 "PDB + 30 A fuse (est. 16 x 13 x 9)": (B(-28, -12, -6.5, 6.5, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 9), 25),
 "Power switch board Pololu 2813 (listed 20 x 25.5 x 4 + wire tails, on the right wall)": (B(*bc.pwr_sw_box()), 3),
 "Power button 12 mm momentary (listed)": (rp.cyl((bc.POWER_BTN[0], bc.POWER_BTN[1], bc.Z_IN_TOP - 15), 12, 15, (0, 0, 1)), 3),
 "Arm button 12 mm (listed)": (rp.cyl((bc.BUTTON[0], bc.BUTTON[1], bc.Z_IN_TOP - 15), 12, 15, (0, 0, 1)), 3),
 "USB-C panel socket CAB-15455 (listed head 22 x 14 x 8)": (B(bc.USBC_SIDE[0] - 11, bc.USBC_SIDE[0] + 11, bc.Y_IN - 14, bc.Y_IN, bc.USBC_SIDE[1] - 4, bc.USBC_SIDE[1] + 4), 6),
 "LD19 USB adapter board (est. 18 x 12 x 6)": (B(-14, 4, 44, 56, Z0 + 7, Z0 + 13), 5),
 "Charge diode + fuse, inline (est.)": (B(-10, 8, -52, -46, Z0 + 2, Z0 + 9), 5),
 "Front rubber bumpers 4 x (Ø11 x 2.5, 3M Bumpon type)": (cq.Compound.makeCompound([rp.cyl((bc.X_FRONT, s * y, z), bc.BUMPER_D, 2.5, (1, 0, 0)) for (y, z) in bc.FRONT_BUMPERS for s in (1, -1)]), 1),
 "COB disc headlights 2 x (listed Ø30 x 4 + solder blobs)": (cq.Compound.makeCompound([rp.cyl((bc.X_IN_F - bc.LIGHT_T, s * bc.LIGHT_Y, bc.LIGHT_Z), bc.LIGHT_D, bc.LIGHT_T, (1, 0, 0)) for s in (1, -1)]
     + [B(bc.X_IN_F - bc.LIGHT_T - 1.5, bc.X_IN_F - bc.LIGHT_T, s * bc.LIGHT_Y - 6, s * bc.LIGHT_Y + 6, bc.LIGHT_Z - 2, bc.LIGHT_Z + 2) for s in (1, -1)]), 8),
 "TSOP38238 IR receivers 4 x (listed 5.6 x 6.0 x 7.0, in the hoods)": (cq.Compound.makeCompound([
     B(-2.8, 2.0, -2.8, 2.8, 1.4, 8.0).rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), f).translate(cq.Vector(x, y, bc.Z_TOP)) for (x, y, f) in bc.IR_RX]), 2),
 "Headlight driver board (D36V6F12 + IRLB8721 on 25 x 20 perfboard, est.)": (cq.Compound.makeCompound([
     B(bc.X_IN_F - bc.DRV_STANDOFF - 1.6, bc.X_IN_F - bc.DRV_STANDOFF, bc.DRV_CENTER[0] - 12.7, bc.DRV_CENTER[0] + 12.7, bc.DRV_CENTER[1] - 10.15, bc.DRV_CENTER[1] + 10.15),
     B(bc.X_IN_F - bc.DRV_STANDOFF - 7.6, bc.X_IN_F - bc.DRV_STANDOFF - 1.6, bc.DRV_CENTER[0] - 10, bc.DRV_CENTER[0] + 10, bc.DRV_CENTER[1] - 7, bc.DRV_CENTER[1] + 7)]), 6),
}
DRAWN = {"Motor plugs + leads L (LK drawing)": (motor_plugs_fn := None, 6)}

def motor_plugs(s):
    # same source as assemble.py's: motors.plug_boxes. These two used to hold a hand-copied MF6015
    # envelope each, so a Ø69 motor was reported as clashing with its own connectors.
    return cq.Compound.makeCompound([B(*b) for b in bc._motors.plug_boxes(bc.MOTOR, s, bc.MOTOR_BACK_Y)])

ALL = {}
for k, (shp, src, g) in P.items(): ALL[k] = (shp, src, g)
for k, (shp, g) in LIST.items(): ALL[k] = (shp, "LIST envelope", g)
ALL["Motor plugs L (from LK drawing)"] = (motor_plugs(1), "DRAW LK drawing, plug sizes estimated", 3)
ALL["Motor plugs R (from LK drawing)"] = (motor_plugs(-1), "DRAW LK drawing, plug sizes estimated", 3)
PRINTED = {"body": body, "front panel": front, "back cover": cover, "shelf": shelf, "contact plate": cap, "tray": tray,
           "eye retainer L": ret_l, "eye retainer R": ret_r, "carrier plate (IP2368)": plate_l, "speaker ring": ring,
           "mic clamp L": micc_l, "mic clamp R": micc_r,
           "headlight retainer L": lret_l, "headlight retainer R": lret_r, "IR hoods 4 x": bc.ir_hoods_placed().val()}
for s_ in (1, -1):
    PRINTED[f"motor {'L' if s_ > 0 else 'R'} (LK STEP)"] = place(motor, s_)
    PRINTED[f"wheel adapter {'L' if s_ > 0 else 'R'}"] = place(adapter, s_)

def vol(a, b):
    try:
        bb1, bb2 = a.BoundingBox(), b.BoundingBox()
        if bb1.xmax < bb2.xmin or bb2.xmax < bb1.xmin or bb1.ymax < bb2.ymin or bb2.ymax < bb1.ymin or bb1.zmax < bb2.zmin or bb2.zmax < bb1.zmin:
            return 0.0
        return a.intersect(b).Volume()
    except Exception:
        return float("nan")

def kind(a, b):
    """'contact' if every overlap piece is thinner than 0.15 mm (parts seated on each other), else 'CLASH'."""
    i = a.intersect(b)
    for so in i.Solids():
        bb = so.BoundingBox()
        if min(bb.xlen, bb.ylen, bb.zlen) > 0.15: return "CLASH"
    return "contact"

if __name__ == "__main__":
    rep = {"parts": {}, "clashes": []}
    names = list(ALL)
    for n in names:
        rep["parts"][n] = ALL[n][1]
    for n in names:                                   # bought vs printed
        for m, ps in PRINTED.items():
            v = vol(ALL[n][0], ps)
            if v > 0.3: rep["clashes"].append([n, m, round(v, 1), kind(ALL[n][0], ps)])
    for a, b in itertools.combinations(names, 2):     # bought vs bought
        v = vol(ALL[a][0], ALL[b][0])
        if v > 0.3: rep["clashes"].append([a, b, round(v, 1), kind(ALL[a][0], ALL[b][0])])
    json.dump(rep, open(RP.out('real_fit_report.json'), 'w'), indent=1)
    for c in rep["clashes"]: print("CLASH", c)
    print(len(rep["clashes"]), "clashes")
