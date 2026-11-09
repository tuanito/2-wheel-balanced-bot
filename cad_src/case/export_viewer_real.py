"""Viewer data with the REAL bought-part models and the wiring (replaces the placeholder blocks)."""
import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import sys, json
src = open(WS + 'case/export_viewer_data.py').read()
head = src.split("cols={")[0]                         # imports, mesh(), printed parts, motors, adapters, wheels
tail = src[src.index("# vertical charging dock"):]               # dock + json dump
exec(head)
exec(open(WS + 'case/real_assembly.py').read().split('if __name__ == "__main__":')[0].split("exec(open(WS + 'case/assemble.py')")[0])
import real_parts as rp, wiring as W
from real_parts import box as B
P = rp.real_parts()
Z0 = bc.SHELF_Z + bc.SHELF_T
# name (first word = viewer key, matched by the labels), shape, colour, explode, source
items = [
 ("battery", "Battery Tattu R-Line 6S 1400", B(bc.BATT_X[0], bc.BATT_X[1], -35, 43, Z0, Z0 + bc.BATT_H), "#e3c832", [-40, 0, 40], "LIST: outer size from the Tattu listing (78 x 39 x 42)"),
 ("6S", "6S BMS (on the back of the battery)", B(bc.BATT_X[0] - 4.0, bc.BATT_X[0] - 0.5, -44, 44, Z0, Z0 + 33.5), "#9a4d9a", [-40, 0, 40], "LIST: board size from the listing (88 x 34 x 3.5)"),
 ("Pi", "Raspberry Pi Zero 2 W", P["Raspberry Pi Zero 2 W"][0], "#2e7d32", [-70, 0, 0], P["Raspberry Pi Zero 2 W"][1]),
 ("perfboard", "Perfboard 5 x 7 cm + female headers", P["Perfboard 5 x 7 cm + female headers"][0], "#3f6e3f", [-70, 0, 0], P["Perfboard 5 x 7 cm + female headers"][1]),
 ("ESP32", "ESP32-S3-DevKitC-1", P["ESP32-S3-DevKitC-1"][0], "#2b2b2b", [-70, 0, 0], P["ESP32-S3-DevKitC-1"][1]),
 ("CAN", "SN65HVD230 CAN board", P["SN65HVD230 CAN board"][0], "#1565c0", [-70, 0, 0], P["SN65HVD230 CAN board"][1]),
 ("regulator", "Pololu D24V50F5 5 V regulator", P["Pololu D24V50F5 regulator"][0], "#2b6cb0", [0, 0, -70], P["Pololu D24V50F5 regulator"][1]),
 ("IMU", "GY-521 IMU", B(-10, 6, -10.5, 10.5, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 7), "#d9534f", [0, 0, -70], "LIST: board size from the listing (21 x 16 x 3)"),
 ("PDB", "PDB + 30 A fuse", B(-28, -12, -6.5, 6.5, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T, bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 9), "#4caf50", [0, 0, -70], "LIST: estimated envelope"),
 ("sonar", "HC-SR04 sonar", P["HC-SR04 sonar"][0], "#35a0a0", [70, 0, 0], P["HC-SR04 sonar"][1]),
 ("Camera", "Raspberry Pi AI Camera (IMX500)", P["Camera Module 3 Wide"][0], "#7b7be0", [70, 0, 0], P["Camera Module 3 Wide"][1]),
 ("tsop", "IR receivers 4 x TSOP38238", cq.Compound.makeCompound([B(-2.8, 2.0, -2.8, 2.8, 1.4, 8.0).rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), f).translate(cq.Vector(x, y, bc.Z_TOP)) for (x, y, f) in bc.IR_RX]), "#202024", [0, 0, 25], "LIST: listed size (5.6 x 6 x 7)"),
 ("headlight", "COB disc headlights (Ø30, 12 V 2.4 W)", cq.Compound.makeCompound([rp.cyl((bc.X_IN_F - bc.LIGHT_T, s * bc.LIGHT_Y, bc.LIGHT_Z), bc.LIGHT_D, bc.LIGHT_T, (1, 0, 0)) for s in (1, -1)]), "#f6e27a", [70, 0, 0], "LIST: listed size (Ø30 x 4)"),
 ("driver", "Headlight driver (D36V6F12 + IRLB8721)", cq.Compound.makeCompound([B(bc.X_IN_F - bc.DRV_STANDOFF - 1.6, bc.X_IN_F - bc.DRV_STANDOFF, bc.DRV_CENTER[0] - 12.7, bc.DRV_CENTER[0] + 12.7, bc.DRV_CENTER[1] - 10.15, bc.DRV_CENTER[1] + 10.15),
     B(bc.X_IN_F - bc.DRV_STANDOFF - 7.6, bc.X_IN_F - bc.DRV_STANDOFF - 1.6, bc.DRV_CENTER[0] - 10, bc.DRV_CENTER[0] + 10, bc.DRV_CENTER[1] - 7, bc.DRV_CENTER[1] + 7)]), "#3f6e3f", [55, 0, 0], "LIST: estimated (25 x 20 perfboard + parts)"),
 ("bumper", "Front rubber bumpers (4 x Ø11 x 2.5)", cq.Compound.makeCompound([rp.cyl((bc.X_FRONT, s * y, z), bc.BUMPER_D, 2.5, (1, 0, 0)) for (y, z) in bc.FRONT_BUMPERS for s in (1, -1)]), "#d0e8f0", [70, 0, 0], "LIST: 3M Bumpon type"),
 ("LED", "LED eye panels (8x8 WS2812B-2020)", cq.Compound.makeCompound([B(bc.X_IN_F - 1.1, bc.X_IN_F + 2.2, s * bc.EYE_Y - 13, s * bc.EYE_Y + 13, bc.EYE_Z - 13, bc.EYE_Z + 13) for s in (1, -1)]), "#7fe3ff", [70, 0, 0], "LIST: panel size from the listing (26 x 26 x 3)"),
 ("pwrsw", "Power switch board (Pololu 2813, MOSFET latch)", B(*bc.pwr_sw_box()), "#1b5e20", [0, 0, 40], "LIST: listed size (20 x 25.5 x 4)"),
 ("pwrbtn", "Power button 12 mm (momentary)", rp.cyl((bc.POWER_BTN[0], bc.POWER_BTN[1], bc.Z_IN_TOP - 15), 12, 15, (0, 0, 1)), "#424242", [0, 0, 60], "LIST: listed size"),
 ("arm", "Arm button 12 mm", rp.cyl((bc.BUTTON[0], bc.BUTTON[1], bc.Z_IN_TOP - 15), 12, 15, (0, 0, 1)), "#e53935", [0, 0, 60], "LIST: listed size"),
 ("lidar", "LD19 lidar", P["LD19 lidar"][0], "#2a2a2a", [0, 0, 60], P["LD19 lidar"][1]),
 ("lidar", "LD19 USB adapter board", B(-14, 4, 44, 56, Z0 + 7, Z0 + 13), "#777777", [-40, 0, 40], "LIST: estimated (18 x 12 x 6)"),
 ("USB-C", "USB-C PD charger IP2368", B(-13.5, 21.5, 67, 77, 71, 108), "#5a5ab0", [0, 60, 0], "LIST: estimated (35 x 47 x 10)"),
 ("USB-C", "USB-C panel socket (CAB-15455)", B(bc.USBC_SIDE[0] - 11, bc.USBC_SIDE[0] + 11, bc.Y_IN - 14, bc.Y_IN, bc.USBC_SIDE[1] - 4, bc.USBC_SIDE[1] + 4), "#333333", [0, 70, 0], "LIST: listed head size"),
 ("charge", "Charge diode + fuse (inline)", B(-10, 8, -52, -46, Z0 + 2, Z0 + 9), "#888888", [-40, 0, 40], "LIST: estimated"),
 ("speaker", "Speaker 40 mm 4 Ω (Adafruit 3968)", P["Speaker 40 mm (Adafruit 3968)"][0], "#3a3a40", [0, -70, 0], P["Speaker 40 mm (Adafruit 3968)"][1]),
 ("I2S", "I2S amp MAX98357A (Adafruit 3006)", P["MAX98357A amp (Adafruit 3006)"][0], "#8e44ad", [0, -70, 0], P["MAX98357A amp (Adafruit 3006)"][1]),
 ("mic", "Microphone L (Adafruit 6049 ICS-43434)", P["ICS-43434 mic L"][0], "#c0392b", [70, 0, 0], P["ICS-43434 mic L"][1]),
 ("mic", "Microphone R (Adafruit 6049 ICS-43434)", P["ICS-43434 mic R"][0], "#c0392b", [70, 0, 0], P["ICS-43434 mic R"][1]),
 ("motor", "Motor plugs (left)", motor_plugs(1), "#e8e2c8", [0, 45, 0], "DRAW: LK drawing, plug sizes estimated"),
 ("motor", "Motor plugs (right)", motor_plugs(-1), "#e8e2c8", [0, -45, 0], "DRAW: LK drawing, plug sizes estimated"),
]
for key, label, shp, col, ex, srcs in items:
    add("c_" + key, label, shp, col, "parts", ex, tol=0.2)
    parts[-1]["src"] = srcs
HS = W.harnesses()
for lbl, shp, col in W.connectors():
    add("k_" + lbl.split()[0], lbl, shp, col, "conn", [0, 0, 0], tol=0.15)
    parts[-1]["src"] = "Mated plug / connector at a harness end"
wd = []
for h in HS:
    wd.append(dict(name=h["name"], desc=h["desc"], a_conn=h["a_conn"], b_conn=h["b_conn"], dia=h["dia"],
                   path=[[round(v, 2) for v in p] for p in h["path"]],
                   conds=[dict(sig=c["sig"], col=c["col"], d=c["d"], a=[round(v, 2) for v in c["a"]], b=[round(v, 2) for v in c["b"]],
                               a_lbl=c["a_lbl"], b_lbl=c["b_lbl"], L=round(W.length(W.conductor_path(h, c)) / 10, 1)) for c in h["conds"]]))
json.dump(wd, open(WS + 'case/wiring_data.json', 'w'))
exec(tail)
