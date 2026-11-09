"""wiring_data.json (for the viewer) + wiring_diagram.json (modules, pins, wires) from wiring.py."""
import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import sys, json
sys.path.insert(0, WS + 'case')
import real_parts as rp
rp.real_parts()                                   # fills the placements used for pin positions
import wiring as W
HS = W.harnesses()
wd = [dict(name=h["name"], desc=h["desc"], a_conn=h["a_conn"], b_conn=h["b_conn"], dia=h["dia"], slack=h.get("slack", 0),
           path=[[round(v, 2) for v in p] for p in h["path"]],
           conds=[dict(sig=c["sig"], col=c["col"], d=c["d"], a=[round(v, 2) for v in c["a"]], b=[round(v, 2) for v in c["b"]],
                       a_lbl=c["a_lbl"], b_lbl=c["b_lbl"], awg=c["awg"], L=round((W.length(W.conductor_path(h, c)) + h.get("slack", 0)) / 10, 1)) for c in h["conds"]]) for h in HS]
json.dump(wd, open(WS + 'case/wiring_data.json', 'w'))
RULES = [("Pi pin", "Raspberry Pi Zero 2 W", "Pi "), ("Pi CSI", "Raspberry Pi Zero 2 W", "Pi "), ("Pi micro-USB", "Raspberry Pi Zero 2 W", "Pi "),
         ("Perfboard: ESP32", "Perfboard: ESP32-S3 + level shifter", "Perfboard: "),
         ("Perfboard", "Perfboard: ESP32-S3 + level shifter", "Perfboard "), ("CAN board", "CAN board (SN65HVD230)", "CAN board "),
         ("Motor L", "Motor L (MF6015)", "Motor L "), ("Motor R", "Motor R (MF6015)", "Motor R "), ("Battery", "Battery 6S 1400", "Battery "),
         ("BMS", "6S BMS", "BMS "), ("Power switch", "Power switch + 30 A fuse", "Power switch "), ("Power button", "Power button (12 mm)", "Power button "), ("PDB", "Power distribution (PDB)", "PDB "),
         ("Regulator", "5 V regulator (D24V50F5)", "Regulator "), ("GY-521", "IMU (GY-521)", "GY-521 "), ("HC-SR04", "Sonar (HC-SR04)", "HC-SR04 "),
         ("Eye R", "Eye R (8x8 WS2812)", "Eye R "), ("Eye L", "Eye L (8x8 WS2812)", "Eye L "), ("Junction", "I2S junction board", "Junction "),
         ("Mic L", "Mic L (ICS-43434)", "Mic L "), ("Mic R", "Mic R (ICS-43434)", "Mic R "), ("Amp", "Amp (MAX98357A)", "Amp "),
         ("Speaker", "Speaker 3 W", "Speaker "), ("Camera", "Camera Module 3", "Camera "), ("LD19 adapter", "LD19 USB adapter", "LD19 adapter "),
         ("Adapter", "LD19 USB adapter", "Adapter "), ("LD19", "LD19 lidar", "LD19 "), ("Contact pad", "Dock contacts", "Contact pad "),
         ("Fuse + diode", "Charge fuse + diode", "Fuse + diode "), ("IP2368", "IP2368 USB-C charger", "IP2368 "), ("Side USB-C", "USB-C socket (side)", "Side "),
         ("Headlight driver", "Headlight driver (D36V6F12 + IRLB8721)", "Headlight driver "), ("Headlight L", "Headlight L (COB disc)", "Headlight L "),
         ("Headlight R", "Headlight R (COB disc)", "Headlight R "), ("Arm button", "Arm button + RGB LED", ""), ("RGB LED", "Arm button + RGB LED", ""), ("Button + LED", "Arm button + RGB LED", ""),
         ("IR receiver", "IR receivers (4 x TSOP38238)", "IR receiver ")]
def mod(l):
    for k, m, strip in RULES:
        if l.startswith(k): return m, (l[len(strip):] if strip and l.startswith(strip) else l)
    raise KeyError(l)
COLS = [["Battery 6S 1400", "6S BMS", "Power switch + 30 A fuse", "Power button (12 mm)", "Power distribution (PDB)", "5 V regulator (D24V50F5)", "Dock contacts", "Charge fuse + diode", "USB-C socket (side)", "IP2368 USB-C charger", "Headlight driver (D36V6F12 + IRLB8721)"],
        ["Perfboard: ESP32-S3 + level shifter", "CAN board (SN65HVD230)", "Raspberry Pi Zero 2 W", "I2S junction board"],
        ["Motor R (MF6015)", "Motor L (MF6015)", "IMU (GY-521)", "Sonar (HC-SR04)", "Eye R (8x8 WS2812)", "Eye L (8x8 WS2812)", "Mic L (ICS-43434)", "Mic R (ICS-43434)", "Amp (MAX98357A)", "Speaker 3 W", "Camera Module 3", "LD19 USB adapter", "LD19 lidar", "Arm button + RGB LED", "Headlight L (COB disc)", "Headlight R (COB disc)", "IR receivers (4 x TSOP38238)"]]
pins = {}
wires = []
for hi, h in enumerate(wd):
    for c in h["conds"]:
        (ma, pa), (mb, pb) = mod(c["a_lbl"]), mod(c["b_lbl"])
        for m, pn in ((ma, pa), (mb, pb)):
            pins.setdefault(m, [])
            if pn not in pins[m]: pins[m].append(pn)
        wires.append(dict(h=hi, harness=h["name"], sig=c["sig"], col=c["col"], d=c["d"], awg=c["awg"], a_lbl=c["a_lbl"], b_lbl=c["b_lbl"], a=[ma, pa], b=[mb, pb], L=c["L"]))
missing = [m for m in pins if not any(m in col for col in COLS)]
assert not missing, missing
json.dump(dict(cols=COLS, pins=pins, wires=wires, harnesses=[h["name"] for h in wd]), open(WS + 'case/wiring_diagram.json', 'w'))
print(len(wires), "wires,", len(pins), "modules")
