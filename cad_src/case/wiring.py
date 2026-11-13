"""Wiring in situ, wire by wire. Every conductor starts on a real pin / pad / terminal (positions from the real part
models via real_parts.to_robot) and ends on one; the conductors of a harness run together along the harness path
(free corridors of the V1.1 body) and fan out to their pins at both ends. The same data drives the 3D viewer and the
wiring diagram.

Corridors: BACK (x -30..-24 behind the boards), TOP (z 125..129), OVER (z 78..98 above the battery), FRONT (x 13..17
between the battery and the sonar), HOLE (shelf cable hole x 19..26, y +-12), GAP (centre gap y +-12).
"""
import math, cadquery as cq
import body_case as bc
import motors as _motors
from real_parts import to_robot, pi_gpio, esp_pin

PI = "Raspberry Pi Zero 2 W"
PBC = -13.0          # top of the 2.54 headers + plugs on the perfboard (perfboard front face x -21.4)
HOLE = (22.5, 0.0)
ZT = bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 9.5      # top of the PDB on the tray
ZI = bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 7        # top of the IMU board

# wire colours (silicone wire, conventional where there is a convention)
RED, BLK, ORG, YEL, GRN, BLU, WHT, PUR, GRY, BRN, PNK = ("#d32f2f", "#212121", "#fb8c00", "#fdd835", "#43a047", "#1e88e5",
                                                         "#f5f5f5", "#8e24aa", "#9e9e9e", "#6d4c41", "#ec407a")
AWG = {12: 3.4, 16: 2.6, 20: 1.8, 22: 1.5, 24: 1.3, 26: 1.1, 28: 0.9}

def pin(n):                 # Pi GPIO header pin n: end of its Dupont housing (17.5 mm above the PCB)
    return to_robot(PI, (*pi_gpio(n), 17.5))
import perfboard_layout as PL   # plug + part positions on the perfboard hole grid
def jp(name, i):            # pin i (0-based) of perfboard plug J1..J7: top of its housing
    return PL.plug_pin(name, i)
def mic_pad(side, i):       # Adafruit 6049 pads along its x at y 2.5: 3V, GND, BCLK, DOUT, WS, SEL (check the silkscreen)
    return to_robot(f"ICS-43434 mic {side}", (1.27 + 2.54 * i, 2.5, 1.6))
def amp_pad(i):             # Adafruit 3006 pads: LRC, BCLK, DIN, GAIN, SD, GND, Vin
    return to_robot("MAX98357A amp (Adafruit 3006)", (1.27 + 2.54 * i, 1.3, 1.6))
def reg_pad(i):             # Pololu D24V50F5 pin row (0.1"): VIN, GND, GND, VOUT used
    return to_robot("Pololu D24V50F5 regulator", (2.54 + 2.54 * i, 1.0, 1.6))
def tof_pin(i):
    """VL53L5CX carrier header: six pins along its 18 mm edge, behind the 30 deg seat."""
    t = math.radians(bc.TOF_TILT)
    seat = (bc.X_IN_F - bc.TOF_SEAT * math.cos(t), 0.0, bc.TOF_Z + bc.TOF_SEAT * math.sin(t))
    return (seat[0] - 4.0, -7.5 + 3.0 * i, seat[2] - 5.0)


def son_pin(i):             # HC-SR04: Vcc, Trig, Echo, Gnd; end of the Dupont housing (pins point down)
    p = to_robot("HC-SR04 sonar", (-3.81 + 2.54 * i, -15.5, -1)); return (p[0], p[1], 56.0)
def imu_pin(i):             # GY-521 header along the back edge of the board: VCC, GND, SCL, SDA
    return (-9.0, -3.81 + 2.54 * i, ZI + 8.5)
JUNC = (4.0, 27.5, bc.BATT_TOP + 2.5)    # I2S junction board (20 x 15 perfboard) on the battery top: its pad row
def jc(i, n=7):
    return (JUNC[0] + (i - (n - 1) / 2) * 2.0, JUNC[1], JUNC[2])
def motor_pin(side, i):     # motor main plug (V+, V-, CAN-H, CAN-L), top of the mated plug
    # z comes from motors.plug_pin_z, not a literal: the MF6015's plugs stand on its rear boss at
    # z 29, the MF7015's stands off its Ø69 rim. |y| 27.75 happens to suit both - it is the middle
    # of the connector on each, which sits 2.9-8.6 mm out from the rear face either way.
    s = 1 if side == "L" else -1
    return (-4.5 + 3.0 * i, s * 27.75, _motors.plug_pin_z(bc.MOTOR))
_x0, _x1, _y0, _y1, _z0, _z1 = bc.pwr_sw_box()                                     # power switch board on the shelf
_cy = (_y0 + _y1) / 2
SW = {"VIN": (_x1, _cy - 9, _z0 + 2), "GND": (_x1, _cy, _z0 + 2), "VOUT": (_x1, _cy + 9, _z0 + 2),          # pads face the front
      "OFF": (_x1, _cy + 9, _z1 - 2), "BTN A": (_x1, _cy - 9, _z1 - 2), "BTN B": (_x1, _cy - 6, _z1 - 2)}
PBTN = lambda i: (bc.POWER_BTN[0] + (2.5 if i else -2.5), bc.POWER_BTN[1], bc.Z_IN_TOP - 16)   # power button tabs
XSPL = (_x0 + 3.5, _y0 - 4.5, (_z0 + _z1) / 2)                                   # heat-shrunk splice of the two negatives, beside the board
PDB_IN = lambda s: (-28.0, s * 3.0, ZT - 4)
PDB_OUT = lambda k, s: (-12.0, s * 3.0 + k, ZT - 4)

DRV_X = bc.X_IN_F - bc.DRV_STANDOFF - 1.6 - 0.9      # headlight driver: wire ends just behind its component side
def drv(pad):                # pads at the board corners, outside the component block (Pololu + flat TO-220 in the middle)
    cy, cz = bc.DRV_CENTER
    return {"VIN": (DRV_X, cy - 11.5, cz - 9), "GND": (DRV_X, cy - 9.0, cz - 9), "PWM": (DRV_X, cy + 11.5, cz - 9), "PWM GND": (DRV_X, cy + 9.0, cz - 9),
            "L+": (DRV_X, cy + 11.5, cz + 9), "L-": (DRV_X, cy + 9.0, cz + 9), "R+": (DRV_X, cy - 11.5, cz + 9), "R-": (DRV_X, cy - 9.0, cz + 9)}[pad]
def disc(side, pol):         # solder pads on the back of a COB disc (+ outer, - inner)
    s = 1 if side == "L" else -1
    return (bc.X_IN_F - bc.LIGHT_T - 0.8, s * (bc.LIGHT_Y + (3.0 if pol == "+" else -3.0)), bc.LIGHT_Z)
IPVB = (2.0, 67.0, 96.0)       # IP2368 VBUS pad (input side of the charger board, facing -y)
def C(sig, col, awg, a, b, a_lbl, b_lbl):
    return dict(sig=sig, col=col, awg=awg, d=AWG[awg], a=tuple(map(float, a)), b=tuple(map(float, b)), a_lbl=a_lbl, b_lbl=b_lbl)

def H(name, path, conds, a_conn, b_conn, desc="", slack=0.0):   # slack: extra length (mm) for a service loop
    return dict(name=name, path=[tuple(map(float, p)) for p in path], conds=conds, a_conn=a_conn, b_conn=b_conn, desc=desc, slack=slack)

COVER_CROSSING = {"BMS -> 30 A fuse -> power switch", "Power switch -> PDB", "Power button -> switch", "5 V regulator -> perfboard J1",
                  "CAN bus -> motor R", "Perfboard J3 -> IMU", "Perfboard J2 -> sonar", "Perfboard J4 -> eye R", "Pi I2S -> I2S junction",
                  "Pi OTG -> LD19 USB adapter", "Arm button + LED -> perfboard J6", "IR receivers -> perfboard J8", "Dock sense -> perfboard J7",
                  "USB sense -> perfboard J7", "Pi -> headlight driver (PWM)"}
COVER_SLACK = 40.0   # every wire between the back-cover module and the body: lift the cover off and lay it down beside the robot
MOTOR_ARC = [(0, 12, 28), (0, 15, 36), (-0.5, 19, 39.5), (-1.5, 24, 38)]   # motor leads: up out of the plug, over and down (R ~7 mm)

def harnesses():
    def esp_s(side, n):
        p = to_robot("ESP32-S3-DevKitC-1", (*esp_pin(side, n), -8.5)); return (-20.6, p[1], p[2])
    def can_p(i):
        return to_robot("SN65HVD230 CAN board", (-3.7, 3.0, -11.3 + 2.54 * i))
    can_t = to_robot("SN65HVD230 CAN board", (25.0, 11.0, -8))
    cam_fpc = to_robot("Camera Module 3 Wide", (20.2, 12.5, 1.5)); pi_csi = to_robot(PI, (63.0, 15.0, 1.6))
    otg = to_robot(PI, (41.4, -1.0, 2.3))
    amp_spk = [to_robot("MAX98357A amp (Adafruit 3006)", (7.1 + 3.5 * k, 17.8, 1.6)) for k in (0, 1)]
    Zs = bc.SHELF_Z + bc.SHELF_T                                     # shelf top (battery + BMS stand on it)
    bms = lambda y, dz: (-29.7, y, Zs + dz)                          # BMS pad: y, height above the shelf top
    out = [
     # ---------------- power ----------------
     H("Battery -> BMS", [(-14, -38.5, 82), (-24, -38.5, 82)],
       [C("B+", RED, 12, (-8, -35, 82.5), bms(-38, 24), "Battery lead +", "BMS B+"),
        C("B-", BLK, 12, (-8, -35, 79.5), bms(-34, 24), "Battery lead -", "BMS B-"),
        C("Balance (7-wire, JST-XH)", WHT, 26, (-2, -35, 84), bms(-30, 28), "Battery balance lead", "BMS JST-XH 7p")],
       "Battery XT60 + balance leads", "BMS input pads + JST-XH", "Battery to the BMS on its back face"),
     # behind the perfboard, between its standoffs and below the contact-plate bosses
     H("BMS -> 30 A fuse -> power switch", [(-28.5, -50, 82), (-25.8, -54, 90), (-25.8, -54, 129), (-25.8, 10, 129)],
       [C("P+ (30 A fuse inline)", RED, 16, bms(-42, 32), SW["VIN"], "BMS P+", "Power switch VIN (BAT+ from the fuse)"),
        C("P-", BLK, 16, bms(-43.5, 30), XSPL, "BMS P-", "Power switch P- splice (beside the board)")],
       "BMS output pads (XT60 female on the body side)", "Power switch board VIN + P- splice (XT60 male pigtail)", "Up the back, beside the perfboard end, to the switch board above the Pi. Unplugs at the XT60 beside the battery"),
     H("Power switch -> PDB", [(-20, 30, 130), (-4, 12, 129), (2, 4, 128),   # over the Pi, then past the DevKit's end
                                      (19, -45, 100), (19, -40, 62), (HOLE[0], -8, 58), (HOLE[0], -8, 48), (HOLE[0], -8, 34), (0, -6, 22), (-24, -4, 8)],
       [C("BAT+ switched", RED, 16, SW["VOUT"], PDB_IN(1), "Power switch VOUT (BAT+ to the PDB)", "PDB in +"),
        C("BAT-", BLK, 16, XSPL, PDB_IN(-1), "Power switch P- splice (beside the board)", "PDB in -")],
       "Power switch VOUT + P- splice (XT60 female pigtail)", "PDB input (XT60 male on the body side)", "Down the front, shelf cable hole, centre gap. Unplugs at the XT60 over the battery"),
     # beside the DevKit, then over the Pi
     H("Power button -> switch", [(4, -30, 140), (-3, -20, 140), (-3, 9, 138), (-20, 20, 138)],
       [C("Button A", WHT, 26, PBTN(0), SW["BTN A"], "Power button A", "Power switch button pad A"),
        C("Button B", WHT, 26, PBTN(1), SW["BTN B"], "Power button B", "Power switch button pad B"),
        C("Switch GND (control)", BLK, 26, XSPL, SW["GND"], "Power switch P- splice (beside the board)", "Power switch GND")],
       "12 mm momentary on top (JST-XH 3p plug)", "Switch board pushbutton pads (in parallel with its own button)", "Under the top skin to the back. Unplugs at the JST-XH beside the switch board"),
     # under the DevKit's end, up between it and the Pi
     H("ESP32 GPIO21 -> switch OFF", [(-14, 8.5, 114), (-14, 8.5, 129), (-22, 30, 142)],
       [C("OFF (flat battery)", YEL, 28, jp("J9", 0), SW["OFF"], "Perfboard J9-1 (GPIO21)", "Power switch OFF")],
       "Perfboard J9", "Power switch OFF input", "Up beside the perfboard to the switch board"),
     H("PDB -> motor L power", [(-8, 8, 14)] + MOTOR_ARC,
       [C("V+", RED, 16, PDB_OUT(2, 1), motor_pin("L", 0), "PDB out +", "Motor L plug V+"),
        C("V-", BLK, 16, PDB_OUT(2, -1), motor_pin("L", 1), "PDB out -", "Motor L plug V-")],
       "PDB output", "Motor L main plug", "Up out of the plug, over and down through the left wall window (no kink at the plug)"),
     H("PDB -> motor R power", [(-8, -8, 14)] + [(x, -y, z) for (x, y, z) in MOTOR_ARC],
       [C("V+", RED, 16, PDB_OUT(-2, 1), motor_pin("R", 0), "PDB out +", "Motor R plug V+"),
        C("V-", BLK, 16, PDB_OUT(-2, -1), motor_pin("R", 1), "PDB out -", "Motor R plug V-")],
       "PDB output", "Motor R main plug", "Up out of the plug, over and down through the right wall window (no kink at the plug)"),
     H("PDB -> 5 V regulator", [(-6, 0, -5), (4, 0, -6)],
       [C("VIN (3 A inline fuse)", RED, 20, PDB_OUT(0, 1), reg_pad(0), "PDB out +", "Regulator VIN"),
        C("GND", BLK, 20, PDB_OUT(0, -1), reg_pad(1), "PDB out -", "Regulator GND")],
       "PDB output", "Pololu D24V50F5 VIN/GND", "On the tray"),
     H("5 V regulator -> perfboard J1", [(HOLE[0], 4, 10), (HOLE[0], 4, 48), (HOLE[0], 4, 58), (15, 4, 59), (14.5, 4, 70), (14.5, 4, 90), (14.5, 4, 102), (-4, 4, 102)],
       [C("5 V", ORG, 20, reg_pad(3), jp("J1", 0), "Regulator VOUT", "Perfboard J1-1 (5 V bus, XT30)"),
        C("GND", BLK, 20, reg_pad(2), jp("J1", 1), "Regulator GND", "Perfboard J1-2 (GND, XT30)")],
       "Pololu VOUT/GND", "Perfboard J1 (XT30, 15 A)", "Gap, shelf hole, front lane, over the battery"),
     # ---------------- Pi <-> perfboard ----------------
     H("Perfboard J5 -> Pi (5 V, GND, UART, eye data)", [(-4, -8, 110), (-3, 9, 114), (-7, 22, 128)],
       [C("5 V", ORG, 24, jp("J5", 0), pin(2), "Perfboard J5-1", "Pi pin 2 (5 V)"),
        C("GND", BLK, 24, jp("J5", 1), pin(6), "Perfboard J5-2", "Pi pin 6 (GND)"),
        C("ESP TX (GPIO17) -> Pi RXD", YEL, 26, jp("J5", 2), pin(10), "Perfboard J5-3 (ESP32 GPIO17)", "Pi pin 10 (GPIO15 RXD)"),
        C("Pi TXD -> ESP RX (GPIO18)", GRN, 26, jp("J5", 3), pin(8), "Perfboard J5-4 (ESP32 GPIO18)", "Pi pin 8 (GPIO14 TXD)"),
        C("Eye data: Pi MOSI -> 74AHCT125", BLU, 26, jp("J5", 4), pin(19), "Perfboard J5-5 (74AHCT125 in)", "Pi pin 19 (GPIO10 MOSI)")],
       "Perfboard J5", "Pi GPIO header (Dupont)", "ESP32 UART at 921600 baud, 5 V to the Pi, WS2812 data to the level shifter"),
     # ---------------- CAN ----------------
     H("ESP32 -> CAN board (perfboard jumpers)", [(-20.0, -60, 103)],
       [C("3V3", RED, 26, esp_s("L", 1), can_p(0), "Perfboard: ESP32 3V3 (pin L1)", "CAN board 3V3"),
        C("GND", BLK, 26, esp_s("L", 22), can_p(1), "Perfboard: ESP32 GND (pin L22)", "CAN board GND"),
        C("CAN TX (GPIO4) -> CTX", YEL, 26, esp_s("L", 4), can_p(2), "Perfboard: ESP32 GPIO4 (pin L4)", "CAN board CTX"),
        C("CAN RX (GPIO5) <- CRX", GRN, 26, esp_s("L", 5), can_p(3), "Perfboard: ESP32 GPIO5 (pin L5)", "CAN board CRX")],
       "ESP32 header (solder side of the perfboard)", "CAN board 4-pin header", "Short jumpers on the perfboard"),
     H("CAN bus -> motor R", [(-6, -30, 102), (4, -28, 100), (14.5, -28, 96), (14.5, -28, 88), (14.5, -28, 70), (HOLE[0], -10, 58), (HOLE[0], -10, 48), (HOLE[0], -10, 30)] + [(x + 1.5, -y, z) for (x, y, z) in MOTOR_ARC],
       [C("CAN-H", YEL, 26, (can_t[0], can_t[1], can_t[2] + 2), motor_pin("R", 2), "CAN board terminal H", "Motor R plug CAN-H"),
        C("CAN-L", GRN, 26, (can_t[0], can_t[1], can_t[2] - 2), motor_pin("R", 3), "CAN board terminal L", "Motor R plug CAN-L")],
       "CAN board screw terminal + JST-XH 2p cover plug", "Motor R main plug (CAN-H/L)", "Twisted pair; 120 ohm at both ends of the bus. Unplugs at the JST-XH beside the CAN board", slack=COVER_SLACK),
     H("CAN bus -> motor L (branch)", [(x + 1.5, -y, z) for (x, y, z) in MOTOR_ARC[::-1]] + [(0, -8, 30), (0, 8, 30)] + [(x + 1.5, y, z) for (x, y, z) in MOTOR_ARC],
       [C("CAN-H", YEL, 26, motor_pin("R", 2), motor_pin("L", 2), "Motor R plug CAN-H", "Motor L plug CAN-H"),
        C("CAN-L", GRN, 26, motor_pin("R", 3), motor_pin("L", 3), "Motor R plug CAN-L", "Motor L plug CAN-L")],
       "Motor R plug", "Motor L main plug", "Daisy-chained across the centre gap"),
     # ---------------- sensors ----------------
     H("Perfboard J3 -> IMU", [(4, -20, 102), (14.5, -20, 98), (14.5, -20, 88), (14.5, -20, 70), (HOLE[0], 6, 58), (HOLE[0], 6, 48), (HOLE[0], 6, 30), (-2, 6, 12)],
       [C("3V3", RED, 26, jp("J3", 0), imu_pin(0), "Perfboard J3-1 (3V3)", "GY-521 VCC"),
        C("GND", BLK, 26, jp("J3", 1), imu_pin(1), "Perfboard J3-2", "GY-521 GND"),
        C("SCL (GPIO9)", WHT, 26, jp("J3", 2), imu_pin(2), "Perfboard J3-3 (ESP32 GPIO9)", "GY-521 SCL"),
        C("SDA (GPIO8)", BLU, 26, jp("J3", 3), imu_pin(3), "Perfboard J3-4 (ESP32 GPIO8)", "GY-521 SDA")],
       "Perfboard J3", "GY-521 header (Dupont)", "Down to the IMU on the tray"),
     H("Perfboard J2 -> sonar", [(-4, -14, 102), (15, -12, 98), (15, -12, 90), (15, -12, 68)],
       [C("5 V", ORG, 26, jp("J2", 0), son_pin(0), "Perfboard J2-1 (5 V)", "HC-SR04 Vcc"),
        C("TRIG (GPIO10)", PUR, 26, jp("J2", 1), son_pin(1), "Perfboard J2-2 (ESP32 GPIO10)", "HC-SR04 Trig"),
        C("ECHO (GPIO11 via 1k/2k)", GRY, 26, jp("J2", 2), son_pin(2), "Perfboard J2-3 (divider -> GPIO11)", "HC-SR04 Echo"),
        C("GND", BLK, 26, jp("J2", 3), son_pin(3), "Perfboard J2-4", "HC-SR04 Gnd")],
       "Perfboard J2", "HC-SR04 header (Dupont)", "Over the battery, down the front lane"),
     H("Perfboard J2T -> zone array", [(-4, -14, 102), (15, -12, 104), (15, -8, 130), (18, -4, 138)],
       [C("3V3", RED, 26, jp("J2T", 0), tof_pin(0), "Perfboard J2T-1 (3V3)", "VL53L5CX VIN"),
        C("GND", BLK, 26, jp("J2T", 1), tof_pin(1), "Perfboard J2T-2", "VL53L5CX GND"),
        C("SDA (GPIO42)", BLU, 26, jp("J2T", 2), tof_pin(2), "Perfboard J2T-3 (ESP32 GPIO42)", "VL53L5CX SDA"),
        C("SCL (GPIO41)", WHT, 26, jp("J2T", 3), tof_pin(3), "Perfboard J2T-4 (ESP32 GPIO41)", "VL53L5CX SCL"),
        C("LPn (GPIO2)", GRN, 26, jp("J2T", 4), tof_pin(4), "Perfboard J2T-5 (ESP32 GPIO2)", "VL53L5CX LPn"),
        C("INT (GPIO47)", YEL, 26, jp("J2T", 5), tof_pin(5), "Perfboard J2T-6 (ESP32 GPIO47)", "VL53L5CX INT")],
       "Perfboard J2T", "VL53L5CX carrier header (Dupont)",
       "Over the battery and up the front lane: the array sits high on the panel, not at the mouth"),
     # ---------------- eyes ----------------
     H("Perfboard J4 -> eye R", [(4, -24, 104), (21, -24, 92.5), (21, -38, 92.5), (21, -38, 100)],
       [C("5 V", ORG, 26, jp("J4", 0), (26, -40, 104), "Perfboard J4-1 (5 V, 470 uF)", "Eye R 5V"),
        C("DIN (via 330 ohm)", BLU, 26, jp("J4", 1), (26, -38, 104), "Perfboard J4-2 (74AHCT125 out)", "Eye R DIN"),
        C("GND", BLK, 26, jp("J4", 2), (26, -36, 104), "Perfboard J4-3", "Eye R GND")],
       "Perfboard J4", "Eye R panel pads (through the retainer's wire pass)", "Under the camera, behind the sonar"),
     H("Eye R -> eye L (chain)", [(21, -36, 107), (21, -22, 133), (21, 22, 133), (21, 36, 107)],
       [C("5 V", ORG, 26, (26, -40, 106), (26, 36, 104), "Eye R 5V out", "Eye L 5V"),
        C("DOUT -> DIN", BLU, 26, (26, -38, 106), (26, 38, 104), "Eye R DOUT", "Eye L DIN"),
        C("GND", BLK, 26, (26, -36, 106), (26, 40, 104), "Eye R GND out", "Eye L GND")],
       "Eye R output pads", "Eye L input pads", "Over the top of the camera board (x 21)"),
     # ---------------- audio: Pi I2S -> junction -> mics + amp ----------------
     H("Pi I2S -> I2S junction", [(0, 40, 114), (6, 32, 102)],
       [C("BCLK (GPIO18)", YEL, 26, pin(12), jc(0), "Pi pin 12", "Junction BCLK"),
        C("WS/LRCLK (GPIO19)", GRN, 26, pin(35), jc(1), "Pi pin 35", "Junction WS"),
        C("Mic DOUT -> Pi DIN (GPIO20)", BLU, 26, pin(38), jc(2), "Pi pin 38", "Junction DOUT"),
        C("Amp DIN (GPIO21)", WHT, 26, pin(40), jc(3), "Pi pin 40", "Junction DIN (amp)"),
        C("3V3", RED, 26, pin(17), jc(4), "Pi pin 17 (3V3)", "Junction 3V3"),
        C("5 V (amp)", ORG, 24, pin(4), jc(5), "Pi pin 4 (5 V)", "Junction 5 V"),
        C("GND", BLK, 24, pin(39), jc(6), "Pi pin 39 (GND)", "Junction GND")],
       "Pi GPIO header (Dupont)", "I2S junction board (on the battery)", "Splits BCLK/WS to both mics and the amp"),
     H("I2S junction -> mic L", [(14, 40, 98), (21, 50, 94), (21, 57.4, 99.4)],
       [C("3V", RED, 26, jc(4), mic_pad("L", 0), "Junction 3V3", "Mic L 3V"),
        C("GND", BLK, 26, jc(6), mic_pad("L", 1), "Junction GND", "Mic L GND"),
        C("BCLK", YEL, 26, jc(0), mic_pad("L", 2), "Junction BCLK", "Mic L BCLK"),
        C("DOUT", BLU, 26, jc(2), mic_pad("L", 3), "Junction DOUT", "Mic L DOUT"),
        C("WS", GRN, 26, jc(1), mic_pad("L", 4), "Junction WS", "Mic L WS"),
        C("SEL -> GND (left)", BLK, 26, jc(6), mic_pad("L", 5), "Junction GND", "Mic L SEL")],
       "I2S junction", "Mic L pads (through the clamp's wire pass)", ""),
     H("I2S junction -> mic R", [(10, 0, 105), (21, -30, 92.5), (21, -57, 92.5), (21, -66.6, 99.4)],
       [C("3V", RED, 26, jc(4), mic_pad("R", 0), "Junction 3V3", "Mic R 3V"),
        C("GND", BLK, 26, jc(6), mic_pad("R", 1), "Junction GND", "Mic R GND"),
        C("BCLK", YEL, 26, jc(0), mic_pad("R", 2), "Junction BCLK", "Mic R BCLK"),
        C("DOUT", BLU, 26, jc(2), mic_pad("R", 3), "Junction DOUT", "Mic R DOUT"),
        C("WS", GRN, 26, jc(1), mic_pad("R", 4), "Junction WS", "Mic R WS"),
        C("SEL -> 3V3 (right)", RED, 26, jc(4), mic_pad("R", 5), "Junction 3V3", "Mic R SEL")],
       "I2S junction", "Mic R pads (through the clamp's wire pass)", ""),
     H("I2S junction -> amp", [(4, 10, 110), (-4, 0, 137), (-2, -60, 127), (-2, -73, 124)],
       [C("LRC (WS)", GRN, 26, jc(1), amp_pad(0), "Junction WS", "Amp LRC"),
        C("BCLK", YEL, 26, jc(0), amp_pad(1), "Junction BCLK", "Amp BCLK"),
        C("DIN", WHT, 26, jc(3), amp_pad(2), "Junction DIN (amp)", "Amp DIN"),
        C("GND", BLK, 24, jc(6), amp_pad(5), "Junction GND", "Amp GND"),
        C("5 V", ORG, 24, jc(5), amp_pad(6), "Junction 5 V", "Amp Vin")],
       "I2S junction", "Amp pads (Adafruit 3006)", "Under the top skin to the amp on the speaker ring"),
     H("Amp -> speaker", [(-2, -66, 103), (-2, -58.5, 96)],
       [C("SPK+", RED, 24, amp_spk[0], (-4, -58.5, 92), "Amp terminal +", "Speaker +"),
        C("SPK-", BLK, 24, amp_spk[1], (0, -58.5, 92), "Amp terminal -", "Speaker -")],
       "Amp screw terminal", "Speaker solder tabs", ""),
     # ---------------- camera, lidar ----------------
     H("Camera FPC (22 -> 15 pin)", [(22, 0, 99), (0, 35, 107), (-12, 62, 95), (-17, 72, 93), (-18.5, pi_csi[1], 96)],
       [C("FPC (15/22 conductors)", "#c9b38a", 22, cam_fpc, pi_csi, "Camera FPC connector", "Pi CSI connector (22-pin)")],
       "Camera FPC connector", "Pi CSI connector", "Flat flexible cable, 15 cm, folded over the battery"),
     H("Pi OTG -> LD19 USB adapter", [(otg[0], otg[1], 78), (-6, 52, 72)],
       [C("USB (90° OTG cable)", "#424242", 16, otg, (-2, 50, 66), "Pi micro-USB OTG", "LD19 adapter USB")],
       "Pi OTG port (90° plug)", "LD19 USB adapter", ""),
     H("LD19 -> USB adapter", [(-10, -31, 154), (bc.LIDAR_CABLE[0], bc.LIDAR_CABLE[1], bc.Z_IN_TOP - 1), (-3, -33, bc.Z_IN_TOP - 1.5), (-3, -32, 138), (-3, -32, 110), (-3, 0, 102), (-3, 47, 96), (-3, 47, 90)],
       [C("P5V", RED, 28, (-11.5, -26.5, 155), (1, 47, 66), "LD19 P5V", "Adapter 5V"),
        C("GND", BLK, 28, (-10.5, -26.5, 155), (2, 47, 66), "LD19 GND", "Adapter GND"),
        C("Tx", WHT, 28, (-9.5, -26.5, 155), (3, 47, 66), "LD19 Tx", "Adapter RX"),
        C("PWM", YEL, 28, (-8.5, -26.5, 155), (4, 47, 66), "LD19 PWM", "Adapter (n.c.)")],
       "LD19 lead (ZH1.5 4-pin)", "LD19 USB adapter", "Through the top hole, down beside the ESP32"),
     # ---------------- charging ----------------
     H("Dock pads -> diode + fuse", [(-30, -30, 110), (-30, -49, 80), (-14, -49, 62)],   # under the contact-plate bosses
       [C("Pad + (charge in)", PNK, 22, (-31, bc.PADS["+"][0], bc.PADS["+"][1]), (-10, -48, 57.5), "Contact pad + (ring lug)", "Fuse + diode IN (5 A fuse -> SB560 anode)"),
        C("Pad -", BLK, 22, (-31, bc.PADS["-"][0], bc.PADS["-"][1]), (-10, -50, 57.5), "Contact pad - (ring lug)", "Fuse + diode P- pass-through")],
       "Charge pads (ring lugs inside the cover)", "Inline 5 A fuse + SB560 diode", "Down behind the boards"),
     H("Diode -> BMS P", [(8, -42, 60), (-24, -40, 62)],
       [C("SB560 cathode -> P+", PNK, 22, (8, -49, 57.5), bms(-41, 18), "Fuse + diode OUT (SB560 cathode)", "BMS P+"),
        C("P-", BLK, 22, (8, -47, 57.5), bms(-39, 16), "Fuse + diode P- pass-through", "BMS P-")],
       "Diode output", "BMS P+/P-", ""),
     H("IP2368 <-> side USB-C", [(8, 64, 67), (17, 63, 70)],
       [C("USB-C cable (CAB-15455)", "#424242", 16, (bc.USBC_SIDE[0], bc.Y_IN - 14, bc.USBC_SIDE[1]), (16, 72, 74), "Side USB-C socket", "IP2368 USB-C")],
       "Panel USB-C socket (left side wall)", "IP2368 USB-C", "Short loop from the socket to the charger's USB-C"),
     H("IP2368 -> BMS P", [(-12, 47, 84)],
       [C("BAT+", RED, 20, (-12, 69, 74), bms(42, 28), "IP2368 BAT+", "BMS P+"),
        C("BAT-", BLK, 20, (-12, 71, 74), bms(43.5, 26), "IP2368 BAT-", "BMS P-")],
       "IP2368 BAT pads", "BMS P+/P-", ""),
     H("Arm button + LED -> perfboard J6", [(0, 30, 142), (-2, 8, 136), (-4, 5, 115), (-8, -12, 113)],
       [C("Button (GPIO12)", GRY, 26, (bc.BUTTON[0], bc.BUTTON[1] - 2, 115), jp("J6", 0), "Arm button", "Perfboard J6-1 (GPIO12)"),
        C("LED R (GPIO38)", RED, 26, (bc.LED[0], bc.LED[1] - 1.3, 118), jp("J6", 1), "RGB LED R", "Perfboard J6-2 (GPIO38 via 330 ohm)"),
        C("LED G (GPIO39)", GRN, 26, (bc.LED[0], bc.LED[1], 118), jp("J6", 2), "RGB LED G", "Perfboard J6-3 (GPIO39 via 330 ohm)"),
        C("LED B (GPIO40)", BLU, 26, (bc.LED[0], bc.LED[1] + 1.3, 118), jp("J6", 3), "RGB LED B", "Perfboard J6-4 (GPIO40 via 330 ohm)"),
        C("GND", BLK, 26, (bc.BUTTON[0], bc.BUTTON[1] + 2, 115), jp("J6", 4), "Button + LED common", "Perfboard J6-5 (GND)")],
       "Arm button + RGB LED", "Perfboard J6", ""),
     H("IR receivers -> perfboard J8", [(0, 2, 142), (-3, -4, 130), (-6, -10, 118)],
       [C("3V3 (VS, chained to all 4)", ORG, 28, (bc.IR_RX[0][0], bc.IR_RX[0][1], bc.Z_IN_TOP - 2), jp("J8", 0), "IR receiver front-left VS", "Perfboard J8-1 (3V3)"),
        C("GND (chained to all 4)", BLK, 28, (bc.IR_RX[2][0], bc.IR_RX[2][1], bc.Z_IN_TOP - 2), jp("J8", 1), "IR receiver back-left GND", "Perfboard J8-2 (GND)"),
        C("IR front-left (GPIO13)", WHT, 28, (bc.IR_RX[0][0] + 1, bc.IR_RX[0][1] - 1, bc.Z_IN_TOP - 2), jp("J8", 2), "IR receiver front-left OUT", "Perfboard J8-3 (GPIO13)"),
        C("IR front-right (GPIO14)", YEL, 28, (bc.IR_RX[1][0], bc.IR_RX[1][1], bc.Z_IN_TOP - 2), jp("J8", 3), "IR receiver front-right OUT", "Perfboard J8-4 (GPIO14)"),
        C("IR back-left (GPIO15)", GRN, 28, (bc.IR_RX[2][0] + 1, bc.IR_RX[2][1] - 1, bc.Z_IN_TOP - 2), jp("J8", 4), "IR receiver back-left OUT", "Perfboard J8-5 (GPIO15)"),
        C("IR back-right (GPIO16)", BLU, 28, (bc.IR_RX[3][0], bc.IR_RX[3][1], bc.Z_IN_TOP - 2), jp("J8", 5), "IR receiver back-right OUT", "Perfboard J8-6 (GPIO16)")],
       "4 IR receivers (TSOP38238) under the top hoods", "Perfboard J8", "Under the top skin to the middle, down to the perfboard"),
     # ---------------- sense wires for the ESP32 ADC dividers (GPIO1 / 6 / 7) ----------------
     H("VBAT sense -> perfboard J7", [(-22, 30, 136), (-14.5, 8.5, 129), (-14.5, 8.5, 114), (-6, -18, 110)],
       [C("VBAT sense (switched BAT+)", RED, 26, SW["VOUT"], jp("J7", 0), "Power switch VOUT (BAT+ to the PDB)", "Perfboard J7-1 (VBAT sense)")],
       "Power switch VOUT", "Perfboard J7", "From the switch board, over the battery"),
     H("Dock sense -> perfboard J7", [(-6, -46, 70), (-5, -44, 90), (-5, -28, 110)],
       [C("Dock sense (pad +, before the diode)", PNK, 26, (-10, -46, 57.5), jp("J7", 1), "Fuse + diode IN (5 A fuse -> SB560 anode)", "Perfboard J7-2 (dock sense)")],
       "Fuse + diode IN", "Perfboard J7", "Up beside the CAN board"),
     H("USB sense -> perfboard J7", [(2, 62, 92), (4, 30, 104), (2, 0, 104), (-6, -14, 110)],
       [C("USB VBUS sense", PUR, 26, IPVB, jp("J7", 2), "IP2368 VBUS pad", "Perfboard J7-3 (USB sense)")],
       "IP2368 VBUS pad", "Perfboard J7", "In front of the Pi, over the battery"),
     # ---------------- headlights ----------------
     H("PDB -> headlight driver", [(HOLE[0], 5, 10), (HOLE[0], 6, 46), (HOLE[0], 6, 57), (19, 24, 57), (16, 31, 50)],   # up the shelf hole, over the shelf, down into its notch
       [C("VIN (1 A inline fuse)", RED, 22, PDB_OUT(1, 1), drv("VIN"), "PDB out +", "Headlight driver VIN (1 A inline fuse)"),
        C("GND", BLK, 22, PDB_OUT(1, -1), drv("GND"), "PDB out -", "Headlight driver GND")],
       "PDB output", "Headlight driver VIN/GND", "Up the shelf cable hole to the driver on the front panel (1 A inline fuse)"),
     H("Pi -> headlight driver (PWM)", [(-3, 50, 110), (4, 54, 100), (18, 54.5, 80), (19.5, 54.5, 50)],
       [C("PWM (GPIO12)", WHT, 26, pin(32), drv("PWM"), "Pi pin 32", "Headlight driver PWM (GPIO12)"),
        C("GND", BLK, 26, pin(34), drv("PWM GND"), "Pi pin 34", "Headlight driver GND (PWM)")],
       "Pi GPIO header (Dupont)", "Headlight driver PWM pads", "Down the left side, in front of the lidar adapter"),
     H("Headlight driver -> headlight R", [(20, 28, 57), (20, -22, 57),   # between the shelf top and the sonar
       (21, -36, 62), (22, -42, 68)],
       [C("+12 V", RED, 24, drv("R+"), disc("R", "+"), "Headlight driver R+", "Headlight R +"),
        C("switched -", BLK, 24, drv("R-"), disc("R", "-"), "Headlight driver R-", "Headlight R -")],
       "Headlight driver", "Headlight R pads", "Under the sonar plugs, across to the right disc"),
     H("Headlight driver -> headlight L", [(21, 47, 62), (22, 42, 68)],
       [C("+12 V", RED, 24, drv("L+"), disc("L", "+"), "Headlight driver L+", "Headlight L +"),
        C("switched -", BLK, 24, drv("L-"), disc("L", "-"), "Headlight driver L-", "Headlight L -")],
       "Headlight driver", "Headlight L pads", "Short hop up to the left disc"),
    ]
    for h in out:   # every wire that crosses between the back-cover module and the body gets a service loop
        if h["name"] in COVER_CROSSING: h["slack"] = max(h["slack"], COVER_SLACK)
    for h in out:   # the bundle: average start, path, average end
        A = [c["a"] for c in h["conds"]]; Bp = [c["b"] for c in h["conds"]]
        h["pts"] = [tuple(sum(p[i] for p in A) / len(A) for i in range(3))] + h["path"] + [tuple(sum(p[i] for p in Bp) / len(Bp) for i in range(3))]
        h["dia"] = round(max(2.0, math.sqrt(sum(c["d"] ** 2 for c in h["conds"])) * 1.15), 1)
        h["wires"] = f"{len(h['conds'])} wires"
        h["colour"] = h["conds"][0]["col"]
    return out

def rounded(pts, r=7.0, n=7):
    if len(pts) < 3: return list(pts)
    out = [pts[0]]
    for a, p, b in zip(pts[:-2], pts[1:-1], pts[2:]):
        la, lb = math.dist(a, p), math.dist(p, b)
        if la < 1e-6 or lb < 1e-6: continue
        rr = min(r, la / 2.05, lb / 2.05)
        p1 = tuple(p[i] + (a[i] - p[i]) * rr / la for i in range(3)); p2 = tuple(p[i] + (b[i] - p[i]) * rr / lb for i in range(3))
        for k in range(n + 1):
            t = k / n; out.append(tuple((1 - t) ** 2 * p1[i] + 2 * (1 - t) * t * p[i] + t ** 2 * p2[i] for i in range(3)))
    out.append(pts[-1]); return out

def tube(pts, d, n=3):
    pts = rounded(pts, n=n); parts = []
    for a, b in zip(pts[:-1], pts[1:]):
        v = cq.Vector(b) - cq.Vector(a); L = v.Length
        if L < 0.05: continue
        parts.append(cq.Solid.makeCylinder(d / 2, L, cq.Vector(a), v.normalized()))
    for p in pts[1:-1]:
        parts.append(cq.Solid.makeSphere(d / 2, cq.Vector(p), angleDegrees1=-90, angleDegrees2=90))
    return cq.Compound.makeCompound(parts)

def length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))

def conductor_path(h, c):
    """Points for one conductor: its own pin, then the bundle path, then its own pin (lengths for the cut list)."""
    return [c["a"]] + h["path"] + [c["b"]]

def harness_record(h):
    """One harness as it goes into wiring_data.json. ONE definition on purpose.

    Two scripts write that file - dump_wiring.py and export_viewer_real.py - and their field lists
    had drifted: the viewer one omitted `awg` and `slack` and measured L without the slack. Whichever
    ran second won, so after a viewer export elec/can_ground_check.py died on KeyError 'awg' (and
    check.sh scored it a pass, because the command was piped into tail). Both now call this.
    """
    return dict(name=h["name"], desc=h["desc"], a_conn=h["a_conn"], b_conn=h["b_conn"], dia=h["dia"],
                slack=h.get("slack", 0),
                path=[[round(v, 2) for v in p] for p in h["path"]],
                conds=[dict(sig=c["sig"], col=c["col"], d=c["d"], awg=c["awg"],
                            a=[round(v, 2) for v in c["a"]], b=[round(v, 2) for v in c["b"]],
                            a_lbl=c["a_lbl"], b_lbl=c["b_lbl"],
                            L=round((length(conductor_path(h, c)) + h.get("slack", 0)) / 10, 1))
                       for c in h["conds"]])

def connectors():
    """Mated plug housings / boards at the harness ends: (label, shape, colour)."""
    B = lambda x0, x1, y0, y1, z0, z1: cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))
    out = []
    used = sorted({2, 4, 6, 8, 10, 12, 17, 19, 32, 34, 35, 38, 39, 40})
    for n in used:   # single Dupont housing per Pi pin, 2.54 square, from the header plastic to 17.5 mm
        p = to_robot(PI, (*pi_gpio(n), 0))
        out.append((f"Dupont on Pi pin {n}", B(-23 + 3.5 + 1.0, PBC + 7.5, p[1] - 1.2, p[1] + 1.2, p[2] - 1.2, p[2] + 1.2), "#202020"))
    for name, (k, j, n, p, h, kind) in PL.PLUGS.items():
        out.append((f"Perfboard {name} ({n}-pin {kind})", B(*PL.plug_box(name)), "#f5f5f5" if not kind.startswith("XT30") else "#f2c200"))
    for name in PL.PARTS:
        out.append((f"Perfboard {name}", B(*PL.part_box(name)), "#263238" if name.startswith(("U1", "C1")) else "#c8a165"))
    out.append(("Power P- splice (heat shrink)", B(XSPL[0] - 3, XSPL[0] + 3, XSPL[1] - 2.5, XSPL[1] + 2.5, XSPL[2] - 6, XSPL[2] + 6), "#212121"))
    sx = to_robot("HC-SR04 sonar", (0, -15.5, -1))[0]
    out.append(("Dupont 1x4 on the HC-SR04 pins", B(sx - 1.3, sx + 1.3, -5.1, 5.1, 56, 70), "#202020"))
    out.append(("Dupont 1x4 on the GY-521 header", B(-10.3, -7.7, -5.1, 5.1, ZI, ZI + 8.5), "#202020"))
    out.append(("I2S junction board (20 x 15 perfboard)", B(JUNC[0] - 10, JUNC[0] + 10, JUNC[1] - 7.5, JUNC[1] + 7.5, JUNC[2] - 2.0, JUNC[2] - 0.4), "#3f6e3f"))
    out.append(("Charge-pad ring lugs + M5 nuts", cq.Compound.makeCompound([cq.Solid.makeCylinder(4.6, 4.0, cq.Vector(-32, y, z), cq.Vector(1, 0, 0)) for (y, z) in bc.PADS.values()]), "#b08d57"))
    out.append(("LD19 lead plug (ZH1.5 4-pin)", B(-12.5, -7.5, -27.8, -25.2, 143.5, 147), "#f5f5f5"))
    return out
