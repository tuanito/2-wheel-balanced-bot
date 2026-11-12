"""Electrical model of the robot: every harness wire (from case/wiring.py) + what is inside each board.

Nodes are "Module::pin". Elements:
  link  : same electrical node (PCB trace, a closed switch/fuse, a crimp splice)
  res   : resistor (ohm)            diode : anode -> cathode (forward drop)
Pin attributes (all optional):
  kind  : 'src' (power source, V, Imax) | 'pin' (power input, range, I) | 'gnd' | 'out' (logic out, level)
          | 'in' (logic in, vmax, needs a driver unless pull) | 'io' | 'pass' (passive) | 'bus' (CAN H/L)
"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(HERE, os.pardir, "cad_src", "case"), "/home/claude/case"):
    if os.path.isdir(_p): sys.path.append(_p)       # robot_paths: this machine's layout
import robot_paths as RP
WD = RP.artifact('wiring_diagram.json')

PINS, LINKS, RES, DIODES, NOTES = {}, [], [], [], []


def P(mod, pin, **a):
    PINS[f"{mod}::{pin}"] = a


def L(a, b, why=''):
    LINKS.append((a, b, why))


def Rr(a, b, ohm, why=''):
    RES.append((a, b, ohm, why))


def Dd(a, c, vf, why=''):
    DIODES.append((a, c, vf, why))


# ---------------- parts ----------------
BAT, BMS, KEY, PDB, REG = "Battery 6S 1400", "6S BMS", "Power switch + 30 A fuse", "Power distribution (PDB)", "5 V regulator (D24V50F5)"
PB, CAN, PI, JUN = "Perfboard: ESP32-S3 + level shifter", "CAN board (SN65HVD230)", "Raspberry Pi Zero 2 W", "I2S junction board"
ML, MR = "Motor L (MF6015)", "Motor R (MF6015)"
IMU, SON, EYR, EYL = "IMU (GY-521)", "Sonar (HC-SR04)", "Eye R (8x8 WS2812)", "Eye L (8x8 WS2812)"
TOF = "Zone array (VL53L5CX)"      # fitted INSTEAD of the sonar: same aperture, its own 6-pin plug
MICL, MICR, AMP, SPK = "Mic L (ICS-43434)", "Mic R (ICS-43434)", "Amp (MAX98357A)", "Speaker 3 W"
CAM, LDA, LD = "Camera Module 3", "LD19 USB adapter", "LD19 lidar"
DOCK, FD, USBC, IP, BTN = "Dock contacts", "Charge fuse + diode", "USB-C socket (side)", "IP2368 USB-C charger", "Arm button + RGB LED"
DRV, HLL, HLR = "Headlight driver (D36V6F12 + IRLB8721)", "Headlight L (COB disc)", "Headlight R (COB disc)"
IRR = "IR receivers (4 x TSOP38238)"

# battery + BMS (common-port BMS: B+ = P+, P- switched by its MOSFETs, closed in normal use)
P(BAT, 'lead +', kind='src', V=25.2, Vmin=20.4, Imax=130, label='6S pack')
P(BAT, 'lead -', kind='gnd', ref='B-')
P(BAT, 'balance lead', kind='pass')
P(BMS, 'B+', kind='pass'); P(BMS, 'B-', kind='pass'); P(BMS, 'JST-XH 7p', kind='pass')
P(BMS, 'P+', kind='pass'); P(BMS, 'P-', kind='pass')
L(f"{BMS}::B+", f"{BMS}::P+", 'BMS: common positive')
L(f"{BMS}::B-", f"{BMS}::P-", 'BMS: discharge/charge MOSFETs (closed)')
L(f"{BAT}::balance lead", f"{BMS}::JST-XH 7p", 'balance plug')
# power switch: Pololu Big Pushbutton Power Switch HP (#2813), 4.5-40 V, 16 A continuous / 90 A peak, MOSFETs between
# VIN and VOUT (high side); a push on its button (or on the panel button wired in parallel) toggles it; OFF input high
# = off. GND only feeds its control circuit (~0.1 mA/V). The negatives are spliced beside the board.
PBTN_MOD = "Power button (12 mm)"
P(KEY, 'VIN (BAT+ from the fuse)', kind='pin', range=(4.5, 40), I=0.003, label='Pololu 2813 VIN (control ~0.12 mA/V)')
P(KEY, 'VOUT (BAT+ to the PDB)', kind='pass'); P(KEY, 'P- splice (beside the board)', kind='pass'); P(KEY, 'GND', kind='gnd')
P(KEY, 'OFF', kind='in', vmax=40, vih=1.0, label='Pololu 2813 OFF (high pulse = off)')
P(KEY, 'button pad A', kind='pass'); P(KEY, 'button pad B', kind='pass')
P(PBTN_MOD, 'A', kind='pass'); P(PBTN_MOD, 'B', kind='pass')
KEY_BRIDGE = (f"{KEY}::VIN (BAT+ from the fuse)", f"{KEY}::VOUT (BAT+ to the PDB)", 'power switch on: MOSFETs conduct VIN-VOUT')
# PDB
for n in ('in +', 'in -', 'out +', 'out -'):
    P(PDB, n, kind='pass')
L(f"{PDB}::in +", f"{PDB}::out +", 'PDB + plane'); L(f"{PDB}::in -", f"{PDB}::out -", 'PDB - plane')
# regulator: Pololu D24V50F5, 5 V 5 A, input up to 38 V (VIN min ~5.5 V)
P(REG, 'VIN', kind='pin', range=(5.5, 38), I=None, label='D24V50F5 input'); P(REG, 'GND', kind='gnd')
P(REG, 'VOUT', kind='src', V=5.0, Imax=5.0, label='5 V rail (D24V50F5)')
# motors: LK MF6015-V2 on 24 V nominal (6S)
for M in (ML, MR):
    P(M, 'plug V+', kind='pin', range=(18, 30), I=8.7, label='MF6015 bus: 142 W stand-up peak (sim) / 0.8 / 20.4 V')
    P(M, 'plug V-', kind='gnd')
    P(M, 'plug CAN-H', kind='bus', bus='CAN'); P(M, 'plug CAN-L', kind='bus', bus='CAN')
# CAN board (Waveshare SN65HVD230), 3.3 V, 120 ohm on board
P(CAN, '3V3', kind='pin', range=(3.0, 3.6), I=0.01, label='SN65HVD230'); P(CAN, 'GND', kind='gnd')
P(CAN, 'CTX', kind='in', vmax=3.6, label='SN65HVD230 D'); P(CAN, 'CRX', kind='out', level='3V3')
P(CAN, 'terminal H', kind='bus', bus='CAN', term=120); P(CAN, 'terminal L', kind='bus', bus='CAN')

# ---- perfboard: ESP32-S3-DevKitC-1 + 74AHCT125 + dividers + plugs. Internal nodes start with "PB." ----
E = lambda n: f"{PB}::ESP.{n}"
for g in (1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47):
    P(PB, f"ESP.GPIO{g}", kind='io', vmax=3.6, level='3V3')
P(PB, 'ESP.5V', kind='pin', range=(4.75, 5.25), I=0.30, label='ESP32-S3 DevKitC-1 (5 V in, own 3.3 V LDO)')
P(PB, 'ESP.GND', kind='gnd')
P(PB, 'ESP.3V3', kind='src', V=3.3, Imax=0.5, label='DevKitC 3.3 V LDO (budget left for add-ons)')
# firmware pin roles (include/config.h)
ROLE = {21: 'out', 2: 'out', 41: 'out', 42: 'io', 47: 'in', 4: 'out', 5: 'in', 8: 'io', 9: 'out', 10: 'out', 11: 'in', 12: 'in_pullup', 13: 'in', 14: 'in', 15: 'in', 16: 'in', 17: 'out', 18: 'in', 38: 'out', 39: 'out', 40: 'out', 1: 'adc', 6: 'adc', 7: 'adc'}
for g, r in ROLE.items():
    PINS[E(f"GPIO{g}")]['role'] = r
# jumper pins on the DevKit header (named as in the wiring)
L(f"{PB}::ESP32 3V3 (pin L1)", E('3V3')); L(f"{PB}::ESP32 GND (pin L22)", E('GND'))
L(f"{PB}::ESP32 GPIO4 (pin L4)", E('GPIO4')); L(f"{PB}::ESP32 GPIO5 (pin L5)", E('GPIO5'))
for n in ('ESP32 3V3 (pin L1)', 'ESP32 GND (pin L22)', 'ESP32 GPIO4 (pin L4)', 'ESP32 GPIO5 (pin L5)'):
    P(PB, n, kind='pass')
# plugs J1..J6
J = {'J1-1 (5 V bus, XT30)': '5V', 'J1-2 (GND, XT30)': 'GND', 'J2-1 (5 V)': '5V', 'J2-2 (ESP32 GPIO10)': 'GPIO10', 'J2-3 (divider -> GPIO11)': 'ECHO',
     'J2-4': 'GND', 'J3-1 (3V3)': '3V3', 'J3-2': 'GND', 'J3-3 (ESP32 GPIO9)': 'GPIO9', 'J3-4 (ESP32 GPIO8)': 'GPIO8',
     'J4-1 (5 V, 470 uF)': '5V', 'J4-2 (74AHCT125 out)': 'EYE_OUT', 'J4-3': 'GND', 'J5-1': '5V', 'J5-2': 'GND',
     'J5-3 (ESP32 GPIO17)': 'GPIO17', 'J5-4 (ESP32 GPIO18)': 'GPIO18', 'J5-5 (74AHCT125 in)': 'BUF_A',
     'J6-1 (GPIO12)': 'GPIO12', 'J6-2 (GPIO38 via 330 ohm)': 'LEDR', 'J6-3 (GPIO39 via 330 ohm)': 'LEDG', 'J6-4 (GPIO40 via 330 ohm)': 'LEDB', 'J6-5 (GND)': 'GND',
     'J7-1 (VBAT sense)': 'S_VBAT', 'J7-2 (dock sense)': 'S_DOCK', 'J7-3 (USB sense)': 'S_USB',
     'J8-1 (3V3)': '3V3', 'J8-2 (GND)': 'GND', 'J8-3 (GPIO13)': 'GPIO13', 'J8-4 (GPIO14)': 'GPIO14', 'J8-5 (GPIO15)': 'GPIO15', 'J8-6 (GPIO16)': 'GPIO16', 'J9-1 (GPIO21)': 'GPIO21',
     # the zone array's plug, in the sonar's place on the board: only one of J2 / J2T is populated
     'J2T-1 (3V3)': '3V3', 'J2T-2': 'GND', 'J2T-3 (ESP32 GPIO42)': 'GPIO42',
     'J2T-4 (ESP32 GPIO41)': 'GPIO41', 'J2T-5 (ESP32 GPIO2)': 'GPIO2', 'J2T-6 (ESP32 GPIO47)': 'GPIO47'}
for p in J:
    P(PB, p, kind='pass', plug=p.split('-')[0])
INT = {'5V': E('5V'), 'GND': E('GND'), '3V3': E('3V3')}
for g in (2, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 21, 41, 42, 47):
    INT[f'GPIO{g}'] = E(f'GPIO{g}')
for p, n in J.items():
    if n in INT:
        L(f"{PB}::{p}", INT[n], 'perfboard trace')
# sonar echo divider 1k (top) / 2k (bottom) -> GPIO11
P(PB, 'ECHO_DIV', kind='pass')
Rr(f"{PB}::J2-3 (divider -> GPIO11)", f"{PB}::ECHO_DIV", 1000, 'echo divider top'); Rr(f"{PB}::ECHO_DIV", E('GND'), 2000, 'echo divider bottom')
L(f"{PB}::ECHO_DIV", E('GPIO11'))
# 74AHCT125: VCC 5 V, 1OE low, 1A from the Pi (J5-5), 1Y -> 330 ohm -> J4-2
P(PB, 'U1.VCC', kind='pin', range=(4.5, 5.5), I=0.001, label='74AHCT125'); P(PB, 'U1.GND', kind='gnd'); P(PB, 'U1.1OE', kind='in', vmax=5.5)
P(PB, 'U1.1A', kind='in', vmax=5.5, vih=2.0, label='74AHCT125 1A (TTL in)'); P(PB, 'U1.1Y', kind='out', level='5V')
L(f"{PB}::U1.VCC", E('5V')); L(f"{PB}::U1.GND", E('GND')); L(f"{PB}::U1.1OE", E('GND')); L(f"{PB}::U1.1A", f"{PB}::J5-5 (74AHCT125 in)")
Rr(f"{PB}::U1.1Y", f"{PB}::J4-2 (74AHCT125 out)", 330, 'WS2812 data series resistor')
P(PB, 'C470', kind='pass'); L(f"{PB}::C470", E('5V'))
# status LED: GPIO38 via 330 ohm per the wiring label; G and B labels name no resistor
Rr(E('GPIO38'), f"{PB}::J6-2 (GPIO38 via 330 ohm)", 330, 'LED R')
LED_RES = {'G': None, 'B': None}     # filled in by the check if a resistor is documented
Rr(E('GPIO39'), f"{PB}::J6-3 (GPIO39 via 330 ohm)", 330, 'LED G'); Rr(E('GPIO40'), f"{PB}::J6-4 (GPIO40 via 330 ohm)", 330, 'LED B')
# sense dividers (INSTALL.md: "GPIO1/6/7 battery/dock/USB-charge sense dividers"), inputs from off-board
for g, top, bot, src in ((1, 100e3, 12e3, 'VBAT'), (6, 100e3, 12e3, 'DOCK'), (7, 56e3, 10e3, 'USB')):
    P(PB, f"SENSE_{src}", kind='pass', offboard=True)
    L(f"{PB}::SENSE_{src}", f"{PB}::J7-{ {'VBAT': 1, 'DOCK': 2, 'USB': 3}[src]} ({ {'VBAT': 'VBAT', 'DOCK': 'dock', 'USB': 'USB'}[src]} sense)", 'J7 -> divider top')
    Rr(f"{PB}::SENSE_{src}", E(f'GPIO{g}'), top, f'{src} divider top'); Rr(E(f'GPIO{g}'), E('GND'), bot, f'{src} divider bottom')

# IR beacon receivers: 4 x TSOP38238 (2.5-5.5 V, ~0.45 mA, active-low open-collector out with ~30k internal pull-up),
# VS and GND chained from one to the next under the top skin; outputs to GPIO13..16 (ESP32 decodes the beacon codes)
P(IRR, 'front-left VS', kind='pin', range=(2.5, 5.5), I=0.0018, label='4 x TSOP38238 (0.45 mA each)')
P(IRR, 'back-left GND', kind='gnd')
for nm in ('front-left', 'front-right', 'back-left', 'back-right'):
    P(IRR, f'{nm} OUT', kind='out', level='3V3', label=f'TSOP38238 {nm} (pulled up to VS = 3.3 V)')

# ---- Raspberry Pi Zero 2 W header ----
for n in ('pin 2 (5 V)', 'pin 4 (5 V)'):
    P(PI, n, kind='pass')
L(f"{PI}::pin 2 (5 V)", f"{PI}::PI.5V"); L(f"{PI}::pin 4 (5 V)", f"{PI}::PI.5V")
P(PI, 'PI.5V', kind='pin', range=(4.75, 5.25), I=1.0, label='Pi Zero 2 W + camera')
P(PI, 'pin 6 (GND)', kind='gnd'); P(PI, 'pin 39 (GND)', kind='gnd'); L(f"{PI}::pin 6 (GND)", f"{PI}::pin 39 (GND)", 'Pi ground plane')
P(PI, 'pin 17 (3V3)', kind='src', V=3.3, Imax=0.05, label='Pi 3V3 pin (budget for add-ons)')
P(PI, 'pin 8 (GPIO14 TXD)', kind='out', level='3V3'); P(PI, 'pin 10 (GPIO15 RXD)', kind='in', vmax=3.6)
P(PI, 'pin 19 (GPIO10 MOSI)', kind='out', level='3V3')
P(PI, 'pin 12', kind='out', level='3V3', label='GPIO18 PCM_CLK'); P(PI, 'pin 35', kind='out', level='3V3', label='GPIO19 PCM_FS')
P(PI, 'pin 38', kind='in', vmax=3.6, label='GPIO20 PCM_DIN'); P(PI, 'pin 40', kind='out', level='3V3', label='GPIO21 PCM_DOUT')
P(PI, 'CSI connector (22-pin)', kind='pass'); P(PI, 'micro-USB OTG', kind='pass')
L(f"{PI}::micro-USB OTG", f"{PI}::PI.5V", 'OTG port VBUS is the Pi 5 V rail')
# ---- I2S junction (strip board): each strip is one node ----
for n in ('BCLK', 'WS', 'DOUT', 'DIN (amp)', '3V3', '5 V', 'GND'):
    P(JUN, n, kind='pass')
# ---- devices ----
P(IMU, 'VCC', kind='pin', range=(3.0, 5.5), I=0.005, label='GY-521 (own 3.3 V LDO, 4.7k I2C pull-ups)'); P(IMU, 'GND', kind='gnd')
P(IMU, 'SCL', kind='io', vmax=3.6, pull='3V3'); P(IMU, 'SDA', kind='io', vmax=3.6, pull='3V3')
P(SON, 'Vcc', kind='pin', range=(4.5, 5.5), I=0.015, label='HC-SR04'); P(SON, 'Gnd', kind='gnd')
P(SON, 'Trig', kind='in', vmax=5.5, vih=2.0); P(SON, 'Echo', kind='out', level='5V')
# VL53L5CX carrier (Pololu #3417): 3.3 V part, ~100 mA while ranging, its own I2C bus so the IMU's
# 400 kHz cap does not apply. Address 0x52; the MPU6050 is 0x68, so even a shared bus would not clash.
P(TOF, 'VIN', kind='pin', range=(2.8, 5.5), I=0.100, label='VL53L5CX carrier (on-board regulator)')
P(TOF, 'GND', kind='gnd')
P(TOF, 'SDA', kind='io', vmax=3.6, pull='3V3'); P(TOF, 'SCL', kind='io', vmax=3.6, pull='3V3')
P(TOF, 'LPn', kind='in', vmax=3.6, vih=2.0); P(TOF, 'INT', kind='out', level='3V3')
for EY, out in ((EYR, True), (EYL, False)):
    P(EY, '5V', kind='pin', range=(3.5, 5.3), I=64 * 0.06 * 0.25 + 64 * 0.001, label='64 x WS2812 at the software cap (eyes.py MAX_BRIGHT 0.25)')
    P(EY, 'GND', kind='gnd'); P(EY, 'DIN', kind='in', vmax=5.8, vih_frac=0.7)
    if out:
        P(EY, 'DOUT', kind='out', level='5V'); P(EY, '5V out', kind='pass'); P(EY, 'GND out', kind='pass')
        L(f"{EY}::5V", f"{EY}::5V out", 'panel 5 V'); L(f"{EY}::GND", f"{EY}::GND out", 'panel GND')
for MI in (MICL, MICR):
    P(MI, '3V', kind='pin', range=(1.65, 3.63), I=0.0005, label='ICS-43434'); P(MI, 'GND', kind='gnd')
    P(MI, 'BCLK', kind='in', vmax=3.63); P(MI, 'WS', kind='in', vmax=3.63); P(MI, 'DOUT', kind='out', level='3V3', tristate=True)
    P(MI, 'SEL', kind='in', vmax=3.63)
P(AMP, 'Vin', kind='pin', range=(2.5, 5.5), I=1.0, label='MAX98357A, 3 W into 4 ohm (peak)'); P(AMP, 'GND', kind='gnd')
for n in ('LRC', 'BCLK', 'DIN'):
    P(AMP, n, kind='in', vmax=5.5, vih=1.3)
P(AMP, 'terminal +', kind='pass'); P(AMP, 'terminal -', kind='pass'); P(SPK, '+', kind='pass'); P(SPK, '-', kind='pass')
P(CAM, 'FPC connector', kind='pass')
P(LDA, 'USB', kind='pass'); L(f"{LDA}::USB", f"{LDA}::5V", 'adapter passes USB VBUS to the lidar')
P(LDA, '5V', kind='pass'); P(LDA, 'GND', kind='gnd'); P(LDA, 'RX', kind='in', vmax=5.5); P(LDA, '(n.c.)', kind='pass')
P(LD, 'P5V', kind='pin', range=(4.5, 5.5), I=0.30, label='LD19 (start-up current)'); P(LD, 'GND', kind='gnd')
P(LD, 'Tx', kind='out', level='3V3'); P(LD, 'PWM', kind='pass')
NOTES.append("LD19 adapter: GND reaches the Pi through the USB cable (modelled as a link).")
L(f"{LDA}::GND", f"{PI}::pin 6 (GND)", 'USB cable ground')
# dock + charge path
P(DOCK, '+ (ring lug)', kind='src', V=25.2, Imax=3.0, label='dock charger (CC/CV 25.2 V)', charger=True)
P(DOCK, '- (ring lug)', kind='gnd', ref='DOCK-')
P(FD, 'IN (5 A fuse -> SB560 anode)', kind='pass'); P(FD, 'OUT (SB560 cathode)', kind='pass'); P(FD, 'P- pass-through', kind='pass')
Dd(f"{FD}::IN (5 A fuse -> SB560 anode)", f"{FD}::OUT (SB560 cathode)", 0.5, 'SB560 (60 V 5 A Schottky)')
P(USBC, 'USB-C socket', kind='pass'); P(IP, 'USB-C', kind='pass')
P(IP, 'VBUS pad', kind='src', V=20.0, Imax=3.0, label='USB-C VBUS (PD up to 20 V)', charger=True, external=True)
P(IP, 'BAT+', kind='src', V=25.2, Imax=3.0, label='IP2368 charger output', charger=True); P(IP, 'BAT-', kind='gnd')
P(BTN, 'Arm button', kind='pass'); P(BTN, 'Button + LED common', kind='gnd')
for c in 'RGB':
    P(BTN, f'RGB LED {c}', kind='in', vmax=5, led=True)
L(f"{BTN}::Arm button", f"{BTN}::BTN.closed", 'button (open at rest)')
P(BTN, 'BTN.closed', kind='pass')

# headlights: Pololu D36V6F12 (12 V 0.6 A, input 13..50 V) + IRLB8721 low-side switch, gate from the Pi's PWM0 (GPIO12)
P(DRV, 'VIN (1 A inline fuse)', kind='pin', range=(12.2, 50.0), I=None, label='D36V6F12 input')
P(DRV, 'GND', kind='gnd'); P(DRV, 'GND (PWM)', kind='gnd'); L(f"{DRV}::GND (PWM)", f"{DRV}::GND", 'driver board ground')
P(DRV, 'DRV.12V', kind='src', V=12.0, Imax=0.6, label='12 V headlight rail (D36V6F12)')
P(DRV, 'PWM (GPIO12)', kind='in', vmax=20.0, pull='GND', label='IRLB8721 gate via 100 ohm, 100k pull-down (off while the Pi boots)')
for side in ('L', 'R'):
    P(DRV, f'{side}+', kind='pass'); P(DRV, f'{side}-', kind='pass')
    L(f"{DRV}::DRV.12V", f"{DRV}::{side}+", '12 V out to the discs')
    L(f"{DRV}::{side}-", f"{DRV}::GND", 'IRLB8721 drain-source (on when lit)')
for HL in (HLL, HLR):
    P(HL, '+', kind='pin', range=(10.8, 13.2), I=0.2, label='COB disc 12 V 2.4 W'); P(HL, '-', kind='gnd')
P(PI, 'pin 32', kind='out', level='3V3', label='GPIO12 PWM0 (headlights)'); P(PI, 'pin 34', kind='gnd')
L(f"{PI}::pin 34", f"{PI}::pin 6 (GND)", 'Pi ground plane')

# ---------------- connectors (rating in A per contact) ----------------
CONN = {  # end label prefix -> (connector, A per contact)
    'Perfboard J1': ('XT30PW', 15.0), 'Perfboard J': ('2.54 mm header + plug', 3.0), 'Pi pin': ('Dupont on the Pi header', 3.0), 'HC-SR04': ('Dupont 1x4', 3.0),
    'GY-521': ('Dupont 1x4', 3.0), 'XT60': ('XT60', 60.0), 'Battery lead': ('XT60', 60.0), 'Motor': ('LK motor plug (rating not documented)', None),
    'Junction': ('solder', 99), 'Mic': ('solder', 99), 'Amp': ('solder', 99), 'Eye': ('solder', 99), 'PDB': ('solder', 99),
    'Regulator': ('solder', 99), 'Headlight': ('solder', 99), 'BMS': ('solder', 99), 'CAN board terminal': ('screw terminal', 6.0), 'CAN board': ('Dupont', 3.0),
}
INLINE = {  # harness name -> (inline connector, A per contact): the plugs that let the back cover come off as one module
    'BMS -> 30 A fuse -> power switch': ('XT60 cover plug (female on the BMS side)', 60.0), 'Power switch -> PDB': ('XT60 cover plug (female on the switch side)', 60.0),
    'Power button -> switch': ('JST-XH 3p cover plug', 3.0), 'CAN bus -> motor R': ('JST-XH 2p cover plug', 3.0)}
AWG_A = {12: 41, 14: 32, 16: 22, 18: 16, 20: 11, 22: 7, 24: 3.5, 26: 2.2, 28: 1.4}   # chassis wiring, PVC (silicone ~1.3x)
FUSES = [('BMS -> 30 A fuse -> power switch', 30.0, 'P+ (30 A fuse inline)'), ('Dock pads -> diode + fuse', 5.0, 'Pad + (charge in)'),
         ('PDB -> 5 V regulator', 3.0, 'VIN (3 A inline fuse)'), ('PDB -> headlight driver', 1.0, 'VIN (1 A inline fuse)')]
SILICONE = 1.35   # BOM: silicone wire throughout -> ~1.35 x the PVC chassis rating


def wires():
    D = json.load(open(WD))
    return D['wires']
