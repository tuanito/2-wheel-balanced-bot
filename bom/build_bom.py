from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter

wb = Workbook()
ws = wb.active; ws.title = "BOM"
F = "Arial"
H = Font(name=F, bold=True, color="FFFFFF"); HF = PatternFill("solid", fgColor="1F3864")
SEC = PatternFill("solid", fgColor="D9E1F2"); BLUE = Font(name=F, color="0000FF"); BLK = Font(name=F)
GEO = PatternFill("solid", fgColor="EAF4EA")
thin = Side(style="thin", color="BFBFBF"); BOX = Border(top=thin, bottom=thin, left=thin, right=thin)
wrap = Alignment(wrap_text=True, vertical="top")
LINKF = Font(name=F, color="0563C1", underline="single")
SMALL_I = Font(name=F, italic=True, size=9)

ws["A1"] = "Self-balancing robot (stand-up capable) — Bill of Materials V1.1 (shape C + lidar, contact + USB-C charging, voice)"
ws["A1"].font = Font(name=F, bold=True, size=14)
ws["A2"] = ("v3: all-in-one CAN motors on 6S, ESP32-S3 for real-time balance, Pi Zero 2 W for camera/lidar/remote, LED eyes, wheels under the body, "
            "battery on a slide-in shelf above the motors, contact charging (dock) + USB-C PD charging. Prices in EUR incl. typical VAT, Nov 2026; 'est.' = typical "
            "market price, not a quote. Blue cells are inputs. Green columns F–I: size, how it is fixed, where it sits, and whether it was checked in CAD. "
            "Robot frame: X forward, Y to the robot's left, Z up, origin on the wheel axle (the floor is at Z = −54 mm).")
ws["A2"].font = Font(name=F, italic=True, size=9); ws.merge_cells("A2:P2"); ws["A2"].alignment = wrap
ws.row_dimensions[2].height = 42

ASIA = {  # component: (unit price USD, shop label, url, note)
 "Wheel motor (all-in-one)": (160, "aifitlab (ships from China)", "https://aifitlab.com/products/lkmtech-mf6015-v2-motor",
     "Same offer as the EU column: it already ships from China. LKMTECH's own AliExpress/Taobao store may be cheaper; couldn't verify from here."),
 "Main controller": (15, "LCSC (Shenzhen)", "https://www.lcsc.com/product-detail/C20541519.html", "est. ~US$15; LCSC part C20541519."),
 "CAN transceiver": (4.99, "Waveshare store (Shenzhen)", "https://www.waveshare.com/product/iot-communication/wired-comm-converter/rs232-rs485-can-dali2/sn65hvd230-can-board.htm", "US$4.99; same Waveshare board as the EU column."),
 "IMU (upgrade)": (5.8, "SCCC Ltd (Hong Kong)", "https://scccltd.com/products/icm-42688-p-6-axis-imu-module", "HK$45 (~US$5.8) generic ICM-42688-P module vs ~€55 TDK Pmod board."),
 "2D lidar": (69, "sensorlidar.com (LDROBOT reseller)", "https://sensorlidar.com/products/ldrobot-ld06-ld19-d300-of-laser-radar-lidar-scanner-360-scanning-d500-lidar-kit-support-ros1-and-ros2-for-indoor-and-outdoor",
     "US$69, choose 'LD19 with connector' or 'D300 kit' (= LD19 + USB adapter board)."),
 "Companion computer": (18.85, "AliExpress (SpotPear deal)", "https://www.aliexpress.com/item/1005007982832720.html", "~US$18.85 incl. case & pins in a coin deal; not cheaper than EU."),
}
LINKS = {
 "Wheel motor (all-in-one)": ("aifitlab (LKMTECH dealer)", "https://aifitlab.com/products/lkmtech-mf6015-v2-motor"),
 "Wheel (pair)": ("Wheelspin Models (UK)", "https://wheelspinmodels.co.uk/i/271264/"),
 "Main controller": ("BerryBase (DE)", "https://www.berrybase.de/it/espressif-esp32-s3-devkitc-1-n8r8-wifi-bluetooth-le-8mb-flash-8mb-psram-240mhz-5v-3-3v"),
 "CAN transceiver": ("EXP-Tech (DE)", "https://exp-tech.de/en/products/sn65hvd230-can-board"),
 "Speaker": ("DigiKey NL", "https://www.digikey.nl/en/products/detail/adafruit-industries-llc/3968/9745251"),
 "Speaker amplifier": ("Electrokit (SE)", "https://www.electrokit.com/en/product/forstarkare-klass-d-3w-mono-i2s/"),
 "Microphones": ("Electrokit (SE)", "https://www.electrokit.com/en/mems-microphone-ics-43434-i2s"),
 "IMU (upgrade)": ("DigiKey (EU)", "https://www.digikey.at/en/products/detail/tdk-invensense/QCIOT-ICM42688P/25676496"),
 "Companion computer": ("Welectron (DE)", "https://welectron.com/Raspberry-Pi-Zero-2-W_1"),
 "Camera": ("DigiKey (EU)", "https://www.digikey.es/en/products/detail/raspberry-pi/SC1174/24627137"),
 "Headlight discs": ("Bits & Parts (NL)", "https://www.bitsandparts.nl/en/led-spot-disc-cob-12w-high-power-pure-white-42.5mm-p1905035"),
 "Headlight driver": ("Opencircuit (NL)", "https://opencircuit.shop/Product/12V-600mA-Step-Down-Voltage-Regulator-2"),
 "2D lidar": ("RobotShop EU", "https://eu.robotshop.com/products/dfrobot-dtof-ld19-laser-lidar-sensor-kit-12m"),
 "Power switch": ("Opencircuit (NL)", "https://opencircuit.shop/product/big-pushbutton-power-switch-reverse-voltage-2"),
 "Battery": ("FPV24 (DE)", "https://www.fpv24.com/en/tattu/tattu-r-line-v40-lipo-akku-130c-xt60-1400mah-6s"),
 "Main fuse": ("Galaxus (CH)", "https://www.galaxus.ch/en/s4/product/parts-kfz-flachsicherungs-halter-flachsicherung-mini-30-a-331-mm-1-st-30-a-fuses-56408206"),
 "5 V regulator": ("EXP-Tech (DE)", "https://exp-tech.de/en/products/pololu-5v-5a-step-down-voltage-regulator-d24v50f5"),
 "Prototype board": ("BerryBase (DE)", "https://www.berrybase.de/en/lochrasterplatine-doppelseitig-verschiedene-groessen-set-mit-32-stueck"),
 "Heat-set inserts (M3)": ("3DJake (CH)", "https://www.3djake.ch/en-CH/ruthex/threaded-insert-m3-100-pieces?sai=15206"),
 "Heat-set inserts (M2)": ("3DJake (CH)", "https://www.3djake.ch/en-CH/ruthex/threaded-insert-m2-70-pieces?sai=15204"),
 "6S BMS with balancing": ("Sigmanortec (RO)", "https://sigmanortec.ro/en/lithium-ion-bms-module-6s-40a-252v-with-balance"),
 "Dock charger (25.2 V)": ("akkuteile.de (DE)", "https://akkuteile.de/en/charger/for-lithium-ion-batteries/fuyuang-21-6v-22-2v-25-2v-li-ion-charger-6s-dc-plug-5-5mm-2-1mm_400623_2608"),
 "USB-C panel socket": ("Melopero (IT)", "https://www.melopero.com/en/shop/components/Cavi/panelmountusbcextensioncable6/"),
 "Motor config adapter": ("OpenELAB (EU)", "https://openelab.io/it/collections/brands-lkmtech"),
 "LED eyes": ("PMD Way (UK)", "https://pmdway.com/products/high-density-8x8-grid-addressable-rgb-led-pixel-matrix"),
 "Soldering iron + insert tips": ("3DJake (CH)", "https://3djake.ch/en-CH/ruthex/soldering-tips-melting-aid-set"),
}
NOLINK_PRINTED = "Printed on your printer (filament)"
NOLINK = {"IMU (start)": "Already owned (from your kit)", "Sonar": "Already owned (from your kit)",
          "Gamepad": "Any electronics/gaming retailer (Digitec CH)", "LiPo charger": "Any RC shop (FPV24, Brack CH)",
          "Damping": "Any hardware/DIY store", "Motor cable set": "Same seller as the motors",
          "Motor mounting screws": "Hardware store / screw assortment",
          "Wheel hardware": "Screw assortment + ruthex M4 inserts (3DJake CH)",
          "Case screws": "Any screw shop (Bossard, Schraubenkönig, Galaxus)",
          "Small-board screws": "Any screw shop (Bossard, Schraubenkönig, Galaxus)",
          "Threadlocker": "Any hardware store (Jumbo, Hornbach, Galaxus)",
          "USB-C PD charger board": "AliExpress / Amazon: search 'IP2368 2-6S 100W bidirectional'",
          "Dock contact rods": "Any model shop / Conrad / Modulor: brass rod Ø4, compression spring assortment",
          "Robot charge contacts": "Any screw shop (brass M5 countersunk DIN 963) + Reichelt/Conrad (M5 ring lugs)",
          "Dock DC jack + wire": "Reichelt / Conrad",
          "Power button": "Reichelt / Conrad / AliExpress: 12 mm momentary push button, NO",
          "IR beacon receivers": "Mouser / Reichelt / LCSC: TSOP38238",
          "Dock controller board": "Reichelt / LCSC (ATtiny85, MOSFETs, MF-R110, SMBJ28A) + Pololu D24V5F5",
          "Dock IR beacon LEDs": "Reichelt / Mouser: Vishay TSAL6200",
          "Retro-reflective tape": "Any hardware store / Galaxus: 3M reflective tape",
          "Charge isolation + detect": "Generic parts: Reichelt, Conrad, LCSC"}

# columns
COLS = ["#", "Category", "Component", "Recommended part", "Key spec / why",
        "Size L×W×H (mm)", "Mounting (holes / insert / fixing)", "Location in robot", "CAD fit check",
        "Qty", "Unit price (EUR)", "Line total (EUR)", "Unit mass (g)", "Line mass (g)", "Required?", "Source / note", "Buy link (merchant)",
        "Asia unit price (USD)", "Asia unit price (EUR)", "Asia line total (EUR)", "Asia link (merchant)", "Asia note", "Cheapest line total (EUR)"]
C = {name: get_column_letter(i + 1) for i, name in enumerate(["n", "cat", "comp", "part", "spec", "size", "mount", "loc", "fit",
     "qty", "price", "line", "mass", "lmass", "req", "src", "link", "ausd", "aeur", "aline", "alink", "anote", "cheap"])}
NCOL = len(COLS)
for c, h in enumerate(COLS, 1):
    cell = ws.cell(row=4, column=c, value=h); cell.font = H; cell.fill = HF; cell.alignment = wrap; cell.border = BOX
    if 6 <= c <= 9: cell.fill = PatternFill("solid", fgColor="2E6B30")
    if c >= 18: cell.fill = PatternFill("solid", fgColor="7B2C2C")
ws["R2"] = "USD → EUR rate"; ws["R2"].font = Font(name=F, bold=True)
ws["S2"] = 0.87; ws["S2"].font = BLUE; ws["S2"].fill = PatternFill("solid", fgColor="FFFF00")
ws["T2"] = "Assumption: edit to today's rate. Asia prices exclude shipping and Swiss import VAT (~8.1%)."
ws["T2"].font = SMALL_I

CAD = "OK: modelled in CAD, no clash"
OFF = "Off the robot"
PR = "Printed part: is the CAD"
# (cat, comp, part, spec, size, mount, loc, fit, qty, price, mass, req, src)
rows = [
 ("Drive", None),
 ("Drive", "Wheel motor (all-in-one)", "LKMTECH MF6015-V2, 15T winding, CAN version",
  "Motor + FOC driver + 18-bit encoder in one unit. 24 V class, driver input 7.4–32 V (fine on 6S). ~0.8 N·m rated, several N·m peak. You only send torque commands over CAN (0xA1, 1 Mbps).",
  "Ø58 × 34 (axial) + connector", "Back face: 4 × M2.5 on 20 × 20 mm, 8 mm deep, Ø49 centring boss into the wall pocket", "Back-to-back on the double centre wall, axle at Z 0; back faces at Y ±22",
  "OK: manufacturer STEP in CAD, no clash", 2, 150, 251, "Yes", "aifitlab ~$160. Alt: MyActuator RMD-L-7015. Order CAN, not RS485."),
 ("Drive", "Wheel (pair)", "Fastrax FAST1101B 1/10 short-course wheels, pre-glued, pair",
  "Foam inserts, ready to fit. Any 1/10 SC wheel pair with 12 mm hex works.",
  "Ø108.6 × 45 each, 12 mm hex", "12 mm hex on the printed adapter, M4×12 + washer into the adapter's insert", "Under the body in arches: Y 59 → 104 (2/3 inside, 15 mm out)",
  "OK: proxy cylinder Ø108.6 × 45, 4 mm arch clearance", 1, 17.5, 220, "Yes", "Wheelspin Models UK ~£15. Offset not published: measure it and set NECK_L in wheel_adapter.py."),
 ("Drive", "Wheel adapter", "2 × printed MF6015 → 12 mm hex adapter (mf6015_hex12_adapter.stl)",
  "Ø12.6 spigot centres in the motor's Ø12.7 bore; 12 mm hex + M4 insert holds the wheel. PETG or PA-CF, 100% infill, hex end down.",
  "Ø36 × 31.5", "4 × M3×8 on Ø25 into the rotor face; M4 heat-set insert in the hex", "Between rotor face and wheel hub", PR, 1, 1, 32, "Yes", NOLINK_PRINTED),
 ("Drive", "Wheel hardware", "8 × M3×8 socket screws (ISO 4762), 2 × M4 heat-set inserts (ruthex M4), 2 × M4×12 + washers",
  "M3×8 → ~2.5 mm into the motor's shallow threads: check they don't bottom out.", "—", "—", "Wheel adapters", "—", 1, 5, 8, "Yes", "est."),
 ("Real-time control", None),
 ("Real-time control", "Main controller", "Espressif ESP32-S3-DevKitC-1 (N8R8)",
  "Balance policy at 500–1000 Hz, CAN to motors, IMU, sonar, charge detect, watchdog. UART to the Pi.",
  "63 × 25.4 × 10 (with pins)", "Female headers on the perfboard", "On the perfboard, back cover (right half)", "OK: inside perfboard envelope 70 × 50 × 20", 1, 12, 10, "Yes", "BerryBase (DE)."),
 ("Real-time control", "CAN transceiver", "Waveshare SN65HVD230 CAN Board (3.3 V) + 2 × M3 nylon spacers 2.5 mm + 2 × M3×10 nylon screws + nuts",
  "One needed (ESP32 side); motors have CAN built in. 120 Ω at both bus ends. Spacers clear its solder tails.",
  "~31 × 21 × 8 (est.)", "2 × M3 nylon through the perfboard (drill Ø3.2 under its holes, perfboard columns 1-11)", "On the perfboard, lower right (component side)", "OK: 3D fit check", 1, 7, 3, "Yes", "EXP-Tech (DE)."),
 ("Real-time control", "IMU (start)", "MPU6050 module (GY-521) from your kit", "Good enough for a first build.",
  "21 × 16 × 3", "2 × M2×6 + washers into M2 inserts on the tray (holes 15.5 mm apart)", "Floor tray at the axle line (X −10…6, Z −19)", CAD, 1, 0, 2, "Yes", "Already owned."),
 ("Real-time control", "IMU (upgrade)", "TDK ICM-42688-P Pmod board (QCIOT-ICM42688P)", "Lower noise and drift; SPI at 1–8 kHz.",
  "~25 × 20 × 5 (est.)", "Tray IMU posts (re-drill if holes differ) or foam tape", "Floor tray at the axle line", "Same space as the GY-521: check hole spacing", 1, 55, 3, "Optional", "DigiKey ~€55 incl. VAT."),
 ("Real-time control", "Sonar", "HC-SR04 from your kit",
  "Emergency stop on the ESP32. Echo is 5 V: 1 kΩ/2 kΩ divider to 3.3 V.",
  "45 × 20 × 15.5 (transducers Ø16)", "4 × M2×6 into M2 inserts on 9 mm posts on the front panel (42 × 17 mm pattern)", "Front panel, centre, Z 70–90 ('mouth')", CAD, 1, 0, 9, "Yes", "Already owned."),
 ("Real-time control", "Depth sensor (alternative to the sonar)", "ST VL53L5CX 8 \u00d7 8 zone time-of-flight on a carrier (Pololu #3417)",
  "Fits INSTEAD of the HC-SR04, in the same front aperture. 64 zones over 45 \u00d7 45\u00b0, so every angle that "
  "reaches the first tread is covered at once and the robot measures a step standing still - no nod, no lean. "
  "I\u00b2C (400 kHz\u20131 MHz) on the IMU bus, 3.3 V, ~100 mA while ranging, ~84 kB firmware uploaded at power-on "
  "(~2.8 s at 400 kHz). Needs crosstalk calibration if it sits behind a window.",
  "13 \u00d7 18 \u00d7 3 (carrier)", "2 \u00d7 M2\u00d76 into M2 inserts on the front panel's 30\u00b0 seat (holes 18 mm apart - CHECK against your carrier)",
  "Front panel, centre, Z 134\u2013146 (194 mm above ground, tilted 30\u00b0 down)", CAD, 1, 19, 2, "Optional",
  "Pololu #3417 $19.95; VL53L8CX #3419 $24.95 is the near drop-in upgrade (better in ambient light, SPI option)."),
 ("Real-time control", "Arm button + status LED", "12 mm momentary push-button, 5 mm RGB LED (common cathode), 3 × 330 Ω (one per colour, on the perfboard)",
  "Firmware only arms on an explicit user action.", "Button Ø12 × 15; LED Ø5", "Button: own nut through a Ø12.2 hole; LED: press-fit in Ø5.2 hole", "Top, front-left (button X 10, Y 40; LED X −6, Y 40)", CAD, 1, 2, 3, "Yes", "est."),
 ("Companion computer & sensors", None),
 ("Companion computer & sensors", "Companion computer", "Raspberry Pi Zero 2 W + header + small heatsink",
  "Camera streaming (WebRTC), web app, gamepad, lidar, LED eyes. Nothing time-critical.",
  "65 × 30 × 7 (with heatsink)", "4 × M2×6 + washers into M2 inserts on 9 mm posts (58 × 23 mm pattern)", "Back cover, left half, Z 86–117", CAD, 1, 22, 16, "Yes", "Welectron (DE) €17.90 + header/heatsink ~€4."),
 ("Companion computer & sensors", "microSD card", "32 GB A1/A2 microSD (e.g. SanDisk High Endurance)", "Read-only root / overlay FS.",
  "15 × 11 × 1", "In the Pi", "Pi", "—", 1, 10, 1, "Yes", "est."),
 ("Companion computer & sensors", "Camera", "Raspberry Pi AI Camera (SC1174, Sony IMX500 with on-sensor neural network)",
  "Person / cat detection on the camera (~30 fps), the Pi only gets the boxes. 66° x 52° view, f/1.79, manual focus (set ~1.5 m). Comes with the 22-pin cable for the Pi Zero.",
  "25 × 24 × 11.9", "4 × M2×6 into M2 inserts on 8.5 mm posts (21 × 12.5 mm pattern, same as Camera Module 3)", "Front panel, between the eyes, Z 112; lens ~3 mm behind the face", CAD, 1, 72.7, 6, "Yes", "DigiKey EU €72.66 incl. VAT (Pi Hut £67.20)."),
 ("Companion computer & sensors", "Camera window", "1 mm clear polycarbonate (or acrylic) sheet, 19 × 19 mm piece",
  "Protects the lens when the robot lands on its face. Polycarbonate is near unbreakable.", "19 × 19 × 1", "In the 1.5 mm pocket on the outer face, 2 small dots of clear glue at the corners", "Front panel, over the camera", CAD, 1, 3, 0, "Yes", "est. (any hobby store, offcut)."),
 ("Companion computer & sensors", "LED eyes", "2 × 8×8 RGB matrix WS2812B-2020, ~26 mm square (PMD Way 40003444B)",
  "Programmable eyes + white night light. Cap brightness in software (~1.2 A max).",
  "26 × 26 × 3 each (≤26.4 square fits)", "Pocket 26.4 × 2.2 deep behind the window, held by a printed eye retainer (2 × M2×6 into inserts)", "Front panel, Y ±32, Z 112", CAD, 2, 11, 4, "Yes", "PMD Way (UK) $11.95 each."),
 ("Companion computer & sensors", "Headlight discs", "2 × COB LED disc Ø30 × 4 mm, 12 V 2.4 W, pure white (Bits & Parts)",
  "Night lighting for the camera stream: ~200–250 lm each. Driven at ≤ 70 % (headlight.py MAX_LEVEL): they sit in PETG pockets.", "Ø30 × 4", "In the panel collars behind 45° windows, clamped by the printed headlight retainers", "Front panel, Y ±42, Z 76", CAD, 2, 2.75, 3, "Yes", "Bits & Parts (NL) €2.75 each."),
 ("Companion computer & sensors", "Headlight driver", "Pololu D36V6F12 (12 V 600 mA, 12.2–50 V in) + IRLB8721 MOSFET (TO-220, laid flat) + 100 Ω + 100 kΩ, on a 10 × 8-hole piece of the 2 × 8 cm perfboard",
  "6S -> 12 V for the discs; the Pi's PWM0 (GPIO12, pin 32) switches the MOSFET at 1 kHz. 100 k gate pull-down keeps them off while the Pi boots. Fed from the PDB through a 1 A inline fuse.",
  "25.4 × 20.3 × ~8", "2 × M2×6 + washers into inserts on 8.5 mm front-panel bosses (drill Ø2.2 in the 2 bottom corners, 19 mm apart)", "Front panel inside, left of the sonar (Y 30–55, Z 44–64)", CAD, 1, 13, 4, "Yes", "Opencircuit (NL) €9.85 + MOSFET ~€1.5 + resistors."),
 ("Companion computer & sensors", "Headlight fuse", "1 A blade (mini) fuse + inline holder, 22 AWG", "Protects the thin 22 AWG feed from the PDB.", "~25 × 8", "Inline in the PDB -> driver lead, heat-shrink", "Centre gap", "Not modelled", 1, 2, 2, "Yes", "est."),
 ("Mechanical", "Front bumpers", "4 × clear rubber bumper Ø11 × 2.5 mm, self-adhesive (3M Bumpon SJ5302 type)",
  "The robot lands on its face when it falls and when it stands up from face-down: it rests on these, not on the camera window, eyes or headlights.", "Ø11 × 2.5", "Stuck on the front face at Y ±66, Z 40 and Z 127", "Front face corners", CAD, 1, 3, 1, "Yes", "est."),
 ("Companion computer & sensors", "LED eye driver parts", "74AHCT125 (DIP-14, no socket: 8.5 mm under the DevKit) + 330 Ω + 2 × 470 µF 10 V (eye plug on the perfboard, amp 5 V input)",
  "3.3 V → 5 V data for WS2812.", "DIP-14; cap Ø8 × 12", "Soldered on the perfboard", "Perfboard", "OK: inside perfboard envelope", 1, 2, 2, "Yes", "est."),
 ("Companion computer & sensors", "Eye diffusers", "2 × printed diffuser (robot_eye_diffuser.stl), white PETG, 0.8 mm",
  "Turns 64 dots into a soft glowing eye.", "26.1 × 26.1 × 0.8", "Drops into the eye pocket in front of the LEDs", "Front panel eye pockets", PR, 1, 0.5, 1, "Yes", "Filament only (2nd colour)."),
 ("Companion computer & sensors", "IR beacon receivers", "4 × Vishay TSOP38238 (38 kHz IR receiver module) in 4 printed hoods (robot_ir_hood.stl, PETG, black), 6-wire cable to perfboard J8",
  "Hear the dock's IR beacon from metres away (typically 3-5 m, up to ~10 m in line of sight): coarse bearing from which receivers hear it, 'C' code = on the dock's axis. Outputs open-collector to GPIO13-16 (3.3 V, internal pull-ups). 0.35 mA each.",
  "TSOP: 5.6 × 6.0 × 7.0; hood 11 × 10 × 9", "Receiver stands in its hood, leads through the floor hole and a Ø4.5 hole in the body top skin; hood glued on (CA/epoxy), window facing out at 45°/135°", "Top skin, 4 corners round the lidar (front-left/right, back-left/right)", "real_assembly: hoods + receivers checked", 4, 1.6, 1, "Yes", "est. (Mouser / Reichelt TSOP38238)"),
 ("Companion computer & sensors", "2D lidar", "DFRobot DTOF LD19 Laser Lidar Kit (12 m, 360°)",
  "Kit includes adapter board + USB cable → Pi USB OTG.", "54 × 46.3 × 31.3 with its 3 mounting ears (Waveshare STEP); adapter board ~18 × 12 × 6",
  "Lidar: 3 × M2×6 into M2 inserts on top (two holes 46.8 apart + one 31.9 away). Adapter board: zip-tied to the shelf carrier plate",
  "Top, centred (Z 134–169); adapter on the shelf", "OK: no clash; scan plane clears the body top", 1, 130.9, 47, "Yes", "RobotShop EU €130.90."),
 ("Companion computer & sensors", "USB OTG adapter", "90° micro-USB OTG to USB-A cable, ~10 cm (a straight adapter hits the wheel arch)", "For the lidar's USB cable.", "~30 × 15 × 8", "Plugged in the Pi", "Back cover", "Not modelled: small, free space next to the Pi", 1, 3, 3, "Yes", "est."),
 ("Companion computer & sensors", "Speaker", "Adafruit 3968 speaker, 40 mm, 4 Ω, 3 W (current version rated 5 W)",
  "Voice replies. Fires sideways through a grille in the right wall, above the wheel arch (the top is the lidar's).",
  "Ø40 × 20 (flange 6, magnet Ø~26)", "Pressed against the wall by the printed speaker ring, 3 × M2×8 into wall inserts; 1 mm foam gasket", "Right wall, centre X −2 / Z 86", CAD, 1, 5.2, 27, "Yes", "DigiKey NL €5.15."),
 ("Companion computer & sensors", "Speaker amplifier", "Adafruit MAX98357A I2S class-D mono amp (#3006)",
  "3.2 W into 4 Ω from 5 V (10% THD). I2S from the Pi, no DAC needed. GAIN pin: open = 9 dB, 100 k to GND = 12 dB.",
  "19.4 × 17.8 × 3 (+ terminal block)", "Zip-tied to the tab of the speaker ring", "Right wall, above the speaker (X −11…8, Z 105–123)", CAD, 1, 7.8, 2, "Yes", "Electrokit 89 SEK (~€7.80); Mouser DE €5.12."),
 ("Companion computer & sensors", "Microphones", "2 × Adafruit ICS-43434 I2S MEMS microphone breakout (#6049)",
  "65 dBA SNR, 24-bit, bottom port. 124 mm apart: the Pi steers a beam to the talker (+3 dB) and knows its direction. Speech from 2 m (~54 dB SPL) arrives ~25 dB above the mic noise.",
  "~13 × 17 board (CHECK, pocket is 14 × 18)", "Board in a pocket behind a Ø1.5 port, foam gasket, printed clamp with 1 × M2×6 into an insert", "Front panel, Y ±62, Z 100", CAD, 2, 6.9, 2, "Yes", "Electrokit 79 SEK each (~€6.90)."),
 ("Companion computer & sensors", "Gamepad", "Bluetooth gamepad (e.g. 8BitDo Micro)", "Optional remote.", "—", "—", OFF, OFF, 1, 25, 0, "Optional", "est."),
 ("Power", None),
 ("Power", "Battery", "Tattu R-Line V4.0 (or V5.0) 6S 1400 mAh, XT60",
  "6S for the 24 V motors. ~31 Wh: +33 % runtime over the 1050 (body raised 8 mm to make room). Low-voltage cutoff in firmware (~3.5 V/cell under load).",
  "78 × 39 × 42", "Velcro strap through the shelf's strap slots + anti-slip pad", "On the slide-in shelf above the motors (X −28…13, Z 42–84)", CAD, 1, 41.9, 222, "Yes", "FPV24 (DE) €41.90 (V4; V5 same size)."),
 ("Power", "6S BMS with balancing", "6S LiPo/Li-ion BMS 25.2 V with balancing (e.g. Sigmanortec 6S 40 A) + JST-XH 7-pin lead",
  "Per-cell protection and balancing for dock/USB charging. 40 A so motor peaks don't trip it. Check its balance threshold suits LiPo (4.2 V).",
  "88 × 34 × 3.5", "Against the battery's back face, Kapton tape, under the strap", "Behind the battery (X −31.5…−28, Z 42–75.5)", CAD, 1, 12, 20, "Yes", "est. Sigmanortec (RO, EU)."),
 ("Power", "Main fuse", "Inline mini blade-fuse holder 30 A + fuse", "Protects wiring against a stalled motor or short.",
  "PDB+fuse block ~16 × 13 × 9", "Under the printed clamp bar (2 × M3×10 into tray inserts)", "Floor tray (centre gap)", CAD, 1, 6, 5, "Yes", "Galaxus CH."),
 ("Power", "Power switch", "Pololu Big Pushbutton Power Switch with Reverse Voltage Protection, HP (#2813): 4.5–40 V, 16 A continuous, 90 A peak",
  "Push on, push off. MOSFETs switch BAT+ (battery current never goes through a button contact), reverse-polarity protected. The panel button is wired in parallel with its own button; the ESP32 can switch it OFF (GPIO21, J9) when the battery is flat: the Pi halts first. Charging (dock / USB-C) feeds the BMS before the switch, so it charges when off. Not a full isolation: for storage/transport unplug the battery's XT60.",
  "25.5 × 20 × 4 + wire tails", "VHB foam tape under the top skin, front-right; 16 AWG VIN/VOUT soldered, splice of the negatives beside it", "Under the top, front-right (X 1–27, Y −67…−47)", CAD, 1, 11.55, 3, "Yes", "Opencircuit (NL) €11.55"),
 ("Power", "Power button", "12 mm momentary push button, normally open (panel mount, nut), e.g. 12 mm metal or plastic NO button",
  "On/off. Mirrors the arm button on the other side of the top (Y +40 / −40). 2 × 26 AWG to the switch board's button pads.",
  "Ø12 hole, ~15 deep", "Nut under the top skin", "Top, front-right (X 10, Y −40)", CAD, 1, 2, 3, "Yes", "est."),
 ("Power", "5 V regulator", "Pololu D24V50F5 (5 V 5 A, 6–38 V in)", "Pi + camera + lidar + eyes + ESP32.",
  "20.3 × 17.8 × 8.8", "2 × M2×6 into M2 inserts on the tray (2 mounting holes)", "Floor tray, front (X 6–27, Z −19…−10)", CAD, 1, 29.5, 3, "Yes", "EXP-Tech."),
 ("Power", "Power distribution", "Small PDB or XT30 1→3 parallel harness", "Feeds both motors and the regulator from one lead.",
  "~16 × 13 × 9", "Under the clamp bar", "Floor tray, back", CAD, 1, 8, 10, "Yes", "est."),
 ("Power", "Sense dividers + J7", "100 kΩ + 12 kΩ (VBAT), 56 kΩ + 10 kΩ (USB VBUS), 1 kΩ + 2 kΩ (sonar echo), all 1 %; 3-pin 2.54 header + Dupont (J7); 3 × 26 AWG sense wires",
  "VBAT 25.2 V → 2.70 V (GPIO1), USB 20 V → 3.03 V (GPIO7), echo 5 V → 3.33 V (GPIO11). The dock divider is in the charge row. Sense wires: power switch VOUT, fuse+diode IN, IP2368 VBUS -> J7.",
  "—", "Perfboard (see the to-scale layout on the wiring page)", "Perfboard", "Checked: electrical rules check", 1, 2, 2, "Yes", "est."),
 ("Power", "Perfboard 5 V plug (J1)", "XT30PW-M right-angle PCB plug + XT30 female lead",
  "Carries the whole 5 V rail (up to ~4.7 A worst case): a 2.54 mm header is only ~3 A per contact.", "10.2 × 5.2 × 8", "Soldered on the perfboard (pins 5.08 apart = 2 holes)", "Perfboard J1", "Checked: 3D fit + current check", 1, 2, 2, "Yes", "est."),
 ("Power", "Regulator feed fuse", "Inline mini blade-fuse holder + 3 A fuse",
  "The 20 AWG regulator feed would otherwise only be protected by the 30 A main fuse.", "~30 × 12 × 9", "Inline in the PDB -> regulator VIN lead, heat-shrunk", "Floor tray", "Checked: electrical rules check", 1, 3, 3, "Yes", "est."),
 ("Power", "Bulk capacitor", "470 µF 35 V low-ESR electrolytic", "At the distribution point.", "Ø10 × 16", "Soldered to the PDB, zip tie", "Floor tray", "Not modelled: fits beside PDB", 1, 1, 2, "Yes", "est."),
 ("Power", "6S rail TVS diode", "5KP24A (Littelfuse/Vishay, axial, 5 kW, 24 V stand-off, breakdown 26.7–29.5 V)",
  "Catches braking energy if the BMS ever blocks charge current: holds the rail ≤ ~30 V (motor drives max 32 V) until the firmware switches the motors off (> 26.8 V). Sim: power_sim.py.",
  "Ø8 × 9 body", "Soldered across the PDB output pads, band (cathode) to +, heat-shrink", "Floor tray, PDB", "Not modelled: fits beside the PDB", 1, 1.5, 2, "Yes", "est. Mouser / DigiKey / Reichelt ~€1–2."),
 ("Power", "Power pigtails", "XT60 female pigtail 12 AWG ~10 cm (battery -> BMS B+/B-) + XT30 female pigtail 16-20 AWG ~10 cm (regulator -> J1)",
  "The battery's XT60 plugs into the first; the second mates with the XT30 PCB plug J1.", "—", "Soldered to BMS pads / regulator VOUT-GND, heat-shrink", "—", "—", 1, 5, 8, "Yes", "est."),
 ("Power", "Back-cover plugs", "2 × XT60 pairs (male + female) + JST-XH 2.5 mm pairs: one 3-pin (power button), one 2-pin (CAN to the motors), with crimp contacts",
  "The back cover (perfboard + ESP32 + CAN board, Pi, power switch) comes off as one module. Battery side: the BMS lead ends in a FEMALE XT60, the switch's input pigtail is male; the switch's output pigtail is FEMALE, the PDB lead male — so the two can't be swapped and a live end is never a bare male. Every wire to the cover has 4 cm of extra length (service loop): lay the cover down beside the robot.", "XT60 16 × 8 × 24 mm", "On the wires (heat-shrink), tucked beside the battery", "Above the battery, behind the perfboard", "Not modelled (on the wire runs)", 1, 6, 12, "Yes", "est. AliExpress / Conrad ~€5–8."),
 ("Power", "Silicone wire", "16 AWG red + black 1 m each; 20 AWG red, black, orange 0.5 m each; 22 AWG black + red/pink 0.5 m each; 24 AWG red, black, orange 0.5 m each; 26 AWG 10-colour set (≥ 1 m of black, red, green, blue, yellow; 0.5 m of white, orange, grey, purple, pink)",
  "Totals from the routed harness + 30 %: 16 AWG 1.2 m, 20 AWG 0.6 m, 22 AWG 0.35 m, 24 AWG 0.75 m, 26 AWG 5.8 m. CAN: twist a yellow + green 26 AWG pair. The lidar's 28 AWG lead and the battery's 12 AWG leads come with them.",
  "—", "—", "—", "Lengths from the wiring model", 1, 18, 30, "Yes", "est."),
 ("Power", "Dupont parts", "2.54 mm Dupont housings: 12 × 1-pin, 2 × 3-pin, 5 × 4-pin, 2 × 5-pin; ~60 female crimp contacts; 1 × 40-pin male header strip; crimp tool (or pre-crimped female jumper wires cut and re-housed)",
  "1-pin: the 12 wires on the Pi header. 4-pin: J2, J3, HC-SR04, GY-521, CAN board. 5-pin: J5, J6. 3-pin: J4, J7. Male header: J2-J7 on the perfboard (24 pins).",
  "—", "Crimped", "—", "Counts from the wiring model", 1, 12, 5, "Yes", "est."),
 ("Power", "I2S junction board", "Stripboard (Veroboard) piece 20 × 15 mm, 7 strips",
  "Splits BCLK / WS / DOUT / DIN / 3V3 / 5V / GND from the Pi to both mics and the amp (one strip per signal, wires soldered).", "20 × 15 × 1.6", "Kapton-taped on the battery top", "Over the battery (Y 27, Z 78)", "OK: 3D fit check", 1, 1, 2, "Yes", "est."),
 ("Charging (contact dock + USB-C)", None),
 ("Charging (contact dock + USB-C)", "Dock charger (25.2 V)", "Fuyuang EP-FY2551000 6S Li-ion charger, 25.2 V 1 A CC/CV, 5.5/2.1 mm plug",
  "Real CC/CV charger with short-circuit protection: 1 A → full in ~1.6 h (1400 mAh). Plugs into the dock. Minus the Schottky drop the pack sees ~25.0 V (slightly under full = gentler on the cells).",
  "Wall-wart, ~100 cm cable", "—", OFF + " (wall → dock)", OFF, 1, 15, 0, "Yes", "akkuteile.de, 1 A version (item 400623-1A); price est."),
 ("Charging (contact dock + USB-C)", "Dock contact rods", "2 × brass rod Ø4 × 25 mm + 2 × brass washer Ø5.5 (soldered on as a collar) + 2 × soft compression spring Ø5 × 10 mm (~0.15 N/mm) + 2 × M3 washer",
  "Sprung rods, 10.5 mm out of the dock face, ~4 mm travel: they ride down the robot's funnels onto the M5 screw heads (docked: ~1.5 mm compressed, ~0.4 N each). Brass carries the 1 A charge easily. Soft springs on purpose: the robot's own weight is what presses the plate on (no magnets), stiff springs would push it off.",
  "Ø4 × 25", "Rod through the Ø4.3 bore, collar 16.5 mm from the tip stops it at the front; spring behind the collar; an M3 washer epoxied over the back of the Ø6.6 pocket holds the spring, the wire (soldered to the rod end) goes through it", "Dock face (Y ±25, Z 111 in the docked robot frame)", "Sim: sim/dock_sim.py (rod springs + funnels)", 1, 6, 1, "Yes", "est. (1 m brass rod ~€4 + spring assortment)"),
 ("Charging (contact dock + USB-C)", "Dock DC jack + wire", "5.5/2.1 mm panel DC jack + 4 × 45 cm 22 AWG (red/black)", "Charger → jack → up the column to the controller board (fuse, polarity guard, rod switch) → back down to the rods. Never wire the jack straight to the rods.",
  "Ø8 panel hole", "Nut through the base plate's back edge", "Dock", "OK", 1, 3, 0, "Yes", "est."),
 ("Charging (contact dock + USB-C)", "Dock", "Printed vertical dock: foot (robot_dock_foot.stl: floor plate + lower column) + head (robot_dock_head.stl: shelf + face slab + upper column; ring + glue + 2 × M3×16 onto the foot, 2 more M3 inserts) + mast (robot_dock_mast.stl, plugs on top) + 2 side wings (robot_dock_wing.stl) + 2 wheel-track frames (robot_dock_ridge.stl) in PETG; 2 saddle inserts (robot_dock_ridge_insert.stl) in TPU 95A. Print one of each pair mirrored",
  "The robot finds it with the lidar (the 80 mm column, 310 mm tall), backs in, leans back 8° onto the face slab and charges. Face slab with the 2 sprung rods and a centring tongue; column stepped back low down (room for the robot to roll under itself when it leaves); floor plate behind the wheels; two wheel tracks (PETG frames, T ends into slots in the plate) each carry a ribbed TPU saddle the docked wheel sits in: rear lip 1.2 mm, trough, front lip 1.8 mm, then an 80 mm lead-in (≤2°). Leaning on the dock pushes the robot forward ~0.6 N; the lips hold several N, so it stays put with its motors off. The TPU ribs give the tyres grip to climb out. Wings: keep a bad approach off the rods.",
  "Assembled 181 × 232 × 310 + ridges reaching 76 mm in front of the axle (foot 121 cm³ + head 165, mast 79, ridges 2 × 11; prints on a 220 mm printer); wings 53 × 35 × 62", "Free-standing (4 rubber feet under the base); wings: 2 × M3×12 button each into M3 inserts in the slab sides", OFF + " (floor, against a wall)", PR, 1, 6, 0, "Yes", NOLINK_PRINTED),
 ("Charging (contact dock + USB-C)", "Dock controller board", "One ATtiny85 runs the IR beacon and a safe rod switch: ATtiny85-20PU + DIP-8 socket; Pololu D24V5F5 (5 V buck); 2 × FQP27P06 P-MOSFET (reverse-polarity + rod switch); 2 × IRLB8721 (LED drivers); 2N7000; Bourns MF-R110 PTC fuse; SMBJ28A TVS; 2 × 12 V zener; resistors 10k, 2 × 22k, 3 × 100 Ω, 3 × 100k, 100k/220k/47k 1 %, 2 × 22 Ω 0.5 W, 33 Ω 0.5 W, 1k, 10k; 100 µF + 2 × 100 nF; green 3 mm LED; 40 × 30 perfboard",
  "Rods dead unless the robot is on them: a ≤0.25 mA sense current finds the robot's 112 kΩ detect divider, then the high-side switch puts 25.2 V on the rods; a coin, key, hand or short keeps them off, a short while live switches off in ~10 ms (3 faults: latched off). Re-checks every 2 s. Beacon: 38 kHz, 'A' all round / 'C' centre-line codes. Firmware dock_beacon.ino, logic unit-tested. Schematic + every connection: site page 'Wiring diagram > Dock'.",
  "Board 40 × 30, parts ≤ 9 mm (TO-220s bent flat)", "Stands in the mast's hollow under the LEDs; wires down the column to the DC jack and the rods", OFF + " (dock mast)", "Sim: rod switch modelled in sim/dock_sim.py; logic test test_dock_logic.cpp", 1, 18, 0, "Yes", "est. (Reichelt / LCSC parts + Pololu D24V5F5 ~€6)"),
 ("Charging (contact dock + USB-C)", "Dock IR beacon LEDs", "5 × Vishay TSAL6200 (940 nm, Ø5) + 2 m 26 AWG pairs",
  "4 all-round (two series pairs, 22 Ω each, ~100 mA bursts) in the mast's side holes; 1 in the 22 mm centre tube (±6° beam on the dock axis). ≈10 mA average: invisible, harmless.",
  "Ø5", "Pushed into the Ø5.2 holes at Z 252 (a drop of glue), leads to the controller board", OFF + " (dock mast)", "Sim: sim/dock_sim.py (IR model, homing sweep)", 5, 0.5, 0, "Yes", "est."),
 ("Charging (contact dock + USB-C)", "Retro-reflective tape", "Retro-reflective tape, white or silver (3M Scotchlite 8906/983 type), 25 mm wide, 2 × 141 mm",
  "Lidar target: the LD19 reads it at intensity >200 (paint and walls ~60-160), so the pilot finds the dock's face by its two strips 49 mm apart, even among chair legs.",
  "2 × 25 × 141", "Stuck into the 0.3 mm recessed guides on the column face", OFF + " (dock face)", "Sim: tape intensity model", 1, 6, 0, "Yes", "est. (any reflective tape roll)"),
 ("Charging (contact dock + USB-C)", "Robot charge contacts", "2 × M5×12 brass countersunk screws (DIN 963) + 2 × M5 brass nuts + 2 × M5 ring lugs",
  "Ø9.3 heads at the bottom of 8 mm deep funnels, so a flat metal object or finger can't bridge them; the diode keeps them dead when not docked. Ø9.3 (not M4's Ø8): the rods still land on them ±4.5 mm off centre.",
  "Head Ø9.3, 8 mm deep", "Through Ø5.4 holes in contact plate + cover; ring lug + nut inside (head 8 mm into the 10 mm plate: ~2 mm of thread spare past lug + nut)", "Back, on the contact plate (Y −25 = +, Y +25 = −, Z 111)", CAD, 1, 3.5, 5, "Yes", "est."),
 ("Charging (contact dock + USB-C)", "Contact plate", "Printed contact plate (robot_contact_plate.stl), PETG",
  "10 mm thick: two 45° lead-in funnels (22 mm wide, the mouth stretched 8 mm downwards because the plate drops ~6 mm onto the rods as the robot leans back) down to the M5 contact heads, a vertical V slot for the dock's tongue. No magnets.",
  "104 × 42 × 10", "4 × M3×16 countersunk (ISO 10642) into inserts in cover bosses", "Back cover, outside, Z 90–132", PR, 1, 0.8, 19, "Yes", NOLINK_PRINTED),
 ("Charging (contact dock + USB-C)", "Charge isolation + detect", "Schottky SB560 (5 A 60 V) + 5 A blade fuse + holder + 100 kΩ/12 kΩ divider",
  "Diode: battery can't back-feed the contacts (dead when shorted). Divider from contact + (before diode) → ESP32: charging → motors disabled until undocked.",
  "~18 × 35 × 7 together", "Diode heat-shrunk inline in the charge lead, inline fuse holder, zip-tied to the harness", "Harness beside the battery (right)", CAD, 1, 4, 5, "Yes", "est."),
 ("Charging (contact dock + USB-C)", "USB-C PD charger board", "IP2368 2–6S 100 W bidirectional USB-C PD module, set to 6S Li-ion, charge current ~1 A",
  "Charges from any USB-C PD charger (needs a 20 V PD source). Set 6S by the board's resistors and CHECK the end voltage is 25.2 V before connecting (users report boards stopping at 5S voltage); turn off its discharge/output mode.",
  "35 × 47 × 10 (est.)", "Zip-tied to the left carrier plate (6 mm grid)", "Left wall plate (X −15…20, Y 67–77, Z 70–117)", CAD, 1, 12, 20, "Yes", "est. ~€8–15; check the exact board size before printing."),
 ("Charging (contact dock + USB-C)", "USB-C panel socket", "SparkFun CAB-15455 panel-mount USB-C extension, 15 cm",
  "Moves the IP2368's port to the left side wall, reachable standing or lying down; C female ↔ C male.", "~22 × 14 × 8 head", "2 × M3 (supplied) + 2 × M3 nuts, 18 mm apart, 10.5 × 4.5 cut-out", "Left side wall above the wheel (X 8, Z 66)", CAD, 1, 13.56, 6, "Yes", "Melopero (IT) €13.56."),
 ("Charging (contact dock + USB-C)", "LiPo charger", "ToolkitRC M6AC or similar 6S balance charger",
  "For storage charge and checks; the dock and USB-C are for daily charging.", "—", "—", OFF, OFF, 1, 45, 0, "Optional", "est."),
 ("Mechanical", None),
 ("Mechanical", "Frame / body", "Printed body (robot_body.stl), PETG",
  "One file, two pieces (shell + motor block: the wheel wells cut the floor; panels and the screwed shelf join them), printed standing on the front opening, no supports. Arches, double centre wall with teardrop motor pockets (3 mm air around each motor), open cable slots over the motor plugs, root fillets + boss ribs, 25 insert bosses, lidar mount, top holes, speaker grille + 3 ring bosses in the right wall.",
  "67 × 184 × 169", "—", "Main shell", PR, 1, 5, 138, "Yes", "Filament ~400 g for all parts. Bed ≥ 180 × 170 mm."),
 ("Mechanical", "Battery shelf", "Printed slide-in shelf (robot_shelf.stl), PETG, 5 mm",
  "Slides in flat on the motor-wall tops, then 4 countersunk M3 screws tie the two walls together (FEA: the walls splay without it).", "63 × 113 × 9", "4 × M3×10 countersunk into wall-top inserts", "Above the motors, Z 37–42", PR, 1, 1, 28, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Front panel", "Printed front panel (robot_front.stl), PETG",
  "The face: eyes, camera, sonar, USB-C, 2 microphone ports. Outside face down.", "12 × 178 × 166", "6 × M3×10 countersunk (ISO 10642) into body inserts", "Front", PR, 1, 2, 64, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Back cover", "Printed back cover (robot_cover.stl), PETG",
  "Carries perfboard, Pi and the contact plate.", "12 × 178 × 169", "6 × M3×10 countersunk (ISO 10642) into body inserts", "Back", PR, 1, 2, 71, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Floor tray", "Printed tray (robot_tray.stl) + clamp bar (robot_clamp_bar.stl), PETG",
  "Holds regulator, PDB+fuse and IMU in the centre gap at the axle.", "Tray 56 × 32 × 13; bar 30 × 8 × 3", "Tray: 2 × M3×8 into floor inserts; bar: 2 × M3×10 into tray inserts", "Centre gap floor (Z −32)", PR, 1, 0.5, 7, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Carrier plates", "2 × printed carrier plate (robot_carrier_plate.stl), PETG",
  "6 mm grid for zip ties: IP2368 (left wall), lidar adapter (shelf).", "32 × 60 × 3 each", "2 × M3×6 each into wall / shelf inserts (22 mm pitch)", "Left wall above the arch; on the shelf", PR, 1, 0.5, 9, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Speaker ring + mic clamps", "Printed speaker ring with amp tab (robot_speaker_ring.stl) + 2 mic clamps (robot_mic_clamp.stl, print one mirrored), PETG",
  "Ring clamps the speaker flange to the right wall and carries the amp; clamps hold the mic boards in their pockets.", "Ring 43 × 60 × 2.5; clamps 16 × 26 × 7", "Ring 3 × M2×8, clamps 1 × M2×6 each", "Right wall inside; behind the mic ports", PR, 1, 0.5, 5, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Eye retainers", "2 × printed eye retainer (robot_eye_retainer.stl), PETG",
  "Clamps each LED panel + diffuser into its pocket.", "39 × 38 × 6 each", "2 × M2×6 each into front-panel inserts", "Inside the front panel, behind each eye", PR, 1, 0.5, 7, "Yes", NOLINK_PRINTED),
 ("Mechanical", "Heat-set inserts (M3)", "ruthex RX-M3x5.7, pack of 100",
  "36 used: front 6, back 6, floor 2, left wall plate 2, wall tops 4 (shelf screws), shelf 2 (body/shelf) + perfboard 4, contact plate 4 (cover) + clamp bar 2 (tray) + dock 4 (wing screws). Hole Ø4.0 × 6.7.",
  "Ø4.6 × 5.7", "Pressed with iron + tip", "—", "Bosses Ø8 in CAD", 1, 9.5, 15, "Yes", "3DJake CH CHF 8.90."),
 ("Mechanical", "Heat-set inserts (M2)", "ruthex RX-M2x4, pack of 70",
  "34 used: camera 4, sonar 4, eye retainers 4, mic clamps 2, headlight retainers 4, headlight driver 2 (front panel), lidar 3 (body top), speaker ring 3 (right wall), regulator 2 + IMU 2 (tray), Pi 4 (cover). Hole Ø3.2 × 5. Only two insert sizes in the build: M3 and M2.",
  "Ø3.6 × 4", "Pressed", "—", "Bosses Ø6.4 in CAD", 1, 9.3, 4, "Yes", "3DJake CH CHF 8.70."),
 ("Mechanical", "Case screws", "A2 stainless: 16 × M3×10 + 4 × M3×16 ISO 10642 countersunk hex socket, 2 × M3×8 ISO 7380, 8 × M3×6 ISO 7380, 2 × M3×10 + 4 × M3×12 ISO 7380, 2 × M3 nuts",
  "Countersunk (flush, nothing sticks out when the robot falls): M3×10 front panel 6, back cover 6, battery shelf -> wall tops 4; M3×16 contact plate 4. M3×12 button: dock wings 4. M3×8 button: tray 2. M3×6: perfboard 4, carrier plates 4. M3×10 button: clamp bar 2. Nuts: USB-C socket.",
  "—", "—", "—", "—", 1, 6, 15, "Yes", "est."),
 ("Mechanical", "Small-board screws", "A2: 31 × M2×6, 3 × M2×8 (pan or socket head), 8 × M2 washers",
  "M2×6: camera, sonar, eye retainers, mic clamps, headlight retainers, headlight driver (washers), lidar, regulator, Pi + IMU (with a washer each: their holes are 2.75 / ~3 mm). M2×8: speaker ring. (The only M2.5 screws are the motor mounting screws, into the motors' own threads.)", "—", "—", "—", "—", 1, 5, 4, "Yes", "est."),
 ("Mechanical", "Damping", "Thin foam/rubber pad on the body's top and bottom ends", "Softens flips; ≤3 mm.", "≤3 thick", "Adhesive", "Top/bottom edges", "Not modelled", 1, 3, 4, "Optional", "est."),
 ("Assembly & integration", None),
 ("Assembly & integration", "Motor cable set", "LKMTECH power + CAN/UART cable set for MF6015-V2 (×2)",
  "Check whether the motors ship with mating leads.", "—", "Through the wall slots into the centre gap", "Centre gap", "Slots in CAD", 2, 5, 10, "Yes", "est."),
 ("Assembly & integration", "Motor mounting screws", "8 × M2.5×10 socket screws (ISO 4762), A2",
  "Through the 7 mm centre wall → 5 mm engagement. Never into the rotating face.", "—", "—", "Centre wall", "Hole depth checked in CAD", 1, 3, 4, "Yes", "est."),
 ("Assembly & integration", "Motor config adapter", "LKMTECH Neo2 USB-to-UART adapter (CP2102)", "Set CAN baud, ID, direction once.", "—", "—", OFF, OFF, 1, 9.27, 0, "Yes", "OpenELAB (EU) €9.27."),
 ("Assembly & integration", "USB-CAN adapter", "CANable 2.0 or similar", "Sniff/test the CAN bus from your PC.", "—", "—", OFF, OFF, 1, 25, 0, "Optional", "est."),
 ("Assembly & integration", "Battery mounting", "Velcro strap 20 × 250 mm + anti-slip pad", "Pack must not shift during flips.", "20 wide", "Through the shelf strap slots", "Shelf", "Slots in CAD", 1, 4, 8, "Yes", "est."),
 ("Assembly & integration", "Threadlocker", "Medium (blue), e.g. Loctite 243", "Metal threads only.", "—", "—", "—", "—", 1, 7, 0, "Yes", "est."),
 ("Assembly & integration", "Prototype board", "Perfboard 5 × 7 cm double-sided FR4, 24 × 18 holes (432), 2.54 mm, plated through + 2 × 1×22 female headers (2.54, 8.5 mm tall)",
  "Carrier for ESP32, CAN board, dividers, eye driver, connectors; layout to scale on the wiring page. Corner holes must sit 3 mm from both edges (64 × 44 mm) for M3: if yours differ, drill Ø3.2 there (it clears the outer pads).", "70 × 50 × ~20 with boards", "4 × M3×6 into inserts on 9 mm posts (64 × 44 mm pattern)", "Back cover, right half (Z 77–127)", CAD, 1, 8.8, 15, "Yes", "BerryBase set of 32 double-sided boards (7 × 5×7 cm with 432 = 24 × 18 holes, plus 2×8, 4×6, 7×9) €8.80. Check the listing says 24 × 18 / 432: some 5×7 cm boards are 26 × 18 or 27 × 18 and the layout won't fit."),
 ("Assembly & integration", "Resistor kit", "1% metal-film assortment", "Dividers, LED resistors, spare 120 Ω.", "—", "—", "—", "—", 1, 6, 0, "Yes", "est."),
 ("Assembly & integration", "Consumables", "Heat-shrink, solder, zip ties, Kapton tape, braided sleeve", "", "—", "—", "—", "—", 1, 12, 10, "Yes", "est."),
 ("Assembly & integration", "USB cables", "USB-C data cable (ESP32) + micro-USB cable (Pi)", "Bench flashing.", "—", "—", OFF, OFF, 1, 6, 0, "Yes", "est."),
 ("Assembly & integration", "LiPo safety bag", "Fire-retardant LiPo bag", "Store the pack in it.", "—", "—", OFF, OFF, 1, 8, 0, "Optional", "est."),
 ("Tools (only if you don't own them)", None),
 ("Tools (only if you don't own them)", "Soldering iron + insert tips", "Temperature-controlled iron + ruthex insert tip set", "Electronics + inserts.", "—", "—", "—", "—", 1, 17, 0, "Tool", "3DJake CH CHF 16.80 (tips)."),
 ("Tools (only if you don't own them)", "Multimeter", "Any basic digital multimeter", "Divider calibration, 5 V rail, charger end voltage, CAN termination (~60 Ω).", "—", "—", "—", "—", 1, 25, 0, "Tool", "est."),
 ("Tools (only if you don't own them)", "Hex keys & small screwdrivers", "1.5–3 mm hex keys, PH0/PH1", "", "—", "—", "—", "—", 1, 10, 0, "Tool", "est."),
]

r = 5; n = 0; first_item = None
for row in rows:
    if row[1] is None:
        ws.cell(row=r, column=1, value=row[0]).font = Font(name=F, bold=True)
        for c in range(1, NCOL + 1): ws.cell(row=r, column=c).fill = SEC
        r += 1; continue
    n += 1; first_item = first_item or r
    cat, comp, part, spec, size, mount, loc, fit, qty, price, mass, req, src = row
    vals = [n, cat, comp, part, spec, size, mount, loc, fit, qty, price,
            f"={C['qty']}{r}*{C['price']}{r}", mass, f"={C['qty']}{r}*{C['mass']}{r}", req, src]
    for c, v in enumerate(vals, 1):
        cell = ws.cell(row=r, column=c, value=v); cell.alignment = wrap; cell.border = BOX
        cell.font = BLUE if c in (10, 11, 13) else BLK
        if 6 <= c <= 9: cell.fill = GEO
    ws.cell(row=r, column=11).number_format = '€#,##0.00'; ws.cell(row=r, column=12).number_format = '€#,##0.00'
    ws.cell(row=r, column=13).number_format = '#,##0.0'; ws.cell(row=r, column=14).number_format = '#,##0'
    lc = ws.cell(row=r, column=17); lc.border = BOX; lc.alignment = wrap
    if comp in LINKS:
        label, url = LINKS[comp]; lc.value = label; lc.hyperlink = url; lc.font = LINKF
    else:
        lc.value = NOLINK.get(comp, NOLINK_PRINTED if fit == PR else "Generic part: any electronics shop (Reichelt, Conrad, Digitec)")
        lc.font = SMALL_I
    for c in range(18, NCOL + 1):
        ws.cell(row=r, column=c).border = BOX; ws.cell(row=r, column=c).alignment = wrap
    if comp in ASIA:
        usd, label, url, note = ASIA[comp]
        ws.cell(row=r, column=18, value=usd).font = BLUE
        ws.cell(row=r, column=18).number_format = '$#,##0.00'
        ws.cell(row=r, column=19, value=f"={C['ausd']}{r}*$S$2").number_format = '€#,##0.00'
        ws.cell(row=r, column=20, value=f"={C['qty']}{r}*{C['aeur']}{r}").number_format = '€#,##0.00'
        al = ws.cell(row=r, column=21, value=label); al.hyperlink = url; al.font = LINKF
        ws.cell(row=r, column=22, value=note).font = Font(name=F, size=9)
    else:
        ws.cell(row=r, column=22, value="No cheaper Asian source found / not worth shipping.").font = SMALL_I
    L, A = C['line'], C['aline']
    ws.cell(row=r, column=23, value=f'=IF({A}{r}="",{L}{r},MIN({L}{r},{A}{r}))').number_format = '€#,##0.00'
    r += 1
last = r - 1
r += 1
rng = lambda col: f"{col}{first_item}:{col}{last}"
tot = [("Total — required parts", f'=SUMIFS({rng(C["line"])},{rng(C["req"])},"Yes")', f'=SUMIFS({rng(C["lmass"])},{rng(C["req"])},"Yes")',
        f'=SUMIFS({rng(C["cheap"])},{rng(C["req"])},"Yes")'),
       ("Total — including optional parts and tools", f'=SUM({rng(C["line"])})', f'=SUM({rng(C["lmass"])})', f'=SUM({rng(C["cheap"])})')]
for label, cost, mass, cheap in tot:
    ws.cell(row=r, column=5, value=label).font = Font(name=F, bold=True)
    c = ws.cell(row=r, column=12, value=cost); c.font = Font(name=F, bold=True); c.number_format = '€#,##0.00'
    c = ws.cell(row=r, column=14, value=mass); c.font = Font(name=F, bold=True); c.number_format = '#,##0" g"'
    c = ws.cell(row=r, column=23, value=cheap); c.font = Font(name=F, bold=True); c.number_format = '€#,##0.00'
    r += 1
ws.cell(row=r + 1, column=1, value=(
    "Fit and mass (CAD assembly, Nov 2026): every part marked 'OK' is modelled at its listed size and position, with no collisions found. "
    "Robot ≈ 1.61 kg; centre of mass ≈ 34 mm above the axle (battery above the motors). Lying flat it rests at ~100° face-down and ~97° back-down "
    "(on the contact plate). Items marked '(est.)' in the size column are typical board sizes: measure yours before printing. "
    "Re-simulate the stand-up move with these numbers before ordering motors.")).font = SMALL_I
ws.merge_cells(start_row=r + 1, start_column=1, end_row=r + 1, end_column=16)
ws.cell(row=r + 1, column=1).alignment = wrap; ws.row_dimensions[r + 1].height = 42

widths = [4, 14, 18, 28, 46, 18, 30, 26, 22, 6, 10, 11, 9, 9, 9, 30, 24, 10, 10, 11, 22, 32, 12]
for i, w in enumerate(widths, 1): ws.column_dimensions[get_column_letter(i)].width = w
ws.freeze_panes = "D5"

# ---- Requirements sheet ----
rq = wb.create_sheet("Requirements")
rq["A1"] = "Requirements derived from the simulation"; rq["A1"].font = Font(name=F, bold=True, size=14)
for c, h in enumerate(["Requirement", "Value", "Why / evidence"], 1):
    cell = rq.cell(row=3, column=c, value=h); cell.font = H; cell.fill = HF; cell.border = BOX
req_rows = [
 ("Motor peak torque (each)", "≥ 0.6 N·m", "Flip reached upright in 7/16 randomized runs at 0.6 N·m vs 4–5/16 at 0.3–0.45 N·m"),
 ("Motor no-load speed", "≥ 950 rpm (100 rad/s)", "At ~480 rpm the wheels are near top speed after the flip; 100 rad/s → 12/16 flips"),
 ("Wheel diameter", "~100 mm (108.6 mm chosen)", "3.25 cm wheels never stood up; 5 cm radius + low battery did"),
 ("Battery position", "As low as possible", "Battery at the top: never stood up. Now on a shelf just above the motors."),
 ("Case v2 (CAD)", "CoM ≈ 34 mm above axle; ≈ 1.61 kg; rests at ~100° / 97° lying flat",
  "Above the ~15 mm the sim assumed: stand-up from lying flat must be re-simulated; it may need more torque or a different flip strategy."),
 ("Drive type", "Direct-drive all-in-one BLDC servo (CAN)", "No gearbox backlash; torque control; no FOC code to write"),
 ("Balance loop rate", "100 Hz (policy) / 500–1000 Hz (stand-up)", "Policy trained at 0.01 s; the flip needs faster control"),
 ("Battery / bus voltage", "6S (22.2 V nominal, 25.2 V full)", "Matches the 24 V-class motors; driver input 7.4–32 V"),
 ("Charging", "CC/CV 25.2 V, ≤ 1 A, BMS with balancing", "Dock (pogo pins on recessed pads) or USB-C PD; motors disabled while charging"),
 ("Safety", "Explicit arm action; cut motors on pitch > 57° after a failed attempt, or wheel overspeed", "Stand-up mode must allow starting from ~100°"),
]
for i, row in enumerate(req_rows, 4):
    for j, v in enumerate(row, 1):
        cell = rq.cell(row=i, column=j, value=v); cell.font = BLK; cell.alignment = wrap; cell.border = BOX
for i, w in enumerate([28, 40, 80], 1): rq.column_dimensions[get_column_letter(i)].width = w

# ---- Wiring ----
wi = wb.create_sheet("Wiring", 1)
wi["A1"] = "Connections (V1.1)"; wi["A1"].font = Font(name=F, bold=True, size=14)
for c, h in enumerate(["From", "To", "Link / signal", "Notes"], 1):
    cell = wi.cell(row=3, column=c, value=h); cell.font = H; cell.fill = HF; cell.border = BOX
wires = [
 ("6S battery (XT60)", "6S BMS B+/B− + balance lead", "Pack", "BMS sits between the pack and everything else."),
 ("BMS P+/P−", "30 A fuse → power switch (Pololu 2813, back cover above the Pi) → distribution", "25.2 V max", "Bulk cap at the distribution point (floor tray)."),
 ("Distribution", "Motor L, Motor R", "6S power", "16 AWG, through the wall slots"),
 ("Distribution", "5 V 5 A regulator", "6S in", "Feeds all logic"),
 ("5 V regulator", "Pi Zero 2 W", "5 V via GPIO pins 2/4 + GND", "Short leads"),
 ("5 V regulator", "ESP32-S3 5V pin, sonar, LED eyes", "5 V", "Eyes star-wired from the regulator, 470 µF at the first panel"),
 ("ESP32-S3 (TWAI TX/RX)", "SN65HVD230 CAN board", "3.3 V logic", ""),
 ("CAN board", "Motor L ↔ Motor R", "CAN H/L, 1 Mbps, twisted pair", "120 Ω at both ends; different motor IDs"),
 ("ESP32-S3", "IMU (tray)", "I²C (MPU6050) or SPI (ICM)", "On the axle line"),
 ("ESP32-S3 GPIO", "HC-SR04 sonar", "TRIG; ECHO via 1 k/2 k divider", "If the sonar is fitted"),
 ("ESP32-S3", "VL53L5CX (front panel), plug J2T", "I\u00b2C on its OWN bus: SDA GPIO42, SCL GPIO41, plus LPn GPIO2 and INT GPIO47", "Fitted instead of the sonar, in its place on the perfboard (6-pin J2T). 3.3 V, address 0x52. Not the IMU's bus: the MPU6050 caps that at 400 kHz and an 8 \u00d7 8 frame is ~51 kB/s at 15 Hz."),
 ("ESP32-S3 GPIO", "Arm button, status LED", "Digital I/O", ""),
 ("Battery+ after switch", "ESP32-S3 ADC1 pin", "100 k / 12 k divider + 100 nF", "25.2 V → 2.70 V"),
 ("ESP32-S3 UART", "Pi UART (GPIO14/15)", "3.3 V UART, 921600 baud", "Common GND"),
 ("Pi camera connector", "Camera Module 3 Wide", "CSI (Zero cable)", ""),
 ("Pi GPIO10 (SPI MOSI)", "74AHCT125 → 330 Ω → left eye DIN → right eye", "WS2812 data", ""),
 ("Pi micro-USB OTG", "LD19 adapter board → lidar", "USB serial", ""),
 ("Dock: Fuyuang 25.2 V charger → DC jack", "Dock pogo pins (+ / −)", "25.2 V CC/CV, 1 A", "In the dock, not on the robot"),
 ("Robot pad + (M5 contact)", "5 A fuse → SB560 anode; cathode → BMS P+", "Charge in", "Diode keeps the pads dead when not docked"),
 ("Robot pad −", "BMS P−", "Charge return", ""),
 ("Robot pad + (before diode)", "100 k / 12 k divider → ESP32 GPIO", "Dock detect", "Divider also pulls the pad to 0 V when undocked"),
 ("Side USB-C socket (CAB-15455)", "IP2368 board USB-C", "USB PD (20 V)", ""),
 ("IP2368 BAT+ / BAT−", "BMS P+ / P−", "25.2 V CC/CV, set to ~1 A", "Check end voltage with a multimeter before first use"),
 ("IP2368 VBUS (or its charge-status pin)", "56 k / 10 k divider → ESP32 GPIO", "USB charge detect (20 V → 3.0 V)", "Charging → motors stay disabled"),
 ("Pi 3.3 V (pin 1) / GND", "Both ICS-43434 mics VDD / GND", "3.3 V", "Left mic SEL → GND, right mic SEL → 3.3 V"),
 ("Pi GPIO18 / PCM_CLK (pin 12)", "Both mics BCLK + amp BCLK", "I2S bit clock", "dtoverlay=googlevoicehat-soundcard"),
 ("Pi GPIO19 / PCM_FS (pin 35)", "Both mics WS/LRCL + amp LRC", "I2S word select", ""),
 ("Pi GPIO20 / PCM_DIN (pin 38)", "Both mics DOUT (shared line)", "I2S data in (stereo: L + R mic)", "Keep these wires short and twisted with GND"),
 ("Pi GPIO21 / PCM_DOUT (pin 40)", "MAX98357A DIN", "I2S data out", ""),
 ("5 V regulator", "MAX98357A VIN / GND", "5 V, up to ~0.8 A peaks at 3 W", "470 µF at the amp. 5 V budget ≈ 3.3 A of 5 A incl. lidar (~0.3 A)"),
 ("MAX98357A + / −", "Speaker", "Bridge-tied output", "Never connect either output to GND"),
 ("Pi Wi-Fi / Bluetooth", "Phone browser / gamepad", "WebRTC + WebSocket / BT", ""),
 ("Pi Wi-Fi", "Claude API (api.anthropic.com), optional Whisper server on the Jetson", "HTTPS", "Voice orders need internet; key in /etc/robot.env"),
]
for i, row in enumerate(wires, 4):
    for j, v in enumerate(row, 1):
        cell = wi.cell(row=i, column=j, value=v); cell.font = BLK; cell.alignment = wrap; cell.border = BOX
for i, w in enumerate([32, 40, 32, 46], 1): wi.column_dimensions[get_column_letter(i)].width = w

# ---- Build check ----
bc = wb.create_sheet("Build check", 2)
bc["A1"] = "Build order and checks"; bc["A1"].font = Font(name=F, bold=True, size=14)
for c, h in enumerate(["#", "Build step", "Parts used", "Check / note"], 1):
    cell = bc.cell(row=3, column=c, value=h); cell.font = H; cell.fill = HF; cell.border = BOX
steps = [
 (1, "Print: body (front opening down), shelf, front panel + cover (outside down), contact plate, tray, clamp bar, 2 carrier plates, 2 eye retainers, 2 diffusers (white), 2 headlight retainers, speaker ring, 2 mic clamps, 2 wheel adapters, 4 IR hoods (black), dock", "PETG ~410 g", "No supports. Poke through the 0.2 mm sacrificial layers in screw recesses."),
 (2, "Press inserts: 36 × M3 (4 of them in the dock's face slab sides), 34 × M2, 2 × M4 (wheel adapters)", "ruthex + iron", "Counts per part in the BOM rows. Keep them square."),
 (3, "Mount motors to the centre wall, connector bay pointing UP", "MF6015 ×2, 8 × M2.5×10", "Blue threadlocker. Plug both leads in from above (6-pin + 3-pin, in the bay on top of the Ø49 boss), then lay them through the wall slot into the centre gap."),
 (4, "Fit wheel adapters + wheels", "Adapters, wheels, M3×8, M4", "Measure wheel offset first (NECK_L)."),
 (5, "Configure motors on the bench", "Neo2 adapter, LKMTECH tool", "IDs, 1 Mbps, direction, limits."),
 (6, "Floor tray: regulator, PDB+fuse (under clamp bar), IMU; screw tray to floor", "Tray, clamp bar", ""),
 (7, "Carrier plates: IP2368 (left wall), lidar adapter (shelf). Speaker + ring + amp on the right wall; mics into the front panel", "Plates, zip ties", ""),
 (8, "Slide the shelf in flat on the wall tops, 4 × M3×10 countersunk into the wall inserts; then battery (strap) and BMS upright in front of it", "Shelf, battery, BMS, strap", "The 4 shelf screws are structural: they tie the motor walls together."),
 (9, "Perfboard + Pi on the back cover; contact plate + M5 contacts outside, 4 × M3×16", "Cover", "Ring lugs on the M5 nuts inside."),
 ("9b", "Dock: sprung rods (collar, spring, M3 washer epoxied over the back), wires soldered to the rod ends, up the column to the controller board; wings on with 4 × M3×12; 4 rubber feet; stand it against a wall", "Dock parts", "Rods slide freely and spring back, ~4 mm travel. Polarity: the rod that meets the robot's + pad (robot Y −25, its right-hand side; the LEFT rod when you look at the dock face) = +25.2 V."),
 ("9c", "Dock controller: build the board per the site's 'Wiring diagram > Dock' page, flash firmware/dock_beacon onto the ATtiny85 (8 MHz internal); LEDs into the mast holes (4 sides + 1 in the centre tube), board standing in the mast's hollow, rod wires (+ = LEFT rod looking at the face) and jack wires down the column. Bench test first (page's checklist): rods 0 V with nothing on them, 25.2 V with 112 kΩ across them, off on a short. Stick the 2 reflective strips into the face guides. Robot: 4 TSOP38238 in their hoods on the top skin, leads to J8", "Dock parts, ISP programmer, 112 kΩ + 1 Ω/5 W test resistors", "Phone camera sees the LEDs flicker. Robot telemetry 'ir' shows bits for the receivers facing the dock; walk round it: they change."),
 (10, "Front panel: AI Camera (focus set to ~1.5 m first) + polycarbonate window, sonar, eyes + diffusers + retainers, headlight discs + retainers, headlight driver; 4 rubber bumpers on the face; USB-C socket in the left side wall", "Front panel + left wall", "Headlights: 1 A fuse in the PDB lead; Pi pin 32 -> driver PWM."),
 (11, "Lidar on top; power button (front-right); switch board inside the back cover, above the Pi; arm button", "", ""),
 (12, "Power-up checks: 5 V rail, CAN ~60 Ω, battery divider calibration", "Multimeter", ""),
 (13, "Charging checks BEFORE connecting the battery: dock rods 25.2 V (+ on the rod that meets the robot's + pad); robot leaned on the dock by hand: 25.2 V on the M5 nuts inside; IP2368 end voltage 25.2 V; pads read 0 V undocked", "Multimeter", "Then a supervised first charge in the LiPo bag."),
 ("13b", "Docking: press 'Dock & charge' with the robot anywhere within ~4 m of the dock. It turns until it hears the beacon or sees the tape, homes in, lines up 45 cm out on the dock's axis and reverses in, leans back: CHARGING (blue LED). 'Undock' stands it up and drives it 0.4 m out", "Web app", "The first dock saves the dock's map position; afterwards 'dock' plans a path there from anywhere on the map (and on low battery by itself). The 'ir' bits in telemetry show which receivers hear the beacon."),
 (14, "Flash ESP32, set up Pi; wheels-off test, balance, then stand-up", "Everything", "Re-simulate with real masses first."),
]
for i, row in enumerate(steps, 4):
    for j, v in enumerate(row, 1):
        cell = bc.cell(row=i, column=j, value=v); cell.font = BLK; cell.alignment = wrap; cell.border = BOX
for i, w in enumerate([5, 60, 26, 60], 1): bc.column_dimensions[get_column_letter(i)].width = w


# ---- Screws: every fastener in the build, one line per size/type ----
SCREWS = [  # size, type / standard, material, used, buy, where (count each), into, key
 ("M3×10", "Countersunk hex socket, ISO 10642", "A2 stainless", 16, 20, "Front panel 6, back cover 6, battery shelf -> wall tops 4", "M3 inserts", "2 mm hex"),
 ("M3×16", "Countersunk hex socket, ISO 10642", "A2 stainless", 4, 5, "Contact plate 4 (10 mm plate + 6 into the cover inserts)", "M3 inserts", "2 mm hex"),
 ("M3×12", "Button hex socket, ISO 7380", "A2", 4, 5, "Dock wings -> dock face slab 2 + 2", "M3 inserts (dock)", "2 mm hex"),
 ("M3×10", "Button hex socket, ISO 7380", "A2", 2, 4, "Clamp bar -> tray 2", "M3 inserts", "2 mm hex"),
 ("M3×8", "Button hex socket, ISO 7380", "A2", 2, 4, "Floor tray -> floor 2", "M3 inserts", "2 mm hex"),
 ("M3×8", "Socket head cap, ISO 4762", "A2", 8, 10, "Wheel adapters -> motor rotor face 4 + 4 (check they don't bottom out)", "Motor threads", "2.5 mm hex"),
 ("M3×6", "Button hex socket, ISO 7380", "A2", 8, 10, "Perfboard 4, carrier plates 2 + 2", "M3 inserts", "2 mm hex"),
 ("M3 nut", "Hex nut, ISO 4032", "A2", 2, 4, "USB-C side socket 2 (its 2 M3 screws come with it)", "—", "5.5 mm spanner"),
 ("M3×10 nylon", "Pan head + nylon nut + 2.5 mm nylon spacer", "PA66", 2, 4, "CAN board through the perfboard 2", "Nut", "PH1"),
 ("M2.5×10", "Socket head cap, ISO 4762", "A2", 8, 10, "Motors -> centre wall 4 + 4 (blue threadlocker)", "Motor threads", "2 mm hex"),
 ("M2×6", "Socket head cap, ISO 4762 (or pan head)", "A2", 31, 36, "Camera 4, sonar 4, eye retainers 4, mic clamps 2, headlight retainers 4, headlight driver 2, lidar 3, regulator 2, IMU 2, Pi 4", "M2 inserts", "1.5 mm hex"),
 ("M2×8", "Socket head cap, ISO 4762 (or pan head)", "A2", 3, 5, "Speaker ring 3", "M2 inserts", "1.5 mm hex"),
 ("M2 washer", "Flat washer, ISO 7089", "A2", 8, 12, "Pi 4, IMU 2, headlight driver 2 (holes drilled Ø2.2)", "—", "—"),
 ("M4×12", "Socket head cap, ISO 4762 + M4 washer ISO 7089", "A2", 2, 2, "Wheel -> adapter 1 + 1", "M4 inserts", "3 mm hex"),
 ("M5×12", "Countersunk slotted, DIN 963 + M5 nut + M5 ring lug", "Brass", 2, 2, "Charge contacts 2 (they carry current: brass, not stainless)", "Nut + ring lug", "Flat screwdriver, 8 mm spanner"),
]
sc = wb.create_sheet("Screws", 3)
sc["A1"] = "Screws and fasteners: every one in the build"; sc["A1"].font = Font(name=F, bold=True, size=14)
sc["A2"] = "'Buy' adds spares. All metric. Easiest: one A2 hex-socket assortment (M2/M2.5/M3/M4) + a pack of 20 M3×10 ISO 10642 countersunk + 2 brass M5×12 countersunk. Shops: Bossard eShop, Schraubenkönig, Galaxus (CH)."
sc["A2"].font = SMALL_I
hdr = ["Size", "Type / standard", "Material", "Used", "Buy", "Where (count)", "Into", "Tool"]
for c, h in enumerate(hdr, 1):
    cell = sc.cell(row=4, column=c, value=h); cell.font = H; cell.fill = HF; cell.border = BOX
for i, row in enumerate(SCREWS, 5):
    for j, v in enumerate(row, 1):
        cell = sc.cell(row=i, column=j, value=v); cell.font = BLK; cell.alignment = wrap; cell.border = BOX
t = 5 + len(SCREWS)
sc.cell(row=t, column=3, value="Total").font = Font(name=F, bold=True)
for col in ("D", "E"):
    c = sc[f"{col}{t}"]; c.value = f"=SUM({col}5:{col}{t-1})"; c.font = Font(name=F, bold=True)
for i, w in enumerate([12, 40, 12, 7, 7, 60, 16, 22], 1): sc.column_dimensions[get_column_letter(i)].width = w

wb.save("balancing_robot_BOM.xlsx")
print("rows", first_item, last, "items", n)
