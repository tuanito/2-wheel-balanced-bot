"""dock_wiring.html: the charging dock's electronics (charger input, safe rod switch, IR beacon) - block schematic,
every connection, part values, safety and the bench test. Source of truth for the logic: firmware/dock_beacon/."""
import pathlib as _pl, sys as _sys
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "cad_src" / "case"))
import robot_paths as RP            # where everything is on this machine (BOT_WS still honoured)
import html

SITE = str(RP.SITE)
E = html.escape

HEAD = open(SITE + "/wiring_tpl.html").read().split("</style>")[0] + """
.blk rect{fill:var(--box);stroke:var(--bd);stroke-width:1.2}.blk.p rect{stroke:#d32f2f}.blk.g rect{stroke:#2e7d32}.blk.i rect{stroke:#8e24aa}
svg line,svg polyline{fill:none;stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}
.k{display:inline-block;width:14px;height:4px;border-radius:2px;vertical-align:middle;margin-right:4px}
ul.s{margin:4px 0 0;padding-left:18px}ul.s li{margin:3px 0}
</style></head><body>"""
HEAD = HEAD.replace("<title>Balancing robot V1.1 – wiring diagram</title>", "<title>Balancing robot V1.1 – dock wiring</title>")

# ---------------- block schematic ----------------
W, H = 1260, 740
blocks = [  # key, x, y, w, h, class, title, lines
    ("chg", 20, 40, 180, 78, "p", "Wall charger", ["25.2 V 1 A CC/CV Li-ion", "short-circuit protected", "(certified, SELV)"]),
    ("j1", 240, 40, 140, 78, "p", "J1 DC jack", ["5.5 / 2.1 mm, centre +", "in the base's back edge"]),
    ("f1", 420, 40, 150, 78, "p", "F1 PTC fuse", ["Bourns MF-R110", "1.1 A hold, 60 V"]),
    ("q1", 610, 40, 210, 78, "p", "Q1 reverse-polarity", ["FQP27P06 P-MOSFET", "gate 10 k to GND,", "12 V zener gate-source"]),
    ("d1", 420, 160, 150, 64, "p", "D1 TVS", ["SMBJ28A, +V25 to GND", "(spikes when plugging)"]),
    ("u1", 610, 170, 210, 70, "g", "U1 5 V buck", ["Pololu D24V5F5 (0.5 A)", "100 µF + 100 nF on 5 V"]),
    ("mcu", 610, 300, 210, 220, "g", "ATtiny85 (DIP-8)", ["pin 8 VCC 5 V, pin 4 GND", "pin 5 PB0 → rod switch", "pin 2 PB3 ← rod sense", "pin 6 PB1 → 'A' LEDs", "pin 3 PB4 → 'C' LED", "pin 7 PB2 → status LED", "pin 1 RESET 10 k to 5 V", "firmware: dock_beacon.ino"]),
    ("q2", 900, 40, 330, 96, "p", "Q2 rod switch (high side)", ["FQP27P06 P-MOSFET, source = V25, drain = ROD+", "gate: 22 k to source, 12 V zener gate-source,", "22 k to Q3 drain. Q3 2N7000: gate ← PB0 (100 Ω,", "100 k pull-down): rods OFF at reset / no firmware"]),
    ("sns", 900, 170, 330, 96, "p", "Rod sense", ["R6 100 k from V25 to ROD+ (weak feed: ≤0.25 mA)", "ROD+ → R7 220 k → PB3 → R8 47 k → GND, C 100 nF", "robot's 112 k detect divider → 11 V (ADC ~400)", "open 18 V (660), coin/short 0 V (0): stays OFF"]),
    ("rod", 900, 300, 330, 96, "p", "Charge rods (sprung brass Ø4)", ["ROD+ → LEFT rod, looking at the dock face", "(meets the robot's + pad, its right-hand side)", "GND → RIGHT rod (−)", "22 AWG, down the column to the board"]),
    ("la", 900, 430, 330, 90, "i", "'A' all-round IR LEDs", ["4 × TSAL6200 (940 nm) in the mast's side holes", "2 strings: 5 V → 22 Ω → LED → LED → Q4 drain", "Q4 IRLB8721, gate ← PB1 (100 Ω, 100 k pull-down)", "~100 mA per string in 600 µs bursts at 38 kHz"]),
    ("lc", 900, 545, 330, 80, "i", "'C' centre IR LED", ["1 × TSAL6200 at the end of the 22 mm tube", "5 V → 33 Ω → LED → Q5 drain (IRLB8721, ← PB4)", "~100 mA in 1200 µs bursts: ±6° beam on the axis"]),
    ("st", 610, 560, 210, 76, "g", "Status LED (green 3 mm)", ["PB2 → 1 k → LED → GND", "blip: waiting · on: charging", "2 Hz: short · fast: latched"]),
    ("gnd", 20, 300, 180, 120, "", "GND (one star point)", ["J1 sleeve, TVS, buck GND,", "ATtiny pin 4, Q3/Q4/Q5", "sources, R8, status LED,", "RIGHT rod (−)"]),
]
B = {b[0]: b for b in blocks}
P = lambda k, side, f=0.5: {"l": (B[k][1], B[k][2] + B[k][4] * f), "r": (B[k][1] + B[k][3], B[k][2] + B[k][4] * f),
                            "t": (B[k][1] + B[k][3] * f, B[k][2]), "b": (B[k][1] + B[k][3] * f, B[k][2] + B[k][4])}[side]
wires = [  # colour, points, label
    ("#d32f2f", [P("chg", "r"), P("j1", "l")], ""),
    ("#d32f2f", [P("j1", "r"), P("f1", "l")], ""),
    ("#d32f2f", [P("f1", "r"), P("q1", "l")], ""),
    ("#d32f2f", [P("f1", "b"), P("d1", "t")], ""),
    ("#d32f2f", [P("q1", "r"), (860, P("q1", "r")[1]), (860, P("q2", "l", .3)[1]), P("q2", "l", .3)], "V25"),
    ("#d32f2f", [(860, 88), (860, P("sns", "l", .3)[1]), P("sns", "l", .3)], ""),
    ("#d32f2f", [(860, 140), (840, 140), (840, P("u1", "r")[1]), P("u1", "r")], ""),
    ("#f57c00", [P("u1", "b"), P("mcu", "t")], "5 V"),
    ("#1976d2", [P("mcu", "r", .25), (880, P("mcu", "r", .25)[1]), (880, P("q2", "l", .8)[1]), P("q2", "l", .8)], "PB0"),
    ("#1976d2", [P("mcu", "r", .38), (890, P("mcu", "r", .38)[1]), (890, P("sns", "l", .75)[1]), P("sns", "l", .75)], "PB3"),
    ("#7b1fa2", [P("mcu", "r", .52), (885, P("mcu", "r", .52)[1]), (885, P("la", "l", .7)[1]), P("la", "l", .7)], "PB1"),
    ("#7b1fa2", [P("mcu", "r", .66), (875, P("mcu", "r", .66)[1]), (875, P("lc", "l", .7)[1]), P("lc", "l", .7)], "PB4"),
    ("#2e7d32", [P("mcu", "b"), P("st", "t")], "PB2"),
    ("#d32f2f", [P("q2", "b"), P("sns", "t")], "ROD+"),
    ("#d32f2f", [P("sns", "b"), P("rod", "t")], "ROD+"),
    ("#212121", [P("gnd", "r", .3), (P("d1", "b")[0], P("gnd", "r", .3)[1]), P("d1", "b")], ""),
    ("#212121", [P("gnd", "r", .7), (560, P("gnd", "r", .7)[1]), (560, P("mcu", "l", .66)[1]), P("mcu", "l", .66)], "GND"),
    ("#212121", [P("gnd", "b"), (110, 700), (1245, 700), (1245, P("rod", "r", .75)[1]), P("rod", "r", .75)], "GND → right rod (−), Q4/Q5 sources"),
    ("#212121", [P("j1", "b"), (310, 250), (110, 250), P("gnd", "t")], "sleeve"),
]


def svg():
    o = [f'<svg viewBox="0 0 {W} {H}" width="{W}" height="{H}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Dock electronics block schematic">']
    for col, pts, lab in wires:
        o.append(f'<polyline points="{" ".join(f"{x:.0f},{y:.0f}" for x, y in pts)}" stroke="{col}"/>')
        if lab:
            (x0, y0), (x1, y1) = pts[0], pts[1]
            o.append(f'<text x="{(x0 + x1) / 2 + 4:.0f}" y="{(y0 + y1) / 2 - 5:.0f}" style="fill:{col};font-weight:600">{E(lab)}</text>')
    for k, x, y, w, h, c, t, lines in blocks:
        o.append(f'<g class="blk {c}"><rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8"/>'
                 f'<text class="t" x="{x + 10}" y="{y + 18}">{E(t)}</text>')
        for i, l in enumerate(lines):
            o.append(f'<text x="{x + 10}" y="{y + 36 + 15 * i}">{E(l)}</text>')
        o.append('</g>')
    o.append('</svg>')
    return "".join(o)


# ---------------- every connection ----------------
CONN = [
    ("V_IN", "Charger plug (centre +)", "J1 centre pin", "—", "plug"),
    ("V_IN", "J1 centre", "F1 PTC in", "22 AWG red", "short"),
    ("V_F", "F1 out", "Q1 drain, D1 cathode", "22 AWG red", "on board"),
    ("V25", "Q1 source", "Q2 source, R6, U1 VIN, Q1/Q2 gate resistors", "22 AWG red", "on board"),
    ("Q1 gate", "Q1 gate", "R1 10 k → GND; Z1 12 V: anode gate, cathode source", "—", "on board"),
    ("Q2 gate", "Q2 gate", "R2 22 k → source; Z2 12 V: anode gate, cathode source; R3 22 k → Q3 drain", "—", "on board"),
    ("ROD_EN", "ATtiny pin 5 (PB0)", "R4 100 Ω → Q3 2N7000 gate; R5 100 k gate → GND", "—", "on board"),
    ("ROD+", "Q2 drain", "LEFT rod (looking at the face), R6 100 k from V25, R7 220 k", "22 AWG red, ~45 cm", "down the column"),
    ("SENSE", "R7 / R8 junction", "ATtiny pin 2 (PB3); C3 100 nF → GND; R8 47 k → GND", "—", "on board"),
    ("GND", "J1 sleeve", "star point: D1 anode, U1 GND, ATtiny pin 4, Q3/Q4/Q5 sources, R8, C1-C3, status LED", "22 AWG black", "short"),
    ("ROD−", "GND star point", "RIGHT rod", "22 AWG black, ~45 cm", "down the column"),
    ("5 V", "U1 VOUT", "ATtiny pin 8, C1 100 µF, C2 100 nF, R9/R10 22 Ω, R11 33 Ω, RESET pull-up 10 k (pin 1)", "26 AWG", "on board"),
    ("LED A1", "R9 22 Ω", "LED A1 anode (long leg) → A1 cathode → A2 anode → A2 cathode → Q4 drain", "26 AWG pair", "mast: front + back holes"),
    ("LED A2", "R10 22 Ω", "LED A3 anode → A3 cathode → A4 anode → A4 cathode → Q4 drain", "26 AWG pair", "mast: left + right holes"),
    ("A_DRV", "ATtiny pin 6 (PB1)", "R12 100 Ω → Q4 IRLB8721 gate; R13 100 k gate → GND", "—", "on board"),
    ("LED C", "R11 33 Ω", "LED C anode → cathode → Q5 drain", "26 AWG pair", "centre tube (front, y −20)"),
    ("C_DRV", "ATtiny pin 3 (PB4)", "R14 100 Ω → Q5 IRLB8721 gate; R15 100 k gate → GND", "—", "on board"),
    ("STATUS", "ATtiny pin 7 (PB2)", "R16 1 k → green LED anode, cathode → GND", "26 AWG", "LED in a Ø3 hole in the mast back"),
]

PARTS = [
    ("J1", "DC jack 5.5/2.1, panel (exists in the dock BOM row)"), ("F1", "PTC fuse Bourns MF-R110 (1.1 A hold, 2.2 A trip, 60 V)"),
    ("D1", "TVS SMBJ28A (28 V stand-off)"), ("Q1, Q2", "FQP27P06 P-MOSFET (−60 V, 0.07 Ω, TO-220), bend flat"),
    ("Z1, Z2", "12 V zener 0.5 W (BZX79-C12)"), ("Q3", "2N7000"), ("Q4, Q5", "IRLB8721 (logic level, TO-220)"),
    ("U1", "Pololu D24V5F5 5 V 0.5 A buck"), ("IC1", "ATtiny85-20PU + DIP-8 socket"),
    ("R1", "10 k"), ("R2, R3", "22 k"), ("R4, R12, R14", "100 Ω"), ("R5, R13, R15", "100 k"), ("R6", "100 k 1 %"),
    ("R7", "220 k 1 %"), ("R8", "47 k 1 %"), ("R9, R10", "22 Ω 0.5 W"), ("R11", "33 Ω 0.5 W"), ("R16", "1 k"), ("RESET", "10 k"),
    ("C1", "100 µF 10 V"), ("C2, C3", "100 nF"), ("LEDs", "5 × Vishay TSAL6200 (940 nm, Ø5) + 1 green 3 mm"),
    ("Board", "40 × 30 mm perfboard, standing in the mast's hollow; wires down the column to the jack and rods"),
]

SAFETY = [
    "Only safety extra-low voltage in the dock: 25.2 V DC from a certified, short-circuit-protected wall charger. Nothing in the dock touches mains.",
    "The rods are dead unless the robot is on them. The board feeds a weak sense current (≤ 0.25 mA) and only switches the 25 V on when it sees the robot's own 112 kΩ detect divider for 0.5 s. Nothing on the rods, a coin, a key, a wet hand (a few kΩ) or a short keeps them off.",
    "While charging it switches off for 30 ms every 2 s and checks the robot is still there: the rods go dead within 2 s of the robot leaving (the robot ignores dropouts under 0.15 s).",
    "Short or overload while live (rod voltage under 15 V): off within ~10 ms, 5 s pause, then back to sensing. Three faults within a minute latch it off until the dock is unplugged (status LED blinks fast).",
    "Q2 is off without firmware: its gate is pulled to its source, Q3 is held off by a 100 k pull-down. A blank or crashed ATtiny leaves the rods dead.",
    "F1 (1.1 A PTC) limits any fault current the charger's own protection doesn't; D1 clamps plug-in spikes; Q1 blocks a wrong-polarity supply (the robot would otherwise see −25 V on its sense pin).",
    "Robot side: its pads are dead when off the dock (SB560 diode) and fused at 5 A; the BMS protects each cell; charging at 1 A is 0.7 C for the 1400 mAh pack.",
    "First charges supervised, robot on a non-flammable surface; LiPo bag for storage. The IR LEDs are low-power (≈ 10 mA average) 940 nm: invisible, harmless at normal distances.",
]

TEST = [
    "Before fitting the ATtiny: plug the charger in. V25 = 25.2 V, 5 V rail = 5.0 V, rods 0 V (Q2 off). Reverse a bench supply into J1: nothing on V25 (Q1 blocks).",
    "Flash dock_beacon.ino, fit the ATtiny. Rods open: rod+ reads ~18 V on a meter (the sense feed through 100 k; it collapses under any meter load below ~1 MΩ, so use a 10 MΩ DMM). Status LED blips every 2 s.",
    "Put 112 kΩ (100 k + 12 k) across the rods: after 0.5 s they go to 25.2 V and the status LED stays on. Remove it: 0 V within 2 s.",
    "Short the rods with a 1 Ω / 5 W resistor (not a wire): rods off at once, LED blinks 2 Hz, then retries; after 3 shorts it blinks fast and stays off until unplugged.",
    "Phone camera at the mast: the IR LEDs flicker purple. Robot telemetry 'ir' shows bits for the receivers facing the dock.",
    "First real dock: the robot leans on, the status LED goes on after 0.5 s, the robot shows CHARGING (blue).",
]


def page():
    rows = "".join(f"<tr><td><b>{E(n)}</b></td><td>{E(a)}</td><td>{E(b)}</td><td>{E(w)}</td><td>{E(r)}</td></tr>" for n, a, b, w, r in CONN)
    parts = "".join(f"<tr><td><b>{E(k)}</b></td><td>{E(v)}</td></tr>" for k, v in PARTS)
    return (HEAD.replace("<body>", "<body>") +
            '<header><h1>Balancing robot V1.1 — dock wiring</h1>'
            '<p>The charging dock\'s electronics: charger input, a rod switch that only makes the rods live when the robot is on them, and the IR beacon. One ATtiny85 runs both (firmware/dock_beacon). Dock parts in the BOM, rows "Dock controller board", "Dock IR beacon LEDs" and "Dock contact rods".</p></header>'
            '<nav class="top" aria-label="Pages"><a href="index.html">3D viewer</a><a href="wiring_boards.html" class="on" aria-current="page">Wiring diagram</a><a href="print.html">How it prints</a><a href="docs.html">README &amp; install</a><a href="code.html">Source code</a><a href="bom.html">Parts list</a></nav>'
            '<div class="sub"><a href="wiring_boards.html">Board pictures</a><a href="wiring.html">Pin list diagram + wire table</a><a href="dock_wiring.html" class="on" aria-current="page">Dock</a></div>'
            '<div class="card"><h2>Safety</h2><ul class="s">' + "".join(f"<li>{E(s)}</li>" for s in SAFETY) + '</ul></div>'
            '<div id="wrap">' + svg() + '</div>'
            '<div class="card"><h2>Every connection</h2><div class="tw"><table><tr><th>Net</th><th>From</th><th>To</th><th>Wire</th><th>Where</th></tr>' + rows + '</table></div></div>'
            '<div class="card"><h2>Part values</h2><div class="tw"><table><tr><th>Ref</th><th>Part</th></tr>' + parts + '</table></div>'
            '<p style="color:var(--mut);font-size:13px;margin:8px 0 0">LED current: (5 V − 2 × 1.4 V) / 22 Ω ≈ 100 mA per all-round string, (5 V − 1.45 V) / 33 Ω ≈ 105 mA for the centre LED, in short bursts (≈ 10 mA average). Q2 at 1 A dissipates 0.07 W: no heatsink. Sense thresholds are in firmware/dock_beacon/dock_logic.h and are unit-tested (test_dock_logic.cpp).</p></div>'
            '<div class="card"><h2>Bench test before the first dock</h2><ol class="s">' + "".join(f"<li>{E(s)}</li>" for s in TEST) + '</ol></div>'
            '</body></html>')


open(SITE + "/dock_wiring.html", "w").write(page())
print("dock_wiring.html", len(page()) // 1024, "kB")
