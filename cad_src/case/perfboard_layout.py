"""Perfboard layout on the 2.54 mm hole grid (component side faces +x, towards the front).

Board: standard 5 x 7 cm double-sided FR4, 24 x 18 holes, 70 (robot y) x 50 (robot z), back face at
x = X_BACK + BOARD_STANDOFF, centre bc.PB_CENTER. Hole (k, j): y = Y0 + 2.54 k, z = Z0 + 2.54 j (k 0..23, j 0..17).
Regions: the ESP32 DevKit sits on two 22-pin female header rows (j 8 and j 17) over y -60..-4; under it there is
8.5 mm of height for flat parts (74AHCT125, resistors). The CAN board stands on spacers at y -64..-31, z 81-91.
Plugs go in the free area below the DevKit (y -31..+6, z 77..98).
"""
import body_case as bc

XB = bc.X_BACK + bc.BOARD_STANDOFF          # board back face
XF = XB + 1.6                               # component face
CY, CZ = bc.PB_CENTER
NK, NJ = 24, 18                             # standard 5 x 7 cm board: 24 x 18 holes (432), 2.54 mm, centred
Y0 = CY - (NK - 1) * 2.54 / 2               # hole k = 0 at the -Y end (5.8 mm from the edge)
Z0 = CZ - (NJ - 1) * 2.54 / 2               # hole j = 0 at the bottom (3.4 mm from the edge)
DK_COLS, DK_ROWS = (1, 22), (8, 17)         # DevKit female headers: holes k 1-22 on rows j 8 and 17


def hole(k, j):
    assert 0 <= k < NK and 0 <= j < NJ, (k, j)
    return (Y0 + 2.54 * k, Z0 + 2.54 * j)


# plugs: name -> (k of pin 1, j, pins, pitch in holes, height, kind). Rows j 0 and 3 stay at k >= 12, clear of the
# CAN board standing on spacers over k 0-11 (y -64..-31, z 81-91).
PLUGS = {
    "J2": (12, 0, 4, 1, 8.5, "2.54 header + Dupont"),
    "J1": (18, 0, 2, 2, 8.0, "XT30PW right-angle PCB plug (15 A)"),
    "J7": (12, 3, 3, 1, 8.5, "2.54 header + Dupont"),
    "J5": (17, 3, 5, 1, 8.5, "2.54 header + Dupont"),
    "J3": (1, 6, 4, 1, 8.5, "2.54 header + Dupont"),
    "J4": (6, 6, 3, 1, 8.5, "2.54 header + Dupont"),
    "J6": (10, 6, 5, 1, 8.5, "2.54 header + Dupont"),
    "J8": (16, 6, 6, 1, 8.5, "2.54 header + Dupont"),        # IR receivers: 3V3, GND, GPIO13-16
    "J9": (22, 6, 1, 1, 8.5, "2.54 header + Dupont"),        # power switch OFF: GPIO21 (100 k pull-down on the board)
}


def plug_pin(name, i):
    k, j, n, p, h, _ = PLUGS[name]
    y, z = hole(k + i * p, j)
    return (XF + h, y, z)                    # wire end = top of the housing


def plug_box(name):
    k, j, n, p, h, kind = PLUGS[name]
    y1, z = hole(k, j); y2, _ = hole(k + (n - 1) * p, j)
    if kind.startswith("XT30"):
        return (XF, XF + h, (y1 + y2) / 2 - 5.1, (y1 + y2) / 2 + 5.1, z - 2.6, z + 2.6)
    return (XF, XF + h, y1 - 1.27, y2 + 1.27, z - 2.5, z + 2.5)


# flat parts: name -> (k1, j1, k2, j2, height, what). Resistors 1/4 W lying flat, leads 10.16 mm apart (4 holes);
# all of them and the 74AHCT125 sit under the DevKit (rows 9-16 are free between its header rows, 8.5 mm of height).
PARTS = {
    "U1 74AHCT125 (DIP-14)": (8, 11, 14, 14, 4.5, "Pi MOSI -> 5 V WS2812 data; 1OE to GND, unused inputs to GND"),
    "R4 100k (VBAT top)":    (1, 10, 5, 10, 2.5, "J7-1 -> GPIO1"),
    "R5 12k (VBAT bottom)":  (1, 11, 5, 11, 2.5, "GPIO1 -> GND"),
    "R6 100k (dock top)":    (1, 12, 5, 12, 2.5, "J7-2 -> GPIO6"),
    "R7 12k (dock bottom)":  (1, 13, 5, 13, 2.5, "GPIO6 -> GND"),
    "R8 56k (USB top)":      (1, 14, 5, 14, 2.5, "J7-3 -> GPIO7"),
    "R9 10k (USB bottom)":   (1, 15, 5, 15, 2.5, "GPIO7 -> GND"),
    "R2 1k (echo top)":      (16, 10, 20, 10, 2.5, "J2-3 -> GPIO11"),
    "R3 2k (echo bottom)":   (16, 11, 20, 11, 2.5, "GPIO11 -> GND"),
    "R1 330 (eye data)":     (16, 12, 20, 12, 2.5, "U1 1Y -> J4-2"),
    "R10 330 (LED R)":       (16, 13, 20, 13, 2.5, "GPIO38 -> J6-2"),
    "R11 330 (LED G)":       (16, 14, 20, 14, 2.5, "GPIO39 -> J6-3"),
    "R12 330 (LED B)":       (16, 15, 20, 15, 2.5, "GPIO40 -> J6-4"),
    "C1 470 uF 10 V (6.3 x 11)": (22, 0, 23, 0, 11.0, "5 V bus, beside the 5 V input plug J1 (was under J8)"),
}


def part_box(name):
    k1, j1, k2, j2, h, _ = PARTS[name]
    y1, z1 = hole(k1, j1); y2, z2 = hole(k2, j2)
    if name.startswith("C1"):
        c = ((y1 + y2) / 2, z1)
        return (XF, XF + h, c[0] - 3.2, c[0] + 3.2, c[1] - 3.2, c[1] + 3.2)
    return (XF, XF + h, min(y1, y2) - 1.2, max(y1, y2) + 1.2, min(z1, z2) - 1.2, max(z1, z2) + 1.2)
