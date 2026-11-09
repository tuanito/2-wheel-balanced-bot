"""Fritzing-style 'board pictures' wiring page: each module drawn as it looks, every wire to its real pin.
Reads wiring_diagram.json (see cad_src/case/robot_paths.py), writes wiring_boards.html."""
import pathlib as _pl, sys as _sys
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "cad_src" / "case"))
import robot_paths as RP            # where everything is on this machine (BOT_WS still honoured)
import json, html, sys
RP.add_sources()
import body_case as _bc
bc_ESP_Z, bc_ESP_DY = _bc.ESP_Z, _bc.ESP_DY
D = json.load(open(RP.artifact('wiring_diagram.json')))
E = html.escape
W, H = 1800, 1290
S = []          # svg pieces (boards)
P = {}          # (module, pin) -> (x, y, dir)
DV = {'l': (-1, 0), 'r': (1, 0), 'u': (0, -1), 'd': (0, 1)}


def g(mod, body):
    S.append(f'<g class="bd" data-m="{E(mod)}">{body}</g>')


def rect(x, y, w, h, fill, rx=6, stroke='#0004', extra=''):
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="1.2" {extra}/>'


def text(x, y, s, size=11, fill='#fff', anchor='start', weight='400', cls=''):
    return f'<text x="{x}" y="{y}" font-size="{size}" style="fill:{fill}" text-anchor="{anchor}" font-weight="{weight}" class="{cls}">{E(s)}</text>'


def title(x, y, s, w=None):
    return f'<text x="{x}" y="{y}" class="ttl">{E(s)}</text>'


def pin(mod, name, x, y, d, label=None, lc='#fff', used=True, shape='sq'):
    """Header pin / pad at x,y, wire leaves in direction d; label inside the board."""
    out = ''
    if shape == 'sq':
        out += f'<rect x="{x-4}" y="{y-4}" width="8" height="8" rx="1" fill="#111" stroke="#000"/><rect x="{x-1.6}" y="{y-1.6}" width="3.2" height="3.2" fill="#c9a43c"/>'
    elif shape == 'pad':
        out += f'<circle cx="{x}" cy="{y}" r="4.6" fill="#d8b04a" stroke="#7a5a10"/><circle cx="{x}" cy="{y}" r="1.6" fill="#333"/>'
    elif shape == 'term':
        out += f'<rect x="{x-7}" y="{y-7}" width="14" height="14" rx="2" fill="#2e7d32" stroke="#0005"/><circle cx="{x}" cy="{y}" r="4" fill="#bbb" stroke="#555"/><line x1="{x-3}" y1="{y}" x2="{x+3}" y2="{y}" stroke="#555" stroke-width="1.4"/>'
    elif shape == 'none':
        pass
    if label is not None:
        dx, dy = DV[d]
        if d in 'lr':
            off = 11 if shape in ('pad','term') else 9
            tx, ty, an = x - dx * off, y + 3.5, ('start' if d == 'l' else 'end')
            out += f'<text x="{tx}" y="{ty}" font-size="9.5" style="fill:{lc}" text-anchor="{an}">{E(label)}</text>'
        else:
            tx, ty = x + 3.5, y - dy * 9
            an = 'end' if d == 'u' else 'start'
            out += f'<text x="{tx}" y="{ty}" font-size="9.5" style="fill:{lc}" text-anchor="{an}" transform="rotate(-90 {tx} {ty})">{E(label)}</text>'
    if used:
        P[(mod, name)] = (x, y, d)
    return out


# ---------------- left: power ----------------
def battery(x, y):
    m = "Battery 6S 1400"
    b = rect(x, y, 200, 96, '#2b4a8b', 10) + rect(x + 6, y + 6, 188, 84, '#33569e', 7, 'none')
    b += text(x + 14, y + 36, '6S LiPo', 18, '#fff', weight='700') + text(x + 14, y + 56, '1400 mAh · 22.2 V', 12, '#dbe4ff')
    # leads
    b += f'<path d="M{x+200} {y+22} H{x+226}" stroke="#d32f2f" stroke-width="5"/><path d="M{x+200} {y+42} H{x+226}" stroke="#222" stroke-width="5"/>'
    b += rect(x + 226, y + 14, 26, 36, '#f2c200', 3) + text(x + 239, y + 36, 'XT60', 8, '#333', 'middle')
    b += f'<path d="M{x+200} {y+72} H{x+222}" stroke="#ddd" stroke-width="4"/>' + rect(x + 222, y + 62, 30, 20, '#f4f4f4', 2, '#999') + text(x + 237, y + 75, 'XH', 8, '#333', 'middle')
    b += pin(m, 'lead +', x + 252, y + 22, 'r', shape='none') + pin(m, 'lead -', x + 252, y + 42, 'r', shape='none') + pin(m, 'balance lead', x + 252, y + 72, 'r', shape='none')
    b += title(x, y - 8, m)
    g(m, b)


def bms(x, y):
    m = "6S BMS"
    b = rect(x, y, 150, 120, '#1f6f3f', 4) + text(x + 75, y + 64, '6S BMS', 14, '#cfe9d8', 'middle', '700')
    b += rect(x + 50, y + 20, 50, 22, '#222', 2)  # mosfets
    b += pin(m, 'B+', x + 10, y + 26, 'l', 'B+', shape='pad') + pin(m, 'B-', x + 10, y + 50, 'l', 'B−', shape='pad')
    b += rect(x + 4, y + 82, 36, 22, '#f4f4f4', 2, '#999') + pin(m, 'JST-XH 7p', x + 4, y + 93, 'l', shape='none') + text(x + 22, y + 116, 'BAL', 8, '#cfe9d8', 'middle')
    b += pin(m, 'P+', x + 140, y + 40, 'r', 'P+', shape='pad') + pin(m, 'P-', x + 140, y + 80, 'r', 'P−', shape='pad')
    b += title(x, y - 8, m)
    g(m, b)


def pwrswitch(x, y):
    m = "Power switch + 30 A fuse"
    b = rect(x, y, 150, 86, '#1b5e20', 6) + text(x + 75, y + 30, 'Pololu 2813', 10, '#dff0e0', 'middle', '700')
    b += text(x + 75, y + 44, 'pushbutton power switch', 8, '#cfe8d0', 'middle')
    b += rect(x + 100, y - 2, 26, 16, '#b71c1c', 3) + text(x + 113, y + 10, '30A', 7, '#fff', 'middle')
    b += pin(m, 'VIN (BAT+ from the fuse)', x + 12, y + 16, 'l', 'VIN', shape='pad') + pin(m, 'VOUT (BAT+ to the PDB)', x + 12, y + 40, 'l', 'VOUT', shape='pad')
    b += pin(m, 'GND', x + 12, y + 64, 'l', 'GND', shape='pad')
    b += pin(m, 'OFF', x + 138, y + 30, 'r', 'OFF', shape='pad') + pin(m, 'button pad A', x + 138, y + 52, 'r', 'btn', shape='pad') + pin(m, 'button pad B', x + 138, y + 70, 'r', 'btn', shape='pad')
    b += rect(x + 40, y + 60, 50, 16, '#212121', 6) + text(x + 65, y + 71, 'P- splice', 7, '#ddd', 'middle')
    b += pin(m, 'P- splice (beside the board)', x + 65, y + 86, 'd', shape='none')
    b += title(x, y - 8, "Power switch (Pololu 2813, switches BAT+) + 30 A fuse")
    g(m, b)


def powerbutton(x, y):
    m = "Power button (12 mm)"
    b = rect(x, y, 70, 50, '#555c66', 6) + f'<circle cx="{x+35}" cy="{y+25}" r="15" fill="#333" stroke="#111"/><circle cx="{x+35}" cy="{y+25}" r="10" fill="#555"/>'
    b += pin(m, 'A', x + 10, y + 50, 'd', 'A', '#ddd', shape='pad') + pin(m, 'B', x + 60, y + 50, 'd', 'B', '#ddd', shape='pad')
    b += title(x, y - 8, "Power button (12 mm, momentary)")
    g(m, b)


def pdb(x, y):
    m = "Power distribution (PDB)"
    b = rect(x, y, 170, 110, '#1b1b1b', 6) + text(x + 85, y + 60, 'PDB', 16, '#ddd', 'middle', '700')
    b += rect(x + 10, y + 10, 150, 14, '#b87333', 3, 'none') + rect(x + 10, y + 86, 150, 14, '#b87333', 3, 'none')
    b += pin(m, 'in +', x + 160, y + 30, 'r', 'IN+', '#ddd', shape='pad') + pin(m, 'in -', x + 160, y + 50, 'r', 'IN−', '#ddd', shape='pad')
    b += pin(m, 'out +', x + 115, y + 100, 'd', 'OUT+', '#ddd', shape='pad') + pin(m, 'out -', x + 145, y + 100, 'd', 'OUT−', '#ddd', shape='pad')
    b += title(x, y - 8, "Power distribution board")
    g(m, b)


def regulator(x, y):
    m = "5 V regulator (D24V50F5)"
    b = rect(x, y, 110, 70, '#1d3f9c', 3) + rect(x + 52, y + 14, 26, 26, '#222', 2) + text(x + 65, y + 60, 'D24V50F5', 8, '#cfd8ff', 'middle')
    b += pin(m, 'VIN', x + 10, y + 16, 'l', 'VIN') + pin(m, 'GND', x + 10, y + 34, 'l', 'GND') + pin(m, 'VOUT', x + 100, y + 25, 'r', 'VOUT')
    b += title(x, y - 8, "5 V / 5 A regulator (3 A fuse in VIN)")
    g(m, b)


def dock(x, y):
    m = "Dock contacts"
    b = rect(x, y, 70, 90, '#888f99', 6) + f'<circle cx="{x+35}" cy="{y+26}" r="13" fill="#d6b24a" stroke="#7a5a10"/><circle cx="{x+35}" cy="{y+64}" r="13" fill="#d6b24a" stroke="#7a5a10"/>'
    b += text(x + 35, y + 30, '+', 13, '#333', 'middle', '700') + text(x + 35, y + 68, '−', 13, '#333', 'middle', '700')
    b += pin(m, '+ (ring lug)', x + 70, y + 26, 'r', shape='none') + pin(m, '- (ring lug)', x + 70, y + 64, 'r', shape='none')
    b += title(x, y - 8, "Dock contact pads")
    g(m, b)


def fusediode(x, y):
    m = "Charge fuse + diode"
    b = rect(x, y, 150, 70, '#3a3f47', 6)
    b += rect(x + 30, y + 10, 40, 18, '#e8e8e8', 8, '#999') + text(x + 50, y + 23, '5 A', 9, '#333', 'middle')
    b += rect(x + 84, y + 12, 34, 14, '#111', 2) + f'<rect x="{x+110}" y="{y+12}" width="5" height="14" fill="#ccc"/>' + text(x + 101, y + 40, 'SB560', 8, '#ddd', 'middle')
    b += pin(m, 'IN (5 A fuse -> SB560 anode)', x + 10, y + 20, 'l', 'IN', '#ddd', shape='pad')
    b += pin(m, 'P- pass-through', x + 10, y + 52, 'l', '−', '#ddd', shape='pad')
    b += pin(m, 'OUT (SB560 cathode)', x + 140, y + 20, 'r', 'OUT', '#ddd', shape='pad')
    b += f'<line x1="{x+14}" y1="{y+52}" x2="{x+136}" y2="{y+52}" stroke="#222" stroke-width="3"/>' + pin(m, 'P- out', x + 140, y + 52, 'r', shape='pad', used=False)
    b += title(x, y - 8, "Charge input: 5 A fuse + SB560 diode")
    g(m, b)


def usbc(x, y):
    m = "USB-C socket (side)"
    b = rect(x, y, 70, 40, '#444', 6) + rect(x + 12, y + 13, 40, 14, '#bbb', 7, '#666')
    b += pin(m, 'USB-C socket', x + 70, y + 20, 'r', shape='none') + title(x, y - 8, "USB-C socket (left side)")
    g(m, b)


def ip2368(x, y):
    m = "IP2368 USB-C charger"
    b = rect(x, y, 130, 70, '#1d3f9c', 4) + rect(x + 50, y + 20, 30, 30, '#222', 3) + text(x + 65, y + 64, 'IP2368', 9, '#cfd8ff', 'middle')
    b += rect(x - 6, y + 26, 18, 18, '#bbb', 4, '#666') + pin(m, 'USB-C', x - 6, y + 35, 'l', shape='none')
    b += pin(m, 'BAT+', x + 120, y + 22, 'r', 'BAT+', shape='pad') + pin(m, 'BAT-', x + 120, y + 48, 'r', 'BAT−', shape='pad')
    b += pin(m, 'VBUS pad', x + 30, y + 4, 'u', 'VBUS', shape='pad')
    b += title(x, y - 8, "IP2368 USB-C PD charger")
    g(m, b)


# ---------------- centre: controllers ----------------
def perfboard(x, y):
    """To scale, from case/perfboard_layout.py, seen from the component side (robot +y to the left)."""
    import sys; sys.path.insert(0, str(RP.CASE))
    import perfboard_layout as PL
    m = "Perfboard: ESP32-S3 + level shifter"
    S = 6.6
    YMAX, ZMAX = PL.CY + 35, PL.CZ + 25
    X = lambda yy: x + (YMAX - yy) * S
    Yp = lambda zz: y + (ZMAX - zz) * S
    w, h = 70 * S, 50 * S
    b = rect(x, y, w, h, '#c9a26b', 4, '#7a5b2e')
    for kk in range(PL.NK):
        for jj in range(PL.NJ):
            hy, hz = PL.hole(kk, jj)
            b += f'<circle cx="{X(hy):.1f}" cy="{Yp(hz):.1f}" r="2.2" fill="#d8b04a" stroke="#8a6a3a" stroke-width=".5"/>'
    # CAN board footprint (it stands above the perfboard on 2.5 mm spacers)
    b += f'<rect x="{X(-31.2):.1f}" y="{Yp(91.2):.1f}" width="{(63.7-31.2)*S:.1f}" height="{(91.2-80.9)*S:.1f}" fill="none" stroke="#1d4fa8" stroke-dasharray="5 4" stroke-width="1.5"/>'
    b += text(X(-47), Yp(84.5), 'CAN board above (spacers)', 9, '#1d4fa8', 'middle', '700')
    # flat parts under the DevKit
    for name, (k1, j1, k2, j2, hh, what) in PL.PARTS.items():
        y1, z1 = PL.hole(k1, j1); y2, z2 = PL.hole(k2, j2)
        if name.startswith('U1'):
            b += f'<rect x="{min(X(y1),X(y2))-6:.1f}" y="{min(Yp(z1),Yp(z2))-6:.1f}" width="{abs(X(y1)-X(y2))+12:.1f}" height="{abs(Yp(z1)-Yp(z2))+12:.1f}" rx="2" fill="#222"/>'
            b += text((X(y1)+X(y2))/2, (Yp(z1)+Yp(z2))/2+3, '74AHCT125', 8, '#ddd', 'middle')
        elif name.startswith('C1'):
            cx = (X(y1)+X(y2))/2
            b += f'<circle cx="{cx:.1f}" cy="{Yp(z1):.1f}" r="{3.15*S:.1f}" fill="#263238" stroke="#000"/>' + text(cx, Yp(z1)+3, '470µ', 8, '#ddd', 'middle')
        else:
            b += f'<line x1="{X(y1):.1f}" y1="{Yp(z1):.1f}" x2="{X(y2):.1f}" y2="{Yp(z2):.1f}" stroke="#888" stroke-width="1.5"/>'
            b += f'<rect x="{min(X(y1),X(y2))+4:.1f}" y="{Yp(z1)-4:.1f}" width="{abs(X(y1)-X(y2))-8:.1f}" height="8" rx="3" fill="#d9c19a" stroke="#8a6a3a"/>'
            b += text((X(y1)+X(y2))/2, Yp(z1)+3, name.split()[0] + ' ' + name.split()[1], 6.5, '#3d2a0c', 'middle')
    # DevKit outline (translucent so the parts below show) + its two header rows
    ydk = PL.CY - bc_ESP_DY
    b += f'<rect x="{X(ydk+31.37):.1f}" y="{Yp(bc_ESP_Z+12.7):.1f}" width="{62.74*S:.1f}" height="{25.4*S:.1f}" rx="4" fill="#1a1a1a" fill-opacity=".35" stroke="#111" stroke-width="1.5"/>'
    b += text(X(ydk + 31.37) + 6, Yp(bc_ESP_Z + 12.7) + 30, 'DevKitC-1 above (translucent)', 8.5, '#111', 'start', '700')
    used = {1: ('ESP32 3V3 (pin L1)', '3V3'), 4: ('ESP32 GPIO4 (pin L4)', 'IO4'), 5: ('ESP32 GPIO5 (pin L5)', 'IO5'), 22: ('ESP32 GND (pin L22)', 'GND')}
    for i in range(1, 23):
        yy, zl = PL.hole(i, PL.DK_ROWS[0]); _, zr = PL.hole(i, PL.DK_ROWS[1])
        b += f'<rect x="{X(yy)-4:.1f}" y="{Yp(zl)-4:.1f}" width="8" height="8" fill="#111"/><rect x="{X(yy)-4:.1f}" y="{Yp(zr)-4:.1f}" width="8" height="8" fill="#111"/>'
        if i in used:
            b += pin(m, used[i][0], X(yy), Yp(zl), 'u', None) + text(X(yy), Yp(zl) + 15, used[i][1], 7.5, '#ffd54f', 'middle', '700')
    b += text(X(PL.hole(1, PL.DK_ROWS[0])[0]) + 7, Yp(PL.hole(1, PL.DK_ROWS[0])[1]) + 3, 'L1', 8, '#fff') + text(X(PL.hole(1, PL.DK_ROWS[1])[0]) + 7, Yp(PL.hole(1, PL.DK_ROWS[1])[1]) + 3, 'R1', 8, '#fff')
    # plugs
    names = {}
    for w_ in D['wires']:
        for e in ('a', 'b'):
            if w_[e][0] == m and w_[e][1].startswith('J'): names[w_[e][1]] = 1
    lab = {'J1': ['5V', 'GND'], 'J2': ['5V', 'IO10', 'ECHO', 'GND'], 'J3': ['3V3', 'GND', 'IO9', 'IO8'], 'J4': ['5V', 'DIN', 'GND'],
           'J5': ['5V', 'GND', 'TX', 'RX', 'EYE'], 'J6': ['IO12', 'R', 'G', 'B', 'GND'], 'J7': ['VBAT', 'DOCK', 'USB'], 'J8': ['3V3', 'GND', 'IR13', 'IR14', 'IR15', 'IR16'], 'J9': ['OFF']}
    for J, (k, j, n, pstep, hh, kind) in PL.PLUGS.items():
        y0, z0 = PL.hole(k, j); y1, _ = PL.hole(k + (n - 1) * pstep, j)
        xt = kind.startswith('XT30')
        xa, xb = min(X(y0), X(y1)), max(X(y0), X(y1))
        b += f'<rect x="{xa - 6:.1f}" y="{Yp(z0) - 8:.1f}" width="{xb - xa + 12:.1f}" height="26" rx="2" fill="{"#f2c200" if xt else "#f4f4f4"}" stroke="#666"/>'
        b += text((xa + xb) / 2, Yp(z0) + 15, J + (' XT30' if xt else ''), 8, '#3d2a0c', 'middle', '700')
        for i in range(n):
            yy, _ = PL.hole(k + i * pstep, j)
            full = next((nm for nm in names if nm.startswith(f'{J}-{i+1}')), f'{J}-{i+1}')
            b += pin(m, full, X(yy), Yp(z0), 'd', None, used=full in names)
            b += f'<text x="{X(yy)+2.5:.1f}" y="{Yp(z0)+13:.1f}" font-size="6.5" style="fill:#3d2a0c" text-anchor="end" transform="rotate(-90 {X(yy)+2.5:.1f} {Yp(z0)+21:.1f})">{lab[J][i]}</text>'
    b += title(x, y - 8, "Perfboard 5 x 7 cm, 24 x 18 holes, to scale (component side): DevKit, 74AHCT125, dividers, J1-J9")
    g(m, b)


def canboard(x, y):
    m = "CAN board (SN65HVD230)"
    b = rect(x, y, 110, 150, '#1d4fa8', 4) + rect(x + 36, y + 50, 34, 22, '#222', 2) + text(x + 55, y + 88, 'SN65HVD230', 8, '#cfd8ff', 'middle')
    b += text(x + 55, y + 104, '120 Ω on board', 7.5, '#cfd8ff', 'middle')
    for i, (n, l) in enumerate([('3V3', '3V3'), ('GND', 'GND'), ('CTX', 'CTX'), ('CRX', 'CRX')]):
        b += pin(m, n, x + 10, y + 18 + i * 14, 'l', l)
    b += pin(m, 'terminal H', x + 38, y + 136, 'd', 'H', shape='term') + pin(m, 'terminal L', x + 72, y + 136, 'd', 'L', shape='term')
    b += title(x, y - 8, "CAN transceiver (Waveshare)")
    g(m, b)


def pizero(x, y):
    m = "Raspberry Pi Zero 2 W"
    w, h, k = 325, 190, 12.7
    b = rect(x, y, w, h, '#2e7d32', 14) + rect(x + 130, y + 116, 50, 40, '#222', 3) + text(x + 16, y + 176, 'Raspberry Pi Zero 2 W', 11, '#d7f0d9', 'start', '700')
    b += rect(x + 30, y + 42, 20 * k + 2, 2 * k + 4, '#111', 2, 'none')
    lbl = {2: '5V', 4: '5V', 6: 'GND', 8: 'TXD', 10: 'RXD', 12: 'BCLK', 17: '3V3', 19: 'MOSI', 35: 'WS', 38: 'DIN', 39: 'GND', 40: 'DOUT'}
    names = {n: None for n in range(1, 41)}
    for nm in D['pins'][m]:
        if nm.startswith('pin '):
            names[int(nm.split()[1])] = nm
    for n in range(1, 41):
        col = (n - 1) // 2
        px = x + 38 + col * k
        py = y + 50 + (0 if n % 2 == 0 else k)    # even pins on the outer edge
        if names[n]:
            b += pin(m, names[n], px, py, 'u', None)
            ty = py - 8 if n % 2 == 0 else py + 8
            an = 'start' if n % 2 == 0 else 'end'
            b += f'<text x="{px+2.6}" y="{ty}" font-size="8" style="fill:{'#1b5e20' if n%2==0 else '#ffeb3b'}" font-weight="700" text-anchor="{an}" transform="rotate(-90 {px+2.6} {ty})">{n} {lbl.get(n,"")}</text>'
        else:
            b += pin(m, f'p{n}', px, py, 'u', None, used=False)
    b += text(x + 26, y + 50 + k + 3, 'odd', 8, '#d7f0d9', 'end') + text(x + 26, y + 53, 'even', 8, '#d7f0d9', 'end')
    b += rect(x + w - 8, y + 48, 14, 50, '#ddd', 2, '#777') + pin(m, 'CSI connector (22-pin)', x + w + 6, y + 73, 'r', shape='none')
    b += rect(x + 210, y + h - 8, 24, 12, '#bbb', 2, '#666') + text(x + 222, y + h - 14, 'USB', 8, '#d7f0d9', 'middle') + pin(m, 'micro-USB OTG', x + 222, y + h + 4, 'd', shape='none')
    b += rect(x + 255, y + h - 8, 24, 12, '#bbb', 2, '#666') + text(x + 267, y + h - 14, 'PWR', 8, '#d7f0d9', 'middle')
    b += title(x, y - 16, m + " (40-pin header, pin 1 bottom-left)")
    g(m, b)


def junction(x, y):
    m = "I2S junction board"
    pins = [('BCLK', 'BCLK'), ('WS', 'WS'), ('DOUT', 'DOUT'), ('DIN (amp)', 'DIN'), ('3V3', '3V3'), ('5 V', '5V'), ('GND', 'GND')]
    b = rect(x, y, 30 + len(pins) * 26, 70, '#c9a26b', 3, '#7a5b2e') + f'<rect x="{x}" y="{y}" width="{30+len(pins)*26}" height="70" fill="url(#holes)"/>'
    for i, (n, l) in enumerate(pins):
        px = x + 22 + i * 26
        b += f'<rect x="{px-3}" y="{y+8}" width="6" height="56" fill="#b0b0b0" rx="1"/>' + pin(m, n, px, y + 8, 'u', l, '#3d2a0c')
    b += title(x, y - 8, "I2S junction (strip board: splits the bus to 2 mics + amp)")
    g(m, b)


def motor(x, y, side):
    m = f"Motor {side} (MF6015)"
    b = f'<circle cx="{x}" cy="{y}" r="78" fill="#2a2d33" stroke="#000"/><circle cx="{x}" cy="{y}" r="62" fill="#3a3e46"/><circle cx="{x}" cy="{y}" r="16" fill="#777"/>'
    for a in range(6):
        import math
        b += f'<circle cx="{x+40*math.cos(a*math.pi/3):.1f}" cy="{y+40*math.sin(a*math.pi/3):.1f}" r="3" fill="#111"/>'
    b += text(x, y + 104, f'LK MF6015 · {side}', 12, 'var(--fg)', 'middle', '700')
    b += text(x, y + 120, 'CAN ID ' + ('1' if side == 'L' else '2'), 10, 'var(--mut)', 'middle')
    # connector bay on top
    b += rect(x - 34, y - 98, 68, 26, '#eee', 3, '#888')
    for i, (n, l) in enumerate([('plug V+', 'V+'), ('plug V-', 'V−'), ('plug CAN-H', 'H'), ('plug CAN-L', 'L')]):
        b += pin(m, n, x - 24 + i * 16, y - 92, 'u', None) + text(x - 24 + i * 16, y - 76, l, 8, '#333', 'middle', '700')
    b += rect(x + 40, y - 92, 26, 14, '#eee', 2, '#888') + text(x + 53, y - 66, 'UART', 7, 'var(--mut)', 'middle')
    g(m, b)


# ---------------- right: devices ----------------
def armbutton(x, y):
    m = "Arm button + RGB LED"
    b = rect(x, y, 150, 80, '#555c66', 6) + f'<circle cx="{x+50}" cy="{y+40}" r="22" fill="#c62828" stroke="#600"/><circle cx="{x+50}" cy="{y+40}" r="15" fill="#e53935"/>'
    b += f'<circle cx="{x+110}" cy="{y+40}" r="11" fill="#fff" stroke="#aaa"/><circle cx="{x+110}" cy="{y+40}" r="6" fill="url(#rgb)"/>'
    b += pin(m, 'Arm button', x + 8, y + 14, 'l', shape='pad') + pin(m, 'Button + LED common', x + 8, y + 66, 'l', shape='pad')
    for i, c in enumerate('RGB'):
        b += pin(m, f'RGB LED {c}', x + 92 + i * 16, y + 76, 'd', c, '#ddd', shape='pad')
    b += text(x + 22, y + 18, 'BTN', 7.5, '#ddd') + text(x + 22, y + 70, 'COM', 7.5, '#ddd')
    b += title(x, y - 8, m)
    g(m, b)


def imu(x, y):
    m = "IMU (GY-521)"
    b = rect(x, y, 110, 112, '#1d4fa8', 4) + rect(x + 54, y + 36, 28, 28, '#222', 2) + text(x + 68, y + 82, 'MPU-6050', 8, '#cfd8ff', 'middle')
    for i, (n, l) in enumerate([('VCC', 'VCC'), ('GND', 'GND'), ('SCL', 'SCL'), ('SDA', 'SDA'), ('XDA', 'XDA'), ('XCL', 'XCL'), ('AD0', 'AD0'), ('INT', 'INT')]):
        b += pin(m, n, x + 10, y + 12 + i * 12.5, 'l', None, used=(i < 4)) + text(x + 19, y + 15.5 + i * 12.5, l, 8, '#cfd8ff')
    b += title(x, y - 8, "IMU GY-521 (MPU-6050)")
    g(m, b)


def sonar(x, y):
    m = "Sonar (HC-SR04)"
    b = rect(x, y, 220, 96, '#1d4fa8', 4)
    for cx in (x + 52, x + 168):
        b += f'<circle cx="{cx}" cy="{y+44}" r="34" fill="#cfd3d8" stroke="#888"/><circle cx="{cx}" cy="{y+44}" r="25" fill="url(#mesh)"/>'
    b += rect(x + 94, y + 8, 32, 14, '#cfd3d8', 3, '#888')
    for i, n in enumerate(['Vcc', 'Trig', 'Echo', 'Gnd']):
        b += pin(m, n, x + 80 + i * 20, y + 88, 'd', n)
    b += title(x, y - 8, "Ultrasonic sonar HC-SR04")
    g(m, b)


def eye(x, y, side, out):
    m = f"Eye {side} (8x8 WS2812)"
    b = rect(x, y, 186, 130, '#111', 4)
    for i in range(8):
        for j in range(8):
            b += f'<rect x="{x+38+i*14}" y="{y+12+j*14}" width="8" height="8" rx="1" fill="#f5f5f5" stroke="#999" stroke-width=".5"/>'
    b += pin(m, '5V', x - 2, y + 40, 'l', '5V', '#fff', shape='pad') + pin(m, 'DIN', x - 2, y + 64, 'l', 'DIN', '#fff', shape='pad') + pin(m, 'GND', x - 2, y + 88, 'l', 'GND', '#fff', shape='pad')
    if out:
        b += pin(m, '5V out', x + 188, y + 40, 'r', '5V', '#fff', shape='pad') + pin(m, 'DOUT', x + 188, y + 64, 'r', 'DO', '#fff', shape='pad') + pin(m, 'GND out', x + 188, y + 88, 'r', 'GND', '#fff', shape='pad')
    b += title(x, y - 8, f"Eye {side}: 8×8 WS2812")
    g(m, b)


def mic(x, y, side):
    m = f"Mic {side} (ICS-43434)"
    b = rect(x, y, 66, 92, '#2b2b2b', 4) + f'<rect x="{x+38}" y="{y+40}" width="14" height="12" rx="2" fill="#b0b0b0"/><circle cx="{x+45}" cy="{y+46}" r="2" fill="#333"/>'
    for i, n in enumerate(['3V', 'GND', 'BCLK', 'DOUT', 'WS', 'SEL']):
        b += pin(m, n, x + 8, y + 12 + i * 13.5, 'l', n, '#fff', shape='pad')
    b += title(x, y - 8, f"Mic {side} ICS-43434" + (" (SEL→GND)" if side == 'L' else " (SEL→3V3)"))
    g(m, b)


def amp(x, y):
    m = "Amp (MAX98357A)"
    b = rect(x, y, 100, 110, '#1d4fa8', 4) + rect(x + 46, y + 40, 22, 22, '#222', 2) + text(x + 66, y + 80, 'MAX98357A', 8, '#cfd8ff', 'middle')
    for i, n in enumerate(['LRC', 'BCLK', 'DIN', 'GAIN', 'SD', 'GND', 'Vin']):
        nm = n
        b += pin(m, nm, x + 8, y + 12 + i * 14, 'l', n, '#cfd8ff', used=n not in ('GAIN', 'SD'), shape='pad')
    b += pin(m, 'terminal +', x + 92, y + 40, 'r', '+', '#cfd8ff', shape='term') + pin(m, 'terminal -', x + 92, y + 64, 'r', '−', '#cfd8ff', shape='term')
    b += title(x, y - 8, "I2S amp MAX98357A")
    g(m, b)


def speaker(x, y):
    m = "Speaker 3 W"
    b = f'<circle cx="{x}" cy="{y}" r="44" fill="#333" stroke="#000"/><circle cx="{x}" cy="{y}" r="32" fill="#4a4a4a"/><circle cx="{x}" cy="{y}" r="12" fill="#222"/>'
    b += pin(m, '+', x - 46, y - 14, 'l', shape='pad') + pin(m, '-', x - 46, y + 14, 'l', shape='pad') + text(x - 54, y - 20, '+', 10, 'var(--fg)', 'end') + text(x - 54, y + 26, '−', 10, 'var(--fg)', 'end')
    b += title(x - 44, y - 54, "Speaker 40 mm 3 W")
    g(m, b)


def camera(x, y):
    m = "Camera Module 3"
    b = rect(x, y, 96, 100, '#2e7d32', 6) + f'<rect x="{x+28}" y="{y+36}" width="40" height="40" rx="3" fill="#222"/><circle cx="{x+48}" cy="{y+56}" r="13" fill="#0d1b3a" stroke="#555"/><circle cx="{x+44}" cy="{y+52}" r="3" fill="#6b8ccf"/>'
    b += rect(x + 18, y + 4, 60, 12, '#eee', 2, '#999') + pin(m, 'FPC connector', x + 48, y + 4, 'u', shape='none')
    b += title(x, y + 120, "Camera Module 3 Wide")
    g(m, b)


def ld19adapter(x, y):
    m = "LD19 USB adapter"
    b = rect(x, y, 110, 76, '#1d4fa8', 4) + rect(x + 14, y - 6, 22, 16, '#bbb', 2, '#666') + pin(m, 'USB', x + 25, y - 6, 'u', shape='none')
    b += rect(x + 64, y + 10, 22, 22, '#222', 2)
    for i, (n, l) in enumerate([('5V', '5V'), ('GND', 'GND'), ('RX', 'RX'), ('(n.c.)', 'n.c.')]):
        b += pin(m, n, x + 25 + i * 20, y + 68, 'd', l, '#cfd8ff')
    b += title(x, y - 8, "LD19 USB-UART adapter")
    g(m, b)


def ld19(x, y):
    m = "LD19 lidar"
    b = rect(x - 50, y - 44, 100, 88, '#1e1e1e', 10) + f'<circle cx="{x}" cy="{y}" r="34" fill="#2b2b2b" stroke="#000"/><rect x="{x-16}" y="{y-9}" width="32" height="18" rx="3" fill="#000"/><circle cx="{x-7}" cy="{y}" r="4" fill="#5c0f0f"/><circle cx="{x+7}" cy="{y}" r="4" fill="#0f2a5c"/>'
    b += rect(x - 40, y - 56, 80, 12, '#f4f4f4', 2, '#999')
    for i, n in enumerate(['P5V', 'GND', 'Tx', 'PWM']):
        b += pin(m, n, x - 30 + i * 20, y - 50, 'u', n, '#ddd')
    b += title(x + 56, y + 4, "LD19 lidar")
    g(m, b)


def headlight_driver(x, y):
    m = "Headlight driver (D36V6F12 + IRLB8721)"
    b = rect(x, y, 170, 110, '#c9a26b', 4, '#7a5b2e')
    b += rect(x + 22, y + 30, 40, 50, '#1d3f9c', 3) + text(x + 42, y + 58, 'D36V6F12', 7, '#cfd8ff', 'middle')
    b += rect(x + 86, y + 34, 46, 40, '#222', 2) + text(x + 109, y + 58, 'IRLB8721', 7, '#ddd', 'middle')
    b += pin(m, 'VIN (1 A inline fuse)', x + 10, y + 100, 'd', 'VIN') + pin(m, 'GND', x + 30, y + 100, 'd', 'GND')
    b += pin(m, 'PWM (GPIO12)', x + 140, y + 100, 'd', 'PWM') + pin(m, 'GND (PWM)', x + 160, y + 100, 'd', 'GND')
    b += pin(m, 'R+', x + 10, y + 10, 'u', 'R+') + pin(m, 'R-', x + 30, y + 10, 'u', 'R−')
    b += pin(m, 'L+', x + 140, y + 10, 'u', 'L+') + pin(m, 'L-', x + 160, y + 10, 'u', 'L−')
    b += title(x, y - 8, "Headlight driver: 6S -> 12 V, PWM switch")
    g(m, b)


def headlight(x, y, side):
    m = f"Headlight {side} (COB disc)"
    b = f'<circle cx="{x}" cy="{y}" r="34" fill="#cfd3d8" stroke="#555"/><circle cx="{x}" cy="{y}" r="26" fill="#f6e27a" stroke="#b8a040"/>'
    b += pin(m, '+', x - 14, y + 40, 'd', '+') + pin(m, '-', x + 14, y + 40, 'd', '−')
    b += title(x - 34, y - 42, f"Headlight {side}: COB disc 12 V 2.4 W")
    g(m, b)


def irrx(x, y):
    m = "IR receivers (4 x TSOP38238)"
    b = rect(x, y, 420, 90, '#2f3238', 8)
    for i, pos in enumerate(("front-left", "front-right", "back-left", "back-right")):
        cx = x + 20 + i * 100
        b += rect(cx, y + 14, 60, 40, '#151618', 4) + f'<circle cx="{cx+30}" cy="{y+30}" r="11" fill="#3a3d44" stroke="#666"/>'
        b += text(cx + 30, y + 68, pos, 9, '#ddd', 'middle')
        b += pin(m, f'{pos} OUT', cx + 30, y + 90, 'd', 'OUT', '#ddd', shape='pad')
    b += pin(m, 'front-left VS', x, y + 24, 'l', 'VS (chained)', shape='pad') + pin(m, 'back-left GND', x, y + 52, 'l', 'GND (chained)', shape='pad')
    b += title(x, y - 8, "IR receivers: 4 x TSOP38238 in hoods on the top skin (VS, GND chained; one OUT each)")
    g(m, b)


# ---------- layout ----------
battery(30, 40); bms(330, 30); pwrswitch(330, 205); powerbutton(510, 205)
pdb(30, 330); regulator(330, 340)
dock(30, 560); fusediode(180, 560); usbc(30, 720); ip2368(190, 710)
motor(150, 940, 'R'); motor(390, 940, 'L')
perfboard(590, 40); canboard(1110, 60)
pizero(600, 470); camera(1010, 740); junction(970, 480)
ld19adapter(770, 740); ld19(820, 1010)
armbutton(1330, 40); imu(1330, 170); sonar(1490, 160); eye(1340, 320, 'R', True); eye(1590, 320, 'L', False)
mic(1360, 540, 'L'); mic(1360, 680, 'R'); amp(1520, 560); speaker(1700, 640)
headlight_driver(1360, 900); headlight(1620, 870, 'L'); headlight(1620, 1040, 'R')
irrx(1180, 1150)

# ---------- wires ----------
missing = []
WI = []
for i, w in enumerate(D['wires']):
    a, b = tuple(w['a']), tuple(w['b'])
    if a not in P or b not in P:
        missing.append((a if a not in P else b)); continue
    (x1, y1, d1), (x2, y2, d2) = P[a], P[b]
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    k = min(140, 40 + dist * .28)
    c1 = (x1 + DV[d1][0] * k, y1 + DV[d1][1] * k); c2 = (x2 + DV[d2][0] * k, y2 + DV[d2][1] * k)
    d = f'M{x1:.1f} {y1:.1f} C{c1[0]:.1f} {c1[1]:.1f} {c2[0]:.1f} {c2[1]:.1f} {x2:.1f} {y2:.1f}'
    sw = max(2.2, w['d'] * 2.1)
    WI.append(f'<path class="u" data-i="{i}" d="{d}" stroke-width="{sw+1.6:.1f}"/><path class="w" data-i="{i}" d="{d}" stroke="{w["col"]}" stroke-width="{sw:.1f}"/>')
    WI.append(f'<circle class="e" data-i="{i}" cx="{x1}" cy="{y1}" r="2.6" fill="{w["col"]}" stroke="#000a"/><circle class="e" data-i="{i}" cx="{x2}" cy="{y2}" r="2.6" fill="{w["col"]}" stroke="#000a"/>')
assert not missing, missing

defs = '''<defs><pattern id="holes" width="10" height="10" patternUnits="userSpaceOnUse"><circle cx="5" cy="5" r="1.5" fill="#8a6a3a" opacity=".55"/></pattern>
<pattern id="mesh" width="4" height="4" patternUnits="userSpaceOnUse"><rect width="4" height="4" fill="#8a8f96"/><circle cx="2" cy="2" r="1" fill="#55595f"/></pattern>
<radialGradient id="rgb"><stop offset="0" stop-color="#fff"/><stop offset=".5" stop-color="#7fd"/><stop offset="1" stop-color="#c3f"/></radialGradient></defs>'''
svg = (f'<svg id="svg" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" role="img" aria-label="Wiring drawn on pictures of the boards">'
       + defs + '<rect id="bg" width="100%" height="100%" fill="transparent"/>' + ''.join(S) + '<g id="wires">' + ''.join(WI) + '</g></svg>')

tpl = open(RP.SITE / 'wiring_boards_tpl.html').read()
out = tpl.replace('__SVG__', svg).replace('__DATA__', json.dumps(D))
open(RP.SITE / 'wiring_boards.html', 'w').write(out)
print(len(D['wires']), 'wires drawn,', len(P), 'pins, ', len(out) // 1000, 'kB')
