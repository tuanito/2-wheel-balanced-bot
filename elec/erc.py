"""Electrical rules check of the robot wiring. Run: python3 erc.py  -> erc_report.json + erc_report.md + robot.net

Model: netlist.py (every harness wire from case/wiring.py + board internals). Checks:
  1 shorts between supplies / supply and ground          6 firmware + Pi pin maps vs the wiring
  2 one common ground                                     7 CAN bus: pair, nodes, termination
  3 every power input supplied, in its voltage range      8 current per wire and per connector contact, rail budgets
  4 logic nets: a driver, no two drivers, no input over its maximum, high level above the input threshold
  5 dividers solved (sonar echo, sense inputs)            9 fuses vs the wires they protect
"""
import json, re, os, sys, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
for _p in (os.path.join(HERE, os.pardir, "cad_src", "case"), "/home/claude/case"):
    if os.path.isdir(_p): sys.path.append(_p)       # robot_paths: this machine's layout
import robot_paths as RP
import netlist as N

LEVEL = {'3V3': 3.3, '5V': 5.0}
FINDINGS = []
UNRATED = []


def F(sev, check, msg, fix=''):
    FINDINGS.append(dict(sev=sev, check=check, msg=msg, fix=fix))


class UF:
    def __init__(s): s.p = {}
    def f(s, x):
        s.p.setdefault(x, x)
        while s.p[x] != x:
            s.p[x] = s.p[s.p[x]]; x = s.p[x]
        return x
    def u(s, a, b): s.p[s.f(a)] = s.f(b)


def node(m, p): return f"{m}::{p}"


def build(wires, key_in=True, fixed_key=False, button=False):
    """Return (uf, edges) where edges are (a, b, kind, ref) for path finding."""
    uf, edges = UF(), []
    for n in N.PINS: uf.f(n)
    for a, b, why in N.LINKS:
        uf.u(a, b); edges.append((a, b, 'link', why))
    for i, w in enumerate(wires):
        a, b = node(*w['a']), node(*w['b'])
        uf.u(a, b); edges.append((a, b, 'wire', i))
    if key_in:
        a, b, why = N.KEY_BRIDGE
        if fixed_key:
            a, b = node(N.KEY, 'contact A'), node(N.KEY, 'contact B')
        uf.u(a, b); edges.append((a, b, 'key', why))
    if button:
        uf.u(node(N.BTN, 'BTN.closed'), node(N.BTN, 'Button + LED common'))
    return uf, edges


def nets_of(uf):
    nets = collections.defaultdict(list)
    for n in list(uf.p):
        nets[uf.f(n)].append(n)
    return nets


def attrs(n): return N.PINS.get(n, {})


def fix_key(wires):
    """The XT60 loop-key wiring as it must be: contact A = BAT+ from the fuse, contact B = BAT+ to the PDB,
    the two negatives spliced next to the socket (the key only interrupts the positive)."""
    out = []
    for w in wires:
        w = dict(w); a, b = list(w['a']), list(w['b'])
        if b == [N.KEY, 'panel +'] and w['harness'].startswith('BMS'): b = [N.KEY, 'contact A']
        if a == [N.KEY, 'panel +'] and w['harness'].startswith('XT60'): a = [N.KEY, 'contact B']
        if b == [N.KEY, 'panel -']: b = [N.KEY, 'P- splice']
        if a == [N.KEY, 'panel -']: a = [N.KEY, 'P- splice']
        w['a'], w['b'] = a, b; out.append(w)
    for n in ('contact A', 'contact B', 'P- splice'):
        N.PINS.setdefault(node(N.KEY, n), dict(kind='pass'))
    return out


# ---------------------------------------------------------------- checks
def check_supplies(uf, tag):
    nets = nets_of(uf)
    bad = []
    for r, ns in nets.items():
        srcs = [n for n in ns if attrs(n).get('kind') == 'src']
        gnds = [n for n in ns if attrs(n).get('kind') == 'gnd']
        if srcs and gnds:
            bad.append((ns, f"supply {', '.join(srcs)} is tied to ground {', '.join(gnds[:3])}"))
        vs = {round(attrs(n)['V'], 1) for n in srcs if not attrs(n).get('charger')}
        if len(vs) > 1:
            bad.append((ns, f"supplies at different voltages on one net: {srcs}"))
    return bad


def ground_net(uf):
    return uf.f(node(N.BAT, 'lead -'))


def net_voltage(uf, root, nets):
    """DC voltage of a net: a supply on it, or through a diode from a supplied net."""
    for n in nets[root]:
        a = attrs(n)
        if a.get('kind') == 'src': return a['V']
        if a.get('kind') == 'gnd': return 0.0
    for an, ca, vf, _ in N.DIODES:
        if uf.f(ca) == root:
            ra = uf.f(an)
            if ra != root:
                for n in nets[ra]:
                    if attrs(n).get('kind') == 'src': return attrs(n)['V'] - vf
    return None


def module(n): return n.split('::')[0]


def solve_levels(uf, nets):
    """Logic/DC level of every net: drivers fix a net; resistor-only nets are solved by nodal analysis."""
    fixed = {}
    for r, ns in nets.items():
        v = net_voltage(uf, r, nets)
        if v is not None: fixed[r] = v; continue
        outs = [LEVEL[attrs(n)['level']] for n in ns if attrs(n).get('kind') == 'out' and attrs(n).get('level')]
        outs += [LEVEL[attrs(n)['level']] for n in ns if attrs(n).get('role') == 'out']
        if outs: fixed[r] = max(outs)
    # resistor network
    unk = sorted({uf.f(x) for a, b, _, _ in N.RES for x in (a, b)} - set(fixed))
    idx = {u: i for i, u in enumerate(unk)}
    import numpy as np
    G = np.zeros((len(unk), len(unk))); I = np.zeros(len(unk))
    for a, b, ohm, _ in N.RES:
        ra, rb, g = uf.f(a), uf.f(b), 1.0 / ohm
        for x, y in ((ra, rb), (rb, ra)):
            if x in idx:
                G[idx[x], idx[x]] += g
                if y in idx: G[idx[x], idx[y]] -= g
                else: I[idx[x]] += g * fixed[y]
    levels = dict(fixed)
    floating = set()
    for u in unk:
        if G[idx[u], idx[u]] == 0: floating.add(u)
    if unk:
        keep = [u for u in unk if u not in floating]
        k = [idx[u] for u in keep]
        if k:
            v = np.linalg.lstsq(G[np.ix_(k, k)], I[k], rcond=None)[0]
            for u, x in zip(keep, v): levels[u] = float(x)
    return levels


def has_res_path_to_fixed(uf, root, levels):
    return any(uf.f(a) == root or uf.f(b) == root for a, b, _, _ in N.RES)


def check_power_inputs(uf, nets, levels):
    g = ground_net(uf)
    rows = []
    for n, a in N.PINS.items():
        if a.get('kind') != 'pin': continue
        r = uf.f(n); v = net_voltage(uf, r, nets)
        mod = module(n)
        gnd_ok = any(uf.f(p) == g for p, aa in N.PINS.items() if aa.get('kind') == 'gnd' and module(p) == mod)
        lo, hi = a['range']
        rows.append((n, v, a['range'], gnd_ok))
        if v is None:
            F('ERROR', 'power inputs', f"{n} ({a.get('label','')}) has no supply.")
        elif not (lo <= v <= hi):
            F('ERROR', 'power inputs', f"{n}: {v:.1f} V outside its {lo}-{hi} V range.")
        if not gnd_ok:
            F('ERROR', 'power inputs', f"{mod}: no ground pin on the common ground.")
    return rows


def check_logic(uf, nets, levels):
    rows = []
    for r, ns in nets.items():
        ins = [n for n in ns if attrs(n).get('kind') in ('in', 'io') and attrs(n).get('role') not in ('out',)]
        if not ins: continue
        drivers = [n for n in ns if attrs(n).get('kind') == 'out' or attrs(n).get('role') == 'out' or attrs(n).get('kind') in ('src', 'gnd')]
        pulls = [n for n in ns if attrs(n).get('pull') or attrs(n).get('role') == 'in_pullup']
        pushpull = [n for n in ns if (attrs(n).get('kind') == 'out' and not attrs(n).get('tristate')) or attrs(n).get('role') == 'out']
        v = levels.get(r)
        if len(pushpull) > 1:
            F('ERROR', 'logic', f"two outputs drive the same net: {pushpull}")
        if not drivers and not pulls and not has_res_path_to_fixed(uf, r, levels):
            sev = 'ERROR' if any(attrs(n).get('role') in ('in', 'adc') for n in ins) else 'WARN'
            F(sev, 'logic', f"input with nothing driving it: {', '.join(ins)}")
        for n in ins:
            a = attrs(n)
            if v is not None and a.get('vmax') is not None and v > a['vmax'] + 1e-6:
                F('ERROR', 'levels', f"{n} sees {v:.2f} V, its maximum is {a['vmax']} V.")
            th = a.get('vih') or (a.get('vih_frac') * 5.0 if a.get('vih_frac') else None)
            if th and v is not None and 0 < v < th and drivers:
                F('ERROR', 'levels', f"{n}: high level {v:.2f} V is below its {th:.2f} V threshold.")
        rows.append(dict(net=sorted(ns), drivers=drivers, v=None if v is None else round(v, 2)))
    return rows


def device_ends(uf, nets, n, through_res=True):
    """Pins of other modules reachable from n (same net, or one resistor away)."""
    roots = {uf.f(n)}
    if through_res:
        for a, b, _, _ in N.RES:
            if uf.f(a) in roots: roots.add(uf.f(b))
            elif uf.f(b) in roots: roots.add(uf.f(a))
    return {p for r in roots for p in nets[r] if module(p) != module(n) or '.' in p.split('::')[1][:4]}


def check_firmware_map(uf, nets):
    cfg = open(RP.FIRMWARE_CONFIG).read()
    pins = dict((k, int(v)) for k, v in re.findall(r'constexpr int (PIN_\w+)\s*=\s*(\d+)', cfg))
    expect = {'PIN_CAN_TX': node(N.CAN, 'CTX'), 'PIN_CAN_RX': node(N.CAN, 'CRX'), 'PIN_I2C_SDA': node(N.IMU, 'SDA'),
              'PIN_I2C_SCL': node(N.IMU, 'SCL'), 'PIN_PI_TX': node(N.PI, 'pin 10 (GPIO15 RXD)'), 'PIN_PI_RX': node(N.PI, 'pin 8 (GPIO14 TXD)'),
              'PIN_SONAR_TRIG': node(N.SON, 'Trig'), 'PIN_SONAR_ECHO': node(N.SON, 'Echo'), 'PIN_ARM_BTN': node(N.BTN, 'Arm button'),
              'PIN_LED_R': node(N.BTN, 'RGB LED R'), 'PIN_LED_G': node(N.BTN, 'RGB LED G'), 'PIN_LED_B': node(N.BTN, 'RGB LED B'),
              'PIN_PWR_OFF': node(N.KEY, 'OFF'), 'PIN_IR_FL': node(N.IRR, 'front-left OUT'), 'PIN_IR_FR': node(N.IRR, 'front-right OUT'),
              'PIN_IR_BL': node(N.IRR, 'back-left OUT'), 'PIN_IR_BR': node(N.IRR, 'back-right OUT'),
              'PIN_VBAT': 'battery +', 'PIN_DOCK_SENSE': 'dock pad + (before the diode)', 'PIN_USB_SENSE': 'IP2368 VBUS'}
    rows = []
    for k, g in sorted(pins.items(), key=lambda x: x[1]):
        n = node(N.PB, f"ESP.GPIO{g}")
        ends = device_ends(uf, nets, n)
        # follow the divider for the echo input
        want = expect.get(k)
        ok = want in ends if want and '::' in want else False
        if want and '::' not in want:          # sense inputs: the divider top must reach an off-board wire
            src = node(N.PB, {'PIN_VBAT': 'SENSE_VBAT', 'PIN_DOCK_SENSE': 'SENSE_DOCK', 'PIN_USB_SENSE': 'SENSE_USB'}[k])
            wired = [p for p in nets[uf.f(src)] if module(p) != N.PB]
            ok = bool(wired)
            if not ok:
                F('ERROR', 'firmware pin map', f"GPIO{g} ({k}) reads the {want} through a divider, but no wire brings the {want} to the perfboard: "
                  f"the ADC reads 0 V.", f"Add a 26 AWG sense wire from the {want} to the divider input on the perfboard.")
        elif not ok:
            F('ERROR', 'firmware pin map', f"GPIO{g} ({k}) should reach {want}; it reaches {sorted(ends)}.")
        rows.append(dict(pin=k, gpio=g, expect=want, ok=ok))
    strap = {0, 3, 45, 46}; usb = {19, 20}; psram = {35, 36, 37}
    for k, g in pins.items():
        if g in strap: F('WARN', 'firmware pin map', f"{k} uses strapping pin GPIO{g}.")
        if g in usb: F('WARN', 'firmware pin map', f"{k} uses USB pin GPIO{g}.")
        if g in psram: F('ERROR', 'firmware pin map', f"{k} uses GPIO{g}, taken by the octal PSRAM on the N8R8 module.")
    return rows


def check_pi_map(uf, nets):
    inst = open(RP.PI_INSTALL).read()
    rows = []
    def chk(name, cond, msg, fix=''):
        rows.append(dict(item=name, ok=bool(cond)))
        if not cond: F('ERROR', 'Pi pin map', msg, fix)
    chk('UART on GPIO14/15 (PL011, console off)', 'do_serial_hw 0' in inst and 'do_serial_cons 1' in inst and 'miniuart-bt' in inst, 'Pi UART not set up')
    chk('SPI for the eyes', 'do_spi 0' in inst, 'SPI not enabled')
    chk('I2S overlay', 'googlevoicehat-soundcard' in inst, 'no I2S overlay')
    pairs = [(node(N.PI, 'pin 8 (GPIO14 TXD)'), node(N.PB, 'ESP.GPIO18'), 'Pi TXD -> ESP32 RX'),
             (node(N.PI, 'pin 10 (GPIO15 RXD)'), node(N.PB, 'ESP.GPIO17'), 'ESP32 TX -> Pi RXD'),
             (node(N.PI, 'pin 19 (GPIO10 MOSI)'), node(N.PB, 'U1.1A'), 'Pi MOSI -> 74AHCT125'),
             (node(N.PI, 'pin 12'), node(N.MICL, 'BCLK'), 'BCLK -> mic L'), (node(N.PI, 'pin 12'), node(N.MICR, 'BCLK'), 'BCLK -> mic R'),
             (node(N.PI, 'pin 12'), node(N.AMP, 'BCLK'), 'BCLK -> amp'), (node(N.PI, 'pin 35'), node(N.AMP, 'LRC'), 'LRCLK -> amp'),
             (node(N.PI, 'pin 35'), node(N.MICL, 'WS'), 'LRCLK -> mics'), (node(N.PI, 'pin 38'), node(N.MICL, 'DOUT'), 'mic data -> PCM_DIN'),
             (node(N.PI, 'pin 38'), node(N.MICR, 'DOUT'), 'mic R data -> PCM_DIN'), (node(N.PI, 'pin 40'), node(N.AMP, 'DIN'), 'PCM_DOUT -> amp'),
             (node(N.MICL, 'SEL'), node(N.BAT, 'lead -'), 'mic L SEL = GND (left channel)'), (node(N.MICR, 'SEL'), node(N.PI, 'pin 17 (3V3)'), 'mic R SEL = 3V3 (right)'),
             (node(N.PB, 'J4-2 (74AHCT125 out)'), node(N.EYR, 'DIN'), 'level shifter -> eye R'), (node(N.EYR, 'DOUT'), node(N.EYL, 'DIN'), 'eye R -> eye L')]
    for a, b, what in pairs:
        chk(what, uf.f(a) == uf.f(b), f"{what}: not connected ({a} / {b}).")
    return rows


def check_can(uf, nets):
    H, Lw = uf.f(node(N.CAN, 'terminal H')), uf.f(node(N.CAN, 'terminal L'))
    rows = dict(H=sorted(nets[H]), L=sorted(nets[Lw]))
    if H == Lw: F('ERROR', 'CAN', 'CAN-H and CAN-L are the same net.')
    for M in (N.ML, N.MR):
        if uf.f(node(M, 'plug CAN-H')) != H: F('ERROR', 'CAN', f"{M} CAN-H is not on the bus H line.")
        if uf.f(node(M, 'plug CAN-L')) != Lw: F('ERROR', 'CAN', f"{M} CAN-L is not on the bus L line.")
    terms = [n for n in nets[H] if attrs(n).get('term')]
    F('CHECK', 'CAN', f"Termination: 120 ohm on the CAN board ({len(terms)} known); the LK motors' built-in termination is not documented. "
      "The bus needs 120 ohm at each end (60 ohm between H and L).",
      "Power off, measure H-L at the motor L plug: ~60 ohm OK; ~120 ohm -> add 120 ohm across H/L there; ~40 ohm -> one motor's terminator must be switched off.")
    return rows


def path(edges, uf, a, b):
    """Shortest path a->b over edges inside one net (BFS). Returns the list of edges."""
    adj = collections.defaultdict(list)
    for e in edges:
        adj[e[0]].append((e[1], e)); adj[e[1]].append((e[0], e))
    prev = {a: None}; q = collections.deque([a])
    while q:
        x = q.popleft()
        if x == b: break
        for y, e in adj[x]:
            if y not in prev: prev[y] = (x, e); q.append(y)
    if b not in prev: return None
    out, x = [], b
    while prev[x]: x, e = prev[x]; out.append(e)
    return out


def check_currents(uf, nets, edges, wires):
    I = collections.Counter()           # wire index -> A
    budget = collections.Counter()      # src -> A
    loads = [(n, a) for n, a in N.PINS.items() if a.get('kind') == 'pin' and a.get('I')]
    srcs = {uf.f(n): n for n, a in N.PINS.items() if a.get('kind') == 'src' and not a.get('charger')}
    # regulator input from its output load
    def gnd_of(mod):
        for p, a in N.PINS.items():
            if module(p) == mod and a.get('kind') == 'gnd': return p
    def flow(n, cur, src):
        p = path(edges, uf, n, src)
        if p is None: return
        for e in p:
            if e[2] == 'wire': I[e[3]] += cur
        g1, g2 = gnd_of(module(n)), gnd_of(module(src))
        if g1 and g2:
            for e in path(edges, uf, g1, g2) or []:
                if e[2] == 'wire': I[e[3]] += cur
    order = sorted(loads, key=lambda x: 0 if module(x[0]) != N.REG else 1)
    for n, a in order:
        r = uf.f(n); s = srcs.get(r)
        if not s: continue
        flow(n, a['I'], s); budget[s] += a['I']
    # 3.3 V add-ons are already inside ESP.5V / PI.5V figures; the regulator draws its 5 V output power from the battery
    p5 = budget[node(N.REG, 'VOUT')] * 5.0 / 0.90 / N.PINS[node(N.BAT, 'lead +')]['Vmin']
    s_ = srcs.get(uf.f(node(N.REG, 'VIN')))
    if s_: flow(node(N.REG, 'VIN'), p5, s_)
    budget[node(N.BAT, 'lead +')] += p5
    p12 = budget[node(N.DRV, 'DRV.12V')] * 12.0 / 0.90 / N.PINS[node(N.BAT, 'lead +')]['Vmin']     # headlight buck
    s_ = srcs.get(uf.f(node(N.DRV, 'VIN (1 A inline fuse)')))
    if s_: flow(node(N.DRV, 'VIN (1 A inline fuse)'), p12, s_)
    budget[node(N.BAT, 'lead +')] += p12
    rows = []
    for s, a in budget.items():
        lim = N.PINS[s]['Imax']
        rows.append(dict(src=s, label=N.PINS[s].get('label', ''), A=round(a, 2), max=lim, pct=round(100 * a / lim)))
        if a > lim: F('ERROR', 'current', f"{N.PINS[s].get('label', s)}: {a:.2f} A worst case > {lim} A.")
        elif a > 0.8 * lim: F('WARN', 'current', f"{N.PINS[s].get('label', s)}: {a:.2f} A worst case = {100*a/lim:.0f} % of {lim} A.")
    wrows = []
    for i, w in enumerate(wires):
        cur = I.get(i, 0.0)
        amp = round(N.AWG_A.get(w['awg'], 99) * N.SILICONE, 1)
        cons = []
        for lbl in (w['a_lbl'], w['b_lbl']):
            for pre, (cn, ca) in N.CONN.items():
                if lbl.startswith(pre): cons.append((cn, ca, lbl)); break
        for pre, (cn, ca) in getattr(N, 'INLINE', {}).items():
            if w['harness'].startswith(pre): cons.append((cn, ca, f"{w['harness']} / {w['sig']} (inline)"))
        wrows.append(dict(i=i, harness=w['harness'], sig=w['sig'], awg=w['awg'], A=round(cur, 2), awg_A=amp,
                          conn=[f"{c[0]} ({c[1]} A)" for c in cons]))
        if cur > amp:
            F('ERROR', 'current', f"{w['harness']} / {w['sig']}: {cur:.1f} A in {w['awg']} AWG (rated ~{amp} A).")
        for cn, ca, lbl in cons:
            if ca is None:
                if cur > 3: UNRATED.append((lbl, cur, cn))
                continue
            if cur > ca:
                F('ERROR', 'current', f"{lbl}: {cur:.1f} A through one {cn} contact (rated {ca} A).",
                  'Use a connector rated for the current (JST-VH 10 A, XT30) or solder the wire.')
            elif cur > 0.8 * ca and ca < 50:
                F('WARN', 'current', f"{lbl}: {cur:.1f} A through one {cn} contact ({100*cur/ca:.0f} % of {ca} A).")
    seen = set()
    for lbl, cur, cn in UNRATED:
        k = lbl.split(' plug')[0]
        if k in seen: continue
        seen.add(k)
        F('CHECK', 'current', f"{k} plug: {cur:.1f} A peak on V+/V- (stand-up, from the sim) through the {cn}.",
          "Check the LK cable set's contact rating; it must take ~10 A for a second.")
    return rows, wrows, I


def check_fuses(wires, I):
    rows = []
    for harness, rating, sig in N.FUSES:
        idx = [i for i, w in enumerate(wires) if w['harness'] == harness and w['sig'] == sig]
        if not idx: F('ERROR', 'fuses', f"fuse '{harness}' not found in the wiring."); continue
        # wires carrying the fused current: every wire with current on the same rail downstream
        own = {h for h, _, _ in N.FUSES if h != harness}
        down = [i for i, w in enumerate(wires) if I.get(i, 0) > 0 and w['harness'] not in own and (w['harness'] == harness or
                (rating == 30 and (w['harness'].startswith('XT60') or w['harness'].startswith('PDB'))))]
        under = collections.OrderedDict()
        for i in (down or idx):
            wa = round(N.AWG_A.get(wires[i]['awg'], 99) * N.SILICONE)
            if wa < rating: under.setdefault(wires[i]['harness'], (wires[i]['awg'], wa, I.get(i, 0)))
        rows.append(dict(fuse=f"{rating:.0f} A", harness=harness, under=[f"{h} ({g} AWG ~{a} A)" for h, (g, a, _) in under.items()]))
        big = {h: v for h, v in under.items() if v[2] > 3}
        small = {h: v for h, v in under.items() if v[2] <= 3}
        if big:
            F('WARN', 'fuses', f"{rating:.0f} A fuse ahead of " + ", ".join(f"{h} ({g} AWG silicone, ~{a} A)" for h, (g, a, _) in big.items()) + ".",
              "Load is ~18 A for < 1 s at stand-up (sim), so the wire is fine in use; for full protection use 14 AWG or a 25 A fuse.")
        for h, (g, a, cur) in small.items():
            F('ERROR', 'fuses', f"{h}: {g} AWG (~{a} A) carries {cur:.1f} A but is only protected by the {rating:.0f} A main fuse: a short there "
              "overheats the wire before the fuse blows.", f"Add an inline fuse at the start of '{h}' (3 A for the regulator feed).")
    return rows


def check_misc(uf, nets):
    # status LED series resistors
    for c, g in (('G', 39), ('B', 40)):
        if not any(node(N.PB, f'ESP.GPIO{g}') in (a, b) for a, b, _, _ in N.RES):
            F('WARN', 'logic', f"Status LED {c} (GPIO{g}) has no series resistor in the wiring/labels (R has 330 ohm).",
              f"Put a resistor on GPIO{g} too (~100-330 ohm), or the LED pulls the pin to its 40 mA limit.")
    # key off must cut the load side
    F('INFO', 'power', "The Pi is fed through its GPIO 5 V pins (J5 -> pin 2): this bypasses the Pi's USB input protection. "
      "Fine with the regulated 5 V rail; never plug a USB supply into the Pi's PWR port at the same time.")
    F('INFO', 'power', "MAX98357A GAIN and SD are left open: 9 dB gain, SD pulled up on the Adafruit board -> (L+R)/2 mix. Matches a mono speaker.")
    F('INFO', 'power', "Dock charger and IP2368 both feed BMS P+; they can charge at the same time (each is a current-limited CC/CV source).")


def kicad_net(uf, nets, path_out):
    used = [r for r, ns in nets.items() if len(ns) > 1]
    comps = sorted({module(n) for r in used for n in nets[r]})
    ref = {m: f"U{i+1}" for i, m in enumerate(comps)}
    q = lambda s: '"' + s.replace('"', "'") + '"'
    out = ['(export (version "E")', '  (design (source "robot/elec/erc.py") (tool "erc.py"))', '  (components']
    for m in comps:
        out.append(f'    (comp (ref "{ref[m]}") (value {q(m)}))')
    out.append('  )')
    out.append('  (nets')
    for k, r in enumerate(sorted(used, key=lambda r: sorted(nets[r])[0])):
        name = next((n.split('::')[1] for n in nets[r] if attrs(n).get('kind') in ('src', 'gnd')), sorted(nets[r])[0].split('::')[1])
        out.append(f'    (net (code "{k+1}") (name {q("/" + name)})')
        for n in sorted(nets[r]):
            out.append(f'      (node (ref "{ref[module(n)]}") (pin {q(n.split("::")[1])}))')
        out.append('    )')
    out.append('  )')
    out.append(')')
    open(path_out, 'w').write('\n'.join(out))
    return ref


def main():
    wires0 = N.wires()
    unknown = sorted({node(*w[e]) for w in wires0 for e in ('a', 'b')} - set(N.PINS))
    for u in unknown:
        F('ERROR', 'model', f"wire end not modelled: {u}")
    # 1) the power switch as wired in the CAD
    uf, _ = build(wires0, key_in=True)
    sh = check_supplies(uf, 'as wired')
    key_short = any(node(N.BAT, 'lead +') in ns for ns, _ in sh)
    if key_short:
        F('ERROR', 'shorts', "Power switch: BMS P+ and P- both land on the switch's power path (VIN and VOUT). "
          "Switching it on would short the battery (the 30 A fuse blows).",
          "VIN = BAT+ from the 30 A fuse, VOUT = BAT+ to the PDB; splice the two negatives (BMS P- and PDB in-) "
          "beside the board, with a thin tap to its GND.")
    else:
        for ns, why in sh:
            F('ERROR', 'shorts', why)
    # all further checks on the corrected key wiring (or the original if it was already right)
    wires = fix_key(wires0) if key_short else wires0
    uf, edges = build(wires, key_in=True, fixed_key=key_short)
    for ns, why in check_supplies(uf, 'fixed'):
        F('ERROR', 'shorts', why)
    nets = nets_of(uf)
    # key out: nothing after the key may be powered
    uf_off, _ = build(wires, key_in=False, fixed_key=key_short)
    nets_off = nets_of(uf_off)
    if net_voltage(uf_off, uf_off.f(node(N.PDB, 'in +')), nets_off) not in (None,):
        F('ERROR', 'power', 'With the key out, the PDB is still powered.')
    # ground
    g = ground_net(uf)
    gnds = [n for n, a in N.PINS.items() if a.get('kind') == 'gnd']
    off = [n for n in gnds if uf.f(n) != g]
    for n in off:
        F('ERROR', 'ground', f"{n} is not on the common ground.")
    levels = solve_levels(uf, nets)
    pw = check_power_inputs(uf, nets, levels)
    lg = check_logic(uf, nets, levels)
    fw = check_firmware_map(uf, nets)
    pm = check_pi_map(uf, nets)
    cn = check_can(uf, nets)
    rails, wcur, I = check_currents(uf, nets, edges, wires)
    fu = check_fuses(wires, I)
    check_misc(uf, nets)
    echo = levels.get(uf.f(node(N.PB, 'ECHO_DIV')))
    ref = kicad_net(uf, nets, os.path.join(N.HERE, 'robot.net'))
    rep = dict(findings=FINDINGS, rails=rails, wires=wcur, power_inputs=[dict(pin=n, V=v, range=r, gnd=g_) for n, v, r, g_ in pw],
               firmware=fw, pi=pm, can=cn, fuses=fu, echo_at_gpio11=echo, nets=len([1 for ns in nets.values() if len(ns) > 1]),
               wires_n=len(wires), pins_n=len(N.PINS), key_fix_applied=key_short)
    json.dump(rep, open(os.path.join(N.HERE, 'erc_report.json'), 'w'), indent=1)
    sev = collections.Counter(f['sev'] for f in FINDINGS)
    print(f"{len(wires)} wires, {len(N.PINS)} pins, {rep['nets']} nets | " + ", ".join(f"{k} {v}" for k, v in sev.items()))
    for f in sorted(FINDINGS, key=lambda f: ['ERROR', 'WARN', 'CHECK', 'INFO'].index(f['sev'])):
        print(f"[{f['sev']}] {f['check']}: {f['msg']}" + (f"\n        fix: {f['fix']}" if f['fix'] else ''))
    print('rails:', [(r['label'], r['A'], r['max']) for r in rails])
    print('echo at GPIO11: %.2f V' % echo if echo is not None else 'echo ?')
    top = sorted(wcur, key=lambda r: -r['A'])[:12]
    for r in top: print(f"  {r['A']:6.2f} A  {r['awg']} AWG  {r['harness']} / {r['sig']}  {r['conn']}")


if __name__ == '__main__':
    main()
