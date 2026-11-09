"""Mutation tests: each deliberate wiring mistake must be reported as an ERROR by erc.py."""
import copy, importlib, json, sys
import netlist as N, erc as E

BASE = N.wires()

def run(mut):
    importlib.reload(N); importlib.reload(E)
    W = copy.deepcopy(BASE); mut(W)
    N.wires = lambda: W
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        E.main()
    return [f for f in E.FINDINGS if f['sev'] == 'ERROR']

def find(W, sig, harness=None):
    return next(w for w in W if w['sig'] == sig and (harness is None or w['harness'].startswith(harness)))

def swap_uart(W):
    a, b = find(W, 'ESP TX (GPIO17) -> Pi RXD'), find(W, 'Pi TXD -> ESP RX (GPIO18)')
    a['b'], b['b'] = b['b'], a['b']
def old_key(W):
    """BMS P- landed on the power switch's VOUT instead of the negative splice: switching on shorts the battery."""
    find(W, 'P-')['b'] = [N.KEY, 'VOUT (BAT+ to the PDB)']
def no_divider(W):
    find(W, 'ECHO (GPIO11 via 1k/2k)')['a'] = [N.PB, 'ESP.GPIO11']
def lose_ground(W):
    W.remove(find(W, 'GND', 'Perfboard J3'))
def can_swap(W):
    h = find(W, 'CAN-H', 'CAN bus -> motor L'); l = find(W, 'CAN-L', 'CAN bus -> motor L')
    h['b'], l['b'] = l['b'], h['b']
def mic_5v(W):
    find(W, '3V', 'I2S junction -> mic L')['a'] = [N.JUN, '5 V']
def eye_from_pi_direct(W):
    find(W, 'DIN (via 330 ohm)')['a'] = [N.PB, 'J3-4 (ESP32 GPIO8)']
def no_sense(W):
    W.remove(find(W, 'VBAT sense (switched BAT+)'))

CASES = [('UART TX/RX swapped', swap_uart), ('power switch: P- on VOUT', old_key), ('sonar echo without divider', no_divider),
         ('IMU ground missing', lose_ground), ('CAN H/L swapped at motor L', can_swap), ('mic on 5 V', mic_5v),
         ('eye data from a 3.3 V I2C pin', eye_from_pi_direct), ('VBAT sense wire missing', no_sense)]
ok = True
base = run(lambda W: None)
print(f"baseline: {len(base)} errors")
for name, m in CASES:
    errs = run(m)
    hit = bool(errs)
    ok &= hit
    print(f"{'caught' if hit else 'MISSED':7s} {name}: " + (errs[0]['msg'][:110] if errs else ''))
sys.exit(0 if ok and not base else 1)
