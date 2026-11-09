"""CAN bus load / timing and ground offset between the ESP32's CAN transceiver and the motors' CAN reference.
Lengths and gauges come from the harness model (case/wiring_data.json)."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(HERE, os.pardir, "cad_src", "case"), "/home/claude/case"):
    if os.path.isdir(_p): sys.path.append(_p)       # robot_paths: this machine's layout
import robot_paths as RP
WD = json.load(open(RP.artifact("wiring_data.json")))
OHM_PER_M = {12: 0.00521, 16: 0.0132, 20: 0.0333, 22: 0.0530, 24: 0.0842, 26: 0.134}
def wire(h, sig):
    for x in WD:
        if x["name"] == h:
            for c in x["conds"]:
                if c["sig"] == sig: return c["L"] / 100 * OHM_PER_M[c["awg"]], c["L"], c["awg"]
    raise KeyError((h, sig))
out = {}
# ---- CAN timing (1 Mbit/s, standard 11-bit frames, 8 data bytes) ----
BIT = 1e-6
def frame_bits(n=8, worst=True):
    base = 47 + 8 * n                       # SOF..EOF incl. CRC delimiter, ACK, EOF (no IFS)
    stuff = (34 + 8 * n - 1) // 4 if worst else 0
    return base + stuff + 3                 # + intermission
CYCLE = 0.002                               # control period (500 Hz)
f = frame_bits()
on = 3 * f * BIT                            # 0x280 torque frame + 2 replies
off = 4 * f * BIT                           # 2 x 0x9C + 2 replies (motors off)
out["can"] = dict(frame_us_worst=round(f * BIT * 1e6), per_cycle_on_us=round(on * 1e6), load_on_pct=round(100 * on / CYCLE),
                  per_cycle_off_us=round(off * 1e6), load_off_pct=round(100 * off / CYCLE),
                  retransmits_that_fit_in_a_cycle=int((CYCLE - on) / ((frame_bits() + 23) * BIT)),
                  bus_off_recovery_ms=round(128 * 11 * BIT * 1e3, 2), can_timeout_ms=20,
                  note="Replies are read at the start of the next cycle (1 cycle of latency, modelled in the sim). "
                       "A bus-off costs ~1.4 ms + one cycle: below the 20 ms CAN timeout, so the robot keeps balancing.")
# ---- ground offset: transceiver GND vs motor V- (the motors' CAN reference; the bus has no ground wire) ----
I_MOTOR_PEAK, I_5V_PEAK, I_REG_IN = 16.5, 4.66, 1.2
r_mot = wire("PDB -> motor L power", "V-")[0]
r_j1 = wire("5 V regulator -> perfboard J1", "GND")[0]
r_reg = wire("PDB -> 5 V regulator", "GND")[0]
R_CONTACTS = 0.002                          # XT30 + solder joints + PDB copper (estimate)
v_motor = I_MOTOR_PEAK * (r_mot + 0.0005)
v_board = I_5V_PEAK * (r_j1 + R_CONTACTS) + (I_REG_IN + I_5V_PEAK) * r_reg
out["ground"] = dict(motor_return_mOhm=round(r_mot * 1e3, 2), perfboard_gnd_mOhm=round(r_j1 * 1e3, 2),
                     worst_offset_mV=round((v_motor + v_board) * 1e3), transceiver_tolerance="-2 V ... +7 V common mode (SN65HVD230)",
                     margin_x=round(2.0 / (v_motor + v_board)))
print(json.dumps(out, indent=1))
json.dump(out, open("can_ground_report.json", "w"), indent=1)
