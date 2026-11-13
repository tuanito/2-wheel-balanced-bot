"""The two hub motors, as ONE definition each.

Before this file the motor interface was spelled out in four places that were not linked to each
other - body_case.py's constants, wheel_adapter/motor_mount.py, wheel_adapter/wheel_adapter.py, and
the bare `+29` in assemble.py - plus the plug envelope in three more. Changing motor meant finding
all seven and hoping. Same reason wiring.harness_record() and FloorWatch exist: one definition.

Mechanical numbers are MEASURED off each motor's STEP and cross-checked against the manufacturer's
2D drawing, not copied from a shop listing. Electrical numbers come from the manufacturer parameter
sheet. Where the two disagree the STEP wins for geometry and the sheet wins for electricals - the
L-7015's drawing title block says 0.26 kg, but that is the (simplified, hollow) CAD model's mass;
the parameter sheet's 360 g is the real part.

  MF6015-V2  the fitted motor. Ø58 x 34, with a Ø49 x 8 rear boss that the wall pocket locates on
             and the connector bay on top of that boss, so the leads exit AXIALLY.
  MF7015-V2  the candidate (= MyActuator RMD-L-7015). Ø69 x 33.9, a plain cylinder with flat faces
             and NO rear boss, connector recessed in the side rim, so the leads exit RADIALLY and
             the wall has to provide its own register.

Both are all-in-one units: motor + FOC driver + encoder behind one connector. The driver variant is
chosen at purchase and is baked into the part number, which is why `drive` below records it - the
MyActuator "-200-C" build carries an MC200 (12-24 V, V3 protocol) and would be over-volted by a 6S
pack at full charge, while LKMTECH's build carries a DF70V2 (7.4-32 V, LK V2 protocol) and is the
one this robot can use.
"""

MOTORS = {
    "mf6015": dict(
        name="LKMTECH MF6015-V2 15T",
        # ---- envelope (STEP bbox: x 34.00, y 58.00, z 58.00) ----
        od=58.0, length=34.0,
        # ---- body-wall interface ----
        boss_d=49.0,            # Ø49 x 8 rear centring boss
        pocket_d=49.4,          # the wall counterbore it seats in (0.4 clearance)
        pocket_dep=2.0,
        bolt_sq=20.0,           # 4 x M2.5 on a 20 x 20 square, 8 mm deep
        bolt_clear=2.9,         # M2.5 clearance
        bolt_name="M2.5", bolt_depth=8.0,
        # ---- the connector bay: on TOP of the rear boss, leads exit axially ----
        bay="axial", bay_w=15.0, slot_w=24.0, slot_z=(17.0, 43.0), plug_h=10.5,
        # ---- output face (the wheel adapter bolts here) ----
        bore_d=12.7, pcd=25.0, out_bolt="M3", out_clear=3.4, out_depth=2.5,
        # ---- how the STEP sits in its own frame ----
        step_back_face=29.0,    # x of the rear face in the STEP (assemble.py translates by this)
        step_axis=+1,           # +x runs from output face towards the rear face
        # ---- mass and electricals ----
        mass=0.251, stator_frac=0.60, rotor_j=515e-7,
        kt=0.14, kv_rpm_v=53.0, r_line=0.89, i_max=16.5, tau_max=2.31,
        drive="LKMTECH, 7.4-32 V in, LK V2 protocol (0xA1 torque, 1 Mbps)",
        encoder_bits=18, reply_bits=16,   # 18-bit sensor, but the CAN reply field is 16-bit
    ),
    "mf7015": dict(
        name="LKMTECH MF7015-V2 23T (= MyActuator RMD-L-7015)",
        # ---- envelope (STEP bbox: x 33.90, y 69.00, z 69.00; drawing agrees) ----
        od=69.0, length=33.9,
        # ---- body-wall interface: NO boss. The wall registers on the OD instead. ----
        boss_d=None,
        pocket_d=69.4,          # counterbore around the motor's own OD, 0.4 clearance
        pocket_dep=2.0,
        bolt_sq=30.0,           # 4 x M3 on a 30 x 30 square, 10.4 mm deep (drawing: 4xM3 v10.4)
        bolt_clear=3.4,         # M3 clearance
        bolt_name="M3", bolt_depth=10.4,
        # ---- the connector: a recess in the side RIM, leads exit radially ----
        # measured on the STEP: a flat 1.14 mm below the rim, spanning 5.7 mm axially, 2.9-8.6 mm
        # from the rear face. The 4-fold bolt square lets it be clocked; point it UP, as INSTALL.md
        # already says for the MF6015.
        bay="radial", bay_w=15.66, bay_axial=(2.9, 8.6), bay_depth=1.14,
        slot_w=24.0, slot_z=(17.0, 43.0), plug_h=10.0,   # ESTIMATE: see plug_boxes()
        # ---- output face: bore and bolt circle UNCHANGED, but the screws go M3 -> M4 ----
        bore_d=12.7, pcd=25.0, out_bolt="M4", out_clear=4.5, out_depth=3.5,
        # ---- how the STEP sits in its own frame (axis runs the OTHER way to the MF6015's) ----
        step_back_face=-33.9,
        step_axis=-1,           # -x runs from output face (x=0) towards the rear face (x=-33.9)
        # ---- mass and electricals (MyActuator L-7015 parameter sheet, 23T) ----
        mass=0.360, stator_frac=0.60, rotor_j=933e-7,
        kt=0.28, kv_rpm_v=34.0, r_line=1.14, i_max=15.0, tau_max=3.80,
        drive="LKMTECH DF70V2, 7.4-32 V in, LK V2 protocol. NOT the MyActuator MC200 build: "
              "that is 12-24 V and V3, and a 6S pack at 25.2 V would over-volt it",
        encoder_bits=18, reply_bits=16,
    ),
}


def get(profile):
    """One motor's record. Raises with the known names rather than a KeyError on a typo."""
    try:
        return MOTORS[profile]
    except KeyError:
        raise KeyError("unknown motor profile %r: know %s" % (profile, sorted(MOTORS)))


def stator_rotor(profile):
    """(stator kg, rotor kg): what stays in the body, and what turns with the wheel."""
    m = get(profile)
    return m["mass"] * m["stator_frac"], m["mass"] * (1.0 - m["stator_frac"])


if __name__ == "__main__":
    for k, m in MOTORS.items():
        st, ro = stator_rotor(k)
        print(f"{k}: {m['name']}")
        print(f"   {m['od']:.1f} dia x {m['length']:.1f} long, {m['mass']*1000:.0f} g "
              f"(stator {st*1000:.0f} / rotor {ro*1000:.0f})")
        print(f"   wall: 4 x {m['bolt_name']} on {m['bolt_sq']:.0f} x {m['bolt_sq']:.0f}, "
              f"register {m['pocket_d']:.1f} dia x {m['pocket_dep']:.1f}, "
              f"boss {m['boss_d'] or 'NONE'}, leads {m['bay']}")
        print(f"   output: {m['bore_d']} bore, 4 x {m['out_bolt']} on PCD {m['pcd']:.0f}")
        print(f"   Kt {m['kt']} N.m/A, {m['kv_rpm_v']} rpm/V, {m['r_line']} ohm, "
              f"{m['i_max']} A -> {m['tau_max']} N.m, rotor J {m['rotor_j']*1e7:.0f} g.cm2")


def plug_boxes(profile, side, back_y, with_leads=False):
    """The mated connector envelope, as plain (x0, x1, y0, y1, z0, z1) boxes in the ROBOT frame.

    Data, not geometry, so assemble.py and real_assembly.py can each build it with their own box
    helper and cannot drift apart - they used to hold a hand-copied MF6015 envelope each, which is
    how a Ø69 motor ended up reported as clashing with its own connectors.

    `side` is +1 (left) / -1 (right); `back_y` is body_case.MOTOR_BACK_Y.

    MF6015: two plugs standing on top of the Ø49 rear boss, leads exiting AXIALLY (measured off the
    LK drawing; plug sizes were always estimates).
    MF7015: one socket recessed in the side RIM - measured on the STEP as 15.66 wide x 5.7 axial,
    1.14 deep, 2.9-8.6 mm from the rear face - with the mated plug standing off RADIALLY. Clock the
    motor so it points up; the 4-fold bolt square allows 90 deg steps.

    The one number here that is NOT measured is the mated plug's height above the rim (plug_h). It
    is taken as 10 mm, the same stack the MF6015's own mated plugs make, and it decides how much
    room the shelf has to leave. Check it against the real part before printing.
    """
    m = get(profile)
    def y(a, b):
        return (min(side * a, side * b), max(side * a, side * b))
    if m["bay"] == "axial":
        out = [(-7.0, 7.0, *y(25.7, 29.8), 18.5, 29.0),        # 6-pin housing (PH/GH-class), mated
               (-4.5, 2.0, *y(22.6, 25.5), 18.5, 27.0)]        # 3-pin GH1.25, mated
        if with_leads:
            out += [(-5.0, 5.0, *y(23.0, 29.0), 29.0, 33.0),
                    (-5.0, 5.0, *y(12.0, 24.0), 29.0, 33.0)]
        return out
    # radial: the plug stands off the rim, pointing up, over the socket's own axial span
    hw = m["bay_w"] / 2.0
    y0, y1 = back_y + m["bay_axial"][0], back_y + m["bay_axial"][1]
    # Measure the envelope from the can's OUTER surface, not from the recessed shelf: the plug's
    # base sits down in the recess, inside the motor's own volume, and starting there just makes the
    # envelope intersect the can it is plugged into (which read as a 176 mm3 "clash" with itself).
    z0 = m["od"] / 2.0
    out = [(-hw, hw, *y(y0, y1), z0, z0 + m["plug_h"])]
    if with_leads:                                             # lead arcs over and inboard
        out += [(-5.0, 5.0, *y(y0 - 2.0, y1), z0 + m["plug_h"], z0 + m["plug_h"] + 4.0),
                (-5.0, 5.0, *y(12.0, y0), z0 + m["plug_h"], z0 + m["plug_h"] + 4.0)]
    return out


def plug_pin_z(profile):
    """z of the mated plug's pin plane: where a harness actually lands on the motor.

    The third copy of this geometry lived in wiring.py as a bare 29.0. On the MF6015 that is the top
    of the 6-pin housing standing on the rear boss; on the MF7015 the plug stands off the rim, so it
    is the can's radius plus the mated plug's height.
    """
    m = get(profile)
    return 29.0 if m["bay"] == "axial" else m["od"] / 2.0 + m["plug_h"]
