"""The motor profile, for the simulator. One definition, shared with the CAD.

`cad_src/case/motors.py` is the single record per motor (envelope, interfaces, mass split,
electricals). This is the thin bridge that hands the simulator the half it needs, so that
`botsim.py` and `build_model.py` stop carrying their own copies of Kt, Kv, R and the mass split -
which is how the plant model and the CAD come apart without anything failing.

`ROBOT_MOTOR` selects the profile, exactly as it does for the CAD. Unset = mf6015, the robot that
exists; the committed `robot_c.xml`, `gains.hpp` and `sim/results/` are that build.

    ROBOT_MOTOR=mf7015 python build_model.py      # the bigger motor's plant instead

The electrical numbers are what `botsim.py` turns a torque command into a deliverable torque with:

    i      = clamp(tau / KT, +-I_MAX)              current limit (the drive's, not the motor's)
    i_hi   = (vbat - KE * w) / R_LINE              back-EMF against the bus
    tau    = KT * clamp(i, i_lo, i_hi)

I_MAX is a property of the DRIVE, not of the motor, and on the MF7015 it has not been measured on a
bench: the profile's 15 A is the published instantaneous figure. Treat it as an assumption.
"""
import importlib.util, math, os, pathlib

# Loaded by PATH, under a different name, on purpose. Several sim scripts put cad_src/case on
# sys.path (robot_paths.add_sources), so a module called `motors` in sim/ would shadow the CAD's
# `motors` - or be shadowed BY it - depending on import order, which is the silent wrong-module bug
# this whole refactor exists to remove. Hence this file is motor_profile.py and the CAD record is
# loaded as cad_motors from its own path.
_CAD_PATH = pathlib.Path(__file__).resolve().parents[1] / "cad_src" / "case" / "motors.py"
_spec = importlib.util.spec_from_file_location("cad_motors", _CAD_PATH)
_cad = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_cad)

PROFILE = os.environ.get("ROBOT_MOTOR", "mf6015")
M = _cad.get(PROFILE)
NAME = M["name"]

# ---- electricals (botsim.py) --------------------------------------------------------------------
KT = M["kt"]                                    # N.m/A           mf6015 0.14,  mf7015 0.28
KV_RPM_V = M["kv_rpm_v"]                        # rpm/V           mf6015 53,    mf7015 34
KE = 1.0 / (KV_RPM_V * 2 * math.pi / 60)        # V per rad/s
WIRING_R = 0.10                                 # harness + connectors, both builds
R_LINE = M["r_line"] + WIRING_R                 # ohm             mf6015 0.99,  mf7015 1.24
I_MAX = M["i_max"]                              # A, the DRIVE's limit - see the module docstring
TAU_MAX = M["tau_max"]                          # N.m             mf6015 2.31,  mf7015 3.80

# ---- mass and inertia (build_model.py) ----------------------------------------------------------
STATOR_KG, ROTOR_KG = _cad.stator_rotor(PROFILE)    # body side / wheel side
STATOR_R = _cad.stator_geom_r(PROFILE)              # m, the modelled stator cylinder
ROTOR_J = M["rotor_j"]                              # kg.m^2, referred to the wheel joint (armature)
OD = M["od"] / 1000.0
LENGTH = M["length"] / 1000.0

# The MuJoCo actuator's ctrlrange has to leave headroom over the motor's peak torque, or the command
# is silently clipped and the sim quietly models a weaker motor than the one being evaluated.
CTRL = math.ceil(TAU_MAX + 0.2)                     # N.m             mf6015 3,     mf7015 4


def summary():
    return (f"{PROFILE}: {NAME}\n"
            f"   Kt {KT:.3f} N.m/A, {KV_RPM_V:.0f} rpm/V (Ke {KE:.4f} V/rad/s), R_line {R_LINE:.2f} ohm "
            f"(motor {M['r_line']:.2f} + harness {WIRING_R:.2f}), I_max {I_MAX:.1f} A, tau_max {TAU_MAX:.2f} N.m\n"
            f"   stator {STATOR_KG*1000:.0f} g (r {STATOR_R*1000:.1f} mm) / rotor {ROTOR_KG*1000:.0f} g, "
            f"rotor J {ROTOR_J*1e7:.0f}e-7 kg.m2, ctrlrange +-{CTRL}")


if __name__ == "__main__":
    print(summary())
