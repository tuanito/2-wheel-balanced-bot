"""The motor's numbers exist in two languages. Check they agree, for every profile.

`cad_src/case/motors.py` is the single record per motor and `core/motor.hpp` is the C++ mirror of
the control-relevant half, because the controller has to compile for the ESP32 with no Python. Two
copies of a number is how this project has lost days: a Kt or a speed ceiling that disagrees between
the plant and the controller does not fail, it just produces a wrong result that looks physical.
Measured once: the MF7015 scored 0/30 on the stand-up instead of 30/30 because the controller kept
the MF6015's torque limit, and the explanation that fitted the numbers was a motor speed ceiling.

Checked per profile, by compiling a probe that prints what the C++ actually sees:
  TAU_MAX   == tau_max        ROTOR_J  == rotor_j
  KT        == kt             W_MAX    <= the no-load ceiling from kv_rpm_v at VBAT_NOM

W_MAX is an inequality on purpose: it is the runaway guard's reference, and the guard has to be
reachable, so the number may sit at or a little under the true no-load speed (mf7015: 78 against
79.0) but never above it - above it, the guard can never fire.

    python check_motor.py            # every profile, not just ROBOT_MOTOR's
"""
import math
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
CORE = HERE.parent / "core"
HPP = CORE / "motor.hpp"

VBAT_NOM = 22.2          # 6S LiPo nominal, the voltage both ceilings are quoted at
DEFINE = {"mf6015": [], "mf7015": ["-DBOT_MOTOR_MF7015"]}

PROBE = r'''
#include "motor.hpp"
#include <cstdio>
int main() {
  std::printf("NAME %s\n", bot::motor::NAME);
  std::printf("TAU_MAX %.6f\n", (double)bot::motor::TAU_MAX);
  std::printf("KT %.6f\n", (double)bot::motor::KT);
  std::printf("W_MAX %.6f\n", (double)bot::motor::W_MAX);
  std::printf("ROTOR_J %.10f\n", (double)bot::motor::ROTOR_J);
  return 0;
}
'''


def cpp_values(profile, td):
    src = pathlib.Path(td) / f"probe_{profile}.cpp"
    src.write_text(PROBE)
    exe = pathlib.Path(td) / f"probe_{profile}"
    r = subprocess.run(["c++", "-std=c++17", "-I", str(CORE), *DEFINE[profile], str(src), "-o", str(exe)],
                       capture_output=True, text=True)
    if r.returncode:
        raise SystemExit("could not compile the motor probe for %s: %s"
                         % (profile, r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "?"))
    out = subprocess.run([str(exe)], capture_output=True, text=True).stdout
    v = {}
    for line in out.split("\n"):
        if not line.strip():
            continue
        k, val = line.split(None, 1)
        v[k] = val.strip() if k == "NAME" else float(val)
    return v


def main():
    sys.path.insert(0, str(HERE))
    import motor_profile                                     # loads cad_src/case/motors.py by path
    cad = motor_profile._cad

    bad = []
    with tempfile.TemporaryDirectory() as td:
        for profile in sorted(DEFINE):
            if profile not in cad.MOTORS:
                bad.append(f"  core/motor.hpp has a {profile} block but cad_src/case/motors.py does not")
                continue
            m = cad.get(profile)
            c = cpp_values(profile, td)
            if c["NAME"] != profile:
                bad.append(f"  {profile}: core/motor.hpp NAME is {c['NAME']!r} - the #if selected the wrong block")
            for key, field, tol in (("KT", "kt", 1e-4), ("ROTOR_J", "rotor_j", 1e-9)):
                if abs(c[key] - m[field]) > tol:
                    bad.append(f"  {profile}: motor.hpp {key} {c[key]:g} != motors.py {field} {m[field]:g}")
            # TAU_MAX is a CHOSEN limit, not a capability: it must not exceed what the motor can do,
            # and must not sit far under it either. The as-built MF6015 commands 2.3 against a 2.31
            # datasheet peak, which is the rounding the robot has always shipped with.
            if c["TAU_MAX"] > m["tau_max"] + 1e-6:
                bad.append(f"  {profile}: motor.hpp TAU_MAX {c['TAU_MAX']:g} N.m EXCEEDS the motor's "
                           f"peak {m['tau_max']:g} - the drive would clip and the sim would not")
            elif c["TAU_MAX"] < 0.95 * m["tau_max"]:
                bad.append(f"  {profile}: motor.hpp TAU_MAX {c['TAU_MAX']:g} N.m is more than 5% under "
                           f"the motor's peak {m['tau_max']:g} - torque left on the table")
            ceiling = m["kv_rpm_v"] * VBAT_NOM * 2 * math.pi / 60.0
            if c["W_MAX"] > ceiling + 1e-6:
                bad.append(f"  {profile}: motor.hpp W_MAX {c['W_MAX']:.1f} rad/s is ABOVE the no-load "
                           f"ceiling {ceiling:.1f} ({m['kv_rpm_v']:.0f} rpm/V at {VBAT_NOM} V): the "
                           f"runaway guard could never fire")
            elif c["W_MAX"] < 0.9 * ceiling:
                bad.append(f"  {profile}: motor.hpp W_MAX {c['W_MAX']:.1f} rad/s is more than 10% under "
                           f"the ceiling {ceiling:.1f}: the guard would cut a healthy motor")

    for mp in sorted(cad.MOTORS):
        if mp not in DEFINE:
            bad.append(f"  cad_src/case/motors.py has {mp} but core/motor.hpp has no block for it "
                       f"(and sim/check_motor.py no -D for it)")

    if bad:
        print("MOTOR PROFILE MISMATCH (core/motor.hpp vs cad_src/case/motors.py):")
        print("\n".join(bad))
        return 1
    print("motor profiles OK: %s agree between core/motor.hpp and cad_src/case/motors.py"
          % ", ".join(sorted(DEFINE)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
