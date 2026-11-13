"""Is a bigger motor's advantage its electricals, or just its extra mass sitting on the axle?

The MF7015-V2 23T beats the fitted MF6015-V2 on every measure in motor_compare/wobble_compare. It has
twice the torque constant AND 109 g more mass per motor, and in a balancer those pull in completely
different ways: torque is an actuator property, while motor mass sits at axle height and drags the
centre of mass DOWN - which is this robot's known weak point (CoM 40.6 mm above the axle).

A table of candidate motors cannot separate the two, because every real motor changes both at once.
So build two chimeras that do not exist: one motor's electricals with the other's mass, and the
reverse. Whichever half moves the score is the half that matters.

  python motor_isolate.py

Result: the PUSH rows pair up exactly by mass - the two chimeras score what their mass says they
should, not what their windings say - so the MF7015's push advantage is entirely its 109 g per motor
sitting on the axle, and ~220 g of ballast buys the same thing. STAND-UP is the honest exception:
there the extra torque does show (the 7015-electricals/6015-mass chimera scores 40/40 against the
real 6015's 38/40), because the kick is torque-limited in a way that holding station is not.
"""
import sys
from multiprocessing import Pool

import motor_compare as mc
import wobble_compare as wc

#                                   Kt     rpm/V   R ohm  tau_max  mass kg  rotor inertia
mc.MOTORS["6015 elec, 7015 mass"] = (0.140, 53.0,  0.89,  2.31,    0.360,   933e-7)
mc.MOTORS["7015 elec, 6015 mass"] = (0.280, 34.0,  1.14,  3.80,    0.251,   515e-7)

CASES = ["MF6015-V2 15T (current)", "MF7015-V2 23T", "6015 elec, 7015 mass", "7015 elec, 6015 mass"]
IMPS = (2.0, 2.2, 2.5)          # 2.0-2.2 N.s is where the two motors actually separate
# Same seed base as wobble_compare's own push sweep (300+i), so these rows sit on the SAME robots as
# the published table and can be read next to it. 20 rather than 10: the interesting impulses are
# right on the half-survival point of the curve, where a 10-robot sample swings by 3/10 on seed
# choice alone - which is how one early run read 5/10 and another 8/10 for the same motor.
N = 20

if __name__ == "__main__":
    print(f"push survival, {N} randomised robots each (the impulses where the motors differ)")
    with Pool(4) as p:
        for name in CASES:
            got = [sum(p.map(wc.push, [(name, "abs16", 300 + i, imp) for i in range(N)])) for imp in IMPS]
            st = p.map(mc.stand, [(name, 8000 + i, sd) for i in range(N) for sd in (1, -1)])
            print(f"  {name:26s} " + "  ".join(f"{v} N.s {c:2d}/{N}" for v, c in zip(IMPS, got))
                  + f"   stand-up {sum(x[0] for x in st)}/{2 * N}", flush=True)
    print("\nPush rows pair by MASS, not by electricals. Stand-up is where the torque shows.")
