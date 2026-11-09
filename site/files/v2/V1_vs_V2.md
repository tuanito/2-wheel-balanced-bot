# V1 vs V2

**V1** is shape C: the wheels sit under the body, the lidar is on top, and the battery is on a shelf above the motors. It is frozen as it was on 5 Nov 2026 (`versions/V1`, and the V1 page of the viewer).

**V2** puts the motors back on the side walls with the wheels outside the body. It has no lidar, and the battery lies on the floor between the motors at axle height.

## Geometry and mass (from the CAD assembly)

| | V1 (shape C) | V2 |
|---|---|---|
| Height (floor to top) | 223 mm (lidar top) | 170 mm |
| Width (over the wheels) | 208 mm | 274 mm |
| Track (wheel centres) | 163 mm | 229 mm |
| Mass (sim model) | 1.55 kg | 1.40 kg |
| Centre of mass above the axle, whole robot | 33 mm | **14 mm** |
| Centre of mass above the axle, body only (wheels and rotors removed) | 47 mm | 21 mm |
| Body pitch inertia about the axle | 5.6 g·m² | 2.9 g·m² |
| Lying on its back / face (settled in the sim) | 97° / 101° from upright | 98° / 103° from upright |
| BOM, required parts | ≈ €867, 1.61 kg | ≈ €732, 1.46 kg (no lidar or USB OTG adapter) |

## Simulation (same controller code, randomised robots)

| Test | V1 | V2 |
|---|---|---|
| Stand up from face-down | 40/40, peak wheel speed 4.6 m/s, travels 0.7 m | **30/30**, peak 0.8 m/s, travels 0.19 m |
| Stand up from back-down | 40/40 | **30/30** (29 on the first try) |
| Stand still, 10 s | drift ≤ 4 mm | drift ≤ 2 mm |
| Shove, 1 N·s front and back | 26/26 recovered | **9/20** recovered |
| 1 m/s, then stop | stops within 0.6 m, lean 17° | stops within 1.3 m (acceleration capped at 0.4 m/s²), lean 16–18° |
| Turns (90°, 180°, −135°, to 30°) | < 1° nominal | < 0.2° nominal, < 2.2° randomised |
| Knocked over → stands up again | yes | yes |
| Backflip from the back (KENBOT style) | 0 clean flips | 0 clean flips |
| Explore and map a room | lidar: 0 collisions, 3 cm error | not simulated (the sonar mapping is unit-tested only) |

## What we learned

1. **Lowering the centre of mass helps stand-ups but hurts balancing.**
   - The good part: the body is light about the axle, so stand-ups are gentle (1.0 N·m kick instead of 1.6). They are short and slow: 0.19 m of travel instead of 0.7 m.
   - The bad part: to accelerate or brake, a two-wheel balancer has to lean, so that gravity balances the wheel torque. The lean needed grows as wheel radius ÷ CoM height. With the CoM 21 mm above a 54 mm axle, V2 leans about twice as much as V1 for the same acceleration. That limits how hard it can brake, which is why it recovers fewer hard shoves.
   - A study in the sim moved only the battery up by 80 mm (V2 layout, CoM 25 mm above the axle). That gave stand-up 20/20, shoves 20/20 and 11° of lean at 1 m/s. This is the best of all the variants tried, and it is still lower and lighter than V1. To repeat it: `sim/make_variant.sh b80 80`, then run `study.py` with `BOT_XML=robot_b80.xml BOT_LIB=libbot_b80.so`.
2. **The backflip still doesn't happen.** The limit is not the CoM.
   - Lying on its back, a kick only spins the wheels up. They reach about 120 rad/s in about 20 ms, while the body barely moves.
   - The wheels plus rotors can store only about 0.08 N·m·s of momentum to throw the body with. Even the lighter V2 body needs more than that to get off the floor and rotate a full turn.
   - It would need much heavier wheels (about 3× the inertia, e.g. metal rims or brass weights) or a much lighter robot.
3. **Without the lidar, the map comes from the sonar.**
   - The Pi now marks free space along the sonar beam (up to 2 m) and the echo as an occupied arc. Readings taken while leaning more than 3.4° are skipped.
   - The avoider remembers mapped obstacles within 1.5 m, so it still avoids walls the sonar is no longer pointing at.
   - Expect a coarse map: one 15° beam instead of 450 points per turn, and no scan matching. That means heading drift is not corrected.

## Files

- `case/body_case_v2.py` (parts) and `case/assemble_v2.py` (interference check: 0 clashes; CoM; lying angles; dock pins; sim data). STL and STEP are in `case/v2/`.
- `sim/`: `robot_v2.xml`, gains for V2 (`design_lqr.py 0.06 0.4 10`), `run_tests.py`, `flip_v2.py`, `study.py`, `make_variant.sh`.
- Controller defaults for V2: `lift_torque` 1.0, `lift_gain` 15, `energy_gain` 0.005, `max_accel` 0.4.
- Firmware builds. Pi: `"lidar": false` in `config.json` (set `true` for V1). 11 tests pass.
- `bom/balancing_robot_BOM_V2.xlsx`. V1 is in `bom/balancing_robot_BOM_V1.xlsx`.
