# 2-wheel-balanced-bot — notes for Claude Code

Two-wheel self-balancing robot, version **V1.1** (body "shape C", lidar, voice orders via Claude, vertical
charging dock). The owner is a C++ programmer; explain in C++ terms, keep answers short, and say plainly
what was verified (sim, build, check) and what wasn't.

## Hardware in one paragraph
2 × LKMTECH MF6015-V2 hub motors (CAN, torque commands, 0xA1 at 1 Mbps) back-to-back on a printed PETG body;
ESP32-S3 DevKitC-1 (balance controller, 500 Hz) on a perfboard with an SN65HVD230 CAN board; Raspberry Pi
Zero 2 W (lidar LD19, docking, voice, Claude API); 6S 1400 mAh LiPo + 6S BMS, Pololu 2813 push-button power
switch (ESP32 GPIO21 can switch it off), 30 A fuse; IMU GY-521; 4 × TSOP38238 IR receivers in hoods on top for
the dock's beacon. Dock: printed foot + head + mast, ATtiny85 controller that keeps the sprung brass rods dead
unless the robot is on them (sense divider, short = latch off). Full list: `bom/build_bom.py` → BOM xlsx.

## Layout
| Folder | What |
|---|---|
| `core/` | The controller (`controller.cpp/.hpp`), LQR gains (`gains.hpp`, from `sim/design_lqr.py`), learned stand-up policy (`standup_policy.hpp`, generated), C API for the sim (`sim_api.cpp`). The **same C++** runs in the sim and on the ESP32. |
| `firmware/esp32/` | PlatformIO (`pio run -e esp32s3`), host tests `test_host/run_tests.sh`. `lib/botcore/` is a COPY of `core/` — keep them in sync after editing `core/`. |
| `firmware/dock_beacon/` | ATtiny85 dock controller + `test_dock_logic.cpp`. |
| `pi/` | Python on the Pi: `robot/brain.py`, `dock.py` (DockPilot), `ld19.py`, `link.py`; tests `python -m pytest -q` in `pi/` (48). |
| `sim/` | MuJoCo + `botsim.py` (loads `libbot.so`/`.dylib`). `run_tests.py` (stand-up, balance, pushes, missions), `dock_sim.py` (40-start docking sweep), `nav_sim.py`, `build_model.py` (regenerates the XML from the CAD), `design_lqr.py 0.06 0.6 10` (the weights we use), `rebuild.sh` (all of that in order). |
| `ml/` | RL: the stand-up and the stair-measuring nod, see `ml/README.md`. `setup_mac.sh` (venv, `--cad` also installs CadQuery), `run_training.sh` (the stand-up loop). Stairs: `stairs_env.py` (when to nod), `stairs_baseline.py` (the hand-scripted recipe it has to beat), `train_stairs.py`, `evaluate_stairs.py`. |
| `elec/` | `netlist.py` + `erc.py` (electrical rules check; `test_erc.py` mutation tests). Current result: CHECK 3, WARN 1, INFO 5 (all known). |
| `cad_src/case/` | CadQuery: `body_case.py` (all printed parts, named constants), `wiring.py` (3D harness routes), `real_assembly.py` (fit check), `wire_check.py`, `export_*`, `assemble.py`. `robot_paths.py` says where everything is — import it, never hard-code a path. `cad_src/rebuild.sh` runs the chain in order. |
| `cad/` | Exported STL/STEP for printing. |
| `site/` | The project website (same as the Claude artifact). Builders: `build_docs.py`, `build_boards.py`, `build_dock_wiring.py`, `tools/site_rebuild.py`. View: `python3 -m http.server -d site 8000`. |
| `tools/` | `make_workspace.sh` (see below), `cad_baseline.py` (CAD regression: volume + bbox of 28 parts), `wc_power.py` (quick wire check of chosen harnesses). |

## Build / check commands
Prefer the scripts: each one is the documented sequence, in order, and they are what gets maintained.
```bash
./check.sh                  # everything checkable here (~40 s): firmware host tests, ERC + its 8
                            # mutation tests, CAN check, the 48 Pi tests, models + 30-robot sim suite.
                            # Takes section names: ./check.sh elec pi. Exits non-zero on failure.
core/build_lib.sh           # the controller library: libbot.dylib on macOS, libbot.so elsewhere
sim/rebuild.sh              # models -> LQR gains -> library -> sim suite
ml/run_training.sh          # the learned stand-up loop end to end (see ml/README.md)
cad_src/rebuild.sh          # the CAD chain; --fast skips the two slow viewer exports
```
By hand, when you want one step only:
```bash
cd sim && python run_tests.py          # 30 robots, ~11 s on an M4 Mac; writes results/report.json
cd sim && python dock_sim.py           # 40 starts (slow)
cd firmware/esp32 && pio run -e esp32s3 && test_host/run_tests.sh
cd pi && python -m pytest -q
cd elec && python erc.py && python test_erc.py && python erc.py   # see the trap below
```
`./check.sh` does not mutate the repo: it snapshots the files the checks regenerate, reports whether a
fresh run disagrees with what is committed, and restores them. Last-digit differences in
`sim/results/report.json` between macOS/clang and Linux/g++ are normal; a changed success count is not.

**Paths.** Every script finds its own files through `cad_src/case/robot_paths.py`, so a plain checkout works
with no setup. Override any root with `ROBOT_CASE`, `ROBOT_WHEEL_ADAPTER`, `ROBOT_ROOT`, `ROBOT_OUT`,
`ROBOT_MODELS`, `ROBOT_MF6015_STEP`. `BOT_WS` is honoured too, for the workspace `tools/make_workspace.sh`
builds (optional now, and only its directories that actually exist are used). Never hard-code a path.

Vendor STEP models are not in the repo (licences): `models/fetch_models.sh` downloads the nine that are
public, and `models/bought/mf6015.step` must be fetched by hand from the LKMTECH / dealer page. Only the fit
and wire checks and the "real parts" view need them; `cad_src/rebuild.sh` checks for them up front.

CAD change workflow: `python tools/cad_baseline.py /tmp/before.json` → edit → `cad_baseline.py /tmp/after.json`
→ compare (refactors must change nothing, all 28 parts). Then `cad_src/rebuild.sh`, which runs `dump_wiring.py`
→ `assemble.py` → `real_assembly.py` (6 known contacts: mics, camera, OTG) → `wire_check.py` (clean except the
known Pi OTG/Dupont contacts) → `export_print_data.py` → the viewer exports → ERC. Then the site builders.

## Traps worth knowing
- `elec/test_erc.py` calls `erc.main()`, which overwrites `erc_report.json` with the last mutation case.
  Run `erc.py` again afterwards. `check.sh` already does.
- `ml/train.py` always prints that it saved `best.zip`, but only writes it once `EvalCallback` has fired
  (~100k steps / n_envs). Short runs leave `latest.zip` only; `run_training.sh` falls back to it.
- **`best.zip` does not mean best.** `EvalCallback` ranks checkpoints by eval reward, and that signal is
  far too noisy here to rank anything (sigma ~2.5 on a mean of ~8). One 2 M-step run's `best.zip` was a
  100k-step checkpoint scoring 26/60 compiled while the same run's final policy scored 171/180 - and it
  was exported and reported as the result. `run_training.sh` now exports and scores *every* candidate with
  `evaluate.py --compiled` and keeps the winner. Never judge a run by `best.zip` alone.
- `evaluate.py <model>` (torch, mode 2) and `evaluate.py --compiled` (the C++ policy, mode 1) can disagree
  wildly, and only `--compiled` is the gate: it is what the robot runs. The two forward passes differ by
  ~1e-7 from float32 summation order, which a chaotic swing-up amplifies - traced once to an identical
  `+1.00000` first action and pitch splitting at the 4th decimal eleven ticks later. A policy whose two
  numbers disagree by more than a robot or two is sitting on a knife edge; `run_training.sh` prints both
  and says so.
- `cad_src/case/wall_check.py` imports `design_variants`, which is not in the repo. Dead script.
- `firmware/esp32/lib/botcore/` is a copy of `core/`: run `sync_core.sh` after editing `core/`. It now
  verifies the copy with `cmp` and exits non-zero on a mismatch - it used to omit `standup_policy.hpp`
  and still print "updated", so the ESP32 could have been built against the placeholder policy.
- `BotSim.step()` returns `BotSim.out` itself, one ctypes struct the C++ refills in place every tick, not
  a snapshot. Anything about a particular instant must be copied out before stepping the sim on. Reading
  `o.pitch_rate` after a later loop reported 5.5 rad/s for a handover that happened at 16.9.
- **Publishing `site/index.html` as the Artifact page double-wraps it.** The Artifact tool always
  wraps the page in its own `<!doctype html><html><head>` skeleton, and `site/index.html` is already
  a complete document (it has to be - `site/` is served directly too). Publishing it as `file_path`
  nests one document inside the other: two `<html>`, two `<head>`, and the three.js **importmap**
  ends up inside the body, so `import 'three'` fails and the 3D viewer renders nothing. Strip the
  outer `<!DOCTYPE>/<html>/<head>/<body>` tags into a throwaway copy and publish that; keep the repo
  file a valid standalone page. Supporting files passed in `files` (e.g. `standup.html`) are served
  unwrapped, so those keep their own doctype.
- **A manoeuvre tested on one robot tells you nothing.** `sim/stairs_look.py` ran the look-down nod on
  a single robot and it worked; over randomised robots it lost 7 in 10, and every failure was a robot
  that drew two control periods of CAN latency (`BotSim.delay == 2`, which the randomisation draws half
  the time). At the full LQR pitch gains the nod's PD is a ~130 rad/s rate loop and 4 ms of latency takes
  it unstable: the pitch rings, the wheels get pumped to 5 m/s and the 114 rad/s runaway guard cuts the
  motors. Half gains (`look_gain`, as the dock lean already used) gives 10/10 at the same peak pitch.
  Any new high-gain pitch loop has to be measured against delay-2 robots, not the nominal one.
- **Never hand a manoeuvre back to the balance law with `v_ref_ = 0` while the robot is moving.** A 34 deg
  nod ends at ~0.55 m/s; zeroing the reference demands all of it back in one tick, `K[3]` saturates the
  torque, the body over-rotates to -20 deg in 30 ms and the wheels run away. Hand over `v_ref_ = phi_rate_
  * R` and let the normal `max_accel` ramp take the speed down. Same shape of bug as the stand-up's
  fixed-torque kick tripping the runaway guard.
- **A one-sided flick from a standstill costs ~1 m of travel towards the drop.** A flick IS a forward
  acceleration, so with no reverse momentum to cancel it the robot keeps what it gains: 0.75 rad (the
  ~32 deg that can see a tread) costs **+899 +- 264 mm** from a dead stop. Reversing for 0.9 s first -
  still from a standstill, still one forward look - reaches -0.47 m/s and the same flick costs
  **+62 +- 86 mm**, 100% upright. `sim/stairs_windup.py` has the table. Corollary: **retrying deeper is
  a trap.** 0.90 rad costs +504 mm against 0.75 rad's +62 mm, so a deeper second look spends exactly
  the margin it is trying to earn. Retry closer at the same depth, never deeper.
- **Sensor HEIGHT is what decides whether a step can be measured at all.** A ray reaches the first
  tread only if it clears the nose and lands before the next one, which needs
  `D < h * tread / rise`; and the landing occludes everything below it short of the edge, so the
  vertical riser face is never visible from behind the edge at any angle. Hence 170 mm risers are
  unreachable beyond 408 mm even from the lidar at 231 mm, and a time-of-flight sensor mounted LOW
  would be the worst possible place for one (at 80 mm the same riser needs to be inside 141 mm).
  The VL53L5CX therefore goes high on the front panel, 194 mm up, looking 30 deg down.
- **A zone is a cone, not a ray.** Modelling each VL53L5CX zone as its centre ray alone quietly
  deletes every mixed pixel from the problem - and this robot looks straight at a depth
  discontinuity, where mixed pixels live. With 5 sub-rays per zone and the nearest taken, as the
  part reports, the riser error went from 3.8 mm to 12.8 mm and the success rate from 1.000 to 0.938.
  The flattering number was the model, not the sensor.
- **`bot::Inputs` and `bot::Command` are mirrored by hand in `sim/botsim.py` as ctypes structs.**
  Adding a field to either in C++ without adding it there shifts every field after it and the sim
  reads garbage, silently. Check `ctypes.sizeof` against `sizeof` in C++ after any such change.
- **GPIO 33-37 are the octal PSRAM bus on the N8R8 module** and 26-32 the flash: a pin assignment
  there compiles and then does not boot. `elec/erc.py` checks this, and it is what caught the first
  VL53L5CX pinout. Free on this build: 2, 41, 42, 47 (and the sonar's 10/11 when it is not fitted).
- **The sonar's edge event is only a measurement when the body is level and steady.** "The floor echo
  is gone" means two different things: the floor really ended (the edge), or the beam is pointing over
  floor that is still there (the robot leaned back). The second fires during every brake and puts
  +80 mm of error into the distance - towards the drop. The event counts only on a confirmed-floor to
  lost transition, held two pings, with |pitch| < 0.05 rad and |pitch rate| < 0.6 rad/s. With that gate
  the estimate runs +3 to +6 mm all episode; without it, +25 to +144 mm.
- **Calibrate the event with the SAME detector you run.** The factor turning the event into a distance
  is defined against a detection rule; a calibration that used a different rule (or a different ping
  rate) is meaningless. Two copies of the rule drifted apart once and cost a day. `FloorWatch` in
  `sim/stairs_sonar.py` is the single definition, used by both the calibration and the environment.
- **Grazing beams are hopeless range finders.** The floor range goes as h/sin(tilt+15), so its
  sensitivity to body pitch is h*cos(a)/sin(a)^2: -1.94 m/rad at 0 deg tilt against -0.46 at 15 deg.
  Measured end to end, the edge event is +141 +- 87 mm at 0 deg and -4 +- 12 mm at 15 deg. More tilt is
  better until the event falls inside D_FENCE (250 mm), which rules out 20 deg and makes 15 deg the
  pick for the as-built 134 mm mounting.
- **A retry phase that can only reverse walks the robot away from the edge.** The look is entered
  reversing, so each one ends ~180 mm further back than it started. A reposition step that only closes
  in when it is too far - and never comes forward when it is too far back - put the third flick's peak
  pitch at 1200 mm, where nothing is visible: 0 returns on every retry. Reposition from either side.
- **Tilting the sonar down makes an edge detector, not a sensor.** Modelled as a real +-15 deg cone
  taking the nearest echo (`sim/stairs_sonar.py`, which reproduces h/sin(tilt+15) to 4 mm), a 10 deg
  tilt loses the floor echo at **263 +- 12 mm** from the drop with no odometry involved - the safety
  signal the margin failures need. But the same reading swings **245 -> 448 mm over +-7 deg of body
  pitch**, and a balancing robot sways that much holding station, so the forward obstacle stop has to
  predict the floor range from the IMU and gate it out or it trips constantly. The sonar can never
  contribute to the reconstruction: one range, no bearing, over a 30 deg cone.
- **A riser is a DIFFERENCE between two surfaces, so returns on the tread measure nothing on their
  own.** `estimate()` finds the landing first and subtracts the first level surface below it; with no
  returns on the landing it gives `rise=None` however good the view of the tread is. The swing taken
  from a short standoff fails exactly this way - one episode put 597 rays on the first tread and
  produced no riser at all, because every ray had cleared the landing. The flick does not have the
  problem only because it looks from further back while reversing. `n_seen` alone therefore does not
  mean a measurement: `land_seen` in the info dict says which of the two failures it was.
- **`build_model.py --stairs` landings are finite.** The landing runs from `edge - 3.0` to `edge`, so a
  script that reverses for a second or two can drive off the BACK of it, and that reads exactly like
  falling over. Two of the nod-cost measurements were wrong that way before the start was moved forward.
- `mujoco` is pinned to `==4.0.0` in `ml/requirements.txt` because the committed `sim/results/report.json`
  is version-dependent. 4.0 split the distribution (mujoco-core + mujoco-studio) and an in-place upgrade
  from 3.x can delete files that moved: uninstall first. Verified: 4.0.0 reproduces the committed counts.

## Rules that must not be broken
- **No secrets in git.** The Claude API key lives only in `/etc/robot.env` on the Pi (root, chmod 600), loaded
  by `EnvironmentFile=-/etc/robot.env` in `robot.service`. Never in `config.json`. `.gitignore` blocks `*.env`.
- **Dock safety:** the rods are dead unless the robot's detect divider is sensed (ADC 310–480); ON check < 15 V
  → fault; 3 faults → latched. Don't weaken this.
- **Power:** XT60s between the back-cover module and the body are gender-keyed (battery side female in, switch
  side female out). Peak current on the main feed 18.9 A (ERC).
- Keep `firmware/esp32/lib/botcore/` = `core/`.
- After any site change: rebuild and commit `site/` (the owner wants the site kept in the repo).

## State (Nov 2026) and open points
- Shelf raised 10 mm (SHELF_Z 47, Z_TOP 152) so the motor leads loop gently; back cover unplugs as a module.
- Cost: CoM 40.6 mm above the axle → hand-tuned stand-up 25/30 face-down, 23/30 back-down within 5 s (was
  30/30), 52/60 within 12 s; docking 34/40 (was 37/40).
- **The learned stand-up now beats the hand-tuned one in the sim** (Nov 2026, third training run):
  `evaluate.py --compiled` 60/60 on the usual seeds 3000-3029 and 60/60 on two held-out sets (9000, 12000),
  against 155/180 for the hand-tuned one; median travel 0.84 m vs 0.97. It hands over to the balance
  controller at 1.77 rad/s against the hand-tuned 3.66, and reaches the catch far more repeatably
  (p90 0.80 s vs 1.64 s). `core/standup_policy.hpp` holds the trained weights and `STANDUP_POLICY_TRAINED`
  is `true`, **but `standup_mode` is still 0** (`core/controller.hpp:104`), so the robot behaves exactly as
  before. The sim gate is met; flipping it to 1 is a hardware decision and has not been taken.
- **The shipped policy was trained at 1.2 rad, and that is better than training on the real task.** The
  curriculum ratchet used to gate on full success (~12%), never cleared `UP_OK`, and left every worker at
  `PITCH_START` for all 2 M steps - it had silently collapsed to "always train at 1.2 rad". That gate is
  now fixed (it ratchets on a catchable handover: a success, or `gentle >= GENTLE_OK`, i.e. pitch rate
  <= ~6.7 rad/s) and verified: a fourth run climbed to the real 1.8 rad by 200k steps and stayed there.
  It scored **171/180**, against **180/180** for the policy trained entirely at 1.2 rad. So the easy start
  pitch is a better teacher, not a lucky accident - the alternative was measured. The shipped weights are
  therefore the ones from before the gate was fixed: to regenerate them, check out `ml/` at `e4c9d26`.
  Retraining with today's `ml/` should land near 171/180, which still beats the hand-tuned 155/180.
- `env/success_rate` (caught first attempt, survived 1 s) is not the same measure as `evaluate.py` (upright
  within 5 s, retries allowed), which is why one reads ~0.13 where the other reads 1.00. Still open, and
  still worth reconciling: the gate is `evaluate.py`.
- Earlier runs failed for reward-design reasons, both diagnosed, both worth not repeating: run 1 (`ent_coef`
  0) annealed its exploration away before ever reaching the success bonus; run 2 gave a flat +2.0 for
  touching the catch window, so the policy slammed through it at 16.9 rad/s and lost all 60. The fix was to
  scale that credit by `exp(-(rate/8)^2)`. `ml/train.py` now logs `env/catch_rate_rad_s`, `env/catch_v_m_s`,
  `env/success_rate` and `env/start_pitch` to TensorBoard: the eval-reward curve alone (sigma ~2.5) would
  not have shown either failure, and both were obvious in those four numbers within 100 k steps.
- Demo videos in `site/files/` still show the pre-raise body.
- `ml/evaluate.py` used to fail on macOS (multiprocessing spawns there, so the workers never saw the model
  path); fixed.
- Motors: cheaper alternatives were reviewed (StepperOnline, PGFUN, VXB, gimbal motors): none fits; the only
  candidate is a low-KV outrunner + ODESC driver, not pursued.
