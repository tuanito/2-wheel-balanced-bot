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
| `ml/` | RL for the stand-up: see `ml/README.md`. `setup_mac.sh` (venv, `--cad` also installs CadQuery), `run_training.sh` (the whole loop). |
| `elec/` | `netlist.py` + `erc.py` (electrical rules check; `test_erc.py` mutation tests). Current result: CHECK 3, WARN 1, INFO 3 (all known). |
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
- `cad_src/case/wall_check.py` imports `design_variants`, which is not in the repo. Dead script.
- `firmware/esp32/lib/botcore/` is a copy of `core/`: run `sync_core.sh` after editing `core/`. It now
  verifies the copy with `cmp` and exits non-zero on a mismatch - it used to omit `standup_policy.hpp`
  and still print "updated", so the ESP32 could have been built against the placeholder policy.
- `BotSim.step()` returns `BotSim.out` itself, one ctypes struct the C++ refills in place every tick, not
  a snapshot. Anything about a particular instant must be copied out before stepping the sim on. Reading
  `o.pitch_rate` after a later loop reported 5.5 rad/s for a handover that happened at 16.9.
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
- Two things about that run are not what they look like, and should be fixed before trusting a fourth one:
  the start-pitch curriculum never advanced (`env/start_pitch` was 1.200 for all 2 M steps - it only
  ratchets on full success, which stayed at ~12%), so the policy trained entirely at 1.2 rad and generalises
  to the real 1.8 rad by luck rather than design; and `env/success_rate` (first attempt, survived 1 s) is
  not the same measure as `evaluate.py` (upright within 5 s, retries allowed), which is why one reads 0.125
  where the other reads 1.00. The project gate is `evaluate.py`.
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
