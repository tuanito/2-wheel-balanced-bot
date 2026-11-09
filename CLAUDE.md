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
| `sim/` | MuJoCo + `botsim.py` (loads `libbot.so`/`.dylib`). `run_tests.py` (stand-up, balance, pushes, missions), `dock_sim.py` (40-start docking sweep), `nav_sim.py`, `build_model.py` (regenerates the XML from the CAD), `design_lqr.py 0.06 0.6 10` (the weights we use). |
| `ml/` | (branch `ml-standup`) RL for the stand-up: see `ml/README.md`. |
| `elec/` | `netlist.py` + `erc.py` (electrical rules check; `test_erc.py` mutation tests). Current result: CHECK 3, WARN 1, INFO 3 (all known). |
| `cad_src/case/` | CadQuery: `body_case.py` (all printed parts, named constants), `wiring.py` (3D harness routes), `real_assembly.py` (fit check), `wire_check.py`, `export_*`, `assemble.py`. |
| `cad/` | Exported STL/STEP for printing. |
| `site/` | The project website (same as the Claude artifact). Builders: `build_docs.py`, `build_boards.py`, `build_dock_wiring.py`, `tools/site_rebuild.py`. View: `python3 -m http.server -d site 8000`. |
| `tools/` | `make_workspace.sh` (see below), `cad_baseline.py` (CAD regression: volume + bbox of 28 parts), `wc_power.py` (quick wire check of chosen harnesses). |

## Build / check commands
```bash
g++ -O2 -std=c++17 -shared -fPIC core/controller.cpp core/sim_api.cpp -o sim/libbot.so   # macOS: clang++ ... -o sim/libbot.dylib
cd sim && python run_tests.py          # ~10 min; writes results/report.json
cd sim && python dock_sim.py           # 40 starts, ~15 min
cd firmware/esp32 && pio run -e esp32s3 && test_host/run_tests.sh
cd pi && python -m pytest -q
cd elec && python netlist.py && python erc.py && python test_erc.py
```
CAD / wiring / site scripts expect the old workspace layout: run `tools/make_workspace.sh` once (makes
`~/bot-ws` with links into this repo; the scripts find it). Vendor STEP models are not in the repo (licences):
only the fit / wire checks and the "real parts" view need them.

CAD change workflow: `python tools/cad_baseline.py /tmp/before.json` → edit → `cad_baseline.py /tmp/after.json`
→ compare (refactors must change nothing). Then `body_case.py` → `assemble.py` → `export_print_data.py` →
`export_viewer_real.py` → `dump_wiring.py` → `wire_check.py` (must be clean except the known Pi OTG/Dupont contacts)
→ `real_assembly.py` (6 known contacts: mics, camera, OTG) → ERC → site builders.

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
  30/30), 52/60 within 12 s; docking 34/40 (was 37/40). Fix in progress: learned stand-up (`ml-standup`), to be
  trained on the Mac; merge only if `ml/evaluate.py --compiled` clearly beats the hand-tuned one.
- Demo videos in `site/files/` still show the pre-raise body.
- Motors: cheaper alternatives were reviewed (StepperOnline, PGFUN, VXB, gimbal motors): none fits; the only
  candidate is a low-KV outrunner + ODESC driver, not pursued.
