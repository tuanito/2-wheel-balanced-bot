# Balancing robot V1.1: shape C + voice — simulation, ESP32 firmware, Pi software

**V1.1** (current) = V1 (shape C, wheels under the body, LD19 lidar on top) + microphones, speaker and voice orders
through Claude. V2 (side motors, no lidar, low battery) is archived on the viewer's V2 page; the flip work done on it is
not part of V1.1's focus. Step-by-step setup: **INSTALL.md**.

```
core/            controller.cpp/.hpp + gains.hpp: balance (LQR), speed & heading control, lying-side detection,
                 stand-up, safety. Plain C++17. The SAME files run in the firmware and in the simulation.
sim/             MuJoCo model of shape C built from the CAD (build_model.py), LQR design (design_lqr.py),
                 the simulator harness (botsim.py), tests (run_tests.py), navigation test (nav_sim.py), results/
firmware/esp32/  PlatformIO project for the ESP32-S3: CAN motors (LK protocol), MPU6050, sonar, Pi UART link,
                 battery + charger sensing, arm button, status LED. Builds (pio run): RAM 6.5 %, flash 10 %.
pi/              Raspberry Pi Zero 2 W: ESP32 link, LD19 lidar driver, mapping + scan matching, obstacle
                 avoidance, frontier exploration, go-to-point, camera stream, LED eyes, phone web app, voice (mics, speaker,
                 Claude). 20 tests pass.
cad/             printable parts (STL + STEP) + assembly STEP: in the separate V1.1_print_files.zip download
cad_scripts/     CadQuery scripts that generate the case, the assembly check, the wheel adapter and the BOM.
```

## Voice: talk to the robot, Claude gives the orders

Hardware, all on the Pi's I2S bus (`dtoverlay=googlevoicehat-soundcard`, same mic + amp pair as the Google AIY Voice Kit):
- 2 x ICS-43434 MEMS mics in the front panel, 124 mm apart (Y ±62, Z 100), 65 dBA SNR, behind Ø1.5 ports.
- MAX98357A 3.2 W class-D amp on a tab of the printed speaker ring.
- 40 mm 4-ohm speaker on the RIGHT side wall above the wheel arch (centre X −2, Z 86), firing sideways through a
  grille (the top belongs to the lidar). The right carrier plate is gone; the charge diode + fuse sit inline in the lead.
Wiring: BOM, "Wiring" sheet.

Flow (`pi/robot/audio.py`, `speech.py`, `voice.py`):
1. Mics at 48 kHz -> high-pass -> 16 kHz -> two-mic beam steered at the talker (GCC-PHAT) -> speech gate -> utterance.
   Speech from 2 m arrives ~25 dB above the mic noise; the beam adds ~3 dB against room noise and gives the direction.
2. Speech-to-text: offline Vosk on the Pi, or a Whisper server (e.g. faster-whisper on the Jetson) via `stt_url`.
3. "Hey robot, ..." (or the Talk button / text box in the phone app) -> Claude (`claude-haiku-5-5`) with tools:
   drive, turn, stop, go_to (lidar map), explore, flip, set_eyes, headlight, follow (a person, by name if taught), who_is_there (AI camera), look (camera photo), status.
   Limits enforced on the robot (0.6 m/s, 6 s per drive; obstacle avoidance and the sonar stop stay on).
4. Claude's answer is spoken (piper voice, or espeak-ng). The mics are muted while it talks. "Stop!" is handled
   locally, instantly, and interrupts whatever is running.

Setup: `pi/install.sh` (overlay, Vosk + piper models, `/etc/robot.env`); put `ANTHROPIC_API_KEY=...` in
`/etc/robot.env` (root-only, loaded by robot.service). Settings: `"voice"` in `pi/config.json`.
Not tested on hardware yet: the sound card, mic gain at 2 m in your room, piper speed on the Pi Zero 2 W, recognition
with the motors running (tests use synthetic audio and a fake Claude client). The sim now also clips the IMU at its
real range (±2000 dps, ±16 g).

## Stress check (FEA, CalculiX)

`fea/fea.py` meshes the floor + motor-wall block (gmsh, tet10) and solves it with CalculiX: peak motor torque
2.3 N·m per motor plus 25 N per wheel (≈ 3 g), loads through the 4 motor screws, supports at the 4 floor bosses of the
front panel / back cover screws. `python3 fea/fea.py reinf` (current design, shelf screwed), `NOSHELF=1` to leave the
shelf out, then `python3 fea/fig2.py` for the pictures.

- The floor + motor walls are a **separate piece** from the shell (the wheel wells cut the floor): the front panel,
  back cover and the screwed battery shelf hold them together. Tighten all of them.
- Before (shelf only sliding on dovetail rails): max ~20 MPa at the wall roots and front floor bosses, walls splay,
  motor axis tilts 3.3°. The wheel's 61 mm lever arm, not the torque, does this.
- Now (shelf screwed to the wall tops with 4 × M3, 45° root fillets, wall-to-boss ribs): max 5.4 MPa, motor axis tilts
  0.24°. (Computed with the cable slot open to the top; the final closed window only adds material.) PETG layer bond is ~20–30 MPa.

## USB-C charging socket

In the left side wall above the wheel (X 8, Z 66), so the robot can be charged standing, lying face-down or on its back. A plugged-in USB-C cable clears the tyre by 8 mm; the IP2368 carrier plate on that wall now ends 20 mm below its screw line (z 72) to leave room for the socket.

## Panel screws

Front panel, back cover and contact plate use countersunk hex-socket M3×10 (ISO 10642), 0.2 mm below the surface, so
nothing sticks out to mark the floor when the robot falls. The panels are 1.5 mm thicker (4.5 mm) on their inside
around each screw so 2.4 mm of plastic stays under the head; the body's bosses moved 1.5 mm inwards to make room.
Stress check after the change: 5.3 MPa peak (was 5.4), motor-axis tilt 0.23°. Inserts: only M3 and M2 (the Pi and the
IMU take M2 screws with a washer). Perfboard: standard 5 × 7 cm, 24 × 18 holes.


The three M3 screws holding the front and back panels to the body sit 2 mm further from the panel edges than before (top front (Y_IN-6, 75), top back (76.6, 119.6), floor (40, Z_FLOOR+5.5)), so there is more material around each hole.

## Motor plugs, clearances

- The MF6015's connectors (a 6-pin and a 3-pin header) sit in a 15 mm bay on **top** of its Ø49 × 8 mm rear boss;
  the plugs go in from above. Mount the motors with that bay pointing up, plug them in **before** the shelf, and thread
  the leads through the window in each wall (24 x 16 mm, rounded) into the centre gap; a 2 mm notch above the bay lets
  the 3-pin plug go straight down. (An open slot printed as a 20 mm overhang: the window prints as a 16 mm bridge.)
- The shelf sits at z 37 (4 mm higher than before): 4 mm of air above the plug leads.
- 3 mm of air all around each motor (front, back, floor; 8 mm to the shelf): the body is 67 mm deep (was 62).
- Motor pockets are teardrops pointing to the back (the body prints on its front face): no flat overhang.

## Fit check with the real part models (Nov 2026)

`cad_scripts/real_parts.py` places every bought part from its real 3D model; `real_assembly.py` checks each one
against every printed part and every other bought part; `wiring.py` + `wire_check.py` route and check the wiring.
`models/fetch_models.sh` downloads the models. Sources:

| Part | Model |
|---|---|
| AI Camera (Camera Module 3 STEP as proxy: same PCB + holes, 0.4 mm deeper) | STEP Raspberry Pi RP-008154 |
| HC-SR04 sonar | STEP FreeCAD-library (community, 'original size') |
| ICS-43434 mic L | STEP Adafruit 6049 |
| ICS-43434 mic R | STEP Adafruit 6049 |
| Speaker 40 mm (Adafruit 3968) | STEP Adafruit 3968 |
| MAX98357A amp (Adafruit 3006) | STEP Adafruit 3006 (terminal block not in the model) |
| Pololu D24V50F5 regulator | STEP Pololu 0J1437 |
| LD19 lidar | STEP Waveshare (LDROBOT LD19) |
| Perfboard 5 x 7 cm + female headers | DRAW (standard 2.54 perfboard, 8.5 mm female headers) |
| ESP32-S3-DevKitC-1 | DRAW Espressif DXF v1.1 + WROOM-1 STEP |
| SN65HVD230 CAN board | STEP Waveshare (on 2.5 mm spacers: solder tails) |
| Raspberry Pi Zero 2 W | DRAW Raspberry Pi RP-008358 (heights from connector datasheets) |
| Battery Tattu R-Line 6S 1400 (listed 78 x 39 x 42) | LIST envelope |
| 6S BMS board (listed 88 x 34 x 3.5) | LIST envelope |
| IP2368 USB-C PD board (est. 35 x 47 x 10) | LIST envelope |
| 90° micro-USB OTG plug (cable to the LD19 adapter) | LIST envelope |
| LED eye panels 2 x (listed 26 x 26 x 3) | LIST envelope |
| GY-521 IMU (listed 21 x 16 x 3) | LIST envelope |
| PDB + 30 A fuse (est. 16 x 13 x 9) | LIST envelope |
| Power switch board Pololu 2813 + 12 mm power button | LIST envelope |
| Arm button 12 mm (listed) | LIST envelope |
| USB-C panel socket CAB-15455 (listed head 22 x 14 x 8) | LIST envelope |
| LD19 USB adapter board (est. 18 x 12 x 6) | LIST envelope |
| Charge diode + fuse, inline (est.) | LIST envelope |
| Motor plugs L (from LK drawing) | DRAW LK drawing, plug sizes estimated |
| Motor plugs R (from LK drawing) | DRAW LK drawing, plug sizes estimated |

STEP = manufacturer or published model. DRAW = built from the official mechanical drawing (Pi Zero 2 W: no official
STEP exists; ESP32-S3-DevKitC-1: Espressif DXF + the WROOM-1 module STEP). LIST = no model published: a box of the
listed size. Result: no clashes (only parts seated on their posts/pockets, and the OTG plug in its port).

What the real models changed:
- **Camera**: the lens is 6.15 mm off the centre of the hole pattern and 8.1 mm deep. The 4 posts moved up 6.15 mm
  and are 5.6 mm long (lens 0.5 mm behind the front face: it no longer touches the floor when the robot falls forward).
- **Sonar** (FreeCAD-library model): holes 41 x 17 (was 42 x 17), transducers 27 apart (was 26), posts Ø5.4 (Ø6.4
  clipped the transducers). Measure your board.
- **Microphones**: the mic package is 0.76 mm off the board centre: pockets offset so the port sits on the hole; both
  boards the same way up; clamp wire pass moved over the solder pads.
- **Regulator**: the Pololu's diagonal holes are mirrored compared with the old tray; tray holes fixed.
- **Speaker**: real flange 3 mm (Ø40) then a Ø34 step: ring bore 35 mm, ring face 3 mm from the wall. Speaker is
  25 mm deep (was assumed 20): fits.
- **Amp**: tab 1.5 mm lower (the board touched the body's inner top-corner curve).
- **LD19** (Waveshare STEP): 54 x 46 x 31 with its 3 mounting ears; the 3-hole pattern matches the top exactly.
- **CAN board**: solder tails on its back: mount it on 2.5 mm spacers.
- **BMS**: blocked the sonar header when standing in front of the battery: now on the battery's back face, inside the
  strap. Battery moved 2.5 mm back.
- **ESP32 USB-C**: 24 mm inside the right wall, the side slot was useless and is gone. Flash with the back cover off
  (or from the Pi with esptool over the existing UART, adding EN and GPIO0 wires).
- **OTG**: a straight micro-USB OTG adapter hits the left wheel arch: use a 90° OTG cable (10 cm).

## Wiring in place (routed and collision-checked)

Every conductor is modelled end to end, with its plug at each end. Lengths are as routed; add ~30 % slack when cutting. Viewer: "Wiring" view (tap a wire to see both ends). Wiring page: board pictures (perfboard to scale, a soldering guide) and the pin list.

| Harness | Conductors | From | To | Routed |
|---|---|---|---|---|
| Battery -> BMS | 3: B+, B-, Balance (7-wire, JST-XH) | Battery XT60 + balance leads | BMS input pads + JST-XH | 3 cm |
| BMS -> 30 A fuse -> power switch | 2: P+ (30 A fuse inline), P- | BMS output pads | Power switch VIN + P- splice | 15 cm |
| Power switch -> PDB | 2: BAT+ switched, BAT- | Power switch VOUT + P- splice | PDB input | 20 cm |
| Power button -> switch | 3: button A, B, switch GND tap | 12 mm button + splice | Switch pushbutton pads + GND | 6 cm |
| ESP32 GPIO21 -> switch OFF | 1: OFF | Perfboard J9 | Switch OFF pin | 15 cm |
| PDB -> motor L power | 2: V+, V- | PDB output | Motor L main plug | 6 cm |
| PDB -> motor R power | 2: V+, V- | PDB output | Motor R main plug | 6 cm |
| PDB -> 5 V regulator | 2: VIN (3 A inline fuse), GND | PDB output | Pololu D24V50F5 VIN/GND | 4 cm |
| 5 V regulator -> perfboard J1 | 2: 5 V, GND | Pololu VOUT/GND | Perfboard J1 (XT30, 15 A) | 14 cm |
| Perfboard J5 -> Pi (5 V, GND, UART, eye data) | 5: 5 V, GND, ESP TX (GPIO17) -> Pi RXD, Pi TXD -> ESP RX (GPIO18), Eye data: Pi MOSI -> 74AHCT125 | Perfboard J5 | Pi GPIO header (Dupont) | 7 cm |
| ESP32 -> CAN board (perfboard jumpers) | 4: 3V3, GND, CAN TX (GPIO4) -> CTX, CAN RX (GPIO5) <- CRX | ESP32 header (solder side of the perfboard) | CAN board 4-pin header | 7 cm |
| CAN bus -> motor R | 2: CAN-H, CAN-L | CAN board screw terminal | Motor R main plug (CAN-H/L) | 13 cm |
| CAN bus -> motor L (branch) | 2: CAN-H, CAN-L | Motor R plug | Motor L main plug | 6 cm |
| Perfboard J3 -> IMU | 4: 3V3, GND, SCL (GPIO9), SDA (GPIO8) | Perfboard J3 | GY-521 header (Dupont) | 17 cm |
| Perfboard J2 -> sonar | 4: 5 V, TRIG (GPIO10), ECHO (GPIO11 via 1k/2k), GND | Perfboard J2 | HC-SR04 header (Dupont) | 7 cm |
| Perfboard J4 -> eye R | 3: 5 V, DIN (via 330 ohm), GND | Perfboard J4 | Eye R panel pads (through the retainer's wire pass) | 8 cm |
| Eye R -> eye L (chain) | 3: 5 V, DOUT -> DIN, GND | Eye R output pads | Eye L input pads | 7 cm |
| Pi I2S -> I2S junction | 7: BCLK (GPIO18), WS/LRCLK (GPIO19), Mic DOUT -> Pi DIN (GPIO20), Amp DIN (GPIO21), 3V3, 5 V (amp), GND | Pi GPIO header (Dupont) | I2S junction board (on the battery) | 6 cm |
| I2S junction -> mic L | 6: 3V, GND, BCLK, DOUT, WS, SEL -> GND (left) | I2S junction | Mic L pads (through the clamp's wire pass) | 6 cm |
| I2S junction -> mic R | 6: 3V, GND, BCLK, DOUT, WS, SEL -> 3V3 (right) | I2S junction | Mic R pads (through the clamp's wire pass) | 12 cm |
| I2S junction -> amp | 5: LRC (WS), BCLK, DIN, GND, 5 V | I2S junction | Amp pads (Adafruit 3006) | 14 cm |
| Amp -> speaker | 2: SPK+, SPK- | Amp screw terminal | Speaker solder tabs | 3 cm |
| Camera FPC (22 -> 15 pin) | 1: FPC (15/22 conductors) | Camera FPC connector | Pi CSI connector | 10 cm |
| Pi OTG -> LD19 USB adapter | 1: USB (90° OTG cable) | Pi OTG port (90° plug) | LD19 USB adapter | 4 cm |
| LD19 -> USB adapter | 4: P5V, GND, Tx, PWM | LD19 lead (ZH1.5 4-pin) | LD19 USB adapter | 15 cm |
| Dock pads -> diode + fuse | 2: Pad + (charge in), Pad - | Charge pads (ring lugs inside the cover) | Inline 5 A fuse + SB560 diode | 9 cm |
| Diode -> BMS P | 2: SB560 cathode -> P+, P- | Diode output | BMS P+/P- | 5 cm |
| IP2368 <-> side USB-C | 1: USB-C cable (CAB-15455) | Panel USB-C socket (left side wall) | IP2368 USB-C | 3 cm |
| IP2368 -> BMS P | 2: BAT+, BAT- | IP2368 BAT pads | BMS P+/P- | 4 cm |
| Arm button + LED -> perfboard J6 | 5: Button (GPIO12), LED R (GPIO38), LED G (GPIO39), LED B (GPIO40), GND | Arm button + RGB LED | Perfboard J6 | 10 cm |
| VBAT sense -> perfboard J7 | 1: VBAT sense (switched BAT+) | Power switch VOUT | Perfboard J7 | 9 cm |
| Dock sense -> perfboard J7 | 1: Dock sense (pad +, before the diode) | Fuse + diode IN | Perfboard J7 | 6 cm |
| USB sense -> perfboard J7 | 1: USB VBUS sense | IP2368 VBUS pad | Perfboard J7 | 10 cm |
| PDB -> headlight driver | 2: VIN (1 A inline fuse), GND | PDB output | Headlight driver VIN/GND | 11 cm |
| Pi -> headlight driver (PWM) | 2: PWM (GPIO12), GND | Pi GPIO header (Dupont) | Headlight driver PWM pads | 8 cm |
| Headlight driver -> headlight R | 2: +12 V, switched - | Headlight driver | Headlight R pads | 10 cm |
| Headlight driver -> headlight L | 2: +12 V, switched - | Headlight driver | Headlight L pads | 2 cm |

CAN bus: CAN board terminal H/L -> motor R CAN-H/CAN-L -> motor L (one twisted pair, daisy-chained). The Waveshare board carries 120 ohm; measure H-L at the motor L plug with power off: ~60 ohm = both ends terminated, ~120 ohm = add 120 ohm there. Motor plug pin order is taken from the LK drawing: check it against the motor label or the supplied cable before crimping.

## Electrical rules check (elec/)

`elec/netlist.py` models every wire of the wiring above plus what is inside each board (perfboard traces, dividers,
level shifter, Pi header, BMS, PDB, power switch). `elec/erc.py` checks it: shorts between supplies, one common ground,
every power input supplied and in its voltage range, every logic input driven, no two outputs on one net, no input
above its maximum (resistor dividers are solved), firmware `config.h` and Pi `install.sh` pin maps against the wiring,
the CAN pair, current per wire and per connector contact (loads: Pi, lidar, amp, eyes at the software brightness cap,
motors at the stand-up peak from the sim), rail budgets and fuses. Output: `erc_report.json`, a netlist `robot.net`.
`elec/test_erc.py` injects 8 typical wiring mistakes (TX/RX swapped, loop key across + and -, echo without divider,
missing ground, CAN H/L swapped, mic on 5 V, eye data from a 3.3 V pin, sense wire missing): all 8 are reported.

Found and fixed in the CAD wiring:

| Finding | Fix |
|---|---|
| **Loop key would short the battery**: BMS P+ and P- landed on the two contacts of the panel XT60, which the key bridges | Contact A = BAT+ from the fuse, contact B = BAT+ to the PDB, negatives spliced beside the socket |
| GPIO1/6/7 sense dividers had no input wires (ADC would read 0 V: "battery low" after 5 s) | 3 sense wires to a new plug J7: XT60 contact B, fuse+diode input, IP2368 VBUS |
| J1 carried the whole 5 V rail (up to 4.7 A) through a 3 A 2.54 mm contact | J1 = XT30 PCB plug |
| 20 AWG regulator feed only protected by the 30 A fuse | 3 A inline fuse in the regulator VIN lead |
| Status LED G and B without series resistors | 330 ohm on all three |
| Perfboard plugs J2/J3 overlapped, eye-data pin inside J5, J5/J6 off the board edge, DevKit headers off the 2.54 grid | New perfboard layout on the hole grid (`case/perfboard_layout.py`), 3D-checked against the DevKit, its headers and the CAN board |

Still to check on the bench: CAN termination (60 ohm between H and L with power off), the LK cable's contact rating
(~9 A per motor for under a second at stand-up), and the 5 V rail at 93 % of 5 A in the worst case (Pi + camera +
lidar + amp peak + both eyes at their cap): fine for peaks; lower `MAX_BRIGHT` in `pi/robot/eyes.py` if the rail sags.

## Firmware in the loop (hil/)

The real ESP32-S3 firmware runs in Espressif's QEMU (`-machine esp32s3`, instruction-counted clock ~250 MIPS) against
the MuJoCo robot. Build env `esp32s3_sim` is the same code; only what QEMU does not emulate is replaced
(`src/sim_hil.h`): the IMU (I2C) and the CAN controller exchange frames with the physics host over UART0, which
advances the physics by exactly the time the firmware measured. The host plays two LK motors with the LK V2 protocol
(0x88/0x80 echo, 0xA1/0x9C state replies, 0x280 multi-motor torque) and the battery-limited motor model of the sim;
the Pi side is the real `pi/robot/link.py`. Run: `hil/build_flash.sh`, then `python3 hil/run_hil.py`.

Bugs it found in `main.cpp` / `lk_motor.cpp` (the MuJoCo tests only ran the controller core, never these files):

| Bug | Effect on the robot | Fix |
|---|---|---|
| CAN timeout used `now - last_ms` with `now` read before the replies were stamped: a reply newer than `now` wrapped to ~4 billion ms | Spurious motor fault (disarm, fall) whenever a millisecond tick lands inside `poll()`: likely seconds after the motors switch on | Signed age |
| Same pattern in the Pi command watchdog | Random one-cycle "stop" commands while driving | Signed age |
| Motor "off" (0x80) sent once; lost on the bus -> motors keep the last torque | After a CAN glitch the robot falls with the wheels still driving | Re-send "off" every 50 ms while off |
| 0x88 (motor on) echo parsed as a state frame | One-cycle encoder/speed jump to 0 when the motors switch on | Only 0xA1/0x9C carry state |
| Encoder range fixed at 16 bit; the LK manual allows 14/15/16 bit | With a 14-bit encoder: 1.10 m/s instead of 0.5, 236 mm drift | Range detected at run time (wrap size + speed ratio); boot log prints it |

Results with the fixed firmware, 3 randomised robots each (`hil/results.json`):

| Test | Result |
|---|---|
| Stand up face-down / back-down | 6/6, 0.47-0.50 s from arming to balancing |
| Pushes 1.0 / -1.0 / 1.5 N·s while balancing | 9/9 recovered |
| Drive 0.5 m/s for 3 s, stop, turn 90° | 0.47-0.48 m/s true, odometry within 5 %, turn 86-89° |
| Pi goes silent while driving at 0.5 m/s | Stops driving (watchdog 300 ms) and keeps balancing: < 3 cm/s 2.5 s later |
| CAN unplugged 100 ms | Fault after 20-24 ms, motors off 22-28 ms after the bus is back |
| 14 / 15 / 16-bit motor encoders | Detected correctly, same driving and balance |
| Control period | 2.0 ms median, 2.3-2.5 ms 99th percentile |

Not covered: the I2C IMU driver, ADC, LEDC and sonar timing (QEMU has no model for them), and the Pi's Linux side.

## Testing the CAN driver and code (bench/)

See `bench/README.md`. In short: `firmware/esp32/test_host/run_tests.sh` (54 PC unit tests of `lk_motor.cpp`: frame
packing, reply parsing, encoder unwrap and range detection, timeouts; they found and fixed a reply-gap unwrap bug), then
on the bench with a USB-CAN adapter: the CAN probe firmware (`esp32s3_canprobe`: transceiver self-test, motor scan,
direction and encoder checks), `bench/can_monitor.py` (decoded live traffic), and `bench/bench_hil.py`: the robot
firmware (`esp32s3_bench`, real TWAI) against two virtual LK motors over the real bus, with the physics on the PC.

## Simulation results, V1.1 (with the audio parts, every run randomised)

| Test | Result |
|---|---|
| Stand up from face-down / back-down | 30/30 and 30/30 (all on the first try), median 0.17 s |
| Stand still, 10 s | drift ≤ 3 mm |
| Push (1 N·s, front and back) | 20/20 recovered |
| 1 m/s, then stop | stops within 0.6 m, max lean 17° |
| Knocked over (2 N·s) | falls, detects the side, stands up again |

Mass 1611 g (deeper body, reinforcements, shelf screws), centre of mass 34.4 mm above the axle. Gains: `design_lqr.py 0.06 0.6 10`.
Re-run after the last CAD change (panel screws moved 2 mm inwards, Pi moved, 1615 g, CoM 34.4 mm): stand-up 30/30 face-down and 29/30 back-down (one randomised run failed), pushes 20/20, 10 s balance drift ≤ 3.3 mm.

**Power button instead of the XT60 loop key (Nov 2026).** A 12 mm momentary button on the top (front-right, the mirror
of the arm button) drives a Pololu Big Pushbutton Power Switch HP (#2813, 16 A continuous, 90 A peak, 4.5–40 V): push
on, push off, the battery current goes through its MOSFETs, never through the button. The ESP32 can switch it off
(GPIO21 → OFF): battery below 3.3 V/cell for 30 s while disarmed and not charging → telemetry flag 32 → the Pi halts
cleanly → 20 s later the ESP32 pulses OFF (`src/power_off.h`, host test 10 checks). The chargers feed the BMS before
the switch, so it charges while off. For storage, still unplug the battery's XT60 (a MOSFET switch is not an isolator). The switch board sits inside the back cover above
the Pi; its wires run behind the perfboard (battery side) and over the Pi (to the PDB, button and GPIO21).

**Taller shelf for the motor leads + a back cover that unplugs (Nov 2026).** The motor plugs' leads leave the plug
straight up; with the battery shelf 8 mm above the plug they had to kink 90° into the wall window. The shelf went up
10 mm (SHELF_Z 37 → 47; the wall windows to z 43) and the body with it (Z_TOP 142 → 152): the leads now rise out of the
plug and bend back down into the window with ~7 mm radius (`MOTOR_ARC` in case/wiring.py). Everything above the
battery is placed from its top (`BATT_TOP` in body_case.py), so the perfboard, Pi, power switch and CAN board moved
up with it; the front panel items stayed. The shelf's front corners are cut away (`SHELF_FRONT_NOTCH`) for the
headlight driver and collars. **Working inside:** the back cover with its boards is one module: two XT60s (genders
keyed, the battery side female), a 3-pin JST-XH for the power button, a 2-pin JST-XH for CAN, and the J-headers / Pi
Dupont plugs that were already there. Every wire that crosses has 4 cm of slack, so the cover lies down beside the
robot and the battery slides out the back.
*Cost of the extra height (sim, Nov 2026):* 1692 g, CoM 40.6 mm above the axle (was 37.1); LQR gains redesigned
(`design_lqr.py 0.06 0.6 10`). Balance and pushes unchanged (drift ≤ 3.7 mm, 20/20 pushes), missions OK. **Stand-up
got harder:** 25/30 face-down and 23/30 back-down within 5 s (was 30/30), 52/60 within 12 s; the failures are kicks
that reach the motors' speed limit before the taller body is up. A stronger kick (1.9 N·m) did not help. Docking sweep: 34/40 docked (was 37/40), all 34 undocked, no falls.

**Bigger battery + taller body (Nov 2026).** 6S 1400 mAh (Tattu R-Line, 78 × 39 × 42, 222 g, ~31 Wh: +33 % runtime)
instead of the 1050 (34 mm tall, 180 g). It is 8 mm taller and would have hit the perfboard above it, so the body top
went up 8 mm (Z_TOP 134 → 142; perfboard, lidar, buttons, IR hoods move up with it). The eyes moved 8 mm down and
6 mm outwards (EYE_Z 112 → 104, EYE_Y 32 → 38) and the headlights 8 mm down (LIGHT_Z 76 → 68, driver board with them),
so less of their light spills next to the camera lens. Now 1680 g, CoM 37.4 mm above the axle (+3 mm) [after the +10 mm shelf: 1692 g, CoM 40.6 mm]; LQR gains
redesigned (`design_lqr.py 0.06 0.6 10`). Re-run: stand-up 30/30 face-down and 30/30 back-down (29 first try each),
pushes 20/20, 10 s balance drift ≤ 3.7 mm; no interferences in the assembly check.

## Simulation results (shape C, every run randomised)

Each run varies the parameters you can't know exactly before building: mass ±8 %, centre of mass ±4 mm (x) and −6…+8 mm (z), inertia −15…+20 %, tyre grip 0.5–1.2, battery 21–25 V, motor torque ±10 %, 2–4 ms actuation delay, IMU bias, and 1.5° of IMU mounting tilt. The motor model is the LK drive: 16.5 A current limit, back-EMF, 0.14 N·m/A.

| Test | Result |
|---|---|
| Stand up from face-down | 40/40 (side detected 40/40, 39 on the first try), median 0.18 s |
| Stand up from back-down | 40/40 (side detected 40/40, 40 on the first try), median 0.17 s |
| Stand still, 10 s | drift ≤ 4 mm, pitch wobble 0.01° |
| Push (1 N·s, front and back) | 26/26 recovered without falling |
| Accelerate to 1 m/s, then stop | speed reached; stops within 0.6 m; max lean 17° |
| Turn +90°, +180°, −135°, to 30° | heading error < 1° (nominal), < 3.5° (randomised, from gyro bias) |
| Knocked over (2 N·s) | falls, detects the side, stands up again |
| Explore an 8 × 8 m room, 150 s | 0 collisions; map built; final position error 3 cm |

## What the simulation taught us (and how the code handles it)

- **Stand-up = kick + energy shaping + catch.**
  - The kick: full push towards the side the top points to.
  - Energy shaping: the motors then trim the body's energy so it arrives upright with almost no speed left.
  - The catch: LQR takes over below 20°.
  - The stand-up needs about 0.7 m of free floor, and the wheels briefly reach ~4 m/s.
- **It learns its own robot.**
  - If a stand-up falls back, the next attempt aims higher. If it overshoots, it aims lower.
  - The firmware stores this correction in flash.
- **Low centre of mass means a big lean when accelerating.**
  - The wheel torque leans the body much more than on a tall robot: 0.6 m/s² already gives about 9°.
  - Acceleration is therefore limited to 0.6 m/s² and yaw rate to 1.2 rad/s. In the simulation, wobble grows above about 1.8 rad/s; check this on the real robot.
- **The lidar tilts with the body.**
  - The robot stands leaning about 4° (CoM 2 mm behind the axle). When braking it leans up to 17°.
  - The scan plane then hits the floor at 1–3 m. `level_scan()` removes those rays.
  - The map is only updated while the body is within 5° of level.
- **Scan matching corrects heading only.**
  - Wheel odometry was good for distance. Letting the matcher also shift position made the map worse in tests.
- **The sonar sees what the lidar can't.** An 18 cm post below the 21 cm scan plane is still avoided.
- **Gyro bias is learned whenever the robot is still** (at boot, lying down, or charging).

## Before the first power-up (bench checklist)

0. **Power switch:** with a meter (battery out), VIN-VOUT open and no continuity between + and −. Battery in: one push of the top button switches on (board LED), another off. Boot log: `encoder range detected: N counts/turn` once the wheels have turned.

1. **Motors (LK PC tool + Neo2 adapter).**
   - Set IDs 1 (left) and 2 (right), CAN 1 Mbps.
   - With the wheels off the ground, send +0.2 N·m to each motor. Both must turn so the robot would roll forward. If not, flip `DIR_LEFT` / `DIR_RIGHT` in `firmware/esp32/include/config.h`.
2. **Encoder range.** Check that the 0xA1 reply's encoder spans 0–65535 per turn (`ENC_COUNTS`).
3. **IMU axes.**
   - Upright and still: the boot log must show acc ≈ (0, 0, +9.8).
   - Tilt the top forward: ax goes negative and gy is positive while tilting.
   - If not, change the `IMU_MAP_*` macros.
4. **Battery divider.** Compare the telemetry `vbat` with a multimeter, then adjust `VBAT_DIV`.
5. **CAN bus.** About 60 Ω between CAN H and L with the power off.
5a. **Motor-side CAN timeout (LK PC tool).** Set each motor's communication-timeout / "CAN disconnect protection" so it stops when commands stop (≈50–100 ms). Then, wheels off the ground, motors spinning at +0.2 N·m, unplug the ESP32's USB or hold its reset: both wheels must stop within that time. If the tool has no such setting, a crashed ESP32 leaves the last torque applied: tell me and the firmware gets a hardware watchdog that cuts the XT60 path.
5b. **CAN bus-off recovery.** Robot armed, wheels off the ground: pull the CAN plug from motor R for ~1 s and reconnect. The log shows `CAN bus-off #1: restarting the controller`, then `CAN back after … ms`; the robot disarms (motor fault) and arms again with the button.
5c. **TVS on the 6S rail.** 5KP24A soldered across the PDB output (band to +). At full charge (25.2 V) it must stay cold.
6. **First balance tests.** Hold the robot upright and press arm (button or app). Use the LiPo bag and a clear floor of at least 2 × 2 m.
7. **Lidar angle.** With a wall straight ahead, set `lidar_offset_deg` in `pi/config.json` so the wall reads at 0°.
8. **IP2368.** Measure 25.2 V at its battery output before connecting the pack.

## AI Camera: people, cats, follow me (Nov 2026)

The camera is now the **Raspberry Pi AI Camera** (Sony IMX500). Same PCB and holes as the Camera Module 3, so it uses the same 4 posts, 8.5 mm long now.
- **On the camera:** an SSD MobileNet (COCO) detector runs on the sensor at ~30 fps; the Pi only reads the boxes from each frame's metadata (`pi/robot/vision.py`). Kept: `person` and `cat`.
- **Who it is:** a few times a second the Pi crops the head of each person box and runs OpenCV's YuNet face detector + SFace face embedding, compared with the photos you taught it (web app: name + 3–5 face photos -> "Teach face"; stored in `data/people/<name>/`). Match threshold 0.36 cosine.
- **Follow mode** (web button "Follow me", or by voice: "follow Anna"): keeps the person centred and at ~1.5 m (box height 0.6 of the image), turns first when they're at the edge, turns slowly to search when nobody is in view. Obstacle avoidance stays on. A recognised person is then tracked by box overlap + clothing colour, so it keeps following when they turn away, and never switches to another recognised person.
- **Voice tools:** `follow {name?}`, `who_is_there` (names, cats, left/ahead/right).
- **Limits:** 66° × 52° view (vs ~102° before): the robot turns more. Manual focus: set it to ~1.5 m before assembly (the lens tool comes with the camera). Faces work face-on, in decent light, up to ~3 m; from behind, the clothing-colour lock carries on.
- Tests: `pytest` covers the follow controller, the tracker (including someone else walking through), face matching and the headlight PWM.

## Headlights (Nov 2026)

Two **COB LED discs** (Ø30 × 4 mm, 12 V 2.4 W, ~200–250 lm each) beside the sonar, for the camera at night: ~75 lux at 1 m, ~20 lux at 2 m (the eyes alone: ~8 / ~2 lux).
- **Mounting:** each disc sits in a collar on the inside of the front panel, 3 mm behind a 45° window (Ø24 -> Ø30, prints face-down without support), clamped by a printed retainer with 2 × M2×6 into inserts.
- **Driver:** Pololu D36V6F12 (6S -> 12 V, 600 mA) + IRLB8721 MOSFET on a 25 × 20 mm piece of perfboard, on 2 M2 bosses left of the sonar. 1 A inline fuse in its PDB feed. The Pi's hardware PWM0 (GPIO12, header pin 32) switches it at 1 kHz (no banding in the video); 100 k pull-down keeps it off while the Pi boots. Software cap 70 % (`headlight.py`): the discs sit in PETG.
- **Use:** the existing "Headlight" button / voice tool now switches the discs and the eyes' white light together.
- Checks: electrical rules (12 V rail 0.4 of 0.6 A, fuse vs 22 AWG), fit check with the real parts (clear), wire routing (clear).

## Face-first landings

The robot lands on its front when it falls, and every stand-up from face-down starts there. So:
- **Camera lens ~3 mm behind the face** (was 0.5 mm) behind a **1 mm polycarbonate window** in a pocket 0.5 mm below the face.
- **4 rubber bumpers** (Ø11 × 2.5, self-adhesive) on the face (Y ±48.5, Z 39.5 / 127 — just outside the eye wells, from `BUMPER_*` in body_case.py): the robot rests on them, not on the windows. Headlight discs are 3 mm back; the eye diffusers 0.8 mm back.

## Vertical charging dock, autonomous docking (Nov 2026)

The robot finds the charger itself, backs into it, leans back 8° onto it and charges with its motors off; it stands
up off it again on command. Simulated in `sim/dock_sim.py`: the real C++ controller + the real Pi docking code
(`pi/robot/dock.py`) + the dock as collision geometry, the rods and funnels as forces, a noisy 360-ray lidar.

**The dock** (`build_dock` in case/body_case.py; prints as foot + head + mast + 2 wings + 2 floor ridges, none needs support — the column is split where it steps back, and the face slab reaches down to that plane so its 176 mm-wide sides grow from the bed):
- An 80 mm column, 310 mm tall: the lidar finds its flat face, also with the robot leaning 8° from 0.6 m away.
- Face slab (leaning back 8°) with two sprung Ø4 brass rods and a centring tongue; soft side wings.
- **No magnets. A saddle under each wheel holds it instead:** rear lip 1.2 mm, a trough tilted 0.3 mm towards the front,
  front lip 1.8 mm starting 5.5 mm ahead of the axle, then an 80 mm lead-in (≤ 2°). The trough and front lip are a ribbed TPU 95A insert (0.4 mm ribs,
  grip for climbing out) that pushes up into a dovetail window in a PETG track frame; the frames drop into T slots in
  the floor plate. Leaning on the dock pushes the robot forward only ~0.6 N, so it stays put with the motors off.
- The column steps back 32 mm below the slab and the floor plate starts 62 mm behind the axle: room to roll back
  under itself when it leaves.

**Robot side:** 10 mm contact plate with two lead-in funnels (mouth stretched downwards: the plate drops ~6 mm onto
the rods as the robot leans back) down to **M5** brass countersunk heads (Ø9.3: contact up to ±4.5 mm off centre).

**Finding the dock: reflective tape + IR beacon** (`dock.py` on the Pi):
- *Tape:* two 25 mm retro-reflective strips on the column face, 49 mm apart, in 0.3 mm recessed guides. The LD19
  reports intensity per point: the strips read >200, walls and furniture 60–160. So the pilot locks onto the face by
  its tape (position ±few mm, angle from the two strip centres) and can't mistake a chair leg or a box for the dock.
  Range for a good fix: ~1.2 m.
- *IR beacon* (`firmware/dock_beacon`, ATtiny85, 38 kHz): code 'A' (600 µs bursts) from 4 LEDs all round the
  column top, code 'C' (1200 µs) from 1 LED in a tube on the face, a ±6° beam along the dock's axis. Robot: 4
  TSOP38238 receivers in black hoods on the top skin at (±15, ±54) mm — symmetric, fully on the flat top (`IR_RX_XY`), 90° windows facing the four diagonals — on GPIO13–16;
  the ESP32 decodes them (`ir_decode.h`, 100 ms windows) into the telemetry `ir` byte. Which receivers hear 'A' gives
  the bearing to ±22°; it carries from metres away (typically 3–5 m, more in line of sight).
- *Estimate:* every tape fix is a measurement of the same fixed dock pose in the odometry frame; they are averaged
  (close fixes weigh more) and odometry carries it between scans, so the steering doesn't jitter with lidar noise.

**Dock electronics** (`firmware/dock_beacon/`, schematic on the site: *Wiring diagram → Dock*): one ATtiny85 runs
the IR beacon and a rod switch. Safety extra-low voltage only (25.2 V from a certified, short-proof charger), a 1.1 A
PTC fuse, a TVS and a reverse-polarity MOSFET at the input. **The rods are dead unless the robot is on them:** a weak
sense current (≤ 0.25 mA) looks for the robot's own 112 kΩ detect divider; only after 0.5 s of that does a high-side
P-MOSFET put 25.2 V on them. A coin, key, hand, short or nothing keeps them off; a short while live switches off in
~10 ms (3 faults latch it off); every 2 s it drops the rods for 30 ms to check the robot is still there, so they go
dead within 2 s of it leaving. Without firmware the switch is held off by its pull-ups. The logic is unit-tested
(`test_dock_logic.cpp`, 16 checks) and modelled in the dock simulation.

**How it docks** (`DockPilot`, then `ST_DOCK_LAND` / `ST_UNDOCK` in the controller):
1. Known dock and far away: plan a path to its approach point first (the first dock saves its map pose in
   `data/dock.json`; low battery then docks by itself).
2. *Find:* turn on the spot until it sees the tape or hears the beacon. *Home:* turn to the beacon's bearing and drive
   to it (the closest lidar return near that bearing sharpens it); if it ends up behind or beside the column (something
   45 cm ahead, no tape), it circles the column at 50 cm until the tape shows up.
3. *Line up:* back off if too close to turn, drive to the point 45 cm out on the dock's axis, turn to face away.
4. *Reverse* at 7 → 4 cm/s, steering onto the axis to ±1–3 mm (the robot can't slide sideways, so the lidar has to
   centre it, not the funnels). The distance is corrected for the body's lean (the lidar is 16 cm above the axle).
5. With the axle on the ridge's crest, the ESP32 leans it back under control (a pitch target): it rolls back down
   into place as it leans. Contacts touch → damping only → still → CHARGING, motors off.
6. Undock: wheels roll back under the body (speed-limited) until upright, then normal balancing; it drives 0.4 m out
   over the ridge. If the contacts open again within a minute of docking, the Pi docks again.

**What the simulation changed on the way:** magnets strong enough to hold it made it spin its wheels leaving a
slippery floor; free wheels without anything in front let it roll out like a ladder → the ridge. A 4–6 mm wave, or
one raised 2 mm, made it stall and then surge into the dock; leaning from behind the ridge meant rolling down it
into the dock uncontrolled → 1.5 mm, floor level under the wheel, lean starts on the crest. Stopping hard after
the undock peel made the wheels hop and spin → hand over with the speed it has. The attitude filter trusts the gyro
while landing/undocking/docked (the plate bumps fool the accelerometer by up to 5°).

**Results** (every run randomised: mass, CoM, inertia, floor friction 0.4–1.0, battery, torque, delay, IMU bias
and tilt, lidar noise; 0.15 N rod friction; `sim/dock_sim.py`):
- *From close by* (40 starts anywhere 15–50 cm from the face, ±65° off the axis, any heading; tape + line-up):
  **36/40 docked and held charging with the motors off (28 first try), 36/36 undocked, none fell.** Docked lean
  8.9–9.2°, 11–52 s. Misses: 3 × contacts not meeting after 3 tries, 1 line-up that didn't converge (now capped at
  35 s per try). `results/dock_report.json`, `dock_sweep.png`.
  Re-run with the dock's rod switch modelled (rods live 0.5 s after contact, 30 ms re-check gaps): identical, 36/40.
- *IR homing from 1–4 m* (24 starts all round, behind the dock too; IR model: hoods ±55°, 5 m range with ±20 %
  spread, 5 % missed windows, line of sight by ray cast): **every run found the dock; 21/24 docked** (16 first try),
  26–65 s. Misses: 2 × contacts not meeting after 3 tries, 1 line-up that never converged.
  `results/home_report.json`, `home_sweep.png`.
- These are with the taller body and 1400 mAh pack (CoM +3 mm). The same sweeps on the shorter robot gave 37/40 and
  22/24 with fewer retries: the higher body makes the lean-back landing a little less sure. Worth tuning
  (`dock_lean_rate`, `X_LEAN`) once the real robot can be measured.
- Videos: `results/dock_5starts.mp4` (5 starts round the dock, all docked) and `results/undock_2runs.mp4` (leaving on a
  slippery and a grippy floor), made by `sim/dock_video.py` and `sim/undock_video.py`.
- Earlier saddle results (starts 0.6 m out facing away, lidar shape fit only): 33/41. The tape lifted it to 37/40
  from much harder starts. A 2.5 mm front lip docked at only ~6° lean (contacts barely touching). Results from before
  the MuJoCo degrees-vs-radians fix (floor bumps were a staircase) are void.

## Electrical checks in simulation (Nov 2026)

What the wiring check and the CAN unit tests could not see: real voltages, currents and bus timing. Scripts: `sim/power_sim.py`, `elec/can_ground_check.py`, results in `sim/results/power_report.json` and `elec/can_ground_report.json`.

**Battery sag at stand-up** (robot sim with the battery as open-circuit voltage + series resistance; 16 stand-ups per case, 4 charge levels × 3 pack resistances 35 / 75 / 120 mΩ):
- Stand-up works down to an empty pack (19.8 V) with a bad, 120 mΩ pack: 191/192 overall, 180 at the first try. The one failure (half charge, 75 mΩ) is the same ~1-in-30 miss the fixed-voltage tests show from the back, not a power limit: the empty/bad cases all got up.
- Peak current from the battery 25–33 A, above 30 A for at most 10 ms: the 30 A blade fuse (rated to carry 135 % for minutes) and the BMS's 40 A trip are not reached.
- Lowest bus voltage 16.8 V (empty + bad pack): the 5 V regulator (needs > ~6 V) and the motor drives (7.4–32 V) keep running. The firmware's low-battery filter ignores these few-ms dips.

**Braking energy (regeneration):**
- Braking sends up to 6 A back into the battery, up to 1.7 J per stand-up or hard stop. With a healthy BMS the pack absorbs it: the rail rises at most 0.7 V (25.9 V with a full, worn pack).
- If the BMS blocks charge current (a cell at its over-voltage limit, e.g. right after a full charge with an unbalanced cell), that energy has only the ~0.9 mF of capacitors on the rail: **0.18 J is enough to reach 32 V**, the drives' limit, in about a millisecond. Too fast for firmware.
- Fix, two layers: a **5KP24A TVS diode** across the PDB output (conducts from ~27 V, holds the rail ≤ ~30 V at 6 A; ~€1), and the firmware switches the motors off (motor fault, event `6S over-voltage … V: motors off`) when the rail reads > 26.8 V twice in a row. Motors off ends the braking current (back-EMF stays below the rail).

**CAN bus timing** (1 Mbit/s, 500 Hz control loop):
- 138 µs per frame worst case (bit stuffing); 3 frames per cycle while balancing = 21 % bus load (28 % while off). 9 retransmissions would still fit in one cycle.
- New in the firmware: **bus-off recovery**. A noise burst or a loose plug can make the ESP32's CAN controller shut itself off; before this fix it never restarted (robot falls, needs a reset). Now `LkMotors::service()` restarts it: ~1.4 ms + one cycle, well under the 20 ms CAN timeout, so a single bus-off doesn't even disarm. Logged as `CAN bus-off #n` / `CAN back after … ms`. Unit tests: 68 checks (bus-off → recovery → restart, a second bus-off, a stuck bus → motor fault, then recovery).

**Ground offset:** the CAN pair has no ground wire; the transceiver (ESP32 ground) and the motors (their V−) meet at the PDB. Worst case 60 mV between them (16.5 A in a motor's 6 cm 16 AWG return + 4.7 A in the perfboard's 14 cm 20 AWG ground); the SN65HVD230 tolerates −2…+7 V: 33× margin.

**Still for the bench:** the motors' own CAN timeout (5a), real reply delay, motor PWM noise into the IMU/mics/amp, a USB ground loop with the battery connected, and the BMS's real over-voltage threshold (read it in its datasheet: ≥ 4.25 V/cell wanted).

## Build and run

- **Firmware:**
  ```
  cd firmware/esp32 && ./sync_core.sh && pio run -t upload
  ```
- **Pi** (Raspberry Pi OS Bookworm Lite 64-bit). Copy the repo to `/home/pi/robot`, then:
  ```
  cd pi && ./install.sh
  ```
  Reboot, then open `http://<hostname>.local:8080` on the phone.
  - The web app has live video, a joystick, arm/disarm, turn 90°/180°, explore, auto stand-up and avoidance toggles, the headlight, and the live map. Tap the map to send the robot there.
  - Maps go to `data/map.png` and `data/map.npz`. Every lidar scan is stored with its pose in `data/scans/*.npz`.
- **Simulation:**
  ```
  cd sim && python3 build_model.py && python3 build_model.py --arena && python3 design_lqr.py 0.06 0.6 10
  cd ../core && g++ -O2 -std=c++17 -shared -fPIC controller.cpp sim_api.cpp -o ../sim/libbot.so
  cd ../sim && python3 run_tests.py 40 && python3 make_video.py && python3 nav_sim.py 150 --video
  ```
- **CAD and electrical scripts** (`cad_src/`, `elec/`). The electrical ones are plain Python; the CAD ones need
  [CadQuery](https://cadquery.readthedocs.io) and the bought-part models (`models/fetch_models.sh`, plus the MF6015
  STEP from the dealer's page, saved as `models/bought/mf6015.step`).
  ```
  python3 elec/erc.py && python3 elec/test_erc.py     # electrical rules check + its 8 mutation tests
  python3 cad_src/case/dump_wiring.py                 # -> wiring_data.json, wiring_diagram.json
  python3 cad_src/case/assemble.py                    # -> assembly/, sim/meshes/*.stl, sim/cad_mass.json
  ```
  These scripts were written on a machine that kept the sources in `/home/claude/case`, `/home/claude/wheel_adapter`
  and `/home/claude/robot`. They no longer assume that: `cad_src/case/robot_paths.py` works out where everything is
  from its own location, so the same code runs from a checkout at any path and from the original layout. Override any
  root with `ROBOT_CASE`, `ROBOT_WHEEL_ADAPTER`, `ROBOT_ROOT`, `ROBOT_OUT`, `ROBOT_MODELS` or `ROBOT_MF6015_STEP`.
  Generated artifacts go next to the case sources by default (git-ignored); set `ROBOT_OUT` to collect them elsewhere.
- **Tests:**
  ```
  cd pi && python3 -m pytest -q tests
  ```
  The tests compile the real firmware protocol header with g++ and check that it matches the Python parser byte for byte.

## Not tested yet (needs the hardware)

These can't be simulated: the AI Camera stream and its detections, the LED-eye panel wiring order (`serpentine`), the real LD19 byte stream, the real bus's reply delays, motor encoder scaling, ADC calibration, the IP2368 6S setting, and how the real tyres scrub when turning. The simulation's contact model is a rounded tyre touching at one point.

**Lower-latency video (optional).** MJPEG works in every browser. For WebRTC, install mediamtx with its `rpiCamera` source and point the web page's `<img>` to its WHEP page instead.

## Cheaper motors (simulation, `sim/wobble_compare.py`)

Same robot and firmware, 10 randomized runs each (battery 21–25 V, ±10 % torque, 1–2 cycles of CAN delay, IMU noise and bias).

| Motor (pair price) | Standing 10 s: pitch wobble p-p | Torque jitter | Push survived (1.5 / 2.0 N·s) | Drive 1 m/s, stop, turn | Stand-up |
|---|---|---|---|---|---|
| MF6015-V2 (current, ~$320) | 0.38° | 0.5 mN·m | 10/10 · 0/10 | 10/10 | yes |
| MF5015-V2 10T (~$240) | 0.39° | 0.6 mN·m | 10/10 · 0/10 | 10/10 | no (0.7 N·m) |
| DM-H6215, firmware speed from Hall counts (~$120) | 1.97° | 39.5 mN·m | 9/10 · 0/10 | 10/10 | no (~1.7 m/s top) |
| DM-H6215, drive's speed with 5 ms lag | 53° (limit cycle) | 1458 mN·m | 9/10 · 1/10 | 10/10 | no |

- MF5015-V2 balances exactly like the MF6015; the push limit is set by the controller, not the motor.
- DM-H6215: the coarse Hall encoder (84 counts/rev) is fine only if the firmware computes wheel speed from the counts over ~20 ms. Feeding the controller a delayed speed sample makes it oscillate (MF5015 too: 2 ms of lag already gives 3.6° p-p), so the LK driver must keep using the fresh speed in each reply.
- H6215 numbers use an estimated rotor inertia (600 g·cm²) and Kt from the no-load speed.
