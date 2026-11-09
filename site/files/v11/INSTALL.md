# Installation guide: balancing robot V1.1

From a box of parts to a robot that balances, stands up and takes voice orders through Claude.
Take it in order: each stage checks the one before it. Allow a weekend for the build, an evening for the software.

Everything referred to here is in `robot_software_V1.1.zip` and `V1.1_print_files.zip`.
Part numbers, prices and links: the BOM ("BOM" sheet). Every wire: the BOM's "Wiring" sheet.

---

## 0. What you need on your computer

- **PlatformIO** (VS Code extension or `pip install platformio`) to build and flash the ESP32-S3.
- **Raspberry Pi Imager** to write the Pi's SD card.
- **LKMTECH motor tool** (Windows) + a USB-CAN adapter (Neo2) to set up the motors once.
- Optional, for the simulation: Python 3.11+, `pip install mujoco numpy scipy`, and g++.
- A **Claude API key** (console.anthropic.com → API keys). Voice orders need it; everything else works without it.

## 1. Print the parts

All parts are in `V1.1_print_files.zip` (STL to print, STEP to edit). PETG, 0.2 mm layers, no supports.

| Part | Qty | Orientation / note |
|---|---|---|
| body | 1 | Front opening down. The file holds two pieces (shell + motor block): print both |
| shelf, tray, clamp_bar, carrier_plate | 1 / 1 / 1 / 2 | Flat side down |
| front, cover | 1 / 1 | Outside face down |
| contact_plate | 1 | **Inner face down** (cover side): funnels open upwards, no overhang |
| dock_foot | 1 | Standing, floor plate on the bed (123 mm), no supports; 3 walls / 15 % infill is plenty. 2 M3 heat-set inserts into the columns at the top of its back wall |
| dock_head | 1 | **Standing on the shelf's underside** (the face slab's bottom edge is on the bed too, 86 mm tall): no support |
| dock_mast | 1 | **Upside down** (closed top on the bed, spigot up, 133 mm): no support. The spigot pushes into the top of the head's column |
| dock_wing | 2 | **Standing on its back edge** (the side that bolts to the slab): no support; second one mirrored |
| dock_ridge | 2 | PETG, flat; print the second one mirrored. Drops into the T slots at the front of the dock's floor plate |
| dock_ridge_insert | 2 | **TPU 95A**, flat, ribs up, slow (~20 mm/s); second one mirrored. Pushed up into the frame's window from below |
| eye_retainer, eye_diffuser | 2 / 2 | Diffusers in **white** |
| ir_hood | 4 | **Black** PETG (light-tight), base down |
| speaker_ring | 1 | Flange down |
| mic_clamp | 2 | Flat side down |
| mf6015_hex12_adapter | 2 | Hex end down, 100 % infill, PETG or PA-CF. Screw-head seats print with the two-layer bridge trick at 0.2 mm layers: nothing to drill out |

Then poke through the 0.2 mm sacrificial layers in the screw recesses and press the heat-set inserts
(34 × M3, 17 × M2, 6 × M2.5, 2 × M4). Four of the M3 go into the tops of the two motor walls, for the shelf screws. Keep them square.

## 2. Configure the motors (before they go in)

With the LKMTECH tool and the USB-CAN adapter, one motor at a time:

1. Set **ID 1 = left**, **ID 2 = right**, CAN **1 Mbps**.
2. Set a current limit of 16.5 A.
3. Send +0.2 N·m to each with the wheel free. Note the direction. Both must turn so the robot would roll
   **forward** once mounted; if one doesn't, flip `DIR_LEFT` / `DIR_RIGHT` in `firmware/esp32/include/config.h`
   later.
4. Read the encoder: it must span 0–65535 per turn (`ENC_COUNTS` in `config.h`).

## 3. Assemble

Follow the BOM's "Build check" sheet. Short version:

1. Motors onto the centre wall (8 × M2.5×10, blue threadlocker), **connector bay pointing up**. Plug the 6-pin and
   3-pin leads into the bay from above, then thread them through the window in the wall into the centre gap.
2. Wheel adapters + wheels. Measure the wheel offset first; if it differs, set `NECK_L` in
   `cad_scripts/wheel_adapter.py` and reprint.
3. Floor tray: 5 V regulator, distribution board + 30 A fuse under the clamp bar, IMU on the axle line.
4. Left wall: IP2368 USB-C charger on its carrier plate. Shelf: lidar adapter board.
4a. **Front panel extras (Nov 2026):**
   - **AI Camera:** first set its focus to ~1.5 m with the supplied tool (look at a printed page at 1.5 m on the
     live view), then 4 × M2×6 onto the 8.5 mm posts. Glue the 19 × 19 mm polycarbonate window into the pocket on the
     outside (2 dots of clear glue in the corners, not over the lens).
   - **Headlights:** solder ~12 cm of red/black 24 AWG to each COB disc (+ outer, − inner pad), drop the disc into its
     collar from the inside (LEDs facing out), thread the wires through the retainer's hole, 2 × M2×6.
   - **Headlight driver** (10 × 8 holes cut from the 2 × 8 cm perfboard of the set): Pololu D36V6F12 (VIN, GND, VOUT),
     IRLB8721 laid flat (gate via 100 Ω from the PWM pad, 100 kΩ gate to GND, source to GND, drain = the discs' −),
     VOUT -> both discs' +. Pads at the corners: VIN/GND bottom-inner, PWM/GND bottom-outer, discs at the top.
     Drill Ø2.2 in the two bottom corners (19 mm apart), 2 × M2×6 + washers onto the bosses left of the sonar.
   - **Feed:** PDB + -> 1 A inline fuse -> VIN, PDB − -> GND (22 AWG, up the shelf cable hole). Pi pin 32 (GPIO12) ->
     PWM, pin 34 -> its GND (Dupont).
   - **Bumpers:** 4 rubber bumpers on the face, Y ±66 mm, Z 40 and 127 (about 23 mm in from each side edge).
5. **Audio (new in V1.1):**
   - Speaker into the speaker ring, ring onto the **right** wall over the grille (3 screws). The
     MAX98357A amp goes on the ring's tab with zip ties. 470 µF across the amp's 5 V input.
   - The two ICS-43434 mic boards into the front panel pockets, port side to the Ø1.5 holes, held by the
     mic clamps. **Left mic SEL → GND, right mic SEL → 3.3 V.** Twist the I2S wires with GND, keep them short.
   - Charge diode + 5 A fuse: inline in the dock-pad lead (heat-shrink), no plate on the right anymore.
6. Slide the shelf in flat on the wall tops and screw it down: 4 × M3×10 countersunk into the wall-top inserts.
   These screws are structural (they tie the motor walls together; without them the walls splay under load).
   Then the battery with the BMS against its back face, both under the strap.
7. Perfboard: build it from the to-scale layout on the wiring page ("Board pictures", perfboard drawn from the component
   side) on a standard 5 × 7 cm board with 24 × 18 holes: DevKit on two 22-pin female header rows (holes 2-23 counted
   from the right, rows 9 and 18 from the bottom), the 74AHCT125,
   12 resistors and the dividers flat under the DevKit, plugs J1-J7 and the 470 µF below it. J1 is an **XT30 PCB plug**
   (the whole 5 V rail goes through it, too much for a 2.54 mm header). Then perfboard (CAN board on 2.5 mm spacers, 2 × M3 nylon through Ø3.2 holes you drill) and Pi (4 × M2 + washers) on the back cover; contact plate with the M4
   contacts outside. The lidar's USB adapter plugs into the Pi's OTG port with a 90° micro-USB OTG cable (a straight
   adapter hits the wheel arch). Wiring routes and lengths: README, "Wiring in place", and the viewer's Wiring view.
8. Front panel: camera, sonar, eyes + diffusers + retainers, mics. USB-C charging socket in the left side wall above the
   wheel (2 × M3 + nuts), so the robot can charge standing, face-down or on its back.
9. Front panel and back cover: 6 × M3×10 countersunk hex-socket screws each (ISO 10642, 2.5 mm key), heads flush:
   nothing sticks out to mark the floor when the robot falls. Same for the 4 contact-plate screws.
   Lidar on top, power button (front-right) and arm button (front-left). **Power switch:** Pololu 2813 board screwed (or VHB-taped)
   inside the back cover, above the Pi (pads facing forward): VIN = BAT+ from the 30 A fuse, VOUT = BAT+ to the PDB (16 AWG); the two
   negatives (BMS P- and PDB in-) spliced beside it, a 26 AWG tap from the splice to its GND. The 12 mm button's two
   wires go to the board's pushbutton pads (in parallel with its own button); J9 (GPIO21) to its OFF pin. Before the
   first battery connection: VIN-VOUT open when off, no continuity between + and -.
   **Cover plugs (so the back cover comes off as one module):** the switch's input pair (VIN + P- splice) ends in a MALE
   XT60, the BMS lead in a FEMALE one; the switch's output pair (VOUT + splice) ends in a FEMALE XT60, the PDB lead in a
   MALE one (they can't be swapped; no bare live male). JST-XH 3-pin in the power-button lead beside the switch board,
   JST-XH 2-pin in the CAN pair beside the CAN board. Leave 4 cm of slack on every wire that goes to the cover.
   **Motor leads:** route them up out of the plug, over and down through the wall window in a loose loop (the shelf is
   18 mm above the plug for this); don't pull them tight against the plug.
10. Sense wires to J7: VBAT from the power switch VOUT, dock sense from the charge fuse+diode input, USB sense from the
    IP2368 VBUS pad (26 AWG each). 3 A inline fuse in the PDB -> regulator VIN lead.
11. **IR receivers:** a TSOP38238 in each black hood, dome facing the window, leads bent straight down. Thread the
    leads through the Ø4.5 hole in the top skin, glue the hood on (window facing out: front-left, front-right,
    back-left, back-right). Pin order seen from the dome: OUT, GND, VS. Chain VS and GND from receiver to receiver,
    one OUT wire each; 6-wire cable to perfboard plug J8 (3V3, GND, GPIO13 FL, 14 FR, 15 BL, 16 BR).

### Wiring at a glance

| ESP32-S3 | To |
|---|---|
| GPIO4 / GPIO5 | CAN board TX / RX |
| GPIO8 / GPIO9 | IMU SDA / SCL |
| GPIO17 / GPIO18 | Pi GPIO15 (RX) / GPIO14 (TX), 921600 baud, common GND |
| GPIO10 / GPIO11 | Sonar TRIG / ECHO (ECHO through 1 k / 2 k divider) |
| GPIO12 | Arm button to GND |
| GPIO38 / 39 / 40 | Status LED R / G / B, 330 Ω each |
| GPIO1 / 6 / 7 | Battery / dock / USB-charge sense dividers (inputs on J7) |

| Raspberry Pi | To |
|---|---|
| Pin 17 (3.3 V), pin 39 GND | Both mics 3V / GND (through the I2S junction) |
| GPIO18, pin 12 | Mics BCLK + amp BCLK |
| GPIO19, pin 35 | Mics LRCL + amp LRC |
| GPIO20, pin 38 | Both mics DOUT (shared) |
| GPIO21, pin 40 | Amp DIN |
| GPIO10 (SPI MOSI) | 74AHCT125 → 330 Ω → eyes DIN |
| Micro-USB OTG | Lidar adapter |
| GPIO12, pin 32 (+ pin 34 GND) | Headlight driver PWM (hardware PWM0) |
| CSI | AI Camera (22-pin cable supplied) |

Amp VIN from Pi pin 4: the same regulated 5 V rail (it reaches the Pi through J5, not the Pi's USB input). Never
connect either speaker output to GND.

## 4. Power-up checks (battery still disconnected where noted)

1. **CAN:** about 60 Ω between CAN H and L, power off.
2. **5 V rail** on a bench supply first: 5.0–5.2 V at the Pi and the amp.
3. **Charging, before the battery is connected:** IP2368 output 25.2 V; dock rods 25.2 V, + on the rod that meets
   the robot's + pad (the LEFT rod looking at the dock face); robot leaned back on the dock by hand: 25.2 V on the M5
   nuts inside; pads at 0 V when undocked.
4. First charge supervised, in a LiPo bag.

### The dock

1. Rods: brass Ø4 × 25 mm, solder a Ø5.5 brass washer on as a collar 16.5 mm from the tip and a 40 cm wire to the
   back end. Push the rod in from the back (tip out through the Ø4.3 bore), spring behind the collar, then epoxy an
   M3 washer over the back of the pocket (wire through its hole). It must slide freely, ~4 mm.
2. Rod wires up the column to the controller board in the mast; the DC jack in the base's back edge is wired up to the board too (step 5).
3. Head onto the foot (its groove over the foot's ring, a little glue, 2 × M3×16 down through the shelf behind the column into
   the foot's inserts), mast on (spigot into the column top, a drop of glue), TPU saddle inserts pushed up into the track frames from below, frames into their T slots, wings on (2 × M3×12
   each into the slab-side inserts),
   4 rubber feet; stand the dock against a wall with ~1 m of free floor in front. Keep the ridges clean and flat on the
   floor (no rug under them): they are what holds the robot on the dock.
4. **Reflective tape:** cut two 141 mm strips of 25 mm retro-reflective tape and press them into the two recessed
   guides on the column face. This is what the lidar locks onto (it reads >200 intensity, walls 60–160).
5. **Dock controller (IR beacon + safe rod switch):** build the 40 × 30 mm board from the site's *Wiring diagram →
   Dock* page (every connection and part value is there; schematic logic in `firmware/dock_beacon/`). Flash
   `dock_beacon.ino` onto the ATtiny85 (ATTinyCore, 8 MHz internal, ISP programmer or an Arduino as ISP). Push the 4
   all-round TSAL6200 into the mast's side holes and 1 into the centre tube (a drop of glue), stand the board in the
   mast's hollow, run the jack and rod wires down the column (+ = the LEFT rod looking at the face).
   **Never wire the DC jack straight to the rods**: the board keeps them dead unless the robot is on them.
6. **Bench test the dock before the robot touches it** (10 MΩ multimeter):
   - nothing on the rods: 0 V on them, status LED blips every 2 s;
   - 112 kΩ (100 k + 12 k) across the rods: 25.2 V after 0.5 s, LED on; remove it: 0 V within 2 s;
   - 1 Ω / 5 W across the rods: stays off, LED blinks; 3 shorts: fast blink until unplugged;
   - a phone camera sees the IR LEDs flicker.

### First docking

1. Check the beacon: the web app's telemetry `ir` value shows non-zero bits when a receiver hears it; turn the robot
   by hand and watch them move round. Then put the robot anywhere within ~4 m of the dock (any heading, also behind
   it) and press **Dock & charge**.
2. It turns until it hears the beacon or sees the tape, drives towards the beacon (circling the column if it came from
   behind), lines up 45 cm out on the dock's axis facing away, reverses in steering on the two tape strips, leans back
   8° onto the face and goes to CHARGING (blue LED). If the contacts don't touch it stands up, lines up again and
   retries (3 tries).
3. That first dock saves the dock's position on the map (`data/dock.json`). From then on **Dock & charge** (or "go
   charge" by voice) works from anywhere on the map, and the robot docks by itself when the battery runs low
   (`"auto_dock": false` in config.json turns that off).
4. **Undock** stands it up off the dock and drives it 0.4 m forward.

## 5. Flash the ESP32-S3

```
cd firmware/esp32
./sync_core.sh            # copies the simulation-tested controller from ../../core
pio run -t upload         # USB-C on the DevKitC
pio device monitor -b 115200
```

On boot, with the robot upright and still, the log must show acc ≈ (0, 0, +9.8). Tilt the top forward: ax goes
negative and gy positive. If not, fix the `IMU_MAP_*` macros in `config.h`. Compare `vbat` with a multimeter
and adjust `VBAT_DIV`.

## 6. Set up the Raspberry Pi

1. **Raspberry Pi Imager:** Raspberry Pi OS **Bookworm Lite 64-bit**. In the settings, set hostname (e.g. `robot`),
   user `pi`, your Wi-Fi, and enable SSH.
2. Copy the software:
   ```
   scp -r robot_V1.1 pi@robot.local:/home/pi/robot
   ```
3. Install (UART, SPI, I2S sound card, camera, Python packages, Vosk + piper voice models, the service):
   ```
   ssh pi@robot.local
   cd ~/robot/pi && ./install.sh
   ```
   It downloads ~150 MB of models; give it 15–20 minutes on the Zero 2 W.
4. **Claude API key.** It is kept out of `config.json`, in a file only root can read:
   ```
   sudo nano /etc/robot.env
   ANTHROPIC_API_KEY=sk-ant-...
   ```
   Check: `sudo ls -l /etc/robot.env` shows `-rw------- root root`.
5. `sudo reboot`. The robot service starts on its own.
6. Open **http://robot.local:8080** on your phone.
7. **Teach it faces:** in the app, type a name, pick 3–5 clear face photos of that person (front, slightly left/right,
   good light), press **Teach face**. It reports how many photos had a usable face. Then "Follow me" or say
   "robot, follow Anna".

### Settings (`pi/config.json`)

- `lidar_offset_deg`: with a wall straight ahead, set it so the wall reads at 0°.
- `ai_camera`: `true` with the AI Camera (detections + follow); `false` for a plain Camera Module 3 (stream only).
- `auto_dock`: `true` = go to the charger by itself when the battery is low (once it knows where the dock is).
- `voice.wake_phrase`: words that start an order (`"hey robot|robot"`).
- `voice.push_to_talk_only`: `true` = only the app's Talk button listens.
- `voice.language`: `"de"` for German, with `vosk-model-small-de-0.15` and a German piper voice.
- `voice.stt`: `"vosk"` (offline, on the Pi) or `"http"` with `stt_url` pointing to a Whisper server.
- `voice.mic_gain_db`: raise if it misses you at 2 m, lower if it clips.

After changing it: `sudo systemctl restart robot`.

## 7. Test the audio

```
arecord -l                                   # the "googlevoicehat" card must be listed
arecord -D plughw:CARD=sndrpigooglevoi -c 2 -r 48000 -f S32_LE -d 5 /tmp/t.wav
aplay -D plughw:CARD=sndrpigooglevoi /tmp/t.wav
speaker-test -D plughw:CARD=sndrpigooglevoi -c 2 -t sine -f 440 -l 1
```

Talk from 2 m during the recording: you should hear yourself clearly, with both channels alive (`sox /tmp/t.wav -n stat`
per channel if in doubt). A silent channel means a SEL pin or DOUT wire problem; noise only means BCLK/LRCL.

Then in the app: tap **Talk**, say "drive forward half a metre". The status line shows the recognised text,
Claude's tool calls and the reply, which the robot speaks. Saying **"Stop!"** works instantly, without Claude.

## 8. First balance

1. Wheels off the ground, press **arm** (button or app): the wheels should react to tilting the body.
2. On a clear floor of at least 2 × 2 m, hold it upright and arm. It should balance on its own.
3. Lay it on its face, then on its back: with auto stand-up on, it gets up by itself
   (it needs ~0.7 m of free floor).
4. Joystick, turns, then **Explore** to build the lidar map. Tap the map to send it somewhere.

## 9. Run the tests and the simulation (optional)

```
cd pi && python3 -m pytest -q tests          # 20 tests, includes the voice pipeline with synthetic audio
cd sim && python3 build_model.py && python3 build_model.py --arena && python3 design_lqr.py 0.06 0.6 10
cd ../core && g++ -O2 -std=c++17 -shared -fPIC controller.cpp sim_api.cpp -o ../sim/libbot.so
cd ../sim && python3 run_tests.py 40
```

If you change the robot (heavier battery, other wheels), update `sim/cad_mass.json` or re-run
`cad_scripts/assemble.py`, redesign the gains, then `./sync_core.sh` and reflash.

## Troubleshooting

| Symptom | Check |
|---|---|
| Motors don't respond | CAN 60 Ω, IDs 1/2, 1 Mbps, common GND |
| Falls straight away forward/back | IMU axis mapping, motor directions |
| Oscillates while balancing | Wheel offset / mass differs from the sim: re-run `design_lqr.py`, reflash |
| No sound card | `dtoverlay=googlevoicehat-soundcard` in `/boot/firmware/config.txt`, reboot |
| Voice status says "ANTHROPIC_API_KEY not set" | `/etc/robot.env` content, then `sudo systemctl restart robot` |
| Voice misses you at 2 m | `mic_gain_db` +6, check both mic channels, keep the motors' cable away from the mic wires |
| Speech is slow | piper is heavy on the Zero 2 W: set `"tts": "espeak"`, or use the Whisper server for STT |
| Logs | `journalctl -u robot -f` |
