# Testing the CAN driver and code

Three levels, cheapest first. Do them in order: each one rules out a class of problems before the next.

## 1. Unit tests on the PC (no hardware)

```
firmware/esp32/test_host/run_tests.sh
```

Builds `src/lk_motor.cpp` unchanged with g++ against a fake ESP-IDF TWAI driver and feeds it LK frames. 54 checks:
torque packing in the 0x280 frame (motor 1 bytes 0-1, motor 2 mirrored, clamp ±2048), reply parsing (temperature,
current, speed, direction), the 0x88/0x80 echoes and foreign IDs ignored, encoder unwrap forwards and backwards for
14/15/16-bit encoders, range detection from the wrap and from the speed ratio, a 100 ms reply gap while the wheel keeps
turning, the CAN timeout (including a reply stamped after `now`), a full TX queue never blocking.

They found two problems, both fixed: motion before the encoder range is known was counted at the 16-bit guess
(now learned after 90° instead of 180°: at most a 1.2 rad one-off offset, no jump), and a reply gap could be taken
for an encoder wrap and switch the range (now unwrapped using the reported speed).

## 2. Bench with a USB-CAN adapter (no motors, no battery)

Hardware: the ESP32-S3 DevKit + the Waveshare CAN board wired as on the robot (GPIO4 -> CTX, GPIO5 -> CRX, 3.3 V,
GND) and a USB-CAN adapter: a candleLight / CANable (gs_usb firmware, ~€15-30) works on Linux (SocketCAN), macOS and
Windows (`gs_usb`); a CANable with slcan firmware works everywhere (`slcan:`). Adapter CAN_H/CAN_L/GND to the CAN
board's H/L/GND, termination ON at the adapter (the CAN board has its own 120 ohm): ~60 ohm between H and L.

```
pip install python-can pyserial mujoco numpy      # + gs_usb on macOS/Windows with a candleLight
sudo ip link set can0 up type can bitrate 1000000 # Linux SocketCAN only
```

### 2a. Transceiver self-test and bus scan (CAN probe firmware)

```
cd firmware/esp32 && pio run -e esp32s3_canprobe -t upload && pio device monitor -e esp32s3_canprobe
```

Prints `self-test: 20/20 frames came back through the transceiver`: the ESP32 sends frames to itself through the
SN65HVD230 (no other node needed). 0/20 = CTX/CRX swapped, no 3.3 V or a bad joint. Then it scans IDs 1-4 every 2 s.

### 2b. Watch the robot firmware's traffic

Flash the robot firmware (`pio run -e esp32s3 -t upload`) and run

```
python3 bench/can_monitor.py socketcan:can0
```

Disarmed: 0x9C requests to 0x141/0x142 at ~480/s each, with a 0x80 every 50 ms; nothing answers yet (the adapter
ACKs the frames). Error frames must stay at 0. The adapter must be the only extra node.

### 2c. The robot firmware against two virtual motors over the real bus

```
cd firmware/esp32 && pio run -e esp32s3_bench -t upload          # robot firmware, IMU from the PC
python3 bench/bench_hil.py --serial /dev/ttyUSB0 --can socketcan:can0
```

`esp32s3_bench` is the robot firmware with the real TWAI driver; only the IMU comes from the PC (DevKit **UART** USB
port, 921600, the Pi's arm/drive commands share it). The PC plays motors 1 and 2 on the real bus, runs the physics
and feeds the IMU, so the robot stands up, balances, drives, gets pushed - with the real CAN driver, transceiver,
wiring, termination and 1 Mbit/s timing. Same scenarios and output as the QEMU test (`hil/run_hil.py`).
`can_loss` mutes the virtual motors for 100 ms; also pull the CAN cable by hand while it balances: the firmware must
log `motor fault: no CAN reply for ~20 ms`, disarm, and keep sending 0x80.

`bench/test_bench.py` checks the kit itself without hardware (virtual CAN bus + the bench build in QEMU).

## 3. One real motor, wheel off the ground

1. With LK's PC tool on the motor's UART connector: ID 1 (left) or 2 (right), CAN 1 Mbit/s.
2. Motor on its supply (or the battery through the 30 A fuse), CAN board H/L to the motor, 60 ohm H-L with power off.
3. CAN probe firmware (2a): the scan must show `motor 1: state ... enc ...`.
4. `e`: turn the wheel one full turn by hand -> counts per turn (cross-check of the firmware's own detection).
5. `+`: 0.2 N·m for 0.5 s. If the wheel turned so the robot would roll forward, keep `DIR_LEFT = +1`, else -1.
   Same with `2` for the right motor and `DIR_RIGHT`.
6. Only then the robot firmware: boot log `encoder range detected: N counts/turn` once the wheel has turned.
