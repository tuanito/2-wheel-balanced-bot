"""Bench test over a REAL CAN bus: the real ESP32-S3 (firmware env esp32s3_bench) and its CAN board talk to two virtual
LK motors that this PC plays through a USB-CAN adapter; the robot's physics (MuJoCo) runs here and feeds the IMU over
the DevKit's "UART" USB port. Same scenarios as hil/run_hil.py (QEMU), but with the real TWAI driver, transceiver,
wiring, termination and timing.

  python3 bench_hil.py --serial /dev/ttyUSB0 --can socketcan:can0 [scenario ...] [--seeds 1,2,3]
  scenarios: standup_face standup_back balance_push drive cmd_timeout can_loss   (default: all)

Wiring: DevKit "UART" port -> PC (serial), CAN board H/L -> adapter CAN_H/CAN_L + GND, 120 ohm at the adapter end
(most adapters have a switch/jumper) and the 120 ohm already on the Waveshare board. No motors, no battery needed.
"""
import argparse, json, os, pathlib, struct, sys, threading, time
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "hil")); sys.path.insert(0, str(ROOT / "pi")); sys.path.insert(0, str(HERE))
import run_hil                                                    # noqa: E402  (scenarios)
from lkbus import Plant, VirtualLK, crc as sim_crc, frame as sim_frame, CTRL_DT   # noqa: E402
from robot.link import (Telemetry, TELEM_FMT, MSG_TELEM, MSG_EVENT, CMD_FMT, MSG_CMD, F_ARM, F_DISARM,  # noqa: E402
                        F_AUTO_STANDUP, F_OBST_STOP, MODE_VELOCITY, crc8 as pi_crc, frame as pi_frame)
from canbus import open_bus                                       # noqa: E402

CFG = dict(serial=None, can=None, reset=True)


class BenchLink:
    """The Pi's side of the link, on the same serial port as the IMU frames."""
    def __init__(self, host): self.h = host; self.telem = Telemetry(); self.seq = 0

    def send(self, v=0.0, yaw_rate=0.0, mode=MODE_VELOCITY, angle=0.0, arm=False, disarm=False, auto_standup=True,
             obstacle_stop=True, new_turn=False):
        if new_turn: self.seq = (self.seq + 1) & 0xFFFF
        flags = (F_ARM if arm else 0) | (F_DISARM if disarm else 0) | (F_AUTO_STANDUP if auto_standup else 0) | (F_OBST_STOP if obstacle_stop else 0)
        self.h.write(pi_frame(MSG_CMD, struct.pack(CMD_FMT, mode, flags, self.seq, float(v), float(yaw_rate), float(angle))))


class _Alive:
    def poll(self): return None


class BenchHost:
    """Drop-in for run_hil.Host: same attributes the scenarios use."""
    def __init__(self, seed=1, randomize=True, log_dir="/tmp/bench"):
        import serial
        self.plant = Plant(seed, randomize)
        self.vlk = VirtualLK(self.plant)
        self.lock = threading.Lock(); self.wlock = threading.Lock()
        self.log_dir = pathlib.Path(log_dir); self.log_dir.mkdir(exist_ok=True)
        self.can_mute_until = -1.0
        self.events = []; self.telem = []; self.cycles = 0; self.dts = []; self.host_ms = []; self.wall = []
        self.stop = False; self.q = _Alive()
        self.ser = serial.serial_for_url(CFG["serial"], baudrate=921600, timeout=0.01)
        self.bus = open_bus(CFG["can"])
        self.link = BenchLink(self)
        self.can_rx = 0; self.can_tx = 0; self.can_errors = 0

    def write(self, b):
        with self.wlock: self.ser.write(b)

    def start(self, tag):
        self.tag = tag
        self.con = open(self.log_dir / f"{tag}_console.log", "wb")
        threading.Thread(target=self.serial_loop, daemon=True).start()   # listening before the ESP32 boots
        threading.Thread(target=self.can_loop, daemon=True).start()
        if CFG["reset"] and hasattr(self.ser, "rts"):
            try:                                                   # DevKit auto-reset: RTS -> EN, DTR -> IO0 (keep high)
                self.ser.dtr = False; self.ser.rts = True; time.sleep(0.1); self.ser.rts = False
            except Exception: pass

    def close(self):
        self.stop = True; time.sleep(0.05)
        for f in (self.ser.close, self.bus.shutdown, self.con.close):
            try: f()
            except Exception: pass
        print(f"   CAN: {self.can_rx} frames from the ESP32, {self.can_tx} replies, {self.can_errors} error frames")

    # -------------------------------------------------- real CAN: the two virtual motors
    def can_loop(self):
        import can
        while not self.stop:
            try: m = self.bus.recv(0.01)
            except Exception: break
            if m is None: continue
            if m.is_error_frame: self.can_errors += 1; continue
            self.can_rx += 1
            with self.lock:
                self.vlk.mute_until = self.can_mute_until
                out = self.vlk.handle(m.arbitration_id, m.data)
            for i, d in out:
                try: self.bus.send(can.Message(arbitration_id=i, data=d, is_extended_id=False)); self.can_tx += 1
                except Exception: pass

    # -------------------------------------------------- serial: IMU lock-step + Pi frames + console text
    def serial_loop(self):
        buf = bytearray(); P = self.plant
        while not self.stop:
            try: data = self.ser.read(max(1, self.ser.in_waiting))   # return as soon as bytes arrive (not after 512 / timeout)
            except Exception: break
            if not data: continue
            buf += data
            while True:
                i = buf.find(b"\xA5\x5A")
                if i < 0: self.con.write(bytes(buf[:-1])); del buf[:-1]; break
                if i: self.con.write(bytes(buf[:i])); del buf[:i]
                if len(buf) < 4: break
                n = 4 + buf[3] + 1
                if len(buf) < n: break
                body = bytes(buf[2:n - 1]); c = buf[n - 1]; t, p = body[0], body[2:]
                if t in (MSG_TELEM, MSG_EVENT) and pi_crc(body) == c:
                    if t == MSG_TELEM and len(p) == struct.calcsize(TELEM_FMT): self.link.telem = Telemetry.unpack(p)
                    elif t == MSG_EVENT: self.events.append((P.t, p.decode(errors="replace")))
                    del buf[:n]; continue
                if t == ord("S") and sim_crc(body) == c:
                    t_us, = struct.unpack("<I", p)
                    w0 = time.perf_counter()
                    with self.lock:
                        dt = CTRL_DT if P.t_fw is None else ((t_us - P.t_fw) & 0xFFFFFFFF) * 1e-6
                        P.t_fw = t_us
                        if self.cycles > 0: P.advance(min(dt, 0.05)); self.dts.append(dt)
                        self.cycles += 1
                        imu = P.imu()
                    self.write(sim_frame(ord("I"), imu))
                    self.host_ms.append(1e3 * (time.perf_counter() - w0))
                    del buf[:n]; continue
                del buf[:2]                                        # not a frame: resync

    # -------------------------------------------------- scenario API (same as run_hil.Host)
    def run_for(self, seconds, **cmd):
        t_end = self.plant.t + seconds; deadline = time.time() + seconds * 5 + 20
        while self.plant.t < t_end and time.time() < deadline:
            if cmd.get("silent") is not True: self.link.send(**{k: v for k, v in cmd.items() if k != "silent"})
            self.telem.append((self.plant.t, self.link.telem))
            time.sleep(0.05)
        return self.plant.t >= t_end

    def wait_boot(self, timeout=15):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any("state" in e for _, e in self.events): return True
            time.sleep(0.05)
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", required=True, help="DevKit UART port, e.g. /dev/ttyUSB0, COM4, socket://host:port")
    ap.add_argument("--can", required=True, help="adapter, e.g. socketcan:can0, gs_usb:0, slcan:/dev/ttyACM0 (see canbus.py)")
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--no-reset", action="store_true", help="don't reset the ESP32 through RTS before each scenario")
    ap.add_argument("--out", default="bench_results.json")
    ap.add_argument("scenarios", nargs="*")
    a = ap.parse_args()
    CFG.update(serial=a.serial, can=a.can, reset=not a.no_reset)
    run_hil.Host = BenchHost                                      # the scenarios now drive the real ESP32
    names = a.scenarios or list(run_hil.SCEN)
    res = {}
    for n in names:
        for s in [int(x) for x in a.seeds.split(",")]:
            t0 = time.time(); r = run_hil.SCEN[n](s); r["wall_s"] = round(time.time() - t0, 1)
            res[f"{n}_{s}"] = r
            print(n, s, json.dumps(r), flush=True)
    pathlib.Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
