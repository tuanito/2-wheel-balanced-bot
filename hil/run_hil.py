"""Hardware-in-the-loop test: the real ESP32-S3 firmware (env esp32s3_sim) runs in Espressif QEMU against the MuJoCo
robot. The Pi side is the real pi/robot/link.py.

  QEMU UART1  -> Pi link (921600 protocol of link_proto.h), driven here through pi/robot/link.Link
  QEMU UART0  -> this host: virtual LK MF6015 motors (CAN frames, LK protocol V2) + IMU samples, in lock-step
                 (console text shares UART0; bytes outside frames are written to the console log):
                 every control cycle the firmware sends its own clock; the physics advances by exactly that much.

Usage: python3 run_hil.py [scenario ...]   (scenarios: standup_face standup_back balance_push drive cmd_timeout can_loss)
       ENC_BITS=14 python3 run_hil.py ...  -> virtual motors with a 14-bit encoder (the LK manual allows 14/15/16 bit)
"""
import math, os, socket, struct, subprocess, sys, threading, time, json, pathlib, collections
import numpy as np, mujoco

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim")); sys.path.insert(0, str(ROOT / "pi")); sys.path.insert(0, str(ROOT / "hil"))
from lkbus import Plant, VirtualLK, crc, frame, quat_pitch, CTRL_DT, ID_L, ID_R    # noqa: E402  (plant + virtual motors)
from robot.link import Link, MODE_VELOCITY, MODE_TURN_BY        # noqa: E402  (the Pi's own link code)

QEMU = os.environ.get("QEMU", "/opt/dl/qemu/qemu/bin/qemu-system-xtensa")
FLASH = os.environ.get("FLASH", "/tmp/flash_sim.bin")


class SockPort:
    """read(n)/write(b) over a socket, for pi/robot/link.Link."""
    def __init__(self, s): self.s = s; self.s.settimeout(0.01)
    def read(self, n):
        try: return self.s.recv(n)
        except (socket.timeout, BlockingIOError): return b""
        except OSError: return b""
    def write(self, b):
        try: self.s.sendall(b)
        except OSError: pass


class Host:
    def __init__(self, seed=1, randomize=True, log_dir="/tmp/hil"):
        self.plant = Plant(seed, randomize)
        self.vlk = VirtualLK(self.plant)
        self.log_dir = pathlib.Path(log_dir); self.log_dir.mkdir(exist_ok=True)
        self.can_mute_until = -1.0
        self.events = []; self.telem = []
        self.cycles = 0; self.dts = []; self.host_ms = []; self.wall = []
        self.stop = False

    # ------------------------------------------------------------ QEMU + sockets
    def start(self, tag):
        self.tag = tag
        srv = []
        for _ in range(2):
            s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(("127.0.0.1", 0)); s.listen(1); srv.append(s)
        p1, p2 = (s.getsockname()[1] for s in srv)
        self.console = self.log_dir / f"{tag}_console.log"
        cmd = [QEMU, "-nographic", "-accel", os.environ.get("QEMU_ACCEL", "tcg"), "-machine", "esp32s3", *os.environ.get("QEMU_EXTRA", "-icount shift=2,sleep=on,align=off").split(), "-m", "8M", "-global", "ssi_psram.is_octal=true", "-global", "ssi_psram.size_mbytes=8",
               "-drive", f"file={self.flash_copy()},if=mtd,format=raw",
               "-serial", f"tcp:127.0.0.1:{p2},nodelay=on", "-serial", f"tcp:127.0.0.1:{p1},nodelay=on", "-monitor", "none"]
        self.con = open(self.console, "wb")
        self.q = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for s in srv: s.settimeout(20)
        self.bus, _ = srv[1].accept(); self.pi_sock, _ = srv[0].accept()
        self.bus.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.link = Link(SockPort(self.pi_sock), on_event=lambda txt: self.events.append((self.plant.t, txt)))
        self.th = threading.Thread(target=self.bus_loop, daemon=True); self.th.start()

    def flash_copy(self):
        dst = self.log_dir / f"flash_{os.getpid()}.bin"
        if not dst.exists(): dst.write_bytes(pathlib.Path(FLASH).read_bytes())   # fresh NVS every scenario
        return dst

    def close(self):
        self.stop = True
        try: self.con.flush()
        except Exception: pass
        try: self.q.terminate(); self.q.wait(5)
        except Exception: self.q.kill()
        for p in self.log_dir.glob(f"flash_{os.getpid()}.bin"): p.unlink()

    # ------------------------------------------------------------ virtual CAN bus + IMU (lock-step)
    def bus_loop(self):
        buf = bytearray(); P = self.plant
        self.bus.settimeout(0.5)
        while not self.stop:
            try: data = self.bus.recv(4096)
            except socket.timeout: continue
            except OSError: break
            if not data: break
            buf += data
            out = bytearray()
            while True:
                i = buf.find(b"\xA5\x5A")
                if i < 0:
                    self.con.write(bytes(buf[:-1])); del buf[:-1]; break
                if i: self.con.write(bytes(buf[:i])); del buf[:i]
                if len(buf) < 4: break
                n = 4 + buf[3] + 1
                if len(buf) < n: break
                body = bytes(buf[2:n - 1]); ok = crc(body) == buf[n - 1]
                t, p = body[0], body[2:]
                del buf[:n]
                if not ok: continue
                if t == ord("C"):
                    cid, dlc, d = struct.unpack("<IB8s3x", p)
                    out += self.on_can(cid, d)
                elif t == ord("S"):
                    t_us, = struct.unpack("<I", p)
                    w0 = time.perf_counter()
                    dt = CTRL_DT if P.t_fw is None else ((t_us - P.t_fw) & 0xFFFFFFFF) * 1e-6
                    P.t_fw = t_us
                    if self.cycles > 0: P.advance(min(dt, 0.05)); self.dts.append(dt)
                    self.cycles += 1
                    out += frame(ord("I"), P.imu())
                    self.host_ms.append(1e3 * (time.perf_counter() - w0)); self.wall.append(w0)
            if out: self.bus.sendall(bytes(out))

    def on_can(self, cid, d):
        self.vlk.mute_until = self.can_mute_until
        return b"".join(frame(ord("C"), struct.pack("<IB8s3x", i, 8, data)) for i, data in self.vlk.handle(cid, d))

    # ------------------------------------------------------------ Pi side
    def run_for(self, seconds, **cmd):
        """Keep sending Pi commands at 20 Hz (like the Pi brain) while the physics advances `seconds`."""
        t_end = self.plant.t + seconds; deadline = time.time() + seconds * 40 + 30
        while self.plant.t < t_end and time.time() < deadline and self.q.poll() is None:
            if cmd.get("silent") is not True: self.link.send(**{k: v for k, v in cmd.items() if k != "silent"})
            self.telem.append((self.plant.t, self.link.telem))
            time.sleep(0.05 * max(1.0, self.slowdown()))
        return self.plant.t >= t_end

    def slowdown(self):
        """Wall time per simulated second (QEMU runs the firmware in real time; the host keeps up)."""
        return 1.0

    def wait_boot(self, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any("state" in e for _, e in self.events): return True
            time.sleep(0.05)
        return False


# ---------------------------------------------------------------- scenarios
def summary(h, extra=None):
    tr = np.array([(t, p, x, y) for t, p, x, y, *_ in h.plant.trace]) if h.plant.trace else np.zeros((1, 4))
    st = [(round(t, 3), e) for t, e in h.events if e.startswith("state")]
    dts = np.array(h.dts) if h.dts else np.array([0])
    r = dict(cycles=h.cycles, sim_s=round(h.plant.t, 2), dt_ms_median=round(1e3 * float(np.median(dts)), 3),
             dt_ms_p99=round(1e3 * float(np.percentile(dts, 99)), 3), dt_ms_max=round(1e3 * float(dts.max()), 2),
             host_ms_p99=round(float(np.percentile(h.host_ms, 99)), 3) if h.host_ms else None,
             host_ms_max=round(float(max(h.host_ms)), 2) if h.host_ms else None,
             states=st, final_state=h.link.telem.state_name, motor_fault=h.link.telem.motor_fault, armed=h.link.telem.armed,
             batt_low=h.link.telem.battery_low, final_pitch_deg=round(math.degrees(tr[-1, 1]), 1))
    if extra: r.update(extra)
    return r


def standup(seed, side):
    h = Host(seed); h.plant.place(1.8 * side); h.start(f"standup_{'face' if side > 0 else 'back'}_{seed}")
    ok = h.wait_boot()
    h.run_for(0.5, arm=False)                     # booted, lying, disarmed
    h.run_for(4.0, arm=True)                      # arm: auto stand-up
    tr = np.array([(t, p) for t, p, *_ in h.plant.trace])
    t_arm = tr[0, 0] + 0.5
    up = [t for t, e in h.events if "BALANCE" in e]
    r = summary(h, dict(booted=ok, stood_up=bool(up), t_standup_s=round(up[0] - t_arm, 2) if up else None,
                        upright_at_end=bool(abs(tr[-1, 1]) < 0.3)))
    h.close(); return r


def balance_push(seed):
    h = Host(seed); h.plant.place(0); h.start(f"balance_{seed}")
    h.wait_boot()
    h.plant.hold = h.plant.d.qpos.copy()           # a hand holds it upright while it boots and arms
    h.run_for(0.5, arm=True); h.plant.hold = None
    h.run_for(2.0)
    x0 = h.plant.d.qpos[0]
    res = []
    for k, imp in enumerate((1.0, -1.0, 1.5)):
        h.plant.push = np.array([imp / 0.1, 0, 0]); h.plant.push_until = h.plant.t + 0.1
        h.run_for(3.0)
        res.append(dict(impulse_Ns=imp, upright=bool(abs(quat_pitch(h.plant.d.qpos[3:7])) < 0.3), state=h.link.telem.state_name))
    tr = np.array([(t, p, x) for t, p, x, *_ in h.plant.trace])
    calm = tr[(tr[:, 0] > tr[0, 0] + 0.6) & (tr[:, 0] < tr[0, 0] + 2.4)]
    r = summary(h, dict(pushes=res, still_pitch_std_deg=round(math.degrees(float(calm[:, 1].std())), 3) if len(calm) else None,
                        still_drift_mm=round(1e3 * float(np.ptp(calm[:, 2]))) if len(calm) else None))
    h.close(); return r


def drive(seed):
    h = Host(seed); h.plant.place(0); h.start(f"drive_{seed}")
    h.wait_boot()
    h.plant.hold = h.plant.d.qpos.copy()           # a hand holds it upright while it boots and arms
    h.run_for(0.5, arm=True); h.plant.hold = None
    h.run_for(1.0)
    p0 = h.plant.d.qpos[:2].copy()
    h.run_for(3.0, v=0.5)
    v_true = [(t, x) for t, _, x, *_ in h.plant.trace[-250:]]
    v_meas = (v_true[-1][1] - v_true[0][1]) / (v_true[-1][0] - v_true[0][0])
    h.run_for(2.5, v=0.0)
    p1 = h.plant.d.qpos[:2].copy()
    odo = h.link.telem
    h.link.send(mode=MODE_TURN_BY, angle=math.pi / 2, new_turn=True)
    h.run_for(3.0, mode=MODE_TURN_BY, angle=math.pi / 2)
    yaw = h.plant.true_yaw()
    r = summary(h, dict(speed_cmd=0.5, speed_true=round(v_meas, 3), dist_true_m=round(float(np.linalg.norm(p1 - p0)), 3),
                        odo_x_m=round(odo.x, 3), turn_cmd_deg=90, yaw_true_deg=round(math.degrees(yaw), 1),
                        upright=bool(abs(quat_pitch(h.plant.d.qpos[3:7])) < 0.3)))
    h.close(); return r


def cmd_timeout(seed):
    """Pi goes silent while driving at 0.5 m/s: the firmware must stop driving within ~0.3 s and keep balancing."""
    h = Host(seed); h.plant.place(0); h.start(f"cmd_timeout_{seed}")
    h.wait_boot()
    h.plant.hold = h.plant.d.qpos.copy()           # a hand holds it upright while it boots and arms
    h.run_for(0.5, arm=True); h.plant.hold = None
    h.run_for(1.0); h.run_for(2.0, v=0.5)
    t_cut = h.plant.t
    h.run_for(4.0, silent=True)
    tr = np.array([(t, x) for t, _, x, *_ in h.plant.trace])
    after = tr[tr[:, 0] > t_cut + 2.5]
    v_end = (after[-1, 1] - after[0, 1]) / (after[-1, 0] - after[0, 0]) if len(after) > 2 else None
    r = summary(h, dict(speed_after_silence=round(v_end, 3) if v_end is not None else None,
                        upright=bool(abs(quat_pitch(h.plant.d.qpos[3:7])) < 0.3)))
    h.close(); return r


def can_loss(seed):
    """CAN unplugged for 100 ms while balancing: firmware must declare a motor fault and switch the motors off."""
    h = Host(seed); h.plant.place(0); h.start(f"can_loss_{seed}")
    h.wait_boot()
    h.plant.hold = h.plant.d.qpos.copy()           # a hand holds it upright while it boots and arms
    h.run_for(0.5, arm=True); h.plant.hold = None
    h.run_for(1.5)
    t_cut = h.plant.t; h.can_mute_until = t_cut + 0.1
    h.run_for(1.0)
    t_off = [t for t, _, _, _, on, *_ in h.plant.trace if t > t_cut + 0.1 and not on]
    fault = [t - t_cut for t, e in h.events if t > t_cut and ("fault" in e or "FAULT" in e or "DISARMED" in e)]
    r = summary(h, dict(fault_flag=h.link.telem.motor_fault, motors_on_at_end=bool(h.plant.on[ID_L] or h.plant.on[ID_R]),
                        reaction_s=round(fault[0], 3) if fault else None,
                        motors_off_after_bus_back_s=round(t_off[0] - t_cut - 0.1, 3) if t_off else None))
    h.close(); return r


SCEN = {"standup_face": lambda s: standup(s, +1), "standup_back": lambda s: standup(s, -1), "balance_push": balance_push,
        "drive": drive, "cmd_timeout": cmd_timeout, "can_loss": can_loss}

if __name__ == "__main__":
    names = sys.argv[1:] or list(SCEN)
    seeds = [int(x) for x in os.environ.get("SEEDS", "1").split(",")]
    allr = {}
    for n in names:
        for s in seeds:
            t0 = time.time()
            r = SCEN[n](s); r["wall_s"] = round(time.time() - t0, 1)
            allr[f"{n}_{s}"] = r
            print(n, s, json.dumps(r), flush=True)
    out = pathlib.Path(os.environ.get("OUT", "/tmp/hil/results.json"))
    out.write_text(json.dumps(allr, indent=1))
