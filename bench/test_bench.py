"""Self-test of the bench kit without hardware.

1. CAN side: BenchHost's virtual motors on python-can's in-process virtual bus; a second bus endpoint plays the ESP32
   and sends the robot firmware's frames (0x9C, 0x88, 0x280, 0x80, 0xA1); replies are checked byte by byte.
2. Serial side: the bench firmware (env esp32s3_bench_qemu = esp32s3_bench with DIO flash) runs in QEMU; bench_hil talks
   to its UART0 over TCP exactly as it would over the DevKit's USB-UART: IMU lock-step, Pi command frames (arm) in,
   telemetry + events out. QEMU has no CAN bus, so the motors never answer: the firmware must arm, try to switch the
   motors on and report a motor fault (that is the expected end state here).
Run: python3 test_bench.py   (part 2 needs QEMU + /tmp/flash_bench.bin, see bench/README.md)
"""
import os, socket, struct, subprocess, sys, time, pathlib
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / "hil"))
import can
import bench_hil as B
from lkbus import ID_L, ID_R, ID_MULTI, DIR, IQ_MAX

fails = 0
def check(c, msg):
    global fails
    print(("  ok   " if c else "  FAIL ") + msg)
    if not c: fails += 1


def can_side():
    print("1. virtual motors on a python-can bus")
    B.CFG.update(serial="loop://", can="virtual:benchtest", reset=False)
    h = B.BenchHost(seed=3); h.plant.place(0)
    h.start("cantest")
    esp = can.Bus(interface="virtual", channel="benchtest")
    def xfer(i, d, n):
        esp.send(can.Message(arbitration_id=i, data=bytes(d), is_extended_id=False))
        out = []
        t0 = time.time()
        while len(out) < n and time.time() - t0 < 1:
            m = esp.recv(0.1)
            if m: out.append(m)
        return out
    r = xfer(ID_L, [0x9C, 0, 0, 0, 0, 0, 0, 0], 1)
    check(len(r) == 1 and r[0].arbitration_id == ID_L and r[0].data[0] == 0x9C and r[0].dlc == 8, "0x9C read state -> one 0x9C reply from 0x141")
    r = xfer(ID_R, [0x88, 0, 0, 0, 0, 0, 0, 0], 1)
    check(len(r) == 1 and bytes(r[0].data) == bytes([0x88] + [0] * 7) and h.plant.on[ID_R], "0x88 -> echo of the command, motor 2 on")
    r = xfer(ID_L, [0x88, 0, 0, 0, 0, 0, 0, 0], 1)
    r = xfer(ID_MULTI, list(struct.pack("<hhhh", 300, -300, 0, 0)), 2)
    ids = sorted(m.arbitration_id for m in r)
    check(ids == [ID_L, ID_R] and all(m.data[0] == 0xA1 for m in r), "0x280 -> 0xA1 replies from 0x141 and 0x142")
    check(h.plant.iq_cmd[ID_L] == 300 and h.plant.iq_cmd[ID_R] == -300, "0x280 iq applied to motor 1 / motor 2")
    rep = {m.arbitration_id: struct.unpack("<BbhhH", bytes(m.data)) for m in r}
    check(rep[ID_L][2] == 300 and rep[ID_R][2] == -300, "reply iq = commanded iq")
    xfer(ID_MULTI, list(struct.pack("<hhhh", 5000, -5000, 0, 0)), 2)
    check(h.plant.iq_cmd[ID_L] == IQ_MAX and h.plant.iq_cmd[ID_R] == -IQ_MAX, "iq clamped to +-2048")
    r = xfer(ID_L, [0x80, 0, 0, 0, 0, 0, 0, 0], 1)
    check(len(r) == 1 and r[0].data[0] == 0x80 and not h.plant.on[ID_L] and h.plant.iq_cmd[ID_L] == 0, "0x80 -> echo, motor 1 off, torque 0")
    h.can_mute_until = h.plant.t + 10
    r = xfer(ID_L, [0x9C, 0, 0, 0, 0, 0, 0, 0], 1)
    check(len(r) == 0, "muted (CAN unplugged): no reply")
    esp.shutdown(); h.close()


def serial_side():
    print("2. bench firmware in QEMU <-> bench_hil over UART0")
    flash = "/tmp/flash_bench.bin"
    if not os.path.exists(flash): print("  skipped: build env esp32s3_bench_qemu into /tmp/flash_bench.bin first"); return
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    q = subprocess.Popen([os.environ.get("QEMU", "/opt/dl/qemu/qemu/bin/qemu-system-xtensa"), "-nographic", "-accel", "tcg",
                          "-machine", "esp32s3", "-icount", "shift=2,sleep=on,align=off", "-m", "8M",
                          "-global", "ssi_psram.is_octal=true", "-global", "ssi_psram.size_mbytes=8",
                          "-drive", f"file={flash},if=mtd,format=raw", "-serial", f"tcp:127.0.0.1:{port},server=on,wait=on,nodelay=on",
                          "-monitor", "none"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    try:
        B.CFG.update(serial=f"socket://127.0.0.1:{port}", can="virtual:qemutest", reset=False)
        h = B.BenchHost(seed=1); h.plant.place(0); h.plant.hold = h.plant.d.qpos.copy()
        h.start("serialtest")
        booted = h.wait_boot(60)
        check(booted, "boot: IMU answered over the shared port, state event received as a Pi frame")
        h.run_for(0.4, arm=False)
        cyc0 = h.cycles
        h.run_for(1.0, arm=True)
        import numpy as np
        d = np.array(h.dts[-200:]) if h.dts else np.array([0])
        check(h.cycles - cyc0 > 300, f"lock-step running: {h.cycles - cyc0} control cycles in 1 s of robot time")
        check(abs(float(np.median(d)) - 0.002) < 0.0002, f"control period {1e3 * float(np.median(d)):.2f} ms")
        check(h.link.telem.t_ms > 0, f"telemetry frames parsed (t_ms {h.link.telem.t_ms})")
        check(h.link.telem.armed, "arm command reached the firmware (armed flag in telemetry)")
        ev = " | ".join(e for _, e in h.events)
        check("motor fault" in ev, "no motors on the bus -> firmware reports a CAN motor fault: " + ev[-160:])
        h.close()
    finally:
        q.terminate(); q.wait(5)


if __name__ == "__main__":
    can_side()
    serial_side()
    print(f"\n{'all passed' if not fails else f'{fails} FAILED'}")
    sys.exit(1 if fails else 0)
