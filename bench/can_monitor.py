"""Watch the robot's CAN bus and decode the LK traffic, once per second.

  python3 can_monitor.py socketcan:can0            (see canbus.py for other adapters)

Per second: frames per ID, the torque the ESP32 commands (0x280, in N·m per motor), the motors' replies (temperature,
current, speed, encoder), error frames and the bus load. What to expect from the robot firmware:
  disarmed  : 0x141/0x142 0x9C requests at ~480/s each, every 25th cycle 0x80 instead; replies 0x9C from both motors
  armed     : 0x280 torque at 500/s + 0xA1 replies from 0x141 and 0x142 at 500/s each
  bus load  : ~40 % armed at 1 Mbit/s (3 frames per 2 ms)
"""
import collections, struct, sys, time
from canbus import open_bus

KT, IQ_FULL, IQ_MAX = 0.14, 16.5, 2048
BITS_PER_FRAME = 130          # standard 8-byte frame incl. typical bit stuffing


def iq_nm(v): return v * IQ_FULL / IQ_MAX * KT


def main(spec, seconds=None):
    bus = open_bus(spec)
    t0 = last = time.time()
    cnt = collections.Counter(); errs = 0; latest = {}; total = 0
    print(f"listening on {spec} (Ctrl-C to stop)")
    try:
        while seconds is None or time.time() - t0 < seconds:
            m = bus.recv(0.05)
            if m is not None:
                total += 1
                if m.is_error_frame: errs += 1
                else:
                    cmd = m.data[0] if m.dlc else None
                    cnt[(m.arbitration_id, cmd if m.arbitration_id != 0x280 else "iq")] += 1
                    latest[(m.arbitration_id, cmd)] = bytes(m.data)
            if time.time() - last >= 1.0:
                dt = time.time() - last; last = time.time()
                line = [f"{time.strftime('%H:%M:%S')}  load {100 * sum(cnt.values()) * BITS_PER_FRAME / 1e6 / dt:4.1f} %  errors {errs}"]
                for (i, c), n in sorted(cnt.items(), key=lambda x: (x[0][0], str(x[0][1]))):
                    tag = "torque 0x280" if i == 0x280 else f"0x{i:03X} {c:#04x}"
                    line.append(f"   {tag}: {n / dt:5.0f}/s")
                for (i, c), d in sorted(latest.items(), key=lambda x: (x[0][0], x[0][1] or 0)):
                    if i == 0x280 and len(d) >= 4:
                        a, b = struct.unpack("<hh", d[:4]); line.append(f"   cmd: motor1 {iq_nm(a):+.3f} N·m  motor2 {iq_nm(b):+.3f} N·m")
                    elif c in (0x9C, 0xA1) and len(d) == 8 and 0x140 < i < 0x145:
                        t, iq, dps, enc = struct.unpack("<bhhH", d[1:])
                        line.append(f"   motor {i - 0x140}: {t} C  {iq * IQ_FULL / IQ_MAX:+.2f} A  {dps:+5d} dps  enc {enc}")
                print("\n".join(line), flush=True)
                cnt.clear(); errs = 0; latest.clear()
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
    return total


if __name__ == "__main__":
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else None)
