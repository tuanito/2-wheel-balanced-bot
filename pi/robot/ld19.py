"""LDROBOT LD19 lidar driver (230400 baud, 47-byte packets, ~4500 points/s, 10 Hz).

Packet: 0x54 0x2C | speed u16 (deg/s) | start angle u16 (0.01 deg) | 12 x (distance u16 mm, intensity u8)
        | end angle u16 (0.01 deg) | timestamp u16 (ms) | crc8 (poly 0x4D)
The LD19 measures angles clockwise seen from above; the robot frame is counter-clockwise (X forward, Y left).
"""
import math, struct, threading, time
import numpy as np

HEADER, VERLEN, PKT_LEN, N_PTS = 0x54, 0x2C, 47, 12


def _crc_table(poly=0x4D):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC_TABLE = _crc_table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data: c = CRC_TABLE[(c ^ b) & 0xFF]
    return c


def parse_packet(p: bytes):
    """-> (angles_deg[12] lidar frame, dist_m[12], intensity[12], speed_dps) or None if invalid."""
    if len(p) != PKT_LEN or p[0] != HEADER or p[1] != VERLEN or crc8(p[:-1]) != p[-1]:
        return None
    speed, start = struct.unpack_from("<HH", p, 2)
    pts = struct.unpack_from("<" + "HB" * N_PTS, p, 6)
    end, _ts = struct.unpack_from("<HH", p, 6 + 3 * N_PTS)
    start, end = start / 100.0, end / 100.0
    span = (end - start) % 360.0
    ang = (start + span * np.arange(N_PTS) / (N_PTS - 1)) % 360.0
    dist = np.array(pts[0::2], dtype=float) / 1000.0
    inten = np.array(pts[1::2], dtype=float)
    return ang, dist, inten, speed


class Scan:
    def __init__(self, t, angles, ranges, intensity):
        self.t, self.angles, self.ranges, self.intensity = t, angles, ranges, intensity   # robot frame, rad / m

    def points(self, min_r=0.05, max_r=8.0, min_intensity=10):
        ok = (self.ranges > min_r) & (self.ranges < max_r) & (self.intensity >= min_intensity)
        a, r = self.angles[ok], self.ranges[ok]
        return np.stack([r * np.cos(a), r * np.sin(a)], 1)


LIDAR_HEIGHT = 0.231   # m, scan plane above the floor when the robot stands upright (CAD)


def level_scan(scan: "Scan", pitch: float, roll: float = 0.0, height: float = LIDAR_HEIGHT) -> "Scan":
    """The robot leans (up to ~17° when braking): the lidar plane tilts with it. Drop the rays that hit the floor and
    project the others onto the horizontal plane. pitch > 0 = leaning forward (towards robot +X), roll > 0 = left up."""
    a = scan.angles
    tilt = pitch * np.cos(a) - roll * np.sin(a)               # downward elevation of each ray
    r = scan.ranges
    drop = r * np.sin(np.clip(tilt, 0, None))
    floor = (r > 0) & (drop > height - 0.03)
    r2 = np.where(floor, 0.0, r * np.cos(tilt))
    return Scan(scan.t, a, r2, scan.intensity)


class LD19:
    def __init__(self, port="/dev/ttyUSB0", offset_deg=0.0, clockwise=True, on_scan=None, self_mask_m=0.06):
        """offset_deg: robot-frame angle of the lidar's 0° mark (set it so a wall straight ahead reads at 0°)."""
        if isinstance(port, str):
            import serial
            port = serial.Serial(port, 230400, timeout=0.05)
        self.ser, self.offset, self.cw = port, math.radians(offset_deg), clockwise
        self.on_scan = on_scan
        self.self_mask = self_mask_m
        self.latest = None
        self._acc = []
        self._last_ang = None
        self._buf = bytearray()
        self._run = True
        threading.Thread(target=self._reader, daemon=True).start()

    def to_robot(self, ang_deg):
        a = np.radians(ang_deg)
        return (self.offset - a) if self.cw else (self.offset + a)

    def feed(self, data: bytes):
        self._buf += data
        while True:
            i = self._buf.find(bytes([HEADER, VERLEN]))
            if i < 0:
                del self._buf[:-1]; return
            if len(self._buf) - i < PKT_LEN:
                del self._buf[:i]; return
            pkt = bytes(self._buf[i:i + PKT_LEN])
            res = parse_packet(pkt)
            if res is None:
                del self._buf[:i + 1]; continue
            del self._buf[:i + PKT_LEN]
            ang, dist, inten, _ = res
            if self._last_ang is not None and ang[0] < self._last_ang - 180:   # wrapped past 0°: one revolution done
                self._emit()
            self._last_ang = ang[-1]
            self._acc.append((ang, dist, inten))

    def _emit(self):
        if not self._acc: return
        ang = np.concatenate([a for a, _, _ in self._acc]); d = np.concatenate([x for _, x, _ in self._acc])
        it = np.concatenate([x for _, _, x in self._acc]); self._acc = []
        d = np.where(d < self.self_mask, 0.0, d)
        a = np.arctan2(np.sin(self.to_robot(ang)), np.cos(self.to_robot(ang)))
        order = np.argsort(a)
        scan = Scan(time.time(), a[order], d[order], it[order])
        self.latest = scan
        if self.on_scan: self.on_scan(scan)

    def _reader(self):
        while self._run:
            data = self.ser.read(512)
            if data: self.feed(data)
            else: time.sleep(0.005)

    def close(self): self._run = False


def make_packet(start_deg, end_deg, dists_mm, intens=None, speed=3600, ts=0) -> bytes:
    """Build a valid packet (used by the tests and the simulator bridge)."""
    intens = intens if intens is not None else [200] * N_PTS
    body = struct.pack("<BBHH", HEADER, VERLEN, speed, int(round(start_deg * 100)) % 36000)
    for d, i in zip(dists_mm, intens): body += struct.pack("<HB", int(d), int(i))
    body += struct.pack("<HH", int(round(end_deg * 100)) % 36000, ts)
    return body + bytes([crc8(body)])
