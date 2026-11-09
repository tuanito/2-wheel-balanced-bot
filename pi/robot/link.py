"""UART link to the ESP32 (mirror of firmware/esp32/include/link_proto.h)."""
import struct, threading, time
from dataclasses import dataclass

SYNC = b"\xA5\x5A"
MSG_CMD, MSG_TELEM, MSG_EVENT = 0x01, 0x81, 0x82
F_ARM, F_DISARM, F_AUTO_STANDUP, F_OBST_STOP = 1, 2, 4, 8
CMD_FMT = "<BBHfff"                       # mode, flags, seq, v, yaw_rate, angle
TELEM_FMT = "<IBbBB" + "f" * 12 + "B"     # t_ms, state, side, flags, tries, 12 floats, ir (57 bytes)
STATES = ["DISARMED", "LYING", "LIFT", "COAST", "BALANCE", "FALLEN", "CHARGING", "FAULT",
          "FLIP_RUN", "FLIP_BRAKE", "FLIP_AIR", "DOCK_LAND", "UNDOCK"]
MODE_VELOCITY, MODE_TURN_TO, MODE_TURN_BY, MODE_FLIP, MODE_DOCK, MODE_UNDOCK = 0, 1, 2, 3, 4, 5


def crc8(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    return SYNC + body + bytes([crc8(body)])


@dataclass
class Telemetry:
    t_ms: int = 0
    state: int = 0
    side: int = 0
    flags: int = 0
    tries: int = 0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    vbat: float = 0.0
    sonar: float = -1.0
    tl: float = 0.0
    tr: float = 0.0
    standup_adj: float = 0.0
    motor_temp: float = 0.0
    ir: int = 0           # IR beacon bits: 2i 'all round', 2i+1 'centre'; i 0 FL, 1 FR, 2 BL, 3 BR
    rx_time: float = 0.0

    @property
    def state_name(self): return STATES[self.state] if self.state < len(STATES) else str(self.state)
    @property
    def armed(self): return bool(self.flags & 1)
    @property
    def charging(self): return bool(self.flags & 2)
    @property
    def obstacle(self): return bool(self.flags & 4)
    @property
    def battery_low(self): return bool(self.flags & 8)
    @property
    def motor_fault(self): return bool(self.flags & 16)
    @property
    def power_off_pending(self): return bool(self.flags & 32)   # flat battery: the ESP32 switches the robot off in 20 s

    @staticmethod
    def unpack(p: bytes) -> "Telemetry":
        f = struct.unpack(TELEM_FMT, p)
        return Telemetry(*f, rx_time=time.time())


class Parser:
    """Byte-stream -> (type, payload) frames, resynchronising on garbage."""
    def __init__(self): self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]; break
            if i: del self.buf[:i]
            if len(self.buf) < 4: break
            n = 4 + self.buf[3] + 1
            if len(self.buf) < n: break
            body = bytes(self.buf[2:n - 1])
            if crc8(body) == self.buf[n - 1]:
                out.append((body[0], body[2:]))
                del self.buf[:n]
            else:
                del self.buf[:2]
        return out


class Link:
    """Background reader + command sender. `port` can be any object with read(n)/write(b) (tests use a fake)."""
    def __init__(self, port="/dev/serial0", baud=921600, on_event=None):
        if isinstance(port, str):
            import serial
            port = serial.Serial(port, baud, timeout=0.01)
        self.ser = port
        self.telem = Telemetry()
        self.on_event = on_event or (lambda txt: print("[esp32]", txt))
        self._parser = Parser()
        self._lock = threading.Lock()
        self._run = True
        self.seq = 0
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self._run:
            data = self.ser.read(256)
            if not data:
                time.sleep(0.002); continue
            for t, p in self._parser.feed(data):
                if t == MSG_TELEM and len(p) == struct.calcsize(TELEM_FMT):
                    self.telem = Telemetry.unpack(p)
                elif t == MSG_EVENT:
                    self.on_event(p.decode(errors="replace"))

    def send(self, v=0.0, yaw_rate=0.0, mode=MODE_VELOCITY, angle=0.0, arm=False, disarm=False,
             auto_standup=True, obstacle_stop=True, new_turn=False):
        if new_turn: self.seq = (self.seq + 1) & 0xFFFF
        flags = (F_ARM if arm else 0) | (F_DISARM if disarm else 0) | (F_AUTO_STANDUP if auto_standup else 0) | \
                (F_OBST_STOP if obstacle_stop else 0)
        payload = struct.pack(CMD_FMT, mode, flags, self.seq, float(v), float(yaw_rate), float(angle))
        with self._lock:
            self.ser.write(frame(MSG_CMD, payload))

    def close(self):
        self._run = False
