"""Two 8x8 WS2812B-2020 panels as eyes, driven from the Pi's SPI MOSI (GPIO10) through the 74AHCT125.
Expressions follow the robot state; the pupils look where the robot is turning. Brightness is capped in software
(128 LEDs at full white would draw ~5 A from the 5 V regulator)."""
import math, threading, time

N = 64
MAX_BRIGHT = 0.25


def _disc(cx, cy, r):
    return {(x, y) for x in range(8) for y in range(8) if (x - cx) ** 2 + (y - cy) ** 2 <= r * r}


SHAPES = {
    "open": _disc(3.5, 3.5, 3.6),
    "half": {p for p in _disc(3.5, 3.5, 3.6) if p[1] >= 4},
    "closed": {(x, 5) for x in range(1, 7)},
    "happy": {(x, y) for x in range(8) for y in range(8) if 2.2 <= math.hypot(x - 3.5, y - 6.5) <= 3.6 and y <= 5},
    "x": {(i, i) for i in range(1, 7)} | {(i, 7 - i) for i in range(1, 7)},
}


class Eyes:
    def __init__(self, serpentine=True):
        self.serpentine = serpentine
        self.mode, self.look, self.color, self.light = "open", 0.0, (0, 160, 255), False
        try:
            import board, neopixel_spi
            self.px = neopixel_spi.NeoPixel_SPI(board.SPI(), 2 * N, brightness=MAX_BRIGHT, auto_write=False,
                                                pixel_order=neopixel_spi.GRB)
        except Exception as e:
            print("eyes unavailable:", e); self.px = None
        threading.Thread(target=self._run, daemon=True).start()

    def idx(self, panel, x, y):
        if self.serpentine and y % 2: x = 7 - x
        return panel * N + y * 8 + x

    def override(self, shape, color, seconds):
        """Expression chosen by the voice agent, for a while."""
        self._ov = (shape if shape in SHAPES else "open", color, time.time() + seconds)

    def set_from_telemetry(self, t, yaw_rate_cmd=0.0):
        s = t.state_name
        self.look = max(-1.0, min(1.0, yaw_rate_cmd))
        ov = getattr(self, "_ov", None)
        if ov and time.time() < ov[2] and s not in ("FAULT", "CHARGING"):
            self.mode, self.color = ov[0], ov[1] or self.color; return
        if s == "CHARGING": self.mode, self.color = "closed", (0, 60, 160)
        elif s in ("FAULT",) or t.battery_low: self.mode, self.color = "x", (255, 30, 0)
        elif s in ("LYING", "FALLEN", "LIFT", "COAST"): self.mode, self.color = "open", (255, 150, 0)
        elif s == "DISARMED": self.mode, self.color = "half", (80, 80, 120)
        elif t.obstacle: self.mode, self.color = "open", (255, 220, 0)
        else: self.mode, self.color = "open", (0, 170, 255)

    def _run(self):
        blink_at = time.time() + 3
        while True:
            now = time.time()
            shape = SHAPES["closed"] if now > blink_at else SHAPES[self.mode]
            if now > blink_at + 0.12: blink_at = now + 2.5 + 3 * ((now * 7.3) % 1)
            if self.px is not None:
                self.px.fill((0, 0, 0))
                if self.light:                                  # headlight mode for the camera at night
                    self.px.fill((255, 255, 255))
                else:
                    dx = int(round(-1.5 * self.look))           # positive yaw = turning left: pupils move left
                    for p in (0, 1):
                        for (x, y) in shape:
                            pupil = self.mode == "open" and abs(x - 3.5 - dx) < 1.1 and abs(y - 3.5) < 1.1
                            c = (255, 255, 255) if pupil else self.color
                            self.px[self.idx(p, x, y)] = c
                self.px.show()
            time.sleep(0.04)
