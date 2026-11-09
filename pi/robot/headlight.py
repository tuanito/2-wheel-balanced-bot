"""Two 12 V COB disc headlights, switched by an IRLB8721 from the Pi's hardware PWM0 on GPIO12 (header pin 32).
Needs `dtoverlay=pwm,pin=12,func=4` in /boot/firmware/config.txt (install.sh adds it); then PWM0 appears as
/sys/class/pwm/pwmchip0/pwm0. 1 kHz: far above the camera's frame rate, so no banding in the stream.
Brightness is capped (MAX_LEVEL): the discs sit in PETG pockets and 2.4 W each at full power runs them hot."""
import os

CHIP = "/sys/class/pwm/pwmchip0"
PERIOD_NS = 1_000_000          # 1 kHz
MAX_LEVEL = 0.7


class Headlight:
    def __init__(self, chip=CHIP):
        self.chip, self.level, self.ok = chip, 0.0, False
        try:
            pwm = os.path.join(chip, "pwm0")
            if not os.path.isdir(pwm):
                with open(os.path.join(chip, "export"), "w") as f: f.write("0")
            self._w("period", PERIOD_NS); self._w("duty_cycle", 0); self._w("enable", 1)
            self.ok = True
        except Exception as e:
            print("headlight unavailable:", e)

    def _w(self, name, v):
        with open(os.path.join(self.chip, "pwm0", name), "w") as f: f.write(str(int(v)))

    def set(self, level):
        """level 0..1 of the allowed maximum (0 = off)."""
        self.level = max(0.0, min(1.0, float(level)))
        if self.ok:
            try: self._w("duty_cycle", round(PERIOD_NS * MAX_LEVEL * self.level))
            except Exception as e: print("headlight:", e)
        return self.level
