"""Microphones and speaker on the Pi's I2S bus.

Hardware: 2 x ICS-43434 I2S MEMS microphones in the front panel, 124 mm apart (left: SEL=GND, right: SEL=3.3 V) on PCM_DIN, MAX98357A 3 W amp on
PCM_DOUT, shared BCLK/LRCLK. Enable with  dtoverlay=googlevoicehat-soundcard  (the same mic + amp pair as the
Google AIY Voice Kit). Capture is 48 kHz stereo 32-bit, processed here to 16 kHz mono for speech recognition.

Voice at 2 m: normal speech is ~54 dB SPL there, the ICS-43434 hears it at about -66 dBFS with 29 dBA self-noise,
so the raw signal is quiet but clean. We
  1. high-pass at 100 Hz (motor rumble, handling), decimate to 16 kHz,
  2. steer the two-mic beam to the talker (GCC-PHAT direction estimate) and sum: +3 dB against room noise,
  3. detect speech with an adaptive energy gate,
  4. normalise each utterance to -3 dBFS peak before speech recognition.
The robot is half-duplex: the mics are ignored while the speaker plays (and 0.3 s after), so it never answers itself.
"""
import math, threading, time, queue
import numpy as np

FS_HW = 48000
FS = 16000
MIC_SPACING = 0.124          # m, centre to centre (V1.1 front panel; V2 was 0.086) - set "mic_spacing_m" in config
C_SOUND = 343.0


# ----------------------------------------------------------------------------- signal processing (pure, tested)
class HighPass:
    """2nd-order Butterworth high-pass with state kept between blocks (scipy); falls back to DC removal."""
    def __init__(self, fs=FS, fc=100.0):
        try:
            from scipy.signal import butter, lfilter, lfilter_zi
            self.b, self.a = butter(2, fc / (fs / 2), "highpass")
            self.zi = lfilter_zi(self.b, self.a) * 0.0
            self._lf = lfilter
        except Exception:
            self._lf = None

    def __call__(self, x):
        x = np.asarray(x, dtype=np.float64)
        if self._lf is None: return x - x.mean()
        y, self.zi = self._lf(self.b, self.a, x, zi=self.zi)
        return y


def decimate3(x):
    """48 kHz -> 16 kHz: 31-tap windowed-sinc low-pass at 7 kHz, keep every 3rd sample."""
    n = np.arange(-15, 16)
    h = np.sinc(2 * 7000 / FS_HW * n) * np.hamming(31); h /= h.sum()
    return np.convolve(x, h, mode="same")[::3]


def gcc_phat(left, right, fs=FS, max_tau=MIC_SPACING / C_SOUND, interp=8):
    """Time difference of arrival right-vs-left (s, positive = sound reaches the LEFT mic first) by GCC-PHAT."""
    n = len(left) + len(right)
    L, R = np.fft.rfft(left, n), np.fft.rfft(right, n)
    X = R * np.conj(L)
    f = np.fft.rfftfreq(n, 1 / fs)
    X[(f < 200) | (f > 4000)] = 0                     # speech band only: empty bins would dominate after PHAT
    X /= np.abs(X) ** 0.8 + 1e-6 * (np.abs(X).max() + 1e-30)   # beta-PHAT (0.8) is more robust than pure PHAT
    cc = np.fft.irfft(X, n * interp)
    m = min(int(interp * fs * max_tau), n * interp // 2)
    cc = np.concatenate((cc[-m:], cc[:m + 1]))
    return (np.argmax(np.abs(cc)) - m) / float(interp * fs)


def direction_deg(tau, spacing=MIC_SPACING):
    """Bearing of the talker from straight ahead (deg, + = to the robot's left), from the TDOA."""
    s = max(-1.0, min(1.0, tau * C_SOUND / spacing))
    return math.degrees(math.asin(s))


def beam(left, right, tau, fs=FS):
    """Delay-and-sum toward the talker: delay the mic that hears it first by |tau| (fractional, via FFT) and average."""
    n = len(left)
    k = np.fft.rfftfreq(n, 1 / fs)
    def shift(x, d):
        return np.fft.irfft(np.fft.rfft(x) * np.exp(-2j * np.pi * k * d), n)
    if tau >= 0: left = shift(left, tau)
    else: right = shift(right, -tau)
    return 0.5 * (left + right)


def normalise(x, peak_dbfs=-3.0):
    p = np.max(np.abs(x)) + 1e-12
    return x * (10 ** (peak_dbfs / 20) / p)


def to_wav_bytes(x, fs=FS):
    import io, wave
    b = io.BytesIO()
    with wave.open(b, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(fs)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())
    return b.getvalue()


class SpeechGate:
    """Adaptive energy voice-activity detector on 20 ms frames. feed() returns a finished utterance (np array) or None.
    Starts when energy exceeds the noise floor by `on_db` for 3 frames, ends after `hang` s below `off_db`."""
    def __init__(self, fs=FS, frame=0.02, on_db=9.0, off_db=5.0, hang=0.7, min_len=0.35, max_len=10.0, pre=0.3):
        self.fs, self.n = fs, int(fs * frame)
        self.on_db, self.off_db, self.hang, self.min_len, self.max_len = on_db, off_db, hang, min_len, max_len
        self.floor = None
        self.buf = np.zeros(0)
        self.pre = [np.zeros(0)] * max(1, int(pre / frame))
        self.active, self.utt, self.quiet, self.loud = False, [], 0.0, 0

    def reset(self):
        self.active, self.utt, self.quiet, self.loud = False, [], 0.0, 0

    def feed(self, x):
        self.buf = np.concatenate((self.buf, x))
        out = None
        while len(self.buf) >= self.n:
            f, self.buf = self.buf[:self.n], self.buf[self.n:]
            e = 10 * math.log10(np.mean(f * f) + 1e-14)
            if self.floor is None: self.floor = e
            if not self.active:
                self.floor += (0.05 if e > self.floor else 0.3) * (e - self.floor)   # tracks noise, slowly upwards
                self.loud = self.loud + 1 if e > self.floor + self.on_db else 0
                self.pre = self.pre[1:] + [f]
                if self.loud >= 3:
                    self.active, self.utt, self.quiet = True, list(self.pre), 0.0
            else:
                self.utt.append(f)
                self.quiet = self.quiet + self.n / self.fs if e < self.floor + self.off_db else 0.0
                length = len(self.utt) * self.n / self.fs
                if self.quiet >= self.hang or length >= self.max_len:
                    u = np.concatenate(self.utt); self.reset()
                    if length - self.quiet >= self.min_len: out = u
        return out


class Frontend:
    """(N, 2) float blocks at 48 kHz -> (mono 16 kHz beam toward the talker, bearing deg)."""
    def __init__(self, spacing=MIC_SPACING):
        self.hp = (HighPass(FS_HW), HighPass(FS_HW))
        self.spacing = spacing

    def __call__(self, stereo48):
        l = decimate3(self.hp[0](stereo48[:, 0])); r = decimate3(self.hp[1](stereo48[:, 1]))
        tau = gcc_phat(l, r, max_tau=self.spacing / C_SOUND)
        return beam(l, r, tau), direction_deg(tau, self.spacing)


# ----------------------------------------------------------------------------- hardware
class Audio:
    """Continuous capture + playback on the I2S sound card (sounddevice / PortAudio). Without hardware it still
    works as a no-op so the rest of the robot runs (and the tests can inject audio with feed())."""
    def __init__(self, device=None, gain_db=0.0, on_utterance=None, spacing=MIC_SPACING):
        self.spacing = spacing
        self.on_utterance = on_utterance or (lambda u, bearing: None)
        self.gate = SpeechGate()
        self.gain = 10 ** (gain_db / 20)
        self.speaking_until = 0.0
        self.listening = True            # False = push-to-talk only and not pressed
        self.level_db = -120.0
        self.bearing = 0.0
        self._q = queue.Queue(maxsize=50)
        self.ok = False
        try:
            import sounddevice as sd
            self.sd = sd
            self.stream = sd.InputStream(device=device, samplerate=FS_HW, channels=2, dtype="float32", blocksize=4800,
                                         callback=lambda d, *_: self._q.put_nowait(d.copy()) if not self._q.full() else None)
            self.stream.start(); self.ok = True
        except Exception as e:
            print("audio input unavailable:", e); self.sd = None
        threading.Thread(target=self._run, daemon=True).start()

    def feed(self, stereo48):
        self._q.put(np.asarray(stereo48, dtype=np.float64))

    def _run(self):
        bearings = []
        fe = Frontend(self.spacing)
        while True:
            d = self._q.get()
            mono, b = fe(d * self.gain)
            self.level_db = 20 * math.log10(np.sqrt(np.mean(mono * mono)) + 1e-12)
            if time.time() < self.speaking_until or not self.listening:
                self.gate.reset(); continue
            if self.gate.active: bearings.append(b)
            u = self.gate.feed(mono)
            if u is not None:
                self.bearing = float(np.median(bearings)) if bearings else b
                bearings = []
                self.on_utterance(normalise(u), self.bearing)

    def play(self, x, fs):
        """Play mono float audio (blocking). Mutes the microphones meanwhile."""
        x = np.asarray(x, dtype=np.float32)
        dur = len(x) / fs
        self.speaking_until = time.time() + dur + 0.3
        if self.sd is None:
            time.sleep(min(dur, 0.05)); return
        if fs != FS_HW:                                  # simple linear resample to the card rate
            t = np.arange(int(len(x) * FS_HW / fs)) * fs / FS_HW
            x = np.interp(t, np.arange(len(x)), x).astype(np.float32)
        self.sd.play(x, FS_HW, blocking=True)
        self.speaking_until = time.time() + 0.3
