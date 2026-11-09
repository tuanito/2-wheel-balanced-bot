"""Raspberry Pi AI Camera (or Camera Module 3) -> MJPEG stream, encoded on the Pi's hardware JPEG/ISP path (Picamera2).

MJPEG works in every browser with a plain <img> tag and costs little CPU. For lower bandwidth use WebRTC via
mediamtx (see README: 'Lower-latency video'), which needs no change here.
"""
import io, threading, time


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.cond = threading.Condition()
        self.count = 0

    def write(self, buf):
        with self.cond:
            self.frame = bytes(buf); self.count += 1
            self.cond.notify_all()
        return len(buf)

    def wait(self, last_count, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.count != last_count, timeout)
            return self.frame, self.count


class Camera:
    def __init__(self, size=(800, 600), fps=20, quality=70, ai_model=None):
        self.output = StreamingOutput()
        self.ok = False
        self.imx500 = None
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import MJPEGEncoder, Quality
            from picamera2.outputs import FileOutput
            num = None
            if ai_model:      # AI Camera: load the network onto the IMX500 before opening the camera
                try:
                    from picamera2.devices import IMX500
                    self.imx500 = IMX500(ai_model); num = self.imx500.camera_num
                except Exception as e:
                    print("AI camera network not loaded:", e)
            self.cam = Picamera2(num) if num is not None else Picamera2()
            cfg = self.cam.create_video_configuration(main={"size": size, "format": "RGB888"}, controls={"FrameRate": fps}, buffer_count=6)
            self.cam.configure(cfg)
            q = Quality.MEDIUM if quality < 80 else Quality.HIGH
            self.cam.start_recording(MJPEGEncoder(), FileOutput(self.output), quality=q)
            try:   # continuous autofocus on the Camera Module 3 (the AI Camera has manual focus: no-op)
                from libcamera import controls
                self.cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
            except Exception:
                pass
            self.ok = True
        except Exception as e:        # no camera (e.g. on a laptop): serve a placeholder
            print("camera unavailable:", e)
            threading.Thread(target=self._placeholder, daemon=True).start()

    def _placeholder(self):
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return
        while True:
            im = Image.new("RGB", (640, 480), (40, 44, 52)); d = ImageDraw.Draw(im)
            d.text((20, 220), time.strftime("no camera  %H:%M:%S"), fill=(220, 220, 220))
            b = io.BytesIO(); im.save(b, "JPEG"); self.output.write(b.getvalue())
            time.sleep(0.5)

    def close(self):
        if self.ok: self.cam.stop_recording()
