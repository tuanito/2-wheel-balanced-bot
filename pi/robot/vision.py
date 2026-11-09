"""What the AI Camera sees: people and cats, and who the people are.

Detection runs ON the camera (Sony IMX500, an SSD MobileNet COCO model, ~30 fps): the Pi only receives boxes in the
frame metadata. Identity runs on the Pi: a face detector (OpenCV YuNet) + face embedding (OpenCV SFace) on the face
inside a person box, compared with embeddings of the photos in <data_dir>/people/<name>/*.jpg. A recognised person is
then tracked frame to frame by box overlap + clothing colour, so the robot keeps following them from behind.

Pure helpers (iou, colour histogram, Tracker, FaceBook matching) have no camera dependency and are unit-tested."""
import math, pathlib, threading, time
import numpy as np

MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
YUNET = "/home/pi/models/face_detection_yunet_2023mar.onnx"
SFACE = "/home/pi/models/face_recognition_sface_2021dec.onnx"
WANT = {"person", "cat"}
MIN_SCORE = 0.5
MATCH = 0.36            # SFace cosine similarity above this = same person (OpenCV's recommended threshold 0.363)


class Det:
    __slots__ = ("label", "score", "box", "name", "sim", "hist")

    def __init__(self, label, score, box, name=None, sim=0.0, hist=None):
        self.label, self.score, self.box, self.name, self.sim, self.hist = label, float(score), tuple(map(float, box)), name, sim, hist

    def as_dict(self):
        return dict(label=self.label, score=round(self.score, 2), box=[round(v, 3) for v in self.box],
                    name=self.name, sim=round(self.sim, 2))


def iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def colour_hist(rgb, box, bins=8):
    """Normalised HSV-ish hue/saturation histogram of the torso (middle of the person box): clothing colour."""
    h, w = rgb.shape[:2]
    x0, y0, x1, y1 = box
    ty0, ty1 = y0 + 0.25 * (y1 - y0), y0 + 0.6 * (y1 - y0)
    crop = rgb[int(ty0 * h):max(int(ty1 * h), int(ty0 * h) + 1), int(x0 * w):max(int(x1 * w), int(x0 * w) + 1)].reshape(-1, 3).astype(np.float32)
    if crop.size == 0: return None
    mx, mn = crop.max(1), crop.min(1)
    sat = np.where(mx > 0, (mx - mn) / np.maximum(mx, 1), 0)
    r, g, b = crop[:, 0], crop[:, 1], crop[:, 2]
    hue = (np.degrees(np.arctan2(math.sqrt(3) * (g - b), 2 * r - g - b)) + 360) % 360
    hist, _, _ = np.histogram2d(hue, sat, bins=bins, range=[[0, 360], [0, 1]])
    hist = hist.ravel(); s = hist.sum()
    return hist / s if s else None


def hist_sim(a, b):
    if a is None or b is None: return 0.0
    return float(np.sqrt(a * b).sum())        # Bhattacharyya coefficient, 1 = identical


class Tracker:
    """Keeps one chosen person across frames: box overlap first, clothing colour when overlap is ambiguous.
    `name` (optional) restricts the lock to a recognised person; once locked, the lock survives frames without a face."""

    def __init__(self, name=None, lost_after=1.5):
        self.name, self.lost_after = name, lost_after
        self.box = self.hist = None
        self.t_seen = 0.0

    @property
    def locked(self):
        return self.box is not None

    def update(self, dets, now):
        people = [d for d in dets if d.label == "person"]
        if not self.locked:
            cands = [d for d in people if self.name is None or d.name == self.name]
            if not cands: return None
            d = max(cands, key=lambda d: (d.box[2] - d.box[0]) * (d.box[3] - d.box[1]))     # nearest = biggest
            self.box, self.hist, self.t_seen = d.box, d.hist, now
            return d
        if now - self.t_seen > self.lost_after:
            self.box = self.hist = None
            return self.update(dets, now)
        best, best_s = None, 0.25
        for d in people:
            if self.name and d.name and d.name != self.name: continue        # a recognised *other* person: never switch
            s = iou(self.box, d.box) + 0.6 * hist_sim(self.hist, d.hist) + (0.5 if self.name and d.name == self.name else 0)
            if s > best_s: best, best_s = d, s
        if best is not None:
            self.box, self.t_seen = best.box, now
            if best.hist is not None:
                self.hist = best.hist if self.hist is None else 0.9 * self.hist + 0.1 * best.hist
        return best


def follow_cmd(box, target_h=0.6, v_max=0.5, w_max=1.2):
    """Velocity command that keeps a person box centred and at a set apparent size (box height ~ distance).
    box in normalised image coords (x0, y0, x1, y1). Returns (v m/s, w rad/s); camera faces forward, +w = turn left."""
    if box is None: return 0.0, 0.0
    cx = 0.5 * (box[0] + box[2]); h = box[3] - box[1]
    w = max(-w_max, min(w_max, -2.4 * (cx - 0.5)))                 # 66° view: 0.5 off-centre ~ 33°
    v = 1.2 * (target_h - h)                                        # too small = too far -> forward
    if abs(cx - 0.5) > 0.3: v *= 0.3                                # turn first when the person is at the edge
    return max(-0.25, min(v_max, v)), w


class FaceBook:
    """Embeddings of the enrolled people. add(name, feature) / match(feature) -> (name, similarity)."""

    def __init__(self):
        self.people = {}

    def add(self, name, feat):
        f = np.asarray(feat, np.float32).ravel(); f = f / (np.linalg.norm(f) or 1)
        self.people.setdefault(name, []).append(f)

    def match(self, feat):
        if feat is None or not self.people: return None, 0.0
        f = np.asarray(feat, np.float32).ravel(); f = f / (np.linalg.norm(f) or 1)
        best, bs = None, -1.0
        for n, fs in self.people.items():
            s = max(float(f @ g) for g in fs)
            if s > bs: best, bs = n, s
        return (best, bs) if bs >= MATCH else (None, bs)


class FaceID:
    """OpenCV YuNet + SFace on the Pi CPU. Only run on face-sized crops of person boxes, a few times a second."""

    def __init__(self, people_dir, yunet=YUNET, sface=SFACE):
        self.book, self.ok = FaceBook(), False
        try:
            import cv2
            self.cv2 = cv2
            self.det = cv2.FaceDetectorYN.create(yunet, "", (320, 320), 0.8)
            self.rec = cv2.FaceRecognizerSF.create(sface, "")
            self.ok = True
        except Exception as e:
            print("face ID unavailable:", e); return
        self.enroll_dir(people_dir)

    def feature(self, bgr):
        h, w = bgr.shape[:2]
        self.det.setInputSize((w, h))
        _, faces = self.det.detect(bgr)
        if faces is None or len(faces) == 0: return None
        f = max(faces, key=lambda r: r[2] * r[3])
        return self.rec.feature(self.rec.alignCrop(bgr, f))

    def enroll_dir(self, people_dir):
        n = 0
        for d in sorted(pathlib.Path(people_dir).glob("*")):
            if not d.is_dir(): continue
            for img in sorted(d.glob("*.jp*g")) + sorted(d.glob("*.png")):
                im = self.cv2.imread(str(img))
                f = self.feature(im) if im is not None else None
                if f is not None: self.book.add(d.name, f); n += 1
                else: print("no face found in", img)
        print(f"face ID: {n} photos of {len(self.book.people)} people")

    def identify(self, rgb, box):
        h, w = rgb.shape[:2]
        x0, y0, x1, y1 = box
        crop = rgb[int(y0 * h):int((y0 + 0.45 * (y1 - y0)) * h), int(x0 * w):int(x1 * w)]   # head + shoulders
        if crop.shape[0] < 40 or crop.shape[1] < 30: return None, 0.0                       # too far for a face
        return self.book.match(self.feature(self.cv2.cvtColor(crop, self.cv2.COLOR_RGB2BGR)))


class Vision:
    """Reads the IMX500 detections from each frame's metadata; every ~0.4 s also grabs a frame for colour + faces."""

    def __init__(self, camera, people_dir, model=MODEL):
        self.camera, self.latest, self.t = camera, [], 0.0
        self.lock = threading.Lock()
        self.faces = FaceID(people_dir)
        self.imx = getattr(camera, "imx500", None)
        if self.imx is None or not getattr(camera, "ok", False):
            print("vision: no AI camera (detections off)"); return
        try:
            self.labels = list(self.imx.network_intrinsics.labels)
        except Exception:
            self.labels = [l.strip() for l in open("/usr/share/imx500-models/coco_labels.txt")] if pathlib.Path("/usr/share/imx500-models/coco_labels.txt").exists() else []
        threading.Thread(target=self._run, daemon=True).start()

    def _parse(self, md):
        out = self.imx.get_outputs(md, add_batch=True)
        if out is None: return []
        boxes, scores, classes = out[0][0], out[1][0], out[2][0]
        iw, ih = self.imx.get_input_size()
        dets = []
        for b, s, c in zip(boxes, scores, classes):
            if s < MIN_SCORE: continue
            lab = self.labels[int(c)] if int(c) < len(self.labels) else str(int(c))
            if lab not in WANT: continue
            y0, x0, y1, x1 = b                           # SSD post-processed: normalised (y0, x0, y1, x1) of the input tensor
            if max(b) > 1.5: y0, x0, y1, x1 = y0 / ih, x0 / iw, y1 / ih, x1 / iw
            dets.append(Det(lab, s, (max(0, x0), max(0, y0), min(1, x1), min(1, y1))))
        return dets

    def _run(self):
        last_frame = 0.0
        while True:
            try:
                md = self.camera.cam.capture_metadata()
                dets = self._parse(md)
                now = time.time()
                if dets and now - last_frame > 0.4 and any(d.label == "person" for d in dets):
                    last_frame = now
                    rgb = self.camera.cam.capture_array("main")[:, :, :3]
                    for d in dets:
                        if d.label != "person": continue
                        d.hist = colour_hist(rgb, d.box)
                        if self.faces.ok: d.name, d.sim = self.faces.identify(rgb, d.box)
                with self.lock:
                    self.latest, self.t = dets, now
            except Exception as e:
                print("vision:", e); time.sleep(0.5)

    def snapshot(self, max_age=0.5):
        with self.lock:
            return list(self.latest) if time.time() - self.t < max_age else []

    def enroll(self, name, jpeg_bytes_list, people_dir):
        """Save photos to <people_dir>/<name>/ and add them to the face book."""
        d = pathlib.Path(people_dir) / name; d.mkdir(parents=True, exist_ok=True)
        added = 0
        for i, b in enumerate(jpeg_bytes_list):
            p = d / f"{int(time.time())}_{i}.jpg"; p.write_bytes(b)
            if self.faces.ok:
                im = self.faces.cv2.imread(str(p)); f = self.faces.feature(im) if im is not None else None
                if f is not None: self.faces.book.add(name, f); added += 1
        return added
