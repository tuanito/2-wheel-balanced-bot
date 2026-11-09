"""Tests for the Pi software. Run:  cd pi && python3 -m pytest -q tests"""
import math, struct, subprocess, sys, time, pathlib, threading, json
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pi"))
from robot import link as L
from robot import ld19
from robot.mapper import Mapper, GridMap
from robot.navigation import Avoider, Planner, ROBOT_RADIUS


class FakeSerial:
    def __init__(self): self.rx = bytearray(); self.tx = bytearray(); self.lock = threading.Lock()
    def read(self, n):
        with self.lock:
            d = bytes(self.rx[:n]); del self.rx[:n]; return d
    def write(self, b):
        with self.lock: self.tx += b
    def inject(self, b):
        with self.lock: self.rx += b


# ---------------------------------------------------------------- link
def test_crc_and_frame_roundtrip():
    p = L.Parser()
    cmd = struct.pack(L.CMD_FMT, 2, L.F_ARM | L.F_OBST_STOP, 7, 0.5, -1.0, 1.57)
    frames = p.feed(b"\x00\xff\xA5garbage" + L.frame(L.MSG_CMD, cmd) + b"\xA5\x5A\x01\x10" + b"x" * 17)   # 2nd frame: bad CRC
    assert frames == [(L.MSG_CMD, cmd)]


def test_telemetry_layout_matches_the_firmware_header(tmp_path):
    """Compile a tiny C++ program with the REAL firmware header, let it emit a telemetry frame, parse it in Python."""
    src = tmp_path / "t.cpp"
    src.write_text(r'''
#include <cstdio>
#include <cstring>
#include "link_proto.h"
int main() {
  pilink::TelemMsg t{}; t.t_ms = 123456; t.state = 4; t.side = -1; t.flags = 5; t.tries = 2;
  t.pitch = 0.1f; t.roll = -0.2f; t.yaw = 3.0f; t.x = 1.5f; t.y = -2.5f; t.v = 0.75f; t.vbat = 23.1f; t.sonar = 0.42f;
  t.tl = 0.3f; t.tr = -0.3f; t.standup_adj = 4.0f; t.motor_temp = 41.0f; t.ir = 0x21;
  unsigned char hdr[4] = {pilink::SYNC0, pilink::SYNC1, pilink::MSG_TELEM, (unsigned char)sizeof t};
  unsigned char crc = pilink::crc8(hdr + 2, 2); crc = pilink::crc8((const unsigned char*)&t, sizeof t, crc);
  fwrite(hdr, 1, 4, stdout); fwrite(&t, 1, sizeof t, stdout); fwrite(&crc, 1, 1, stdout);
}''')
    exe = tmp_path / "t"
    subprocess.run(["g++", "-std=c++17", "-I", str(ROOT / "firmware/esp32/include"), str(src), "-o", str(exe)], check=True)
    out = subprocess.run([str(exe)], capture_output=True, check=True).stdout
    frames = L.Parser().feed(out)
    assert len(frames) == 1 and frames[0][0] == L.MSG_TELEM
    t = L.Telemetry.unpack(frames[0][1])
    assert (t.t_ms, t.state_name, t.side, t.tries) == (123456, "BALANCE", -1, 2)
    assert t.armed and not t.charging and t.obstacle
    assert t.x == pytest.approx(1.5) and t.vbat == pytest.approx(23.1) and t.motor_temp == pytest.approx(41.0)
    assert t.ir == 0x21


def test_link_sends_commands_and_reads_telemetry():
    fs = FakeSerial()
    events = []
    lk = L.Link(fs, on_event=events.append)
    payload = struct.pack(L.TELEM_FMT, 1, 4, 0, 1, 0, *([0.0] * 12), 0)
    fs.inject(L.frame(L.MSG_TELEM, payload) + L.frame(L.MSG_EVENT, b"hello"))
    time.sleep(0.05)
    assert lk.telem.state_name == "BALANCE" and events == ["hello"]
    lk.send(v=0.4, yaw_rate=0.2, arm=True)
    (t, p), = L.Parser().feed(bytes(fs.tx))
    mode, flags, seq, v, w, a = struct.unpack(L.CMD_FMT, p)
    assert t == L.MSG_CMD and flags & L.F_ARM and v == pytest.approx(0.4) and w == pytest.approx(0.2)
    lk.close()


# ---------------------------------------------------------------- lidar
def _revolution(dist_mm=1000):
    pkts = b""
    for k in range(38):                                       # 38 packets x 12 points ≈ one turn
        a0 = k * 360 / 38; a1 = a0 + 360 / 38 * 11 / 12
        pkts += ld19.make_packet(a0, a1, [dist_mm] * 12)
    return pkts


def test_ld19_packets_to_scan():
    got = []
    dev = ld19.LD19(FakeSerial(), offset_deg=0.0, clockwise=True, on_scan=got.append)
    data = b"\x12\x34" + _revolution(1000) + b"\x54\x2c\x00" + _revolution(1000) + _revolution(1000)[:47]
    dev.feed(data)
    assert len(got) >= 1
    s = got[0]
    assert np.allclose(s.ranges[s.ranges > 0], 1.0)
    pts = s.points()
    assert np.allclose(np.linalg.norm(pts, axis=1), 1.0)
    # a point the lidar reports at 90° (clockwise) is on the robot's right, i.e. -90° in the robot frame
    assert np.isclose(dev.to_robot(np.array([90.0]))[0], -math.pi / 2)


def test_bad_crc_packet_is_ignored():
    p = bytearray(ld19.make_packet(0, 10, [500] * 12)); p[10] ^= 0xFF
    assert ld19.parse_packet(bytes(p)) is None


def test_level_scan_drops_floor_hits():
    s = ld19.Scan(0, np.array([0.0, math.pi]), np.array([3.0, 3.0]), np.array([200.0, 200.0]))
    lv = ld19.level_scan(s, pitch=math.radians(6))           # leaning forward: the front ray hits the floor at ~2 m
    assert lv.ranges[0] == 0.0 and lv.ranges[1] > 2.9


# ---------------------------------------------------------------- mapping & navigation
def _room_scan(pose, n=360, half=2.0):
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False); r = []
    for a in ang:
        th = pose[2] + a; c, s = math.cos(th), math.sin(th); best = 9.0
        for px, py, nx, ny in ((half, 0, 1, 0), (-half, 0, -1, 0), (0, half, 0, 1), (0, -half, 0, -1)):
            den = c * nx + s * ny
            if den > 1e-6:
                t = ((px - pose[0]) * nx + (py - pose[1]) * ny) / den
                best = min(best, t) if t > 0 else best
        r.append(best)
    return ld19.Scan(time.time(), ang, np.array(r), np.full(n, 200.0))


def test_mapper_builds_walls_and_corrects_heading(tmp_path):
    m = Mapper(folder=str(tmp_path), log_scans=True)
    true = (0.2, -0.3, 0.4)
    for _ in range(10): m.update(true, _room_scan(true))
    occ = m.grid.occupied_world()
    assert len(occ) > 200
    assert np.all(np.max(np.abs(occ), axis=1) > 1.85)          # every occupied cell is on the 4 x 4 m walls
    drifted = (0.2, -0.3, 0.4 + math.radians(2))               # gyro drift: odometry heading 2° off
    for _ in range(4): p = m.update(drifted, _room_scan(true))
    assert abs(p[2] - true[2]) < math.radians(0.6)
    m.save()
    assert (tmp_path / "map.png").exists() and list((tmp_path / "scans").glob("*.npz"))


def test_avoider_passes_free_path_and_stops_for_walls():
    av = Avoider()
    assert av.safe_command(0.5, 0.0, np.zeros((0, 2))) == (0.5, 0.0, False)
    wall = np.array([[0.35, y] for y in np.linspace(-1, 1, 41)])
    v, w, blocked = av.safe_command(0.6, 0.0, wall)
    from robot.navigation import arc_points, clearance_along, DECEL
    n = max(1, int(math.ceil((abs(v) / DECEL + 0.3) / 0.1)))
    assert blocked or clearance_along(arc_points(v, w), wall)[:n].min() >= ROBOT_RADIUS
    v2, w2, _ = av.safe_command(0.5, 0.0, np.zeros((0, 2)), sonar=0.15)   # sonar-only obstacle right ahead
    assert v2 < 0.2


def test_planner_finds_way_around_a_wall():
    g = GridMap(size_m=8, res=0.05)
    g.L[:] = -2.0                                              # all known free
    c = g.to_cell(np.array([[1.0, y] for y in np.arange(-2, 2, 0.02)]))
    g.L[c[:, 1], c[:, 0]] = 3.0                                # a wall at x = 1 from y = -2 to 2
    path = Planner().search(g, (0.0, 0.0), goal_xy=(2.0, 0.0))
    assert path is not None
    assert max(p[1] for p in path) > 2.0 or min(p[1] for p in path) < -2.0


def test_sonar_mapping_without_lidar(tmp_path):
    """V2 (no lidar): turning on the spot between two walls (x = +-1 m), the sonar (nearest echo in its 15 deg cone)
    marks the walls and clears the space in between; the avoider remembers walls the sonar no longer points at."""
    m = Mapper(folder=str(tmp_path), match=False, log_scans=False)
    for k in range(720):
        th = math.radians(k * 0.5)
        cone = th + np.radians(np.linspace(-7.5, 7.5, 31))
        c = np.cos(cone)
        rng = np.where(np.abs(c) > 1e-6, 1.0 / np.maximum(np.abs(c), 1e-6), 99.0)
        r = float(rng.min()) - 0.031
        m.update_sonar((0.0, 0.0, th), r if r < 1.9 else -1.0)
    occ = m.grid.occupied_world()
    assert len(occ) > 20
    assert np.mean(np.abs(np.abs(occ[:, 0]) - 1.0) < 0.15) > 0.7          # most occupied cells are on the walls
    cx, cy = m.grid.to_cell(np.array([[0.5, 0.0]]))[0]
    assert m.grid.L[cy, cx] < 0                                         # space between robot and wall is free
    m.pose = (0.0, 0.0, math.pi / 2)                                    # facing along the walls now
    obs = m.nearby_obstacles(radius=1.5)
    assert len(obs) and np.all(np.abs(obs[:, 1]) > 0.7)                 # walls left and right, in the robot frame


# ---------------------------------------------------------------- whole Pi app with fake hardware
async def test_web_app_joystick_reaches_the_esp32(aiohttp_client, tmp_path):
    from robot.brain import Brain
    from robot.camera import Camera
    from robot.server import make_app
    esp, lidar = FakeSerial(), FakeSerial()
    brain = Brain(dict(esp32_port=esp, lidar_port=lidar, eyes=False, data_dir=str(tmp_path), log_scans=False,
                       lidar=bool(getattr(test_web_app_joystick_reaches_the_esp32, "lidar", False))))
    payload = struct.pack(L.TELEM_FMT, 1, 4, 0, 1, 0, *([0.0] * 12), 0)          # balancing, armed
    esp.inject(L.frame(L.MSG_TELEM, payload))
    cam = Camera.__new__(Camera)
    from robot.camera import StreamingOutput
    cam.output = StreamingOutput(); cam.ok = False
    client = await aiohttp_client(make_app(brain, cam))
    assert (await client.get("/")).status == 200
    assert (await client.get("/map.png")).status == 200
    ws = await client.ws_connect("/ws")
    st = json.loads((await ws.receive()).data)
    assert st["state"] == "BALANCE"
    import asyncio
    esp.tx.clear()
    for _ in range(6):
        await ws.send_str(json.dumps({"type": "joy", "v": 0.5, "w": 0.3})); await asyncio.sleep(0.05)
    cmds = [struct.unpack(L.CMD_FMT, p) for t, p in L.Parser().feed(bytes(esp.tx)) if t == L.MSG_CMD]
    assert cmds and any(abs(c[3] - 0.5) < 1e-6 and abs(c[4] - 0.3) < 1e-6 for c in cmds)
    await ws.send_str(json.dumps({"type": "cmd", "name": "turn", "angle": 90})); await asyncio.sleep(0.15)
    cmds = [struct.unpack(L.CMD_FMT, p) for t, p in L.Parser().feed(bytes(esp.tx)) if t == L.MSG_CMD]
    assert cmds[-1][0] == L.MODE_TURN_BY and abs(cmds[-1][5] - math.pi / 2) < 1e-6
    await ws.close()


# ---------------- AI camera: tracking, following, face matching ----------------
def test_follow_cmd_centres_and_keeps_distance():
    from robot.vision import follow_cmd
    v, w = follow_cmd((0.6, 0.3, 0.8, 0.6))          # small box to the right
    assert v > 0 and w < 0                           # drive closer, turn right
    v, w = follow_cmd((0.3, 0.0, 0.7, 0.95))         # big, centred: too close
    assert v < 0 and abs(w) < 0.05
    assert follow_cmd(None) == (0.0, 0.0)


def test_tracker_stays_on_named_person_and_survives_lost_face():
    from robot.vision import Tracker, Det
    import numpy as np
    red, blue = np.zeros(64), np.zeros(64); red[3] = 1; blue[40] = 1
    t = Tracker(name="anna")
    d1 = [Det("person", .9, (.1, .2, .3, .9), name="ben", hist=blue), Det("person", .9, (.6, .2, .8, .9), name="anna", hist=red)]
    assert t.update(d1, 0.0).name == "anna"
    # next frames: no faces recognised (anna turned away), boxes moved a little
    d2 = [Det("person", .9, (.12, .2, .32, .9), hist=blue), Det("person", .9, (.58, .2, .78, .9), hist=red)]
    got = t.update(d2, 0.1)
    assert got is not None and got.box[0] > 0.5
    # ben walks into anna's old spot while anna steps right: colour keeps the lock on anna
    d3 = [Det("person", .9, (.5, .2, .7, .9), name="ben", hist=blue), Det("person", .9, (.62, .2, .82, .9), hist=red)]
    assert t.update(d3, 0.2).box[0] == 0.62
    # lost for too long -> unlock, then only re-lock on anna
    assert t.update([Det("person", .9, (.1, .1, .3, .9), name="ben")], 2.0) is None and not t.locked


def test_facebook_match_threshold():
    from robot.vision import FaceBook
    import numpy as np
    rng = np.random.default_rng(1)
    a, b = rng.normal(size=128), rng.normal(size=128)
    fb = FaceBook(); fb.add("anna", a); fb.add("ben", b)
    assert fb.match(a + 0.2 * rng.normal(size=128))[0] == "anna"
    assert fb.match(rng.normal(size=128))[0] is None       # a stranger


def test_colour_hist_and_iou():
    from robot.vision import colour_hist, hist_sim, iou
    import numpy as np
    img = np.zeros((100, 100, 3), np.uint8); img[:, :50] = (200, 20, 20); img[:, 50:] = (20, 20, 200)
    hl, hr = colour_hist(img, (0, 0, .5, 1)), colour_hist(img, (.5, 0, 1, 1))
    assert hist_sim(hl, hl) > 0.99 and hist_sim(hl, hr) < 0.2
    assert iou((0, 0, 1, 1), (0, 0, 1, 1)) == 1.0 and iou((0, 0, .5, .5), (.5, .5, 1, 1)) == 0.0


def test_headlight_sysfs(tmp_path):
    from robot.headlight import Headlight, PERIOD_NS, MAX_LEVEL
    (tmp_path / "pwm0").mkdir()
    for f in ("period", "duty_cycle", "enable"): (tmp_path / "pwm0" / f).write_text("0")
    h = Headlight(str(tmp_path)); assert h.ok
    h.set(1.0); assert int((tmp_path / "pwm0" / "duty_cycle").read_text()) == round(PERIOD_NS * MAX_LEVEL)
    h.set(0); assert (tmp_path / "pwm0" / "duty_cycle").read_text() == "0"
