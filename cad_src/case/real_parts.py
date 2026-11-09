"""Bought parts as real 3D models, placed in the robot frame (x forward, y left, z up, origin on the axle).

Every part says where its geometry comes from:
  STEP   - manufacturer / published model, downloaded (models/bought/)
  DRAW   - built here from the official mechanical drawing (models/drawings/)
  LIST   - outer envelope from the seller's listed dimensions (no model published): a box, labelled as such
"""
import math, cadquery as cq
import body_case as bc
from load_models import load

AX = {"x": 0, "y": 1, "z": 2}

def orient(shape, xr, yr, zr):
    """Rigid rotation: robot x = model axis xr (e.g. '-z'), etc. Must be a proper rotation."""
    rows = []
    for a in (xr, yr, zr):
        r = [0.0, 0.0, 0.0]; r[AX[a[-1]]] = -1.0 if a[0] == "-" else 1.0; rows.append(r)
    det = (rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1]) - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
           + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
    assert abs(det - 1) < 1e-9, (xr, yr, zr)
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    t = gp_Trsf()
    t.SetValues(rows[0][0], rows[0][1], rows[0][2], 0, rows[1][0], rows[1][1], rows[1][2], 0, rows[2][0], rows[2][1], rows[2][2], 0)
    out = cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())
    return out, (lambda p: tuple(sum(rows[i][j] * p[j] for j in range(3)) for i in range(3)))

PLACE = {}   # name -> (axes, ref_model, target): to map model-frame pin positions to the robot frame
def to_robot(name, p):
    axes, ref, tgt = PLACE[name]
    _, R = orient(cq.Solid.makeBox(1, 1, 1), *axes)
    a, r = R(p), R(ref)
    return tuple(tgt[i] + a[i] - r[i] for i in range(3))

def place(shape, axes, ref_model, target, name=None):
    if name: PLACE[name] = (axes, ref_model, target)
    s, R = orient(shape, *axes)
    r = R(ref_model)
    return s.translate((target[0] - r[0], target[1] - r[1], target[2] - r[2]))

def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))

def cyl(p, d, h, axis):   # cylinder from point p along +axis
    return cq.Solid.makeCylinder(d / 2, h, cq.Vector(*p), cq.Vector(*axis))

# ---------------- built from official drawings ----------------
def pi_zero_2w(header=True):
    """Raspberry Pi Zero 2 W, from the official mechanical drawing (RP-008358): 65 x 30 PCB, holes 3.5 in, 58 x 23,
    2x20 GPIO header centred 32.5 from the left / 3.5 from the top edge, mini-HDMI at 12.4, micro-USB at 41.4 and 54
    on the bottom edge, CSI on the right end. Board frame: x along 65, y along 30 (0 = bottom edge), z up (PCB top = 1.0).
    Connector heights from their datasheets (mini-HDMI 3.2, micro-USB B 2.6, CSI 1.2 mm, 22-pin FPC)."""
    pcb = cq.Workplane().add(box(0, 65, 0, 30, 0, 1.0)).edges("|Z").fillet(3.0).val()
    for (hx, hy) in ((3.5, 3.5), (61.5, 3.5), (61.5, 26.5), (3.5, 26.5)):
        pcb = pcb.cut(cyl((hx, hy, -1), 2.75, 3, (0, 0, 1)))
    parts = [pcb,
             box(12.4 - 5.6, 12.4 + 5.6, -0.5, 7.0, 1.0, 4.2),          # mini-HDMI
             box(41.4 - 4.0, 41.4 + 4.0, -1.0, 4.6, 1.0, 3.6),          # micro-USB (OTG data)
             box(54.0 - 4.0, 54.0 + 4.0, -1.0, 4.6, 1.0, 3.6),          # micro-USB (power)
             box(61.0, 65.5, 7.5, 22.5, 1.0, 2.2),                      # CSI 22-pin FPC connector
             box(-1.5, 11.0, 9.5, 21.0, -1.3, 0.0),                     # microSD socket + card (underside)
             box(20.0, 34.0, 8.0, 22.0, 1.0, 7.0)]                      # RP3A0 SiP + small heatsink (14 x 14 x 5)
    if header:   # soldered 2x20 male header: 2.5 mm plastic, pins 6 mm above
        hx0 = 32.5 - 19 * 2.54 / 2 - 1.27
        parts.append(box(hx0, hx0 + 20 * 2.54, 30 - 3.5 - 2.54 / 2 - 1.27, 30 - 3.5 + 2.54 / 2 + 1.27 - 2.54, 1.0, 3.5))
        parts.append(box(hx0 + 0.6, hx0 + 20 * 2.54 - 0.6, 30 - 3.5 - 2.54 - 0.6 + 1.27, 30 - 3.5 + 0.6 + 1.27 - 1.27, 3.5, 9.5))
    return cq.Compound.makeCompound(parts)

def pi_gpio(pin):
    """Board-frame (x, y) of a GPIO header pin (1..40). Pin 1 is at the left end, inner row (towards the board centre)."""
    col = (pin - 1) // 2; inner = (pin % 2 == 1)
    x = 32.5 - 19 * 2.54 / 2 + col * 2.54
    y = 30 - 3.5 - (2.54 / 2 if inner else -2.54 / 2)
    return x, y

def esp32_s3_devkitc1():
    """ESP32-S3-DevKitC-1 v1.1 from Espressif's dimension drawing (DXF/PDF 2022-04-29): 62.74 x 25.4 PCB, 2 x 22
    pins 1.27 in from the long edges at 2.54 pitch, the ESP32-S3-WROOM-1 (Espressif STEP) at the top end, two
    USB-C at the bottom end (UART at 6.0, USB at 19.4 from the left edge). Board frame: x across (25.4), y along
    (0 = USB end), z up. Male headers pointing down (into female headers on the perfboard)."""
    pcb = box(0, 25.4, 0, 62.74, 0, 1.6)
    mod = load("espressif_esp32s3_wroom1.step")            # 18 x 25.5, antenna end at its y = 25.5
    mod = mod.translate(((25.4 - 18.0) / 2, 62.74 - 25.5, 1.6))
    parts = [pcb, mod]
    for xc in (6.0, 19.4):                                  # USB-C receptacles (8.94 x 7.35 x 3.26), 0.6 past the edge
        parts.append(box(xc - 4.47, xc + 4.47, -0.6, 6.75, 1.6, 4.86))
    for xs in (1.27, 25.4 - 1.27):                          # 22-pin male headers, plastic 2.5 under the PCB, pins 6 below
        parts.append(box(xs - 1.27, xs + 1.27, 62.74 - 2.54 * 22 - 1.0, 62.74 - 1.0, -2.5, 0))
    for (bx, by) in ((6.0, 12.5), (19.4, 12.5)):            # BOOT / RESET buttons
        parts.append(box(bx - 2.0, bx + 2.0, by - 1.7, by + 1.7, 1.6, 3.6))
    return cq.Compound.makeCompound(parts)

def esp_pin(side, n):
    """Board-frame (x, y) of header pin n (1 = top, next to the module) on side 'L' (x 1.27) or 'R' (x 24.13)."""
    return (1.27 if side == "L" else 25.4 - 1.27), 62.74 - 2.54 * (n - 1) - 2.54

SRC = {}
def real_parts():
    """dict name -> (shape, source, mass_g)"""
    P = {}
    # camera: lens 6.15 mm off the hole-pattern centre (model x); PCB lens face rests on 4 front-panel posts
    cam = load("rpi_camera_module_3_wide.step")
    P["Camera Module 3 Wide"] = (place(cam, ("-z", "-y", "-x"), (8.25, 12.5, -0.7),
                                       (bc.X_IN_F - bc.STANDOFF_F, 0.0, bc.CAM_HOLE_Z), name="Camera Module 3 Wide"), "STEP Raspberry Pi RP-008154 (Camera Module 3, proxy for the AI Camera: same PCB + holes; AI Camera 0.4 mm deeper)", 6)
    # sonar: transducers forward, header pins down
    so = load("hc_sr04_freecadlib.step")
    P["HC-SR04 sonar"] = (place(so, ("z", "x", "y"), (0, 0, bc.SONAR_PCB_TOP), (bc.X_IN_F - bc.SONAR_POST, 0, bc.SONAR_Z), name="HC-SR04 sonar"),
                          "STEP FreeCAD-library (community, 'original size')", 9)
    # microphones: port (under the mic package) on the panel hole; board bottom in the pocket floor
    mic = load("adafruit_6049_ics43434.step")
    for s, nm in ((1, "L"), (-1, "R")):
        axes = ("-z", "y", "x")            # both the same way up: pins at the bottom
        P[f"ICS-43434 mic {nm}"] = (place(mic, axes, (8.255, 7.11, 0), (bc.X_IN_F + bc.MIC_POCKET, s * bc.MIC_YZ[0], bc.MIC_YZ[1]), name=f"ICS-43434 mic {nm}"),
                                    "STEP Adafruit 6049", 1)
    # speaker: flange against the right wall, magnet inward (+y)
    spk = load("adafruit_3968_speaker40.step")
    P["Speaker 40 mm (Adafruit 3968)"] = (place(spk, ("x", "z", "-y"), (0, 0, 0), (bc.SPK_XZ[0], bc.SPK_Y0, bc.SPK_XZ[1]), name="Speaker 40 mm (Adafruit 3968)"),
                                          "STEP Adafruit 3968", 27)
    # amp on the ring tab, components inward (+y), pins at the top
    amp = load("adafruit_3006_max98357.step")
    ytab = bc.SPK_Y0 + bc.SPK_FLANGE + bc.RING_T
    P["MAX98357A amp (Adafruit 3006)"] = (place(amp, ("x", "z", "-y"), (8.89, 9.525, 0), ((bc.AMP_TAB[0] + bc.AMP_TAB[1]) / 2, ytab, (bc.AMP_TAB[2] + bc.AMP_TAB[3]) / 2), name="MAX98357A amp (Adafruit 3006)"),
                                          "STEP Adafruit 3006 (terminal block not in the model)", 2)
    # regulator on its tray posts (hole pattern taken from the real board)
    reg = load("pololu_d24v50f5.step")
    ztr = bc.Z_FLOOR + bc.FLOOR_BOSS_H + bc.TRAY_T + 4.0
    P["Pololu D24V50F5 regulator"] = (place(reg, ("y", "-x", "z"), (8.9, 10.2, 0), (16.5, 0, ztr), name="Pololu D24V50F5 regulator"), "STEP Pololu 0J1437", 3)
    # lidar: ears on the 3 top inserts
    ld = load("ldrobot_ld19_waveshare.step")
    P["LD19 lidar"] = (place(ld, ("-y", "x", "z"), (0, -8.62, 0), (bc.LIDAR_PAIR_X, 0, bc.Z_TOP), name="LD19 lidar"), "STEP Waveshare (LDROBOT LD19)", 47)
    # perfboard on the back cover + ESP32 devkit (female headers) + CAN board
    xb = bc.X_BACK + bc.BOARD_STANDOFF                          # perfboard back face
    cy, cz = bc.PB_CENTER
    pb = box(xb, xb + 1.6, cy - 35, cy + 35, cz - 25, cz + 25)
    esp = esp32_s3_devkitc1()
    # devkit: board x (25.4) -> robot z, board y (62.74) -> robot -y (USB end towards the right wall), components towards +x
    P["ESP32-S3-DevKitC-1"] = (place(esp, ("z", "-y", "x"), (12.7, 31.37, 0.0), (xb + 1.6 + 8.5 + 2.5, cy - bc.ESP_DY, bc.ESP_Z), name="ESP32-S3-DevKitC-1"),
                               "DRAW Espressif DXF v1.1 + WROOM-1 STEP", 10)
    # female headers exactly under the DevKit's two pin rows (positions from the placed DevKit, see perfboard_layout.py)
    pins = [to_robot("ESP32-S3-DevKitC-1", (*esp_pin(sd, n), 0)) for sd in ("L", "R") for n in (1, 22)]
    ys = [p[1] for p in pins]; zl, zr = pins[0][2], pins[2][2]
    fh = cq.Compound.makeCompound([box(xb + 1.6, xb + 1.6 + 8.5, min(ys) - 1.27, max(ys) + 1.27, zz - 1.27, zz + 1.27) for zz in (zl, zr)])
    P["Perfboard 5 x 7 cm + female headers"] = (cq.Compound.makeCompound([pb, fh]), "DRAW (standard 2.54 perfboard, 8.5 mm female headers)", 20)
    can = load("waveshare_sn65hvd230_can_board.step")   # PCB in its XZ plane, parts on +y
    P["SN65HVD230 CAN board"] = (place(can, ("y", "x", "-z"), (15.0, 0.0, -8.0), (xb + 1.6 + 2.5, cy + bc.CAN_DY, bc.CAN_Z), name="SN65HVD230 CAN board"), "STEP Waveshare (on 2.5 mm spacers: solder tails)", 4)
    # Pi on the back cover standoffs, components towards +x, GPIO header along the top edge
    pi = pi_zero_2w()
    P["Raspberry Pi Zero 2 W"] = (place(pi, ("z", "x", "y"), (32.5, 15.0, 0.0), (bc.X_BACK + bc.BOARD_STANDOFF, bc.PI_CENTER[0], bc.PI_CENTER[1]), name="Raspberry Pi Zero 2 W"),
                                  "DRAW Raspberry Pi RP-008358 (heights from connector datasheets)", 11)
    return P
