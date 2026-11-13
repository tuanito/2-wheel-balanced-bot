import math
"""Parametric printed body v2 for the two-wheel balancing robot (CadQuery).
Wheels UNDER the body: 2/3 of each wheel's width is inside the body outline, 1/3 sticks out.

Robot frame: X forward, Y left (wheel axle), Z up, origin on the axle. Ground at z = -WHEEL_R.

Layout (bottom to top):
  - two MF6015 motors back-to-back on a double centre wall, axle at z = 0
  - centre gap between the walls: power distribution, regulator, fuse, IMU (right at the axle)
  - wheel arches over the wheels
  - battery shelf above the motors, battery strapped on it
  - upper bay: front panel (LED eyes, camera, sonar), back cover (perfboard + Pi Zero)
  - lidar on top

Parts:
  body         - one print, open front and back. Print standing on its front opening.
  front        - removable front panel, 6 x M3 into heat-set inserts. Print outside face down.
  cover        - removable back cover, 6 x M3 into heat-set inserts, carries perfboard + Pi.
  eye_diffuser - print 2 in white/natural PETG.

Run: python3 body_case.py
"""
import cadquery as cq
import math, os, pathlib

import motors as _motors

# ------------------------------------------------------------------ wheels / motors
# Which hub motor this body is cut for. Every motor-derived dimension below comes from motors.py,
# so a swap is one name, not a hunt through four files. ROBOT_MOTOR overrides it the same way
# ROBOT_CASE and friends override the paths.
MOTOR       = os.environ.get("ROBOT_MOTOR", "mf6015")   # mf6015 | mf7015
_M          = _motors.get(MOTOR)

WHEEL_R     = 54.3           # Fastrax SC wheel, 108.6 mm
WHEEL_W     = 45.0
WELL_R      = WHEEL_R + 4.2  # wheel arch clearance
ARCH_T      = 3.0
MOTOR_LEN   = _M["length"]   # back face -> rotor face (mf6015 34.0, mf7015 33.9)
WHEEL_GAP   = 3.0            # rotor face -> wheel inner face (set NECK_L in wheel_adapter.py to match)
GAP_HALF    = 17.0           # half width of the centre gap between the two motor walls
MWALL_T     = 7.0            # motor wall thickness
# The wall's register for the motor. The MF6015 has a Ø49 x 8 rear boss and this is the counterbore
# it seats in; the MF7015 has NO boss, so the same counterbore goes round its Ø69 outside diameter
# instead and does the identical job.
POCKET_D    = _M["pocket_d"]
POCKET_DEP  = _M["pocket_dep"]
MOTOR_BACK_Y = GAP_HALF + MWALL_T - POCKET_DEP          # 22: motor back face
WHEEL_IN_Y  = MOTOR_BACK_Y + MOTOR_LEN + WHEEL_GAP      # 59
WHEEL_OUT_Y = WHEEL_IN_Y + WHEEL_W                      # 104
Y_OUT       = WHEEL_OUT_Y - WHEEL_W / 3                 # 89: 1/3 of the wheel sticks out
WELL_Y0     = WHEEL_IN_Y - 2.0                          # arch starts here

M25_CLEAR   = _M["bolt_clear"]   # motor screw clearance: M2.5 on the mf6015, M3 on the mf7015
MOTOR_SQ    = _M["bolt_sq"]     # 20 x 20 on the mf6015, 30 x 30 on the mf7015
SLOT_W      = _M["slot_w"]   # over the motor's connector bay (top of the Ø49 boss, 15 mm wide): plugs + leads + fingers
SLOT_Z      = (17.0, 43.0)   # top raised with the shelf: the motor leads arc through its upper part; closed window (a slot open to the top printed as a 20 mm cantilever); plugs + leads pass through it; open to the top: plug in from above before the shelf; the screwed shelf closes it

# ------------------------------------------------------------------ overall box
WALL       = 3.0
SIDE       = 3.0
# The body's depth and ride height are set by the motor's RADIUS and nothing else. This used to be
# three hand-set literals (35 / -32 / -35) whose comments said "3 mm air ... (r 29)", which is why a
# fatter motor interfered on three sides at once with no symbol to change.
MOTOR_R    = _M["od"] / 2.0  # mf6015 29.0, mf7015 34.5
MOTOR_AIR  = 3.0             # air around the can: in front, behind, and under
X_FRONT    = MOTOR_R + MOTOR_AIR + WALL    # mf6015 35.0
X_BACK     = -(MOTOR_R + MOTOR_AIR)        # mf6015 -32.0
Z_BOT      = -(MOTOR_R + MOTOR_AIR + WALL) # mf6015 -35.0
# Top of the motor walls = underside of the shelf. It is set by how much room the motor's LEADS need
# above the can: 18 mm on the MF6015, so they rise and bend back down into the wall window gently
# (R ~8 mm; it was a 90 deg kink in 8 mm before the shelf went up 10 mm). Tracking MOTOR_R keeps that
# clearance when the can grows. NOTE: 18 mm was measured against the MF6015's AXIAL plugs on top of
# its rear boss. The MF7015's connector is a socket in the side rim, so the mating plug's height is
# a different number and has to be checked against the real part.
SHELF_CLEAR = 18.0
SHELF_Z    = MOTOR_R + SHELF_CLEAR         # mf6015 47.0, mf7015 52.5
SHELF_T     = 5.0
BATT_H      = 42.0           # 6S 1400 pack height
UPPER_BAY   = 58.0           # shelf + battery -> lid: the front panel / back cover bay
Z_TOP      = SHELF_Z + SHELF_T + BATT_H + UPPER_BAY   # mf6015 152.0 (134 -> 142 for the 6S pack -> 152 when the shelf rose 10 mm)
TOP_T      = 4.0
COVER_T    = 3.0
FILLET     = 4.0
X_IN_F  = X_FRONT - WALL     # 28
Y_IN    = Y_OUT - SIDE       # 86
Z_FLOOR = Z_BOT + WALL       # -32
Z_IN_TOP = Z_TOP - TOP_T     # 130

# ------------------------------------------------------------------ battery shelf
# SHELF_Z / SHELF_T / BATT_H are up in the overall-box block: the shelf height is motor-derived and
# Z_TOP is measured from it.
BATT_TOP    = SHELF_Z + SHELF_T + BATT_H   # 94: the back-cover boards are placed from here
RAIL_W      = (3.0, 6.0)     # dovetail rail on each motor wall: width at base, width at top
RAIL_H      = 2.5
RAIL_CLR    = 0.3            # clearance in the shelf's groove
BATT_X      = (-27.5, 12.5)  # BMS on its back face; 5 mm in front for the sonar pins
STRAP_XS    = (-29.6, 14.5)  # the strap wraps battery + BMS
STRAP_W     = 22.0
STRAP_SLOT  = 3.0
SHELF_SCREW_X = (-27.0, 21.0)  # 2 M3 countersunk screws per rail: shelf -> heat-set insert in the wall top
SHELF_BOSS_D = 9.0              # local wall thickening around those inserts (z 20..33)
ROOT_CHAMFER = (5.0, 2.5)       # 45-deg fillet at the wall/floor joint: centre-gap side, motor side
CABLE_HOLE  = (19.0, 26.0, -12.0, 12.0)   # x0, x1, y0, y1 in the shelf, motor/power cables go up here

# ------------------------------------------------------------------ front panel
CAM_Z       = 112.0
CAM_HOLES   = (21.0, 12.5)
CAM_HOLE_Z  = 112.0 + 6.15   # real CM3: lens centre 6.15 below the hole-pattern centre (FPC connector down)
CAM_WIN     = (15.0, 15.0)     # AI Camera: lens 3 mm behind the face, view cone 66 x 52 deg through the 3 mm wall
CAM_PC_WIN  = (19.0, 19.0, 1.5)   # pocket on the outer face for a 1 mm polycarbonate window (sits 0.5 mm below the face)
STANDOFF_F  = 8.5           # Raspberry Pi AI Camera (same PCB + holes as CM3, 11.9 deep): lens front ~8.5 above the PCB -> ~3 mm behind the front face
SONAR_Z     = 80.0
# VL53L5CX zone array: the alternative to the sonar, high on the panel and looking down.
# The height is the whole point. A ray reaches the first tread only when it clears the step nose and
# lands before the next one, which needs D < h * tread / rise - so at the sonar's 134 mm a 170 mm
# riser is unreachable beyond 237 mm, and at 194 mm it is reachable to 342 mm. Measured sweep in
# sim/tof_sensor.py: 30 deg down answers best at a 250 mm standoff and still sees floor 1.47 m ahead.
TOF_Z       = 140.0          # panel z -> 194 mm above the ground with the 54.3 mm wheels
TOF_TILT    = 30.0           # degrees below horizontal
TOF_HOLES   = 18.0           # M2 mounting holes, y spacing: CHECK against your carrier (Pololu #3417)
TOF_SEAT    = 3.0            # mm the seat stands behind the inner face, along the tilted normal. Kept
                             # short on purpose: the window has to flare at 30 deg per side for the
                             # receiver, so every mm of standoff widens the bore and eats the bosses.
TOF_WIN     = (8.0, 17.0)    # window cone: diameter at the seat -> at the outer face. The Rx needs
                             # 61 deg vertical clear, so the bore has to flare, not run straight.
TOF_PAD     = (22.0, 16.0)   # seat pad y, z: the carrier is 13 x 18 mm
SONAR_HOLES = (41.0, 17.0)   # HC-SR04 holes per the FreeCAD-library model (41 x 17): verify on your board
SONAR_PCB_TOP = 2.0          # model: PCB top face (transducers 11.2 above it)
SONAR_POST  = 9.0            # PCB sits 9 mm behind the panel -> transducers flush with the front face
SONAR_PITCH = 27.0           # transducer centres per the model
SONAR_D     = 17.5
PILOT_M2    = 1.7
PILOT_M25   = 2.2
EYE_Y       = 38.0          # was 32
EYE_Z       = 104.0          # 8 mm below the camera (112) and EYE_Y 6 mm further out: less eye light near the lens
EYE_WIN     = 24.0
EYE_WIN_R   = 4.0
EYE_PCB     = 26.4
EYE_POCKET  = 2.2
DIFFUSER_T  = 0.8

# ------------------------------------------------------------------ top / lidar (LD19 datasheet V2.5)
LIDAR_PAIR_Y = 46.8 / 2
LIDAR_PAIR_X = 8.8
LIDAR_SINGLE_X = LIDAR_PAIR_X - 31.92
LIDAR_CABLE = (-10.0, -34.0, 10.0)
BUTTON      = (10.0, 36.0, 12.2)       # arm button (the power button is its mirror image)
LED         = (-6.0, 36.0, 5.2)
# IR beacon receivers (TSOP38238, 38 kHz) in 4 printed hoods on the top corners, each looking out diagonally (~90°
# each: together all round, clear of the lidar housing in the middle). x, y, facing (deg, 0 = forward, + = left)
IR_RX_XY    = (15.0, 54.0)             # hood centres at (±x, ±y): fully on the flat top (x -32..25, |y| <= 64), clear of the buttons
IR_RX       = [(sx * IR_RX_XY[0], sy * IR_RX_XY[1], math.degrees(math.atan2(sy, sx))) for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
IR_LEAD_HOLE = 4.5           # Ø of the hole in the top skin under each hood (3 leads)
IR_HOOD     = (11.0, 10.0, 9.0)    # hood footprint x, y and height (top at Z_TOP + 9: well below the lidar beam, 159)
IR_HOOD_FILLET = (2.5, 1.5)   # vertical edges, top edges
IR_TSOP_POCKET = (-3.2, 2.4, 3.2, 1.2, 8.2)   # receiver pocket: x from, x to, half width, z from, z to (TSOP38238
                              # 5.6 x 6 x 7 + clearance)
IR_WINDOW   = (2.0, 5.5, 1.2) # 90° window: z start, height, apex x (the dome sits just behind it)
IR_HOOD_LEADS = (-0.5, 2.0, 1.5)   # lead hole through the hood floor: x, Ø/2, depth
# power: momentary push button on top, mirrored with the arm button, wired to a Pololu Big Pushbutton Power Switch HP
# (#2813, MOSFET latch: battery current never goes through the button). The board sits on the back cover's inside,
# above the Pi (VHB tape); pads face the front.
POWER_BTN   = (BUTTON[0], -BUTTON[1], BUTTON[2])       # x, y, hole d (12 mm momentary, NO)
PWR_SW      = (X_BACK + 3.0, 28.0, BATT_TOP + 43.0, 5.0, 25.5, 20.0)   # switch board on the back cover's inside, above the Pi
                                                        # (VHB tape): centre x, y, z; size x (thickness), y, z. Pads face the front


def pwr_sw_box():
    """(x0, x1, y0, y1, z0, z1) of the power switch board (Pololu 2813) on the back cover, above the Pi."""
    cx, cy, cz, tx, wy, hz = PWR_SW
    return (cx - tx / 2, cx + tx / 2, cy - wy / 2, cy + wy / 2, cz - hz / 2, cz + hz / 2)

# ------------------------------------------------------------------ side walls
USB_SLOT    = (-20.0, 120.0, 12.0, 7.0) # right side (-Y): ESP32 USB-C (moved up/back for the speaker)
# speaker (Adafruit 3968: 40 mm, 4 ohm, 3-5 W, 20 mm deep) on the RIGHT side wall above the wheel arch, firing sideways
SPK_XZ      = (-2.0, 86.0)
SPK_D       = 40.0
SPK_FLANGE  = 3.0            # Adafruit 3968 STEP: 3 mm flange (Ø40), then a Ø34 step
SPK_MAG_D   = 31.0           # ring bore = SPK_MAG_D + 4 = 35 (clears the Ø34 step)
SPK_GRILLE_R = 15.0
SPK_GRILLE_PITCH = (3.9, 4.5)  # hole grid pitch (x, z); every other column shifted half a pitch in z
SPK_GRILLE_HOLE = 2.6          # teardrop hole Ø
SPK_CLEAR   = 4.0              # flange clearance Ø over the speaker Ø
SPK_BOSSES  = [(-2.0, 110.0), (-19.0, 69.0), (15.0, 69.0)]   # x, z: 3 x M2 inserts for the speaker ring (r = 24 mm)
SPK_Y0      = -(Y_IN - 1.0)           # flange front face (1 mm gasket gap to the wall at the bottom)
RING_T      = 2.5
AMP_TAB     = (-12.0, 8.0, 103.5, 122.5)  # 2 mm lower: the real amp touched the inner top-corner curve  # x0, x1, z0, z1: tab on the ring that carries the MAX98357A
# microphones: 2 x ICS-43434 I2S breakout (bottom port) in the front panel, 124 mm apart
MIC_YZ      = (62.0, 100.0)
MIC_PCB     = (14.0, 18.0)          # pocket (y, z): CHECK against your board
MIC_POCKET  = 1.6
MIC_PORT_D  = 1.5
MIC_PAD_DY  = 4.61           # Adafruit 6049: solder pads 4.61 mm from the port, along z
MIC_CHIP_DY = 0.76           # Adafruit 6049: the mic package is 0.76 mm off the board centre -> pocket offset
MIC_BOSS_DZ = 13.0
VENTS_Z     = (98.0, 116.0)
VENTS_X     = (14.0, 19.0, 24.0)   # left-wall vent slots: x of each slot
VENT_W      = 2.5                  # slot width
WALL_CUT_DEPTH = 7.0         # side cuts reach 7 mm in, so they still pierce a rounded / tapered wall
# mounting points (M3 inserts) for modules whose hole patterns vary by seller -> universal carrier plates
WALL_PLATE_PTS = [(-8.0, 92.0), (14.0, 92.0)]    # x, z on the LEFT side wall (pitch 22 mm); the right wall carries the speaker
WALL_BOSS_L = 6.0
FLOOR_TRAY_PTS = [(-24.0, -13.0), (21.0, 13.0)]  # x, y: electronics tray in the centre gap
FLOOR_BOSS_H = 6.0
TRAY_T      = 3.0
SHELF_FRONT_NOTCH = (14.5, 26.0)   # shelf front corners cut away (x >, |y| >): the headlight driver board and collars sit there
SHELF_PLATE_PTS = [(-22.0, 50.0), (8.0, 50.0)]   # 6 mm back: clear of the headlight driver board  # x, y on top of the shelf (lidar USB adapter)
EYE_RET_PTS = [(16.0, -16.0), (16.5, 11.0)]      # eye retainer bosses, offsets (dy toward the outside, dz) per eye
# COB disc headlights (Bits & Parts Ø30 x 4 mm, 12 V 2.4 W): beside the sonar, below the eyes and mic ports
LIGHT_Y, LIGHT_Z = 42.0, 68.0   # lowered 8 mm with the eyes (was 76)
LIGHT_D, LIGHT_T = 30.0, 4.0
LIGHT_BORE = 30.6            # collar bore (disc + 0.3 mm a side)
LIGHT_COLLAR = 34.0          # collar OD on the panel's inside
LIGHT_WIN = (24.0, 30.0)     # window through the 3 mm wall: Ø24 inside -> Ø30 outside (45°: prints face-down without support)
LIGHT_RET_PTS = [(17.0, 10.0), (-13.0, -14.0)]   # retainer bosses, offsets (dy toward the outside, dz): clear of sonar posts + eye retainer
BUMPER_D    = 11.0           # stick-on rubber bumpers (3M Bumpon type, Ø11 x 2.5)
BUMPER_Y    = WELL_Y0 - BUMPER_D / 2 - 3.0                 # inside the narrow lower part of the panel (clear of the wheel wells)
FRONT_BUMPERS = [(BUMPER_Y, Z_TOP - 15.0),                 # mirrored in y: 4 bumpers in a rectangle, the face lands on these
                 (BUMPER_Y, LIGHT_Z - LIGHT_COLLAR / 2 - BUMPER_D / 2 - 6.0)]   # (was y 66: the lower pair sat in the wheel wells)
DRV_CENTER = (43.0, 54.0)    # headlight driver board (2.54 perfboard 10 x 8 holes = 25.4 y x 20.3 z), behind the lowered headlight, left of the sonar (the right side has the BAT+ run)
DRV_SIZE = (25.4, 20.3)
DRV_STANDOFF = 8.5           # board front face 8.5 mm behind the panel's inner face (clear of the headlight retainers)
DRV_PTS = [(-9.5, -7.0), (9.5, -7.0)]   # 2 x M2 bosses at the two bottom corners (offsets dy, dz from DRV_CENTER): the top ones would hit the lowered headlight

# ------------------------------------------------------------------ inserts, cover, boards
# brass heat-set inserts (ruthex): (hole diameter, hole depth, boss diameter)
INS_M2      = (3.2, 5.0, 6.4)
INS_M25     = (4.0, 6.7, 8.0)   # no longer used: Pi and IMU take M2 (two insert sizes only: M3 + M2)
INS_M3      = (4.0, 6.7, 8.0)
INSERT_D    = INS_M3[0]
BOSS_D      = INS_M3[2]
BOSS_L      = 8.0
PANEL_SCREWS = [(Y_IN - 6.0, 75.0), (76.6, Z_TOP - 22.4), (40.0, Z_FLOOR + 5.5)]   # mirrored in Y; each 2 mm further from the panel edge (was 82/75, 78/121, 40/-28.5)
FLOOR_SCREW = PANEL_SCREWS[2]
HEAD_RECESS = 1.2
CSK_D       = 7.2           # countersink for M3 ISO 10642 (hex socket, head 6.72 x 1.86), 90 degrees
CSK_SINK    = 0.2           # head 0.2 mm below the surface: nothing sticks out when the robot falls
PAD_T       = 1.5           # panels are 1.5 mm thicker around their screws (inside)
PAD_D_FRONT = 6.0           # front pads sit in 1.5 mm pockets in the bosses (the bosses still start on the print bed)
SAC         = 0.2
PB_HOLES    = (64.0, 44.0)   # 5 x 7 cm perfboard, 70 along Y
PB_CENTER   = (-29.0, BATT_TOP + 26.0)   # perfboard centre (y, z) on the back cover, above the battery
ESP_Z, ESP_DY = PB_CENTER[1] + 10.16, -2.15  # ESP32-S3-DevKitC-1 on the perfboard (centre z, offset along -y): header pins on holes k1-22, rows j8/j17 of a 24 x 18 board
CAN_DY, CAN_Z = -16.0, BATT_TOP + 10.0    # SN65HVD230 board on the perfboard
PI_HOLES    = (58.0, 23.0)
PI_CENTER   = (44.0, BATT_TOP + 16.5)   # (y, z); 2 mm towards the centre: clears the moved top-back screw boss
BOARD_STANDOFF = 9.0


# ------------------------------------------------------------------ charging: contact plate (dock) + USB-C
CONTACT_PLATE = (-52.0, 52.0, 90.0, 132.0)   # y0, y1, z0, z1 on the back cover (outside)
CONTACT_T   = 10.0           # thick plate: deep lead-in funnels for the dock rods
CONTACT_SCREWS = [(47.0, 96.0), (47.0, 125.0)]   # mirrored in Y: 4 x M3 into cover inserts
CONTACT_BOSS_L = 4.0
PADS        = {"+": (-25.0, 111.0), "-": (25.0, 111.0)}   # y, z of the two contacts (funnel axes)
PAD_HEAD    = 9.3            # M5 brass countersunk screw head (DIN 963) = the contact (also the terminal: ring lug + nut
                             # inside). Ø9.3: the rods still make contact ±4.5 mm off centre (sim: the guidance lands within ~±3)
PAD_SCREW   = 5.4            # its clearance hole
FUNNEL      = (22.0, 8.0)    # lead-in funnel: entrance width at the plate face, depth to the screw-head seat (45°: catches ±7 mm)
FUNNEL_DROP = 8.0            # the entrance reaches this much further down (an obround 22 x 30)
CONTACT_PLATE_EDGE = (6.0, 1.0)   # plate: corner fillet (around x), chamfer on its outer face edges
PAD_SEAT_CLEAR = 0.3          # the funnel's bottom / the countersink is this much wider than the screw head (radius)
GUIDE_SLOT  = (20.0, 8.0, 7.0)   # vertical V slot at y 0 for the dock tongue: width at the face, at the bottom, depth
# No magnets: docked, the wheels sit in a shallow wave in the dock's floor strips (DOCK_WAVE). Leaning on the dock pushes
# the robot forward (a ladder on a slippery floor); the wave's front crest stops it, and the robot rolls over it
# easily when it drives away.
DOCK_FLOOR  = 1.0            # mm: the trough's top above the floor = the TPU insert's thinnest part
# Saddle under each docked wheel (option "A+"): rear lip, flat trough, front lip, long gentle lead-in. Heights above
# the trough, x relative to the docked axle (mm). The trough + front lip are a ribbed TPU insert (grip to climb out),
# the rest is the PETG frame it drops into.
SADDLE_REAR  = (1.2, -34.0, -22.0, -14.0)   # height, start, crest, end of the rear lip
SADDLE_FRONT = (1.8, 5.5, 19.5, 99.5)       # height, start, crest, end of the lead-in (80 mm, max 2°). 1.8 mm, not 2.5: a balancing
                             # robot climbs a lip by leaning about as steeply as the lip meets the tyre (2.5 mm: ~17°, up to 30° in the sim),
                             # and the round tyre meets a 2.5 mm lip so early that the robot stopped at a 6° lean (contacts barely pressed)
SADDLE_TILT  = 0.3           # the trough bottom falls 0.3 mm towards the front lip: the wheel always settles against it
                             # (sim: on a flat bottom it stopped wherever, some robots leaning only 5.6°, contacts barely touching)
SADDLE_LEAD  = 20.0          # then down from DOCK_FLOOR to the floor over this length
SADDLE_INSERT = (-16.0, 44.0)               # x range of the TPU insert (trough bottom + front lip)
SADDLE_RIB  = (0.4, 1.6)     # TPU grip ribs across the travel direction: height, pitch (under the tyre's squash)
DOCK_WAVE_Y = (55.0, 108.0)  # |y| of the wheel track (tyre 59-104)
DOCK_WAVE_X0 = 0.0
# vertical dock: the robot backs in, leans back onto it and switches its motors off
DOCK_LEAN   = 8.0            # deg: robot pitched back while docked (its weight holds the plate on the dock face)
DOCK_GAP    = 1.0            # plate face -> dock face when docked (the dock face's 1 mm pads)
DOCK_FACE   = (6.0, 88.0, 84.0, 140.0)       # face slab: thickness, half width, z0, z1 (robot frame); wings bolt to its sides
DOCK_TOWER  = (34.0, 40.0, 265.0)            # column behind the face: depth, half width, top z (robot frame): ~100 mm above the lidar plane, still seen from the approach point with the robot leaning 8°
DOCK_BASE   = (170.0, 176.0, 4.0, -62.0)     # floor plate: length, width, thickness, front edge x (clear of the wheels, also
                                             # when the robot rolls back ~35 mm under itself to lean off the dock face)
DOCK_TAPE   = [(-37.0, -12.0), (12.0, 37.0)]   # y ranges of the 2 retro-reflective strips on the column face (25 mm wide, 24 mm
                             # gap): the lidar sees them far brighter than anything else -> the dock's ID and its centre
DOCK_TAPE_Z = (104.0, 245.0) # robot-frame z (docked) of the strips: world ~150-290 mm, the lidar plane (213) ± a 8° lean at 0.7 m
DOCK_IR_Z   = 252.0          # robot-frame z (docked) of the IR beacon LEDs, in the mast (world ~300 mm)
DOCK_IR_TUBE = 22.0          # length of the centre-line LED's tube (narrows its beam to about ±6°)
DOCK_JOINT_SPIGOT = (2.0, 1.6, 8.0, 0.25)   # foot -> head joint: ring inset from the column's outside, reach in past the
                             # wall's inside, height, clearance in the head's groove
DOCK_JOINT_BOSS = (25.0, 4.0)  # 2 M3 insert columns inside the foot's back wall: |y|, radius (an M3x16 from the shelf top)
DOCK_BOSS_GUSSET = 14.0      # 45° wedge under each rod boss (it sticks back into the hollow column)
DOCK_SPLIT  = 200.0          # world z: the tower is printed up to here, the mast (upper column) plugs on with a spigot
DOCK_STEP   = 32.0           # the column's lower part (below the face slab) is set back this much: room for the robot's
                             # lower back when it undocks (sim/dock_sim.py: with a straight column it wedged between slab and column foot)
DOCK_ROD    = (4.0, 10.5, 25.0)              # brass rod Ø, free length out of the dock face, total length (sprung, ~4 mm travel;
                                             # soft spring ~0.15 N/mm, 0.2 N preload: docked ~0.4 N per contact)
DOCK_TONGUE = (18.0, 7.0, 6.5)               # wedge into the guide slot: width at the face, at the tip, height out of the face
DOCK_WING   = (93.0, 120.0, 40.0, 3.0, (82.0, 138.0))   # side wings: inner |y| at the face (body 87-89), at the mouth, length forward, thickness, z range
DOCK_WALL     = 2.4          # column / mast wall thickness (hollow inside: the wire channel)
DOCK_WALL_FRONT = 2.9        # front wall of the upper column (under the tape guides)
DOCK_SLAB_JOIN = 0.5         # the column reaches this far into the face slab (one solid)
DOCK_SHELF_H  = 12.0         # the shelf under the face slab where the column steps back
DOCK_FOOT_Z   = -120.0       # the lower column is built down to here, then trimmed at the floor plate
DOCK_FILLET   = (4.0, 3.0)   # rounded vertical edges: face slab, column
DOCK_PASS     = (8.0, 4.0, 6.0, 3.0, 9.0, 13.0)   # wire passage: half width, inset from the column walls, from the column
                             # front, z from / to above the shelf bottom, top where it opens into the upper column
DOCK_TAPE_GUIDE = (0.3, 0.1) # tape guides: depth into the face, overcut in front
DOCK_TONGUE_INSET = 4.0      # the tongue stops this short of the face slab's top and bottom
DOCK_IR_LED   = (5.2, 4.0, 20.0, 10.0)   # IR LED holes: Ø (TSAL6200 Ø5 + 0.2), depth into the wall, |y| (or x) offset from
                             # the column centre line for the front/back LEDs and the centre tube, centre tube boss Ø
POGO_D      = 4.3            # bore for the 4 mm brass rod
USBC_SIDE   = (8.0, 66.0)                    # left side wall (+Y), above the wheel: x, z of the panel-mount USB-C socket
                                              # (reachable upright, face-down and back-down)
USBC_CUT    = (10.5, 4.5)                    # SparkFun CAB-15455 / Parts Express style, M3 holes 18 mm apart
USBC_PITCH  = 18.0
USBC_SCREW  = 3.2            # its two screw holes (M3 self-tapping into the plastic flange)


def contact_screw_points():
    return [(s_ * y, z) for s_ in (1, -1) for (y, z) in CONTACT_SCREWS]


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_x(x0, length, y, z, d):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).center(y, z).circle(d / 2).extrude(length)


def cyl_y(y0, length, x, z, d):
    return cq.Workplane("XZ", origin=(0, y0, 0)).center(x, z).circle(d / 2).extrude(-length)


def teardrop_z(x, y, d, z0, length):
    """Hole along Z with a 45-degree point toward -X (= up when the body prints on its front)."""
    r = d / 2; k = r / math.sqrt(2)
    circ = cq.Workplane("XY", origin=(0, 0, z0)).center(x, y).circle(r).extrude(length)
    tip = (cq.Workplane("XY", origin=(0, 0, z0))
           .polyline([(x, y + k), (x - k, y + k), (x - r * math.sqrt(2), y), (x - k, y - k), (x, y - k)]).close()
           .extrude(length))
    return circ.union(tip)


def teardrop_boss_z(x, y, d, z0, length):
    """Solid boss along Z whose +X side is a 45-degree point: prints without support
    when the body stands on its front face (build direction -X)."""
    r = d / 2; k = r / math.sqrt(2)
    circ = cq.Workplane("XY", origin=(0, 0, z0)).center(x, y).circle(r).extrude(length)
    tip = (cq.Workplane("XY", origin=(0, 0, z0))
           .polyline([(x, y + k), (x + k, y + k), (x + r * math.sqrt(2), y), (x + k, y - k), (x, y - k)]).close()
           .extrude(length))
    return circ.union(tip)


def teardrop_y(x, z, d, y0, length):
    """Hole along +Y from y0 whose -X side is a 45-degree point (prints round when the body stands on its front)."""
    r = d / 2; k = r / math.sqrt(2)
    circ = cq.Workplane("XZ", origin=(0, y0, 0)).center(x, z).circle(r).extrude(-length)
    tip = (cq.Workplane("XZ", origin=(0, y0, 0))
           .polyline([(x, z + k), (x - k, z + k), (x - r * math.sqrt(2), z), (x - k, z - k), (x, z - k)]).close()
           .extrude(-length))
    return circ.union(tip)


def teardrop_boss_y(x, z, d, y0, length):
    """Solid boss along +Y from y0, +X side pointed (prints support-free on the front face)."""
    r = d / 2; k = r / math.sqrt(2)
    circ = cq.Workplane("XZ", origin=(0, y0, 0)).center(x, z).circle(r).extrude(-length)
    tip = (cq.Workplane("XZ", origin=(0, y0, 0))
           .polyline([(x, z + k), (x + k, z + k), (x + r * math.sqrt(2), z), (x + k, z - k), (x, z - k)]).close()
           .extrude(-length))
    return circ.union(tip)


def gusset_xy(x0, ya, yb, z0, z1):
    """45-degree support under a ledge that starts at x0 and spans ya..yb (yb = wall side)."""
    w = abs(yb - ya)
    pts = [(x0, ya), (x0, yb), (x0 + w, yb)]
    return cq.Workplane("XY", origin=(0, 0, z0)).polyline(pts).close().extrude(z1 - z0)


def gusset_xz(x0, za, zb, y0, y1):
    """45-degree support under a ledge that starts at x0 and spans za..zb (zb = floor side)."""
    h = abs(zb - za)
    pts = [(x0, za), (x0, zb), (x0 + h, zb)]
    return cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0)


def wells(x0, x1, r):
    """Both wheel-well cylinders (axis Y), from WELL_Y0 outward."""
    w = None
    for s in (1, -1):
        y0 = WELL_Y0 if s > 0 else -(Y_OUT + 5)
        c = cyl_y(y0, Y_OUT + 5 - WELL_Y0, 0, 0, 2 * r).intersect(box(x0, x1, -200, 200, -200, 200))
        w = c if w is None else w.union(c)
    return w


# ------------------------------------------------------------------ outer shape ("skin")
# SKIN = None -> the original rounded box. Otherwise a dict:
#   {"half_w": [(z, half_width), ...] bottom->top polygon, "r_top": .., "r_bot": .., "front_round": ..}
# Chosen design: proposal C "Bean" (Nov 2026): sides lean in above the eyes, soft corners, fender eyebrows.
# half_w follows Y_OUT: it is the SAME line as the wheel edge, and writing it as a literal 89.0
# meant the silhouette silently stopped matching the wheels whenever the motor length changed.
# The taper at the top keeps its 7 mm, measured off that, not off an absolute 82.
SKIN = {"half_w": [(Z_BOT, Y_OUT), (100.0, Y_OUT), (Z_TOP, Y_OUT - 7.0)],
        "r_top": 21.0, "r_bot": 14.0, "front_round": 10.0}
FENDER_LIPS = True


def fender_lips():
    """'Eyebrow' arches over both wheels on the side faces (3.5 mm proud of the side)."""
    lips = None
    for s_ in (1, -1):
        y0 = Y_OUT - 0.5 if s_ > 0 else -Y_OUT - 3.0
        ring = (cq.Workplane("XZ", origin=(0, y0, 0)).circle(WELL_R + 4.5).circle(WELL_R + 0.3).extrude(-3.5))
        ring = ring.intersect(box(X_BACK, X_IN_F, -200, 200, 18.0, 200))
        ring = ring.edges().fillet(1.2)
        lips = ring if lips is None else lips.union(ring)
    return lips


def skin_solid(x0, x1, t=0.0):
    """Outer silhouette (YZ) extruded along X, shrunk by t (wall thickness) for the inner cavity."""
    if SKIN is None:
        o = box(x0, x1, -Y_OUT + t, Y_OUT - t, Z_BOT + t, Z_TOP - t)
        return o.edges("|X").fillet(max(FILLET - t, 0.5)) if FILLET - t > 0.5 else o
    hw = SKIN["half_w"]
    zs = [z for z, _ in hw]
    pts = []
    for (z, w) in hw:
        zz = min(max(z, Z_BOT + t), Z_TOP - t)
        pts.append((w - t, zz))
    pts = pts + [(-w, z) for (w, z) in reversed(pts)]
    sk = cq.Sketch().polygon(pts + [pts[0]])
    rt, rb = max(SKIN["r_top"] - t, 1.0), max(SKIN["r_bot"] - t, 1.0)
    sk = sk.vertices(">Y").fillet(rt).reset().vertices("<Y").fillet(rb)
    solid = cq.Workplane("YZ", origin=(x0, 0, 0)).placeSketch(sk).extrude(x1 - x0)
    fr = SKIN.get("front_round", 0)
    if fr and t == 0:   # soften the top-front edge (side profile)
        corner = box(X_FRONT - fr, X_FRONT + 1, -300, 300, Z_TOP - fr, Z_TOP + 1)
        corner = corner.cut(cyl_y(-300, 600, X_FRONT - fr, Z_TOP - fr, 2 * fr))
        solid = solid.cut(corner)
    return solid


def outline(x0, x1):
    """Body cross-section (outer skin minus the wheel wells), extruded from x0 to x1."""
    return skin_solid(x0, x1).cut(wells(x0 - 1, x1 + 1, WELL_R))


def build_body():
    inner = (box(X_BACK - 1, X_IN_F + 1, -Y_IN, Y_IN, Z_FLOOR, Z_IN_TOP) if SKIN is None
             else skin_solid(X_BACK - 1, X_IN_F + 1, SIDE).intersect(box(X_BACK - 1, X_IN_F + 1, -Y_IN, Y_IN, Z_FLOOR, Z_IN_TOP)))
    shell = outline(X_BACK, X_IN_F).cut(inner)
    # wheel arches: a 3 mm shell over each well, inside the body outline
    arch = wells(X_BACK, X_IN_F, WELL_R + ARCH_T).cut(wells(X_BACK - 1, X_IN_F + 1, WELL_R))
    arch = arch.intersect(box(X_BACK, X_IN_F, -Y_OUT, Y_OUT, Z_BOT, Z_TOP))
    body = shell.union(arch)

    # motor walls (double centre wall) with centering pocket, M2.5 clearance holes, connector slot
    for s in (1, -1):
        y0 = GAP_HALF if s > 0 else -(GAP_HALF + MWALL_T)
        body = body.union(box(X_BACK, X_IN_F, y0, y0 + MWALL_T, Z_FLOOR, SHELF_Z))
        yp0 = (GAP_HALF + MWALL_T - POCKET_DEP) if s > 0 else -(GAP_HALF + MWALL_T) - 0.5
        body = body.cut(cyl_y(yp0, POCKET_DEP + 0.5, 0, 0, POCKET_D))
        # teardrop toward -X (the print's "up": the body prints front face down): no flat 2 mm ledge at the
        # back of the pocket. The motor still centres on the remaining 270 deg of the circle + its 4 screws.
        rp = POCKET_D / 2
        body = body.cut(cq.Workplane("XZ", origin=(0, yp0 + POCKET_DEP + 0.5, 0))
                        .polyline([(-rp / 2 ** 0.5, rp / 2 ** 0.5), (-rp * 2 ** 0.5, 0), (-rp / 2 ** 0.5, -rp / 2 ** 0.5), (0, 0)]).close()
                        .extrude(POCKET_DEP + 0.5))
        for (x, z) in [(-MOTOR_SQ / 2, -MOTOR_SQ / 2), (MOTOR_SQ / 2, -MOTOR_SQ / 2),
                       (MOTOR_SQ / 2, MOTOR_SQ / 2), (-MOTOR_SQ / 2, MOTOR_SQ / 2)]:
            body = body.cut(cyl_y(y0 - 1, MWALL_T + 2, x, z, M25_CLEAR))
        body = body.cut(box(-SLOT_W / 2, SLOT_W / 2, y0 - 1, y0 + MWALL_T + 1, SLOT_Z[0], SLOT_Z[1]).edges("|Y").fillet(3.0))   # round corners: FEA hot spot
        yo_ = y0 + MWALL_T if s > 0 else y0          # outer (motor-side) face
        body = body.cut(box(-6.0, 3.5, min(yo_, yo_ - s * 2.2), max(yo_, yo_ - s * 2.2), SLOT_Z[1] - 1, SHELF_Z + 0.1))   # 2 mm notch: the 3-pin plug goes in from above

    # reinforcement (FEA, Nov 2026): 45-deg fillets at the wall roots, ribs from each wall to the front/back
    # floor bosses, and thickened wall tops for the shelf-screw inserts
    for s_ in (1, -1):
        yi, yo = s_ * GAP_HALF, s_ * (GAP_HALF + MWALL_T)          # inner (gap) and outer (motor) wall faces
        for (yface, c, d) in ((yi, ROOT_CHAMFER[0], -s_), (yo, ROOT_CHAMFER[1], s_)):
            tri = (cq.Workplane("YZ", origin=(X_BACK, 0, 0))
                   .polyline([(yface, Z_FLOOR - 0.5), (yface + d * c, Z_FLOOR - 0.5), (yface, Z_FLOOR + c)]).close()
                   .extrude(X_IN_F - X_BACK))
            body = body.union(tri)
        for (xa, xb) in ((X_IN_F - BOSS_L, X_IN_F), (X_BACK + PAD_T, X_BACK + PAD_T + BOSS_L)):   # wall -> floor boss rib
            ya, yb_ = sorted((yo - s_ * 0.5, s_ * (FLOOR_SCREW[0] - BOSS_D / 2 + 0.5)))
            body = body.union(box(xa, xb, ya, yb_, Z_FLOOR - 0.5, FLOOR_SCREW[1] + BOSS_D / 2))
            if xa == X_BACK:   # back rib: 45-deg gusset under its front face (it would be a ledge when printing front-down)
                body = body.union(gusset_xz(xb, FLOOR_SCREW[1] + BOSS_D / 2, Z_FLOOR - 0.5, ya, yb_))
        yc = s_ * (GAP_HALF + MWALL_T / 2)
        for xs in SHELF_SCREW_X:
            body = body.union(cq.Workplane("XY", origin=(0, 0, 20.0)).center(xs, yc).circle(SHELF_BOSS_D / 2).extrude(SHELF_Z - 20.0)
                              .intersect(box(X_BACK, X_IN_F, -200, 200, 0, 100)))

    # wall tops: the battery shelf sits flat on them and is screwed down (2 x M3 per wall, countersunk from the
    # shelf into heat-set inserts). The screws tie the two walls together; there are no dovetail rails anymore.
    for s_ in (1, -1):
        yc = s_ * (GAP_HALF + MWALL_T / 2)
        for xs in SHELF_SCREW_X:
            body = body.cut(cq.Workplane("XY", origin=(0, 0, SHELF_Z - 8.0)).center(xs, yc).circle(INS_M3[0] / 2).extrude(8.1))

    # insert bosses for front panel (start on the print bed) and back cover (get 45-degree gussets,
    # because they would otherwise start in mid-air when the body prints on its front face)
    for front in (True, False):
        xb0 = X_IN_F - BOSS_L if front else X_BACK + PAD_T     # back: the cover's pads sit in front of the bosses
        for s_ in (1, -1):
            for (yb, zb) in PANEL_SCREWS:
                y = s_ * yb
                body = body.union(cyl_x(xb0, BOSS_L, y, zb, BOSS_D))
                if abs(yb) > 60:   # tied to the side wall
                    ya, yw = y - s_ * BOSS_D / 2, s_ * Y_IN
                    body = body.union(box(xb0, xb0 + BOSS_L, min(ya, yw), max(ya, yw), zb - BOSS_D / 2, zb + BOSS_D / 2))
                    if not front:
                        body = body.union(gusset_xy(xb0 + BOSS_L, ya, yw, zb - BOSS_D / 2, zb + BOSS_D / 2))
                else:              # tied to the floor
                    ztop = zb + BOSS_D / 2
                    body = body.union(box(xb0, xb0 + BOSS_L, y - BOSS_D / 2, y + BOSS_D / 2, Z_FLOOR, ztop))
                    if not front:
                        body = body.union(gusset_xz(xb0 + BOSS_L, ztop, Z_FLOOR, y - BOSS_D / 2, y + BOSS_D / 2))
                hx0 = X_IN_F - PAD_T - INS_M3[1] if front else X_BACK + PAD_T - 1
                body = body.cut(cyl_x(hx0, INS_M3[1] + 1, y, zb, INSERT_D))
                if front:   # pocket for the front panel's pad (its roof is a short bridge around the insert hole)
                    body = body.cut(cyl_x(X_IN_F - PAD_T - 0.2, PAD_T + 1.2, y, zb, PAD_D_FRONT + 0.4))

    # top: lidar holes + bosses, cable hole, arm button, power button, LED (teardrops)
    lid_pts = [(LIDAR_PAIR_X, LIDAR_PAIR_Y), (LIDAR_PAIR_X, -LIDAR_PAIR_Y), (LIDAR_SINGLE_X, 0.0)]
    for (x, y) in lid_pts:
        body = body.union(teardrop_boss_z(x, y, INS_M2[2], Z_IN_TOP - 3, 3.5))
        body = body.cut(cq.Workplane("XY", origin=(0, 0, Z_TOP - INS_M2[1])).center(x, y).circle(INS_M2[0] / 2).extrude(INS_M2[1] + 1))
    for (x, y, d) in (LIDAR_CABLE, BUTTON, POWER_BTN, LED):
        body = body.cut(teardrop_z(x, y, d, Z_IN_TOP - 1, TOP_T + 2))
    for (x, y, _) in IR_RX:                                    # IR receiver leads down through the top skin
        body = body.cut(teardrop_z(x, y, IR_LEAD_HOLE, Z_IN_TOP - 1, TOP_T + 2))

    # side-wall mounting points (carrier plates for the charger boost and the wireless receiver)
    for s_ in (1,):
        y0 = Y_IN - WALL_BOSS_L if s_ > 0 else -Y_IN
        for (x, z) in WALL_PLATE_PTS:
            body = body.union(teardrop_boss_y(x, z, INS_M3[2], y0, WALL_BOSS_L + 0.5 if s_ > 0 else WALL_BOSS_L))
            hy0 = Y_IN - WALL_BOSS_L - 0.1 if s_ > 0 else -Y_IN + WALL_BOSS_L - INS_M3[1]
            body = body.cut(cyl_y(hy0, INS_M3[1] + 0.1, x, z, INS_M3[0]))
    # floor mounting points for the electronics tray
    for (x, y) in FLOOR_TRAY_PTS:
        body = body.union(teardrop_boss_z(x, y, INS_M3[2], Z_FLOOR - 0.5, FLOOR_BOSS_H + 0.5))
        body = body.cut(cq.Workplane("XY", origin=(0, 0, Z_FLOOR + FLOOR_BOSS_H - INS_M3[1])).center(x, y)
                        .circle(INS_M3[0] / 2).extrude(INS_M3[1] + 0.1))

    # right side: USB; both sides: vents
    for (x, z, w, h) in ():   # USB_SLOT removed: the real DevKitC USB-C is 24 mm inside the wall (flash with the cover off, or from the Pi)
        body = body.cut(box(x - w / 2, x + w / 2, -Y_OUT - 1, -Y_IN + WALL_CUT_DEPTH, z - h / 2, z + h / 2))
    for s in (1,):                                   # vents on the left; the right wall has the speaker grille
        for xv in VENTS_X:
            ya, yb_ = s * (Y_IN - WALL_CUT_DEPTH), s * (Y_OUT + 1)
            body = body.cut(box(xv - VENT_W / 2, xv + VENT_W / 2, min(ya, yb_), max(ya, yb_), VENTS_Z[0], VENTS_Z[1]))
    # speaker: grille through the right wall (teardrop holes, point toward -X = up when printing on the front face),
    # clearance for flange + ring, 3 insert bosses for the ring
    sx, sz = SPK_XZ
    for i in range(-4, 5):
        for j in range(-4, 5):
            px, pz = SPK_GRILLE_PITCH
            x, z = sx + px * i, sz + pz * j + (pz / 2 if i % 2 else 0)
            if (x - sx) ** 2 + (z - sz) ** 2 <= SPK_GRILLE_R ** 2:
                body = body.cut(teardrop_y(x, z, SPK_GRILLE_HOLE, -Y_OUT - 6, 14))
    body = body.cut(cyl_y(SPK_Y0, SPK_FLANGE + RING_T + 0.5, sx, sz, SPK_D + SPK_CLEAR))
    yb = SPK_Y0 + SPK_FLANGE                          # boss face = back of the flange = ring face
    for (bx, bz) in SPK_BOSSES:
        body = body.union(teardrop_boss_y(bx, bz, INS_M2[2], -Y_OUT + 0.5, yb + Y_OUT - 0.5))
        body = body.cut(cyl_y(yb - INS_M2[1], INS_M2[1] + 0.1, bx, bz, INS_M2[0]))
    # USB-C charging socket (panel-mount extension, M3 screws 18 mm apart along X) in the left side wall, above the
    # wheel: reachable when the robot stands, lies face-down or lies on its back
    ux, uz = USBC_SIDE
    body = body.cut(box(ux - USBC_CUT[0] / 2, ux + USBC_CUT[0] / 2, Y_IN - 1, Y_OUT + 1, uz - USBC_CUT[1] / 2, uz + USBC_CUT[1] / 2))
    for dx in (-USBC_PITCH / 2, USBC_PITCH / 2):
        body = body.cut(cyl_y(Y_IN - 1, SIDE + 2, ux + dx, uz, USBC_SCREW))
    if SKIN is not None:
        body = body.intersect(skin_solid(X_BACK - 1, X_IN_F + 1).cut(wells(X_BACK - 2, X_IN_F + 2, WELL_R)))
    if SKIN is not None and FENDER_LIPS:
        body = body.union(fender_lips())
    return body


def csk_x(x_face, outward, y, z, depth_total):
    """Countersunk M3 hole along X from an outside face at x_face (outward = +1: face looks to +x)."""
    cone_h = (CSK_D - 3.4) / 2
    if outward > 0:
        h = cyl_x(x_face - CSK_SINK, CSK_SINK + 1, y, z, CSK_D)
        c = cq.Workplane("YZ", origin=(x_face - CSK_SINK, 0, 0)).center(y, z).circle(CSK_D / 2).workplane(offset=-cone_h).circle(1.7).loft()
        t = cyl_x(x_face - depth_total - 1, depth_total + 1, y, z, 3.4)
    else:
        h = cyl_x(x_face - 1, CSK_SINK + 1, y, z, CSK_D)
        c = cq.Workplane("YZ", origin=(x_face + CSK_SINK, 0, 0)).center(y, z).circle(CSK_D / 2).workplane(offset=cone_h).circle(1.7).loft()
        t = cyl_x(x_face, depth_total + 1, y, z, 3.4)
    return h.union(c).union(t)


def screw_holes(panel, x_face_out, x_face_in, outward):
    """Countersunk M3 (ISO 10642) panel screws: a 1.5 mm pad inside around each hole makes room for the countersink."""
    t = abs(x_face_out - x_face_in) + PAD_T
    for s in (1, -1):
        for (yb, zb) in PANEL_SCREWS:
            y = s * yb
            if outward > 0:   # front: outside face at larger x, pad on the inside (smaller x), into a pocket in the boss
                panel = panel.union(cyl_x(x_face_in - PAD_T, PAD_T + 0.5, y, zb, PAD_D_FRONT))
            else:
                panel = panel.union(cyl_x(x_face_in - 0.5, PAD_T + 0.5, y, zb, BOSS_D))
            panel = panel.cut(csk_x(x_face_out, outward, y, zb, t))
    return panel


def build_front():
    panel = outline(X_IN_F, X_FRONT)
    panel = screw_holes(panel, X_FRONT, X_IN_F, +1)
    # camera window + M2 posts
    panel = panel.cut(cq.Workplane("YZ", origin=(X_IN_F - 1, 0, 0)).center(0, CAM_Z).rect(*CAM_WIN).extrude(WALL + 2).edges("|X").fillet(2.0))
    panel = panel.cut(cq.Workplane("YZ", origin=(X_FRONT - CAM_PC_WIN[2], 0, 0)).center(0, CAM_Z).rect(CAM_PC_WIN[0], CAM_PC_WIN[1]).extrude(CAM_PC_WIN[2] + 1).edges("|X").fillet(2.0))
    hw, hh = CAM_HOLES[0] / 2, CAM_HOLES[1] / 2
    for (y, z) in [(-hw, CAM_HOLE_Z - hh), (hw, CAM_HOLE_Z - hh), (hw, CAM_HOLE_Z + hh), (-hw, CAM_HOLE_Z + hh)]:
        panel = panel.union(cyl_x(X_IN_F - STANDOFF_F, STANDOFF_F + 0.5, y, z, INS_M2[2]))
        panel = panel.cut(cyl_x(X_IN_F - STANDOFF_F - 1, INS_M2[1] + 1, y, z, INS_M2[0]))   # M2 insert from the post end
    # LED eyes: window + pocket for panel and diffuser
    for y in (-EYE_Y, EYE_Y):
        panel = panel.cut(cq.Workplane("YZ", origin=(X_IN_F - 1, 0, 0)).center(y, EYE_Z)
                          .rect(EYE_WIN, EYE_WIN).extrude(WALL + 2).edges("|X").fillet(EYE_WIN_R))
        panel = panel.cut(cq.Workplane("YZ", origin=(X_IN_F - 1, 0, 0)).center(y, EYE_Z)
                          .rect(EYE_PCB, EYE_PCB).extrude(1 + EYE_POCKET))
    # sonar "mouth": transducer holes + 4 M2 insert posts for the HC-SR04 corner holes
    for y in (-SONAR_PITCH / 2, SONAR_PITCH / 2):
        panel = panel.cut(cyl_x(X_IN_F - 1, WALL + 2, y, SONAR_Z, SONAR_D))
    hw, hh = SONAR_HOLES[0] / 2, SONAR_HOLES[1] / 2
    for (y, z) in [(-hw, SONAR_Z - hh), (hw, SONAR_Z - hh), (hw, SONAR_Z + hh), (-hw, SONAR_Z + hh)]:
        panel = panel.union(cyl_x(X_IN_F - SONAR_POST, SONAR_POST + 0.5, y, z, 5.4))   # Ø5.4: Ø6.4 clipped the real transducers by 0.2 mm
        panel = panel.cut(cyl_x(X_IN_F - SONAR_POST - 1, INS_M2[1] + 1, y, z, INS_M2[0]))
    # VL53L5CX seat: a 30 deg pad with two M2 insert bosses, and a window that flares along the same
    # axis. Fitted INSTEAD of the sonar; the sonar mouth stays in the print so one panel serves both.
    # The bore cannot run straight: the receiver needs 61 deg of vertical clearance, so it is a cone.
    t = math.radians(TOF_TILT)
    d = cq.Vector(math.cos(t), 0, -math.sin(t))          # optical axis: forward and down
    seat = cq.Vector(X_IN_F, 0, TOF_Z) - d * TOF_SEAT    # the board's back face rests here
    for y in (-TOF_HOLES / 2, TOF_HOLES / 2):
        b = seat + cq.Vector(0, y, 0)
        panel = panel.union(cq.Workplane(obj=cq.Solid.makeCylinder(INS_M2[2] / 2, TOF_SEAT + 4.0, b, d)))
        panel = panel.cut(cq.Workplane(obj=cq.Solid.makeCylinder(
            INS_M2[0] / 2, INS_M2[1] + 2, b - d * 1.0, d)))        # M2 insert, pressed in from the seat
    panel = panel.cut(cq.Workplane(obj=cq.Solid.makeCone(
        TOF_WIN[0] / 2, TOF_WIN[1] / 2, TOF_SEAT + WALL + 4.0, seat - d * 1.0, d)))
    # nothing may stand proud of the face: the body's front bumpers land on it
    panel = panel.cut(box(X_FRONT, X_FRONT + 40, -200, 200, -200, 200))
    # (USB-C charging socket moved to the left side wall: see build_body)
    # eye retainers: 2 M2 insert bosses per eye (diagonal), 5 mm tall
    for sy in (1, -1):                               # microphone ports, board pockets, clamp bosses
        my, mz = sy * MIC_YZ[0], MIC_YZ[1]
        panel = panel.cut(cyl_x(X_IN_F - 1, WALL + 2, my, mz, MIC_PORT_D))
        pc = my - MIC_CHIP_DY                       # both boards the same way up
        panel = panel.cut(box(X_IN_F - 1, X_IN_F + MIC_POCKET, pc - MIC_PCB[0] / 2, pc + MIC_PCB[0] / 2,
                              mz - MIC_PCB[1] / 2, mz + MIC_PCB[1] / 2))
        panel = panel.union(cyl_x(X_IN_F - 5.0, 5.5, my, mz + MIC_BOSS_DZ, INS_M2[2]))
        panel = panel.cut(cyl_x(X_IN_F - 5.0 - 1, INS_M2[1] + 1, my, mz + MIC_BOSS_DZ, INS_M2[0]))
    for (y, z) in eye_retainer_points():
        panel = panel.union(cyl_x(X_IN_F - 5.0, 5.5, y, z, INS_M2[2]))
        panel = panel.cut(cyl_x(X_IN_F - 5.0 - 1, INS_M2[1] + 1, y, z, INS_M2[0]))
    for sy in (1, -1):                               # COB disc headlights: collar + 45° window + 2 retainer bosses each
        ly = sy * LIGHT_Y
        panel = panel.union(cyl_x(X_IN_F - LIGHT_T, LIGHT_T + 0.5, ly, LIGHT_Z, LIGHT_COLLAR))
        panel = panel.cut(cyl_x(X_IN_F - LIGHT_T - 1, LIGHT_T + 1, ly, LIGHT_Z, LIGHT_BORE))
        cone = cq.Solid.makeCone(LIGHT_WIN[0] / 2, LIGHT_WIN[1] / 2, WALL, cq.Vector(X_IN_F, ly, LIGHT_Z), cq.Vector(1, 0, 0))
        panel = panel.cut(cq.Workplane().add(cone)).cut(cyl_x(X_IN_F - 0.5, 0.6, ly, LIGHT_Z, LIGHT_WIN[0]))
        panel = panel.cut(cyl_x(X_FRONT - 0.01, 1.0, ly, LIGHT_Z, LIGHT_WIN[1]))
    for (dy, dz) in DRV_PTS:                         # headlight driver board: 2 long M2 bosses
        y, z = DRV_CENTER[0] + dy, DRV_CENTER[1] + dz
        panel = panel.union(cyl_x(X_IN_F - DRV_STANDOFF, DRV_STANDOFF + 0.5, y, z, INS_M2[2]))
        panel = panel.cut(cyl_x(X_IN_F - DRV_STANDOFF - 1, INS_M2[1] + 1, y, z, INS_M2[0]))
    for (y, z) in light_retainer_points():
        panel = panel.union(cyl_x(X_IN_F - 5.0, 5.5, y, z, INS_M2[2]))
        panel = panel.cut(cyl_x(X_IN_F - 5.0 - 1, INS_M2[1] + 1, y, z, INS_M2[0]))
    return panel


def light_retainer_points(side=None):
    pts = []
    for s_ in ((1, -1) if side is None else (side,)):
        for (dy, dz) in LIGHT_RET_PTS:
            pts.append((s_ * (LIGHT_Y + dy), LIGHT_Z + dz))
    return pts


def _hull(pts):
    pts = sorted(set(pts))
    def cross(o, a, b): return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0: hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def build_light_retainer(side=1):
    """Plate that clamps one COB disc into its collar: 2 x M2x6 into the panel inserts; a ring presses the disc's rim,
    a Ø18 hole passes the two wires and lets air reach the disc's back. Print flat, ring up."""
    yc, zc = side * LIGHT_Y, LIGHT_Z
    xs = X_IN_F - 5.0 - 2.0                                   # plate x 25..27 (boss faces at 27)
    circ = []
    for (cy, cz, r) in [(yc, zc, (LIGHT_BORE - 1.0) / 2 + 1.0)] + [(y, z, 3.4) for (y, z) in light_retainer_points(side)]:
        for i in range(48):
            a = 2 * math.pi * i / 48
            circ.append((round(cy + r * math.cos(a), 3), round(cz + r * math.sin(a), 3)))
    h = _hull(circ)
    plate = cq.Workplane("YZ", origin=(xs, 0, 0)).polyline(h).close().extrude(2.0)
    plate = plate.cut(cyl_x(xs - 1, 4, yc, zc, 18.0))
    ring = cyl_x(xs + 2.0, X_IN_F - LIGHT_T - xs - 2.0, yc, zc, LIGHT_BORE - 1.0).cut(cyl_x(xs + 1.0, 4, yc, zc, LIGHT_D - 6.0))
    plate = plate.union(ring)
    for (y, z) in light_retainer_points(side):
        plate = plate.cut(cyl_x(xs - 1, 4, y, z, 2.4))
    return plate


def eye_retainer_points():
    pts = []
    for s_ in (1, -1):
        for (dy, dz) in EYE_RET_PTS:
            y_off = dy if dz > 0 else -dy      # top boss on the outside, bottom boss on the inside
            pts.append((s_ * (EYE_Y + y_off), EYE_Z + dz))
    return pts


def build_eye_retainer(side=1):
    """Plate that clamps one LED-eye panel (and diffuser) into its pocket. 2 x M2x6 into the inserts.
    A 2 mm frame presses on the PCB edge. Print flat, frame up."""
    yc, zc = side * EYE_Y, EYE_Z
    pts = [p for p in eye_retainer_points() if (p[0] > 0) == (side > 0)]
    xs = X_IN_F - 5.0 - 2.0
    ys = [p[0] for p in pts] + [yc - 15, yc + 15]; zs = [p[1] for p in pts] + [zc - 15, zc + 15]
    plate = box(xs, xs + 2.0, min(ys) - 3.4, max(ys) + 3.4, min(zs) - 3.4, max(zs) + 3.4).edges("|X").fillet(3)
    pcb_back = X_IN_F + EYE_POCKET - DIFFUSER_T - 2.5        # back of the LED panel PCB
    ring = (cq.Workplane("YZ", origin=(xs + 2.0, 0, 0)).center(yc, zc).rect(EYE_PCB - 1, EYE_PCB - 1)
            .rect(EYE_PCB - 6, EYE_PCB - 6).extrude(pcb_back - xs - 2.0))
    plate = plate.union(ring)
    plate = plate.cut(cq.Workplane("YZ", origin=(xs, 0, 0)).center(yc, zc).rect(10, 6).extrude(2.0))  # wire pass
    hw, hh = CAM_HOLES[0] / 2, CAM_HOLES[1] / 2
    for (y, z) in [(-hw, CAM_HOLE_Z - hh), (hw, CAM_HOLE_Z - hh), (hw, CAM_HOLE_Z + hh), (-hw, CAM_HOLE_Z + hh)]:
        plate = plate.cut(cyl_x(xs - 1, 10, y, z, INS_M2[2] + 1.0))      # clear the (longer) AI Camera posts
    for (y, z) in pts:
        plate = plate.cut(cyl_x(xs - 1, 4, y, z, 2.4))
    return plate


def build_cover():
    x0 = X_BACK - COVER_T
    cover = outline(x0, X_BACK)
    cover = screw_holes(cover, x0, X_BACK, -1)
    # perfboard (M3 into inserts) and Pi Zero (M2 into inserts) standoffs
    w, h = PB_HOLES[0] / 2, PB_HOLES[1] / 2
    cy, cz = PB_CENTER
    for (y, z) in [(cy - w, cz - h), (cy + w, cz - h), (cy + w, cz + h), (cy - w, cz + h)]:
        cover = cover.union(cyl_x(X_BACK - 0.5, BOARD_STANDOFF + 0.5, y, z, INS_M3[2]))
        cover = cover.cut(cyl_x(X_BACK + BOARD_STANDOFF - INS_M3[1], INS_M3[1] + 1, y, z, INS_M3[0]))
    w, h = PI_HOLES[0] / 2, PI_HOLES[1] / 2
    cy, cz = PI_CENTER
    for (y, z) in [(cy - w, cz - h), (cy + w, cz - h), (cy + w, cz + h), (cy - w, cz + h)]:
        cover = cover.union(cyl_x(X_BACK - 0.5, BOARD_STANDOFF + 0.5, y, z, INS_M2[2]))      # Pi: M2 screws (its holes are 2.75)
        cover = cover.cut(cyl_x(X_BACK + BOARD_STANDOFF - INS_M2[1], INS_M2[1] + 1, y, z, INS_M2[0]))
    # the top-right perfboard standoff reaches under the lidar's rear insert boss: notch its top edge
    cover = cover.cut(cq.Workplane("XY", origin=(0, 0, Z_IN_TOP - 3.5)).center(LIDAR_SINGLE_X, 0.0).circle(INS_M2[2] / 2 + 0.5).extrude(10))
    # contact plate: 4 short insert bosses (inserts pressed in from the outside) + 2 terminal screw holes
    for (y, z) in contact_screw_points():
        cover = cover.union(cyl_x(X_BACK - 0.5, CONTACT_BOSS_L + 0.5, y, z, INS_M3[2]))
        cover = cover.cut(cyl_x(x0 - 1, INS_M3[1] + 1, y, z, INS_M3[0]))
    for (y, z) in PADS.values():
        cover = cover.cut(cyl_x(x0 - 1, COVER_T + 2, y, z, PAD_SCREW))
    # vents behind the battery
    for sgn in (1, -1):
        for z in (45.0, 52.0):
            cover = cover.cut(box(x0 - 1, X_BACK + 1, min(sgn * 56, sgn * 80), max(sgn * 56, sgn * 80), z - 1.5, z + 1.5)
                              .edges("|X").fillet(1.4))
    # keep the standoffs clear of the body's back screw bosses and their wall ties (0.3 mm gap)
    for s_ in (1, -1):
        for (yb, zb) in PANEL_SCREWS:
            y = s_ * yb
            cover = cover.cut(cyl_x(X_BACK + PAD_T - 0.01, BOSS_L + 1, y, zb, BOSS_D + 0.6))
            if abs(yb) > 60:
                ya, yw = y - s_ * (BOSS_D / 2 + 0.3), s_ * Y_IN
                cover = cover.cut(box(X_BACK + PAD_T - 0.01, X_BACK + PAD_T + BOSS_L + 1, min(ya, yw), max(ya, yw), zb - BOSS_D / 2 - 0.3, zb + BOSS_D / 2 + 0.3))
    return cover


def build_contact_plate():
    """Charging contact plate on the OUTSIDE of the back cover, at the top, for the vertical dock.
    Two lead-in funnels (22 mm mouth, stretched 8 mm downwards, 8 mm deep) guide the dock's sprung rods onto the heads
    of two M5×12 brass countersunk screws (the contacts; through plate + cover to a ring lug + nut inside; the diode
    inside keeps them dead unless a charger is on; nothing flat can reach them). The robot cannot slide sideways, so
    the lidar guidance has to land it within ~±3 mm (sim/dock_sim.py); the Ø9.3 heads take ±4.5. A vertical V slot
    between them takes the dock's tongue. No magnets: the robot's weight presses the plate on, the floor wave keeps it there.
    Print inner face down (funnels open upwards: no overhang)."""
    y0, y1, z0, z1 = CONTACT_PLATE
    x_in = X_BACK - COVER_T
    x_out = x_in - CONTACT_T
    plate = box(x_out, x_in, y0, y1, z0, z1).edges("|X").fillet(CONTACT_PLATE_EDGE[0]).edges("<X").chamfer(CONTACT_PLATE_EDGE[1])
    for (y, z) in contact_screw_points():                    # countersunk M3 (ISO 10642): flush
        plate = plate.cut(csk_x(x_out, -1, y, z, CONTACT_T))
    fd, fdep = FUNNEL
    for (y, z) in PADS.values():
        # entrance stretched downward: backing in upright, the funnel sits ~6 mm above the rod and drops onto it as the
        # robot leans back (docked_transform), so the lead-in reaches 7 mm further down than up
        cone = (cq.Workplane("YZ", origin=(x_out - 0.01, 0, 0)).center(y, z - FUNNEL_DROP / 2).slot2D(fd + FUNNEL_DROP, fd, 90)
                .workplane(offset=fdep).center(0, FUNNEL_DROP / 2).circle(PAD_HEAD / 2 + PAD_SEAT_CLEAR).loft()).val()
        csk = cq.Solid.makeCone(PAD_HEAD / 2 + PAD_SEAT_CLEAR, PAD_SCREW / 2, PAD_HEAD / 2 + PAD_SEAT_CLEAR - PAD_SCREW / 2, cq.Vector(x_out + fdep - 0.02, y, z), cq.Vector(1, 0, 0))
        plate = plate.cut(cq.Workplane().add(cone)).cut(cq.Workplane().add(csk)).cut(cyl_x(x_out - 1, CONTACT_T + 2, y, z, PAD_SCREW))
    sw0, sw1, sd = GUIDE_SLOT                                 # V slot, open at top and bottom
    pts = [(x_out - 0.01, -sw0 / 2), (x_out - 0.01, sw0 / 2), (x_out + sd, sw1 / 2), (x_out + sd, -sw1 / 2)]
    slot = cq.Workplane("XY", origin=(0, 0, z0 - 1)).polyline(pts).close().extrude(z1 - z0 + 2)
    plate = plate.cut(slot)
    return plate


def docked_transform(shape):
    """Robot frame -> world frame of the docked robot: pitched back DOCK_LEAN about the axle, wheels on the floor (z = 0)."""
    return shape.rotate((0, 0, 0), (0, 1, 0), -DOCK_LEAN).translate((0, 0, WHEEL_R + DOCK_FLOOR))


def dock_wave_profile(x, ribs=False):
    """Top of a wheel track (mm above the floor) at x (docked axle at x = 0)."""
    def cb(x, a, b, h0, h1):
        t = min(1.0, max(0.0, (x - a) / (b - a)))
        return h0 + (h1 - h0) * (1 - math.cos(math.pi * t)) / 2
    hr, r0, r1, r2 = SADDLE_REAR
    hf, f0, f1, f2 = SADDLE_FRONT
    t = DOCK_FLOOR
    if x > f2: return max(0.0, t * (1 - (x - f2) / SADDLE_LEAD))
    if x < r0: z = 0.0
    elif x < r1: z = cb(x, r0, r1, 0, hr)
    elif x < r2: z = cb(x, r1, r2, hr, SADDLE_TILT)
    elif x < f0: z = SADDLE_TILT * (f0 - x) / (f0 - r2)     # trough bottom tilts ~1° down towards the front lip
    elif x < f1: z = cb(x, f0, f1, 0, hf)
    else: z = cb(x, f1, f2, hf, 0)
    z += t
    if ribs:
        h, p = SADDLE_RIB
        u = (x % p) / p
        z += h * (u / 0.5 if u < 0.5 else (1 - u) / 0.5)
    return z


def _track(xa, xb, y0, y1, ribs=False, step=1.0):
    n = max(2, int(round((xb - xa) / step)))
    top = [(xa + (xb - xa) * i / n, dock_wave_profile(xa + (xb - xa) * i / n, ribs)) for i in range(n + 1)]
    while top and top[-1][1] < 1e-3: top.pop()
    while top and top[0][1] < 1e-3: top.pop(0)
    pts = [(xa, 0.0)] + top + [(xb, 0.0)]
    return cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close().extrude(y1 - y0)


def _dovetail(xa, xb, y0, y1, grow, c=0.0):
    """Window cross-section: 'grow' mm wider each side at the floor than at 4 mm up (the insert can't lift out)."""
    pts = [(y0 - grow - c, 0.0), (y1 + grow + c, 0.0), (y1 + c, 6.0), (y0 - c, 6.0)]
    return cq.Workplane("YZ", origin=(xa, 0, 0)).polyline(pts).close().extrude(xb - xa)


def dock_wave_strip(sgn, clearance=0.0, part="frame"):
    """One floor ridge (left = sgn +1; the right one is its mirror). part 'frame' (PETG): the wheel track outside the
    insert (rear lip, lead-in), side bands around the insert window, a rail outside the tyre and a T tie that drops into
    a slot in the floor plate (it can't pull out forwards, the way the robot pushes). part 'insert' (TPU 95A): the
    ribbed trough + front lip, a dovetail that drops into the frame window from below.
    clearance > 0: the floor-plate slot shape instead."""
    if sgn < 0:
        return dock_wave_strip(1, clearance, part).mirror("XZ")
    y0, y1 = DOCK_WAVE_Y
    c = clearance
    if c:
        neck = box(-78.0 - c, -57.0, 70.0 - c, 116.0 + c, -1.0, 5.0)
        head = box(-88.0 - c, -78.0 + c, 64.0 - c, 88.0 + c, -1.0, 5.0)
        return neck.union(head)
    xi0, xi1 = SADDLE_INSERT
    xe = SADDLE_FRONT[3] + SADDLE_LEAD
    if part == "insert":
        ins = _track(xi0 + 0.15, xi1 - 0.15, y0 - 1.0, y1 + 1.0, ribs=True, step=0.2)
        return ins.intersect(_dovetail(xi0 + 0.15, xi1 - 0.15, y0, y1, 1.0, c=-0.15))
    fy0, fy1 = y0 - 4.0, y1 + 4.0                                  # frame: 4 mm bands either side of the track
    tr = _track(-62.0, xe, fy0, fy1)
    tr = tr.cut(_dovetail(xi0, xi1, y0, y1, 1.0))
    rail = box(-78.0, xe, 110.0, 116.0, 0, 3.0)                   # outside the insert window (it reaches 109)
    neck = box(-78.0, -62.0, 70.0, 116.0, 0, 4.0)
    head = box(-88.0, -78.0, 64.0, 88.0, 0, 4.0)
    return tr.union(rail).union(neck).union(head)


def build_dock(part="tower"):   # "tower" | "mast" | "wing" | "ridge" | "ridge_insert" | viewer: "tape" | "ir_leds" | "board" | "rods" | "jack" | "wires_pos" | "wires_neg" | "wires_led"
    """Vertical charging dock, in the world frame of the docked robot (floor z = 0, wheel axle at x = y = 0, robot
    facing +x). The robot backs in upright; the wings (on the face slab) centre it within ±25 mm, the tongue enters
    its V slot, the two sprung brass rods enter its funnels; then it leans back DOCK_LEAN onto the face, the floor wave
    in front of the wheels holds it there and the motors go off. 'foot' = floor plate + lower column, 'head' = shelf +
    face slab + upper column up to DOCK_SPLIT (both print standing, no support), 'tower' = both together (viewer, sim),
    'mast' = top of the column (plugs into the head), 'wing' =
    one side wing (print two, the second mirrored)."""
    ft, fw, fz0, fz1 = DOCK_FACE
    x_face = X_BACK - COVER_T - CONTACT_T - DOCK_GAP           # dock face (robot frame, at the docked pose)
    if part == "tower":                                         # viewer / sim: the two printed pieces together
        return build_dock("foot").union(build_dock("head"))
    if part == "ridge":
        return dock_wave_strip(1)
    if part == "ridge_insert":
        return dock_wave_strip(1, part="insert")
    if part == "wing":
        wi, wo, wl, wt, (wz0, wz1) = DOCK_WING
        fw = DOCK_FACE[1]
        prof = [(x_face - ft, fw), (x_face, fw), (x_face, wi), (x_face + wl, wo), (x_face + wl, wo + wt), (x_face, wi + wt), (x_face - ft, wi + wt)]
        w = cq.Workplane("XY", origin=(0, 0, wz0)).polyline(prof).close().extrude(wz1 - wz0)
        for z in (wz0 + 12, wz1 - 12):                         # 2 x M3 into the face slab's side inserts
            w = w.cut(cyl_y(fw - 1, wi + wt - fw + 2, x_face - ft / 2, z, 3.4))
        return docked_transform(w)
    tt, tw, tz1 = DOCK_TOWER
    wt, wf = DOCK_WALL, DOCK_WALL_FRONT
    xb = x_face - ft + DOCK_SLAB_JOIN                           # column front (upper part, behind the slab)
    zs = fz0 - DOCK_SHELF_H                                     # step: shelf under the face slab = the foot / head joint
    # the slab reaches down to the shelf's bottom: the head prints standing on that plane and the slab's sides
    # (wider than the column) grow from the bed instead of hanging. Below the contact zone, so the robot never touches it.
    face = box(x_face - ft, x_face, -fw, fw, zs, fz1).edges("|X").fillet(DOCK_FILLET[0])
    xlb, xlf = xb - tt - DOCK_STEP, xb - DOCK_STEP               # lower column back / front faces
    col = box(xb - tt, xb, -tw, tw, zs, tz1).edges("|X").fillet(DOCK_FILLET[1])
    col = col.cut(box(xb - tt + wt, xb - wf, -tw + wt, tw - wt, zs + DOCK_SHELF_H, tz1 - wt))   # hollow (wire channel inside)
    low = box(xlb, xlf, -tw, tw, DOCK_FOOT_Z, zs + DOCK_SLAB_JOIN).edges("|X").fillet(DOCK_FILLET[1])
    low = low.cut(box(xlb + wt, xlf - wt, -tw + wt, tw - wt, DOCK_FOOT_Z, zs + 1))      # open at the top: no roof to bridge
    shelf = box(xlb, xb, -tw, tw, zs, zs + DOCK_SHELF_H)
    # wire passage: down the upper column, along inside the shelf, down the lower column to the DC jack
    col = col.union(low).union(shelf)
    # (cut after the union: the upper column's solid floor used to fill the passage again)
    ph, pin, pfr, pz0, pz1, pzt = DOCK_PASS
    col = col.cut(box(xlb + pin, xb - pfr, -ph, ph, zs + pz0, zs + pz1))
    col = col.cut(box(xb - tt + pin, xb - pfr, -ph, ph, zs + pz0, zs + pzt))
    col = col.cut(box(xlb + pin, xlf - pin, -ph, ph, zs - pz0, zs + pz1))   # through into the lower column
    gd, go = DOCK_TAPE_GUIDE
    for (ya, yb_) in DOCK_TAPE:                                   # guides for the retro-reflective lidar target
        col = col.cut(box(xb - gd, xb + go, ya, yb_, DOCK_TAPE_Z[0], DOCK_TAPE_Z[1]))
    t0, t1, th = DOCK_TONGUE                                    # wedge into the plate's slot
    ti = DOCK_TONGUE_INSET
    tongue = cq.Workplane("XY", origin=(0, 0, fz0 + ti)).polyline([(x_face - 0.01, -t0 / 2), (x_face - 0.01, t0 / 2), (x_face + th, t1 / 2), (x_face + th, -t1 / 2)]).close().extrude(fz1 - fz0 - 2 * ti)
    tz0 = fz0 + ti                                              # 45° underside: the tongue grows out of the face, no overhang
    tongue = tongue.cut(cq.Workplane("XZ", origin=(0, t0, 0)).polyline([(x_face, tz0 - 0.01), (x_face + th + 1, tz0 - 0.01), (x_face + th + 1, tz0 + th + 1)]).close().extrude(2 * t0))
    d = face.union(col).union(tongue)
    # IR beacon (in the mast, above the lidar target): 4 'all round' LEDs, one per column side, and the 'centre line'
    # LED at the end of a tube on the front (beam about ±6°: the robot knows when it is on the dock's axis)
    zl = DOCK_IR_Z
    ld, ldep, loff, lboss = DOCK_IR_LED
    d = d.cut(cyl_x(xb - wf, ldep, loff, zl, ld)).cut(cyl_x(xb - tt - 1, ldep, loff, zl, ld))
    d = d.cut(cyl_y(tw - wf, ldep, xb - tt / 2, zl, ld)).cut(cyl_y(-tw - 1, ldep, xb - tt / 2, zl, ld))
    d = d.union(cyl_x(xb - DOCK_SLAB_JOIN, DOCK_IR_TUBE + DOCK_SLAB_JOIN, -loff, zl, lboss)).cut(cyl_x(xb - wf, DOCK_IR_TUBE + ldep, -loff, zl, ld))
    if part in ("tape", "ir_leds", "board", "rods", "jack", "wires_pos", "wires_neg", "wires_led"):   # viewer only
        rd, rfree, rlen = DOCK_ROD
        xc = xb - tt / 2                                          # column centre line (upper part)
        xl = xc - DOCK_STEP                                       # lower column centre line
        bl_, bw_, bt_, bx0_ = DOCK_BASE
        lean = math.radians(DOCK_LEAN)
        hgt = WHEEL_R + DOCK_FLOOR
        fwd = lambda p: (p[0] * math.cos(lean) - p[2] * math.sin(lean), p[1], p[0] * math.sin(lean) + p[2] * math.cos(lean) + hgt)
        def col_at_floor(x_axis, y, zw):                          # point on a column axis (docked frame x) at world height zw
            zd = (zw - hgt - x_axis * math.sin(lean)) / math.cos(lean)
            return fwd((x_axis, y, zd))
        jack_w = (bx0_ - bl_ + 6.0, 0.0, 16.0)                   # DC jack in the base's back edge (world)
        def tube(pts, d=2.4):                                     # a wire along a polyline (cylinders + ball joints)
            u = None
            for a, b in zip(pts[:-1], pts[1:]):
                v = cq.Vector(*b) - cq.Vector(*a)
                if v.Length < 0.1: continue
                seg = cq.Solid.makeCylinder(d / 2, v.Length, cq.Vector(*a), v.normalized())
                u = seg if u is None else u.fuse(seg)
                u = u.fuse(cq.Solid.makeSphere(d / 2, cq.Vector(*b)))
            return cq.Workplane().add(u)
        bz0 = zl - 46                                             # board bottom edge
        if part in ("wires_pos", "wires_neg"):                    # charger jack -> board -> rod (+ = the y -25 rod)
            ys = 4.0 if part == "wires_pos" else -4.0
            yr = PADS["+"][0] if part == "wires_pos" else PADS["-"][0]
            up = [(xc + 3.8, ys, bz0 + 2), (xc + 1, ys, bz0 - 4), (xc + 1, ys, zs + 6), (xl + 1, ys, zs + 6)]
            w_up = [fwd(p) for p in up] + [col_at_floor(xl + 1, ys, 10.0)]
            w_jack = [(jack_w[0] + 14, ys, 10.0), (jack_w[0] + 13, ys / 2, jack_w[2])]
            zr = PADS["+"][1]
            rod = [(xc - 2, ys * 2, bz0 + 2), (xc - 2, ys * 2, zr), (x_face - ft - 10, ys * 2, zr), (x_face - ft - 10, yr, zr), (x_face - ft - 14, yr, zr)]
            return tube(w_up + w_jack).union(docked_transform(tube(rod)))
        if part == "wires_led":                                   # board -> the 5 LEDs (short pairs up the mast)
            leds = [(xb - 4, 20.0, zl), (xb - tt + 4, 20.0, zl), (xc, tw - 4, zl), (xc, -tw + 4, zl), (xb - 1, -20.0, zl)]
            u = None
            for (x, y, z) in leds:
                t = tube([(xc + 3.8, y * 0.4, zl - 17), (xc + 1, y * 0.6, zl - 6), (x, y, z)], d=1.6)
                u = t if u is None else u.union(t)
            return docked_transform(u)
        if part == "rods":                                        # 2 sprung brass rods with their collars
            ex = [cyl_x(x_face + rfree - rlen, rlen, y, z, rd) for (y, z) in PADS.values()] + \
                 [cyl_x(x_face - ft - 6.5, 1.0, y, z, 5.5) for (y, z) in PADS.values()]
        elif part == "jack":                                      # 5.5/2.1 DC jack in the base's back edge
            j = cq.Workplane("YZ", origin=(bx0_ - bl_ - 2, 0, 0)).center(0, jack_w[2]).circle(5.5).extrude(3)
            return j.union(cq.Workplane("YZ", origin=(bx0_ - bl_ + 1, 0, 0)).center(0, jack_w[2]).circle(4.0).extrude(12))
        elif part == "board":                                     # 40 x 30 perfboard standing in the mast's hollow
            xc = xb - tt / 2
            ex = [box(xc + 3.0, xc + 4.6, -20, 20, zl - 46, zl - 16),          # board
                  box(xc - 6.0, xc + 3.0, -14, 14, zl - 42, zl - 20)]          # parts (TO-220s bent flat, ATtiny, buck)
        elif part == "tape":
            ex = [box(xb - 0.3, xb - 0.05, ya, yb_, DOCK_TAPE_Z[0], DOCK_TAPE_Z[1]) for (ya, yb_) in DOCK_TAPE]
        else:
            ex = [cyl_x(xb - 3, 6.5, 20.0, zl, 5.0), cyl_x(xb - tt - 3.5, 6.5, 20.0, zl, 5.0),
                  cyl_y(tw - 3, 6.5, xb - tt / 2, zl, 5.0), cyl_y(-tw - 3.5, 6.5, xb - tt / 2, zl, 5.0),
                  cyl_x(xb + 2, 8.0, -20.0, zl, 5.0)]
        u = ex[0]
        for e in ex[1:]: u = u.union(e)
        return docked_transform(u)
    rd, rfree, rlen = DOCK_ROD
    for (y, z) in PADS.values():                                # rod bores (sprung rods, wire soldered at the back end)
        d = d.union(cyl_x(x_face - ft - 14, 14.0, y, z, 11.0))   # boss behind the face for the rod + spring
        g = DOCK_BOSS_GUSSET                                    # 45° wedge under it, down to the column's front wall
        d = d.union(cq.Workplane("XZ", origin=(0, y + 4.0, 0)).polyline([(xb - 0.5, z - 3.0), (x_face - ft - 14, z - 3.0), (xb - 0.5, z - 3.0 - (xb - 0.5 - (x_face - ft - 14)))]).close().extrude(8.0))
        d = d.cut(cyl_x(x_face - ft - 15, ft + 16, y, z, POGO_D))
        d = d.cut(cyl_x(x_face - ft - 15, 12.0, y, z, 6.6))     # spring pocket (Ø5 spring), open at the back for the wire
    for z in (DOCK_WING[4][0] + 12, DOCK_WING[4][1] - 12):    # side inserts (M3) for the wings
        d = d.cut(cyl_y(fw - INS_M3[1], INS_M3[1] + 1, x_face - ft / 2, z, INS_M3[0]))
        d = d.cut(cyl_y(-fw - 1, INS_M3[1] + 1, x_face - ft / 2, z, INS_M3[0]))
    # wire channel from the rods down the column, DC jack at the bottom back
    d = d.cut(box(x_face - ft - 14, x_face - ft - 6, -28, 28, 105, 117))
    d = d.cut(box(x_face - ft - 14, x_face - ft - 6, -4, 4, fz0, 117))
    # tongue: 45° chamfer under it so the dock prints standing up without support
    w = docked_transform(d)
    bl, bw, bt, bx0 = DOCK_BASE
    w = w.intersect(cq.Workplane("XY").add(cq.Solid.makeBox(600, 600, 400, cq.Vector(-300, -300, bt - 0.01))))
    base = box(bx0 - bl, bx0, -bw / 2, bw / 2, 0, bt).edges("|Z").fillet(10)
    for yy in (-50, 50):                                       # lightening windows (the base only has to resist tipping)
        base = base.cut(box(bx0 - bl + 15, bx0 - 15, yy - 30, yy + 30, -1, bt + 1).edges("|Z").fillet(8))
    w = w.union(base)
    for sgn in (1, -1):                                         # slots for the two floor ridges (separate prints)
        w = w.cut(dock_wave_strip(sgn, clearance=0.25))
    w = w.cut(cq.Workplane("YZ", origin=(bx0 - bl - 1, 0, 0)).center(0, 16).circle(4.0).extrude(30))   # 5.5/2.1 DC jack, back edge
    xlb = xb - tt - DOCK_STEP                                       # lower column back face (docked frame) -> world at z 10
    zd = (10.0 - WHEEL_R - DOCK_FLOOR - xlb * math.sin(math.radians(DOCK_LEAN))) / math.cos(math.radians(DOCK_LEAN))
    xw = xlb * math.cos(math.radians(DOCK_LEAN)) - zd * math.sin(math.radians(DOCK_LEAN))
    w = w.cut(box(xw - 6, xw + 6, -8, 8, 6.0, 15.0))                 # wire exit to the DC jack
    # two prints, both standing, no overhang: 'foot' = floor plate + lower column (up to the shelf), 'head' = shelf +
    # face slab + upper column up to DOCK_SPLIT. Joint: a ring on the foot's column top in a groove in the shelf, glued,
    # + 2 M3x16 from the shelf's top (behind the upper column) into inserts in two columns inside the foot's back wall.
    up = lambda z0, z1: docked_transform(box(-300, 300, -300, 300, z0, z1))   # slab of the docked frame (tilted 8° in the world)
    si, sr, sh, sc = DOCK_JOINT_SPIGOT
    def ring(o, i, z0, z1):                                     # rectangular ring o mm inside the outside, i mm past the wall's inside
        r = box(xlb + o, xlf - o, -tw + o, tw - o, z0, z1)
        return r.cut(box(xlb + wt + i, xlf - wt - i, -tw + wt + i, tw - wt - i, z0 - 1, z1 + 1))
    by, brad = DOCK_JOINT_BOSS
    bx = xlb + wt + brad                                      # merges into the back wall, clear of the ring
    if part in ("foot", "head"):
        if part == "foot":
            f = w.intersect(up(-300, zs))
            ext = ring(si, sr, zs - 1, zs + sh)
            for yy in (-by, by):
                ext = ext.union(cq.Workplane("XY", origin=(0, 0, DOCK_FOOT_Z)).center(bx, yy).circle(brad).extrude(zs - DOCK_FOOT_Z))
            ext = docked_transform(ext).intersect(cq.Workplane("XY").add(cq.Solid.makeBox(600, 600, 400, cq.Vector(-300, -300, 0.5))))
            f = f.union(ext)
            for yy in (-by, by):
                f = f.cut(docked_transform(cq.Workplane("XY", origin=(0, 0, zs - INS_M3[1] - 1)).center(bx, yy).circle(INS_M3[0] / 2).extrude(INS_M3[1] + 1.5)))
            return f
        hd = w.intersect(up(zs, 300)).intersect(cq.Workplane("XY").add(cq.Solid.makeBox(600, 600, DOCK_SPLIT, cq.Vector(-300, -300, 0))))
        hd = hd.cut(docked_transform(ring(si - sc, sr + sc, zs - 1, zs + sh + sc)))
        for yy in (-by, by):
            hd = hd.cut(docked_transform(cq.Workplane("XY", origin=(0, 0, zs - 1)).center(bx, yy).circle(1.7).extrude(DOCK_SHELF_H + 2)))
        return hd
    # the mast (above DOCK_SPLIT, so the head fits a 220 mm printer) plugs into the head with a hollow spigot
    big = 600.0
    mast = w.intersect(cq.Workplane("XY").add(cq.Solid.makeBox(big, big, big, cq.Vector(-big / 2, -big / 2, DOCK_SPLIT))))
    xc = xb - tt / 2                                               # spigot along the (leaning) column axis
    zr = (DOCK_SPLIT - WHEEL_R - xc * math.sin(math.radians(DOCK_LEAN))) / math.cos(math.radians(DOCK_LEAN))
    c = 0.3                                                        # clearance in the tower's hollow
    sp = box(xb - tt + 2.4 + c, xb - 2.9 - c, -tw + 2.4 + c, tw - 2.4 - c, zr - 22, zr + 6)
    sp = sp.cut(box(xb - tt + 4.4 + c, xb - 4.9 - c, -tw + 4.4 + c, tw - 4.4 - c, zr - 23, zr + 7))
    sp = docked_transform(sp).intersect(cq.Workplane("XY").add(cq.Solid.makeBox(big, big, DOCK_SPLIT + 3, cq.Vector(-big / 2, -big / 2, 0))))
    root = box(xb - tt + 2.0, xb - 2.5, -tw + 2.0, tw - 2.0, zr - 2, zr + 10)        # fused into the mast's walls
    root = root.cut(box(xb - tt + 4.4 + c, xb - 4.9 - c, -tw + 4.4 + c, tw - 4.4 - c, zr - 3, zr + 11))
    root = docked_transform(root).intersect(cq.Workplane("XY").add(cq.Solid.makeBox(big, big, big, cq.Vector(-big / 2, -big / 2, DOCK_SPLIT))))
    return mast.union(root).union(sp)


def build_shelf():
    """Battery shelf. Slides in from the back flat on the motor-wall tops, then 4 x M3x10 countersunk screws
    go into the wall-top inserts: the shelf ties the two walls together (FEA: the walls splay without it).
    Captured front/back by the front panel and the back cover. Print it right side up."""
    x0, x1 = X_BACK + 0.4, X_IN_F - 0.4
    shelf = box(x0, x1, -(WELL_Y0 - 0.5), WELL_Y0 - 0.5, SHELF_Z, SHELF_Z + SHELF_T)
    for xs in STRAP_XS:
        shelf = shelf.cut(box(xs - STRAP_SLOT / 2, xs + STRAP_SLOT / 2, -STRAP_W / 2, STRAP_W / 2, SHELF_Z - 1, SHELF_Z + SHELF_T + 1))
    nx, ny = SHELF_FRONT_NOTCH
    for sg in (1, -1):
        shelf = shelf.cut(box(nx, x1 + 1, min(sg * ny, sg * WELL_Y0), max(sg * ny, sg * WELL_Y0), SHELF_Z - 1, SHELF_Z + SHELF_T + 1))
    cx0, cx1, cy0, cy1 = CABLE_HOLE
    shelf = shelf.cut(box(cx0, cx1, cy0, cy1, SHELF_Z - 1, SHELF_Z + SHELF_T + 1))
    # 2 M3 insert bosses on top for the lidar-adapter carrier plate
    for (x, y) in SHELF_PLATE_PTS:
        shelf = shelf.union(cq.Workplane("XY", origin=(0, 0, SHELF_Z + SHELF_T - 0.1)).center(x, y).circle(INS_M3[2] / 2).extrude(4.1))
        shelf = shelf.cut(cq.Workplane("XY", origin=(0, 0, SHELF_Z + SHELF_T + 4 - INS_M3[1])).center(x, y)
                          .circle(INS_M3[0] / 2).extrude(INS_M3[1] + 0.1))
    # 2 countersunk M3 holes per rail (screws into the wall-top inserts): the shelf ties the two walls together
    for s_ in (1, -1):
        yc = s_ * (GAP_HALF + MWALL_T / 2)
        for xs in SHELF_SCREW_X:
            shelf = shelf.cut(cq.Workplane("XY", origin=(0, 0, SHELF_Z - 1)).center(xs, yc).circle(1.7).extrude(SHELF_T + 2))
            shelf = shelf.cut(cq.Solid.makeCone(1.7, 3.3, 1.6, cq.Vector(xs, yc, SHELF_Z + SHELF_T - 1.6)))
            shelf = shelf.cut(cq.Workplane("XY", origin=(0, 0, SHELF_Z + SHELF_T - 0.01)).center(xs, yc).circle(3.3).extrude(1))
    # finger notch at the back edge to pull it out
    shelf = shelf.cut(box(x0 - 1, x0 + 6, -15, 15, SHELF_Z + SHELF_T - 2, SHELF_Z + SHELF_T + 1))
    return shelf


def build_tray():
    """Removable electronics tray in the centre gap (between the motor walls), on 2 floor inserts.
    Carries: IMU (GY-521, 2 x M2.5 inserts), 5 V regulator (Pololu D24V50F5, 2 x M2 inserts),
    power distribution + fuse holder under a clamp bar (2 x M3 inserts) and zip-tie slots.
    Print flat, posts up. Local frame = robot frame (tray sits on the floor bosses)."""
    z0 = Z_FLOOR + FLOOR_BOSS_H
    t = box(-28.0, 28.0, -16.0, 16.0, z0, z0 + TRAY_T).edges("|Z").fillet(3)
    for (x, y) in FLOOR_TRAY_PTS:
        t = t.cut(cq.Workplane("XY", origin=(0, 0, z0 - 1)).center(x, y).circle(1.7).extrude(TRAY_T + 2))
    def post(x, y, ins, h):
        nonlocal t
        t = t.union(cq.Workplane("XY", origin=(0, 0, z0 + TRAY_T - 0.1)).center(x, y).circle(ins[2] / 2).extrude(h + 0.1))
        t = t.cut(cq.Workplane("XY", origin=(0, 0, z0 + TRAY_T + h - ins[1])).center(x, y).circle(ins[0] / 2).extrude(ins[1] + 0.1))
    for (x, y) in IMU_HOLES_ON_TRAY:
        post(x, y, INS_M2, 4.0)        # GY-521: M2 screws + washers (its holes are ~3 mm)
    for (x, y) in REG_HOLES_ON_TRAY:
        post(x, y, INS_M2, 4.0)
    for (x, y) in CLAMP_POSTS_ON_TRAY:
        post(x, y, INS_M3, 10.0)
    for x in (-24.0, -12.0):                       # zip-tie slots for the fuse holder / wires
        for y in (-13.0, 13.0):
            t = t.cut(box(x - 1.5, x + 1.5, y - 2.0, y + 2.0, z0 - 1, z0 + TRAY_T + 1))
    return t


IMU_HOLES_ON_TRAY = [(-7.5, -7.75), (-7.5, 7.75)]          # GY-521: 2 holes 15.5 mm apart on one long edge
REG_HOLES_ON_TRAY = [(16.5 - 8.0, 6.7), (16.5 + 8.0, -6.7)]  # D24V50F5 (Pololu STEP): diagonal holes 16.0 x 13.4 mm
CLAMP_POSTS_ON_TRAY = [(-17.0, -11.0), (-17.0, 11.0)]


def build_clamp_bar(length=30.0, pitch=22.0):
    """Clamp bar: presses a board without mounting holes (PDB, BMS...) onto foam tape. 2 x M3."""
    bar = box(-length / 2, length / 2, -4, 4, 0, 3).edges("|Z").fillet(2)
    for x in (-pitch / 2, pitch / 2):
        bar = bar.cut(cq.Workplane("XY", origin=(0, 0, -1)).center(x, 0).circle(1.7).extrude(5))
    return bar


def build_carrier_plate(w=32.0, h_lo=20.0, h_hi=30.0, pitch=22.0):
    """Universal carrier plate for modules whose holes vary by seller (CC/CV charger, lidar USB adapter). 2 x M3 to the
    body's inserts (pitch 22 mm, on the plate's y = 0 line); module screws through the 6 mm grid of 3.2 mm holes
    (M3 / M2.5 screws + nuts) or zip ties. 20 mm below the screw line (on the left wall that keeps it above the
    side USB-C socket), 30 mm above. Print flat."""
    p = box(-w / 2, w / 2, -h_lo, h_hi, 0, 3).edges("|Z").fillet(3)
    grid = [(x, y) for x in [i * 6.0 for i in range(-2, 3)] for y in [j * 6.0 for j in range(-4, 5)] if abs(y) > 4 and -h_lo + 3 < y < h_hi - 3]
    p = p.cut(cq.Workplane("XY", origin=(0, 0, -1)).pushPoints(grid).circle(1.6).extrude(5))
    p = p.cut(cq.Workplane("XY", origin=(0, 0, -1)).pushPoints([(-pitch / 2, 0), (pitch / 2, 0)]).circle(1.7).extrude(5))
    for x in (-w / 2 + 4, w / 2 - 4):              # zip-tie slots
        p = p.cut(box(x - 1.5, x + 1.5, -h_lo + 3, -h_lo + 7, -1, 4))
        p = p.cut(box(x - 1.5, x + 1.5, h_hi - 7, h_hi - 3, -1, 4))
    return p


def build_speaker_ring():
    """Clamps the speaker flange against the right wall (3 x M2x8 into the wall bosses); the tab on top carries the
    MAX98357A amp (zip ties). Print flat."""
    sx, sz = SPK_XZ
    y1 = SPK_Y0 + SPK_FLANGE                      # ring face on the back of the speaker flange
    r = (cq.Workplane("XZ", origin=(0, y1, 0)).center(sx, sz).circle(SPK_D / 2 + 1.5).circle(SPK_MAG_D / 2 + 2.0).extrude(-RING_T))
    for (bx, bz) in SPK_BOSSES:
        r = r.union(cq.Workplane("XZ", origin=(0, y1, 0)).center(bx, bz).circle(INS_M2[2] / 2 + 0.8).extrude(-RING_T))
        r = r.cut(cyl_y(y1 - 1, RING_T + 2, bx, bz, 2.4))
    x0, x1, z0, z1 = AMP_TAB
    tab = box(x0, x1, y1, y1 + RING_T, z0, z1).edges("|Y").fillet(2.0)
    for (zz, xx) in ((z0 + 4, x0 + 3), (z0 + 4, x1 - 3), (z1 - 3, x0 + 3), (z1 - 3, x1 - 3)):
        tab = tab.cut(box(xx - 1.5, xx + 1.5, y1 - 1, y1 + RING_T + 1, zz - 2, zz + 2))   # zip-tie slots
    return r.union(tab)


def build_mic_clamp(side=1):
    """Holds one ICS-43434 board in its front-panel pocket (port on the hole, 1 mm foam gasket). 1 x M2x6. Print flat."""
    my, mz = side * MIC_YZ[0], MIC_YZ[1]
    pc = my - MIC_CHIP_DY                         # board centre (the port is on MIC_YZ)
    xs = X_IN_F - 5.0 - 2.0
    by, bz = my, mz + MIC_BOSS_DZ
    plate = box(xs, xs + 2.0, pc - MIC_PCB[0] / 2 - 1, pc + MIC_PCB[0] / 2 + 1, mz - MIC_PCB[1] / 2 - 1, bz + 3.4).edges("|X").fillet(2.5)
    pcb_back = X_IN_F + MIC_POCKET - 1.5            # Adafruit 6049 PCB is 1.5 mm
    rib = (cq.Workplane("YZ", origin=(xs + 2.0, 0, 0)).center(pc, mz).rect(MIC_PCB[0] - 0.6, MIC_PCB[1] - 0.6)
           .rect(MIC_PCB[0] - 4.0, MIC_PCB[1] - 4.0).extrude(pcb_back - xs - 2.0))
    plate = plate.union(rib).cut(cyl_x(xs - 1, 4, by, bz, 2.4))
    # wire pass over the 6 solder pads (Adafruit 6049: a row along z at 4.61 mm from the mic port)
    return plate.cut(cq.Workplane("YZ", origin=(xs - 1, 0, 0)).center(my - MIC_PAD_DY, mz - 0.63).rect(5, 15.5).extrude(4))


def build_ir_hood():
    """Hood for one TSOP38238 IR receiver, in its local frame (looking along +x, base on the top skin at z = 0).
    The receiver stands in the pocket with its dome to the 90°-wide window; leads down through the floor hole and the
    body's top-skin hole; glued down (CA or epoxy) at IR_RX, turned to its facing. The window's roof keeps sunlight and
    the room's ceiling lights off it."""
    hx, hy, hz = IR_HOOD
    h = box(-hx / 2, hx / 2, -hy / 2, hy / 2, 0, hz).edges("|Z").fillet(IR_HOOD_FILLET[0]).edges(">Z").fillet(IR_HOOD_FILLET[1])
    px0, px1, pw, pz0, pz1 = IR_TSOP_POCKET
    h = h.cut(box(px0, px1, -pw, pw, pz0, pz1))                # receiver body
    wz, wh, wa = IR_WINDOW
    win = cq.Workplane("XY", origin=(0, 0, wz)).polyline([(wa, 0), (hx, -hx + wa), (hx, hx - wa)]).close().extrude(wh)
    h = h.cut(win)                                             # 90° window: the dome sits behind it
    lx, lr, ldp = IR_HOOD_LEADS
    h = h.cut(cq.Workplane("XY", origin=(0, 0, -0.1)).center(lx, 0).circle(lr).extrude(ldp))   # leads
    return h


def ir_hoods_placed():
    """The 4 hoods where they sit on the robot (for the assembly / fit checks)."""
    hs = [build_ir_hood().val().rotate((0, 0, 0), (0, 0, 1), f).translate((x, y, Z_TOP)) for (x, y, f) in IR_RX]
    return cq.Workplane().add(cq.Compound.makeCompound(hs))


def build_diffuser():
    d = EYE_PCB - 0.3
    return cq.Workplane("YZ").rect(d, d).extrude(DIFFUSER_T).edges("|X").fillet(1.0)


if __name__ == "__main__":
    out = pathlib.Path(__file__).parent
    parts = {"body": build_body(), "shelf": build_shelf(), "front": build_front(), "cover": build_cover(),
             "contact_plate": build_contact_plate(), "dock_foot": build_dock("foot"), "dock_head": build_dock("head"), "dock_mast": build_dock("mast"), "dock_wing": build_dock("wing"), "dock_ridge": build_dock("ridge"), "dock_ridge_insert": build_dock("ridge_insert"), "eye_diffuser": build_diffuser(), "ir_hood": build_ir_hood(),
             "tray": build_tray(), "eye_retainer": build_eye_retainer(1), "clamp_bar": build_clamp_bar(),
             "carrier_plate": build_carrier_plate(), "speaker_ring": build_speaker_ring(), "mic_clamp": build_mic_clamp(1), "light_retainer": build_light_retainer(1)}
    for name, part in parts.items():
        cq.exporters.export(part, str(out / f"robot_{name}.step"))
        cq.exporters.export(part, str(out / f"robot_{name}.stl"), tolerance=0.03, angularTolerance=0.1)
        v = part.val().Volume() / 1000
        bb = part.val().BoundingBox()
        print(f"{name}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, {v:.0f} cm3 -> ~{v*1.27*0.75:.0f} g PETG (est.)")
    print(f"motor back face y=+-{MOTOR_BACK_Y}, wheel {WHEEL_IN_Y}..{WHEEL_OUT_Y}, body side +-{Y_OUT} "
          f"-> {WHEEL_OUT_Y - Y_OUT:.0f} mm of {WHEEL_W:.0f} sticks out")
