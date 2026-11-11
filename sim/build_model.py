"""Builds robot_c.xml: the shape-C robot for MuJoCo, from the CAD assembly.

Printed parts are the real STL meshes (mass from volume x PETG density at typical infill, inertia from the mesh).
Off-the-shelf parts are boxes with their real mass at their CAD position. Units: m, kg.
Robot frame: X forward, Y left, Z up, origin on the wheel axle. The floor is at z = -WHEEL_R in robot frame.

  python3 build_model.py [--arena]   # --arena adds a room with obstacles, a 360-ray lidar and a sonar
  python3 build_model.py --stairs [--rise 0.10] [--tread 0.30] [--steps 6] [--edge 1.2]
        # a landing that ENDS at --edge metres, then a staircase descending in +X, plus the
        # lidar fan and sonar. Riser and tread are parameters on purpose: whether the robot can
        # see a step and whether it can survive one are different questions with different answers,
        # so both get measured across a range instead of being fixed here.
"""
import json, sys, pathlib, math

HERE = pathlib.Path(__file__).parent
CAD = json.load(open(HERE / "cad_mass.json"))
R = CAD["R"] / 1000.0
WY0, WY1 = (y / 1000.0 for y in CAD["wheel_y"])
WHEEL_W = WY1 - WY0
WHEEL_Y = (WY0 + WY1) / 2
MOTOR_STATOR = 0.150     # kg each, stays with the body (stator + driver board)
MOTOR_ROTOR = 0.101      # kg each, turns with the wheel (MF6015 251 g total)
WHEEL_M = 0.110 + 0.0225 + MOTOR_ROTOR   # tyre/rim + printed adapter & screws + rotor
WIRES = 0.040
LIDAR_RAYS = 120         # simulated LD19: one ray every 3 deg (the real one gives ~450 points/rev)

COLLIDE = {"body", "front", "cover", "cap"}


def comp_geoms():
    out = []
    for name, c in CAD["comps"].items():
        lo, hi = c["lo"], c["hi"]
        pos = [(a + b) / 2000 for a, b in zip(lo, hi)]
        size = [max((b - a) / 2000, 0.0005) for a, b in zip(lo, hi)]
        col = name.startswith("lidar LD19")
        tag = "lidar" if col else "c%d" % len(out)
        out.append(f'<geom name="{tag}" type="box" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" '
                   f'size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}" mass="{c["mass"]/1000:.4f}" '
                   f'contype="{1 if col else 0}" conaffinity="{1 if col else 0}" rgba="0.75 0.45 0.2 1" group="1"/>')
    return out


def xml(arena=False, stairs=None):
    meshes = "".join(f'<mesh name="m_{k}" file="meshes/{k}.stl" scale="0.001 0.001 0.001" inertia="{'shell' if k == 'body' else 'exact'}"/>' for k in CAD["printed"])
    printed = []
    for k, g in CAD["printed"].items():
        c = 1 if k in COLLIDE else 0
        rgba = "0.70 0.86 0.78 1" if k in ("body", "cover") else ("0.98 0.97 0.94 1" if k in ("front", "cap") else "0.9 0.6 0.2 1")
        printed.append(f'<geom name="{k}" type="mesh" mesh="m_{k}" mass="{g/1000:.4f}" contype="{c}" conaffinity="{c}" rgba="{rgba}"/>')
    stators = "".join(
        f'<geom name="stator{s}" type="cylinder" size="0.026 0.017" pos="0 {s*0.039:.3f} 0" zaxis="0 1 0" mass="{MOTOR_STATOR}" '
        f'contype="0" conaffinity="0" rgba="0.6 0.6 0.65 1"/>' for s in ("+1", "-1") for s in [int(s)])
    wires = f'<geom name="wires" type="box" size="0.02 0.03 0.01" pos="0 0 0.03" mass="{WIRES}" contype="0" conaffinity="0" group="1"/>'
    wheels = ""
    for side, s in (("left", 1), ("right", -1)):
        wheels += f'''
      <body name="wheel_{side}" pos="0 {s*WHEEL_Y:.4f} 0">
        <!-- contact: ellipsoid = rounded foam tyre touching at one point (a flat MuJoCo cylinder scrubs at both rim edges) -->
        <joint name="{side}_hinge" type="hinge" axis="0 1 0" damping="0.0005" armature="0.0000515"/>
        <geom name="tyre_{side}" type="ellipsoid" size="{R:.4f} {WHEEL_W/2:.4f} {R:.4f}" mass="{WHEEL_M:.4f}" rgba="0.12 0.12 0.13 1"
              friction="1.0 0.01 0.0001" condim="4"/>
        <geom type="cylinder" size="{R:.4f} {WHEEL_W/2:.4f}" zaxis="0 1 0" mass="0" contype="0" conaffinity="0" rgba="0.12 0.12 0.13 1"/>
        <geom type="box" size="{R*0.6:.4f} {WHEEL_W/2+0.0005:.4f} 0.004" contype="0" conaffinity="0" mass="0" rgba="0.85 0.85 0.8 1"/>
      </body>'''
    rays = ""
    sensors_extra = ""
    sense = arena or stairs is not None       # the ray fan and sonar: needed by both
    if sense:
        lid_z = 0.152 + 0.025          # body top (Z_TOP 152 mm above the axle) + LD19 scan plane 25 mm above its base
        for i in range(LIDAR_RAYS):
            a = 2 * math.pi * i / LIDAR_RAYS
            # rangefinder looks along the site's +z: point z along (cos a, sin a, 0)
            rays += f'<site name="ray{i}" pos="0 0 {lid_z}" zaxis="{math.cos(a):.5f} {math.sin(a):.5f} 0" size="0.002"/>'
            sensors_extra += f'<rangefinder name="lidar{i}" site="ray{i}" cutoff="12"/>'
        rays += '<site name="sonar" pos="0.032 0 0.080" zaxis="1 0 0" size="0.003"/>'
        sensors_extra += '<rangefinder name="sonar" site="sonar" cutoff="4"/>'
    world = ""
    if arena:
        walls = [((0, 4.0), (8.2, 0.1)), ((0, -4.0), (8.2, 0.1)), ((4.0, 0), (0.1, 8.2)), ((-4.0, 0), (0.1, 8.2)),
                 ((1.2, 0.6), (0.6, 0.4)), ((-1.5, 1.8), (1.0, 0.3)), ((2.2, -2.0), (0.5, 0.5)),
                 ((-2.0, -1.5), (0.4, 1.2)), ((0.0, 2.7), (2.0, 0.2)), ((2.6, 2.2), (0.3, 0.3)), ((-0.6, -2.6), (0.3, 0.3))]
        for i, ((x, y), (sx, sy)) in enumerate(walls):
            world += f'<geom name="obst{i}" type="box" pos="{x} {y} {-R + 0.15:.4f}" size="{sx/2} {sy/2} 0.15" rgba="0.55 0.5 0.45 1"/>'
        # a low post (18 cm): below the lidar's scan plane (21 cm), only the sonar sees it
        world += f'<geom name="leg0" type="cylinder" pos="1.0 -1.0 {-R + 0.09:.4f}" size="0.03 0.09" rgba="0.3 0.2 0.1 1"/>'
    floor_z = -R
    # The default floor is an infinite plane, which cannot have an edge to fall off. A staircase
    # therefore replaces it with a finite landing, one box per tread, and a floor at the bottom.
    if stairs is None:
        floor = f'<geom name="floor" type="plane" pos="0 0 {floor_z:.4f}" size="10 10 0.1" material="grid"/>'
    else:
        rise, tread, nstep, edge = stairs["rise"], stairs["tread"], stairs["steps"], stairs["edge"]
        # How far the landing runs BEHIND the edge. It used to be 3 m, and that was a reward loophole
        # rather than a detail: an RL policy that had 5.5 s and 0.6 m/s to play with could reverse off
        # the back of the world, which the controller reports as a fall - and a fall (-5) was cheaper
        # than never looking at the staircase (-8), so it learned to drive backwards off the landing
        # and scored 0.00 while looking perfectly safe in the margin statistics. 8 m is more than any
        # episode can cross.
        back = stairs.get("back", 8.0)
        HW, TH = 0.60, 0.15                   # half-width of the treads, half-thickness of each slab
        g = [f'<geom name="landing" type="box" pos="{(edge - back) / 2:.4f} 0 {floor_z - TH:.4f}" '
             f'size="{(edge + back) / 2:.4f} {HW:.4f} {TH:.4f}" material="grid"/>']
        for i in range(nstep):
            top = floor_z - (i + 1) * rise
            g.append(f'<geom name="tread{i}" type="box" pos="{edge + (i + 0.5) * tread:.4f} 0 {top - TH:.4f}" '
                     f'size="{tread / 2:.4f} {HW:.4f} {TH:.4f}" material="grid"/>')
        bottom = floor_z - nstep * rise
        g.append(f'<geom name="bottom" type="box" pos="{edge + nstep * tread + 1.5:.4f} 0 {bottom - TH:.4f}" '
                 f'size="1.5 {HW:.4f} {TH:.4f}" material="grid"/>')
        # Stairwell walls, so the lidar has something to range against at head height like a real
        # staircase; they also stop the robot steering off the side of the steps.
        for s, y in (("l", HW), ("r", -HW)):
            g.append(f'<geom name="wall_{s}" type="box" pos="{edge + nstep * tread / 2:.4f} {y:+.4f} {floor_z + 0.5:.4f}" '
                     f'size="{(nstep * tread + 3.0) / 2:.4f} 0.05 0.9" rgba="0.55 0.5 0.45 1"/>')
        floor = "".join(g)
    return f'''<mujoco model="robot_c">
  <compiler meshdir="." autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <size memory="20M"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/></visual>
  <default><geom friction="0.9 0.005 0.0001" solref="0.008 1"/></default>
  <asset>
    {meshes}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.86 0.84" rgb2="0.74 0.74 0.72" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="16 16"/>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.92 0.95" rgb2="0.6 0.65 0.7" width="64" height="64"/>
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true" castshadow="false"/><light pos="2 -2 3" dir="-0.5 0.5 -1" directional="true" diffuse="0.3 0.3 0.3" castshadow="false"/>
    {floor}
    {world}
    <body name="chassis" pos="0 0 0">
      <freejoint name="root"/>
      {"".join(printed)}
      {"".join(comp_geoms())}
      {stators}
      {wires}
      <site name="imu" pos="-0.002 0 -0.0175"/>
      {rays}
      {wheels}
    </body>
  </worldbody>
  <actuator>
    <motor name="m_left" joint="left_hinge" ctrlrange="-3 3"/>
    <motor name="m_right" joint="right_hinge" ctrlrange="-3 3"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu" noise="0.005"/>
    <accelerometer name="accel" site="imu" noise="0.05"/>
    <jointpos name="ql" joint="left_hinge"/>
    <jointpos name="qr" joint="right_hinge"/>
    <jointvel name="wl" joint="left_hinge"/>
    <jointvel name="wr" joint="right_hinge"/>
    {sensors_extra}
  </sensor>
</mujoco>'''


def _opt(name, default):
    return type(default)(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default


if __name__ == "__main__":
    arena = "--arena" in sys.argv
    stairs = None
    if "--stairs" in sys.argv:
        stairs = dict(rise=_opt("--rise", 0.10), tread=_opt("--tread", 0.30),
                      steps=_opt("--steps", 6), edge=_opt("--edge", 1.2), back=_opt("--back", 8.0))
    default_name = "robot_c_stairs.xml" if stairs else ("robot_c_arena.xml" if arena else "robot_c.xml")
    out = HERE / _opt("--out", default_name)
    out.write_text(xml(arena, stairs))
    if stairs:
        print(f"{out.name}: landing ends at x={stairs['edge']} m, then {stairs['steps']} steps of "
              f"{stairs['rise']*1000:.0f} mm rise x {stairs['tread']*1000:.0f} mm tread "
              f"({stairs['rise']/R:.1f} wheel radii per step)")
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(out))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ch = m.body("chassis").id
    tot = sum(m.body_mass)
    com = d.subtree_com[ch]
    print(f"{out.name}: total {tot*1000:.0f} g, chassis {m.body_mass[ch]*1000:.0f} g, robot CoM z {com[2]*1000:+.1f} mm, x {com[0]*1000:+.1f} mm, "
          f"chassis CoM z {m.body_ipos[ch][2]*1000:+.1f} mm, chassis Iyy {m.body_inertia[ch][1]*1e6:.0f} kg mm2 (about its CoM)")
