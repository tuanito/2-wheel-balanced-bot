"""Every printed part in its print orientation on the bed (z = 0 = bed), plus an overhang analysis.
Writes print_parts_v11.json (see robot_paths.OUT) for print.html."""
import sys, json, base64, math, numpy as np
import robot_paths as RP; RP.add_sources()
import cadquery as cq, body_case as bc, wheel_adapter as wa

def rot(shape, axis, deg):
    return shape if not deg else shape.rotate((0, 0, 0), axis, deg)

def on_bed(shape):
    b = shape.BoundingBox()
    return shape.translate((-(b.xmin + b.xmax) / 2, -(b.ymin + b.ymax) / 2, -b.zmin))

def tess(shape, tol=0.2):
    v, t = shape.tessellate(tol, 0.3)
    return np.array([[p.x, p.y, p.z] for p in v]), np.array(t, dtype=np.int64)

def bed_area(shape):
    V, T = tess(on_bed(shape), 0.4)
    A = V[T]; n = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0]); a = np.linalg.norm(n, axis=1) / 2
    nz = n[:, 2] / (2 * a + 1e-12)
    return a[(nz < -0.99) & (A[:, :, 2].max(1) < 0.05)].sum()

def auto(shape):   # the axis-aligned orientation with the largest flat area on the bed
    cands = [((1, 0, 0), 0), ((1, 0, 0), 180), ((1, 0, 0), 90), ((1, 0, 0), -90), ((0, 1, 0), 90), ((0, 1, 0), -90)]
    return max(cands, key=lambda c: bed_area(rot(shape, *c)))

Y = (0, 1, 0); X = (1, 0, 0)
P = [  # name, label, shape, orientation (axis, deg) or None = auto, colour, how/why
 ("body", "Body (shell + motor block)", bc.build_body().val(), (Y, 90), "#3d7cc9",
  "Standing on its front opening: layers stack from the front to the back. Holes across the layers are teardrops, the back insert bosses and back ribs have 45° gussets and the motor pockets are teardrops, so nothing needs support. The orange areas are short bridges: the two motor-wall cable windows (16 mm), the XT60 slot in the top (16 mm) and screw seats. The file holds two pieces (shell + motor block): print both."),
 ("front", "Front panel", bc.build_front().val(), (Y, 90), "#5aa0e0",
  "Outside face down: the face comes out smooth, the insert bosses and eye pockets grow upward. Red at the top edge: the first millimetre of the rounded top-front corner, fine on any printer. Orange: 0.2 mm sacrificial layers in the screw recesses (poke through after printing) and the 2 mm ledge of the camera-window pocket (a short bridge). The headlight windows are 45° cones: no support."),
 ("cover", "Back cover", bc.build_cover().val(), (Y, -90), "#5aa0e0",
  "Outside face down; standoffs for the Pi, perfboard and contact plate grow upward."),
 ("shelf", "Battery shelf", bc.build_shelf().val(), None, "#6fa8e8",
  "Flat, as it sits in the robot: countersinks and the lidar-plate bosses on top."),
 ("tray", "Electronics tray", bc.build_tray().val(), None, "#f0a040", "Flat, posts up."),
 ("clamp_bar", "Clamp bar", bc.build_clamp_bar().val(), None, "#f0a040", "Flat."),
 ("carrier_plate", "Carrier plate (x2)", bc.build_carrier_plate().val(), None, "#f0a040", "Flat. Print two."),
 ("speaker_ring", "Speaker ring + amp tab", bc.build_speaker_ring().val(), None, "#f0a040", "Flat, wall side down."),
 ("mic_clamp", "Mic clamp (x2)", bc.build_mic_clamp(1).val(), None, "#f0a040", "Flat. Print two (they are mirror-identical)."),
 ("eye_retainer", "Eye retainer (x2)", bc.build_eye_retainer(1).val(), (Y, -90), "#f0a040", "Flat, frame up. Print two."),
 ("light_retainer", "Headlight retainer (x2)", bc.build_light_retainer(1).val(), (Y, -90), "#f0a040", "Flat, ring up. Print two (mirror-identical)."),
 ("diffuser", "Eye diffuser (x2)", bc.build_diffuser().val(), None, "#e8e8e8", "Flat, in white/natural PETG. Print two."),
 ("ir_hood", "IR receiver hood (x4)", bc.build_ir_hood().val(), None, "#2a2a2e", "Flat, base down, in black PETG (light-tight). The window's roof is a ~5 mm bridge. Print four; each is glued on the top skin over its hole, window facing out."),
 ("contact_plate", "Contact plate", bc.build_contact_plate().val(), ((0, 1, 0), 90), "#c0c4cc", "Inner face down (the cover side): the funnels open upwards, so their 45° walls and the screw-head seats print with no overhang; the contact face is the top surface."),
 ("dock_foot", "Charging dock: foot (floor plate + lower column)", bc.build_dock("foot").val(), ((0, 1, 0), 0), "#e7e9ee", "Standing, floor plate on the bed (123 mm). The column leans back 8° and is open at the top (no roof to bridge); two M3 insert columns grow up inside its back wall; the joint ring on top. Orange: the ring's 1.6 mm inner lip, the jack hole and the edges of the base's windows."),
 ("dock_head", "Charging dock: head (shelf + face slab + upper column)", bc.build_dock("head").val(), ((0, 1, 0), bc.DOCK_LEAN), "#e7e9ee", "Standing on the shelf's underside (86 mm): the face slab now reaches down to that plane, so its wide sides grow from the bed instead of hanging 48 mm out over the column. The tongue has a 45° underside, the rod bosses 45° wedges. Orange: the roof of the joint groove (2.6 mm) and of the wire passage (16 mm), small horizontal holes. Glue onto the foot's ring + 2 × M3×16 from the top of the shelf."),
 ("dock_mast", "Charging dock: mast", bc.build_dock("mast").val(), ((1, 0, 0), 180), "#e7e9ee", "Upside down: its closed top on the bed, the spigot pointing up. The hollow's roof becomes the floor: no bridge, no support (133 mm tall)."),
 ("dock_ridge", "Dock wheel track frame (x2, PETG)", bc.build_dock("ridge").val(), ((1, 0, 0), 0), "#cfd6e0", "Flat, as it lies on the floor. Print two, one mirrored. The lead-in ramp and the rear lip of the saddle, a dovetail window for the TPU insert, a rail outside the tyre and the T end that drops into its slot in the dock's floor plate."),
 ("dock_ridge_insert", "Dock saddle insert (x2, TPU 95A)", bc.build_dock("ridge_insert").val(), ((1, 0, 0), 0), "#3a3a40", "Flat, ribs up, in TPU 95A (slow, ~20 mm/s). The trough the wheel sits in and the front lip, with 0.4 mm grip ribs. Push it up into the frame's window from below (dovetail: it can't lift out). Print two, one mirrored."),
 ("dock_wing", "Dock side wing (x2)", bc.build_dock("wing").val(), ((0, 1, 0), 270), "#cfd6e0", "Standing on its back edge (the side that bolts to the face slab): the angled 3 mm wall grows straight up, the two M3 holes are small horizontal holes. No support (lying flat it had ~2700 mm² of overhang). Print two, one mirrored."),
 ("adapter", "Wheel adapter (x2)", wa.build().val(), (X, 0), "#e08a2e",
  "Hex end down, 100 % infill. The 45° cone under the flange needs no support; the screw-head seats use the two-layer bridge trick (a slot, then a square, then the round hole) so they print clean with nothing to drill out; the insert hole has a 45° pointed end. Print two."),
]

def mesh(V, T):
    lo = V.min(0); hi = V.max(0); sc = hi - lo; sc[sc == 0] = 1
    Q = np.round((V - lo) / sc * 65535).astype(np.uint16)
    idx = T.astype(np.uint16 if len(V) < 65536 else np.uint32)
    return dict(lo=lo.round(4).tolist(), sc=sc.round(4).tolist(), v=base64.b64encode(Q.tobytes()).decode(),
                i=base64.b64encode(idx.tobytes()).decode(), i32=bool(len(V) >= 65536))

out = []
for name, label, shape, ori, col, how in P:
    if ori is None:
        ori = auto(shape)
    s = on_bed(rot(shape, *ori))
    b = s.BoundingBox()
    V, T = tess(s, 0.25 if name == "body" else 0.15)
    A = V[T]; n = np.cross(A[:, 1] - A[:, 0], A[:, 2] - A[:, 0]); a = np.linalg.norm(n, axis=1) / 2; nz = n[:, 2] / (2 * a + 1e-12)
    up = A[:, :, 2].min(1) > 0.3
    flat = a[(nz < -0.99) & up].sum()                       # horizontal ceilings: bridges (screw-head seats, sacrificial layers, small holes)
    steep = a[(nz < -0.72) & (nz >= -0.99) & up].sum()    # sloped >45 deg
    out.append(dict(name=name, label=label, color=col, how=how, size=[round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)],
                    vol=round(s.Volume() / 1000, 1), bridge_mm2=round(float(flat)), steep_mm2=round(float(steep)), **mesh(V, T)))
    print(f"{name:14s} {b.xlen:6.1f} x {b.ylen:6.1f} x {b.zlen:6.1f}  bridges {flat:7.0f} mm2  >45deg {steep:6.0f} mm2  ori {ori}")
json.dump(out, open(RP.out('print_parts_v11.json'), 'w'))
