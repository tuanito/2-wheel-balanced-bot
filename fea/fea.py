"""Linear static FEA (CalculiX) of the lower body under motor loads.
Usage: python3 fea.py old|new
Geometry: body_case.build_body() cut to the lower part (z -36..36), tet10 mesh (gmsh).
Loads per motor, through the 4 M2.5 screws (hole wall + area under the head):
  - reaction torque T about the axle (both motors same direction: worst for the body)
  - wheel force Fz (ground pushing up, bumps/landing), moved from the wheel centre to the wall
    -> force + bending moment about x
Fixed: the 4 floor insert bosses of the front panel / back cover screws (the only link to the shell).
"""
import sys, os, subprocess, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cad_scripts"))
import cadquery as cq, body_case as bc, gmsh

VAR = sys.argv[1]
T_NMM = float(os.environ.get("T", 2300.0))   # 2.3 N.m: 16.5 A x 0.14 N.m/A, motor peak
FZ = float(os.environ.get("FZ", 25.0))        # N per wheel: ~3 g on half the robot's weight
E, NU = 2000.0, 0.38                          # PETG, MPa
H = float(os.environ.get("H", 1.4))           # mesh size at the motor walls (mm)
WITH_SHELF = VAR == "reinf" and os.environ.get("NOSHELF", "0") != "1"
SHELF = os.environ.get("SHELF", "0") == "1"   # 1: the battery shelf holds the rail tops (y, z)
OUT = os.path.join(HERE, "out", VAR) + ("_noshelf" if VAR == "reinf" and not WITH_SHELF else "") + ("_shelf" if SHELF else "") + os.environ.get("TAG", ""); os.makedirs(OUT, exist_ok=True)

# the two tray bosses touch the motor walls tangentially (r 4 at y 13 vs wall at 17): a zero-width sliver
# that no mesher handles; merge them 1 mm into the wall for the analysis (no effect away from the floor)
bc.FLOOR_TRAY_PTS = [(-24.0, -12.0), (21.0, 12.0)]
if VAR == "old":
    bc.SLOT_W, bc.SLOT_Z = 19.0, (13.0, bc.SHELF_Z + 0.1)
body = bc.build_body().val()
ZC = 36.0
# the floor + motor-walls block is a separate solid from the shell (the wheel wells cut the floor off)
lower = [s for s in body.Solids() if s.BoundingBox().zmin < -30][0]
if WITH_SHELF:   # shelf screwed to the wall tops: modelled as bonded
    lower = lower.fuse(bc.build_shelf().val()).clean()
cq.exporters.export(cq.Workplane().add(lower), f"{OUT}/lower.step")

# ---------------- mesh ----------------
gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
for k_, v_ in [("Geometry.OCCFixDegenerated", 1), ("Geometry.OCCFixSmallEdges", 1), ("Geometry.OCCFixSmallFaces", 1), ("Geometry.OCCSewFaces", 1)]: gmsh.option.setNumber(k_, v_)
gmsh.model.occ.importShapes(f"{OUT}/lower.step"); gmsh.model.occ.healShapes(); gmsh.model.occ.removeAllDuplicates(); gmsh.model.occ.synchronize()
f = gmsh.model.mesh.field
b1 = f.add("Box"); f.setNumber(b1, "VIn", H); f.setNumber(b1, "VOut", 3.2)
f.setNumber(b1, "XMin", -32); f.setNumber(b1, "XMax", 29); f.setNumber(b1, "YMin", -26); f.setNumber(b1, "YMax", 26)
f.setNumber(b1, "ZMin", -36); f.setNumber(b1, "ZMax", 37); f.setNumber(b1, "Thickness", 8)
f.setAsBackgroundMesh(b1)
gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0); gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
gmsh.option.setNumber("Mesh.ElementOrder", 2); gmsh.option.setNumber("Mesh.HighOrderOptimize", 0); gmsh.option.setNumber("Mesh.SecondOrderLinear", 1)
gmsh.option.setNumber("Mesh.Algorithm3D", int(os.environ.get("ALG3", 1)))
gmsh.model.mesh.generate(3)
tag, xyz, _ = gmsh.model.mesh.getNodes()
X = xyz.reshape(-1, 3); idx = {int(t): i for i, t in enumerate(tag)}
et, etags, enodes = gmsh.model.mesh.getElements(3)
k = list(et).index(11)   # 10-node tet
tets = np.array([idx[int(n)] for n in enodes[k]]).reshape(-1, 10)
gmsh.finalize()
print(f"{VAR}: {len(X)} nodes, {len(tets)} tet10")

# ---------------- BCs and loads ----------------
# supports: the 4 floor bosses where the front panel and back cover screw on (M3 inserts, y +-40, z -28.5).
# The panels tie this block to the upper shell; they are treated as rigid.
zb = bc.Z_FLOOR + 3.5
ri = bc.INSERT_D / 2
fixed = np.where((np.abs(np.hypot(np.abs(X[:, 1]) - 40.0, X[:, 2] - zb) - ri) < 0.08) &
                 ((X[:, 0] > bc.X_IN_F - bc.INS_M3[1] - 0.1) | (X[:, 0] < bc.X_BACK + bc.INS_M3[1] + 0.1)))[0]
rails = np.where(X[:, 2] > bc.SHELF_Z + bc.RAIL_H - 0.05)[0] if SHELF else np.array([], int)
print("fixed nodes", len(fixed), "rail nodes", len(rails))
F = np.zeros_like(X)
HOLES = [(-10, -10), (10, -10), (10, 10), (-10, 10)]
rh = bc.M25_CLEAR / 2
for s in (1, -1):
    yin = s * bc.GAP_HALF                     # inner wall face (screw heads)
    ywall = s * (bc.GAP_HALF + bc.MWALL_T / 2)
    yw = s * (bc.WHEEL_IN_Y + bc.WHEEL_W / 2)
    d = yw - ywall
    for (xc, zc) in HOLES:
        rr = np.hypot(X[:, 0] - xc, X[:, 2] - zc)
        inwall = (s * X[:, 1] > bc.GAP_HALF - 0.05) & (s * X[:, 1] < bc.GAP_HALF + bc.MWALL_T + 0.05)
        shank = inwall & (np.abs(rr - rh) < 0.08)
        head = (np.abs(X[:, 1] - yin) < 0.05) & (rr > rh - 0.05) & (rr < 2.4)
        sel = np.where(shank | head)[0]
        fh = np.array([T_NMM / (4 * 200.0) * zc, 0.0, -T_NMM / (4 * 200.0) * xc])  # sum r x F = T about y
        fh += np.array([0.0, -d * FZ * zc / 400.0, FZ / 4])                        # Fz + moment d*Fz about x
        F[sel] += fh / len(sel)
assert abs(F[:, 2].sum() - 2 * FZ) < 1e-6

# ---------------- CalculiX ----------------
with open(f"{OUT}/job.inp", "w") as o:
    o.write("*NODE, NSET=NALL\n")
    for i, p in enumerate(X): o.write(f"{i+1},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}\n")
    o.write("*ELEMENT, TYPE=C3D10, ELSET=EALL\n")
    perm = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8]   # gmsh -> abaqus/ccx edge-node order
    for j, t in enumerate(tets): o.write(f"{j+1}," + ",".join(str(t[p] + 1) for p in perm) + "\n")
    o.write("*NSET, NSET=FIX\n" + "\n".join(str(i + 1) for i in fixed) + "\n")
    o.write(f"*MATERIAL, NAME=PETG\n*ELASTIC\n{E},{NU}\n*SOLID SECTION, ELSET=EALL, MATERIAL=PETG\n")
    if len(rails): o.write("*NSET, NSET=RAIL\n" + "\n".join(str(i + 1) for i in rails) + "\n")
    o.write("*STEP\n*STATIC\n*BOUNDARY\nFIX,1,3,0\n" + ("RAIL,2,3,0\n" if len(rails) else "") + "*CLOAD\n")
    for i in np.where(np.abs(F).sum(1) > 0)[0]:
        for c in range(3):
            if F[i, c] != 0: o.write(f"{i+1},{c+1},{F[i, c]:.6e}\n")
    o.write("*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n")
subprocess.run(["ccx", "-i", "job"], cwd=OUT, check=True, stdout=open(f"{OUT}/ccx.log", "w"))

# ---------------- read results ----------------
def read_frd(path, name, ncomp):
    out = {}; on = False
    for line in open(path):
        if line.startswith(" -4") and name in line: on = True; continue
        if on:
            if line.startswith(" -3"): break
            if line.startswith(" -1"):
                n = int(line[3:13]); out[n] = [float(line[13 + 12 * c:25 + 12 * c]) for c in range(ncomp)]
    a = np.zeros((len(X), ncomp))
    for n, v in out.items(): a[n - 1] = v
    return a
U = read_frd(f"{OUT}/job.frd", "DISP", 3)
S = read_frd(f"{OUT}/job.frd", "STRESS", 6)   # xx yy zz xy yz zx
sx, sy, sz, txy, tyz, tzx = S.T
vm = np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))
np.savez_compressed(f"{OUT}/res.npz", X=X, tets=tets[:, :4], U=U, S=S, vm=vm, F=F)

# exclude the load-introduction points (local bearing under the screws, real but not a structural weakness)
near = np.zeros(len(X), bool)
for (xc, zc) in HOLES:
    near |= (np.hypot(X[:, 0] - xc, X[:, 2] - zc) < 3.2) & (np.abs(X[:, 1]) < 25)
def region(m):
    i = np.where(m & ~near)[0]; j = i[np.argmax(vm[i])]
    return dict(vm=round(float(vm[j]), 2), sx_tension=round(float(sx[i].max()), 2), at=[round(float(v), 1) for v in X[j]])
wall = (np.abs(X[:, 1]) > 16.9) & (np.abs(X[:, 1]) < 24.1)
res = {
    "variant": VAR, "nodes": len(X), "T_Nm": T_NMM / 1000, "Fz_N": FZ,
    "max_disp_mm": round(float(np.linalg.norm(U, axis=1).max()), 4),
    "motor_wall_twist_deg": None,
    "everywhere": region(np.ones(len(X), bool)),
    "motor_walls": region(wall),
    "wall_above_axle_z12_36": region(wall & (X[:, 2] > 12)),
    "wall_floor_junction": region(wall & (X[:, 2] < -27)),
    "floor_bosses": region((np.abs(np.abs(X[:, 1]) - 40) < 8) & (X[:, 2] < -20)),
    "rest_of_body": region(~wall & (X[:, 2] < bc.SHELF_Z - 0.05)),
    "shelf": region(X[:, 2] > bc.SHELF_Z + 0.05) if (X[:, 2] > bc.SHELF_Z + 0.05).any() else None,
    "screw_bearing_max": round(float(vm[near].max()), 2),
}
# rotation of the motor mount (rigid-body fit of the hole-node displacements) -> how much the motor axis tilts
for s in (1, -1):
    m = (s * X[:, 1] > bc.GAP_HALF - 0.05) & (s * X[:, 1] < bc.GAP_HALF + bc.MWALL_T + 0.05) & near
    P = X[m] - X[m].mean(0); D = U[m] - U[m].mean(0)
    A = np.zeros((3 * len(P), 3)); b = D.reshape(-1)
    for q, p in enumerate(P):   # D = w x p
        A[3 * q] = [0, p[2], -p[1]]; A[3 * q + 1] = [-p[2], 0, p[0]]; A[3 * q + 2] = [p[1], -p[0], 0]
    w = np.linalg.lstsq(A, b, rcond=None)[0]
    res[f"mount_rotation_deg_side{s:+d}"] = [round(float(np.degrees(v)), 4) for v in w]
    res[f"mount_shift_mm_side{s:+d}"] = [round(float(v), 4) for v in U[m].mean(0)]
del res["motor_wall_twist_deg"]
json.dump(res, open(f"{OUT}/summary.json", "w"), indent=1)
print(json.dumps(res, indent=1))
