"""List body surfaces that face the print bed (build direction = -X, bed at x = X_IN_F) and are not on the bed.
Faces steeper than 45 deg from vertical, i.e. normal_x > cos(45), would need support (small bridges are OK)."""
import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import sys, importlib, math
sys.path.insert(0, WS + 'case')
import cadquery as cq
from OCP.BRepGProp import BRepGProp_Face
from OCP.gp import gp_Pnt, gp_Vec
mod = importlib.import_module(sys.argv[1] if len(sys.argv) > 1 else 'body_case')
body = mod.build_body().val()
bed = mod.X_IN_F
rows = []
for f in body.Faces():
    u0, u1, v0, v1 = f._uvBounds()
    prop = BRepGProp_Face(f.wrapped); p = gp_Pnt(); n = gp_Vec()
    prop.Normal((u0 + u1) / 2, (v0 + v1) / 2, p, n)
    n.Normalize() if n.Magnitude() > 1e-9 else None
    if n.X() > math.cos(math.radians(45)) + 1e-3:
        bb = f.BoundingBox()
        if abs(bb.xmin - bed) < 0.05 and abs(bb.xmax - bed) < 0.05:
            continue  # on the bed
        span = min(bb.ylen, bb.zlen)  # bridge length ~ the shorter side
        rows.append((f.Area(), span, round(bb.xmin, 1), round(bb.ymin, 1), round(bb.ymax, 1), round(bb.zmin, 1), round(bb.zmax, 1)))
rows.sort(reverse=True)
big = [r for r in rows if r[0] > 15]
print(f"{len(rows)} downward faces off the bed; {len(big)} larger than 15 mm2:")
for r in big[:25]:
    print(f"  area {r[0]:7.1f} mm2  span {r[1]:5.1f}  x={r[2]}  y {r[3]}..{r[4]}  z {r[5]}..{r[6]}")
