import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import cadquery as cq, os, glob
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
_here = os.path.dirname(os.path.abspath(__file__))
D = os.environ.get("ROBOT_MODELS") or next((p for p in (os.path.join(_here, "..", "models", "bought"), WS + "models/bought") if os.path.isdir(p)), os.path.join(_here, "..", "models", "bought"))
def load(name):
    """STEP -> cq.Shape, cached as BREP (loading the camera STEP takes ~1 min)."""
    br = os.path.join(D, name.rsplit(".", 1)[0] + ".brep")
    if os.path.exists(br):
        s = TopoDS_Shape(); BRepTools.Read_s(s, br, BRep_Builder()); return cq.Shape.cast(s)
    sh = cq.importers.importStep(os.path.join(D, name))
    s = cq.Compound.makeCompound([v for v in sh.vals()]) if len(sh.vals()) > 1 else sh.val()
    BRepTools.Write_s(s.wrapped, br)
    return s
