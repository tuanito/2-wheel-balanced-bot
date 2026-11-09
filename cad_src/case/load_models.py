import cadquery as cq, os, glob
import robot_paths as RP
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
D = str(RP.models_dir())
def load(name):
    """STEP -> cq.Shape, cached as BREP (loading the camera STEP takes ~1 min)."""
    br = os.path.join(D, name.rsplit(".", 1)[0] + ".brep")
    if os.path.exists(br):
        s = TopoDS_Shape(); BRepTools.Read_s(s, br, BRep_Builder()); return cq.Shape.cast(s)
    sh = cq.importers.importStep(os.path.join(D, name))
    s = cq.Compound.makeCompound([v for v in sh.vals()]) if len(sh.vals()) > 1 else sh.val()
    BRepTools.Write_s(s.wrapped, br)
    return s
