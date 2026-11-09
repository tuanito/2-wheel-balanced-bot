import sys, importlib; sys.path.insert(0,"/home/claude/case")
import body_case as bc
from design_variants import VARIANTS
for name,spec in VARIANTS.items():
    importlib.reload(bc); bc.SKIN=spec["skin"]
    sk=bc.skin_solid(bc.X_BACK,bc.X_FRONT)
    bad=[]
    pts=[(y,z) for (y,z) in bc.PANEL_SCREWS]+[(-y,z) for (y,z) in bc.PANEL_SCREWS]
    for (y,z) in pts:
        c=bc.cyl_x(bc.X_IN_F-8, 8+3, y, z, 4.0+2*1.2)
        v=c.cut(sk).val().Volume()
        if v>0.01: bad.append(((y,z),round(v,1)))
    print(name,"insert walls <1.2mm at:",bad)
