import pathlib as _pl, sys as _sys
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "cad_src" / "case"))
import robot_paths as RP            # where everything is on this machine (BOT_WS still honoured)
import json, base64
S=str(RP.SITE) + "/"
s=open(S+"index.html").read(); L=s.split("\n")
for i,l in enumerate(L):
    if l.startswith("const PARTS="): L[i]="const PARTS="+open(RP.artifact("viewer_parts_v11.json")).read()+";"
    elif l.startswith("const WIRES="): L[i]="const WIRES="+open(RP.artifact("wiring_data.json")).read()+";"
open(S+"index.html","w").write("\n".join(L))
open(S+"print.html","w").write(open(S+"print_tpl.html").read().replace("__PARTS__", open(RP.artifact("print_parts_v11.json")).read()))
open(S+"wiring.html","w").write(open(S+"wiring_tpl.html").read().replace("__DATA__", open(RP.artifact("wiring_diagram.json")).read()))
for name, path in [("V1.1_print_files.zip","/tmp/V1.1_print_files.zip"),("balancing_robot_BOM_V1.1.xlsx",str(RP.BOM / "balancing_robot_BOM_V1.1.xlsx"))]:
    json.dump({"name":name,"b64":base64.b64encode(open(path,"rb").read()).decode()}, open(S+"files/v11/"+name+".json","w"))
print("site rebuilt")
