import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import json, base64
S=WS + "v2site/"
s=open(S+"index.html").read(); L=s.split("\n")
for i,l in enumerate(L):
    if l.startswith("const PARTS="): L[i]="const PARTS="+open(WS + "case/viewer_parts_v11.json").read()+";"
    elif l.startswith("const WIRES="): L[i]="const WIRES="+open(WS + "case/wiring_data.json").read()+";"
open(S+"index.html","w").write("\n".join(L))
open(S+"print.html","w").write(open(S+"print_tpl.html").read().replace("__PARTS__", open(WS + "case/print_parts_v11.json").read()))
open(S+"wiring.html","w").write(open(S+"wiring_tpl.html").read().replace("__DATA__", open(WS + "case/wiring_diagram.json").read()))
for name, path in [("V1.1_print_files.zip","/tmp/V1.1_print_files.zip"),("balancing_robot_BOM_V1.1.xlsx",WS + "bom/balancing_robot_BOM_V1.1.xlsx")]:
    json.dump({"name":name,"b64":base64.b64encode(open(path,"rb").read()).decode()}, open(S+"files/v11/"+name+".json","w"))
print("site rebuilt")
