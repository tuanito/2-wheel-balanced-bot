import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import sys, math
sys.path.insert(0,WS + 'case')
exec(open(WS + 'case/real_assembly.py').read().split('if __name__ == "__main__":')[0])
import importlib, wiring as W
targets = dict(PRINTED); targets.update({k: v[0] for k, v in ALL.items()})
if '-b' in sys.argv:
    for n,s in targets.items():
        if any(k in n for k in ('ESP32','Pi Zero','back cover','perfboard','body','Power','CAN')):
            b=s.BoundingBox(); print(n[:40],[round(v,1) for v in (b.xmin,b.xmax,b.ymin,b.ymax,b.zmin,b.zmax)])
def bb_apart(a, b):
    p, q = a.BoundingBox(), b.BoundingBox()
    return p.xmax < q.xmin or q.xmax < p.xmin or p.ymax < q.ymin or q.ymax < p.ymin or p.zmax < q.zmin or q.zmax < p.zmin
for h in W.harnesses():
    if not any(k in h["name"] for k in sys.argv[1:] or ("switch","Power button")): continue
    t = W.tube(h["pts"], h["dia"]); hits = []
    ends = [c["a"] for c in h["conds"]] + [c["b"] for c in h["conds"]] + [h["pts"][0], h["pts"][-1]]
    for n, s in targets.items():
        if bb_apart(t, s): continue
        try: i = t.intersect(s)
        except Exception: continue
        for so in i.Solids():
            if so.Volume() < 0.2: continue
            c = so.Center(); c = (c.x, c.y, c.z)
            if min(math.dist(c, e) for e in ends) < 7: continue
            hits.append((n[:20], round(so.Volume(), 1), [round(v, 1) for v in c]))
    print(h["name"][:40], h["dia"], [tuple(round(v,1) for v in p) for p in h["pts"]], "\n   ", hits or "OK")
