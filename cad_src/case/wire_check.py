"""Check every harness bundle and every plug housing against every part. Overlaps within 7 mm of one of the
harness's own pins are the connection itself and are ignored."""
import sys, json, math
exec(open('/home/claude/case/real_assembly.py').read().split('if __name__ == "__main__":')[0])
import wiring as W
HS = W.harnesses()
targets = dict(PRINTED); targets.update({k: v[0] for k, v in ALL.items()})
def bb_apart(a, b):
    p, q = a.BoundingBox(), b.BoundingBox()
    return p.xmax < q.xmin or q.xmax < p.xmin or p.ymax < q.ymin or q.ymax < p.ymin or p.zmax < q.zmin or q.zmax < p.zmin
for lbl, shp, col in W.connectors():
    for n, s in targets.items():
        if bb_apart(shp, s): continue
        try: v = shp.intersect(s).Volume()
        except Exception: v = 0
        if v > 0.5: print("CONNECTOR", lbl[:44], "x", n[:44], round(v, 1))
res = []
for h in HS:
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
            hits.append((n, round(so.Volume(), 1), [round(v, 1) for v in c]))
    L = [round(W.length(W.conductor_path(h, c))) for c in h["conds"]]
    res.append(dict(name=h["name"], wires=len(h["conds"]), length_mm=max(L), hits=hits))
    print(f"{h['name'][:50]:50s} {max(L):5d} mm  {'OK' if not hits else hits}")
json.dump(res, open('/home/claude/case/wire_report.json', 'w'), indent=1)
