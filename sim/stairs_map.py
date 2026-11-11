"""Reconstruct the step from the accumulated cloud, and say how much to believe it.

The single-shot estimator in `stairs_probe.estimate()` takes the first level surface below the landing
and calls the height difference the riser. That is right when the scan reached the first tread and
wrong - silently, confidently wrong - when it overflew it and found the second tread instead, which
reads as a riser of exactly twice the truth. Those are the "fake" measurements: on the held-out set the
flick returns a number in 93% of episodes and only 39% of them are right.

What the robot can check without being told the answer:

  * **where the surface starts.** The first tread begins at the edge. A surface that begins 300 mm
    beyond the edge is not it, whatever its height, and the edge is measurable: it is where the landing
    the robot is standing on stops returning.
  * **how wide it is.** A handful of rays clipping a nose is not a tread.
  * **whether two looks from different standoffs agree.** Geometry does not move between looks; a
    wrong surface identification does, because the overflying ray lands on a different step.

`reconstruct()` applies the first two and reports a confidence; `fuse()` applies the third. Both run on
what the robot has: its own cloud, in the frame its odometry and IMU built, and the knowledge that it
is standing on the landing.

  python sim/stairs_map.py --study 60      # what separates right readings from wrong ones
"""
import argparse, math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "ml"))

TOL = 0.02              # m: height tolerance that groups returns into one level surface
MIN_PTS = 12            # returns below which a "surface" is a few rays clipping something
MIN_SPAN = 0.040        # m: a tread narrower than this in x was not really seen
MIN_GAP = -0.010        # m: the tread starts AT the edge, never behind it. Surfaces that start behind
                        # the edge are the landing's own grazing-incidence tail, which reads 23 mm deep
                        # every time and accounted for 8 of 26 wrong readings
MAX_GAP = 0.300         # m: measured, not chosen - right readings start 176 +- 45 mm past the edge
                        # (worst 268), wrong ones 1330 +- 1029
MIN_TREAD_PTS = 80      # returns on the tread below which the reading is not trusted: with the gap
                        # test this pair keeps 23 of 26 right readings and 0 of 26 wrong ones
AGREE = 0.015           # m: two looks agreeing this closely are taken as confirmation


def surfaces(pts, tol=TOL, min_pts=MIN_PTS):
    """Level surfaces in the cloud, grouped by height (not by x: beyond the edge the hits jump out)."""
    if len(pts) < min_pts:
        return []
    z = pts[:, 2]
    out = []
    for zc in np.arange(z.min(), z.max() + tol, tol):
        m = np.abs(z - zc) <= tol / 2
        if m.sum() >= min_pts:
            out.append(dict(z=float(np.median(z[m])), n=int(m.sum()),
                            x0=float(pts[m, 0].min()), x1=float(pts[m, 0].max())))
    merged = []
    for s in sorted(out, key=lambda s: -s["z"]):
        if merged and abs(merged[-1]["z"] - s["z"]) < tol:
            m = merged[-1]
            m["x0"], m["x1"] = min(m["x0"], s["x0"]), max(m["x1"], s["x1"])
            m["n"] += s["n"]
        else:
            merged.append(dict(s))
    return merged


def reconstruct(pts, landing_z, min_pts=MIN_PTS, min_span=MIN_SPAN, max_gap=MAX_GAP,
                min_gap=MIN_GAP, min_tread_pts=MIN_TREAD_PTS):
    """The first step, with the reasons to doubt it attached.

    Returns a dict with `rise`, `tread`, `edge`, `conf` in 0..1 and `why` naming what failed. A `rise`
    of None means the cloud does not support a measurement - which is a far better answer than a
    number that happens to be twice the truth.
    """
    out = dict(rise=None, tread=None, edge=None, conf=0.0, why="no cloud", n=0, span=0.0, gap=float("nan"))
    surf = surfaces(pts, min_pts=min_pts)
    out["n_surf"] = len(surf)
    if not surf:
        return out
    land = next((s for s in surf if abs(s["z"] - landing_z) < 0.02), None)
    if land is None:
        out["why"] = "the landing was never in the scan: nothing to measure the riser against"
        return out
    out["edge"] = land["x1"]
    below = [s for s in surf if s["z"] < land["z"] - 0.02]
    if not below:
        out["why"] = "nothing below the landing: the scan never cleared the edge"
        return out
    first = below[0]
    gap, span = first["x0"] - land["x1"], first["x1"] - first["x0"]
    out.update(rise=land["z"] - first["z"], tread=span, n=first["n"], span=span, gap=gap)
    if gap > max_gap:
        # the giveaway: the first tread starts AT the edge, so a surface that starts well past it is a
        # later step seen over the top of the one being measured
        out.update(rise=None, why=f"the surface starts {gap*1000:.0f} mm past the edge: a later step")
        return out
    if gap < min_gap:
        out.update(rise=None, why=f"the surface starts {-gap*1000:.0f} mm BEHIND the edge: it is the "
                                  f"landing itself, seen at grazing incidence")
        return out
    if first["n"] < min_tread_pts:
        out.update(rise=None, why=f"only {first['n']} returns on it: not enough to call a tread")
        return out
    if span < min_span:
        out.update(rise=None, why=f"only {span*1000:.0f} mm of surface: too little to call a tread")
        return out
    # confidence: returns, width, and how cleanly it joins the edge. Deliberately crude - its job is to
    # decide whether to look again, not to be a likelihood.
    # calibrated against the study table, not guessed: a right reading has ~400 returns on a 111 mm
    # span starting 176 +- 45 mm past the edge, so the gap term must not start biting until ~200 mm
    out["conf"] = float(min(1.0, first["n"] / 150.0) * min(1.0, span / 0.080)
                        * math.exp(-(max(0.0, gap - 0.20) / 0.10) ** 2))
    out["why"] = "ok"
    return out


def fuse(views, landing_z, **kw):
    """Reconstruct each look separately, then the pooled cloud, and cross-check them.

    Two looks from different standoffs that agree are the strongest evidence available on board: an
    overflying ray lands on a different step when the standoff changes, so a wrong identification
    disagrees with itself while real geometry does not.
    """
    per = [reconstruct(v, landing_z, **kw) for v in views]
    good = [p for p in per if p["rise"] is not None]
    # the best SINGLE look wins. Pooling clouds taken from different standoffs mixes two occlusion
    # patterns: the landing's grazing-incidence tail from one look lands behind the edge seen in the
    # other, and the merged cloud grows surfaces neither look actually saw. Pooling is a fallback only.
    best = max(good, key=lambda p: p["conf"], default=None)
    if best is None:
        pooled = reconstruct(np.concatenate(views) if views else np.zeros((0, 3)), landing_z, **kw)
        if pooled["rise"] is not None:
            best = pooled
    if best is None:
        return dict(rise=None, conf=0.0, agree=False, n_views=len(per),
                    why=per[-1]["why"] if per else "no look taken")
    agree = any(abs(a["rise"] - b["rise"]) < AGREE
                for i, a in enumerate(good) for b in good[i + 1:])
    out = dict(best)
    out.update(agree=agree, n_views=len(per), n_good=len(good))
    if agree:
        out["conf"] = min(1.0, out["conf"] + 0.35)
        out["rise"] = float(np.median([p["rise"] for p in good]))
    return out


# --------------------------------------------------------------------------- study
def study(trials, seed0=20_000):
    """Run the flick recipe and tabulate what the picked surface looked like when it was right and
    when it was wrong. The rejection thresholds above come from this table, not from taste."""
    from stairs_env import StairsLookEnv
    from stairs_baseline import Scripted, run
    env, pol = StairsLookEnv(), Scripted()
    rows = []
    for k in range(trials):
        env.reset(seed=seed0 + k)
        _, info = run(env, pol)
        pts = np.concatenate(env.views) if env.views else np.zeros((0, 3))
        r = reconstruct(pts, env.landing_z, max_gap=9.9, min_span=0.0)   # no rejection: just look
        rows.append(dict(truth=env.rise, tread=env.tread, est=r["rise"], n=r["n"], span=r["span"],
                         gap=r["gap"], conf=r["conf"], why=r["why"]))
    env.close()
    have = [r for r in rows if r["est"] is not None]
    right = [r for r in have if abs(r["est"] - r["truth"]) <= 0.020]
    wrong = [r for r in have if abs(r["est"] - r["truth"]) > 0.040]
    print(f"\n\033[1mWhat the picked surface looks like, {trials} episodes of the flick recipe\033[0m")
    print(f"  a number came out in {len(have)}/{trials}; right {len(right)}, wrong {len(wrong)}\n")
    print(f"  {'':<10} {'returns':>18} {'x span':>18} {'gap past the edge':>26}")
    for tag, rs in (("right", right), ("wrong", wrong)):
        if not rs:
            continue
        n = np.array([r["n"] for r in rs], float)
        sp = np.array([r["span"] for r in rs]) * 1000
        gp = np.array([r["gap"] for r in rs]) * 1000
        print(f"  {tag:<10} {n.mean():7.0f} +-{n.std():4.0f} {sp.mean():10.0f} +-{sp.std():4.0f} mm "
              f"{gp.mean():14.0f} +-{gp.std():4.0f} mm   (worst {gp.max():+.0f})")
    print(f"\n  {'gap cut':>8} {'span cut':>9} {'n cut':>6}   right kept   wrong kept")
    for gcut in (0.25, 0.30, 0.35):
        for scut in (0.30, 0.40, 9.9):
            for ncut in (12, 40, 80):
                kr = sum(1 for r in right if r["gap"] <= gcut and r["span"] <= scut and r["n"] >= ncut)
                kw = sum(1 for r in wrong if r["gap"] <= gcut and r["span"] <= scut and r["n"] >= ncut)
                print(f"  {gcut*1000:7.0f} {scut*1000:8.0f} {ncut:6d}   {kr:4d}/{len(right):<6d} {kw:4d}/{len(wrong)}")
    print("\n  the wrong readings that sit close to the edge, one line each:")
    for r in sorted(wrong, key=lambda r: r["gap"])[:10]:
        print(f"    truth {r['truth']*1000:3.0f} read {r['est']*1000:5.0f} mm  ratio {r['est']/r['truth']:4.1f}  "
              f"n {r['n']:4d}  span {r['span']*1000:5.0f}  gap {r['gap']*1000:6.0f} mm")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--study", type=int, default=0)
    a = ap.parse_args()
    if a.study:
        study(a.study)


if __name__ == "__main__":
    main()
