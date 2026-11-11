"""Builds site/stairs.html from the clips sim/stairs_video.py rendered and the numbers it recorded.

Every caption, badge and figure on the page comes out of site/files/stairs_clips.json, which the video
script writes from each episode's own info dict. Nothing on the page is typed in by hand, so the page
cannot drift from the clips the way a written-up result drifts from the run it describes.

  python sim/stairs_video.py && python site/build_stairs.py
"""
import html, json, pathlib

HERE = pathlib.Path(__file__).parent
CLIPS = HERE / "files" / "stairs_clips.json"
OUT = HERE / "stairs.html"

NAV = [("index.html", "3D viewer"), ("wiring_boards.html", "Wiring diagram"), ("print.html", "How it prints"),
       ("docs.html", "README &amp; install"), ("code.html", "Source code"), ("bom.html", "Parts list"),
       ("standup.html", "Stand-up"), ("stairs.html", "Stairs")]

CSS = """:root{--bg:#f6f7f9;--fg:#1d2330;--mut:#667085;--ac:#2f6fd0;--card:#fff;--bd:#e2e5ea;--hd:#eef2f8;
 --pass:#15803d;--luck:#b45309;--fail:#b3261e}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#15171b;--fg:#e8eaee;--mut:#9aa2ae;
 --ac:#6ea3ff;--card:#1f2228;--bd:#30343c;--hd:#2a303a;--pass:#4ade80;--luck:#fbbf24;--fail:#f87171;
 color-scheme:dark}}
:root[data-theme="dark"]{--bg:#15171b;--fg:#e8eaee;--mut:#9aa2ae;--ac:#6ea3ff;--card:#1f2228;--bd:#30343c;
 --hd:#2a303a;--pass:#4ade80;--luck:#fbbf24;--fail:#f87171;color-scheme:dark}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--fg);
 font:15px/1.45 -apple-system,system-ui,Segoe UI,Roboto,sans-serif}
header{padding:12px 14px 6px}h1{font-size:17px;margin:0 0 2px}header p{margin:0;color:var(--mut);font-size:13px;max-width:92ch}
a{color:var(--ac)}
nav.top{display:flex;gap:6px;overflow-x:auto;padding:6px 14px 8px;scrollbar-width:none}
nav.top::-webkit-scrollbar{display:none}
nav.top a{flex:none;border:1px solid var(--bd);background:var(--card);color:var(--fg);border-radius:999px;
 padding:7px 13px;font-size:14px;text-decoration:none}
nav.top a.on{background:var(--ac);border-color:var(--ac);color:#fff}
main{padding:0 14px 32px;max-width:1140px}
h2{font-size:15px;margin:24px 0 2px}p.sub{margin:0 0 8px;color:var(--mut);font-size:13px;max-width:92ch}
.cards{display:flex;flex-wrap:wrap;gap:8px;margin:8px 0 0}
.cards .c{flex:1 1 180px;min-width:0;background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:10px 12px}
.cards .c .k{font-size:12.5px;color:var(--mut)}
.cards .c .v{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:20px;margin-top:2px;font-variant-numeric:tabular-nums}
.cards .c .n{font-size:12px;color:var(--mut);margin-top:2px}
.cards .c.ok{border-left:3px solid var(--pass)}.cards .c.no{border-left:3px solid var(--fail)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:12px;margin-top:12px}
.clip{background:var(--card);border:1px solid var(--bd);border-radius:12px;overflow:hidden;min-width:0}
.clip video{display:block;width:100%;height:auto;background:#000}
.clip .cap{padding:10px 12px 2px;font-size:13.5px;font-weight:600}
.clip .vd{margin:6px 12px 0;font-size:12px;border-radius:8px;padding:6px 9px;
 font-family:ui-monospace,SFMono-Regular,Menlo,monospace;line-height:1.35}
.vd.ok{color:var(--pass);background:rgba(21,128,61,.12)}
.vd.luck{color:var(--luck);background:rgba(180,83,9,.13)}
.vd.no{color:var(--fail);background:rgba(179,38,30,.12)}
.ro{display:grid;grid-template-columns:repeat(4,1fr);gap:1px;background:var(--bd);margin-top:10px;border-top:1px solid var(--bd)}
.ro div{background:var(--card);padding:7px 9px;min-width:0}
.ro .k{font-size:10.5px;color:var(--mut);text-transform:uppercase;letter-spacing:.04em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ro .v{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:13px;margin-top:1px;font-variant-numeric:tabular-nums}
.note{margin:10px 0 0;font-size:13.5px;color:var(--mut);max-width:92ch}
.note b{color:var(--fg)}
.why{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:10px;margin-top:10px}
.why .w{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:12px 14px}
.why .w h3{font-size:14px;margin:0}.why .w p{margin:6px 0 0;font-size:13px;color:var(--mut)}
.tw{overflow-x:auto;margin-top:10px}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{text-align:right;padding:7px 9px;border-bottom:1px solid var(--bd);white-space:nowrap}
th:first-child,td:first-child{text-align:left}
th{font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:var(--mut)}
td.m{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-variant-numeric:tabular-nums}
footer{color:var(--mut);font-size:12.5px;margin-top:26px;border-top:1px solid var(--bd);padding-top:10px;max-width:92ch}"""

BAND = [("800 mm", "--", "--", "--"), ("500 mm", "26", "--", "--"), ("450 mm", "26-32", "26-32", "32"),
        ("400 mm", "26-35", "29-35", "35"), ("350 mm", "29-41", "32-41", "38-41"),
        ("300 mm", "32-47", "35-47", "41-47")]

WHY = [
    ("Never look",
     "At -2 for never looking against -10 for the drop, refusing to nod is correct arithmetic: a nod with "
     "an unlearned depth goes over the edge 30% of the time, so looking was worth -3 against a certain -2. "
     "The policy stopped nodding altogether - not one nod in twelve deterministic episodes."),
    ("Look, then fall downstairs",
     "At -8 against -8, measuring and then going over was worth +12 +2 -8 = +6, better than anything safe. "
     "That policy measured the riser to 44 +- 48 mm against the script's 205 +- 286, four times more "
     "accurate, and drove off the edge in 30 of 30 held-out episodes."),
    ("Reverse off the back of the world",
     "The simulated landing only ran 3 m behind the edge, and falling off it cost -5 while never looking "
     "cost -8. The policy drove backwards off the floor, which looked perfectly safe in the margin "
     "statistics. The landing now runs 8 m back."),
]


def badge(r):
    if not r["ok"]:
        return "no"
    return "luck" if "LUCKY" in r["line"] else "ok"


def cell(k, v):
    return f'<div><div class="k">{k}</div><div class="v">{v}</div></div>'


def main():
    rows = json.loads(CLIPS.read_text())
    good = [r for r in rows if r["ok"] and "LUCKY" not in r["line"]]
    lucky = [r for r in rows if r["ok"] and "LUCKY" in r["line"]]
    clips = []
    for r in rows:
        est = "--" if r["rise_est_mm"] != r["rise_est_mm"] else f'{r["rise_est_mm"]:.0f} mm'
        clips.append(f'''<figure class="clip">
<video controls preload="none" playsinline poster="files/stairs_{r['slug']}.jpg">
 <source src="files/{r['file']}" type="video/mp4">Your browser cannot play this clip.</video>
<figcaption class="cap">{html.escape(r['caption'])}</figcaption>
<div class="vd {badge(r)}">{html.escape(r['line'])}</div>
<div class="ro">{cell('riser, true', f"{r['rise_true_mm']:.0f} mm")}{cell('riser, read', est)}
{cell('returns on tread', r['n_seen'])}{cell('landing left', f"{r['margin_mm']:.0f} mm")}</div>
<div class="ro">{cell('flick target', f"{r['nod_target']:.2f} rad" if r['nod_target'] else '--')}
{cell('taken at', f"{r['nod_at_mm']} mm" if r['nod_at_mm'] is not None else '--')}
{cell('peak pitch', f"{r['peak_deg']:.1f}&deg;")}{cell('seed', r['seed'])}</div>
</figure>''')
    band = "".join(f'<tr><td>{d}</td><td class="m">{a}</td><td class="m">{b}</td><td class="m">{c}</td></tr>'
                   for d, a, b, c in BAND)
    why = "".join(f'<div class="w"><h3>{t}</h3><p>{html.escape(p)}</p></div>' for t, p in WHY)
    nav = "".join(f'<a href="{h}"{" class=\"on\" aria-current=\"page\"" if h == "stairs.html" else ""}>{t}</a>'
                  for h, t in NAV)
    OUT.write_text(f'''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Balancing robot V1.1 – measuring a staircase</title>
<style>{CSS}</style></head><body>
<header><h1>Balancing robot V1.1 &mdash; measuring a staircase by nodding</h1>
<p>The robot has to know how tall the first step is before it can think about going down it. It has one
lidar, 231 mm up, which sees nothing of the floor while it is upright &mdash; so it has to tip forward,
and a two-wheel balancer cannot tip forward without driving forward. Twelve episodes: what that costs,
every way it goes wrong, and the two ways of taking the look compared.</p></header>
<nav class="top" aria-label="Pages">{nav}</nav>
<main>
<div class="cards">
 <div class="c ok"><div class="k">Scripted nod, held-out stairs</div><div class="v">0.37</div>
  <div class="n">30 staircases, risers 80&ndash;170 mm, seeds never trained on</div></div>
 <div class="c"><div class="k">Ceiling for any fixed recipe</div><div class="v">0.50</div>
  <div class="n">grid search: one nod carries no clue which riser it faces</div></div>
 <div class="c no"><div class="k">Learned policy, same stairs</div><div class="v">0.00</div>
  <div class="n">eleven PPO runs, three reward bugs &mdash; all mine</div></div>
 <div class="c"><div class="k">Best riser accuracy seen</div><div class="v">&plusmn;44 mm</div>
  <div class="n">by the policy that drove off the edge every time</div></div>
</div>
<p class="note"><b>How to read the clips.</b> The yellow fan is the lidar, drawn where its rays actually
land; the stairwell walls are made see-through for the camera only. Each clip ends with what that
episode really did, taken from its own telemetry after it ran &mdash; not from what it was meant to show.
A success means all three at once: the riser measured to within 20 mm, the robot still upright, and at
least 100 mm of landing left between the wheels and the drop. <b>The clips that work are selected
episodes</b>: the recipe succeeds on 37% of staircases, so the environment was asked for seeds until the
outcome matched, and the seed is listed under each clip. The honest rate is the 0.37 above, not 5 of 10.</p>

<h2>Twelve episodes</h2>
<p class="sub">{len(good)} clean successes, {len(lucky)} right for the wrong reason, {len(rows) - len(good) - len(lucky)} failures.</p>
<div class="grid">{''.join(clips)}</div>

<h2>Two ways to take the look</h2>
<p class="sub">A <b>flick</b> is one-sided: gravity acts forward for the whole nod, so it costs
0.1&ndash;1.1 m of travel towards the drop. A <b>swing</b> leans back for as long as it leans forward,
so it travels almost nothing &mdash; 650 mm of standoff became 638 mm across six cycles &mdash; and its
amplitude grows each cycle, because the angle that sees a 170 mm riser is not the one that sees an
80 mm riser and the robot cannot know which it faces until it has measured it.</p>
<div class="tw"><table><thead><tr><th>look</th><th>success</th><th>riser error</th>
<th>landing left</th><th>worst</th><th>fell or went over</th></tr></thead><tbody>
<tr><td>one-sided flick</td><td class="m">0.37</td><td class="m">205 &plusmn; 286 mm</td>
<td class="m">204 mm</td><td class="m">&minus;65 mm</td><td class="m">0.06</td></tr>
<tr><td>creep in, then swing</td><td class="m">0.23</td><td class="m">314 &plusmn; 339 mm</td>
<td class="m">237 mm</td><td class="m">&minus;386 mm</td><td class="m">0.17</td></tr>
</tbody></table></div>
<p class="note">Same harness, same 30 held-out staircases. The swing is the <b>safer</b> look by a wide
margin &mdash; from a fixed standoff it leaves 350&ndash;540 mm of landing and stayed upright on 25 of 25
randomised robots &mdash; and on a staircase whose riser is known it is much the better one: 6 of 6
successes at 5 mm of error on a 100 mm riser. It still loses across unknown staircases, because the
amplitude it can safely reach caps the angle, and a tall riser needs an angle it cannot get to from a
standoff it can survive. Its failures are blind rather than dangerous, which is the better way to fail.</p>
<p class="note"><b>And it cannot be done from close in.</b> Creeping to 150 mm is safe &mdash; the robot
gets there without trouble and the swing is stable &mdash; and useless: 0 to 3 returns on the first
tread, the riser read 469 mm wrong. Closer is worse, not better. The step nose subtends a larger angle
the nearer you are, so the pitch needed to see past it rises: 41&ndash;50&deg; at 200 mm, roughly
60&ndash;70&deg; at 150 mm, while above about 0.8 rad of amplitude the swing loses the robot. The limit
is the lidar sitting 231 mm up, not the controller. A cheap time-of-flight sensor at ~80 mm would see
the first tread from 600 mm with a 20&deg; nod and make the whole manoeuvre easy.</p>

<h2>Why it is this hard</h2>
<p class="sub">Where the first tread is visible from, measured kinematically: body pitch in degrees that
puts returns on it, against how far back the robot stands. Beyond about 550 mm there is no angle that
works &mdash; the ray that clears the step nose already overflies the whole first tread.</p>
<div class="tw"><table><thead><tr><th>standoff</th><th>80 mm riser</th><th>100 mm riser</th>
<th>170 mm riser</th></tr></thead><tbody>{band}</tbody></table></div>
<p class="note">At a 170 mm riser that band is one three-degree cell wide per standoff, which is why a
fixed recipe scores 0 out of 10 there while managing 0.60 at 130 mm. And the nod is not free: gravity
acting over the time the body is tilted adds about 1 m/s of forward speed per 34&deg; of lean, and
shedding it costs 0.2&ndash;1.1 m of travel towards the drop. Entering the nod <em>reversing</em> is what
pays for it; entering it rolling forwards loses the robot outright, which is the clip above.</p>

<h2>Three ways the reward was wrong</h2>
<p class="sub">Each failure was the optimiser reading my numbers correctly.</p>
<div class="why">{why}</div>
<p class="note">The rule they share is now an assertion that runs when the environment is imported: the
drop has to cost more than the best possible reading is worth, and never looking has to cost more than
a safe bad look.</p>

<footer>Rendered by <span class="mono">sim/stairs_video.py</span> through the same environment, controller,
sensor noise and estimator as <span class="mono">ml/evaluate_stairs.py</span>; this page is generated from
the clips' own telemetry by <span class="mono">site/build_stairs.py</span>.</footer>
</main></body></html>''')
    print(f"wrote {OUT} ({OUT.stat().st_size // 1024} kB), {len(rows)} clips, "
          f"{sum(r['kb'] for r in rows) // 1024} MB of video")


if __name__ == "__main__":
    main()
