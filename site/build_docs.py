"""Build the V1.1 docs pages: code.html (all source), docs.html (README + INSTALL), bom.html (interactive BOM)."""
import pathlib as _pl, sys as _sys
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "cad_src" / "case"))
import robot_paths as RP            # where everything is on this machine (BOT_WS still honoured)
import json, os, html, re, subprocess, shutil, base64
import markdown, openpyxl
from pygments import highlight
from pygments.lexers import get_lexer_for_filename, TextLexer
from pygments.formatters import HtmlFormatter

SITE = str(RP.SITE); F = SITE + "/files/v11/"
SRC = "/tmp/swz/robot_V1.1"
BOM = str(RP.BOM / "balancing_robot_BOM_V1.1.xlsx")

# ---------- shared look ----------
HEAD = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title><style>
:root{{--bg:#f6f7f9;--fg:#1d2330;--mut:#667085;--ac:#2f6fd0;--card:#fff;--bd:#e2e5ea;--code:#f3f4f7}}
@media (prefers-color-scheme:dark){{:root{{--bg:#15171b;--fg:#e8eaee;--mut:#9aa2ae;--ac:#6ea3ff;--card:#1f2228;--bd:#30343c;--code:#191b20}}}}
*{{box-sizing:border-box}}html,body{{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 -apple-system,system-ui,Segoe UI,Roboto,sans-serif}}
header{{padding:12px 14px 6px}}h1{{font-size:17px;margin:0 0 2px}}header p{{margin:0;color:var(--mut);font-size:13px}}a{{color:var(--ac)}}
nav.top{{display:flex;gap:6px;overflow-x:auto;padding:6px 14px 8px;scrollbar-width:none}}nav.top::-webkit-scrollbar{{display:none}}
nav.top a,button.pill{{flex:none;border:1px solid var(--bd);background:var(--card);color:var(--fg);border-radius:999px;padding:7px 13px;font:inherit;font-size:14px;text-decoration:none;cursor:pointer}}
nav.top a.on,button.pill.on{{background:var(--ac);border-color:var(--ac);color:#fff}}
.card{{margin:6px 12px 16px;background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:12px 14px;overflow:hidden}}
{extra}</style></head><body>
<header><h1>{h1}</h1><p>{sub}</p></header>
<nav class="top" aria-label="Pages"><a href="index.html">3D viewer</a><a href="wiring_boards.html">Wiring diagram</a><a href="print.html">How it prints</a><a href="docs.html"{on_docs}>README &amp; install</a><a href="code.html"{on_code}>Source code</a><a href="bom.html"{on_bom}>Parts list</a></nav>
"""
def head(title, h1, sub, page, extra=""):
    on = {k: (' class="on" aria-current="page"' if k == page else "") for k in ("docs", "code", "bom")}
    return HEAD.format(title=title, h1=h1, sub=sub, extra=extra, on_docs=on["docs"], on_code=on["code"], on_bom=on["bom"])

# ---------- 1. software zip (with INSTALL.md + updated README) ----------
shutil.rmtree("/tmp/swz", ignore_errors=True); os.makedirs("/tmp/swz")
# What must NOT be copied, and why. This used to be a plain copytree of the whole working tree,
# which quietly swept in .venv (~2.4 GB, so the build took minutes and the zip was nonsense) and
# models/bought - the vendor STEP models, which are not ours to redistribute (the MF6015 one comes
# from the LKMTECH product page). The zip is published, so this list is a licence matter, not tidiness.
SKIP = {".git", ".venv", "venv", "env", "node_modules", "__pycache__", ".pytest_cache",
        ".mypy_cache", ".ruff_cache", ".DS_Store", "bought"}
SKIP_PATH = {"site/files", "site/vendor", "ml/runs"}   # the downloads themselves, vendored three.js, RL checkpoints
BIG = 1_000_000   # every file this size in this repo is a GENERATED blob - the viewer's base64 part
                  # data, the built pages, the exported meshes - and all of them are rebuildable from
                  # what is in the zip. Left in, they took it from 3 MB to 25 MB, and the base64 copy
                  # the site serves to 32 MB, past the 16 MB a published file may be.
def _skip(d, names):
    rel = os.path.relpath(d, RP.REPO)
    out = {n for n in names if n in SKIP}
    for n in names:
        full, r = os.path.join(d, n), (n if rel == "." else os.path.normpath(os.path.join(rel, n)))
        if r in SKIP_PATH or (os.path.isfile(full) and os.path.getsize(full) >= BIG):
            out.add(n)
    return out
shutil.copytree(RP.REPO, SRC, symlinks=True, ignore=_skip)
for d in ["firmware/esp32/.pio", "cad", "sim/results/dock_frames", "sim/results/undock_frames", "sim/results/frames"]:   # CAD files: in V1.1_print_files.zip
    shutil.rmtree(os.path.join(SRC, d), ignore_errors=True)
for root, dirs, files in os.walk(SRC):
    for d in list(dirs):
        if d in ("__pycache__", ".pytest_cache"): shutil.rmtree(os.path.join(root, d)); dirs.remove(d)
    for f in files:
        if f.endswith(".so") or f.endswith(".npy") or f.endswith(".mp4") or f == "tmp.xml": os.remove(os.path.join(root, f))   # videos: separate downloads
os.makedirs(SRC + "/cad_scripts", exist_ok=True)
for p in [RP.source("body_case.py"), RP.source("assemble.py"), RP.source("export_viewer_data.py"),
          RP.WHEEL_ADAPTER / "wheel_adapter.py", RP.BOM / "build_bom.py",
          RP.source("load_models.py"), RP.source("real_parts.py"), RP.source("real_assembly.py"),
          RP.source("wiring.py"), RP.source("dump_wiring.py"), RP.source("wire_check.py"), RP.source("export_viewer_real.py"),
          RP.source("export_print_data.py"), RP.source("perfboard_layout.py"), RP.SITE / "build_boards.py", RP.SITE / "build_dock_wiring.py",
          RP.artifact("real_fit_report.json"), RP.artifact("wire_report.json")]:
    shutil.copy(p, SRC + "/cad_scripts/")
if os.path.exists("/tmp/robot_software_V1.1.zip"): os.remove("/tmp/robot_software_V1.1.zip")
subprocess.run(["zip", "-qr", "/tmp/robot_software_V1.1.zip", "robot_V1.1"], cwd="/tmp/swz", check=True)
json.dump({"name": "robot_software_V1.1.zip", "b64": base64.b64encode(open("/tmp/robot_software_V1.1.zip", "rb").read()).decode()},
          open(F + "robot_software_V1.1.zip.json", "w"))
shutil.copy(SRC + "/README.md", F + "README.md"); shutil.copy(SRC + "/INSTALL.md", F + "INSTALL.md")
zip_mb = os.path.getsize("/tmp/robot_software_V1.1.zip") / 1e6

# ---------- 2. code.html ----------
TEXT_EXT = {".py", ".cpp", ".hpp", ".h", ".ini", ".sh", ".json", ".md", ".txt", ".html", ".service", ".xml", ".log"}
ORDER = ["README.md", "INSTALL.md", "core/", "firmware/", "pi/", "sim/", "cad_scripts/"]
files = []
for root, _, fs in os.walk(SRC):
    for f in fs:
        p = os.path.join(root, f); rel = os.path.relpath(p, SRC)
        ext = os.path.splitext(f)[1]
        if ext not in TEXT_EXT or "/results/" in "/" + rel or "/meshes/" in rel: continue
        if rel.startswith("cad/"): continue
        # The site's built pages are OUTPUT, not source, and pygments inflates each one about 5x:
        # wiring_boards.html alone cost 1.6 MB of the bundle. The sources are the builders and the
        # *_tpl.html templates, which stay. (source.json is published, and a published file is
        # capped at 16 MB - with the pages in, it was 17.)
        if rel.startswith("site/") and ext == ".html" and not rel.endswith("_tpl.html"): continue
        files.append(rel)
def key(r):
    for i, o in enumerate(ORDER):
        if r == o or r.startswith(o): return (i, r.count("/"), r)
    return (99, 0, r)
files.sort(key=key)
fmt = HtmlFormatter(nowrap=True, classprefix="")
out = []
for rel in files:
    txt = open(os.path.join(SRC, rel), encoding="utf-8", errors="replace").read()
    try:
        lx = get_lexer_for_filename(rel, stripnl=False)
    except Exception:
        lx = TextLexer(stripnl=False)
    if rel.endswith(".service"): lx = get_lexer_for_filename("x.ini", stripnl=False)
    out.append({"p": rel, "n": txt.count("\n") + 1, "h": highlight(txt, lx, fmt)})
json.dump(out, open(F + "source.json", "w"))
src_kb = os.path.getsize(F + "source.json") / 1e3
tot_lines = sum(o["n"] for o in out)

css_light = HtmlFormatter(style="default").get_style_defs(".hl")
css_dark = HtmlFormatter(style="github-dark").get_style_defs(".hl")
css_dark = "@media (prefers-color-scheme:dark){" + re.sub(r"\.hl\s*\{[^}]*\}", "", css_dark) + "}"
css_light = re.sub(r"\.hl\s*\{[^}]*\}", "", css_light)
CODE_CSS = css_light + css_dark + """
.hl{background:transparent}
#layout{display:flex;gap:0;margin:0 12px 16px;border:1px solid var(--bd);border-radius:12px;background:var(--card);overflow:hidden;min-height:70vh}
#tree{width:260px;flex:none;border-right:1px solid var(--bd);overflow:auto;max-height:80vh;font-size:13px;padding:6px 0}
#tree .dir{color:var(--mut);font-weight:600;padding:8px 12px 2px}
#tree a{display:block;padding:5px 12px 5px 20px;color:var(--fg);text-decoration:none;word-break:break-all;min-height:32px}
#tree a.on{background:var(--ac);color:#fff}
#main{flex:1;min-width:0;display:flex;flex-direction:column}
#bar{display:flex;align-items:center;gap:8px;padding:8px 12px;border-bottom:1px solid var(--bd);flex-wrap:wrap}
#bar select{flex:1;min-width:0;font:inherit;font-size:14px;padding:8px;border-radius:8px;border:1px solid var(--bd);background:var(--bg);color:var(--fg);display:none}
#path{font:600 14px ui-monospace,SFMono-Regular,Menlo,monospace;word-break:break-all;flex:1}
#meta{color:var(--mut);font-size:12px}
#bar button{border:1px solid var(--bd);background:var(--bg);color:var(--fg);border-radius:8px;padding:7px 12px;font:inherit;font-size:13px;cursor:pointer;min-height:36px}
#code{overflow:auto;flex:1;background:var(--code)}
#code table{border-collapse:collapse;font:12.5px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#code td.ln{color:var(--mut);text-align:right;padding:0 10px 0 12px;user-select:none;vertical-align:top;white-space:pre;border-right:1px solid var(--bd)}
#code td.src{padding:0 14px;white-space:pre;vertical-align:top}
#find{font:inherit;font-size:14px;padding:7px 10px;border-radius:8px;border:1px solid var(--bd);background:var(--bg);color:var(--fg);width:100%;margin:0 0 4px}
#treebox{display:flex;flex-direction:column}
@media (max-width:760px){#tree{display:none}#bar select{display:block}#path{display:none}#layout{min-height:60vh}}
"""
code_html = head("Balancing robot V1.1 – source code", "Balancing robot V1.1 — source code",
                 f"{len(out)} files, {tot_lines:,} lines: ESP32 firmware, shared C++ controller, Pi software with voice, simulation, CAD scripts. "
                 f"Download everything as robot_software_V1.1.zip from the 3D viewer page.", "code", CODE_CSS) + """
<div id="layout">
 <div id="tree" aria-label="Files"><div style="padding:6px 10px"><input id="find" type="search" placeholder="Filter files" aria-label="Filter files"></div><div id="list"></div></div>
 <div id="main">
  <div id="bar"><select id="sel" aria-label="File"></select><span id="path"></span><span id="meta"></span><button id="copy" type="button">Copy</button></div>
  <div id="code" tabindex="0" aria-label="File content"><p style="padding:14px;color:var(--mut)">Loading…</p></div>
 </div>
</div>
<script>
let FILES=[],cur=-1;
const $=id=>document.getElementById(id);
function dirOf(p){const i=p.lastIndexOf('/');return i<0?'(top level)':p.slice(0,i)}
function buildNav(filter){
  const L=$('list');L.innerHTML='';let last=null;const q=(filter||'').toLowerCase();
  FILES.forEach((f,i)=>{if(q&&!f.p.toLowerCase().includes(q))return;const d=dirOf(f.p);
    if(d!==last){const h=document.createElement('div');h.className='dir';h.textContent=d+'/';L.appendChild(h);last=d}
    const a=document.createElement('a');a.href='#'+encodeURIComponent(f.p);a.textContent=f.p.split('/').pop();a.dataset.i=i;if(i===cur)a.className='on';L.appendChild(a)});
}
function show(i){
  if(i<0||i>=FILES.length)return;cur=i;const f=FILES[i];
  $('path').textContent=f.p;$('meta').textContent=f.n+' lines';$('sel').value=String(i);
  const ln=Array.from({length:f.n},(_,k)=>k+1).join('\\n');
  $('code').innerHTML='<table><tr><td class="ln">'+ln+'</td><td class="src hl">'+f.h+'</td></tr></table>';
  $('code').scrollTop=0;
  document.querySelectorAll('#list a').forEach(a=>a.classList.toggle('on',+a.dataset.i===i));
  document.title=f.p+' – V1.1 source';
}
function fromHash(){const p=decodeURIComponent(location.hash.slice(1));const i=FILES.findIndex(f=>f.p===p);show(i>=0?i:0)}
fetch('files/v11/source.json').then(r=>{if(!r.ok)throw new Error(r.status);return r.json()}).then(d=>{
  FILES=d;const S=$('sel');let last=null,og=null;
  d.forEach((f,i)=>{const dd=dirOf(f.p);if(dd!==last){og=document.createElement('optgroup');og.label=dd+'/';S.appendChild(og);last=dd}
    const o=document.createElement('option');o.value=i;o.textContent=f.p.split('/').pop();og.appendChild(o)});
  buildNav('');fromHash();
}).catch(e=>{$('code').innerHTML='<p style="padding:14px">Could not load the source ('+e.message+'). Download robot_software_V1.1.zip from the 3D viewer page.</p>'});
window.addEventListener('hashchange',fromHash);
$('sel').addEventListener('change',e=>{location.hash=encodeURIComponent(FILES[+e.target.value].p)});
$('find').addEventListener('input',e=>buildNav(e.target.value));
$('copy').addEventListener('click',async()=>{const f=FILES[cur];if(!f)return;
  try{await navigator.clipboard.writeText(document.querySelector('#code td.src').textContent);$('copy').textContent='Copied'}catch(_){
    const r=document.createRange();r.selectNodeContents(document.querySelector('#code td.src'));const s=getSelection();s.removeAllRanges();s.addRange(r);$('copy').textContent='Selected'}
  setTimeout(()=>$('copy').textContent='Copy',1500)});
</script></body></html>"""
open(SITE + "/code.html", "w").write(code_html)

# ---------- 3. docs.html ----------
def md(path):
    t = open(path, encoding="utf-8").read()
    return markdown.markdown(t, extensions=["tables", "fenced_code", "sane_lists"])
DOC_CSS = """
.tabs{display:flex;gap:6px;padding:0 14px 4px}
.doc{font-size:15px;line-height:1.55}.doc h1{font-size:20px;margin:6px 0 10px}.doc h2{font-size:17px;margin:22px 0 8px;padding-top:10px;border-top:1px solid var(--bd)}.doc h3{font-size:15px;margin:16px 0 6px}
.doc pre{background:var(--code);border:1px solid var(--bd);border-radius:8px;padding:10px 12px;overflow-x:auto;font:12.5px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
.doc code{font:13px ui-monospace,SFMono-Regular,Menlo,monospace;background:var(--code);padding:1px 4px;border-radius:4px}.doc pre code{background:none;padding:0}
.doc .tw{overflow-x:auto;margin:8px 0}.doc table{border-collapse:collapse;font-size:13.5px;min-width:100%}.doc th,.doc td{border:1px solid var(--bd);padding:6px 8px;text-align:left;vertical-align:top}.doc th{background:var(--code)}
.doc hr{display:none}.doc li{margin:3px 0}
.pane[hidden]{display:none}
"""
def wrap_tables(h): return h.replace("<table>", '<div class="tw"><table>').replace("</table>", "</table></div>")
docs_html = head("Balancing robot V1.1 – README & install", "Balancing robot V1.1 — README &amp; installation",
                 "Both files are also in robot_software_V1.1.zip.", "docs", DOC_CSS) + f"""
<div class="tabs" role="tablist"><button class="pill on" role="tab" aria-selected="true" data-p="install">Installation guide</button><button class="pill" role="tab" aria-selected="false" data-p="readme">README</button></div>
<div class="card doc pane" id="install" role="tabpanel">{wrap_tables(md(SRC + '/INSTALL.md'))}</div>
<div class="card doc pane" id="readme" role="tabpanel" hidden>{wrap_tables(md(SRC + '/README.md'))}</div>
<script>
function tab(p){{document.querySelectorAll('.tabs button').forEach(b=>{{const on=b.dataset.p===p;b.classList.toggle('on',on);b.setAttribute('aria-selected',on)}});
document.querySelectorAll('.pane').forEach(x=>x.hidden=x.id!==p)}}
document.querySelectorAll('.tabs button').forEach(b=>b.addEventListener('click',()=>{{tab(b.dataset.p);history.replaceState(null,'','#'+b.dataset.p)}}));
if(location.hash==='#readme')tab('readme');
</script></body></html>"""
open(SITE + "/docs.html", "w").write(docs_html)

# ---------- 4. bom.html ----------
wb = openpyxl.load_workbook(BOM); ws = wb["BOM"]
rows = []; cat = None
for r in ws.iter_rows(min_row=5, max_row=ws.max_row):
    v = [c.value for c in r]
    if v[0] and not isinstance(v[0], int): cat = v[0]; continue
    if not isinstance(v[0], int): continue
    link = r[16].hyperlink.target if r[16].hyperlink else None
    rows.append({"n": v[0], "cat": cat, "comp": v[2], "part": v[3], "spec": v[4] or "", "size": v[5] or "", "loc": v[7] or "",
                 "qty": v[9] or 0, "price": v[10] or 0, "mass": v[12] or 0, "req": v[14] or "", "src": v[15] or "",
                 "shop": v[16] or "", "link": link})
def table(name, skip=3):
    s = wb[name]; vals = [r for r in s.iter_rows(min_row=skip, values_only=True) if any(x is not None for x in r)]
    return vals
wiring = table("Wiring"); build = table("Build check"); reqs = table("Requirements")
screws = [r for r in table("Screws", 4) if r[0] is not None]
BOM_CSS = """
#tot{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px;margin:6px 12px 10px}
#tot div{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:10px 12px}#tot b{display:block;font-size:20px}#tot span{color:var(--mut);font-size:12px}
.tabs{display:flex;gap:6px;padding:0 14px 6px;overflow-x:auto}
.ctl{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:0 12px 8px;font-size:13px;color:var(--mut)}
.ctl label{display:flex;gap:6px;align-items:center;min-height:36px}.ctl button{border:1px solid var(--bd);background:var(--card);color:var(--fg);border-radius:8px;padding:7px 12px;font:inherit;cursor:pointer;min-height:36px}
h2.cat{font-size:14px;color:var(--mut);margin:16px 14px 6px;text-transform:uppercase;letter-spacing:.03em}
.it{margin:0 12px 8px;background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:10px 12px}
.it.opt{opacity:.75}.it h3{font-size:15px;margin:0 0 2px}.it .part{font-size:14px}.it .spec{color:var(--mut);font-size:13px;margin:4px 0}
.it details{font-size:13px;color:var(--mut)}.it summary{cursor:pointer;min-height:28px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-top:6px;font-size:14px}
.row input{width:64px;font:inherit;font-size:15px;padding:6px 8px;border-radius:8px;border:1px solid var(--bd);background:var(--bg);color:var(--fg)}
.row input.p{width:84px}.row .lt{font-weight:600;margin-left:auto}
.badge{font-size:11px;border-radius:999px;padding:2px 8px;border:1px solid var(--bd);color:var(--mut)}.badge.y{background:var(--ac);border-color:var(--ac);color:#fff}
.tw{overflow-x:auto;margin:0 12px 16px;background:var(--card);border:1px solid var(--bd);border-radius:12px}
.tw table{border-collapse:collapse;font-size:13px;min-width:640px}.tw th,.tw td{border-bottom:1px solid var(--bd);padding:7px 9px;text-align:left;vertical-align:top}.tw th{position:sticky;top:0;background:var(--card)}
.pane[hidden]{display:none}
"""
def esc(x): return html.escape(str(x)) if x is not None else ""
def htable(vals):
    h = "<tr>" + "".join(f"<th>{esc(c)}</th>" for c in vals[0] if c is not None) + "</tr>"
    n = sum(1 for c in vals[0] if c is not None)
    b = "".join("<tr>" + "".join(f"<td>{esc(c)}</td>" for c in r[:n]) + "</tr>" for r in vals[1:])
    return f'<div class="tw"><table>{h}{b}</table></div>'
bom_html = head("Balancing robot V1.1 – parts list", "Balancing robot V1.1 — parts list",
                "Edit quantities and prices: totals update live and are kept in this browser. Prices in EUR incl. VAT, Nov 2026; "
                "the .xlsx with all columns (Asia prices, mounting, CAD check) is on the 3D viewer page.", "bom", BOM_CSS) + f"""
<div id="tot"><div><b id="t_req">–</b><span>Required parts</span></div><div><b id="t_all">–</b><span>Incl. optional + tools</span></div><div><b id="t_mass">–</b><span>Mass, required parts</span></div><div><b id="t_n">–</b><span>Lines</span></div></div>
<div class="tabs" role="tablist"><button class="pill on" data-p="parts" role="tab" aria-selected="true">Parts</button><button class="pill" data-p="wiring" role="tab" aria-selected="false">Wiring</button><button class="pill" data-p="screws" role="tab" aria-selected="false">Screws</button><button class="pill" data-p="build" role="tab" aria-selected="false">Build order</button><button class="pill" data-p="reqs" role="tab" aria-selected="false">Requirements</button></div>
<div class="pane" id="parts" role="tabpanel">
<div class="ctl"><label><input type="checkbox" id="onlyreq"> Required parts only</label><button type="button" id="reset">Reset my edits</button><button type="button" id="csv">Copy as CSV</button><span id="msg" role="status" aria-live="polite"></span></div>
<div id="items"></div></div>
<div class="pane" id="wiring" role="tabpanel" hidden>{htable(wiring)}</div>
<div class="pane" id="build" role="tabpanel" hidden>{htable(build)}</div>
<div class="pane" id="screws" role="tabpanel" hidden><p class="ctl">Every fastener in the build. &quot;Buy&quot; includes spares. One A2 hex-socket assortment (M2/M2.5/M3/M4) + 25 × M3×10 ISO 10642 countersunk + 2 brass M4×10 countersunk covers it.</p>{htable(screws)}</div>
<div class="pane" id="reqs" role="tabpanel" hidden>{htable(reqs)}</div>
<script>
const ROWS={json.dumps(rows, ensure_ascii=False)};
const KEY='bom_v11_edits';let E={{}};
try{{E=JSON.parse(localStorage.getItem(KEY)||'{{}}')||{{}}}}catch(_){{E={{}}}}
function save(){{try{{localStorage.setItem(KEY,JSON.stringify(E))}}catch(_){{}}}}
const eur=x=>'€'+x.toLocaleString('en-US',{{minimumFractionDigits:0,maximumFractionDigits:2}});
const q=r=>E[r.n]&&E[r.n].qty!=null?E[r.n].qty:r.qty, p=r=>E[r.n]&&E[r.n].price!=null?E[r.n].price:r.price;
const req=r=>String(r.req).toLowerCase().startsWith('yes');
function esc(s){{return String(s).replace(/[&<>"]/g,c=>({{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}})[c])}}
function totals(){{let a=0,b=0,m=0;ROWS.forEach(r=>{{const lt=q(r)*p(r);b+=lt;if(req(r)){{a+=lt;m+=q(r)*r.mass}}}});
$('t_req').textContent=eur(Math.round(a));$('t_all').textContent=eur(Math.round(b));$('t_mass').textContent=(m/1000).toFixed(2)+' kg';$('t_n').textContent=ROWS.length}}
const $=id=>document.getElementById(id);
function render(){{const only=$('onlyreq').checked;let h='',last=null;
ROWS.forEach(r=>{{if(only&&!req(r))return;if(r.cat!==last){{h+='<h2 class="cat">'+esc(r.cat)+'</h2>';last=r.cat}}
 const shop=r.link?'<a href="'+esc(r.link)+'" target="_blank" rel="noopener">'+esc(r.shop||'Buy')+'</a>':esc(r.shop);
 h+='<div class="it'+(req(r)?'':' opt')+'"><h3>'+r.n+'. '+esc(r.comp)+' <span class="badge'+(req(r)?' y':'')+'">'+esc(r.req)+'</span></h3>'
 +'<div class="part">'+esc(r.part)+'</div>'+(r.spec?'<div class="spec">'+esc(r.spec)+'</div>':'')
 +'<details><summary>Size, location, source</summary>'+(r.size?'<div>Size: '+esc(r.size)+'</div>':'')+(r.loc?'<div>Where: '+esc(r.loc)+'</div>':'')+(r.src?'<div>Note: '+esc(r.src)+'</div>':'')+'</details>'
 +'<div class="row"><label>Qty <input type="number" min="0" step="1" inputmode="numeric" data-n="'+r.n+'" data-f="qty" value="'+q(r)+'"></label>'
 +'<label>€ <input class="p" type="number" min="0" step="0.01" inputmode="decimal" data-n="'+r.n+'" data-f="price" value="'+p(r)+'"></label>'
 +'<span>'+shop+'</span><span class="lt" id="lt'+r.n+'">'+eur(q(r)*p(r))+'</span></div></div>'}});
$('items').innerHTML=h;totals()}}
$('items').addEventListener('input',e=>{{const t=e.target;if(!t.dataset.n)return;const n=+t.dataset.n,v=parseFloat(t.value);
 E[n]=E[n]||{{}};E[n][t.dataset.f]=isNaN(v)?0:v;save();const r=ROWS.find(x=>x.n===n);$('lt'+n).textContent=eur(q(r)*p(r));totals()}});
$('onlyreq').addEventListener('change',render);
$('reset').addEventListener('click',()=>{{E={{}};save();render();$('msg').textContent='Back to the original prices.'}});
$('csv').addEventListener('click',async()=>{{const L=[['#','Category','Component','Part','Qty','Unit EUR','Line EUR','Required','Shop','Link'].join(',')];
 ROWS.forEach(r=>L.push([r.n,r.cat,r.comp,r.part,q(r),p(r),(q(r)*p(r)).toFixed(2),r.req,r.shop,r.link||''].map(x=>'"'+String(x).replace(/"/g,'""')+'"').join(',')));
 try{{await navigator.clipboard.writeText(L.join('\\n'));$('msg').textContent='Copied: paste into any spreadsheet.'}}catch(_){{$('msg').textContent='Copy not allowed here.'}}}});
document.querySelectorAll('.tabs button').forEach(b=>b.addEventListener('click',()=>{{document.querySelectorAll('.tabs button').forEach(x=>{{const on=x===b;x.classList.toggle('on',on);x.setAttribute('aria-selected',on)}});
 document.querySelectorAll('.pane').forEach(x=>x.hidden=x.id!==b.dataset.p)}}));
render();
</script></body></html>"""
open(SITE + "/bom.html", "w").write(bom_html)
req_total = sum(r["qty"] * r["price"] for r in rows if str(r["req"]).lower().startswith("yes"))
print(f"zip {zip_mb:.1f} MB, source {len(out)} files {tot_lines} lines {src_kb:.0f} kB, bom rows {len(rows)} req €{req_total:.0f}")
