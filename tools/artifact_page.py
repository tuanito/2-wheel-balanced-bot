"""Build the artifact copy of site/index.html: de-skeletoned AND self-contained.

Two separate traps live here, and the viewer has been dark because of them.

1. **The double wrap.** The Artifact tool wraps whatever it publishes in its own
   `<!doctype html><html><head>`, and site/index.html is already a complete document (it has to be,
   site/ is served directly too). Publishing it raw nests one document inside the other and the
   importmap ends up inside the body, where it does nothing. So the skeleton tags are stripped.

2. **Module resolution.** Even de-skeletoned, the page imported `three` through an importmap - first
   from a CDN, then from `./vendor/`. An importmap value resolves against the document's base URL,
   and an artifact page's base URL is not something this script can know; get it wrong and the import
   404s, the module never runs, and the canvas stays blank with nothing in the page to show for it.
   So the artifact copy has NO imports at all: three.js and OrbitControls are inlined ahead of the
   viewer code, their export/import statements removed, and `THREE` is rebuilt as a plain object from
   three's own export list. Nothing left to resolve.

The repo's site/index.html keeps the importmap and site/vendor/, which is the right shape for
`python3 -m http.server -d site` and for any ordinary web host.

  python tools/artifact_page.py [out.html] [src.html] [--standalone]

`--standalone` keeps the document skeleton, for a page published as a SUPPORTING file
(the Artifact tool's `files`): those are served unwrapped, so trap 2 does not apply to
them - but trap 1 does, because their base URL is just as unknowable.
"""
import pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SITE = ROOT / "site"
DEFAULT_OUT = pathlib.Path("/private/tmp/claude-502/-Users-tuan-data-projects-2-wheel-balanced-bot/"
                           "b4c1c743-8a14-49ee-8c13-d1a132f33e0f/scratchpad/publish/index.html")


def three_bundle():
    """three.js + OrbitControls as plain script text, with every import and export removed."""
    three = (SITE / "vendor" / "three.module.js").read_text()
    orbit = (SITE / "vendor" / "addons" / "controls" / "OrbitControls.js").read_text()

    m = re.search(r"^export \{([^}]*)\};\s*$", three, re.M)
    if not m:
        raise SystemExit("three.module.js: no trailing export block - check the version")
    names = [n.strip() for n in m.group(1).split(",") if n.strip()]
    if any(" as " in n for n in names):
        raise SystemExit("three.module.js exports an alias; the THREE object below would be wrong")
    three = three[:m.start()] + three[m.end():]

    orbit = re.sub(r"^import \{[^}]*\} from ['\"]three['\"];\s*", "", orbit, flags=re.M | re.S)
    orbit = re.sub(r"^export \{[^}]*\};\s*$", "", orbit, flags=re.M)
    if "import" in orbit.split("\n")[0]:
        raise SystemExit("OrbitControls: import block not removed")
    # OrbitControls goes in a closure, NOT at the top level beside three.js. Both declare `_ray`
    # (and others) at module scope, and concatenating them raises "Identifier '_ray' has already
    # been declared" - which kills the whole module, so the canvas stays blank with no clue why.
    # node --check caught it; a browser would just have shown nothing, again.
    return (three
            + "\nconst THREE = {" + ", ".join(names) + "};\n"
            + "const OrbitControls = (function(){\n" + orbit + "\nreturn OrbitControls;\n})();\n")


def build(src_html=None, keep_skeleton=False):
    """src_html: a path, or the HTML itself. keep_skeleton: for a page published as a SUPPORTING
    file (via the Artifact tool's `files`), which is served unwrapped and therefore needs to stay a
    complete document - trap 2 does not apply to it, but trap 1 still does."""
    s = (src_html or (SITE / "index.html")).read_text() if not isinstance(src_html, str) else src_html
    # 1) no imports: drop the importmap and the two import lines, inline the libraries instead
    s = re.sub(r'<script type="importmap">.*?</script>\s*', "", s, flags=re.S)
    s = re.sub(r"^\s*import \* as THREE from ['\"]three['\"];\s*$", "", s, flags=re.M)
    s = re.sub(r"^\s*import \{OrbitControls\} from ['\"][^'\"]*OrbitControls\.js['\"];\s*$", "",
               s, flags=re.M)
    s = s.replace('<script type="module">', "<script type=\"module\">\n" + three_bundle(), 1)
    if keep_skeleton:
        return s.strip() + "\n"
    # 2) no nested document: the Artifact tool supplies the skeleton for the PAGE it publishes
    for pat in (r"(?is)<!doctype[^>]*>", r"(?is)</?html[^>]*>", r"(?is)</?head[^>]*>",
                r"(?is)<body[^>]*>", r"(?is)</body>"):
        s = re.sub(pat, "", s)
    return s.strip() + "\n"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    standalone = "--standalone" in sys.argv          # keep the skeleton: published via `files`
    src = pathlib.Path(args[1]) if len(args) > 1 else None
    out = pathlib.Path(args[0]) if args else DEFAULT_OUT
    s = build(src, keep_skeleton=standalone)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(s)
    low = s.lower()
    bad = []
    if not standalone and (low.count("<!doctype") or low.count("<html") or low.count("<head>")):
        bad.append("skeleton tags survived")
    if standalone and not low.count("<!doctype"):
        bad.append("--standalone but no doctype: a supporting file is served unwrapped")
    if "importmap" in low:
        bad.append("importmap survived")
    if re.search(r"^\s*import\s", s, re.M):
        bad.append("an import statement survived")
    if "new THREE.WebGLRenderer" not in s or "class OrbitControls" not in s:
        bad.append("the bundle did not land")
    print(f"wrote {out} ({len(s)//1024} kB)")
    print("  checks:", "FAILED - " + "; ".join(bad) if bad else "clean: no skeleton, no imports, "
          "three and OrbitControls inlined")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
