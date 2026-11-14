"""Build the site page for a NON-DEFAULT motor profile, from site/index.html.

The committed build is one build: site/index.html, sim/results/report.json, cad/ and the BOM all
describe whichever `ROBOT_MOTOR` is the default (today mf6015). A second profile is still buildable
from the same CAD, and this makes the second one visible without pretending it is the shipped one.

    ROBOT_MOTOR=mf7015 ROBOT_OUT=/tmp/out7015 cad_src/case/export_viewer_real.py   # first
    python tools/site_variant.py mf7015 /tmp/out7015

It takes site/index.html and swaps the two data lines (`const PARTS=` / `const WIRES=`, exactly the
two that tools/site_rebuild.py writes) for that profile's exports, retitles the page, and replaces
the download section with a note saying what is different and what has NOT been verified. Everything
else - the viewer, the CSS, the vendored three.js importmap - is shared, so the variant page cannot
drift from the main one.
"""
import pathlib, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SITE = ROOT / "site"

NOTE = {"mf7015": """<section id="dl" aria-labelledby="dlh">
<h2 id="dlh">What this build is</h2>
<p style="margin:0 0 8px;font-size:13px">The same robot with <b>LKMTECH MF7015-V2 23T</b> hub motors
in place of the MF6015-V2 15T: Ø69&nbsp;&times;&nbsp;33.9&nbsp;mm instead of Ø58&nbsp;&times;&nbsp;34.0,
360&nbsp;g instead of 251&nbsp;g, 3.8&nbsp;N&middot;m peak instead of 2.3. Both motors build from the
same CAD, selected by one constant (<code>ROBOT_MOTOR</code>), so every part here is generated from
the same scripts as the main page.</p>
<h3>What changes</h3>
<ul style="margin:0 0 8px;padding-left:20px;font-size:13px">
<li><b>Axially identical</b>, so the wheel track, the body width, the panels and the dock's saddle do
not move at all. That one fact removes most of the redesign.</li>
<li><b>Radially +5.5&nbsp;mm</b>: the body goes 11&nbsp;mm deeper, front and back. Ground clearance
19.3&nbsp;&rarr;&nbsp;13.8&nbsp;mm.</li>
<li><b>And 15.5&nbsp;mm taller</b>, which is the expensive part. The shelf height is clearance over the
motor's <i>leads</i>, and this motor's connector is a socket in the side rim whose mated plug tops out
10&nbsp;mm above the can, where the MF6015's sits level with it. 18&nbsp;mm over that puts the shelf at
62.5 instead of 47, so the whole upper body follows it up and the centre of mass rises &mdash; against a
build whose CoM is already the weak point. The motor's own 109&nbsp;g per side sits on the axle and
pulls the other way.</li>
<li>The wheel adapter keeps the Ø12.70 spigot and the Ø25 bolt circle; only the output bolts change,
M3&nbsp;&rarr;&nbsp;M4.</li>
<li>Every harness is re-routed for the higher shelf. The fit check reports <b>3 contacts against the
MF6015's 4</b> &mdash; the deeper body resolves the Raspberry Pi OTG clash, the only real one.</li>
<li>The HC-SR04, which is the <i>alternative</i> to the zone array, has its pin header on the underside
of its board, and on this body the raised shelf comes to 4&nbsp;mm below that board. Fitted here it needs
a right-angle header, or the board moved up about 14&nbsp;mm. The fitted front sensor, the VL53L5CX, is
clear. This is the only thing <code>wire_check.py</code> still reports on this body.</li>
</ul>
<h3>What is NOT verified</h3>
<p style="margin:0 0 8px;font-size:13px">This is the mechanical design only. The plant model, the LQR
gains, the controller's torque and wheel-speed limits, the firmware and both learned policies are
still the MF6015's, and the motor has not been bought or bench-tested: the encoder reply scaling, the
drive's current limit and the wheel directions all need a bench. Pick the LKMTECH
<b>MF7015-V2</b> with the DF70V2 drive (7.4&ndash;32&nbsp;V, LK V2 protocol), not the MyActuator
RMD-L-7015-200 (MC200: 12&ndash;24&nbsp;V and V3 &mdash; a 6S pack at 25.2&nbsp;V over-volts it).</p>
<p style="margin:0;font-size:13px"><a href="index.html">&larr; back to the V1.1 build (MF6015)</a></p>
</section>"""}

TITLE = {"mf7015": "Balancing robot V1.1 (MF7015) – 3D parts viewer"}
H1 = {"mf7015": """<header><h1>Balancing robot V1.1 &mdash; MF7015 build</h1><p class="ver">Same robot on the bigger \
&Oslash;69 hub motors: 11 mm deeper body, shelf and lid 5.5 mm up &middot; mechanical design only, \
not yet built &middot; <a href="index.html">V1.1 as built (MF6015)</a></p><p>Drag to rotate \
&middot; pinch or scroll to zoom &middot; two fingers to pan &middot; tap a part for details</p></header>"""}


def build(profile, out_dir):
    out_dir = pathlib.Path(out_dir)
    parts = (out_dir / "viewer_parts_v11.json").read_text()
    wires = (out_dir / "wiring_data.json").read_text()
    src = (SITE / "index.html").read_text()
    lines = src.split("\n")
    swapped = 0
    for i, l in enumerate(lines):
        if l.startswith("const PARTS="):
            lines[i] = "const PARTS=" + parts + ";"; swapped += 1
        elif l.startswith("const WIRES="):
            lines[i] = "const WIRES=" + wires + ";"; swapped += 1
    if swapped != 2:
        raise SystemExit("site/index.html: found %d of the 2 data lines - has it been rebuilt?" % swapped)
    s = "\n".join(lines)
    # header
    i0 = s.index("<header>"); i1 = s.index("</header>") + len("</header>")
    s = s[:i0] + H1[profile] + s[i1:]
    # title
    s = s.replace("<title>Balancing robot V1.1 – 3D parts viewer</title>",
                  "<title>%s</title>" % TITLE[profile], 1)
    # download section -> the note
    j0 = s.index('<section id="dl"'); j1 = s.index("</section>", j0) + len("</section>")
    s = s[:j0] + NOTE[profile] + s[j1:]
    dst = SITE / (profile + ".html")
    dst.write_text(s)
    return dst, len(s)


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    profile, out_dir = sys.argv[1], sys.argv[2]
    if profile not in NOTE:
        raise SystemExit("no page copy written for profile %r (add it to NOTE/TITLE/H1)" % profile)
    dst, n = build(profile, out_dir)
    print("wrote %s (%d kB)" % (dst, n // 1024))


if __name__ == "__main__":
    main()
