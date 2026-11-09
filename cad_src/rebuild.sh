#!/usr/bin/env bash
# Regenerate the CAD and electrical artifacts, in dependency order.
#
#   cad_src/rebuild.sh              # the lot
#   cad_src/rebuild.sh --fast       # skip the two viewer exports (the slow, multi-MB ones)
#
# Needs CadQuery (ml/setup_mac.sh --cad) and the bought-part models (models/fetch_models.sh),
# plus models/bought/mf6015.step, which is not redistributable: download it from the LKMTECH
# or dealer product page. Everything is written where robot_paths.py says (ROBOT_OUT).
#
# Order matters: assemble.py builds the printed parts the fit check and the exporters reuse,
# and dump_wiring.py produces the wiring_data.json that elec/can_ground_check.py reads.
set -euo pipefail
cd "$(dirname "$0")/.."
PY="$PWD/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"

FAST=0
for a in "$@"; do
  case "$a" in
    --fast) FAST=1 ;;
    *) echo "unknown option: $a (try --fast)" >&2; exit 2 ;;
  esac
done

# ---- preflight: fail here with something actionable, not 10 minutes in -------------------
"$PY" -c "import cadquery" 2>/dev/null || {
  echo "CadQuery is not installed. Run:  ml/setup_mac.sh --cad   (or: pip install cadquery)" >&2; exit 1; }
if ! ls models/bought/*.step >/dev/null 2>&1; then
  echo "== bought-part models missing, downloading"
  models/fetch_models.sh
fi
"$PY" - <<'PREFLIGHT' || exit 1
import sys
sys.path.insert(0, "cad_src/case")
import robot_paths as RP
try:
    RP.mf6015_step()
except FileNotFoundError as e:
    sys.exit(str(e))            # the message already says where to put it
PREFLIGHT

step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
cd cad_src/case

step "wiring harnesses -> wiring_data.json, wiring_diagram.json"
"$PY" dump_wiring.py

step "assembly: printed parts, meshes and masses for sim/"
"$PY" assemble.py

step "fit check against the real bought-part models"
"$PY" real_assembly.py

step "harness and connector collision check"
"$PY" wire_check.py

step "printed parts in print orientation -> print_parts_v11.json"
"$PY" export_print_data.py

if [ "$FAST" = 0 ]; then
  step "viewer data (placeholder components)"
  "$PY" export_viewer_data.py
  step "viewer data (real models + wiring)"
  "$PY" export_viewer_real.py
fi

cd ../..
step "electrical rules check against the regenerated wiring"
(cd elec && "$PY" erc.py | tail -3 && "$PY" can_ground_check.py | tail -3)

printf '\nDone. assemble.py also refreshed sim/meshes/ and sim/cad_mass.json:\n'
printf '  check them in, then run sim/rebuild.sh so the MuJoCo models follow the CAD.\n'
