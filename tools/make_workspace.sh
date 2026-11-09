#!/usr/bin/env bash
# Build the folder layout the CAD / BOM / site scripts expect (they were written in a workspace with the folders
# case/, robot/, v2site/, bom/, wheel_adapter/, models/, mf_step/ side by side). Everything links back into this repo.
#   tools/make_workspace.sh            -> ~/bot-ws   (or: BOT_WS=/some/dir tools/make_workspace.sh)
# Then run scripts from the repo as usual, e.g.  python cad_src/case/body_case.py ; the scripts find ~/bot-ws
# by themselves (or set BOT_WS=... if you chose another folder).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"; WS="${BOT_WS:-$HOME/bot-ws}"
mkdir -p "$WS"
ln -sfn "$REPO/cad_src/case" "$WS/case"
ln -sfn "$REPO/cad_src/wheel_adapter" "$WS/wheel_adapter"
ln -sfn "$REPO" "$WS/robot"
ln -sfn "$REPO/site" "$WS/v2site"
ln -sfn "$REPO/bom" "$WS/bom"
ln -sfn "$REPO/tools" "$WS/tools"
mkdir -p "$WS/models" "$WS/mf_step"
echo "workspace: $WS"
echo "Vendor 3D models are NOT in the repo (their licences): put the manufacturers' STEP files in $WS/models/bought"
echo "and the LK MF6015 STEP in $WS/mf_step (see cad_src/case/load_models.py for the file names). Only the fit and"
echo "wire checks and the 'real parts' 3D view need them; body_case.py, the BOM, the sim and the firmware don't."
