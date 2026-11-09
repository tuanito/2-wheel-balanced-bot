#!/usr/bin/env bash
# Copy the published artifact site (v2site) into the GitHub repo's site/ folder, commit and push.
# Run after every artifact publish:  tools/sync_site.sh "what changed"
set -euo pipefail
# Only useful where the artifact workspace exists: on a checkout, site/ IS the source.
REPO="$(cd "$(dirname "$0")/.." && pwd)"; SRC="${BOT_WS:-/home/claude}/v2site"; DST="$REPO/site"
[ -d "$SRC" ] || { echo "no artifact workspace at $SRC - on a checkout site/ is already the source" >&2; exit 1; }
MSG="${1:-update}"
rm -rf "$DST"; mkdir -p "$DST/files"
for f in "$SRC"/*.html; do case "$(basename "$f")" in *_tpl.html) ;; *) cp "$f" "$DST/";; esac; done
( cd "$SRC/files" && find . -type f ! -name "*.zip.json" -print0 | while IFS= read -r -d '' f; do mkdir -p "$DST/files/$(dirname "$f")"; cp "$f" "$DST/files/$f"; done )
cat > "$DST/README.md" <<'MD'
# Project site (copy of the published artifact)

The same pages as the Claude artifact: 3D viewer, wiring, print guide, parts list, README/INSTALL, source code,
videos. Open it with a local web server (the pages load data files, so double-clicking index.html won't work):

    python3 -m http.server -d site 8000     # then http://localhost:8000

The ZIP download buttons are not included here (they are base64 copies of this repository's own files: use
`cad/`, `bom/` and the source folders directly). Synced automatically after every artifact update.
MD
cd "$REPO"
git add -A site
if git diff --cached --quiet; then echo "site unchanged"; exit 0; fi
git -c user.name="tuanito" -c user.email="operations@n-sigmacapital.com" commit -q -m "Site: $MSG

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Fg7Pb2xfJ1nQFp1vCD25rQ"
git fetch -q origin +main:refs/remotes/origin/main && git rebase -q origin/main
git push -q origin HEAD:main && echo "site pushed: $(git rev-parse --short HEAD)"
