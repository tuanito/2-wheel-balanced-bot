# Project site (copy of the published artifact)

The same pages as the Claude artifact: 3D viewer, wiring, print guide, parts list, README/INSTALL, source code,
videos. Open it with a local web server (the pages load data files, so double-clicking index.html won't work):

    python3 -m http.server -d site 8000     # then http://localhost:8000

The ZIP download buttons are not included here (they are base64 copies of this repository's own files: use
`cad/`, `bom/` and the source folders directly). Synced automatically after every artifact update.
