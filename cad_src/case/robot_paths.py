"""Where everything lives on this machine.

The CAD and electrical scripts were written on the Claude VM, whose layout is

    /home/claude/case            the case sources (this directory)
    /home/claude/wheel_adapter   the wheel-adapter sources
    /home/claude/robot           the git repository
    /home/claude/models/bought   bought-part STEP models
    /home/claude/mf_step         the MF6015 motor STEP

In a git checkout the same files are cad_src/case, cad_src/wheel_adapter and the
repository root itself. Nothing below is hard-coded to either layout: the source
trees are found from this file's own location (it ships next to the case sources,
so it is in the right place on both machines), the repository by walking up to the
directory that holds firmware/ and sim/, and the VM paths remain as a last-resort
fallback. So the same code runs unchanged on the VM and on a developer machine.

Every root can be overridden by an environment variable:

    ROBOT_CASE  ROBOT_WHEEL_ADAPTER  ROBOT_ROOT  ROBOT_OUT  ROBOT_MODELS  ROBOT_MF6015_STEP

From a script in this directory:   import robot_paths as RP
From anywhere else in the repo:    import sys, pathlib
                                   sys.path.insert(0, str(pathlib.Path(__file__).resolve()
                                                          .parents[1] / "cad_src" / "case"))
                                   import robot_paths as RP
"""
import os
import pathlib
import sys

Path = pathlib.Path
_VM = Path("/home/claude")          # the Claude VM layout, used only if it is really there


def _env(name):
    v = os.environ.get(name)
    return Path(v).expanduser() if v else None


# ---------------------------------------------------------------- source trees
# This file sits in the case source directory, so its own parent is the answer on
# both machines: /home/claude/case on the VM, <repo>/cad_src/case in a checkout.
CASE = _env("ROBOT_CASE") or Path(__file__).resolve().parent
# ... and the wheel adapter is its sibling in both layouts.
WHEEL_ADAPTER = _env("ROBOT_WHEEL_ADAPTER") or CASE.parent / "wheel_adapter"


def _find_repo():
    p = _env("ROBOT_ROOT")
    if p:
        return p
    for d in [CASE, *CASE.parents]:                     # a checkout: walk up to the root
        if (d / "firmware" / "esp32").is_dir() and (d / "sim").is_dir():
            return d
    if (_VM / "robot" / "sim").is_dir():                # the VM: the repo is a sibling
        return _VM / "robot"
    raise RuntimeError(
        "cannot find the repository root (no parent of %s holds both firmware/ and sim/, "
        "and %s does not exist). Set ROBOT_ROOT to the checkout." % (CASE, _VM / "robot"))


REPO = _find_repo()
SIM = REPO / "sim"
FIRMWARE_CONFIG = REPO / "firmware" / "esp32" / "include" / "config.h"
PI_INSTALL = REPO / "pi" / "install.sh"


def add_sources():
    """Put the case and wheel-adapter sources on sys.path (body_case, wiring, wheel_adapter...)."""
    for d in (WHEEL_ADAPTER, CASE):
        s = str(d)
        if s not in sys.path:
            sys.path.insert(0, s)


def source(name):
    """A case source file read as text (the scripts that exec parts of each other)."""
    return CASE / name


# ---------------------------------------------------------- generated artifacts
# Where assemble.py and the exporters write. On the VM this is /home/claude/case,
# exactly where they have always written; in a checkout it is cad_src/case, which
# .gitignore keeps out of git. Set ROBOT_OUT to collect them somewhere else.
OUT = _env("ROBOT_OUT") or CASE
ASSEMBLY = OUT / "assembly"

# Artifacts that one script writes and another reads, with the command that makes each.
MAKES = {
    "wiring_data.json":      "python cad_src/case/dump_wiring.py",
    "wiring_diagram.json":   "python cad_src/case/dump_wiring.py",
    "viewer_parts_v11.json": "python cad_src/case/export_viewer_data.py",
    "print_parts_v11.json":  "python cad_src/case/export_print_data.py",
    "real_fit_report.json":  "python cad_src/case/real_assembly.py",
    "wire_report.json":      "python cad_src/case/wire_check.py",
}


def out(name):
    """Where to write a generated artifact (creating the directory if needed)."""
    OUT.mkdir(parents=True, exist_ok=True)
    return OUT / name


def artifact(name):
    """Where to read a generated artifact: the first copy that exists."""
    tried = []
    for d in dict.fromkeys((OUT, CASE, _VM / "case", REPO / "elec")):   # dedup, keep order
        p = d / name
        tried.append(str(p))
        if p.exists():
            return p
    how = MAKES.get(name)
    raise FileNotFoundError(
        "%s not found. Looked in:\n  %s\n%s" % (
            name, "\n  ".join(tried),
            ("Generate it with:  %s  (needs cadquery)" % how) if how else ""))


# ------------------------------------------------------------- bought-part CAD
def models_dir():
    """Directory of bought-part STEP models (models/fetch_models.sh downloads them)."""
    p = _env("ROBOT_MODELS")
    if p:
        return p
    for d in (REPO / "models" / "bought", _VM / "models" / "bought"):
        if d.is_dir():
            return d
    return REPO / "models" / "bought"


def mf6015_step():
    """The MF6015 motor STEP. Not redistributable: it comes from the LKMTECH product page."""
    for p in (_env("ROBOT_MF6015_STEP"), models_dir() / "mf6015.step", _VM / "mf_step" / "mf6015.step"):
        if p and p.exists():
            return p
    raise FileNotFoundError(
        "MF6015 motor STEP not found. It is not in the repo (see the last line of "
        "models/fetch_models.sh): download it from the LKMTECH / dealer product page, save it as "
        "%s, or point ROBOT_MF6015_STEP at it." % (models_dir() / "mf6015.step"))
