"""bot::Inputs / bot::Command against their ctypes mirrors in botsim.py.

Two things go wrong silently with these structs, and both have cost real time:

  * **A field's C++ default is not repeated on the Python side.** ctypes zero-initialises, so the
    field arrives as 0 and the controller reads a perfectly valid - and wrong - value. `drop_m`
    defaulted to 0 instead of -1 ("unknown") for a while, which made check_drop() report "the floor
    ends at the wheels" and pinned forward speed to zero through the one-sided clamp in
    controller.cpp. State stayed BALANCE throughout, so every test that asks only "did it stay
    upright" passed with the robot standing still: run_tests.py's cruise phase read -0.0 m/s against
    a committed 1.114, and check.sh showed it as a one-line DRIFT note naming the file, not the field.

  * **A field is added to one side only.** Then every field after it is read at the wrong offset.
    That is the sizeof check below.

Run directly, or via check.sh. Exits non-zero with the offending field named.
"""
import ctypes, pathlib, re, subprocess, sys, tempfile

HERE = pathlib.Path(__file__).resolve().parent
HPP = HERE.parent / "core" / "controller.hpp"
sys.path.insert(0, str(HERE))
import botsim                                                              # noqa: E402

# the C++ enum values a default may be written as, rather than a literal
ENUMS = {"CMD_VELOCITY": 0, "CMD_TURN_TO": 1, "CMD_TURN_BY": 2, "CMD_BACKFLIP": 3,
         "CMD_DOCK": 4, "CMD_UNDOCK": 5}


def cpp_defaults(struct):
    """{field: default} for every field of `struct` that HAS an initialiser in controller.hpp."""
    src = HPP.read_text()
    m = re.search(r"struct\s+%s\s*\{(.*?)\n\};" % struct, src, re.S)
    assert m, f"struct {struct} not found in {HPP}"
    out = {}
    for line in m.group(1).splitlines():
        line = re.sub(r"//.*", "", line).strip()
        # "float drop_m = -1;"  /  "uint8_t mode = CMD_VELOCITY;"  /  "float step_roll = 0.30f;"
        d = re.match(r"\w+\s+(\w+)\s*=\s*([^;]+);", line)
        if d:
            v = d.group(2).strip().rstrip("f")
            out[d.group(1)] = float(ENUMS[v]) if v in ENUMS else float(v)
    return out


def check_defaults():
    bad = []
    sim = botsim.BotSim(seed=0)
    try:
        actual = {"Inputs": sim.sense(), "Command": sim.cmd}
        for struct, want in (("Inputs", cpp_defaults("Inputs")), ("Command", cpp_defaults("Command"))):
            for field, v in want.items():
                got = float(getattr(actual[struct], field))
                v = float(ctypes.c_float(v).value)        # compare at float32: 0.30f is not 0.3
                # Inputs fields that sense() computes from the model are not defaults to mirror;
                # only the ones C++ gives a non-zero default must survive the ctypes zero-fill.
                if v != 0.0 and got != v:
                    bad.append(f"  {struct}.{field}: C++ default {v}, botsim gives {got}")
    finally:
        sim.close()
    return bad


def check_sizes():
    """Compile a throwaway program that prints the real sizeof/offsetof, and compare."""
    probe = """#include <cstdio>
#include <cstddef>
#include "controller.hpp"
int main(){
  printf("Inputs %zu\\n", sizeof(bot::Inputs));
  printf("Command %zu\\n", sizeof(bot::Command));
  return 0;
}
"""
    with tempfile.TemporaryDirectory() as td:
        src = pathlib.Path(td) / "probe.cpp"
        src.write_text(probe)
        exe = pathlib.Path(td) / "probe"
        r = subprocess.run(["c++", "-std=c++17", "-I", str(HPP.parent), str(src), "-o", str(exe)],
                           capture_output=True, text=True)
        if r.returncode:
            return [f"  could not compile the size probe: {r.stderr.strip().splitlines()[-1]}"]
        sizes = dict(l.split() for l in subprocess.run([str(exe)], capture_output=True, text=True)
                     .stdout.split("\n") if l)
    bad = []
    for name, cls in (("Inputs", botsim.Inputs), ("Command", botsim.Command)):
        want, got = int(sizes[name]), ctypes.sizeof(cls)
        if want != got:
            bad.append(f"  bot::{name} is {want} bytes, botsim.{name} is {got}: a field was added to one side")
    return bad


if __name__ == "__main__":
    bad = check_defaults() + check_sizes()
    if bad:
        print("STRUCT MISMATCH (botsim.py vs core/controller.hpp):")
        print("\n".join(bad))
        sys.exit(1)
    print("structs OK: ctypes sizes match, and every non-zero C++ default is mirrored")
