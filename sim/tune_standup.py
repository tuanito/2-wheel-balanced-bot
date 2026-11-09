"""Sweep stand-up parameters over randomised robots, both lying sides (runs in parallel)."""
import sys, numpy as np, itertools, math
from multiprocessing import Pool
from botsim import BotSim, STATES, LIB

def trial(args):
    seed, side, lift, epush, eg, catch, et = args
    s = BotSim(seed=seed, randomize=True)
    LIB.bot_tune(s.ctl, lift, 0.6, epush, catch); LIB.bot_lift_mode(s.ctl, 0, eg, et)
    s.place(pitch=1.8 * side)
    s.run(5.0)
    L = np.array(s.log); s.close()
    st = L[:, 1].astype(int)
    ok = st[-1] == 4 and np.abs(L[-500:, 2]).max() < math.radians(12)
    up = np.where(st == 4)[0]
    return ok, (L[up[0], 0] - 0.3 if len(up) else np.nan), np.abs(L[:, 5]).max(), np.hypot(L[-1, 9], L[-1, 10])

if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    grid = list(itertools.product([float(v) for v in (sys.argv[2].split(",") if len(sys.argv) > 2 else ["1.6"])], [30], [0.03], [0.35, 0.5], [0, 4, 8]))
    with Pool(8) as pool:
        for lift, epush, eg, catch, et in grid:
            res = pool.map(trial, [(2000 + i, side, lift, epush, eg, catch, et) for i in range(N) for side in (1, -1)])
            ok = sum(r[0] for r in res)
            print(f"lift {lift} epush {epush} eg {eg} catch {catch} et {et}: {ok}/{len(res)}  t_up {np.nanmedian([r[1] for r in res]):.2f}s  "
                  f"vmax {np.median([r[2] for r in res]):.2f}  travel {np.median([r[3] for r in res]):.2f} m", flush=True)
