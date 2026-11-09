import numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from render import surface, draw
def load(d):
    r = np.load(f"out/{d}/res.npz"); return r["X"], surface(r["tets"]), r["vm"]
VMAX = 20.0
cases = [("new", "Before: shelf only sliding on rails (loose)\nmax 20.5 MPa · motor axis tilts 3.3°"),
         ("reinf", "Now: shelf screwed, fillets, ribs, deeper body\nmax 5.4 MPa · motor axis tilts 0.24°")]
fig = plt.figure(figsize=(7.4, 9.0), dpi=130)
for i, (d, t) in enumerate(cases):
    X, F, vm = load(d)
    ax = fig.add_subplot(2, 2, 2 * i + 1, projection="3d"); draw(ax, X, F, vm, 28, -128, VMAX, title=t)
    ax = fig.add_subplot(2, 2, 2 * i + 2, projection="3d"); draw(ax, X, F, vm, -32, 58, VMAX, title="from below")
sm = plt.cm.ScalarMappable(cmap="turbo", norm=plt.Normalize(0, VMAX))
cb = fig.colorbar(sm, ax=fig.axes, orientation="horizontal", fraction=0.03, pad=0.03)
cb.set_label("von Mises (MPa) · peak torque 2.3 N·m + 25 N per wheel · PETG layer bond ≈ 20–30 MPa")
plt.savefig("fea_before_after.png", bbox_inches="tight"); print("ok")
