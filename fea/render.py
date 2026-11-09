"""Render von Mises stress on the outer surface of a CalculiX result (res.npz)."""
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def surface(tets):
    f = np.concatenate([tets[:, [0, 2, 1]], tets[:, [0, 1, 3]], tets[:, [1, 2, 3]], tets[:, [0, 3, 2]]])
    k = np.sort(f, 1); _, idx, cnt = np.unique(k, axis=0, return_index=True, return_counts=True)
    return f[idx[cnt == 1]]

def draw(ax, X, F, vm, elev, azim, vmax, lim=None, title=""):
    if lim is not None:
        c = X[F].mean(1); m = np.all((c >= lim[0]) & (c <= lim[1]), 1); F = F[m]
    tri = X[F]; val = vm[F].mean(1)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
    e, a = np.radians(elev), np.radians(azim)
    view = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    shade = 0.55 + 0.45 * np.abs(n @ view)
    col = plt.cm.turbo(np.clip(val / vmax, 0, 1)); col[:, :3] *= shade[:, None]
    depth = tri.mean(1) @ view; o = np.argsort(depth)
    ax.add_collection3d(Poly3DCollection(tri[o], facecolors=col[o], edgecolors="none", linewidths=0))
    lo, hi = (X.min(0), X.max(0)) if lim is None else (np.array(lim[0]), np.array(lim[1]))
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=10)

if __name__ == "__main__":
    pass
