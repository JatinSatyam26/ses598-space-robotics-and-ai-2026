"""
Compute per-Gaussian reconstruction uncertainty from a splatfacto PLY export.
Proxy: combined opacity-based and volume-based uncertainty, normalized to [0,1].
Output: uncertainty_topdown.png
"""
import numpy as np
import matplotlib.pyplot as plt
from plyfile import PlyData

ply = PlyData.read('../midterm_3dgs_export/splat.ply')
v = ply["vertex"]
x, y = np.array(v["x"]), np.array(v["y"])
opacity = 1.0 / (1.0 + np.exp(-np.array(v["opacity"])))
volume = (np.exp(np.array(v["scale_0"])) *
          np.exp(np.array(v["scale_1"])) *
          np.exp(np.array(v["scale_2"])))
def norm(a): return (a - a.min()) / (a.max() - a.min() + 1e-8)
uncertainty = 0.5 * norm(1.0 - opacity) + 0.5 * norm(volume)
order = np.argsort(uncertainty)
x, y, uncertainty = x[order], y[order], uncertainty[order]
fig, ax = plt.subplots(figsize=(10, 7))
sc = ax.scatter(x, y, c=uncertainty, cmap="hot_r", s=3, alpha=0.8,
                vmin=uncertainty.min(), vmax=uncertainty.max())
plt.colorbar(sc, ax=ax, label="Reconstruction Uncertainty (low → high)")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_aspect("equal")
ax.set_title("Per-Gaussian Reconstruction Uncertainty\nJezero Crater Terrain — Top-Down View (14,156 Gaussians)")
ax.annotate("Dense coverage\n(certain)", xy=(0.5, 0.7), xytext=(2.5, 0.3),
            fontsize=10, color="gray", arrowprops=dict(arrowstyle="->", color="gray"))
ax.annotate("Sparse coverage\n(uncertain)", xy=(-4, 4), xytext=(-3, 3.5),
            fontsize=10, color="darkred")
plt.tight_layout()
plt.savefig("figures/uncertainty_topdown.png", dpi=200, bbox_inches="tight")
print("Saved: figures/uncertainty_topdown.png")
