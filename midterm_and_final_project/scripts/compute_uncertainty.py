"""
compute_uncertainty.py

Reads the exported splatfacto .ply, computes a per-Gaussian uncertainty proxy
combining opacity and scale, and writes a top-down figure of the Jezero terrain
reconstruction colored by uncertainty.

This is Hour 3 of the SES 598 final project pilot. The uncertainty proxy is
deliberately simple: it does not use a calibrated method like Predictive
Photometric Uncertainty or DUSt3R confidence, both of which are out of scope
for the pilot. The proxy combines two signals available in the standard
splatfacto export schema:

    1. Opacity. Stored pre-sigmoid; activated as sigmoid(opacity_raw).
       Low activated opacity means the Gaussian carries low confidence.

    2. Scale magnitude. Stored as log(scale); activated as exp(scale_raw).
       Large activated scale means the Gaussian is spreading itself thin
       to cover an under-constrained region, which signals weak evidence.

Both signals are independently min-max normalized to [0, 1], with low-opacity
and high-scale Gaussians mapped toward 1.0 (high uncertainty). The combined
score is the mean of the two normalized signals.

Output: figures/fig1_uncertainty.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from plyfile import PlyData


def load_gaussians(ply_path):
    ply = PlyData.read(str(ply_path))
    v = ply["vertex"]
    positions = np.column_stack([v["x"], v["y"], v["z"]])
    opacity_raw = np.asarray(v["opacity"])
    scale_raw = np.column_stack([v["scale_0"], v["scale_1"], v["scale_2"]])
    return positions, opacity_raw, scale_raw


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def compute_uncertainty(opacity_raw, scale_raw):
    opacity = sigmoid(opacity_raw)
    scales = np.exp(scale_raw)
    scale_mag = np.linalg.norm(scales, axis=1)

    opacity_unc = 1.0 - opacity
    opacity_unc_norm = (opacity_unc - opacity_unc.min()) / (
        opacity_unc.max() - opacity_unc.min() + 1e-12
    )

    scale_lo, scale_hi = np.percentile(scale_mag, [5, 95])
    scale_clipped = np.clip(scale_mag, scale_lo, scale_hi)
    scale_unc_norm = (scale_clipped - scale_lo) / (scale_hi - scale_lo + 1e-12)

    combined = 0.5 * opacity_unc_norm + 0.5 * scale_unc_norm
    return combined, opacity, scale_mag


def make_figure(positions, uncertainty, output_path):
    x = positions[:, 0]
    z = positions[:, 2]

    order = np.argsort(uncertainty)
    x_sorted = x[order]
    z_sorted = z[order]
    u_sorted = uncertainty[order]

    fig, ax = plt.subplots(figsize=(8, 7))
    scatter = ax.scatter(
        x_sorted,
        z_sorted,
        c=u_sorted,
        cmap="inferno",
        s=4,
        alpha=0.85,
        edgecolors="none",
    )

    cbar = plt.colorbar(scatter, ax=ax, shrink=0.85, pad=0.02)
    cbar.set_label("Uncertainty proxy (low opacity + high scale)", fontsize=10)

    ax.set_xlabel("X (m)", fontsize=10)
    ax.set_ylabel("Z (m)", fontsize=10)
    ax.set_title(
        "Per-Gaussian uncertainty over the Jezero reconstruction\n"
        f"({len(positions):,} Gaussians, top-down view)",
        fontsize=11,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.4)

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ply",
        default="/home/jatin-satyam/midterm_3dgs_output/exports/splat.ply",
    )
    parser.add_argument(
        "--out",
        default=str(
            Path(__file__).resolve().parent.parent
            / "report"
            / "figures"
            / "fig1_uncertainty.png"
        ),
    )
    args = parser.parse_args()

    ply_path = Path(args.ply)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not ply_path.is_file():
        print(f"PLY not found at {ply_path}", file=sys.stderr)
        sys.exit(1)

    positions, opacity_raw, scale_raw = load_gaussians(ply_path)
    uncertainty, opacity, scale_mag = compute_uncertainty(opacity_raw, scale_raw)

    print(f"Loaded {len(positions):,} Gaussians from {ply_path}")
    print(f"Activated opacity: mean={opacity.mean():.3f}, "
          f"min={opacity.min():.3f}, max={opacity.max():.3f}")
    print(f"Scale magnitude:   mean={scale_mag.mean():.3f}, "
          f"min={scale_mag.min():.3f}, max={scale_mag.max():.3f}")
    print(f"Uncertainty proxy: mean={uncertainty.mean():.3f}, "
          f"5th={np.percentile(uncertainty, 5):.3f}, "
          f"95th={np.percentile(uncertainty, 95):.3f}")

    make_figure(positions, uncertainty, out_path)
    print(f"Wrote figure to {out_path}")

    np.savez(
        out_path.parent / "uncertainty_data.npz",
        positions=positions,
        uncertainty=uncertainty,
        opacity=opacity,
        scale_mag=scale_mag,
    )
    print(f"Wrote raw arrays to {out_path.parent / 'uncertainty_data.npz'} "
          "(used by Hour 4)")


if __name__ == "__main__":
    main()
