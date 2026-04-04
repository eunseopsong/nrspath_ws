#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import os
import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np


def create_heatmap_figure(res):
    extent = [res.meta["x_min"], res.meta["x_max"], res.meta["y_min"], res.meta["y_max"]]
    mean_rem = float(res.meta["mean_heatmap_removal"])
    std_rem = float(res.meta["std_heatmap_removal"])

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111)
    im = ax.imshow(res.grid_removal, origin="lower", extent=extent, aspect="auto")
    ax.set_title("Removal Heatmap")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Cell removal [a.u.]")
    text = (
        f"mean = {mean_rem:.6f} [a.u.]\n"
        f"std  = {std_rem:.6f} [a.u.]\n"
        f"samples = {res.meta['samples']}\n"
        f"contact = {res.meta['contact_samples']}"
    )
    ax.text(
        0.02,
        0.98,
        text,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )
    fig.tight_layout()
    return fig


def save_required_plots_only(
    res,
    out_dir: Path,
    w_arrow_stride: int,
    w_arrow_length_mm: float,
) -> list[str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    saved_files: list[str] = []

    xyz = res.state6[:, 0:3]
    wxyz = res.state6[:, 3:6]
    force3 = res.force3
    heatmap_mean = float(res.meta["mean_heatmap_removal"])

    # 1) Removal heatmap
    fig = create_heatmap_figure(res)
    fname = "01_removal_heatmap.png"
    fig.savefig(out_dir / fname, dpi=220)
    plt.close(fig)
    saved_files.append(fname)

    # 2) Time-series of the same quantity as the heatmap.
    fig = plt.figure(figsize=(9, 5))
    ax = fig.add_subplot(111)
    ax.plot(res.t, res.heatmap_value_series, linewidth=1.5)
    ax.axhline(heatmap_mean, linestyle="--", linewidth=1.2, label="heatmap mean")
    ax.set_title("Heatmap Cell Removal at Current Position vs Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cell removal [a.u.]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fname = "02_heatmap_value_vs_time.png"
    fig.savefig(out_dir / fname, dpi=220)
    plt.close(fig)
    saved_files.append(fname)

    # 3) Subplots: x y z wx wy wz fx fy fz
    labels = [
        (xyz[:, 0], "x [mm]"),
        (xyz[:, 1], "y [mm]"),
        (xyz[:, 2], "z [mm]"),
        (wxyz[:, 0], "wx"),
        (wxyz[:, 1], "wy"),
        (wxyz[:, 2], "wz"),
        (force3[:, 0], "fx [N]"),
        (force3[:, 1], "fy [N]"),
        (force3[:, 2], "fz [N]"),
    ]
    fig, axes = plt.subplots(3, 3, figsize=(14, 9), sharex=True)
    for ax, (y, ylabel) in zip(axes.ravel(), labels):
        ax.plot(res.t, y, linewidth=1.3)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")
    fig.suptitle("Recorded Signals: x y z wx wy wz fx fy fz")
    fig.tight_layout(rect=[0, 0.02, 1, 0.97])
    fname = "03_signals_subplot.png"
    fig.savefig(out_dir / fname, dpi=220)
    plt.close(fig)
    saved_files.append(fname)

    # 4) 3D trajectory + angular-velocity direction arrows
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], linewidth=1.6, label="xyz path")

    stride = max(int(w_arrow_stride), 1)
    idx = np.arange(0, xyz.shape[0], stride)
    if idx.size > 0:
        xyz_s = xyz[idx]
        w_s = wxyz[idx]
        w_norm = np.linalg.norm(w_s, axis=1, keepdims=True)
        valid = w_norm[:, 0] > 1e-12
        if np.any(valid):
            dirs = np.zeros_like(w_s)
            dirs[valid] = w_s[valid] / w_norm[valid]
            ax.quiver(
                xyz_s[valid, 0], xyz_s[valid, 1], xyz_s[valid, 2],
                dirs[valid, 0], dirs[valid, 1], dirs[valid, 2],
                length=float(w_arrow_length_mm),
                normalize=False,
                linewidth=0.8,
                arrow_length_ratio=0.25,
            )

    ax.set_title("3D Path with Angular-Velocity Direction (wx, wy, wz)")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")
    ax.legend(loc="best")

    x_range = float(np.ptp(xyz[:, 0])) if xyz.shape[0] > 0 else 1.0
    y_range = float(np.ptp(xyz[:, 1])) if xyz.shape[0] > 0 else 1.0
    z_range = float(np.ptp(xyz[:, 2])) if xyz.shape[0] > 0 else 1.0
    max_range = max(x_range, y_range, z_range, 1.0)
    x_mid = float(np.mean([np.min(xyz[:, 0]), np.max(xyz[:, 0])]))
    y_mid = float(np.mean([np.min(xyz[:, 1]), np.max(xyz[:, 1])]))
    z_mid = float(np.mean([np.min(xyz[:, 2]), np.max(xyz[:, 2])]))
    ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
    ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
    ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)

    fig.tight_layout()
    fname = "04_3d_path_w.png"
    fig.savefig(out_dir / fname, dpi=220)
    plt.close(fig)
    saved_files.append(fname)

    return saved_files