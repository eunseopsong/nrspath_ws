#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ----------------------------
# Robust TXT loaders
# ----------------------------

def _load_numeric_rows(path: str, expected_cols: int) -> np.ndarray:
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != expected_cols:
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue

    if not rows:
        raise RuntimeError(f"No valid numeric rows found in {path} (expected {expected_cols} cols).")
    return np.asarray(rows, dtype=float)


def load_timeseries_xyz(path: str) -> Tuple[np.ndarray, np.ndarray]:
    arr = _load_numeric_rows(path, expected_cols=4)
    return arr[:, 0], arr[:, 1:4]


def load_timeseries_vxyz(path: str) -> Tuple[np.ndarray, np.ndarray]:
    arr = _load_numeric_rows(path, expected_cols=4)
    return arr[:, 0], arr[:, 1:4]


def load_timeseries_fxyz(path: str) -> Tuple[np.ndarray, np.ndarray]:
    arr = _load_numeric_rows(path, expected_cols=4)
    return arr[:, 0], arr[:, 1:4]


def load_timeseries_rpy(path: str) -> Tuple[np.ndarray, np.ndarray]:
    arr = _load_numeric_rows(path, expected_cols=4)
    return arr[:, 0], arr[:, 1:4]


def load_waypoints_9d(path: str) -> np.ndarray:
    return _load_numeric_rows(path, expected_cols=9)


def interp_timeseries(t_src: np.ndarray, y_src: np.ndarray, t_ref: np.ndarray) -> np.ndarray:
    if y_src.ndim == 1:
        y_src = y_src[:, None]

    idx = np.argsort(t_src)
    t_sorted = t_src[idx]
    y_sorted = y_src[idx]
    uniq_t, uniq_idx = np.unique(t_sorted, return_index=True)
    y_sorted = y_sorted[uniq_idx]

    y_ref = np.zeros((t_ref.shape[0], y_sorted.shape[1]), dtype=float)
    for d in range(y_sorted.shape[1]):
        y_ref[:, d] = np.interp(t_ref, uniq_t, y_sorted[:, d])
    return y_ref


# ----------------------------
# RPY -> Rotation (ZYX)
# ----------------------------

def rpy_to_R_zyx(rpy: np.ndarray) -> np.ndarray:
    roll = rpy[:, 0]
    pitch = rpy[:, 1]
    yaw = rpy[:, 2]

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R = np.zeros((rpy.shape[0], 3, 3), dtype=float)
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr

    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr

    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


def parse_tool_axis(axis_str: str) -> np.ndarray:
    axis_str = axis_str.strip().upper()
    if axis_str in ["+Z", "Z", "Z+", "+3"]:
        return np.array([0.0, 0.0, 1.0])
    if axis_str in ["-Z", "Z-", "-3"]:
        return np.array([0.0, 0.0, -1.0])
    if axis_str in ["+X", "X", "X+", "+1"]:
        return np.array([1.0, 0.0, 0.0])
    if axis_str in ["-X", "X-", "-1"]:
        return np.array([-1.0, 0.0, 0.0])
    if axis_str in ["+Y", "Y", "Y+", "+2"]:
        return np.array([0.0, 1.0, 0.0])
    if axis_str in ["-Y", "Y-", "-2"]:
        return np.array([0.0, -1.0, 0.0])
    raise ValueError("tool_axis must be one of ±X/±Y/±Z.")


# ----------------------------
# Removal model (Preston)
# ----------------------------

@dataclass
class RemovalResult:
    t: np.ndarray
    xyz: np.ndarray
    vxyz: np.ndarray
    fxyz: np.ndarray
    rpy: np.ndarray
    dt: np.ndarray
    fn: np.ndarray
    speed: np.ndarray
    removal_rate: np.ndarray
    dremoval: np.ndarray
    cremoval: np.ndarray
    grid_removal: np.ndarray
    grid_hits: np.ndarray
    grid_dt: np.ndarray
    meta: Dict[str, float]


def fft_convolve2d(a: np.ndarray, k: np.ndarray) -> np.ndarray:
    H, W = a.shape
    kh, kw = k.shape
    out_h = H + kh - 1
    out_w = W + kw - 1

    fh = int(2 ** np.ceil(np.log2(out_h)))
    fw = int(2 ** np.ceil(np.log2(out_w)))

    A = np.fft.rfft2(a, s=(fh, fw))
    K = np.fft.rfft2(k, s=(fh, fw))
    y = np.fft.irfft2(A * K, s=(fh, fw))

    start_h = (kh - 1) // 2
    start_w = (kw - 1) // 2
    return y[start_h:start_h + H, start_w:start_w + W]


def compute_removal(
    xyz_path: str,
    vxyz_path: str,
    fxyz_path: str,
    rpy_path: Optional[str],
    cell_mm: float = 1.0,
    k_preston: float = 1.0,
    tool_axis: str = "-Z",
    speed_mode: str = "xy",
    contact_threshold_N: float = 0.5,
    speed_threshold_mm_s: float = 0.1,
    pad_radius_mm: float = 0.0,
) -> RemovalResult:
    t_xyz, xyz = load_timeseries_xyz(xyz_path)
    t_v, vxyz = load_timeseries_vxyz(vxyz_path)
    t_f, fxyz = load_timeseries_fxyz(fxyz_path)

    t_ref = t_xyz.copy()
    vxyz_i = interp_timeseries(t_v, vxyz, t_ref)
    fxyz_i = interp_timeseries(t_f, fxyz, t_ref)

    if rpy_path:
        t_r, rpy = load_timeseries_rpy(rpy_path)
        rpy_i = interp_timeseries(t_r, rpy, t_ref)
    else:
        rpy_i = np.zeros((t_ref.shape[0], 3), dtype=float)

    dt = np.diff(t_ref, prepend=t_ref[0])
    dt0 = np.median(np.diff(t_ref)) if t_ref.shape[0] > 2 else 0.01
    dt[0] = dt0
    dt = np.clip(dt, 1e-6, None)

    if speed_mode.lower() == "xy":
        speed = np.linalg.norm(vxyz_i[:, 0:2], axis=1)
    elif speed_mode.lower() == "xyz":
        speed = np.linalg.norm(vxyz_i, axis=1)
    else:
        raise ValueError("speed_mode must be 'xy' or 'xyz'.")

    axis_tool = parse_tool_axis(tool_axis)
    R = rpy_to_R_zyx(rpy_i)
    axis_base = (R @ axis_tool.reshape(3, 1)).reshape(-1, 3)

    fn1 = np.sum(fxyz_i * axis_base, axis=1)
    fn2 = -fn1
    m1 = np.mean(np.clip(fn1, 0.0, None))
    m2 = np.mean(np.clip(fn2, 0.0, None))
    fn_signed = fn1 if m1 >= m2 else fn2
    fn = np.clip(fn_signed, 0.0, None)

    in_contact = (fn >= contact_threshold_N) & (speed >= speed_threshold_mm_s)

    removal_rate = np.zeros_like(fn)
    removal_rate[in_contact] = k_preston * fn[in_contact] * speed[in_contact]
    dremoval = removal_rate * dt
    cremoval = np.cumsum(dremoval)

    x = xyz[:, 0]
    y = xyz[:, 1]
    x_min = float(np.min(x))
    x_max = float(np.max(x))
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    margin = 0.5 * cell_mm
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    W = int(np.ceil((x_max - x_min) / cell_mm))
    H = int(np.ceil((y_max - y_min) / cell_mm))

    grid_removal = np.zeros((H, W), dtype=float)
    grid_hits = np.zeros((H, W), dtype=float)
    grid_dt = np.zeros((H, W), dtype=float)

    ix = np.floor((x - x_min) / cell_mm).astype(int)
    iy = np.floor((y - y_min) / cell_mm).astype(int)
    ix = np.clip(ix, 0, W - 1)
    iy = np.clip(iy, 0, H - 1)

    for k in range(t_ref.shape[0]):
        if not in_contact[k]:
            continue
        j = ix[k]
        i = iy[k]
        grid_removal[i, j] += dremoval[k]
        grid_hits[i, j] += 1.0
        grid_dt[i, j] += dt[k]

    if pad_radius_mm and pad_radius_mm > 0.0:
        r_cells = int(np.ceil(pad_radius_mm / cell_mm))
        if r_cells >= 1:
            yy, xx = np.mgrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
            mask = (xx ** 2 + yy ** 2) <= (pad_radius_mm / cell_mm) ** 2
            kernel = mask.astype(float)
            kernel /= np.sum(kernel)
            grid_removal = fft_convolve2d(grid_removal, kernel)
            grid_hits = fft_convolve2d(grid_hits, kernel)
            grid_dt = fft_convolve2d(grid_dt, kernel)

    meta = {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "cell_mm": float(cell_mm),
        "k_preston": float(k_preston),
        "contact_threshold_N": float(contact_threshold_N),
        "speed_threshold_mm_s": float(speed_threshold_mm_s),
        "pad_radius_mm": float(pad_radius_mm),
    }

    return RemovalResult(
        t=t_ref,
        xyz=xyz,
        vxyz=vxyz_i,
        fxyz=fxyz_i,
        rpy=rpy_i,
        dt=dt,
        fn=fn,
        speed=speed,
        removal_rate=removal_rate,
        dremoval=dremoval,
        cremoval=cremoval,
        grid_removal=grid_removal,
        grid_hits=grid_hits,
        grid_dt=grid_dt,
        meta=meta,
    )


def save_analysis_outputs(res: RemovalResult, out_dir: str) -> str:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    def _plot_ts(y, title, ylabel, fname):
        plt.figure()
        plt.plot(res.t, y)
        plt.title(title)
        plt.xlabel("t [s]")
        plt.ylabel(ylabel)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(out / fname, dpi=200)
        plt.close()

    _plot_ts(res.fn, "Normal force (auto-signed, clipped)", "Fn [N]", "ts_fn.png")
    _plot_ts(res.speed, "Speed", "speed [mm/s]", "ts_speed.png")
    _plot_ts(res.removal_rate, "Removal rate (relative)", "dR/dt [a.u.]", "ts_removal_rate.png")
    _plot_ts(res.cremoval, "Cumulative removal (relative)", "R [a.u.]", "ts_cum_removal.png")

    extent = [res.meta["x_min"], res.meta["x_max"], res.meta["y_min"], res.meta["y_max"]]

    plt.figure()
    plt.imshow(res.grid_removal, origin="lower", extent=extent, aspect="auto")
    plt.title("Removal heatmap (sum per cell, relative)")
    plt.xlabel("x [mm]")
    plt.ylabel("y [mm]")
    plt.colorbar(label="R_cell [a.u.]")
    plt.tight_layout()
    plt.savefig(out / "hm_removal_sum.png", dpi=220)
    plt.close()

    mean_rate = res.grid_removal / (res.grid_dt + 1e-12)
    plt.figure()
    plt.imshow(mean_rate, origin="lower", extent=extent, aspect="auto")
    plt.title("Removal rate heatmap (R_cell / dt_cell)")
    plt.xlabel("x [mm]")
    plt.ylabel("y [mm]")
    plt.colorbar(label="R/dt [a.u./s]")
    plt.tight_layout()
    plt.savefig(out / "hm_removal_rate.png", dpi=220)
    plt.close()

    plt.figure()
    plt.imshow(res.grid_hits, origin="lower", extent=extent, aspect="auto")
    plt.title("Hit-count heatmap (samples per cell)")
    plt.xlabel("x [mm]")
    plt.ylabel("y [mm]")
    plt.colorbar(label="hits [-]")
    plt.tight_layout()
    plt.savefig(out / "hm_hits.png", dpi=220)
    plt.close()

    np.savez_compressed(
        out / "removal_map.npz",
        grid_removal=res.grid_removal,
        grid_hits=res.grid_hits,
        grid_dt=res.grid_dt,
        meta=np.array([res.meta[k] for k in ["x_min", "x_max", "y_min", "y_max", "cell_mm"]], dtype=float),
        meta_keys=np.array(["x_min", "x_max", "y_min", "y_max", "cell_mm"]),
    )

    summary = (
        f"Samples (xyz): {res.t.shape[0]}\n"
        f"dt median: {np.median(res.dt):.6f} s\n"
        f"Contact threshold: Fn>={res.meta['contact_threshold_N']:.3f} N, speed>={res.meta['speed_threshold_mm_s']:.3f} mm/s\n"
        f"Contact samples: {int(np.sum((res.fn >= res.meta['contact_threshold_N']) & (res.speed >= res.meta['speed_threshold_mm_s'])))}\n"
        f"Cumulative removal (relative): {res.cremoval[-1]:.6f}\n"
        f"Grid: HxW={res.grid_removal.shape[0]}x{res.grid_removal.shape[1]}, cell={res.meta['cell_mm']:.3f} mm\n"
    )
    (out / "summary.txt").write_text(summary, encoding="utf-8")
    return str(out)


def load_removal_npz(npz_path: str) -> Tuple[np.ndarray, Dict[str, float]]:
    data = np.load(npz_path, allow_pickle=True)
    grid = data["grid_removal"]
    meta_vals = data["meta"]
    meta_keys = data["meta_keys"]
    meta = {str(k): float(v) for k, v in zip(meta_keys, meta_vals)}
    return grid, meta


def bilinear_sample(grid: np.ndarray, x: np.ndarray, y: np.ndarray, meta: Dict[str, float]) -> np.ndarray:
    x_min = meta["x_min"]
    y_min = meta["y_min"]
    cell = meta["cell_mm"]
    H, W = grid.shape
    gx = (x - x_min) / cell
    gy = (y - y_min) / cell
    gx = np.clip(gx, 0.0, W - 1.000001)
    gy = np.clip(gy, 0.0, H - 1.000001)

    x0 = np.floor(gx).astype(int)
    y0 = np.floor(gy).astype(int)
    x1 = np.clip(x0 + 1, 0, W - 1)
    y1 = np.clip(y0 + 1, 0, H - 1)

    wx = gx - x0
    wy = gy - y0

    v00 = grid[y0, x0]
    v10 = grid[y0, x1]
    v01 = grid[y1, x0]
    v11 = grid[y1, x1]
    v0 = v00 * (1 - wx) + v10 * wx
    v1 = v01 * (1 - wx) + v11 * wx
    return v0 * (1 - wy) + v1 * wy


def resample_waypoints_weighted(wp: np.ndarray, weight: np.ndarray, n_out: int) -> np.ndarray:
    d = np.linalg.norm(np.diff(wp[:, 0:3], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] <= 1e-9:
        return np.repeat(wp[0:1], n_out, axis=0)

    w = np.clip(weight, 1e-6, None)
    ds = np.diff(s)
    cw = np.zeros_like(s)
    cw[1:] = np.cumsum(0.5 * (w[:-1] + w[1:]) * ds)

    total = cw[-1]
    target = np.linspace(0.0, total, n_out)
    s_target = np.interp(target, cw, s)

    out = np.zeros((n_out, wp.shape[1]), dtype=float)
    for col in range(wp.shape[1]):
        out[:, col] = np.interp(s_target, s, wp[:, col])
    return out


def regen_waypoints(
    waypoint_path: str,
    removal_npz_path: str,
    out_path: str,
    n_out: Optional[int],
    gain: float = 1.0,
    w_min: float = 0.3,
    w_max: float = 3.0,
    eps: float = 1e-9,
    smooth_window: int = 0,
) -> None:
    wp = load_waypoints_9d(waypoint_path)
    grid, meta = load_removal_npz(removal_npz_path)

    rem_i = bilinear_sample(grid, wp[:, 0], wp[:, 1], meta)
    mean_rem = float(np.mean(rem_i[rem_i > 0])) if np.any(rem_i > 0) else float(np.mean(rem_i))

    weight = (mean_rem / (rem_i + eps)) ** gain
    weight = np.clip(weight, w_min, w_max)

    if smooth_window and smooth_window >= 3:
        k = smooth_window
        pad = k // 2
        wpad = np.pad(weight, (pad, pad), mode="reflect")
        kernel = np.ones(k) / k
        weight = np.convolve(wpad, kernel, mode="valid")

    if n_out is None:
        n_out = wp.shape[0]

    wp_new = resample_waypoints_weighted(wp, weight, n_out)
    out_path_obj = Path(out_path)
    out_path_obj.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out_path_obj, wp_new, fmt="%.6f", delimiter="\t")

    dbg = out_path_obj.with_suffix(".debug.txt")
    dbg.write_text(
        f"Input waypoints: {wp.shape[0]}\n"
        f"Output waypoints: {wp_new.shape[0]}\n"
        f"gain={gain}, w_min={w_min}, w_max={w_max}, smooth_window={smooth_window}\n"
        f"mean_rem={mean_rem:.6e}\n"
        f"weight stats: min={weight.min():.3f}, mean={weight.mean():.3f}, max={weight.max():.3f}\n",
        encoding="utf-8",
    )


class PolishingRemovalNode(Node):
    def __init__(self) -> None:
        super().__init__("polishing_removal_node")

        # file paths
        self.declare_parameter("xyz_path", "")
        self.declare_parameter("vxyz_path", "")
        self.declare_parameter("fxyz_path", "")
        self.declare_parameter("rpy_path", "")
        self.declare_parameter("waypoint_path", "")
        self.declare_parameter("removal_npz_path", "")
        self.declare_parameter("out_dir", "./polishing_analysis")
        self.declare_parameter("regenerated_waypoint_path", "./real_flat_filtered_new.txt")

        # analyze params
        self.declare_parameter("cell_mm", 1.0)
        self.declare_parameter("k_preston", 1.0)
        self.declare_parameter("tool_axis", "-Z")
        self.declare_parameter("speed_mode", "xy")
        self.declare_parameter("contact_threshold_N", 0.5)
        self.declare_parameter("speed_threshold_mm_s", 0.1)
        self.declare_parameter("pad_radius_mm", 0.0)

        # regen params
        self.declare_parameter("n_out", 0)
        self.declare_parameter("gain", 1.0)
        self.declare_parameter("w_min", 0.3)
        self.declare_parameter("w_max", 3.0)
        self.declare_parameter("smooth_window", 0)

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.summary_pub = self.create_publisher(String, "~/summary", 10)

        self.analyze_srv = self.create_service(Trigger, "~/analyze", self.handle_analyze)
        self.regen_srv = self.create_service(Trigger, "~/regen", self.handle_regen)

        self.get_logger().info("polishing_removal_node started")
        self.get_logger().info("Services: ~/analyze, ~/regen")

    def _get_param(self, name: str):
        return self.get_parameter(name).value

    def _publish_text(self, topic_pub, text: str) -> None:
        msg = String()
        msg.data = text
        topic_pub.publish(msg)

    def handle_analyze(self, request, response):
        del request
        try:
            xyz_path = self._get_param("xyz_path")
            vxyz_path = self._get_param("vxyz_path")
            fxyz_path = self._get_param("fxyz_path")
            rpy_path = self._get_param("rpy_path") or None
            out_dir = self._get_param("out_dir")

            res = compute_removal(
                xyz_path=xyz_path,
                vxyz_path=vxyz_path,
                fxyz_path=fxyz_path,
                rpy_path=rpy_path,
                cell_mm=float(self._get_param("cell_mm")),
                k_preston=float(self._get_param("k_preston")),
                tool_axis=str(self._get_param("tool_axis")),
                speed_mode=str(self._get_param("speed_mode")),
                contact_threshold_N=float(self._get_param("contact_threshold_N")),
                speed_threshold_mm_s=float(self._get_param("speed_threshold_mm_s")),
                pad_radius_mm=float(self._get_param("pad_radius_mm")),
            )
            saved_dir = save_analysis_outputs(res, out_dir)

            summary_path = Path(saved_dir) / "summary.txt"
            summary_txt = summary_path.read_text(encoding="utf-8") if summary_path.exists() else "analyze done"

            self._publish_text(self.status_pub, f"analyze done: {saved_dir}")
            self._publish_text(self.summary_pub, summary_txt)

            response.success = True
            response.message = f"Analyze completed. Output saved to: {saved_dir}"
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"Analyze failed: {e}"
            self._publish_text(self.status_pub, response.message)
            self.get_logger().error(response.message)
            return response

    def handle_regen(self, request, response):
        del request
        try:
            waypoint_path = self._get_param("waypoint_path")
            removal_npz_path = self._get_param("removal_npz_path")
            out_path = self._get_param("regenerated_waypoint_path")
            n_out_raw = int(self._get_param("n_out"))
            n_out = None if n_out_raw == 0 else n_out_raw

            regen_waypoints(
                waypoint_path=waypoint_path,
                removal_npz_path=removal_npz_path,
                out_path=out_path,
                n_out=n_out,
                gain=float(self._get_param("gain")),
                w_min=float(self._get_param("w_min")),
                w_max=float(self._get_param("w_max")),
                smooth_window=int(self._get_param("smooth_window")),
            )

            debug_path = str(Path(out_path).with_suffix(".debug.txt"))
            self._publish_text(self.status_pub, f"regen done: {out_path}")

            response.success = True
            response.message = f"Regen completed. Waypoints: {out_path}, debug: {debug_path}"
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.success = False
            response.message = f"Regen failed: {e}"
            self._publish_text(self.status_pub, response.message)
            self.get_logger().error(response.message)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = PolishingRemovalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()