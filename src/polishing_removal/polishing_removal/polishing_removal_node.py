#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional
import os
import threading
from datetime import datetime

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix, get_package_share_directory

# ============================================================
# Assumptions for this node
# - /ur10skku/currentP: [x(mm), y(mm), z(mm), wx, wy, wz]
# - /ur10skku/currentF: [fx(N), fy(N), fz(N), tx, ty, tz]
# - Positions are already in mm.
# - Recording starts with ~/start and stops/processes with ~/end.
# - Only 4 PNG files are saved in the session folder.
# - The time plot uses the SAME quantity as the heatmap:
#   "cell removal value" sampled from the final heatmap at each timestep.
# ============================================================
DEFAULT_PACKAGE_NAME = "polishing_removal"
DEFAULT_POSITION_TOPIC = "/ur10skku/currentP"
DEFAULT_FORCE_TOPIC = "/ur10skku/currentF"
DEFAULT_RECORDING_RATE_HZ = 50.0
DEFAULT_TOOL_AXIS = "+Z"
DEFAULT_SPEED_MODE = "xy"
DEFAULT_CONTACT_THRESHOLD_N = 0.5
DEFAULT_SPEED_THRESHOLD_MM_S = 0.1
DEFAULT_CELL_MM = 1.0
DEFAULT_PAD_RADIUS_MM = 20.0
DEFAULT_K_PRESTON = 1.0
DEFAULT_W_ARROW_STRIDE = 20
DEFAULT_W_ARROW_LENGTH_MM = 15.0


@dataclass
class RemovalResult:
    t: np.ndarray              # [N]
    state6: np.ndarray         # [N,6] = [x,y,z,wx,wy,wz]
    force3: np.ndarray         # [N,3] = [fx,fy,fz]
    vxyz: np.ndarray           # [N,3]
    dt: np.ndarray             # [N]
    fn: np.ndarray             # [N]
    speed: np.ndarray          # [N]
    removal_rate: np.ndarray   # [N]
    dremoval: np.ndarray       # [N]
    contact_mask: np.ndarray   # [N]
    grid_removal: np.ndarray   # [H,W]
    grid_hits: np.ndarray      # [H,W]
    grid_dt: np.ndarray        # [H,W]
    xi: np.ndarray             # [N]
    yi: np.ndarray             # [N]
    heatmap_value_series: np.ndarray  # [N], sampled from final heatmap
    meta: Dict[str, float]


def parse_tool_axis(axis_str: str) -> np.ndarray:
    axis_str = axis_str.strip().upper()
    if axis_str in ["+Z", "Z", "Z+", "+3"]:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    if axis_str in ["-Z", "Z-", "-3"]:
        return np.array([0.0, 0.0, -1.0], dtype=float)
    if axis_str in ["+X", "X", "X+", "+1"]:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    if axis_str in ["-X", "X-", "-1"]:
        return np.array([-1.0, 0.0, 0.0], dtype=float)
    if axis_str in ["+Y", "Y", "Y+", "+2"]:
        return np.array([0.0, 1.0, 0.0], dtype=float)
    if axis_str in ["-Y", "Y-", "-2"]:
        return np.array([0.0, -1.0, 0.0], dtype=float)
    raise ValueError("tool_axis must be one of ±X/±Y/±Z")


def fft_convolve2d(a: np.ndarray, k: np.ndarray) -> np.ndarray:
    h, w = a.shape
    kh, kw = k.shape
    out_h = h + kh - 1
    out_w = w + kw - 1

    fh = int(2 ** np.ceil(np.log2(out_h)))
    fw = int(2 ** np.ceil(np.log2(out_w)))

    A = np.fft.rfft2(a, s=(fh, fw))
    K = np.fft.rfft2(k, s=(fh, fw))
    y = np.fft.irfft2(A * K, s=(fh, fw))

    start_h = (kh - 1) // 2
    start_w = (kw - 1) // 2
    return y[start_h:start_h + h, start_w:start_w + w]


def find_package_logs_dir(package_name: str) -> Path:
    """
    Prefer workspace source path: <ws>/src/<package_name>/logs
    Fallback: <install>/share/<package_name>/logs
    """
    try:
        prefix = Path(get_package_prefix(package_name)).resolve()
        for parent in [prefix] + list(prefix.parents):
            src_pkg = parent / "src" / package_name
            if src_pkg.exists():
                logs_dir = src_pkg / "logs"
                logs_dir.mkdir(parents=True, exist_ok=True)
                return logs_dir
    except Exception:
        pass

    share_dir = Path(get_package_share_directory(package_name))
    logs_dir = share_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    return logs_dir


def compute_velocity_mm_s(t: np.ndarray, xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dt = np.diff(t, prepend=t[0])
    if t.shape[0] >= 2:
        dt0 = float(np.median(np.diff(t)))
    else:
        dt0 = 1.0 / 50.0
    dt[0] = max(dt0, 1e-6)
    dt = np.clip(dt, 1e-6, None)

    vxyz = np.zeros_like(xyz)
    if xyz.shape[0] >= 2:
        dxyz = np.diff(xyz, axis=0)
        dt_seg = np.diff(t)
        dt_seg = np.clip(dt_seg, 1e-6, None)
        vxyz[1:] = dxyz / dt_seg[:, None]
        vxyz[0] = vxyz[1]
    return vxyz, dt


def compute_removal_from_recording(
    t: np.ndarray,
    state6: np.ndarray,
    force3: np.ndarray,
    cell_mm: float,
    k_preston: float,
    tool_axis: str,
    speed_mode: str,
    contact_threshold_N: float,
    speed_threshold_mm_s: float,
    pad_radius_mm: float,
) -> RemovalResult:
    xyz = state6[:, 0:3]
    wxyz = state6[:, 3:6]

    vxyz, dt = compute_velocity_mm_s(t, xyz)

    if speed_mode.lower() == "xy":
        speed = np.linalg.norm(vxyz[:, 0:2], axis=1)
    elif speed_mode.lower() == "xyz":
        speed = np.linalg.norm(vxyz, axis=1)
    else:
        raise ValueError("speed_mode must be 'xy' or 'xyz'.")

    axis_base = parse_tool_axis(tool_axis)
    fn1 = np.sum(force3 * axis_base[None, :], axis=1)
    fn2 = -fn1
    score1 = np.mean(np.maximum(fn1, 0.0))
    score2 = np.mean(np.maximum(fn2, 0.0))
    fn = np.maximum(fn1 if score1 >= score2 else fn2, 0.0)

    contact_mask = (fn >= contact_threshold_N) & (speed >= speed_threshold_mm_s)
    removal_rate = np.zeros_like(fn)
    removal_rate[contact_mask] = k_preston * fn[contact_mask] * speed[contact_mask]
    dremoval = removal_rate * dt

    x = xyz[:, 0]
    y = xyz[:, 1]
    x_min = np.floor(np.min(x) / cell_mm) * cell_mm
    x_max = np.ceil(np.max(x) / cell_mm) * cell_mm
    y_min = np.floor(np.min(y) / cell_mm) * cell_mm
    y_max = np.ceil(np.max(y) / cell_mm) * cell_mm

    W = int(np.round((x_max - x_min) / cell_mm)) + 1
    H = int(np.round((y_max - y_min) / cell_mm)) + 1
    W = max(W, 1)
    H = max(H, 1)

    xi = np.clip(np.round((x - x_min) / cell_mm).astype(int), 0, W - 1)
    yi = np.clip(np.round((y - y_min) / cell_mm).astype(int), 0, H - 1)

    grid_removal = np.zeros((H, W), dtype=float)
    grid_hits = np.zeros((H, W), dtype=float)
    grid_dt = np.zeros((H, W), dtype=float)

    # Heatmap quantity: cumulative cell removal.
    np.add.at(grid_removal, (yi, xi), dremoval)
    np.add.at(grid_hits, (yi, xi), 1.0)
    np.add.at(grid_dt, (yi, xi), dt)

    if pad_radius_mm > 0.0:
        r_cells = int(np.ceil(pad_radius_mm / cell_mm))
        if r_cells >= 1:
            yy, xx = np.mgrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
            mask = (xx ** 2 + yy ** 2) <= (pad_radius_mm / cell_mm) ** 2
            kernel = mask.astype(float)
            kernel /= np.sum(kernel)
            grid_removal = fft_convolve2d(grid_removal, kernel)
            grid_hits = fft_convolve2d(grid_hits, kernel)
            grid_dt = fft_convolve2d(grid_dt, kernel)

    # Time-series uses the SAME quantity as the heatmap: final cell removal value.
    heatmap_value_series = grid_removal[yi, xi].copy()

    visited = grid_hits > 0
    grid_vals = grid_removal[visited] if np.any(visited) else np.array([0.0])
    meta = {
        "x_min": float(x_min),
        "x_max": float(x_max),
        "y_min": float(y_min),
        "y_max": float(y_max),
        "cell_mm": float(cell_mm),
        "k_preston": float(k_preston),
        "contact_threshold_N": float(contact_threshold_N),
        "speed_threshold_mm_s": float(speed_threshold_mm_s),
        "pad_radius_mm": float(pad_radius_mm),
        "mean_heatmap_removal": float(np.mean(grid_vals)),
        "std_heatmap_removal": float(np.std(grid_vals)),
        "samples": int(t.size),
        "contact_samples": int(np.sum(contact_mask)),
        "duration_s": float(t[-1] - t[0]) if t.size >= 2 else 0.0,
        "max_speed_mm_s": float(np.max(speed)) if speed.size > 0 else 0.0,
        "max_fn_N": float(np.max(fn)) if fn.size > 0 else 0.0,
        "mean_w_mag": float(np.mean(np.linalg.norm(wxyz, axis=1))) if wxyz.size > 0 else 0.0,
    }

    return RemovalResult(
        t=t,
        state6=state6,
        force3=force3,
        vxyz=vxyz,
        dt=dt,
        fn=fn,
        speed=speed,
        removal_rate=removal_rate,
        dremoval=dremoval,
        contact_mask=contact_mask,
        grid_removal=grid_removal,
        grid_hits=grid_hits,
        grid_dt=grid_dt,
        xi=xi,
        yi=yi,
        heatmap_value_series=heatmap_value_series,
        meta=meta,
    )


def create_heatmap_figure(res: RemovalResult):
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
        f"mean = {mean_rem:.6f} [a.u.]\\n"
        f"std  = {std_rem:.6f} [a.u.]\\n"
        f"samples = {res.meta['samples']}\\n"
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
    res: RemovalResult,
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

    # 2) Time-series of the SAME quantity as heatmap: sampled final cell removal.
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
        valid = (w_norm[:, 0] > 1e-12)
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


class PolishingRemovalNode(Node):
    def __init__(self) -> None:
        super().__init__("polishing_removal_node")

        self.declare_parameter("package_name", DEFAULT_PACKAGE_NAME)
        self.declare_parameter("position_topic", DEFAULT_POSITION_TOPIC)
        self.declare_parameter("force_topic", DEFAULT_FORCE_TOPIC)
        self.declare_parameter("recording_rate_hz", DEFAULT_RECORDING_RATE_HZ)
        self.declare_parameter("tool_axis", DEFAULT_TOOL_AXIS)
        self.declare_parameter("speed_mode", DEFAULT_SPEED_MODE)
        self.declare_parameter("contact_threshold_N", DEFAULT_CONTACT_THRESHOLD_N)
        self.declare_parameter("speed_threshold_mm_s", DEFAULT_SPEED_THRESHOLD_MM_S)
        self.declare_parameter("cell_mm", DEFAULT_CELL_MM)
        self.declare_parameter("pad_radius_mm", DEFAULT_PAD_RADIUS_MM)
        self.declare_parameter("k_preston", DEFAULT_K_PRESTON)
        self.declare_parameter("w_arrow_stride", DEFAULT_W_ARROW_STRIDE)
        self.declare_parameter("w_arrow_length_mm", DEFAULT_W_ARROW_LENGTH_MM)

        self.package_name = self.get_parameter("package_name").get_parameter_value().string_value
        self.position_topic = self.get_parameter("position_topic").get_parameter_value().string_value
        self.force_topic = self.get_parameter("force_topic").get_parameter_value().string_value
        self.recording_rate_hz = self.get_parameter("recording_rate_hz").get_parameter_value().double_value

        self.logs_root = find_package_logs_dir(self.package_name)
        self.latest_state6: Optional[np.ndarray] = None
        self.latest_force3: Optional[np.ndarray] = None
        self.recording = False
        self.record_start_time_sec: Optional[float] = None
        self.session_dir: Optional[Path] = None
        self._heatmap_fig = None

        self.time_buffer: list[float] = []
        self.state_buffer: list[np.ndarray] = []
        self.force_buffer: list[np.ndarray] = []

        self._lock = threading.Lock()

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.summary_pub = self.create_publisher(String, "~/summary", 10)

        self.pose_sub = self.create_subscription(Float64MultiArray, self.position_topic, self.position_callback, 10)
        self.force_sub = self.create_subscription(Float64MultiArray, self.force_topic, self.force_callback, 10)

        self.start_srv = self.create_service(Trigger, "~/start", self.handle_start)
        self.end_srv = self.create_service(Trigger, "~/end", self.handle_end)

        timer_period = 1.0 / max(self.recording_rate_hz, 1e-3)
        self.record_timer = self.create_timer(timer_period, self.record_timer_callback)

        self.get_logger().info("polishing_removal_node started")
        self.get_logger().info(f"position_topic: {self.position_topic}")
        self.get_logger().info(f"force_topic: {self.force_topic}")
        self.get_logger().info("currentP format assumed: [x, y, z, wx, wy, wz]")
        self.get_logger().info("currentF format assumed: [fx, fy, fz, tx, ty, tz]")
        self.get_logger().info(f"logs save root: {self.logs_root}")
        self.get_logger().info("Services: ~/start, ~/end")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_summary(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.summary_pub.publish(msg)

    def position_callback(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=float)
        if data.size < 6:
            self.get_logger().warn("Received currentP with fewer than 6 elements. Ignored.")
            return
        with self._lock:
            self.latest_state6 = data[:6].copy()

    def force_callback(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=float)
        if data.size < 3:
            self.get_logger().warn("Received currentF with fewer than 3 elements. Ignored.")
            return
        with self._lock:
            self.latest_force3 = data[:3].copy()

    def record_timer_callback(self) -> None:
        with self._lock:
            if not self.recording:
                return
            if self.latest_state6 is None or self.latest_force3 is None or self.record_start_time_sec is None:
                return

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            t_rel = now_sec - self.record_start_time_sec
            self.time_buffer.append(float(t_rel))
            self.state_buffer.append(self.latest_state6.copy())
            self.force_buffer.append(self.latest_force3.copy())

    def handle_start(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if self.recording:
                response.success = False
                response.message = "Recording is already running."
                return response

            self.recording = True
            self.record_start_time_sec = self.get_clock().now().nanoseconds * 1e-9
            self.time_buffer = []
            self.state_buffer = []
            self.force_buffer = []

            session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
            self.session_dir = self.logs_root / session_name
            self.session_dir.mkdir(parents=True, exist_ok=True)

        self.publish_status(f"Recording started. Saving to: {self.session_dir}")
        response.success = True
        response.message = f"Recording started. Output dir: {self.session_dir}"
        return response

    def handle_end(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if not self.recording:
                response.success = False
                response.message = "Recording is not running."
                return response

            self.recording = False
            t = np.asarray(self.time_buffer, dtype=float)
            state6 = np.asarray(self.state_buffer, dtype=float)
            force3 = np.asarray(self.force_buffer, dtype=float)
            session_dir = self.session_dir

        if t.size < 2 or state6.shape[0] < 2 or force3.shape[0] < 2:
            response.success = False
            response.message = "Not enough recorded samples to process."
            self.publish_status(response.message)
            return response

        try:
            res = compute_removal_from_recording(
                t=t,
                state6=state6,
                force3=force3,
                cell_mm=float(self.get_parameter("cell_mm").value),
                k_preston=float(self.get_parameter("k_preston").value),
                tool_axis=str(self.get_parameter("tool_axis").value),
                speed_mode=str(self.get_parameter("speed_mode").value),
                contact_threshold_N=float(self.get_parameter("contact_threshold_N").value),
                speed_threshold_mm_s=float(self.get_parameter("speed_threshold_mm_s").value),
                pad_radius_mm=float(self.get_parameter("pad_radius_mm").value),
            )
            assert session_dir is not None
            saved_files = save_required_plots_only(
                res,
                session_dir,
                w_arrow_stride=int(self.get_parameter("w_arrow_stride").value),
                w_arrow_length_mm=float(self.get_parameter("w_arrow_length_mm").value),
            )

            # Show heatmap window immediately after processing.
            try:
                self._heatmap_fig = create_heatmap_figure(res)
                self._heatmap_fig.show()
                plt.show(block=False)
                plt.pause(0.001)
                self.publish_status("Heatmap window displayed.")
            except Exception as e:
                self.publish_status(f"Heatmap window display skipped: {e}")

            summary_text = (
                f"saved_dir: {session_dir}\n"
                f"files: {', '.join(saved_files)}\n"
                f"samples: {res.meta['samples']}\n"
                f"duration_s: {res.meta['duration_s']:.6f}\n"
                f"mean_heatmap_removal: {res.meta['mean_heatmap_removal']:.6f}\n"
                f"std_heatmap_removal: {res.meta['std_heatmap_removal']:.6f}\n"
                f"max_speed_mm_s: {res.meta['max_speed_mm_s']:.6f}\n"
                f"max_fn_N: {res.meta['max_fn_N']:.6f}"
            )
            self.publish_summary(summary_text)
            self.publish_status(f"Recording stopped. Saved 4 files to: {session_dir}")
            response.success = True
            response.message = f"Saved 4 files to {session_dir}"
            return response

        except Exception as e:
            err = f"Failed to process recording: {e}"
            self.publish_status(err)
            response.success = False
            response.message = err
            return response


def main(args=None) -> None:
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