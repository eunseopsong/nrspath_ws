"""Generate and publish one continuous waypoint sequence over a complete STL mesh.

This module intentionally lives beside the existing four-region node without
changing it.  Run it as a Python module after building the workspace:

    python3 -m nrs_waypoint_generator.full_workpiece_waypoint_node --ros-args ...
"""

from pathlib import Path

import numpy as np
import rclpy
import trimesh
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from .waypoint_core import (
    axis_from_str,
    batch_raycast_first_hit,
    erosion_mask_by_radius,
    generate_waypoints_for_region,
    make_uv_basis,
    project_to_uv,
    save_xyz,
    uv_basis_from_mesh_pca,
    uv_basis_from_xy_angle,
    uv_to_xyz,
    visualize_mesh_and_waypoints,
)


def generate_full_workpiece_waypoints(
    mesh_path: str,
    *,
    pad_diameter: float = 0.025,
    step: float = 0.01,
    tool_axis: str = "+Z",
    standoff: float = 0.01,
    scan_dir: str = "auto",
    start_corner: str = "minmin",
    grid_cell: float = 0.01,
    frame_mode: str = "manual_xy_angle",
    frame_angle_deg: float = 45.0,
    flip_u: bool = False,
    flip_v: bool = False,
):
    """Return a single serpentine waypoint sequence covering the whole mesh."""
    if step <= 0.0 or grid_cell <= 0.0 or pad_diameter <= 0.0:
        raise ValueError("step, grid_cell and pad_diameter must be > 0")

    mesh = trimesh.load_mesh(mesh_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError("Loaded object is not a Trimesh")

    tool_axis_vec = axis_from_str(tool_axis)
    tool_axis_vec = tool_axis_vec / np.linalg.norm(tool_axis_vec)

    if frame_mode == "manual_xy_angle":
        u, v = uv_basis_from_xy_angle(tool_axis_vec, frame_angle_deg)
    elif frame_mode == "pca":
        u, v = uv_basis_from_mesh_pca(mesh, tool_axis_vec)
    elif frame_mode == "default":
        u, v = make_uv_basis(tool_axis_vec)
    else:
        raise ValueError("frame_mode must be manual_xy_angle, pca or default")

    if flip_u:
        u = -u
    if flip_v:
        v = -v

    uv_vertices = project_to_uv(mesh.vertices, u, v)
    uv_min = uv_vertices.min(axis=0)
    uv_max = uv_vertices.max(axis=0)

    xs = np.arange(uv_min[0], uv_max[0] + 1e-12, grid_cell)
    ys = np.arange(uv_min[1], uv_max[1] + 1e-12, grid_cell)
    xx, yy = np.meshgrid(xs, ys)
    uv_grid = np.stack([xx.reshape(-1), yy.reshape(-1)], axis=1)

    diagonal = np.linalg.norm(mesh.bounds[1] - mesh.bounds[0])
    origin_offset = tool_axis_vec * (diagonal * 2.0)
    origins = uv_to_xyz(uv_grid, u, v) + origin_offset[None, :]
    directions = np.tile(-tool_axis_vec[None, :], (len(origins), 1))
    hit_mask, _, _ = batch_raycast_first_hit(mesh, origins, directions)
    valid_mask = hit_mask.reshape(len(ys), len(xs))
    safe_mask = erosion_mask_by_radius(
        valid_mask, cell=grid_cell, radius=0.5 * pad_diameter
    )

    full_region = {
        "key": "FULL_WORKPIECE",
        "name": "FULL",
        "u_min": uv_min[0],
        "u_max": uv_max[0],
        "v_min": uv_min[1],
        "v_max": uv_max[1],
    }
    _, surface_hits, waypoints = generate_waypoints_for_region(
        region=full_region,
        uv_min=uv_min,
        uv_max=uv_max,
        safe_mask=safe_mask,
        cell=grid_cell,
        step=step,
        tool_axis=tool_axis_vec,
        u=u,
        v=v,
        mesh=mesh,
        origin_offset=origin_offset,
        standoff=standoff,
        scan_dir=scan_dir,
        start_corner=start_corner,
        auto_w_dist=1.0,
        auto_w_nrm=0.05,
    )
    return mesh, surface_hits, waypoints


class FullWorkpieceWaypointNode(Node):
    def __init__(self):
        super().__init__("full_workpiece_waypoint_node")
        self.declare_parameter("mesh", "")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("pad_diameter", 0.025)
        self.declare_parameter("step", 0.01)
        self.declare_parameter("grid_cell", 0.01)
        self.declare_parameter("tool_axis", "+Z")
        self.declare_parameter("standoff", 0.01)
        self.declare_parameter("scan_dir", "auto")
        self.declare_parameter("start_corner", "minmin")
        self.declare_parameter("frame_mode", "manual_xy_angle")
        self.declare_parameter("frame_angle_deg", 45.0)
        self.declare_parameter("flip_u", False)
        self.declare_parameter("flip_v", False)
        self.declare_parameter("save_output", False)
        self.declare_parameter("output_file", "full_workpiece_waypoints.txt")
        self.declare_parameter("save_png", True)
        self.declare_parameter("png_file", "full_workpiece_waypoints.png")
        self.declare_parameter("visualize", True)

        self.mesh_path = str(self.get_parameter("mesh").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.visualize = bool(self.get_parameter("visualize").value)
        self.publisher = self.create_publisher(PointStamped, "/clicked_point", 10)
        self.timer = None
        self.index = 0
        self.init_ok = False

        if not self.mesh_path or not Path(self.mesh_path).is_file():
            self.get_logger().error(f"Mesh file not found: {self.mesh_path!r}")
            return
        if rate <= 0.0:
            self.get_logger().error("publish_rate_hz must be > 0")
            return

        try:
            self.mesh, surface_hits, points = generate_full_workpiece_waypoints(
                self.mesh_path,
                pad_diameter=float(self.get_parameter("pad_diameter").value),
                step=float(self.get_parameter("step").value),
                grid_cell=float(self.get_parameter("grid_cell").value),
                tool_axis=str(self.get_parameter("tool_axis").value),
                standoff=float(self.get_parameter("standoff").value),
                scan_dir=str(self.get_parameter("scan_dir").value),
                start_corner=str(self.get_parameter("start_corner").value),
                frame_mode=str(self.get_parameter("frame_mode").value),
                frame_angle_deg=float(self.get_parameter("frame_angle_deg").value),
                flip_u=bool(self.get_parameter("flip_u").value),
                flip_v=bool(self.get_parameter("flip_v").value),
            )
        except Exception as exc:
            self.get_logger().error(f"Full-workpiece generation failed: {exc}")
            return

        self.waypoints = points.tolist()
        if not self.waypoints:
            self.get_logger().error("No safe waypoint was generated from the mesh")
            return

        if bool(self.get_parameter("save_output").value):
            output_file = Path(str(self.get_parameter("output_file").value))
            output_file.parent.mkdir(parents=True, exist_ok=True)
            save_xyz(
                output_file,
                points,
                f"# full-workpiece waypoint xyz\n# mesh={self.mesh_path}\n# columns: x y z\n",
            )
            self.get_logger().info(f"Saved waypoints to {output_file}")

        if bool(self.get_parameter("save_png").value):
            png_file = Path(str(self.get_parameter("png_file").value))
            try:
                png_file.parent.mkdir(parents=True, exist_ok=True)
                visualize_mesh_and_waypoints(
                    self.mesh,
                    {1: self.waypoints},
                    output_path=png_file,
                    show=False,
                )
                self.get_logger().info(f"Saved waypoint preview PNG to {png_file.resolve()}")
            except Exception as exc:
                self.get_logger().error(f"Failed to save waypoint preview PNG: {exc}")

        self.get_logger().info(
            f"Generated {len(self.waypoints)} continuous waypoints from the complete STL"
        )
        self.timer = self.create_timer(1.0 / rate, self._publish_next)
        self.init_ok = True

    def _publish_next(self):
        if self.index >= len(self.waypoints):
            self.timer.cancel()
            self.get_logger().info("All full-workpiece waypoints have been published")
            if self.visualize:
                visualize_mesh_and_waypoints(self.mesh, {1: self.waypoints})
            rclpy.shutdown()
            return

        x, y, z = self.waypoints[self.index]
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.point.x = float(x)
        message.point.y = float(y)
        message.point.z = float(z)
        self.publisher.publish(message)
        self.index += 1

        if self.index == 1 or self.index % 100 == 0:
            self.get_logger().info(
                f"Published {self.index}/{len(self.waypoints)} full-workpiece waypoints"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FullWorkpieceWaypointNode()
    if not node.init_ok:
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
