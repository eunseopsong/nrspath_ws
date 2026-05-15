import rclpy
import trimesh

from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from .waypoint_core import (
    generate_waypoints_by_region,
    visualize_mesh_and_waypoints,
)


class GenerateAndPublishClickedPointNode(Node):
    def __init__(self):
        super().__init__("generate_and_publish_clicked_point_node")

        self.init_ok = False

        self.declare_parameter("mesh", "")
        self.declare_parameter("region_id", 1)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("save_output", False)
        self.declare_parameter("output_dir", "cross_corner_waypoints")

        self.mesh_path = self.get_parameter("mesh").value
        self.region_id = int(self.get_parameter("region_id").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.save_output = bool(self.get_parameter("save_output").value)
        self.output_dir = self.get_parameter("output_dir").value

        self.pub_ = self.create_publisher(PointStamped, "/clicked_point", 10)

        self.region_waypoints = {}
        self.waypoints = []
        self.index = 0
        self.timer = None

        if self.region_id not in [1, 2, 3, 4]:
            self.get_logger().error("region_id must be one of 1, 2, 3, 4")
            return

        if not self.mesh_path:
            self.get_logger().error("Parameter 'mesh' is empty.")
            return

        if self.publish_rate_hz <= 0.0:
            self.get_logger().error("publish_rate_hz must be > 0")
            return

        self.get_logger().info(f"Mesh: {self.mesh_path}")
        self.get_logger().info(f"Region: {self.region_id}")

        try:
            self.region_waypoints = generate_waypoints_by_region(
                mesh_path=self.mesh_path,
                save_output=self.save_output,
                out_dir=self.output_dir,
                visualize=False,   # 생성 직후가 아니라 publish 끝난 뒤 시각화
            )
        except Exception as e:
            self.get_logger().error(f"Waypoint generation failed: {e}")
            return

        if self.region_id not in self.region_waypoints:
            self.get_logger().error(f"Region {self.region_id} not found in generated result.")
            return

        self.waypoints = self.region_waypoints[self.region_id]

        if len(self.waypoints) == 0:
            self.get_logger().error(f"Region {self.region_id} has no waypoints.")
            return

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from generated region {self.region_id}"
        )

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.init_ok = True

    def visualize_after_publish(self):
        try:
            self.get_logger().info("Loading mesh for visualization...")
            mesh = trimesh.load_mesh(self.mesh_path, force="mesh")
            self.get_logger().info("Opening 3D visualization window...")
            visualize_mesh_and_waypoints(mesh, self.region_waypoints)
        except Exception as e:
            self.get_logger().warn(f"Visualization failed: {e}")

    def on_timer(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info("All generated waypoints published.")

            if self.timer is not None:
                self.timer.cancel()

            self.visualize_after_publish()

            self.get_logger().info("Shutting down.")
            self.destroy_node()
            rclpy.shutdown()
            return

        pt = self.waypoints[self.index]

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(pt[0])
        msg.point.y = float(pt[1])
        msg.point.z = float(pt[2])

        self.pub_.publish(msg)

        self.get_logger().info(
            f"Published waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"[{pt[0]:.6f}, {pt[1]:.6f}, {pt[2]:.6f}]"
        )

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = GenerateAndPublishClickedPointNode()

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