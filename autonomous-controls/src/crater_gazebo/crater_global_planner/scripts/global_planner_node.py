#!/usr/bin/python3
import json
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from planner_core import build_total_path, dilate_obstacles, grid_to_world, world_to_grid


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("global_planner_node")

        self.declare_parameter("grid_path", "")
        self.declare_parameter("metadata_path", "")
        self.declare_parameter("config_json_path", "")
        self.declare_parameter("topic_name", "/rover/global_path")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("dilation_radius", -1)

        self._grid_path = str(self.get_parameter("grid_path").value)
        self._metadata_path = str(self.get_parameter("metadata_path").value)
        self._config_json_path = str(self.get_parameter("config_json_path").value)
        self._topic_name = str(self.get_parameter("topic_name").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._dilation_radius_override = int(self.get_parameter("dilation_radius").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Path, self._topic_name, qos)

        self._path_world = self._compute_path_world()
        if self._path_world is None or len(self._path_world) < 2:
            self.get_logger().error("Global planner did not produce a valid path.")
            self._path_world = np.array([[0.0, 0.0], [3.0, 0.0], [6.0, 0.0]])

        timer_period = max(0.2, 1.0 / max(0.1, publish_hz))
        self._timer = self.create_timer(timer_period, self._publish_path)
        self.get_logger().info(
            f"Publishing planned path with {len(self._path_world)} points on {self._topic_name}"
        )

    def _compute_path_world(self):
        if not self._grid_path or not self._metadata_path or not self._config_json_path:
            self.get_logger().error(
                "Missing planner inputs. Set grid_path, metadata_path, and config_json_path."
            )
            return None

        try:
            grid = np.load(self._grid_path)
            min_x, min_y, resolution = np.load(self._metadata_path)
            with open(self._config_json_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        except Exception as exc:
            self.get_logger().error(f"Failed loading planner inputs: {exc}")
            return None

        coords = cfg.get("coordinates", {})
        planning = cfg.get("planning", {})
        try:
            base = np.array(coords["base_s4"], dtype=float)
            all_waypoints = np.array(coords["all_waypoints"], dtype=float)
            selected_idx = coords["selected_indices"]
        except KeyError as exc:
            self.get_logger().error(f"Missing required key in planner config: {exc}")
            return None

        if bool(coords.get("flip_y", True)):
            base[1] *= -1.0
            all_waypoints[:, 1] *= -1.0

        selected_wps = all_waypoints[selected_idx]
        route = np.vstack([base, selected_wps, base])
        wp_grid = world_to_grid(route, float(min_x), float(min_y), float(resolution))

        dilation_radius = int(planning.get("dilation_radius", 1))
        if self._dilation_radius_override >= 0:
            dilation_radius = self._dilation_radius_override

        safe_grid = dilate_obstacles(grid.astype(int), radius=dilation_radius)
        path_cells, order = build_total_path(safe_grid, wp_grid)
        if path_cells is None:
            self.get_logger().error(f"No valid path found. Order={order}")
            return None

        self.get_logger().info(f"Planning succeeded. Order={order}, cells={len(path_cells)}")
        return grid_to_world(path_cells, float(min_x), float(min_y), float(resolution))

    def _publish_path(self) -> None:
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        for x, y in self._path_world:
            p = PoseStamped()
            p.header.stamp = now
            p.header.frame_id = self._frame_id
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self._pub.publish(msg)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
