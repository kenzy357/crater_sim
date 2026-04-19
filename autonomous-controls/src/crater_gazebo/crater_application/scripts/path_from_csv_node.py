#!/usr/bin/python3
import csv
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PathFromCsvNode(Node):
    def __init__(self) -> None:
        super().__init__("path_from_csv_node")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("topic_name", "/rover/global_path")
        self.declare_parameter("publish_hz", 1.0)

        self._csv_path = str(self.get_parameter("csv_path").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._topic_name = str(self.get_parameter("topic_name").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Path, self._topic_name, qos)

        self._points = self._load_xy_csv(self._csv_path)
        self._points = self._trim_duplicate_closing_point(self._points)
        if len(self._points) < 2:
            self.get_logger().warn(
                "Path has <2 points; using fallback straight line."
            )
            self._points = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0)]

        timer_period = max(0.2, 1.0 / max(0.1, publish_hz))
        self._timer = self.create_timer(timer_period, self._publish_path)

        self.get_logger().info(
            f"Publishing {len(self._points)} waypoints on {self._topic_name} "
            f"from '{self._csv_path}' in frame '{self._frame_id}'."
        )

    def _load_xy_csv(self, csv_path: str) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        if not csv_path:
            self.get_logger().warn("Parameter csv_path is empty.")
            return points

        try:
            with open(csv_path, "r", newline="") as f:
                reader = csv.DictReader(f)
                if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                    raise ValueError(
                        "CSV must contain headers 'x,y'. "
                        f"Found: {reader.fieldnames}"
                    )

                for row in reader:
                    points.append((float(row["x"]), float(row["y"])))
        except Exception as exc:
            self.get_logger().error(f"Failed to read '{csv_path}': {exc}")

        return points

    def _trim_duplicate_closing_point(
        self, points: List[Tuple[float, float]], tol: float = 0.15
    ) -> List[Tuple[float, float]]:
        # Global planner often exports closed loops with identical first/last point.
        # Remove duplicate end point so follower doesn't consider the goal reached
        # immediately at startup.
        if len(points) < 3:
            return points
        x0, y0 = points[0]
        x1, y1 = points[-1]
        if ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 <= tol:
            return points[:-1]
        return points

    def _publish_path(self) -> None:
        now = self.get_clock().now().to_msg()

        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id

        for x, y in self._points:
            p = PoseStamped()
            p.header.stamp = now
            p.header.frame_id = self._frame_id
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFromCsvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
