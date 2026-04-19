#!/usr/bin/python3
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_node")

        self.declare_parameter("path_topic", "/rover/global_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/rover/cmd_vel")
        self.declare_parameter("lookahead_m", 0.7)
        self.declare_parameter("goal_tolerance_m", 0.35)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("k_ang", 1.6)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("align_path_to_start_pose", True)
        self.declare_parameter("enforce_goal_stop", False)

        self._lookahead = float(self.get_parameter("lookahead_m").value)
        self._goal_tol = float(self.get_parameter("goal_tolerance_m").value)
        self._v_max = float(self.get_parameter("max_linear_speed").value)
        self._w_max = float(self.get_parameter("max_angular_speed").value)
        self._k_ang = float(self.get_parameter("k_ang").value)
        self._align_path = bool(self.get_parameter("align_path_to_start_pose").value)
        self._enforce_goal_stop = bool(self.get_parameter("enforce_goal_stop").value)

        path_topic = str(self.get_parameter("path_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        control_hz = float(self.get_parameter("control_hz").value)

        self._path_raw: List[Tuple[float, float]] = []
        self._path_shifted: List[Tuple[float, float]] = []
        self._goal_reached = False
        self._start_aligned = False
        self._path_origin = (0.0, 0.0)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._have_odom = False

        self._sub_path = self.create_subscription(Path, path_topic, self._on_path, 10)
        self._sub_odom = self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)

        timer_period = max(0.02, 1.0 / max(1.0, control_hz))
        self._timer = self.create_timer(timer_period, self._on_control)

        self.get_logger().info(
            f"Path follower ready. path={path_topic}, odom={odom_topic}, cmd_vel={cmd_vel_topic}"
        )

    def _path_signature(self, points: List[Tuple[float, float]]) -> Tuple[int, float, float, float, float]:
        if not points:
            return (0, 0.0, 0.0, 0.0, 0.0)
        return (
            len(points),
            round(points[0][0], 3),
            round(points[0][1], 3),
            round(points[-1][0], 3),
            round(points[-1][1], 3),
        )

    def _on_path(self, msg: Path) -> None:
        points: List[Tuple[float, float]] = []
        for pose in msg.poses:
            points.append((pose.pose.position.x, pose.pose.position.y))

        if len(points) < 2:
            self.get_logger().warn("Received path with fewer than 2 points.")
            return

        if self._path_signature(points) == self._path_signature(self._path_raw):
            # Ignore periodic republish of the same static path.
            return

        self._path_raw = points
        self._path_shifted = points.copy()
        self._path_origin = points[0]
        self._goal_reached = False
        self._start_aligned = False
        self.get_logger().info(f"Received path with {len(points)} points.")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._x = p.x
        self._y = p.y
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._have_odom = True

        if self._align_path and self._path_raw and not self._start_aligned:
            dx = self._x - self._path_origin[0]
            dy = self._y - self._path_origin[1]
            self._path_shifted = [(x + dx, y + dy) for (x, y) in self._path_raw]
            self._start_aligned = True
            self.get_logger().info(
                "Aligned global path to rover's current start pose."
            )

    def _publish_stop(self) -> None:
        self._pub_cmd.publish(Twist())

    def _choose_target(self) -> Tuple[float, float]:
        assert self._path_shifted

        # Progress anchor: closest point to current pose
        closest_i = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(self._path_shifted):
            d2 = (px - self._x) ** 2 + (py - self._y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                closest_i = i

        # Pick first point beyond lookahead; otherwise final point
        for i in range(closest_i, len(self._path_shifted)):
            px, py = self._path_shifted[i]
            d = math.hypot(px - self._x, py - self._y)
            if d >= self._lookahead:
                return px, py

        return self._path_shifted[-1]

    def _on_control(self) -> None:
        if not self._have_odom or not self._path_shifted:
            self._publish_stop()
            return

        if self._enforce_goal_stop:
            gx, gy = self._path_shifted[-1]
            dist_goal = math.hypot(gx - self._x, gy - self._y)
            if dist_goal <= self._goal_tol:
                if not self._goal_reached:
                    self.get_logger().info("Goal reached. Stopping rover.")
                    self._goal_reached = True
                self._publish_stop()
                return

        tx, ty = self._choose_target()
        heading_to_target = math.atan2(ty - self._y, tx - self._x)
        heading_err = normalize_angle(heading_to_target - self._yaw)

        cmd = Twist()

        # Keep a small forward speed while turning so the rover does not get stuck
        # in rotate-only oscillation when following long global paths.
        if abs(heading_err) > 2.6:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = max(0.06, self._v_max * (1.0 - abs(heading_err) / math.pi))
            cmd.linear.x = min(self._v_max, cmd.linear.x)

        cmd.angular.z = max(-self._w_max, min(self._w_max, self._k_ang * heading_err))
        self._pub_cmd.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
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
