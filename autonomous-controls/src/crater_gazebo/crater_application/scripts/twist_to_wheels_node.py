#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class TwistToWheelsNode(Node):
    def __init__(self) -> None:
        super().__init__("twist_to_wheels_node")

        self.declare_parameter("half_wheelbase", 0.2)   # front/rear axle to centre
        self.declare_parameter("half_track", 0.33)       # left/right wheel to centre
        self.declare_parameter("wheel_radius", 0.15)
        self.declare_parameter("cmd_vel_topic", "/rover/cmd_vel")

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._lx = float(self.get_parameter("half_wheelbase").value)
        self._ly = float(self.get_parameter("half_track").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)

        self._pub_fl = self.create_publisher(Float64, "/rover/front_left_wheel_vel_cmd", 10)
        self._pub_rl = self.create_publisher(Float64, "/rover/rear_left_wheel_vel_cmd", 10)
        self._pub_fr = self.create_publisher(Float64, "/rover/front_right_wheel_vel_cmd", 10)
        self._pub_rr = self.create_publisher(Float64, "/rover/rear_right_wheel_vel_cmd", 10)
        self._sub = self.create_subscription(Twist, cmd_vel_topic, self._on_twist, 20)

        self.get_logger().info(
            "Omnidirectional drive: converting /rover/cmd_vel -> per-wheel velocities on "
            "/rover/*_wheel_vel_cmd"
        )

    def _on_twist(self, msg: Twist) -> None:
        vx = msg.linear.x   # forward
        vy = msg.linear.y   # lateral (positive = left)
        wz = msg.angular.z  # yaw rate

        # Mecanum wheel inverse kinematics
        # l = lx + ly is the geometric factor combining half-wheelbase and half-track
        l = self._lx + self._ly
        r = self._wheel_radius

        omega_fl = (vx - vy - l * wz) / r
        omega_fr = (vx + vy + l * wz) / r
        omega_rl = (vx + vy - l * wz) / r
        omega_rr = (vx - vy + l * wz) / r

        fl = Float64(); fl.data = omega_fl; self._pub_fl.publish(fl)
        fr = Float64(); fr.data = omega_fr; self._pub_fr.publish(fr)
        rl = Float64(); rl.data = omega_rl; self._pub_rl.publish(rl)
        rr = Float64(); rr.data = omega_rr; self._pub_rr.publish(rr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistToWheelsNode()
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
