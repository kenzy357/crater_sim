#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class WheelsCmdDemuxNode(Node):
    def __init__(self) -> None:
        super().__init__("wheels_cmd_demux_node")

        self._pub_fl = self.create_publisher(Float64, "/rover/front_left_wheel_vel_cmd", 10)
        self._pub_fr = self.create_publisher(Float64, "/rover/front_right_wheel_vel_cmd", 10)
        self._pub_rl = self.create_publisher(Float64, "/rover/rear_left_wheel_vel_cmd", 10)
        self._pub_rr = self.create_publisher(Float64, "/rover/rear_right_wheel_vel_cmd", 10)
        self._sub = self.create_subscription(
            Float64MultiArray, "/rover/wheels_cmd", self._on_wheels_cmd, 20
        )

        self.get_logger().info(
            "Listening on /rover/wheels_cmd [fl, fr, rl, rr] "
            "and publishing to per-wheel command topics."
        )

    def _on_wheels_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(
                "Expected 4 values [fl, fr, rl, rr], "
                f"got {len(msg.data)}."
            )
            return

        fl, fr, rl, rr = msg.data

        out_fl = Float64()
        out_fr = Float64()
        out_rl = Float64()
        out_rr = Float64()
        out_fl.data = float(fl)
        out_fr.data = float(fr)
        out_rl.data = float(rl)
        out_rr.data = float(rr)

        self._pub_fl.publish(out_fl)
        self._pub_fr.publish(out_fr)
        self._pub_rl.publish(out_rl)
        self._pub_rr.publish(out_rr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelsCmdDemuxNode()
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
