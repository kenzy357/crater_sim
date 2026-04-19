#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class WheelsCmdDemuxNode(Node):
    def __init__(self) -> None:
        super().__init__("wheels_cmd_demux_node")

        self._pub_fl_steer = self.create_publisher(Float64, "/rover/front_left_steering_pos", 10)
        self._pub_fr_steer = self.create_publisher(Float64, "/rover/front_right_steering_pos", 10)
        self._pub_rl_steer = self.create_publisher(Float64, "/rover/rear_left_steering_pos", 10)
        self._pub_rr_steer = self.create_publisher(Float64, "/rover/rear_right_steering_pos", 10)
        self._pub_fl_vel = self.create_publisher(Float64, "/rover/front_left_wheel_vel_cmd", 10)
        self._pub_fr_vel = self.create_publisher(Float64, "/rover/front_right_wheel_vel_cmd", 10)
        self._pub_rl_vel = self.create_publisher(Float64, "/rover/rear_left_wheel_vel_cmd", 10)
        self._pub_rr_vel = self.create_publisher(Float64, "/rover/rear_right_wheel_vel_cmd", 10)

        self._sub = self.create_subscription(
            Float64MultiArray, "/wheel_commands", self._on_wheel_commands, 20
        )

        self.get_logger().info(
            "Listening on /wheel_commands [δ_FL, δ_FR, δ_RL, δ_RR, ω_FL, ω_FR, ω_RL, ω_RR] "
            "and publishing to 8 per-wheel command topics."
        )

    def _on_wheel_commands(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 8:
            self.get_logger().warn(
                "Expected 8 values [δ_FL, δ_FR, δ_RL, δ_RR, ω_FL, ω_FR, ω_RL, ω_RR], "
                f"got {len(msg.data)}."
            )
            return

        delta_fl, delta_fr, delta_rl, delta_rr, omega_fl, omega_fr, omega_rl, omega_rr = msg.data

        fl_steer = Float64(); fl_steer.data = float(delta_fl)
        fr_steer = Float64(); fr_steer.data = float(delta_fr)
        rl_steer = Float64(); rl_steer.data = float(delta_rl)
        rr_steer = Float64(); rr_steer.data = float(delta_rr)
        fl_vel = Float64(); fl_vel.data = float(omega_fl)
        fr_vel = Float64(); fr_vel.data = float(omega_fr)
        rl_vel = Float64(); rl_vel.data = float(omega_rl)
        rr_vel = Float64(); rr_vel.data = float(omega_rr)

        self._pub_fl_steer.publish(fl_steer)
        self._pub_fr_steer.publish(fr_steer)
        self._pub_rl_steer.publish(rl_steer)
        self._pub_rr_steer.publish(rr_steer)
        self._pub_fl_vel.publish(fl_vel)
        self._pub_fr_vel.publish(fr_vel)
        self._pub_rl_vel.publish(rl_vel)
        self._pub_rr_vel.publish(rr_vel)


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







# ### this one handles 4 values only
# #!/usr/bin/python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64, Float64MultiArray


# class WheelsCmdDemuxNode(Node):
#     def __init__(self) -> None:
#         super().__init__("wheels_cmd_demux_node")

#         self._pub_fl = self.create_publisher(Float64, "/rover/front_left_wheel_vel_cmd", 10)
#         self._pub_fr = self.create_publisher(Float64, "/rover/front_right_wheel_vel_cmd", 10)
#         self._pub_rl = self.create_publisher(Float64, "/rover/rear_left_wheel_vel_cmd", 10)
#         self._pub_rr = self.create_publisher(Float64, "/rover/rear_right_wheel_vel_cmd", 10)
#         self._sub = self.create_subscription(
#             Float64MultiArray, "/rover/wheels_cmd", self._on_wheels_cmd, 20
#         )

#         self.get_logger().info(
#             "Listening on /rover/wheels_cmd [fl, fr, rl, rr] "
#             "and publishing to per-wheel command topics."
#         )

#     def _on_wheels_cmd(self, msg: Float64MultiArray) -> None:
#         if len(msg.data) != 4:
#             self.get_logger().warn(
#                 "Expected 4 values [fl, fr, rl, rr], "
#                 f"got {len(msg.data)}."
#             )
#             return

#         fl, fr, rl, rr = msg.data

#         out_fl = Float64()
#         out_fr = Float64()
#         out_rl = Float64()
#         out_rr = Float64()
#         out_fl.data = float(fl)
#         out_fr.data = float(fr)
#         out_rl.data = float(rl)
#         out_rr.data = float(rr)

#         self._pub_fl.publish(out_fl)
#         self._pub_fr.publish(out_fr)
#         self._pub_rl.publish(out_rl)
#         self._pub_rr.publish(out_rr)


# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = WheelsCmdDemuxNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()
