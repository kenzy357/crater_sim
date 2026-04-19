#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

class GlobalPathBridge(Node):
    def __init__(self):
        super().__init__("global_path_bridge")
        
        self._action_client = ActionClient(self, FollowPath, "/follow_path")
        
        self._sub = self.create_subscription(
            Path,
            "/rover/global_path",
            self._path_callback,
            10
        )
        self._current_goal_handle = None

    def _path_callback(self, path: Path):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Controller action server not available")
            return

        # Cancel any in-progress goal before sending new one
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"   # must match your nav2 controller config
        # goal.speed_limit = 1.0              # 1.0 = no override (use configured max)
        # goal.is_absolute = False            # True = m/s, False = percentage of max

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._current_goal_handle = future.result()
        if not self._current_goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller server")

    def _feedback_callback(self, feedback):
        # feedback.feedback contains speed, distance_to_goal etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()