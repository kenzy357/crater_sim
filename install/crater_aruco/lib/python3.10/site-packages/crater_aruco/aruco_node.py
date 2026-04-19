import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray

import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.cv_bridge = CvBridge()

        self.tag_size_m = 0.144
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.K = None
        self.D = None

        self.TAG_XY = {
            61: (1.0, 4.0),
            62: (3.0, 3.0)
        }

        # === Subscriptions ===

        self.rgb_sub = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.rgb_callback, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/oak/rgb/camera_info", self.info_callback, qos_profile_sensor_data
        )


        # === Publisher ===

        self.ids_pub = self.create_publisher(Int32MultiArray, "/aruco/ids", 10)

    # === Helpers ===

    def wrap_pi(self, a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def solve_pose_two_tags(self, P1, p1, P2, p2):
        dP = P2 - P1
        dp = p2 - p1
        if float(dp @ dp) < 1e-6:
            return None

        phi_world = math.atan2(float(dP[1]), float(dP[0]))
        phi_cam   = math.atan2(float(dp[1]), float(dp[0]))
        theta = self.wrap_pi(phi_world - phi_cam)

        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s],
                    [s,  c]])

        t1 = P1 - R @ p1
        t2 = P2 - R @ p2
        t  = 0.5 * (t1 + t2)
        return float(t[0]), float(t[1]), float(theta)

    # === Callbacks ===

    def info_callback(self, msg: CameraInfo):
        # Camera matrix K
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # Distortion coefficients
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

        self.get_logger().info(
            f"CameraInfo received. K=\n{self.K}\nD={self.D.flatten().tolist()}"
        )
        # Only need it once
        self.destroy_subscription(self.info_sub)


    def rgb_callback(self, msg: Image):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params
        )

        out = Int32MultiArray()
        if ids is None or len(ids) == 0:
            out.data = []
            self.ids_pub.publish(out)
            return

        ids_list = [int(x) for x in ids.flatten().tolist()]
        out.data = ids_list
        self.ids_pub.publish(out)

        if self.K is None or self.D is None:
            return

        s = float(self.tag_size_m)
        obj_pts = np.array(
            [
                [-s / 2,  s / 2, 0.0],
                [ s / 2,  s / 2, 0.0],
                [ s / 2, -s / 2, 0.0],
                [-s / 2, -s / 2, 0.0],
            ],
            dtype=np.float64,
        )

        correspondences = []  # list of (id, p_i (2,), P_i (2,))

        for i, marker_id in enumerate(ids_list):
            img_pts = corners[i].reshape(4, 2).astype(np.float64)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue

            # theta=0 : x_opt||world x, z_opt||world y
            x_opt, y_opt, z_opt = tvec.flatten().tolist()
            self.get_logger().info(f"id={marker_id:3d}  t=[{x_opt:+.3f}, {y_opt:+.3f}, {z_opt:+.3f}] m")

            if marker_id not in self.TAG_XY:
                continue

            p_i = np.array([x_opt, z_opt], dtype=float)
            P_i = np.array(self.TAG_XY[marker_id], dtype=float)
            correspondences.append((marker_id, p_i, P_i))

        # only handles exactly 2 tag detections
        if len(correspondences) != 2:
            return

        (_, p1, P1), (_, p2, P2) = correspondences
        sol = self.solve_pose_two_tags(P1, p1, P2, p2)
        if sol is None:
            return

        x_w, y_w, theta = sol
        self.get_logger().info(f"POSE  x={x_w:+.2f}  y={y_w:+.2f}  theta={theta + math.pi/2:+.2f} rad")

  




def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

