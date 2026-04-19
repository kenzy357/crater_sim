import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("crater_aruco"),
        "config",
        "rgbd.yaml",
    )

    # --- DepthAI driver launch ---
    depthai_share = get_package_share_directory("depthai_ros_driver")
    depthai_launch_path = os.path.join(depthai_share, "launch", "camera.launch.py")

    depthai_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depthai_launch_path),
        launch_arguments={
            "name": "oak",
            "params_file": params_file,
            "use_rviz": "False",
            "pointcloud.enable": "false",
            "rs_compat": "False",
        }.items(),
    )

    # --- Start ArUco node ---
    aruco_node = Node(
        package="crater_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[{
            "image_topic": "/oak/rgb/image_rect",
            "camera_info_topic": "/oak/rgb/camera_info",
            "depth_topic": "/oak/stereo/image_raw",
        }],
    )

    return LaunchDescription([
        depthai_camera,
        aruco_node,
    ])
