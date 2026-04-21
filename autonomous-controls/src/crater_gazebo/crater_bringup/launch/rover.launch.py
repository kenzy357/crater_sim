# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as pkg


def generate_launch_description():
    # Configure ROS nodes for launch

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Setup project paths
    pkg_project_bringup = pkg('crater_bringup')
    pkg_project_gazebo = pkg('crater_simulator')
    pkg_project_description = pkg('crater_description')
    pkg_project_application = pkg('crater_application')
    pkg_project_global_planner = pkg('crater_global_planner')
    pkg_ros_gz_sim = pkg('ros_gz_sim')
    pkg_state_estimation = pkg('state_estimation')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'rover', 'terby.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': [
        PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            LaunchConfiguration('world')
        ])
    ]}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "terby_rover",
            "-string", robot_desc,
            "-x", "0",
            "-y", "0",
            "-z", "2",
        ],
        output="screen",
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'rover.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'crater_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    path_publisher = Node(
        package='crater_application',
        executable='path_from_csv_node.py',
        name='path_from_csv_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'csv_path': LaunchConfiguration('path_csv'),
            'frame_id': 'rover/odom',
            'topic_name': '/rover/global_path',
            'publish_hz': 1.0,
        }],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('autonomy'),
                "' == 'true' and '",
                LaunchConfiguration('use_online_planner'),
                "' != 'true'"
            ])
        ),
        output='screen'
    )

    global_planner = Node(
        package='crater_global_planner',
        executable='global_planner_node.py',
        name='global_planner_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'grid_path': LaunchConfiguration('global_planner_grid'),
            'metadata_path': LaunchConfiguration('global_planner_metadata'),
            'config_json_path': LaunchConfiguration('global_planner_config'),
            'frame_id': 'rover/odom',
            'topic_name': '/rover/global_path',
            'publish_hz': 1.0,
        }],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('autonomy'),
                "' == 'true' and '",
                LaunchConfiguration('use_online_planner'),
                "' == 'true'"
            ])
        ),
        output='screen'
    )

    path_follower = Node(
        package='crater_application',
        executable='path_follower_node.py',
        name='path_follower_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_topic': '/rover/global_path',
            'odom_topic': '/rover/odometry',
            'cmd_vel_topic': '/rover/cmd_vel',
            'lookahead_m': 1.5,
            'goal_tolerance_m': 0.35,
            'max_linear_speed': 0.35,
            'max_angular_speed': 0.8,
            'k_ang': 1.6,
            'control_hz': 20.0,
            'align_path_to_start_pose': True,
            'enforce_goal_stop': False,
        }],
        condition=IfCondition(LaunchConfiguration('autonomy')),
        output='screen'
    )

    twist_to_wheels = Node(
        package='crater_application',
        executable='twist_to_wheels_node.py',
        name='twist_to_wheels_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'half_wheelbase': 0.2,
            'half_track': 0.33,
            'wheel_radius': 0.15,
            'cmd_vel_topic': '/rover/cmd_vel',
        }],
        output='screen'
    )

    wheels_cmd_demux = Node(
        package='crater_application',
        executable='wheels_cmd_demux_node.py',
        name='wheels_cmd_demux_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(pkg_state_estimation, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock (false for real hardware).'),
        DeclareLaunchArgument('world', default_value='marsyard.sdf',
                              description='World SDF file to load.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
        DeclareLaunchArgument('autonomy', default_value='true',
                              description='Run autonomous path pipeline.'),
        DeclareLaunchArgument(
            'use_online_planner',
            default_value='false',
            description='Use crater_global_planner instead of CSV path publisher.'
        ),
        DeclareLaunchArgument(
            'path_csv',
            default_value=os.path.join(
                pkg_project_application, 'config', 'global_path_world.csv'
            ),
            description='CSV path file with columns x,y.'
        ),
        DeclareLaunchArgument(
            'global_planner_config',
            default_value=os.path.join(
                pkg_project_global_planner, 'config', 'planner_config.json'
            ),
            description='Global planner JSON config with base/waypoints.'
        ),
        DeclareLaunchArgument(
            'global_planner_grid',
            default_value='',
            description='Path to traversability grid .npy file.'
        ),
        DeclareLaunchArgument(
            'global_planner_metadata',
            default_value='',
            description='Path to metadata .npy file [min_x, min_y, resolution].'
        ),
        gz_sim,
        #spawn_robot,
        bridge,
        robot_state_publisher,
        ekf_node,
        rviz,
        # path_publisher,
        # global_planner,
        # path_follower,
        twist_to_wheels,
        wheels_cmd_demux,
    ])
