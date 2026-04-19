import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


# Lanch the robot and the navigation stack



def generate_launch_description():
    # Messages are from: https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#launching-nav2
    diff_drive_loaded_message = (
        "Successfully loaded controller diff_drive_base_controller into state active"
    )
    bringup_ready_message = "Successfully loaded controller diff_drive_base_controller into state active"
    navigation_ready_message = "Creating bond timer"

    run_headless = LaunchConfiguration("run_headless")
    world_file = LaunchConfiguration("world_file")

    global_path_bridge = Node(
        package="sam_bot_nav2_gz",
        executable="global_path_bridge",
        name="global_path_bridge",
        output="screen",
    )



    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            ["params_file:=", LaunchConfiguration('params_file')]
        ],
        shell=False,
        output="screen",
    )

    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom'],
        output='screen'
    )



    relay_cmd_vel = Node(
        package="topic_tools",
        executable="relay",
        name="relay_cmd_vel",
        arguments=["/cmd_vel", "/rover/cmd_vel"],
        output="screen",
    )

    waiting_success = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [
                    LogInfo(msg="Ready for navigation!"),
                    global_path_bridge,
                ],

            ),
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=[FindPackageShare("sam_bot_nav2_gz"), "/config/nav2_params.yaml"],
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("sam_bot_nav2_gz"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            LogInfo(msg="Bringup ready. Starting navigation..."),
                static_map_odom_tf,
                relay_cmd_vel,
            TimerAction(
                    period=5.0,  # Shorter delay is fine without slam_toolbox
                    actions=[navigation],
            ),
            waiting_success,
        ]
    )
