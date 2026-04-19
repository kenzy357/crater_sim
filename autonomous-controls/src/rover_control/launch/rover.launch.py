from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('rover_control'),
        'config', 'rover.yaml'
    )

    # Read joy device config from rover.yaml at launch time
    with open(config, 'r') as f:
        params = yaml.safe_load(f)
    gp = params['gamepad_node']['ros__parameters']

    # Use device_name if set, otherwise fall back to device_id
    joy_params = {
        'deadzone':        0.0,   # handled in gamepad_node
        'autorepeat_rate': 0.0,   # only publish on change
    }
    if 'joy_device' in gp:
        joy_params['device'] = gp['joy_device']
    elif 'joy_device_name' in gp:
        joy_params['device_name'] = gp['joy_device_name']
    else:
        joy_params['device_id'] = gp.get('joy_device_id', 0)

    return LaunchDescription([

        # ── joy_node: reads the physical gamepad ─────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params]
        ),

        # ── gamepad_node: /joy → /cmd_vel ─────────────────────────────────────
        Node(
            package='rover_control',
            executable='gamepad_node',
            name='gamepad_node',
            parameters=[config],
            output='screen'
        ),

        # ── kinematics_node: /cmd_vel → /wheel_commands ───────────────────────
        Node(
            package='rover_control',
            executable='kinematics_node',
            name='kinematics_node',
            parameters=[config],
            output='screen'
        ),

    ])
