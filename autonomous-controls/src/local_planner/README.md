# Local Planner

## Setup
1. Install Nav2
2. Run the main simulation: 
```
cd simulation
colcon build && source install/local_setup.bash
ros2 launch crater_bringup rover.launch.py
```
3. Run the Nav2 stack
```
cd local_planner
colcon build && source install/local_setup.bash
ros2 launch crater_local_planner complete_navigation.launch.py
```
4. Send an example goal
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Current Shortcomings
- execution of /rover/cmd_vel commands by the rover appears lacking, linear velocity works, but angular velocity isn't really considered. E.g., publishing
    ```
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{
    linear: {x: 0.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.8}
    }"
    ```
    will result in the rover starting to turn, but stopping after a quarter turn. Because of that, navigation points that aren't in positive x direction don't work. However, the /rover/cmd_vel signal looks correct.
- does not use costmaps in any form yet
