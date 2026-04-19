# Autonomous Controls

## State Estimation (EKF)

The rover uses an Extended Kalman Filter (EKF) via the `robot_localization` package to fuse sensor data into a single pose estimate.

**How it works:** The EKF maintains a state vector (position, orientation, velocity) and iteratively predicts the next state using a motion model, then corrects it with incoming sensor measurements. It handles non-linear systems by linearizing around the current estimate.

**Sensors fused:**
- `wheel/odometry` — contributes linear (vx, vy) and angular (vyaw) velocity
- `imu/data` — contributes roll/pitch orientation and linear acceleration (ax, ay)
- `aruco/pose` — contributes absolute x, y, z position and yaw

The EKF node is launched via `display.launch.py` and configured in `config/ekf.yaml`.
# Navigation and Controls - Current State

This repository currently contains two main modules:

- `Simulation`: ROS 2 Humble + Gazebo Garden rover simulation.
- `Global Planning`: Python-based global planner that exports world-frame path CSV.

## 1) Simulation State (`Simulation`)

The rover simulation is operational and now uses **4-wheel actuation by default**.

### What is implemented

- Gazebo world launch (`crater_bringup/launch/rover.launch.py`).
- ROS <-> Gazebo bridges for:
  - `/clock`
  - `/odom`
  - `/rover/scan`
  - TF/joint state topics
  - per-wheel command topics
- Rover drivetrain changed from diff-drive plugin to **independent wheel joint controllers**:
  - `front_left_wheel_joint`
  - `front_right_wheel_joint`
  - `rear_left_wheel_joint`
  - `rear_right_wheel_joint`
- Application nodes:
  - `path_from_csv_node.py`: publishes `nav_msgs/Path` from CSV.
  - `global_planner_node.py` (new): computes and publishes `nav_msgs/Path` directly from grid + waypoint config.
  - `path_follower_node.py`: follows path using odometry.
  - `twist_to_wheels_node.py`: converts `/rover/cmd_vel` to 4 wheel velocity commands.
  - `wheels_cmd_demux_node.py`: converts `/rover/wheels_cmd` (`[fl, fr, rl, rr]`) to per-wheel topics.

### Control interfaces

- High-level control:
  - `/rover/cmd_vel` (`geometry_msgs/msg/Twist`)
  - Internally converted to all 4 wheel commands.
- Direct 4-wheel control:
  - `/rover/wheels_cmd` (`std_msgs/msg/Float64MultiArray`)
  - Format: `[front_left, front_right, rear_left, rear_right]` in rad/s.

### Current note

- If rover does not move, check that Gazebo physics is running (not paused).

## 2) Global Planning State (`Global Planning`)

Global planning is currently a standalone Python workflow (not yet a ROS package).

### What is implemented

- Traversability/grid generation from terrain points.
- A*/TSP-based route construction.
- Path export to CSV:
  - `Global Planning/out/global_path_world.csv`
  - CSV format: header `x,y`

### Integration status with simulation

- Global planner output CSV is already consumable by simulation path pipeline.
- You can launch simulation with planner output directly as `path_csv`.
- You can also run the planner online in ROS (`crater_global_planner`) and publish `/rover/global_path` without CSV runtime handoff.
- This gives an end-to-end flow:
  - Global planner CSV -> ROS path publisher -> path follower -> 4 wheel commands -> Gazebo rover.

## 3) Run Commands

## Build (WSL)

```bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
cd /mnt/d/Coding/CRATER/Navigation_and_controls/Simulation
colcon build
source install/local_setup.bash
```

## Launch Simulation (4-wheel enabled)

```bash
ros2 launch crater_bringup rover.launch.py autonomy:=false
```

## Direct 4-Wheel Command Example

```bash
ros2 topic pub /rover/wheels_cmd std_msgs/msg/Float64MultiArray "{data: [4.0, 4.0, 4.0, 4.0]}"
```

## Spin in Place Example

```bash
ros2 topic pub /rover/wheels_cmd std_msgs/msg/Float64MultiArray "{data: [4.0, -4.0, 4.0, -4.0]}"
```

## Launch with Global Planner CSV

```bash
ros2 launch crater_bringup rover.launch.py autonomy:=true path_csv:='/mnt/d/Coding/CRATER/Navigation_and_controls/Global Planning/out/global_path_world.csv'
```

## Launch with Online Global Planner (ROS node)

```bash
ros2 launch crater_bringup rover.launch.py autonomy:=true use_online_planner:=true \
  global_planner_grid:='/mnt/d/Coding/CRATER/Navigation_and_controls/Global Planning/grid.npy' \
  global_planner_metadata:='/mnt/d/Coding/CRATER/Navigation_and_controls/Global Planning/metadata.npy'
```

## 4) Next Suggested Step

If desired, convert `Global Planning` into a ROS 2 package so path generation and simulation can run in one launch without manual CSV handoff.
# Crater Rover Prototype

This project contains a simulation of a 4-wheel skid-steer rover prototype for the Crater project.

## Rover Specs
*   **Dimensions**: 72cm (L) x 52cm (W) x 30cm (H)
*   **Configuration**: 4 Wheels (Skid-Steer) with Lidar sensor.
*   **Platform**: ROS 2 Humble & Gazebo Garden

## 🚀 How to Run (WSL / Ubuntu)

### 1. Setup
If you haven't installed ROS 2 yet, run the helper script:
```bash
sudo bash install_ros.sh
```

### 2. Build via Terminal
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
colcon build
```

### 3. Launch Simulation
```bash
# Source the workspace
source install/local_setup.bash

# Launch the rover
ros2 launch crater_bringup rover.launch.py
```

### 4. Autonomous Path Pipeline (new)
The launch now starts a minimal autonomy pipeline by default:
- `path_from_csv_node` publishes `nav_msgs/Path`
- `path_follower_node` subscribes to odometry + path and commands `/rover/cmd_vel`

Use the default built-in path:
```bash
ros2 launch crater_bringup rover.launch.py autonomy:=true
```

Use path generated by Global Planning:
```bash
ros2 launch crater_bringup rover.launch.py autonomy:=true path_csv:="/absolute/path/to/global_path_world.csv"
```

Use online global planner node (no CSV publisher):
```bash
ros2 launch crater_bringup rover.launch.py autonomy:=true use_online_planner:=true \
  global_planner_grid:="/absolute/path/to/grid.npy" \
  global_planner_metadata:="/absolute/path/to/metadata.npy"
```

Disable autonomy and use keyboard teleop as before:
```bash
ros2 launch crater_bringup rover.launch.py autonomy:=false
```

## 🛞 Independent 4-Wheel Control
The rover now supports direct per-wheel velocity commands in addition to `/rover/cmd_vel`.

Per-wheel topics (rad/s):
- `/rover/front_left_wheel_vel_cmd`
- `/rover/front_right_wheel_vel_cmd`
- `/rover/rear_left_wheel_vel_cmd`
- `/rover/rear_right_wheel_vel_cmd`

Convenience topic for all four at once (`[fl, fr, rl, rr]`):
```bash
ros2 topic pub /rover/wheels_cmd std_msgs/msg/Float64MultiArray "{data: [4.0, 4.0, 4.0, 4.0]}"
```

Spin in place example:
```bash
ros2 topic pub /rover/wheels_cmd std_msgs/msg/Float64MultiArray "{data: [4.0, -4.0, 4.0, -4.0]}"
```

## 🎮 How to Control
To drive the rover, open a **Start a new terminal** and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rover/cmd_vel
```
*   Use `i`, `j`, `,`, `l` to move.
*   Use `k` to stop.

## 🖥️ Understanding the Windows
When you launch the simulation, two windows will open:

### 1. **Gazebo** (The Gray/3D World)
*   **What it is**: The Physics Simulator.
*   **What to look for**: This is the "Real World". You see the rover, gravity, collisions, and the ground. If the rover hits a wall here, it crashes.

### 2. **RViz** (The Visualization Tool)
*   **What it is**: The Robot's "Internal Brain" View.
*   **What to look for**: This shows what the robot *sees* and *knows*.
    *   **Red Dots/Lines**: This is the Lidar data detecting objects.
    *   **TF Frames**: The coordinate axes moving with the robot.
    *   **Grid**: The robot's estimated position.
