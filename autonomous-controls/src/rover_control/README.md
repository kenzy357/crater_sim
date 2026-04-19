# rover_control — ROS2 Humble

ROS2 C++ package for the CRATER rover 4-wheel steering (4WS) drive system.  
Reads a gamepad and computes per-wheel steering angles and spin speeds.

---

## Node graph

```
[Gamepad] → joy_node → /joy → gamepad_node → /cmd_vel → kinematics_node → /wheel_commands → [Motors]
```

| Node              | Input                    | Output                        |
|-------------------|--------------------------|-------------------------------|
| `joy_node`        | Physical gamepad         | `/joy` (sensor_msgs/Joy)      |
| `gamepad_node`    | `/joy`                   | `/cmd_vel` (geometry_msgs/Twist) |
| `kinematics_node` | `/cmd_vel`               | `/wheel_commands` (std_msgs/Float64MultiArray) |

---

## /wheel_commands layout

8 values in this order (FL=front-left, FR=front-right, RL=rear-left, RR=rear-right):

```
index 0 = delta_FL   steering angle [rad]
index 1 = delta_FR   steering angle [rad]
index 2 = delta_RL   steering angle [rad]
index 3 = delta_RR   steering angle [rad]
index 4 = omega_FL   spin speed [rad/s]  (negative = reverse)
index 5 = omega_FR   spin speed [rad/s]
index 6 = omega_RL   spin speed [rad/s]
index 7 = omega_RR   spin speed [rad/s]
```

---

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- joy package: `sudo apt install ros-humble-joy`

### Build
```bash
# 1. Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src

# 2. Create package structure
mkdir -p ~/ros2_ws/src/rover_control/src
mkdir -p ~/ros2_ws/src/rover_control/config
mkdir -p ~/ros2_ws/src/rover_control/launch

# 3. Copy files
cp CMakeLists.txt   ~/ros2_ws/src/rover_control/
cp package.xml      ~/ros2_ws/src/rover_control/
cp gamepad_node.cpp    ~/ros2_ws/src/rover_control/src/
cp kinematics_node.cpp ~/ros2_ws/src/rover_control/src/
cp rover.yaml          ~/ros2_ws/src/rover_control/config/
cp rover.launch.py     ~/ros2_ws/src/rover_control/launch/

# (My virtual machine version)
cp /home/kenzy/CRATER/InverseMotion_ROS/CMakeLists.txt  ~/ros2_ws/src/rover_control/
cp /home/kenzy/CRATER/InverseMotion_ROS/package.xml     ~/ros2_ws/src/rover_control/

cp /home/kenzy/CRATER/InverseMotion_ROS/gamepad_node.cpp    ~/ros2_ws/src/rover_control/src/
cp /home/kenzy/CRATER/InverseMotion_ROS/kinematics_node.cpp ~/ros2_ws/src/rover_control/src/

cp /home/kenzy/CRATER/InverseMotion_ROS/rover.yaml      ~/ros2_ws/src/rover_control/config/
cp /home/kenzy/CRATER/InverseMotion_ROS/rover.launch.py   ~/ros2_ws/src/rover_control/launch/

# 4. Build
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select rover_control
source install/setup.bash
```

---

## Configuration — rover.yaml

**All parameters are in `config/rover.yaml`. No recompile needed when you change them.**

### Step 1 — Find your gamepad name

With xbox controller and VM, start the machine then add the controller in the USB settings.

```bash
cat /proc/bus/input/devices | grep "N: Name"
```

Example output:
```
N: Name="Power Button"
N: Name="AT Translated Set 2 keyboard"
N: Name="Sony Interactive Entertainment DualSense Wireless Controller"
```

> ⚠️ **Humble note:** `joy_node` in Humble does **not** support `device_name`. Use the device path or device ID instead.

Find the path:
```bash
cat /proc/bus/input/devices
```
This showed `"Microsoft X-Box 360 pad"` with `Handlers=event7 js2`

The `js2` in the handlers means the joystick index is **2**, so the full path is `/dev/input/js2`.  
Set this in `rover.yaml`:
```yaml
joy_device: '/dev/input/js2'
```

For the real rover where the gamepad is the only joystick, use device ID instead:
```yaml
# joy_device: '/dev/input/js2'   ← comment this out
joy_device_id: 0                  ← use this instead
```

### Step 2 — Find your axis indices

Run joy_node and echo the /joy topic:
```bash
# Terminal 1 — start joy_node with device path (required in Humble)
ros2 run joy joy_node --ros-args -p device:=/dev/input/js2

# Terminal 2
ros2 topic echo /joy
```

Move each stick one at a time and note which `axes[X]` index changes.  
Update `axis_lx`, `axis_ly`, `axis_rx` in `rover.yaml` accordingly.

**DualSense PS5 mapping (for reference):**
```
axes[0] = left  stick X  → axis_lx (strafe)
axes[1] = left  stick Y  → axis_ly (forward/back)
axes[2] = left  trigger
axes[3] = right stick X  → axis_rx (rotation)
axes[4] = right stick Y
axes[5] = right trigger
```

### Step 3 — Set rover geometry

Measure your physical rover and update:
```yaml
L: 1.5    # wheelbase    (front-to-back distance)  [m]
W: 1.0    # track width  (left-to-right distance)  [m]
R: 0.15   # wheel radius                           [m]
```

### Step 4 — Set velocity limits

```yaml
vmax: 2.0    # max linear  velocity [m/s]  — set safe limit before running on hardware
wmax: 0.5    # max angular velocity [rad/s]
```

---

## Run

### All nodes at once
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rover_control rover.launch.py
```

### Individually (for debugging)
```bash
# Terminal 1 — gamepad driver (use device path in Humble)
ros2 run joy joy_node --ros-args -p device:=/dev/input/js2

# Terminal 2 — gamepad → cmd_vel
ros2 run rover_control gamepad_node --ros-args \
  --params-file ~/ros2_ws/install/rover_control/share/rover_control/config/rover.yaml

# Terminal 3 — cmd_vel → wheel_commands
ros2 run rover_control kinematics_node --ros-args \
  --params-file ~/ros2_ws/install/rover_control/share/rover_control/config/rover.yaml
```

---

## Debugging

```bash
# Check cmd_vel while moving sticks
ros2 topic echo /cmd_vel

# Check wheel commands output
ros2 topic echo /wheel_commands

# Check publish rates
ros2 topic hz /cmd_vel
ros2 topic hz /wheel_commands

# List all active nodes
ros2 node list

# List all active topics
ros2 topic list
```

### Test without a gamepad
You can publish a fake cmd_vel directly to test the kinematics node:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 run rover_control kinematics_node --ros-args \
  --params-file ~/ros2_ws/install/rover_control/share/rover_control/config/rover.yaml

# Terminal 2 — display wheel commands
ros2 topic echo /wheel_commands

# Terminal 3 — send a test twist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Updating parameters

Edit `rover.yaml` and copy it to both locations:
```bash
cp rover.yaml ~/ros2_ws/src/rover_control/config/
cp rover.yaml ~/ros2_ws/install/rover_control/share/rover_control/config/
```

No rebuild needed — just relaunch.

---

## File structure

```
rover_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── rover.yaml              ← all tunable parameters
├── launch/
│   └── rover.launch.py         ← starts all 3 nodes
└── src/
    ├── gamepad_node.cpp         ← /joy → /cmd_vel
    └── kinematics_node.cpp      ← /cmd_vel → /wheel_commands
```
