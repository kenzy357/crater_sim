cmake_minimum_required(VERSION 3.8)
project(rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -Wall -Wextra -O2)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs    REQUIRED)

# ── gamepad_node ──────────────────────────────────────────────────────────────
# Subscribes to /joy (sensor_msgs/Joy)
# Publishes  to /cmd_vel (geometry_msgs/Twist)
add_executable(gamepad_node
  src/gamepad_node.cpp
)
ament_target_dependencies(gamepad_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ── kinematics_node ───────────────────────────────────────────────────────────
# Subscribes to /cmd_vel      (geometry_msgs/Twist)
# Publishes  to /wheel_commands (std_msgs/Float64MultiArray)
#   Layout: [delta_FL, delta_FR, delta_RL, delta_RR,
#             omega_FL, omega_FR, omega_RL, omega_RR]
add_executable(kinematics_node
  src/kinematics_node.cpp
)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  std_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  gamepad_node
  kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
