cmake_minimum_required(VERSION 3.5)

project(crater_application)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/path_from_csv_node.py
    scripts/path_follower_node.py
    scripts/twist_to_wheels_node.py
    scripts/wheels_cmd_demux_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
