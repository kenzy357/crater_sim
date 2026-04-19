cmake_minimum_required(VERSION 3.5)

project(crater_global_planner)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/planner_core.py
    scripts/global_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
