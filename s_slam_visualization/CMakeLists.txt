cmake_minimum_required(VERSION 3.16)
project(s_slam_visualization)

find_package(ament_cmake REQUIRED)

# Visualization-only package: it ships RViz configs + launch files, no code.
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
