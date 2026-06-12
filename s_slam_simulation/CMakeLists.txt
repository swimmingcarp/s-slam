cmake_minimum_required(VERSION 3.16)
project(s_slam_simulation)

find_package(ament_cmake REQUIRED)

# Simulation-only package: no compiled targets yet.
install(DIRECTORY launch config urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
