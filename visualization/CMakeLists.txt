cmake_minimum_required(VERSION 3.16)
project(visualization)

find_package(ament_cmake REQUIRED)

# Visualization-only package: it ships RViz configs + launch files, no code.
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
