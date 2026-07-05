cmake_minimum_required(VERSION 3.16)
project(s_slam_replay)

find_package(ament_cmake REQUIRED)

# Replay-only package: it ships rosbag replay presets and helper launches.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  run.py
  scripts/publish_save_dir.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
