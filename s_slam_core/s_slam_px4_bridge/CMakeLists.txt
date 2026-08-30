cmake_minimum_required(VERSION 3.20)
project(s_slam_px4_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(s_slam_interfaces REQUIRED)

add_library(odometry_converter src/odometry_converter.cpp)
target_include_directories(odometry_converter PUBLIC include)
target_link_libraries(odometry_converter Eigen3::Eigen)
ament_target_dependencies(odometry_converter px4_msgs s_slam_interfaces)

add_executable(px4_odometry_bridge src/px4_odometry_bridge.cpp)
target_link_libraries(px4_odometry_bridge odometry_converter)
ament_target_dependencies(px4_odometry_bridge px4_msgs rclcpp s_slam_interfaces)

install(TARGETS odometry_converter px4_odometry_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(odometry_converter_test test/odometry_converter_test.cpp)
  if(TARGET odometry_converter_test)
    target_link_libraries(odometry_converter_test odometry_converter)
    ament_target_dependencies(odometry_converter_test s_slam_interfaces)
  endif()
endif()

ament_package()
