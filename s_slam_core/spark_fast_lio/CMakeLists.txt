cmake_minimum_required(VERSION 3.20)
project(spark_fast_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
set(CMAKE_CXX_FLAGS "-O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -fexceptions")

message(STATUS "Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
option(SPARK_FAST_LIO_ENABLE_MP "Enable OpenMP parallel matching in spark_fast_lio" OFF)
option(SPARK_FAST_LIO_DETERMINISTIC_IKD_TREE
  "Use synchronous ikd-tree rebuilds for deterministic replay results" ON)
if(SPARK_FAST_LIO_ENABLE_MP AND CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(processor_count)
  message(STATUS "Processor number:  ${processor_count}")
  if(processor_count GREATER 4)
    add_definitions(-DMP_ENABLED)
    add_definitions(-DMP_PROC_NUM=3)
    message(STATUS "core for MP: 3")
  elseif(processor_count GREATER 3)
    add_definitions(-DMP_ENABLED)
    add_definitions(-DMP_PROC_NUM=2)
    message(STATUS "core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(PCL_FIND_QUIETLY ON)
set(CLEAR_WARNING_SUPPRESSION OFF)
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CLEAR_WARNING_SUPPRESSION ON)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

if (CLEAR_WARNING_SUPPRESSION)
  unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
endif()

# third_party is shared inside s_slam_core. ikd-Tree / IKFoM_toolkit are
# vendored sources used only by this package.
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
include_directories(
  include
  ${THIRD_PARTY_DIR}/ikd-Tree
  ${THIRD_PARTY_DIR}/IKFoM_toolkit)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#### for ikd-Tree
add_library(ikd_tree ${THIRD_PARTY_DIR}/ikd-Tree/ikd_Tree.cpp)
# To resolve "recompile with -fPIC" issue
set_target_properties(ikd_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(SPARK_FAST_LIO_DETERMINISTIC_IKD_TREE)
  target_compile_definitions(ikd_tree PRIVATE IKD_TREE_DETERMINISTIC_REBUILD)
endif()

# Optional: Livox driver
find_package(livox_ros_driver2 QUIET)

# Create the library as a component
add_library(spark_lio_component SHARED
  src/adapters/kimera_ouster_adapter.cpp
  src/adapters/ouster_adapter.cpp
  src/adapters/robosense_fairy_adapter.cpp
  src/adapters/velodyne_adapter.cpp
  src/data_processors/imu_processor.cpp
  src/data_processors/lidar_processor.cpp
  src/spark_fast_lio.cpp
)

# Example: C++ standard
target_compile_features(spark_lio_component PUBLIC cxx_std_20)
target_compile_definitions(spark_lio_component PRIVATE EIGEN_DONT_PARALLELIZE)
if(SPARK_FAST_LIO_DETERMINISTIC_IKD_TREE)
  target_compile_definitions(spark_lio_component PRIVATE SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER)
endif()

# Link external libs. Replace `ikd_tree` with your actual library name
target_link_libraries(spark_lio_component
  ${PCL_LIBRARIES}
  ikd_tree
)

# If Livox found, link it here
if(livox_ros_driver2_FOUND)
  ament_target_dependencies(
    spark_lio_component
    livox_ros_driver2
  )
  target_compile_definitions(spark_lio_component PRIVATE
    -DLIVOX_ROS_DRIVER_FOUND=${livox_ros_driver2_FOUND}
  )
else()
  message(STATUS "Missing livox driver! AVIA Lidar will be disabled!")
endif()

# Pull in ament dependencies
ament_target_dependencies(
  spark_lio_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

rclcpp_components_register_node(
  spark_lio_component
  PLUGIN "spark_fast_lio::SPARKFastLIO2"
  EXECUTABLE spark_lio_mapping
)

# Install
install(
  TARGETS spark_lio_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Example for installing launch files, etc.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Generate package info
ament_package()
