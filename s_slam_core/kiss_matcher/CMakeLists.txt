cmake_minimum_required(VERSION 3.22)
project(kiss_matcher_ros)

# Set a default build type if none is specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0074 NEW)

# ---- Options (consumed by third_party/find_dependencies.cmake) ----
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(USE_SYSTEM_ROBIN "Use system pre-installed ROBIN from SPARK @ MIT" OFF)
option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)
if (USE_ADDRESS_SANITIZER)
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

# =============================================================================
# KISS-Matcher core library dependencies (Eigen3 / TBB / ROBIN / FLANN / LZ4)
# =============================================================================
include(GNUInstallDirs)
include(FetchContent)
# Several fetched projects use broad cache option names. Pin them before any
# FetchContent dependency is loaded so third-party tests/examples stay out of
# normal workspace builds.
set(BUILD_TESTS OFF CACHE BOOL "Disable fetched dependency tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable fetched dependency examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable fetched dependency benchmarks" FORCE)
# 3rdparty dependency scripts are shared inside s_slam_core/third_party.
include(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/find_dependencies.cmake)

find_library(LZ4_LIBRARY lz4 REQUIRED)

# FLANN is consumed directly via `#include <flann/flann.hpp>` in our public
# headers. Prefer flann's upstream CMake config; fall back to manual discovery
# for distros that ship libflann-dev without a CMake config (e.g. Ubuntu 22.04).
find_package(flann CONFIG QUIET)
if(NOT TARGET flann::flann_cpp)
  find_path(FLANN_INCLUDE_DIR NAMES flann/flann.hpp)
  find_library(FLANN_CPP_LIBRARY NAMES flann_cpp)
  if(NOT FLANN_INCLUDE_DIR OR NOT FLANN_CPP_LIBRARY)
    message(FATAL_ERROR "FLANN not found. Install libflann-dev or provide a flann CMake config.")
  endif()
  add_library(flann::flann_cpp UNKNOWN IMPORTED)
  set_target_properties(flann::flann_cpp PROPERTIES
    IMPORTED_LOCATION "${FLANN_CPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}")
  mark_as_advanced(FLANN_INCLUDE_DIR FLANN_CPP_LIBRARY)
endif()

include(FindOpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

# =============================================================================
# ROS 2 / ament dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(GTSAM CONFIG QUIET)
if(NOT GTSAM_FOUND)
  message(STATUS "GTSAM not found. Fetching pinned GTSAM 4.2.0 for this build.")
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "Use system Eigen for vendored GTSAM" FORCE)
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "Disable GTSAM tests" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "Disable GTSAM examples" FORCE)
  set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Disable CPU-specific GTSAM flags" FORCE)
  set(GTSAM_INSTALL_CPPUNITLITE OFF CACHE BOOL "Disable GTSAM CppUnitLite install" FORCE)
  FetchContent_Declare(
    gtsam
    GIT_REPOSITORY https://github.com/borglab/gtsam.git
    # GTSAM 4.2.0, pinned to the resolved tag commit for reproducible builds.
    GIT_TAG 4f66a491ffc83cf092d0d818b11dc35135521612
  )
  FetchContent_MakeAvailable(gtsam)
endif()

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Load small_gicp
# small_gicp uses broad cache variable names. Pin them before FetchContent so
# a workspace-level cache cannot make third-party tests part of this package.
set(BUILD_WITH_PCL ON CACHE BOOL "Build small_gicp PCL integration" FORCE)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp
  # v1.0.1, pinned to the resolved commit for reproducible builds.
  GIT_TAG 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
)
FetchContent_MakeAvailable(small_gicp)

# =============================================================================
# KISS-Matcher core library (built in-tree from src/ + include/kiss_matcher)
# =============================================================================
# NOTE(hlim): kept STATIC (see upstream Issue #37).
add_library(kiss_matcher_core STATIC
    src/ROBINMatching.cpp
    src/FasterPFH.cpp
    src/KISSMatcher.cpp
    src/GncSolver.cpp
)
add_library(kiss_matcher::kiss_matcher_core ALIAS kiss_matcher_core)
target_compile_features(kiss_matcher_core PUBLIC cxx_std_17)
target_include_directories(kiss_matcher_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/tsl/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(kiss_matcher_core
    PUBLIC Eigen3::Eigen robin::robin flann::flann_cpp TBB::tbb ${LZ4_LIBRARY}
)

# =============================================================================
# Executables (ROS 2 nodes)
# =============================================================================
add_executable(run_kiss_matcher src/run_kiss_matcher.cpp)
target_include_directories(run_kiss_matcher PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(run_kiss_matcher
    kiss_matcher::kiss_matcher_core
    ${PCL_LIBRARIES}
)

add_executable(registration_visualizer src/registration_visualizer.cpp)
ament_target_dependencies(registration_visualizer
  rclcpp tf2_ros sensor_msgs pcl_conversions PCL
)
target_link_libraries(registration_visualizer
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
)

add_executable(kiss_matcher_sam src/slam/pose_graph_manager.cpp
                                src/slam/loop_closure.cpp
                                src/slam/loop_detector.cpp)
ament_target_dependencies(kiss_matcher_sam
  rclcpp tf2_ros sensor_msgs nav_msgs visualization_msgs message_filters pcl_conversions PCL
)
target_link_libraries(kiss_matcher_sam
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

add_executable(inter_frame_alignment src/inter_frame_alignment.cpp
                                     src/slam/loop_closure.cpp)
ament_target_dependencies(inter_frame_alignment
  rclcpp tf2_ros sensor_msgs nav_msgs visualization_msgs pcl_conversions PCL
)
target_link_libraries(inter_frame_alignment
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

install(TARGETS
  registration_visualizer
  run_kiss_matcher
  kiss_matcher_sam
  inter_frame_alignment
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(kiss_matcher_sparse_solution_test
    test/kiss_matcher_sparse_solution_test.cpp)
  target_link_libraries(kiss_matcher_sparse_solution_test
    kiss_matcher::kiss_matcher_core)

  ament_add_gtest(loop_closure_input_validation_test
    test/loop_closure_input_validation_test.cpp
    src/slam/loop_closure.cpp)
  ament_target_dependencies(loop_closure_input_validation_test
    rclcpp tf2_ros sensor_msgs nav_msgs visualization_msgs message_filters pcl_conversions PCL)
  target_link_libraries(loop_closure_input_validation_test
    kiss_matcher::kiss_matcher_core
    ${PCL_LIBRARIES}
    small_gicp
    gtsam)

  ament_add_gtest(pose_graph_input_validation_test
    test/pose_graph_input_validation_test.cpp)
  ament_target_dependencies(pose_graph_input_validation_test
    rclcpp tf2_ros sensor_msgs nav_msgs visualization_msgs message_filters pcl_conversions PCL)
  target_link_libraries(pose_graph_input_validation_test
    ${PCL_LIBRARIES}
    gtsam)
endif()

ament_package()
