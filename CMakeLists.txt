cmake_minimum_required(VERSION 3.24)

project(
  sensor_fusion_replay
  VERSION 0.1.0
  DESCRIPTION "C++20 offline camera and LiDAR replay pipeline"
  LANGUAGES CXX)

include(FetchContent)

option(SFR_BUILD_TESTS "Build public hermetic tests" ON)
option(SFR_ENABLE_ASAN_UBSAN "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SFR_WARNINGS_AS_ERRORS "Treat project compiler warnings as errors" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(SfrProjectOptions)
include(SfrBuildInfo)
sfr_add_build_info_target()

find_package(Eigen3 3.4 CONFIG QUIET)
if(NOT TARGET Eigen3::Eigen)
  message(STATUS "Eigen3 was not found; fetching pinned Eigen 3.4.0")
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR _sfr_header_only
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(eigen)
  add_library(sfr_eigen INTERFACE)
  target_include_directories(sfr_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
  add_library(Eigen3::Eigen ALIAS sfr_eigen)
endif()

set(SFR_OPENCV_HINTS)
if(APPLE)
  list(APPEND SFR_OPENCV_HINTS /opt/homebrew/opt/opencv/lib/cmake/opencv5)
endif()
find_package(
  OpenCV 5 CONFIG QUIET
  COMPONENTS core imgcodecs imgproc
  HINTS ${SFR_OPENCV_HINTS})
if(NOT OpenCV_FOUND)
  find_package(OpenCV 4.5 CONFIG REQUIRED COMPONENTS core imgcodecs imgproc)
endif()

find_package(nlohmann_json 3.11 CONFIG QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
  message(STATUS "nlohmann_json was not found; fetching pinned v3.11.3")
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

if(SFR_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(src/geometry)
add_subdirectory(src/io)
add_subdirectory(src/viz)
add_subdirectory(apps)
add_subdirectory(benchmarks)
add_subdirectory(examples)

if(SFR_BUILD_TESTS)
  find_package(GTest 1.15 CONFIG QUIET)
  if(NOT TARGET GTest::gtest_main)
    message(STATUS "GoogleTest was not found; fetching pinned v1.15.2")
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.15.2
      GIT_SHALLOW TRUE
      EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_subdirectory(tests)
endif()

include(SfrDeveloperTools)
sfr_add_developer_tools(
  FILES
  include/sfr/core/point_types.hpp
  include/sfr/geometry/calibration_sensitivity.hpp
  include/sfr/geometry/frame_graph.hpp
  include/sfr/geometry/point_cloud_projection.hpp
  include/sfr/geometry/projection.hpp
  include/sfr/geometry/rigid_transform.hpp
  include/sfr/io/io_error.hpp
  include/sfr/io/kitti_calibration.hpp
  include/sfr/io/kitti_io.hpp
  include/sfr/io/timestamp.hpp
  include/sfr/viz/overlay.hpp
  include/sfr/viz/panel.hpp
  include/sfr/viz/projection_artifact.hpp
  include/sfr/viz/sensitivity_artifact.hpp
  apps/analyze_calibration.cpp
  apps/project_kitti.cpp
  benchmarks/benchmark_projection.cpp
  examples/generate_synthetic_kitti.cpp
  src/geometry/calibration_sensitivity.cpp
  src/geometry/frame_graph.cpp
  src/geometry/point_cloud_projection.cpp
  src/geometry/projection.cpp
  src/geometry/rigid_transform.cpp
  src/io/io_error.cpp
  src/io/kitti_calibration.cpp
  src/io/kitti_io.cpp
  src/io/timestamp.cpp
  src/viz/overlay.cpp
  src/viz/panel.cpp
  src/viz/projection_artifact.cpp
  src/viz/sensitivity_artifact.cpp
  tests/integration/analyze_calibration_cli_test.cpp
  tests/integration/benchmark_projection_cli_test.cpp
  tests/integration/generate_synthetic_kitti_cli_test.cpp
  tests/integration/project_kitti_cli_test.cpp
  tests/unit/calibration_sensitivity_test.cpp
  tests/unit/calibration_test.cpp
  tests/unit/frame_graph_test.cpp
  tests/unit/kitti_io_test.cpp
  tests/unit/overlay_test.cpp
  tests/unit/panel_test.cpp
  tests/unit/point_cloud_projection_test.cpp
  tests/unit/projection_test.cpp
  tests/unit/projection_artifact_test.cpp
  tests/unit/rigid_transform_test.cpp
  tests/unit/sensitivity_artifact_test.cpp
  tests/unit/timestamp_test.cpp)
