add_library(
  sfr_io
  io_error.cpp
  kitti_calibration.cpp
  kitti_io.cpp
  timestamp.cpp)

add_library(sfr::io ALIAS sfr_io)

target_include_directories(
  sfr_io
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(
  sfr_io
  PUBLIC sfr::geometry opencv_core
  PRIVATE opencv_imgcodecs)
sfr_configure_target(sfr_io)
