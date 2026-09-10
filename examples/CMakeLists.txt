add_executable(generate_synthetic_kitti generate_synthetic_kitti.cpp)
target_link_libraries(
  generate_synthetic_kitti
  PRIVATE opencv_core opencv_imgcodecs opencv_imgproc)
set_target_properties(
  generate_synthetic_kitti
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(generate_synthetic_kitti)
