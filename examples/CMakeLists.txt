add_executable(generate_synthetic_kitti generate_synthetic_kitti.cpp)
target_link_libraries(
  generate_synthetic_kitti
  PRIVATE opencv_core opencv_imgcodecs opencv_imgproc)
set_target_properties(
  generate_synthetic_kitti
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(generate_synthetic_kitti)

add_executable(generate_extrinsic_fixture generate_extrinsic_fixture.cpp)
target_include_directories(generate_extrinsic_fixture PRIVATE "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(generate_extrinsic_fixture PRIVATE sfr::io)
set_target_properties(generate_extrinsic_fixture PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(generate_extrinsic_fixture)

add_executable(run_extrinsic_campaign run_extrinsic_campaign.cpp)
target_include_directories(run_extrinsic_campaign PRIVATE "${PROJECT_SOURCE_DIR}/apps")
target_link_libraries(run_extrinsic_campaign PRIVATE sfr::io sfr::viz nlohmann_json::nlohmann_json)
set_target_properties(run_extrinsic_campaign PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(run_extrinsic_campaign)
