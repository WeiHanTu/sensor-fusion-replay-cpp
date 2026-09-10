include(GoogleTest)

function(sfr_add_test test_name source_file library_target)
  add_executable(${test_name} ${source_file})
  target_link_libraries(${test_name} PRIVATE ${library_target} GTest::gtest_main)
  sfr_configure_target(${test_name})
  gtest_discover_tests(${test_name} TEST_PREFIX "${test_name}.")
endfunction()

sfr_add_test(geometry_rigid_transform_test unit/rigid_transform_test.cpp sfr::geometry)
sfr_add_test(geometry_frame_graph_test unit/frame_graph_test.cpp sfr::geometry)
sfr_add_test(geometry_projection_test unit/projection_test.cpp sfr::geometry)
sfr_add_test(geometry_point_cloud_projection_test unit/point_cloud_projection_test.cpp sfr::geometry)
sfr_add_test(geometry_calibration_sensitivity_test unit/calibration_sensitivity_test.cpp
             sfr::geometry)

sfr_add_test(io_calibration_test unit/calibration_test.cpp sfr::io)
sfr_add_test(io_timestamp_test unit/timestamp_test.cpp sfr::io)
sfr_add_test(io_kitti_test unit/kitti_io_test.cpp sfr::io)

sfr_add_test(viz_overlay_test unit/overlay_test.cpp sfr::viz)
sfr_add_test(viz_panel_test unit/panel_test.cpp sfr::viz)
sfr_add_test(viz_projection_artifact_test unit/projection_artifact_test.cpp sfr::viz)
target_link_libraries(viz_projection_artifact_test PRIVATE nlohmann_json::nlohmann_json)
sfr_add_test(viz_sensitivity_artifact_test unit/sensitivity_artifact_test.cpp sfr::viz)
target_link_libraries(
  viz_sensitivity_artifact_test
  PRIVATE nlohmann_json::nlohmann_json opencv_imgcodecs)

sfr_add_test(cli_project_kitti_test integration/project_kitti_cli_test.cpp sfr::io)
target_link_libraries(
  cli_project_kitti_test
  PRIVATE nlohmann_json::nlohmann_json opencv_imgcodecs)
target_compile_definitions(
  cli_project_kitti_test
  PRIVATE SFR_PROJECT_KITTI_EXE="$<TARGET_FILE:project_kitti>")
add_dependencies(cli_project_kitti_test project_kitti)

add_test(NAME cli_project_kitti_help COMMAND project_kitti --help)

sfr_add_test(cli_analyze_calibration_test integration/analyze_calibration_cli_test.cpp sfr::io)
target_link_libraries(
  cli_analyze_calibration_test
  PRIVATE nlohmann_json::nlohmann_json opencv_imgcodecs)
target_compile_definitions(
  cli_analyze_calibration_test
  PRIVATE SFR_ANALYZE_CALIBRATION_EXE="$<TARGET_FILE:analyze_calibration>")
add_dependencies(cli_analyze_calibration_test analyze_calibration)

add_test(NAME cli_analyze_calibration_help COMMAND analyze_calibration --help)

sfr_add_test(cli_benchmark_projection_test integration/benchmark_projection_cli_test.cpp
             sfr::geometry)
target_link_libraries(cli_benchmark_projection_test PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(
  cli_benchmark_projection_test
  PRIVATE SFR_BENCHMARK_PROJECTION_EXE="$<TARGET_FILE:benchmark_projection>")
add_dependencies(cli_benchmark_projection_test benchmark_projection)

add_test(NAME cli_benchmark_projection_help COMMAND benchmark_projection --help)
