add_executable(project_kitti project_kitti.cpp)
target_link_libraries(project_kitti PRIVATE sfr::io sfr::viz)
set_target_properties(project_kitti PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(project_kitti)

add_executable(analyze_calibration analyze_calibration.cpp)
target_link_libraries(analyze_calibration PRIVATE sfr::io sfr::viz)
set_target_properties(analyze_calibration PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(analyze_calibration)

add_executable(estimate_extrinsics estimate_extrinsics.cpp)
target_link_libraries(estimate_extrinsics PRIVATE sfr::io sfr::viz)
set_target_properties(estimate_extrinsics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(estimate_extrinsics)
