add_library(
  sfr_geometry
  calibration_sensitivity.cpp
  frame_graph.cpp
  point_cloud_projection.cpp
  projection.cpp
  rigid_transform.cpp)

add_library(sfr::geometry ALIAS sfr_geometry)

target_include_directories(
  sfr_geometry
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(sfr_geometry PUBLIC Eigen3::Eigen)
sfr_configure_target(sfr_geometry)
