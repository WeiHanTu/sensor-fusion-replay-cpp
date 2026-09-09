add_library(
  sfr_viz
  overlay.cpp
  panel.cpp
  projection_artifact.cpp
  sensitivity_artifact.cpp)

add_library(sfr::viz ALIAS sfr_viz)

target_include_directories(
  sfr_viz
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(
  sfr_viz
  PUBLIC sfr::geometry opencv_core
  PRIVATE sfr::build_info nlohmann_json::nlohmann_json opencv_imgcodecs opencv_imgproc)
sfr_configure_target(sfr_viz)
