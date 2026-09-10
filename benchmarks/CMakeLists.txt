add_executable(benchmark_projection benchmark_projection.cpp)
target_link_libraries(
  benchmark_projection
  PRIVATE
    sfr::geometry
    sfr::build_info
    nlohmann_json::nlohmann_json)
set_target_properties(benchmark_projection PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
sfr_configure_target(benchmark_projection)

if(SFR_BUILD_TESTS)
  add_test(
    NAME benchmark_projection_smoke
    COMMAND benchmark_projection --smoke)
endif()
