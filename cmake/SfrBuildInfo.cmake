function(sfr_add_build_info_target)
  execute_process(
    COMMAND git rev-parse --verify HEAD
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    RESULT_VARIABLE git_commit_result
    OUTPUT_VARIABLE SFR_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NOT git_commit_result EQUAL 0)
    set(SFR_GIT_COMMIT "uncommitted")
  endif()

  execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    RESULT_VARIABLE git_status_result
    OUTPUT_VARIABLE git_status_output
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NOT git_status_result EQUAL 0 OR NOT git_status_output STREQUAL "")
    set(SFR_GIT_DIRTY true)
  else()
    set(SFR_GIT_DIRTY false)
  endif()

  set(SFR_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
  set(SFR_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
  set(SFR_HOST_OS "${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION}")
  set(SFR_HOST_ARCH "${CMAKE_HOST_SYSTEM_PROCESSOR}")
  set(SFR_HOST_CPU "")
  if(APPLE AND EXISTS "/usr/sbin/sysctl")
    execute_process(
      COMMAND /usr/sbin/sysctl -n machdep.cpu.brand_string
      RESULT_VARIABLE cpu_result
      OUTPUT_VARIABLE SFR_HOST_CPU
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(NOT cpu_result EQUAL 0)
      set(SFR_HOST_CPU "")
    endif()
  elseif(EXISTS "/proc/cpuinfo")
    file(STRINGS "/proc/cpuinfo" cpu_line REGEX "^model name[ \t]*:" LIMIT_COUNT 1)
    if(cpu_line)
      string(REGEX REPLACE "^model name[ \t]*:[ \t]*" "" SFR_HOST_CPU "${cpu_line}")
    endif()
  endif()
  if(SFR_HOST_CPU STREQUAL "")
    set(SFR_HOST_CPU "${SFR_HOST_ARCH}")
  endif()

  set(generated_include_directory "${CMAKE_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${generated_include_directory}/sfr/core")
  configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/build_info.hpp.in"
    "${generated_include_directory}/sfr/core/build_info.hpp"
    @ONLY)

  add_library(sfr_build_info INTERFACE)
  add_library(sfr::build_info ALIAS sfr_build_info)
  target_include_directories(
    sfr_build_info
    INTERFACE "$<BUILD_INTERFACE:${generated_include_directory}>")
endfunction()
