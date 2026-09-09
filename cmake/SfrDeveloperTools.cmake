function(sfr_add_developer_tools)
  set(options)
  set(one_value_args)
  set(multi_value_args FILES)
  cmake_parse_arguments(SFR_TOOLS "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  set(source_files)
  foreach(source_file IN LISTS SFR_TOOLS_FILES)
    list(APPEND source_files "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}")
  endforeach()

  find_program(
    SFR_CLANG_FORMAT
    NAMES clang-format
    HINTS /opt/homebrew/opt/llvm/bin)
  if(SFR_CLANG_FORMAT)
    add_custom_target(
      format
      COMMAND "${SFR_CLANG_FORMAT}" -i ${source_files}
      COMMENT "Formatting project C++ sources"
      VERBATIM)
    add_custom_target(
      format-check
      COMMAND "${SFR_CLANG_FORMAT}" --dry-run --Werror ${source_files}
      COMMENT "Checking C++ formatting"
      VERBATIM)
  else()
    add_custom_target(
      format
      COMMAND "${CMAKE_COMMAND}" -E echo "clang-format was not found"
      COMMAND "${CMAKE_COMMAND}" -E false
      VERBATIM)
    add_custom_target(
      format-check
      COMMAND "${CMAKE_COMMAND}" -E echo "clang-format was not found"
      COMMAND "${CMAKE_COMMAND}" -E false
      VERBATIM)
  endif()

  find_program(
    SFR_CLANG_TIDY
    NAMES clang-tidy
    HINTS /opt/homebrew/opt/llvm/bin)
  if(SFR_CLANG_TIDY)
    set(clang_tidy_platform_args)
    if(APPLE)
      execute_process(
        COMMAND xcrun --show-sdk-path
        OUTPUT_VARIABLE macos_sdk_path
        OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY)
      list(APPEND clang_tidy_platform_args
           "--extra-arg=-isysroot"
           "--extra-arg=${macos_sdk_path}")
    endif()
    set(translation_units)
    foreach(source_file IN LISTS source_files)
      if(source_file MATCHES "\\.(cc|cpp|cxx)$")
        list(APPEND translation_units "${source_file}")
      endif()
    endforeach()
    add_custom_target(
      clang-tidy
      COMMAND "${SFR_CLANG_TIDY}" --quiet -p "${CMAKE_BINARY_DIR}" ${clang_tidy_platform_args}
              ${translation_units}
      COMMENT "Running clang-tidy over project translation units"
      VERBATIM)
  else()
    add_custom_target(
      clang-tidy
      COMMAND "${CMAKE_COMMAND}" -E echo "clang-tidy was not found"
      COMMAND "${CMAKE_COMMAND}" -E false
      VERBATIM)
  endif()
endfunction()
