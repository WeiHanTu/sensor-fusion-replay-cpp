function(sfr_configure_target target_name)
  target_compile_features(${target_name} PUBLIC cxx_std_20)
  set_target_properties(${target_name} PROPERTIES CXX_EXTENSIONS OFF)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    if(SFR_WARNINGS_AS_ERRORS)
      target_compile_options(${target_name} PRIVATE -Werror)
    endif()
  elseif(MSVC)
    target_compile_options(${target_name} PRIVATE /W4)
    if(SFR_WARNINGS_AS_ERRORS)
      target_compile_options(${target_name} PRIVATE /WX)
    endif()
  endif()

  if(SFR_ENABLE_ASAN_UBSAN)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
      message(FATAL_ERROR "ASan/UBSan preset requires a Clang or GCC-compatible compiler")
    endif()
    target_compile_options(
      ${target_name}
      PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(
      ${target_name}
      PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  endif()
endfunction()
