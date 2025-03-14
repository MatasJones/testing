# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_latency_test_talker_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED latency_test_talker_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(latency_test_talker_FOUND FALSE)
  elseif(NOT latency_test_talker_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(latency_test_talker_FOUND FALSE)
  endif()
  return()
endif()
set(_latency_test_talker_CONFIG_INCLUDED TRUE)

# output package information
if(NOT latency_test_talker_FIND_QUIETLY)
  message(STATUS "Found latency_test_talker: 0.0.0 (${latency_test_talker_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'latency_test_talker' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${latency_test_talker_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(latency_test_talker_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${latency_test_talker_DIR}/${_extra}")
endforeach()
