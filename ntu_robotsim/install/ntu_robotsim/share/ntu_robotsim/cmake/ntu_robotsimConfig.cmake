# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ntu_robotsim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ntu_robotsim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ntu_robotsim_FOUND FALSE)
  elseif(NOT ntu_robotsim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ntu_robotsim_FOUND FALSE)
  endif()
  return()
endif()
set(_ntu_robotsim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ntu_robotsim_FIND_QUIETLY)
  message(STATUS "Found ntu_robotsim: 0.0.1 (${ntu_robotsim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ntu_robotsim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ntu_robotsim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ntu_robotsim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ntu_robotsim_DIR}/${_extra}")
endforeach()
