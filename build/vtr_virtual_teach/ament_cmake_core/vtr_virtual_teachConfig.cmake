# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vtr_virtual_teach_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vtr_virtual_teach_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vtr_virtual_teach_FOUND FALSE)
  elseif(NOT vtr_virtual_teach_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vtr_virtual_teach_FOUND FALSE)
  endif()
  return()
endif()
set(_vtr_virtual_teach_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vtr_virtual_teach_FIND_QUIETLY)
  message(STATUS "Found vtr_virtual_teach: 0.1.0 (${vtr_virtual_teach_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vtr_virtual_teach' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vtr_virtual_teach_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vtr_virtual_teach_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${vtr_virtual_teach_DIR}/${_extra}")
endforeach()
