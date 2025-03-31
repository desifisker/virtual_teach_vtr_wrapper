#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vtr_virtual_teach::vtr_virtual_teach" for configuration ""
set_property(TARGET vtr_virtual_teach::vtr_virtual_teach APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(vtr_virtual_teach::vtr_virtual_teach PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libvtr_virtual_teach.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtr_virtual_teach::vtr_virtual_teach )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtr_virtual_teach::vtr_virtual_teach "${_IMPORT_PREFIX}/lib/libvtr_virtual_teach.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
