#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "flann::flann_cpp" for configuration "Release"
set_property(TARGET flann::flann_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(flann::flann_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libflann_cpp.so.1.9.2"
  IMPORTED_SONAME_RELEASE "libflann_cpp.so.1.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS flann::flann_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_flann::flann_cpp "${_IMPORT_PREFIX}/lib/libflann_cpp.so.1.9.2" )

# Import target "flann::flann" for configuration "Release"
set_property(TARGET FLANN::FLANN APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(FLANN::FLANN PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libflann.so.1.9.2"
  IMPORTED_SONAME_RELEASE "libflann.so.1.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS FLANN::FLANN )
list(APPEND _IMPORT_CHECK_FILES_FOR_flann::flann "${_IMPORT_PREFIX}/lib/libflann.so.1.9.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
