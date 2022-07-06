#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cloudparse::cloudparse" for configuration "Release"
set_property(TARGET cloudparse::cloudparse APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cloudparse::cloudparse PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcloudparse.so.1.0.3"
  IMPORTED_SONAME_RELEASE "libcloudparse.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS cloudparse::cloudparse )
list(APPEND _IMPORT_CHECK_FILES_FOR_cloudparse::cloudparse "${_IMPORT_PREFIX}/lib/libcloudparse.so.1.0.3" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
