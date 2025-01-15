#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "FSD_tester" for configuration "Release"
set_property(TARGET FSD_tester APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(FSD_tester PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/FSD_tester"
  )

list(APPEND _cmake_import_check_targets FSD_tester )
list(APPEND _cmake_import_check_files_for_FSD_tester "${_IMPORT_PREFIX}/bin/FSD_tester" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
