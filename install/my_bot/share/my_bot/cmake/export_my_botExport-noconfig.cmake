#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "my_bot::my_bot" for configuration ""
set_property(TARGET my_bot::my_bot APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(my_bot::my_bot PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmy_bot.so"
  IMPORTED_SONAME_NOCONFIG "libmy_bot.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS my_bot::my_bot )
list(APPEND _IMPORT_CHECK_FILES_FOR_my_bot::my_bot "${_IMPORT_PREFIX}/lib/libmy_bot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
