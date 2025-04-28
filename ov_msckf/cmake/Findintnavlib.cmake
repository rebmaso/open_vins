# FindIntnavlib.cmake
# This module locates the intnavlib library.
# It defines the following variables:
#  INTNAVLIB_FOUND - True if the library was found.
#  INTNAVLIB_INCLUDE_DIRS - Include directories for the library.
#  INTNAVLIB_LIBRARIES - Libraries to link against.

find_path(INTNAVLIB_INCLUDE_DIR
  NAMES intnavlib.h
  PATHS
  /usr/local/include/intnavlib
  /usr/include/intnavlib
)

find_library(INTNAVLIB_LIBRARY
  NAMES intnavlib
  PATHS
  /usr/local/lib/intnavlib
  /usr/lib/intnavlib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(intnavlib
  REQUIRED_VARS INTNAVLIB_LIBRARY INTNAVLIB_INCLUDE_DIR
  FAIL_MESSAGE "Could not find intnavlib"
)

if (INTNAVLIB_FOUND)
  set(INTNAVLIB_LIBRARIES ${INTNAVLIB_LIBRARY})
  set(INTNAVLIB_INCLUDE_DIRS ${INTNAVLIB_INCLUDE_DIR})
endif()
