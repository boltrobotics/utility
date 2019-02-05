file(GLOB_RECURSE SOURCES
  "${UTILITY_SOURCE_DIR}/src/${BOARD_FAMILY}/*.c"
  "${UTILITY_SOURCE_DIR}/src/${BOARD_FAMILY}/*.cpp"
  "${UTILITY_SOURCE_DIR}/src/common/*.c"
  "${UTILITY_SOURCE_DIR}/src/common/*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
  include_directories(${UTILITY_SOURCE_DIR}/include)
  add_library(utility STATIC ${SOURCES})
else ()
  message(STATUS "${PROJECT_NAME} has no sources to build")
endif()
