set(BOARD_FAMILY x86)

file(GLOB SOURCES "*.c*" "${PROJECT_SOURCE_DIR}/src/*.c*")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
  include_directories( ${PROJECT_SOURCE_DIR}/include)
  add_library(utility STATIC ${SOURCES})
else ()
  message(STATUS "${PROJECT_NAME} has no sources to build")
endif()
