cmake_minimum_required(VERSION 3.11)
project(utility C CXX)

set(BOARD_FAMILY $ENV{BOARD_FAMILY})

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Werror)

################################################################################
# Unit tests {

option(ENABLE_TESTS "Build unit tests" OFF)

if (ENABLE_TESTS)
  enable_testing()
  add_subdirectory(${PROJECT_SOURCE_DIR}/test)
endif()

# }

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
