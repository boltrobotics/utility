cmake_minimum_required(VERSION 3.11)
project(utility C CXX)

if (NOT OS)
  set(OS OS_X86)
  enable_testing()
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

message(STATUS "${PROJECT_NAME} build type: ${CMAKE_BUILD_TYPE}")

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Werror -Os)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

if (OS MATCHES OS_X86)
  add_subdirectory(${PROJECT_SOURCE_DIR}/test)
endif()
