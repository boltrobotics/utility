cmake_minimum_required(VERSION 3.5)
project(utility C CXX)

if (NOT OS)
  set(OS OS_X86)
  enable_testing()
endif()

set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")

    if (NOT CONFIGURED_ONCE)
        set(CMAKE_CXX_FLAGS "${warnings} -g" CACHE STRING "C++ flags" FORCE)
        set(CONFIGURED_ONCE TRUE CACHE INTERNAL "CMake is configured")
    endif()
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

if (OS MATCHES OS_X86)
  add_subdirectory(${PROJECT_SOURCE_DIR}/test)
endif()
