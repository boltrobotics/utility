cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project("Logger")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(${INCLUDE_DIR})
add_subdirectory(${SOURCE_DIR})
enable_testing()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")

    if (NOT CONFIGURED_ONCE)
        set(CMAKE_CXX_FLAGS "${warnings} -g -std=c++14" CACHE STRING "C++ flags" FORCE)
        set(CONFIGURED_ONCE TRUE CACHE INTERNAL "CMake is configured at least once")
    endif()
endif()
