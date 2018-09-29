cmake_minimum_required(VERSION 3.5)
project(utility)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
include(init)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

include(unit_testing)
