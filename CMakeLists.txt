cmake_minimum_required(VERSION 3.11)
project(utility)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
include(init)

set(BOARD_FAMILY $ENV{BOARD_FAMILY})

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
