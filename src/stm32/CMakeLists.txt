cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(stm32f103c8t6)
include(stm32_project)

find_srcs()
build_lib(SRCS ${SOURCES})
