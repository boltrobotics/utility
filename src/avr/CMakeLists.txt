cmake_minimum_required(VERSION 3.7)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(avr_project)

setup_avr()
find_srcs()
build_lib(SRCS ${SOURCES})
