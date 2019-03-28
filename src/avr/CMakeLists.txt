cmake_minimum_required(VERSION 3.7)
set(CMAKE_MODULE_PATH $ENV{CMAKEHELPERS_HOME}/cmake/Modules)

include(avr_project)

if (NOT AVR_UPLOADTOOL_PORT)
  set(AVR_UPLOADTOOL_PORT "/dev/tty.usbmodemFD14511")
endif ()

setup_avr()

find_srcs()
build_lib(SRCS ${SOURCES})
