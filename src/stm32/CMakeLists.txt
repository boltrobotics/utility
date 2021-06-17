cmake_minimum_required(VERSION 3.16)
project(utility)

include(stm32_project)
find_srcs()

build_lib(SRCS ${SOURCES})
