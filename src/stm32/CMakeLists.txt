cmake_minimum_required(VERSION 3.16)
project(utility)

include(stm32_project)

include(libopencm3)
include_directories(${libopencm3_INC_DIR})
include(freertos)
include_directories(${FREERTOS_INC_DIRS})

add_compile_definitions(BTR_I2C0_ENABLED=1)

find_srcs()
build_lib(SRCS ${SOURCES})
