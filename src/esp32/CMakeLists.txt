cmake_minimum_required(VERSION 3.22)
project(utility)

include(esp32_project)
find_srcs()

build_lib(
  SRCS ${SOURCES}
  ESP_TARGET ${ESP_TARGET}
  LIBS idf::freertos idf::spi_flash
  COMPONENTS freertos esptool_py
  )
