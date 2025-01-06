cmake_minimum_required(VERSION 3.22)
project(utility)

include(esp32_project)

find_srcs()
build_lib(
  ESP_TARGET ${ESP_TARGET}
  SRCS ${SOURCES}
  LIBS idf::freertos idf::spi_flash idf::esp_timer idf::esp_driver_i2c
  COMPONENTS freertos esptool_py esp_driver_i2c
  )
