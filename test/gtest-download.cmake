cmake_minimum_required(VERSION 3.5)
project(gtest-download NONE)

include(ExternalProject)
ExternalProject_Add(gtest
  GIT_REPOSITORY    https://github.com/google/googletest.git
  GIT_TAG           master
  SOURCE_DIR        "${CMAKE_BINARY_DIR}/gtest-download/src"
  BINARY_DIR        "${CMAKE_BINARY_DIR}/gtest-download/build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)