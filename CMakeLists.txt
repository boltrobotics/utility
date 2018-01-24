cmake_minimum_required(VERSION 3.5)
project(utility C CXX)

if (NOT OS)
  set(OS OS_X86)
  enable_testing()
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Werror -Os)
#set(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wextra -Werror")

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

if (OS MATCHES OS_X86)
  add_subdirectory(${PROJECT_SOURCE_DIR}/test)
endif()
