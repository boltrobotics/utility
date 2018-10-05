# Currently, unit testing is done on x86 platform. If a parent project uses cross-compiler that
# doesn't support x86 libraries, the build will fail. Threfore, temporarily change BOARD_FAMILY
# to x86.

set(BOARD_FAMILY "x86")

include(gtest)

include_directories(
  ${gtest_INC_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

file(GLOB SOURCES "*.cpp" "${BOARD_FAMILY}/*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
  add_compile_options(-Wall -Wextra -Werror)
  add_executable(${PROJECT_NAME}_tests ${SOURCES})
  target_link_libraries(${PROJECT_NAME}_tests ${gtest_LIB_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME})
  add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
endif ()
