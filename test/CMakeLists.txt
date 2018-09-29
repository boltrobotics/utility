include(gtest)

include_directories(
  ${gtest_INC_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

file(GLOB SOURCES "*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
  add_compile_options(-Wall -Wextra -Werror)
  add_executable(${PROJECT_NAME}_tests ${SOURCES})
  target_link_libraries(${PROJECT_NAME}_tests ${gtest_LIB_NAME} ${Boost_LIBRARIES} ${PROJECT_NAME})
  add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
endif ()
