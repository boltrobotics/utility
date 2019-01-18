file(GLOB SOURCES "*.cpp" "${BOARD_FAMILY}/*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
  include(gtest)

  ##################################################################################################
  # boost {

  find_package(Boost REQUIRED COMPONENTS system)
  include_directories(${Boost_INCLUDE_DIRS})

  # } boost

  include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${gtest_INC_DIR}
  )

  add_compile_options(-Wall -Wextra -Werror)

  add_executable(${PROJECT_NAME}_tests ${SOURCES})
  target_link_libraries(${PROJECT_NAME}_tests ${gtest_LIB_NAME} utility boost_system)
  add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
endif ()
