# Currently, unit testing is done on x86 platform. If a parent project uses cross-compiler that
# doesn't support x86 libraries (gtest, stl, etc.), the build will fail.

if (NOT BOARD_FAMILY)
  set(BOARD_FAMILY "x86")
endif()

string(COMPARE EQUAL "${BOARD_FAMILY}" x86 _cmp)

if (_cmp)
  file(GLOB SOURCES "*.cpp" "${BOARD_FAMILY}/*.cpp")
  list(LENGTH SOURCES SOURCES_LEN)

  if (SOURCES_LEN GREATER 0)
    include(gtest)

    include_directories(
      ${gtest_INC_DIR}
      ${PROJECT_SOURCE_DIR}/include
    )

    if (TARGET ${PROJECT_NAME})
      set(OWN_LIBRARY ${PROJECT_NAME})
    endif ()

    add_compile_options(-Wall -Wextra -Werror)
    add_executable(${PROJECT_NAME}_tests ${SOURCES})
    target_link_libraries(${PROJECT_NAME}_tests ${gtest_LIB_NAME} ${Boost_LIBRARIES} ${OWN_LIBRARY})
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
  endif ()
else ()
  message(STATUS "${PROJECT_NAME} doesn't support unit testing for ${BOARD_FAMILY}")
endif ()
