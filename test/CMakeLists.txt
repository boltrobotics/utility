include(project_setup)
include(gtest)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_compile_options(-Wall -Wextra -Werror)
file(GLOB SOURCES "*.cpp")
add_executable(utility_tests ${SOURCES})

if (APPLE OR WIN32)
  #target_link_libraries(utility_tests ${UTILITY_LIB} gtest boost_system-mt boost_thread-mt)
  target_link_libraries(utility_tests utility ${GTEST_LIBRARIES} ${Boost_LIBRARIES})
else ()
  #target_link_libraries(utility_tests ${UTILITY_LIB} gtest boost_system boost_thread)
  target_link_libraries(utility_tests utility ${GTEST_LIBRARIES} ${Boost_LIBRARIES})
endif ()

add_test(NAME utility_tests COMMAND utility_tests)
