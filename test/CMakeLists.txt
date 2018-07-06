include(gtest.cmake)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

file(GLOB SOURCES "*.cpp")

if (TARGET utility)
  set(UTILITY_LIB utility)
endif()

if (NOT UTILITY_LIB)
  set(UTILITY_LIB "")
endif()

add_executable(utility_tests ${SOURCES})

if(APPLE OR WIN32)
  target_link_libraries(utility_tests ${UTILITY_LIB} gtest boost_system-mt boost_thread-mt)
else()
  target_link_libraries(utility_tests ${UTILITY_LIB} gtest boost_system boost_thread)
endif()

add_test(NAME utility_tests COMMAND utility_tests)
