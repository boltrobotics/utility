include(gtest)

##################################################################################################
# boost {

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND LIBRARIES boost_system)

# } boost

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${gtest_INC_DIR}
)

file(GLOB SOURCES
  "${PROJECT_SOURCE_DIR}/test/${BOARD_FAMILY}/*.c"
  "${PROJECT_SOURCE_DIR}/test/${BOARD_FAMILY}/*.cpp"
  "${PROJECT_SOURCE_DIR}/test/*.c"
  "${PROJECT_SOURCE_DIR}/test/*.cpp")

list(APPEND LIBRARIES utility)

add_compile_options(-Wall -Wextra -Werror)

add_executable(${PROJECT_NAME}_tests ${SOURCES})
target_link_libraries(${PROJECT_NAME}_tests ${gtest_LIB_NAME} ${LIBRARIES})
add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
