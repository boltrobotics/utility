file(GLOB SOURCES "*.cpp")
add_library(utility SHARED ${SOURCES})
add_subdirectory(test)
