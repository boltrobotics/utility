include(test_project)

add_definitions(-DBTR_FLOAT_ENABLED=1)
find_test_srcs()
build_exe(SRCS ${SOURCES} LIBS utility SUFFIX "-tests")
