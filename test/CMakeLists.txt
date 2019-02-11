include(test_project)

find_test_srcs()
build_exe(SRCS ${SOURCES} SUFFIX "-tests")
