include(test_project)
setup_tests()

find_test_srcs()
build_tests(SRCS ${SOURCES} SUFFIX "-tests")

set(DOXYGEN_WARN NO)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
