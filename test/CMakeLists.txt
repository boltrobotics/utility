include(test_project)

setup()
find_test_srcs()
build_exe(SRCS ${SOURCES} LIBS utility SUFFIX "-tests")

set(DOXYGEN_WARN NO)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
