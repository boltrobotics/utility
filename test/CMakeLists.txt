include(x86_project)
find_test_srcs()
build_exe(SRCS ${SOURCES} SUFFIX "-tests" TEST ON)

set(DOXYGEN_WARN NO)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
