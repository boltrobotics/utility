include(x86_project)

find_program (SOCAT NAMES socat NO_CACHE)

if (NOT SOCAT)
  message(FATAL_ERROR "socat not found. Use: sudo apt install socat")
else ()
  message(STATUS "socat found: ${SOCAT}")
endif ()

find_test_srcs()
build_exe(
  SRCS ${SOURCES}
  LIBS ${PROJECT_NAME} ${BTR_LIBS}
  SUFFIX "-tests"
  INC_DIRS ${utility_INC_DIR}
  TEST ON)

set(DOXYGEN_WARN NO)
set(DOXYGEN_PREPROCESSING YES)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
