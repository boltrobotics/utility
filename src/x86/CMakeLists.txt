set(CMAKE_MACOSX_RPATH 1)

include(x86_project)

find_srcs()
build_lib(SRCS ${SOURCES})
