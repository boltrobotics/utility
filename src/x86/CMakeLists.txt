set(CMAKE_MACOSX_RPATH 1)

include(x86_project)

setup_x86()
find_srcs()
build_lib(SRCS ${SOURCES})
