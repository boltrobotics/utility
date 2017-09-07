file(GLOB SOURCES "*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
    add_library(utility STATIC ${SOURCES})
endif()
