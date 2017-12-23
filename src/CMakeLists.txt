file(GLOB SOURCES "*.cpp")
list(LENGTH SOURCES SOURCES_LEN)

if (SOURCES_LEN GREATER 0)
    include_directories(
        ${PROJECT_SOURCE_DIR}/include
    )
    add_library(utility STATIC ${SOURCES})
endif()
