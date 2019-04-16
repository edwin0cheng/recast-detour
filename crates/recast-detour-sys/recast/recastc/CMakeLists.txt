cmake_minimum_required(VERSION 3.0)

project(RecastC)

# lib versions
SET(SOVERSION 1)
SET(VERSION 1.0.0)

file(GLOB CPP_FILES Source/*.cpp)

set(RecastC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

add_library(RecastC STATIC ${CPP_FILES})

target_include_directories(RecastC PRIVATE ${RecastC_INCLUDE_DIR})
target_link_libraries(RecastC Detour)

install(TARGETS RecastC
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)