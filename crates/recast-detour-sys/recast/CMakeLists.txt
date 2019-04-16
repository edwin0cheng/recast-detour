cmake_minimum_required(VERSION 3.0)

project(Recast)

# lib versions
SET(SOVERSION 1)
SET(VERSION 1.0.0)

set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "Build demo" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "Build tests" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
set(RECASTNAVIGATION_STATIC TRUE CACHE BOOL "Build static libraries" FORCE)

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()

add_subdirectory(recastnavigation)
add_subdirectory(recastc)
