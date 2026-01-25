# Chronos Demo Application CMakeLists.txt
cmake_minimum_required(VERSION 3.18)

project(ChronosDemo LANGUAGES C CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)

# Find required packages
find_package(CUDA REQUIRED)
find_package(PkgConfig REQUIRED)

# Optional OpenCV
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_definitions(-DUSE_OPENCV)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
endif()

# NVIDIA Jetson specific paths
set(JETSON_MULTIMEDIA_API "/usr/src/jetson_multimedia_api")
set(TEGRA_INCLUDE "/usr/include/tegra")

# Check for Jetson platform
if(EXISTS ${JETSON_MULTIMEDIA_API})
    set(ON_JETSON TRUE)
    message(STATUS "Building on Jetson platform")
else()
    set(ON_JETSON FALSE)
    message(STATUS "Building on non-Jetson platform (some features disabled)")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CUDA_INCLUDE_DIRS}
)

if(ON_JETSON)
    include_directories(
        ${JETSON_MULTIMEDIA_API}/include
        ${TEGRA_INCLUDE}
    )
endif()

# Source files
set(CAPTURE_SOURCES
    src/chronos_capture.c
)

set(DEMO_SOURCES
    src/demo_app.cpp
)

# Chronos capture library
add_library(chronos_capture SHARED ${CAPTURE_SOURCES})
target_link_libraries(chronos_capture
    pthread
)

if(ON_JETSON)
    target_link_libraries(chronos_capture
        nvbuf_utils
        ${CUDA_LIBRARIES}
        ${CUDA_CUDA_LIBRARY}
        EGL
    )
endif()

# Demo application
add_executable(chronos_demo ${DEMO_SOURCES})
target_link_libraries(chronos_demo
    chronos_capture
    ${CUDA_LIBRARIES}
)

if(OpenCV_FOUND)
    target_link_libraries(chronos_demo ${OpenCV_LIBS})
endif()

# Test application for synchronization validation
add_executable(sync_test
    src/sync_test.cpp
)
target_link_libraries(sync_test
    chronos_capture
    ${CUDA_LIBRARIES}
)

# Installation
install(TARGETS chronos_capture chronos_demo
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

install(FILES src/chronos_capture.h
    DESTINATION include/chronos
)

# CPack for packaging
set(CPACK_PACKAGE_NAME "chronos-demo")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_VENDOR "Chronos Project")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Chronos Multi-Camera Sync Demo")
include(CPack)
