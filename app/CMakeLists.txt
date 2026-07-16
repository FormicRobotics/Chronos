# Chronos Demo Application CMakeLists.txt
cmake_minimum_required(VERSION 3.18)

project(ChronosDemo LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optional CUDA (device-copy path in the library + GPU processing in the demo)
option(CHRONOS_WITH_CUDA "Enable the optional CUDA device-copy path" ON)

if(CHRONOS_WITH_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 17)
        find_package(CUDAToolkit REQUIRED)
        message(STATUS "CUDA enabled: ${CMAKE_CUDA_COMPILER}")
    else()
        set(CHRONOS_WITH_CUDA OFF)
        message(STATUS "CUDA compiler not found - CUDA path disabled")
    endif()
endif()

# Optional OpenCV
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_definitions(-DUSE_OPENCV)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Source files
set(CAPTURE_SOURCES
    src/chronos_capture.c
)

set(DEMO_SOURCES
    src/demo_app.cpp
)

# Chronos capture library (V4L2 MMAP capture; CUDA copy path optional)
add_library(chronos_capture SHARED ${CAPTURE_SOURCES})
target_link_libraries(chronos_capture
    pthread
)

if(CHRONOS_WITH_CUDA)
    target_compile_definitions(chronos_capture PUBLIC CHRONOS_WITH_CUDA)
    target_link_libraries(chronos_capture CUDA::cudart)
endif()

# Demo application (requires CUDA for its processing kernels)
if(CHRONOS_WITH_CUDA)
    # demo_app.cpp contains __global__ kernels - compile it as CUDA
    set_source_files_properties(${DEMO_SOURCES} PROPERTIES LANGUAGE CUDA)

    add_executable(chronos_demo ${DEMO_SOURCES})
    target_link_libraries(chronos_demo
        chronos_capture
        CUDA::cudart
    )

    if(OpenCV_FOUND)
        target_link_libraries(chronos_demo ${OpenCV_LIBS})
    endif()
else()
    message(STATUS "chronos_demo skipped (needs CUDA)")
endif()

# Test application for synchronization validation (no CUDA needed)
add_executable(sync_test
    src/sync_test.cpp
)
target_link_libraries(sync_test
    chronos_capture
)

# Installation
set(CHRONOS_INSTALL_TARGETS chronos_capture sync_test)
if(CHRONOS_WITH_CUDA)
    list(APPEND CHRONOS_INSTALL_TARGETS chronos_demo)
endif()

install(TARGETS ${CHRONOS_INSTALL_TARGETS}
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
