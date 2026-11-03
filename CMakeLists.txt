#
# Articulated Rigid Body (ARB) 09/03/2026
#
# $$$$$$$$$$$$$$$
# $   ARB.txt   $
# $$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#
# see https://cmake.org/cmake/help/latest/guide/tutorial/index.html
#

cmake_minimum_required(VERSION 3.23...4.0)

project(BDynamics LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/lib)

# --- External Dependencies (Managed via FetchContent) ---
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.3)
FetchContent_Declare(autodiff GIT_REPOSITORY https://github.com/autodiff/autodiff.git GIT_TAG v1.1.2)

# GLM Configuration
set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# Autodiff Configuration
set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(CMAKE_CUDA_COMPILER OFF CACHE BOOL "" FORCE)

# ==============================================================================
# WORKAROUND: Generates a fake Eigen3Config.cmake inside the build directory.
# Required by autodiff's find_package(Eigen3) call.
# ==============================================================================
file(WRITE "${CMAKE_BINARY_DIR}/Eigen3Config.cmake" "
    set(Eigen3_FOUND TRUE)
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
    endif()
")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}")
# ==============================================================================

FetchContent_MakeAvailable(glm autodiff)

# ==============================================================================
# EMBEDDED DEPENDENCY: libccd
# Compiles locally as a C99 static library to bypass old CMake policies & CI blocks.
# ==============================================================================
file(GLOB CCD_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/CCD/src/*.c"
)

add_library(ccd STATIC ${CCD_SOURCES})

target_include_directories(ccd PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/CCD/src"
)

# Enable double-precision for physics simulations (matches libccd's dDOUBLE flag)
target_compile_definitions(ccd PUBLIC dDOUBLE)

set_target_properties(ccd PROPERTIES 
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
# ==============================================================================

# Create a 'virtual' target to hold includes
add_library(model_headers INTERFACE)

# Add BOTH directories to this interface
target_include_directories(model_headers INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/URDF
    ${CMAKE_CURRENT_SOURCE_DIR}/Model
)

# --- Internal Modules ---
add_subdirectory(SpatialAlgebra)
add_subdirectory(Model)
add_subdirectory(URDF)
add_subdirectory(Collisions)
add_subdirectory(Checks)

# --- Main Executable ---
add_executable(dynamics_test CBody.cpp main.cpp)
target_link_libraries(dynamics_test PRIVATE URDF Model Collisions Checks)
