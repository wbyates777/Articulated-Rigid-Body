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

project(BDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/lib)

# --- External Dependencies ---
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.3)
FetchContent_Declare(autodiff GIT_REPOSITORY https://github.com/autodiff/autodiff.git GIT_TAG v1.1.2)
FetchContent_Declare(ccd GIT_REPOSITORY https://github.com/danfis/libccd.git GIT_TAG v2.1)

# GLM
set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# autodiff
set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(CMAKE_CUDA_COMPILER OFF CACHE BOOL "" FORCE)

# ccd
set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

FetchContent_MakeAvailable(glm autodiff ccd)


# Create a 'virtual' target to hold includes
add_library(model_headers INTERFACE)

# Add BOTH directories to this interface
target_include_directories(model_headers INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/URDF
    ${CMAKE_CURRENT_SOURCE_DIR}/Model
)


# --- Internal Modules ---
# Order matters: SpatialAlgebra is the foundation
add_subdirectory(SpatialAlgebra)
add_subdirectory(Model)
add_subdirectory(URDF)
add_subdirectory(Collisions)
add_subdirectory(Checks)

# --- Main Executable ---
add_executable(dynamics_test CBody.cpp main.cpp)
target_link_libraries(dynamics_test PRIVATE URDF Model Collisions Checks) 




