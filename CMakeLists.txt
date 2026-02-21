cmake_minimum_required(VERSION 3.25)

project(BDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Optional: Add warnings
#add_compile_options(-Wall -Wextra)


# --- Sources ---
set(SOURCES
    BJoint.cpp
    BModel.cpp
    BDynamics.cpp
    main.cpp
)

set(HEADERS
    BAutodiff.h
    BAdjoint.h
    BInertia.h
    BJoint.h
    BModel.h
    BDynamics.h
    BABInertia.h		
    BFixedBody.h		
    BSpatialAlgebra.h	
    BSpatialTypes.h
    BBody.h			
    BFunctions.h	
    BProducts.h    
    BMatrix6.h	
    BVector6.h
    BMatrix63.h		
    BRBInertia.h		
    BTransform.h
)

# --- External dependencies into lib/ ---
include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/lib)

# GLM (header-only)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.3
)

# AUTODIFF (header-only)
FetchContent_Declare(
    autodiff
    GIT_REPOSITORY https://github.com/autodiff/autodiff.git
    GIT_TAG        v1.1.2
)

# Populate (download + configure)

set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)

set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(CMAKE_CUDA_COMPILER OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(autodiff)



add_executable(dynamics_test ${SOURCES} ${HEADERS})

target_compile_definitions(dynamics_test PRIVATE AUTODIFF_EIGEN_AVAILABLE=0)

# Include headers
target_include_directories(dynamics_test PRIVATE 
    ${glm_SOURCE_DIR}
    ${autodiff_SOURCE_DIR}
)

