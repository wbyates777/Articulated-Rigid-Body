cmake_minimum_required(VERSION 3.20)

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

# Populate (download + configure)
FetchContent_MakeAvailable(glm)

# --- Executable ---
add_executable(dynamics_test ${SOURCES} ${HEADERS})

# Include GLM headers
target_include_directories(dynamics_test PRIVATE ${glm_SOURCE_DIR})

