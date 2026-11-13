#
# Collisions 09/03/2026
#
# $$$$$$$$$$$$$$$$$$$$$$
# $   Collisions.txt   $
# $$$$$$$$$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#

# --- Embedded libccd Dependency ---
file(GLOB CCD_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/CCD/*.c"
)

add_library(ccd STATIC ${CCD_SOURCES})

target_include_directories(ccd PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/CCD/"
)

# Enable double-precision for physics simulations (matches libccd's dDOUBLE flag)
target_compile_definitions(ccd PUBLIC dDOUBLE)

set_target_properties(ccd PROPERTIES 
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
# ---------------------------------

add_library(Collisions 
    OBJECT 
        BCollider.cpp        
        BGJK.cpp        
)


# Crucial: Allow other targets to see these headers
target_include_directories(Collisions PUBLIC 
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../Solver
)

# Link internal math AND external collision ccd
target_link_libraries(Collisions 
    PUBLIC 
        SpatialAlgebra 
        ccd            # This target was created in the root
)
