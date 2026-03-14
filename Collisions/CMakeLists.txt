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



add_library(Collisions 
    OBJECT 
        BCollider.cpp        
        BContactManager.cpp    
        BGJK.cpp        
)


# Crucial: Allow other targets to see these headers
target_include_directories(Collisions PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link internal math AND external collision ccd
target_link_libraries(Collisions 
    PUBLIC 
        SpatialAlgebra 
    PUBLIC
        ccd            # This target was created in the root
)
