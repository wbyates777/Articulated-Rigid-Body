#
# Solver 09/03/2026
#

add_library(Solver 
    OBJECT 
        BContactManager.cpp    
        BManifold.cpp    
#        BPolytopeLoader.cpp        
)

# Allow other targets to see Solver headers
target_include_directories(Solver PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../Collisions
)

# Solver depends on Collisions and SpatialAlgebra
target_link_libraries(Solver 
    PUBLIC 
        SpatialAlgebra
        Collisions
)
