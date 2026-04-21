#
# URDF 19/04/2026
#
# $$$$$$$$$$$$$$$$
# $   URDF.txt   $
# $$$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#
  

add_library(URDF 
    OBJECT
        tinyxml2.cpp
        URDFTypes.cpp
        URDFReader.cpp
        URDFWriter.cpp
        URDFManager.cpp
)

target_include_directories( URDF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries( URDF PUBLIC SpatialAlgebra PUBLIC model_headers)

