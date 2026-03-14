#
# Model 09/03/2026
#
# $$$$$$$$$$$$$$$$$
# $   Model.txt   $
# $$$$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#


add_library(Model 
    OBJECT
        BJoint.cpp
        BModel.cpp
        BDynamics.cpp
)

target_include_directories( Model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries( Model PUBLIC SpatialAlgebra )

