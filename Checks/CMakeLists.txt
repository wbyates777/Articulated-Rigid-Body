#
# Checks 09/03/2026
#
# $$$$$$$$$$$$$$$$$$
# $   Checks.txt   $
# $$$$$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#



add_library(Checks OBJECT
    BSpatialChecks.cpp
)

target_include_directories(Checks PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(Checks PUBLIC SpatialAlgebra)


