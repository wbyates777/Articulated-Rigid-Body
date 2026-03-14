#
# SpatialAlgebra 09/03/2026
#
# $$$$$$$$$$$$$$$$$$$$$$$$$$
# $   SpatialAlgebra.txt   $
# $$$$$$$$$$$$$$$$$$$$$$$$$$
#
# by W.B. Yates
# Copyright (c) W.B. Yates. All rights reserved.
# History:
#




add_library(SpatialAlgebra INTERFACE)

target_include_directories(SpatialAlgebra INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# SpatialAlgebra depends on glm and autodiff
target_link_libraries(SpatialAlgebra INTERFACE glm::glm autodiff::autodiff)
target_compile_definitions(SpatialAlgebra INTERFACE AUTODIFF_EIGEN_AVAILABLE=0)
