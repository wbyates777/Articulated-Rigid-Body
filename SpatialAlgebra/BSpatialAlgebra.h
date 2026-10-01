/* Spatial Algebra 15/08/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialAlgebra.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 A compact, light-weight, header only impmemenation of spatial algebra as presented in:

 "Rigid Body Dynamics Algorithms" (RBDA), R. Featherstone, Springer, 2008 (see https://royfeatherstone.org). 

 See also
 
 Modern Robotics: Mechanics, Planning, and Control, Lynch K. M., Park F. C., 2017.
 
 Depends on the 3D GLM library (see https://github.com/g-truc/glm).

 ARB ALGEBRA CONVENTIONS

 Underlying 3D matrix library: GLM

 Conventions:
 - Matrices are stored/accessed in GLM column-major form.
 - Vectors are treated as column vectors.
 - Matrix-vector multiplication A * x represents standard linear action on a column vector.
 - Printed mathematical formulas in comments follow standard textbook block-matrix notation.
 - Therefore, explicit arb::transpose() calls may be required in code to match textbook formulas.

 IMPORTANT:
 Do not validate matrix expressions by visual inspection alone.
 Always validate using:
     1) explicit block-matrix equivalence,
     2) round-trip tests,
     3) numerical regression tests.
     
*/

#ifndef BSPATIALALGEBRA_H
#define BSPATIALALGEBRA_H


#ifndef BSPATIALTYPES_H
#include "BSpatialTypes.h"
#endif

#ifndef BSTREAM_H
#include "BStream.h"
#endif

#ifndef BFUNCTIONS_H
#include "BFunctions.h"
#endif

#ifndef BVECTOR6_H
#include "BVector6.h"
#endif

#ifndef BMATRIX6_H
#include "BMatrix6.h"
#endif

#ifndef BPRODUCTS_H
#include "BProducts.h"
#endif

#ifndef BMATRIX63_H
#include "BMatrix63.h"
#endif

#ifndef BINERTIA_H
#include "BInertia.h"
#endif

#ifndef BRBINERTIA_H
#include "BRBInertia.h"
#endif

#ifndef BABINERTIA_H
#include "BABInertia.h"
#endif

#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

#ifndef BADJOINT_H
#include "BAdjoint.h"
#endif

#ifndef BEXPONENTIAL_H
#include "BExponential.h"
#endif

#endif


