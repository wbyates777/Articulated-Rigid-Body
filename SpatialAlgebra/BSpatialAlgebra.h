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

#ifndef __BSPATIALALGEBRA_H__
#define __BSPATIALALGEBRA_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BFUNCTIONS_H__
#include "BFunctions.h"
#endif

#ifndef __BVECTOR6_H__
#include "BVector6.h"
#endif

#ifndef __BMATRIX6_H__
#include "BMatrix6.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif

#ifndef __BMATRIX63_H__
#include "BMatrix63.h"
#endif

#ifndef __BINERTIA_H__
#include "BInertia.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif

#ifndef __BABINERTIA_H__
#include "BABInertia.h"
#endif

#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif

#ifndef __BEXPLOG_H__
#include "BExpLog.h"
#endif

#ifndef __BADJOINT_H__
#include "BAdjoint.h"
#endif

#endif


