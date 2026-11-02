/*!
 * @defgroup SpatialAlgebra Spatial Algebra
 * @brief Mathematical classes and utilities for a differentiable spatial algebra.
 * 
 * ARB is a compact (header-only) implementation of a differentiable spatial algebra, designed for use in computer graphics, simulation, and robotics
 * @see "Rigid Body Dynamics Algorithms" (RBDA), R. Featherstone, Springer, 2008. 
 * @see "Modern Robotics: Mechanics, Planning, and Control", Lynch K. M., Park F. C., 2017.
 */

/*! 

         $$$$$$$$$$$$$$$$$$$$$$$$$
         $   BSpatialAlgebra.h   $
         $$$$$$$$$$$$$$$$$$$$$$$$$

 @file BSpatialAlgebra.h
 @ingroup SpatialAlgebra
 date 15/08/2025
 
 by W.B. Yates
 
 ---
 
 @brief A compact, light-weight, header only impmemenation of differentiable spatial algebra.
 

 ARB ALGEBRA CONVENTIONS
 ------------------------
 Underlying 3D matrix library: glm

 Conventions:
 - Matrices are stored/accessed in glm column-major form.
 - Vectors are treated as column vectors.
 - Matrix-vector multiplication A * x represents standard linear action on a column vector.
 - Printed mathematical formulas in comments follow standard textbook block-matrix notation.
 - Therefore, explicit arb::transpose() calls may be required in code to match textbook formulas.

 IMPORTANT
 ------------------------
 Do not validate matrix expressions by visual inspection alone.
 Always validate using:
 - explicit block-matrix equivalence,
 - round-trip tests,
 - numerical regression tests.
 
 Depends on glm, tinyxml2, aurodiff and libccd libraries.
 
 @see https://royfeatherstone.org
 @see https://github.com/g-truc/glm
 @see https://github.com/autodiff/autodiff
 @see https://github.com/danfis/libccd
 @see https://github.com/leethomason/tinyxml2
 
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

#ifndef BHOMOGENEOUS_H
#include "BHomogeneous.h"
#endif

#endif


