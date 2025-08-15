/* Spatial Algebra 15/08/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialAlgebra.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 Top level include file for a compact, light-weight, header only implementation of spatial algebra as presented in:

  "Rigid Body Dynamics Algorithms" (RBDA), R. Featherstone, Springer, 2008 (see https://royfeatherstone.org). 
 
 Depends on the  header only GLM library (see https://github.com/g-truc/glm).

 Specifically, we employ the 3D types: 
 
   glm::dvec3, glm::dmat3, glm::quat 
 
 and the functions: 
 
   glm::cross(v1, v2), glm::dot(v1, v2), glm::length(v1), glm::inverse(m1), glm::toMat3(q)

*/

#ifndef __BSPATIALALGEBRA_H__
#define __BSPATIALALGEBRA_H__

#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BSPATIALVECTOR_H__
#include "BSpatialVector.h"
#endif

#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif

#ifndef __BMATRIX63_H__
#include "BMatrix63.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif

#ifndef __BABINERTIA_H__
#include "BABInertia.h"
#endif

#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif


#endif


