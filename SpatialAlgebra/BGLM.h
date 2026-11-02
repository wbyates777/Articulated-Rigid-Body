/*!

         $$$$$$$$$$$$$$
         $   BGLM.h   $
         $$$$$$$$$$$$$$

 @file BGLM.h
 @ingroup SpatialAlgebra
 date 30/05/2026
 
 by W.B. Yates
 
 ---
 
 @brief This include file ensures that glm is using the same setup and configuration everywhere.

 Otherwise we can get mysterious build/runtime errors if SIMD or alignment
 is activated in some code units and not others.
 
 A better/safer alternatively is to pass these defines to the compiler i.e -DGLM_FORCE_PURE 
 
*/


#ifndef BGLM_H
#define BGLM_H

//
// glm flags - see glm documentation for details.
//
// Best to experiment and see which combination suits you best
// Note some combinations won't compile. 
//
// #define GLM_FORCE_XYZW_ONLY  // remove unions (can prevent internal layout errors)
// #define GLM_FORCE_PURE       // disable SIMD 
//

// switch on SIMD -- use both switches for maximum effect
//#define GLM_FORCE_INTRINSICS
//#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
// do not use SIMD with ARB_USE_AUTODIFF defined

#endif


