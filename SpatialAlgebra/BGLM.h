/* BGLM 30/05/2026

 $$$$$$$$$$$$$$
 $   BGLM.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Use this include to ensure that glm is using the same 'setup/config' everywhere
 Otherwise we can get mysterious build/runtime errors if SIMD or alignment
 is activated in some code units and not others.
 
 A better/safer alternatively is to pass these defines to the compiler 
  i.e -DGLM_FORCE_PURE 
 
*/


#ifndef __BGLM_H__
#define __BGLM_H__

//
// GLM flags - see GLM documentation for details.
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


