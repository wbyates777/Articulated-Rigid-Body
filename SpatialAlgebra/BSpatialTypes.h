/* BSpatialTypes 19/02/2026

 $$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialTypes.h   $
 $$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Essential typedefs, constant definitions, and template stream operators

*/


#ifndef __BSPATIALTYPES_H__
#define __BSPATIALTYPES_H__


#include <vector>
#include <array>
#include <cassert>

//
// GLM flags
//
// Best to experiment and see which combination suits you best
// Note some combinations won't compile. See GLM documentation for details. 
//
// #define GLM_FORCE_XYZW_ONLY  // remove unions (can prevent internal layout errors)
//
// either
// #define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES 
// or
// #define GLM_FORCE_INTRINSICS // enable SIMD (needs c++23)
// or
// #define GLM_FORCE_PURE       // disable SIMD 
//
// or define none of the above 
//

// select basic underlying type
#define ARB_USE_AUTODIFF

//
// Basic Underlying Type BScalar
//
#if defined(ARB_USE_AUTODIFF)

    #ifndef __BAUTODIFF_H__
    #include "BAutodiff.h"
    #endif

    typedef  autodiff::real BScalar;

#elif defined(ARB_USE_DOUBLE)

    typedef double BScalar;

#elif defined(ARB_USE_FLOAT)

    // causes internal checks to fails due to prec - use B_NEAR_ZERO=1E-1
    typedef float BScalar; 

#endif


#include <glm/vec4.hpp> 
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>

//
// vector and matrix types -- these are potentially differentiable
//

typedef unsigned int          BJointId;
typedef unsigned int          BBodyId;

typedef glm::vec<3,BScalar>   BVector3; 
typedef glm::mat<3,3,BScalar> BMatrix3; 

typedef glm::vec<4,BScalar>   BVector4;
typedef glm::mat<4,4,BScalar> BMatrix4;

typedef glm::qua<BScalar>     BQuat;

typedef std::vector<BScalar>  BMotionSpace;


constexpr  int B_X = 0;
constexpr  int B_Y = 1;
constexpr  int B_Z = 2;

constexpr  BScalar B_SCALAR_MAX     =  std::numeric_limits<BScalar>::max();
constexpr  BScalar B_SCALAR_MIN     = -std::numeric_limits<BScalar>::max();
constexpr  BScalar B_EPS            =  std::numeric_limits<BScalar>::epsilon();
constexpr  BScalar B_EPS2           =  std::numeric_limits<BScalar>::epsilon() * std::numeric_limits<BScalar>::epsilon();

//
// vector and matrix constants
//
#ifdef GLM_FORCE_INTRINSICS
    const BVector3 B_XAXIS(1.0, 0.0, 0.0);
    const BVector3 B_YAXIS(0.0, 1.0, 0.0);
    const BVector3 B_ZAXIS(0.0, 0.0, 1.0); 

    const BVector3 B_ZERO_3(0.0);
    const BVector3 B_ONE_3(1.0);
    const BMatrix3 B_IDENTITY_3x3(1.0);
    const BMatrix3 B_ZERO_3x3(0.0);

    const BVector4 B_ZERO_4(0.0);
    const BVector4 B_ONE_4(1.0);
    const BMatrix4 B_IDENTITY_4x4(1.0); 
    const BMatrix4 B_ZERO_4x4(0.0);

    const BQuat    B_IDENTITY_QUAT(glm::identity<BQuat>());
#else
    constexpr BVector3 B_XAXIS(1.0, 0.0, 0.0);
    constexpr BVector3 B_YAXIS(0.0, 1.0, 0.0);
    constexpr BVector3 B_ZAXIS(0.0, 0.0, 1.0); 

    constexpr BVector3 B_ZERO_3(0.0);
    constexpr BVector3 B_ONE_3(1.0);
    constexpr BMatrix3 B_IDENTITY_3x3(1.0);
    constexpr BMatrix3 B_ZERO_3x3(0.0);

    constexpr BVector4 B_ZERO_4(0.0);
    constexpr BVector4 B_ONE_4(1.0);
    constexpr BMatrix4 B_IDENTITY_4x4(1.0); 
    constexpr BMatrix4 B_ZERO_4x4(0.0);

    constexpr BQuat    B_IDENTITY_QUAT(glm::identity<BQuat>());
#endif

#if defined(ARB_USE_FLOAT)
//
// inter-operability between glm float and glm double 
//
inline glm::dvec3 
operator+( const glm::vec3 &a, const glm::dvec3 &b ) 
{
    return glm::dvec3(a.x + b.x, a.y + b.y, a.z + b.z); 
}
inline glm::dvec3 
operator+( const glm::dvec3 &a, const glm::vec3 &b ) 
{
    return glm::dvec3(a.x + b.x, a.y + b.y, a.z + b.z); 
}
inline glm::dvec4 
operator+( const glm::vec4 &a, const glm::dvec4 &b ) 
{
    return glm::dvec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w); 
}
inline glm::dvec4 
operator+( const glm::dvec4 &a, const glm::vec4 &b ) 
{
    return glm::dvec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w); 
}
#endif

constexpr BScalar B_NEAR_ZERO = static_cast<BScalar>(1E-3);

constexpr std::array<BScalar, 6> B_ZERO_6 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // BVector6

constexpr std::array<std::array<BScalar, 6>, 6> B_IDENTITY_6x6  // BMatrix6
{
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
};

constexpr std::array<std::array<BScalar, 6>, 6> B_ZERO_6x6     // BMatrix6
{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

constexpr std::array<std::array<BScalar, 6>, 3> B_ZERO_3x6
{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

constexpr std::array<std::array<BScalar, 3>, 6> B_ZERO_6x3
{
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0
};

#ifndef __BSTREAM_H__
#include "BStream.h"
#endif


#endif


