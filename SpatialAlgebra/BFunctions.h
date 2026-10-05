/* BFunctions 21/02/2026

 $$$$$$$$$$$$$$$$$$$$
 $   BFunctions.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


 3D functions - mostly from GLM.

*/


#ifndef BFUNCTIONS_H
#define BFUNCTIONS_H

#ifndef BSPATIALTYPES_H
#include "BSpatialTypes.h"
#endif


namespace arb {

    inline BScalar
    length( const BVector3 &v )  { return glm::length(v); } 
    
    inline BScalar
    length( const BQuat &q )  { using std::sqrt; return sqrt(glm::dot(q,q)); } 
    
    inline BScalar
    length2( const BVector3 &v ) { return glm::dot(v,v); } 
    
    inline BVector3
    normalize( const BVector3 &v ) { return v / arb::length(v); } 
    
    inline BQuat
    normalize( const BQuat &q ) { return q / arb::length(q); }
    
    inline constexpr BMatrix3 
    inverse( const BMatrix3 &m ) { return glm::inverse(m); }
    
    inline BScalar
    trace( const BMatrix3 &m )  {  return  m[0][0] + m[1][1] + m[2][2]; }
    
    inline constexpr BScalar 
    determinant( const BMatrix3 &m ) { return glm::determinant(m); }
    
    inline bool
    isinvertible( const BMatrix3 &m ) { using std::abs; return abs(arb::determinant(m)) > B_EPS; }
    
    inline constexpr BMatrix3 
    transpose( const BMatrix3 &m )  { return glm::transpose(m); }


    // 3D rotations - all angles in radians
    inline constexpr BMatrix3 
    rot( BScalar theta, const BVector3 &axis ) 
    // WARNING: axis *must* be normalized
    {
        using std::abs;
        assert(abs(arb::length(axis) - 1.0) < 1E-8);
        return glm::mat3_cast(glm::angleAxis(theta, axis));
    }   

    inline constexpr BMatrix3 
    rotx( BScalar theta ) 
    {
        using std::sin;
        using std::cos;
        
        const BScalar s = sin(theta);
        const BScalar c = cos(theta);
        return BMatrix3( 1.0, 0.0, 0.0,
                         0.0,   c,   s,
                         0.0,  -s,   c );
    }
    
    inline constexpr BMatrix3 
    roty( BScalar theta ) 
    {
        using std::sin;
        using std::cos;
        
        const BScalar s = sin(theta);
        const BScalar c = cos(theta);
        return BMatrix3(   c, 0.0,  -s,
                         0.0, 1.0, 0.0,
                           s, 0.0,   c ); 
    }
    
    inline constexpr BMatrix3 
    rotz( BScalar theta ) 
    {
        using std::sin;
        using std::cos;
        
        const BScalar s = sin(theta);
        const BScalar c = cos(theta);
        return  BMatrix3(  c,   s, 0.0,
                          -s,   c, 0.0,
                         0.0, 0.0, 1.0 );
    }
    
    
    inline bool 
    nearZero( BScalar p ) { return ((p > -B_NEAR_ZERO) && (p < B_NEAR_ZERO)); }

    inline bool 
    nearZero( BScalar p, BScalar tol ) { return ((p > -tol) && (p < tol)); }
    
    inline bool 
    nearZero( const BVector3 &v ) { return (nearZero(v[0]) && nearZero(v[1]) && nearZero(v[2])); }

    inline bool 
    nearZero( const BMatrix3 &m ) { return (nearZero(m[0]) && nearZero(m[1]) && nearZero(m[2])); }

    inline bool 
    nearZero( const BQuat &q ) { return (nearZero(q.w) && nearZero(q.x) && nearZero(q.y) && nearZero(q.z)); }
    

    inline bool 
    isnan(const BVector3 &v) {   return (std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2])); }

    inline bool 
    isnan( const BQuat &q )  { using std::isnan; return (isnan(q.w) || isnan(q.x) || isnan(q.y) || isnan(q.z)); }
    
    inline bool 
    isnan( const BMatrix3 &m ) { return (isnan(m[0]) || isnan(m[1]) || isnan(m[2])); }

    
    
    inline constexpr BScalar 
    min( BScalar v1, BScalar v2 ) { using std::min; return min(v1, v2); }
    
    inline constexpr BVector3 
    min( const BVector3 &v1, BScalar v2 ) { return min(v1, BVector3(v2)); }
    
    inline constexpr BVector3 
    min( BScalar v1, const BVector3 &v2 ) { return min(BVector3(v1), v2); }
    
    inline constexpr BVector3 
    min( const BVector3 &v1, const BVector3 &v2 ) 
    { 
        return glm::min(v1, v2);
    }
      
    inline constexpr BScalar 
    max( BScalar v1, BScalar v2 ) { using std::max; return max(v1, v2); }
    
    inline constexpr BVector3 
    max( const BVector3 &v1, BScalar v2 ) { return max(v1, BVector3(v2)); }
    
    inline constexpr BVector3 
    max( BScalar v1, const BVector3 &v2 ) { return max(BVector3(v1), v2); }
    
    inline constexpr BVector3 
    max( const BVector3 &v1, const BVector3 &v2 )
    { 
        return glm::max(v1, v2);
    }
    
    
    inline constexpr BScalar 
    clamp( BScalar v, BScalar lo, BScalar hi ) { using std::clamp; return clamp(v, lo, hi); }
    
    inline constexpr BVector3 
    clamp( const BVector3 &v, BScalar lo, BScalar hi ) 
    { 
        return BVector3(clamp(v[0], lo, hi), clamp(v[1], lo, hi), clamp(v[2], lo, hi));
    }
    
    inline constexpr BVector3 
    clamp( const BVector3 &v, const BVector3 &lo, const BVector3 &hi ) 
    { 
        return BVector3(clamp(v[0], lo[0], hi[0]), clamp(v[1], lo[1], hi[1]), clamp(v[2], lo[2], hi[2]));
    }
    
}

#endif


