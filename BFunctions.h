/* BFunctions 21/02/2026

 $$$$$$$$$$$$$$$$$$$$
 $   BFunctions.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 3D functions - mostly from GLM.

 
*/


#ifndef __BFUNCTIONS_H__
#define __BFUNCTIONS_H__

#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

namespace arb {

    inline BScalar
    length( const BVector3 &v ) 
    { 
        using std::sqrt; return sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2])); 
        //  return glm::length(v);
    } 
    
    //  m^{-1} - style choice - I prefer arb::inverse(m) to m.inverse()
    inline constexpr BMatrix3 
    inverse( const BMatrix3 &m ) 
    { 
        return glm::inverse(m);
    }
    
    inline constexpr BScalar 
    determinant( const BMatrix3 &m ) 
    { 
        return glm::determinant(m);
        //return   m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        //       - m[1][0] * (m[0][1] * m[2][2] - m[0][2] * m[2][1])
        //       + m[2][0] * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
    }
    
    inline bool
    isinvertible( const BMatrix3 &m )
    {
        using std::abs; return abs(arb::determinant(m)) > std::numeric_limits<BScalar>::epsilon();
    }
    
    //  m^{T} - style choice - I prefer arb::transpose(m) to m.transpose() 
    inline constexpr BMatrix3 
    transpose( const BMatrix3 &m ) 
    { 
        return glm::transpose(m);
        //BMatrix3 retVal;
        //for ( int i = 0; i < 3; ++i )
        //    for ( int j = 0; j < 3; ++j )
        //        retVal[i][j] = m[j][i];
        //return retVal;  
    }
    
    
    inline bool 
    nearZero( BScalar p ) { return ((p > -B_NEAR_ZERO) && (p < B_NEAR_ZERO)); }

    inline bool 
    nearZero( const BVector3 &v ) { return (nearZero(v[0]) && nearZero(v[1]) && nearZero(v[2])); }

    inline bool 
    nearZero( const BMatrix3 &m ) { return (nearZero(m[0]) && nearZero(m[1]) && nearZero(m[2])); }

    inline bool 
    nearZero( const BQuat &q ) { return (nearZero(q.w) && nearZero(q.x) && nearZero(q.y) && nearZero(q.z)); }
    

    inline bool 
    isnan(const BVector3 &v) { return (std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2])); }

    inline bool 
    isnan( const BQuat &q )  { return (std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z)); }
    
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
        //return BVector3(min(v1[0],v2[0]), min(v1[1],v2[1]), min(v1[2],v2[2]));
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
        //return BVector3(max(v1[0],v2[0]), max(v1[1],v2[1]), max(v1[2],v2[2]));
    }
    
    inline constexpr BScalar 
    clamp( BScalar d, BScalar min, BScalar max ) 
    {
        return std::clamp(d,min,max);
    }
    
    inline constexpr BVector3 
    clamp( const BVector3 &v, BScalar min, BScalar max ) 
    { 
        return BVector3(clamp(v[0], min, max), clamp(v[1], min, max), clamp(v[2], min, max));
    }
    
    inline constexpr BVector3 
    clamp( const BVector3 &v, const BVector3 &min, const BVector3 &max ) 
    { 
        return BVector3(clamp(v[0], min[0], max[0]), clamp(v[1], min[1], max[1]), clamp(v[2], min[2], max[2]));
    }
    
}

#endif


