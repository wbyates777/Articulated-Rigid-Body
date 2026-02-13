/* BSpatialTypes 21/02/2024

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

#include <map>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>

#include <cmath>
#include <cassert>

#include <glm/vec3.hpp> 
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>


//
// vector and matrix types
//

typedef double                BScalar;

typedef unsigned int          BJointId;
typedef unsigned int          BBodyId;

typedef glm::tvec3<BScalar>   BVector3; 
typedef glm::tmat3x3<BScalar> BMatrix3; 
typedef glm::tquat<BScalar>   BQuat;

typedef std::vector<BScalar>  BMotionSpace;


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

    const BQuat    B_IDENTITY_QUAT(glm::identity<BQuat>());
#else
    constexpr BVector3 B_XAXIS(1.0, 0.0, 0.0);
    constexpr BVector3 B_YAXIS(0.0, 1.0, 0.0);
    constexpr BVector3 B_ZAXIS(0.0, 0.0, 1.0); 

    constexpr BVector3 B_ZERO_3(0.0);
    constexpr BVector3 B_ONE_3(1.0);
    constexpr BMatrix3 B_IDENTITY_3x3(1.0);
    constexpr BMatrix3 B_ZERO_3x3(0.0);

    constexpr BQuat    B_IDENTITY_QUAT(glm::identity<BQuat>());
#endif


constexpr BScalar B_NEAR_ZERO = static_cast<BScalar>(1E-3);

namespace arb {

    //  m^{-1} - style choice - I prefer arb::inverse(m) to m.inverse()
    inline constexpr BMatrix3 
    inverse( const BMatrix3 &m ) 
    { 
        return glm::inverse(m);
    }

    inline bool
    isinvertible( const BMatrix3 &m )
    {
        return (std::abs(glm::determinant(m)) > std::numeric_limits<BScalar>::epsilon());
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
    nearZero( const BQuat &q )  { return (nearZero(q.w) && nearZero(q.x) && nearZero(q.y) && nearZero(q.z)); }
    
    inline bool 
    nearZero( const BMatrix3 &m ) { return (nearZero(m[0]) && nearZero(m[1]) && nearZero(m[2])); }

    
 
    inline bool 
    isnan(const BVector3 &v) { return (std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2])); }

    inline bool 
    isnan( const BQuat &q )  { return (std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z)); }
    
    inline bool 
    isnan(const BMatrix3 &m) { return (isnan(m[0]) || isnan(m[1]) || isnan(m[2])); }
    
 
    
    inline constexpr BScalar 
    min(BScalar v1, BScalar v2) { return std::min(v1, v2); }
    
    inline constexpr BVector3 
    min(const BVector3 &v1, BScalar v2) { return min(v1, BVector3(v2)); }
    
    inline constexpr BVector3 
    min(BScalar v1, const BVector3 &v2) { return min(BVector3(v1), v2); }
    
    inline constexpr BVector3 
    min(const BVector3 &v1, const BVector3 &v2) 
    { 
        return glm::min(v1, v2);
        //return BVector3(std::min(v1[0],v2[0]), std::min(v1[1],v2[1]), std::min(v1[2],v2[2]));
    }
    
    
    inline constexpr BScalar 
    max(BScalar v1, BScalar v2) { return std::max(v1, v2); }
    
    inline constexpr BVector3 
    max(const BVector3 &v1, BScalar v2) { return max(v1, BVector3(v2)); }
    
    inline constexpr BVector3 
    max(BScalar v1, const BVector3 &v2) { return max(BVector3(v1), v2); }
    
    inline constexpr BVector3 
    max(const BVector3 &v1, const BVector3 &v2)
    { 
        return glm::max(v1, v2);
        //return BVector3(std::max(v1[0],v2[0]), std::max(v1[1],v2[1]), std::max(v1[2],v2[2]));
    }
    
    inline constexpr BScalar 
    clamp(BScalar v, BScalar min, BScalar max) { return std::clamp(v, min, max); }
     
    inline constexpr BVector3 
    clamp(const BVector3 &v, BScalar min, BScalar max) 
    { 
        return glm::clamp(v, min, max);
        //return BVector3(std::clamp(v[0], min, max), std::clamp(v[1], min, max), std::clamp(v[2], min, max));
    }
    
    inline constexpr BVector3 
    clamp(const BVector3 &v, const BVector3 &min, const BVector3 &max) 
    { 
        return glm::clamp(v, min, max);
        //return BVector3(std::clamp(v[0], min[0], max[0]), std::clamp(v[1], min[1], max[1]), std::clamp(v[2], min[2], max[2]));
    }
}


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

//
// my glm template stream operators -- customise spatial algebra output
//

namespace  glm 
{

    template<typename T> inline std::ostream&
    operator<<( std::ostream &ostr, const  glm::tvec3<T> &v )
    {
        ostr << v.x << ',' << v.y << ',' << v.z << ' ';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::tvec3<T> &v )
    {
        char delim;
        istr >> v.x >> delim >> v.y >> delim >> v.z;
        return istr;
    }


    template<typename T> inline std::ostream&
    operator<<( std::ostream &ostr, const glm::tmat3x3<T> &m )
    {
        ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << '\n'
             << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << '\n'
             << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << '\n';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::tmat3x3<T> &m )
    {
        istr >> m[0][0] >> m[0][1] >> m[0][2] 
             >> m[1][0] >> m[1][1] >> m[1][2]
             >> m[2][0] >> m[2][1] >> m[2][2];
        return istr;
    }

   
    template<typename T> inline  std::ostream&
    operator<<( std::ostream &ostr, const glm::tquat<T> &q )
    {
        ostr << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::tquat<T> &q )
    {
        istr >> q.w >> q.x >> q.y >> q.z;
        return istr;
    }
}

//
// my stdlib template stream operators 
//

template <class T, class U>
inline std::ostream&
operator<<(std::ostream &ostr, const std::pair<T,U> &v)
{
    ostr << v.first << ' ' << v.second << ' ';
    return ostr;
}

template <class T, class U>
inline std::istream&
operator>>(std::istream &istr, std::pair<T,U> &v)
{
    istr >> v.first >> v.second;
    return istr;
}

// stream operators;
template <typename T>
std::ostream&
operator<<( std::ostream &ostr, const std::vector<T> &v )
{
    ostr << int(v.size()) << '\n';
    
    int count = 0;
    for (auto  i = v.begin(); i != v.end(); ++i)
    {
        ostr << *i << ((++count % 20) ? ' ' : '\n');
    }

    return ostr;
}

template <typename T>
std::istream& 
operator>>( std::istream &istr, std::vector<T> &v )
{
    int len = 0;    
    istr >> len;
    v.resize(len);

    for (auto  i = v.begin(); i != v.end(); ++i)
    {
        istr >> *i;
    }
    
    return istr;
}

template <typename K, typename T>
std::ostream& 
operator<<(std::ostream &ostr, const std::map<K,T> &m)
{
    ostr << int(m.size()) << '\n';

    for (auto  i = m.begin(); i != m.end(); ++i)
    {
        ostr << i->first << ' ' << i->second <<  '\n';
    }
    
    return ostr;
}

template <typename K, typename T>
std::istream&
operator>>(std::istream &istr, std::map<K,T> &m)
{
    m.clear();

    K key = K();
    T val = T();
    
    int len = 0;
    istr >> len;
    
    auto pos = m.begin();
    for (int i = 0; i < len; ++i)
    {
        istr >> key >> val;
        pos = m.insert( pos, typename std::map<K,T>::value_type( key, val ));
    }
    
    return istr;
}



#endif


