/* BSpatialTypes 19/02/2026

 $$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialTypes.h   $
 $$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Essential typedefs, constant definitions, and template stream operators

*/


#ifndef __BSPATIALTYPESDIFF_H__
#define __BSPATIALTYPESDIFF_H__

#include <map>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>

#include <cmath>
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


#include <glm/vec3.hpp> 
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

//
// vector and matrix types
//

typedef unsigned int          BJointId;
typedef unsigned int          BBodyId;

typedef glm::vec<3,BScalar>   BVector3; 
typedef glm::mat<3,3,BScalar> BMatrix3; 
typedef glm::qua<BScalar>     BQuat;

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

#if defined(ARB_USE_FLOAT)
// interoperability between glm float and double 
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



//
// my glm template stream operators -- customise spatial algebra output
//

namespace  glm 
{

    template<typename T> inline std::ostream&
    operator<<( std::ostream &ostr, const  glm::vec<3,T> &v )
    {
        ostr << v.x << ',' << v.y << ',' << v.z << ' ';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::vec<3,T> &v )
    {
        char delim;
        istr >> v.x >> delim >> v.y >> delim >> v.z;
        return istr;
    }


    template<typename T> inline std::ostream&
    operator<<( std::ostream &ostr, const glm::mat<3,3,T> &m )
    {
        ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << '\n'
             << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << '\n'
             << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << '\n';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::mat<3,3,T> &m )
    {
        istr >> m[0][0] >> m[0][1] >> m[0][2] 
             >> m[1][0] >> m[1][1] >> m[1][2]
             >> m[2][0] >> m[2][1] >> m[2][2];
        return istr;
    }

   
    template<typename T> inline  std::ostream&
    operator<<( std::ostream &ostr, const glm::qua<T> &q )
    {
        ostr << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::qua<T> &q )
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

    for (auto i = v.begin(); i != v.end(); ++i)
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

    for (auto i = m.begin(); i != m.end(); ++i)
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


