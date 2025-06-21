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

typedef std::vector<std::vector<BScalar>> BJointSpace;


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
#else
    constexpr BVector3 B_XAXIS(1.0, 0.0, 0.0);
    constexpr BVector3 B_YAXIS(0.0, 1.0, 0.0);
    constexpr BVector3 B_ZAXIS(0.0, 0.0, 1.0); 

    constexpr BVector3 B_ZERO_3(0.0);
    constexpr BVector3 B_ONE_3(1.0);
    constexpr BMatrix3 B_IDENTITY_3x3(1.0);
    constexpr BMatrix3 B_ZERO_3x3(0.0);   
#endif

constexpr std::array<BScalar, 6> B_ZERO_6 = {0.0};              // BSpatialVector

constexpr std::array<std::array<BScalar, 6>, 6> B_IDENTITY_6x6  // BSpatialMatrix
{
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
};

constexpr std::array<std::array<BScalar, 6>, 6> B_ZERO_6x6     // BSpatialMatrix
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
        ostr << v.x << ", " << v.y << ", " << v.z << ' ';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::tvec3<T> &v )
    {
        istr >> v.x >> v.y >> v.z;
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
        istr >> m[0][0] >> m[0][1] >> m[0][0] 
             >> m[1][0] >> m[1][1] >> m[1][1]
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

template <typename T>
std::ostream&
operator<<( std::ostream &ostr, const std::vector<T> &v )
{
    ostr << v.size() << '\n';
    int count = 0;
    auto print = [&ostr, &count](const T& val) { ostr << val << (!(++count % 10) ? '\n' : ' '); };
    std::for_each(v.begin(), v.end(), print);
    return ostr;
}

template <typename T>
std::istream& 
operator>>( std::istream &istr, std::vector<T> &v )
{
    int len = 0;    
    istr >> len;
    v.resize(len);
    auto read = [&istr](T& val) { istr >> val; };
    std::for_each(v.begin(), v.end(), read);
    return istr;
}

template <typename K, typename T>
std::ostream& 
operator<<(std::ostream &ostr, const std::map<K,T> &m)
{
    ostr << m.size() << '\n';
    auto print = [&ostr](const typename std::map<K,T>::value_type &val) { ostr << val.first << ' ' << val.second <<  '\n'; };
    std::for_each(m.begin(), m.end(), print);
    return ostr;
}

template <typename K, typename T>
std::istream&
operator>>(std::istream &istr, std::map<K,T> &m)
{
    m.clear();

    K key = K();
    T item = T();
    
    int len = 0;
    istr >> len;
    
    auto lastPos = m.begin();
    for (int i = 0; i < len; ++i)
    {
        istr >> key >> item;
        lastPos = m.insert( lastPos, typename std::map<K,T>::value_type( key, item ));
    }
    return istr;
}


#endif


