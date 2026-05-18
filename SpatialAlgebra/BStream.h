/* BStream 11/05/2026

 $$$$$$$$$$$$$$$$$
 $   BStream.h   $
 $$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 All stream operators in one place.
 
*/


#ifndef __BSTREAM_H__
#define __BSTREAM_H__

#include <iostream>
#include <map>
#include <vector>

#include <glm/vec4.hpp> 
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>



//
// my glm template stream operators -- customise spatial algebra output
//

namespace  glm 
{
    // vectors
    template<typename T> inline std::ostream&
    operator<<( std::ostream &ostr, const  glm::vec<2,T> &v )
    {
        ostr << v.x << ", " << v.y << ' ';
        return ostr;
    }
    
    template<typename T> inline std::istream&
    operator>>( std::istream &istr, glm::vec<2,T> &v )
    {
        char delim;
        istr >> v.x >> delim >> v.y;
        return istr;
    }
    
    
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

    
    template<typename T>  inline std::ostream&
    operator<<( std::ostream& ostr, const glm::vec<4,T> &v )
    {
        ostr << v.x << ',' << v.y << ',' << v.z << ',' << v.w << ' ';
        return ostr;
    }

    template<typename T> inline  std::istream&
    operator>>( std::istream& istr, glm::vec<4,T> &v )
    {
        char tmp;
        istr >> v.x >> tmp >> v.y >> tmp >> v.z >> tmp >> v.w;
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
    
    template<typename T> inline std::ostream&
    operator<<( std::ostream& ostr, const glm::mat<4,4,T> &m )
    {
        ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << ' ' << m[0][3] << '\n'
             << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << ' ' << m[1][3] << '\n'
             << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << ' ' << m[2][3] << '\n'
             << m[3][0] << ' ' << m[3][1] << ' ' << m[3][2] << ' ' << m[3][3] << '\n';
        return ostr;
    }

    template<typename T> inline std::istream&
    operator>>( std::istream& istr, glm::mat<4,4,T> &m )
    {
        istr >> m[0][0] >> m[0][1] >> m[0][2] >> m[0][3]
             >> m[1][0] >> m[1][1] >> m[1][2] >> m[1][3]
             >> m[2][0] >> m[2][1] >> m[2][2] >> m[2][3]
             >> m[3][0] >> m[3][1] >> m[3][2] >> m[3][3];
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
