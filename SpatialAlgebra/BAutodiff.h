/* BAutodiff 20/02/2026

 $$$$$$$$$$$$$$$$$$$
 $   BAutodiff.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 autodiff is a c++17 library that facilitates automatic differentiation (AD).
 This file contains the function defs needed for autodiff to work with GLM
 
 To enable AD ensure that glm and the autodiff header-only lib is present, 
 and that this file is included in BSpatialTypes.h 
 
 https://en.wikipedia.org/wiki/Automatic_differentiation

 see https://autodiff.github.io
 see https://github.com/autodiff/autodiff

*/


#ifndef __BAUTODIFF_H__
#define __BAUTODIFF_H__

#define ARB_USE_AUTODIFF

#include <autodiff/forward/real.hpp>

#define GLM_FORCE_UNRESTRICTED_GENTYPE  // required by GLM
#define GLM_FORCE_CTOR_INIT             // required by GLM

#include <glm/vec3.hpp> 

// interoperability between glm and autodiff 
inline glm::dvec3 
operator+( const glm::vec<3,autodiff::real> &a, const glm::dvec3 &b ) 
{
    return glm::dvec3(a.x[0] + b.x, a.y[0] + b.y, a.z[0] + b.z); 
}

inline glm::dvec3 
operator+( const glm::dvec3 &a, const glm::vec<3,autodiff::real> &b ) 
{
    return glm::dvec3(a.x + b.x[0], a.y + b.y[0], a.z + b.z[0]); 
}


// my stream operators need operator>>()
inline std::istream& 
operator>>(std::istream& istr, autodiff::real& val) 
{
    istr >> val.val();
    return istr;
}

// glm stream operators also need operator>>()
namespace  glm { using ::operator>>; }

namespace std {
    
    // v[0] or autodiff::val(v) is value and v[1] or autodiff::derivative(v) is gradient 
    inline bool 
    isnan( const autodiff::real &v ) { return std::isnan(v[0]) || std::isnan(v[1]); }
    
    inline constexpr autodiff::real 
    clamp( const autodiff::real &d, const autodiff::real &min, const autodiff::real &max ) 
    {
        const autodiff::real t = d < min ? min : d;
        return t > max ? max : t;
    }
    
    // some GLM functions i.e. glm::dot and glm::cross have static asserts 
    // static_assert(std::numeric_limits<T>::is_iec559, "this function accepts only floating-point inputs");
    // this ensures that autodiff::real is considered as a floating-point input
    // WARNING: always check that the glm function you are using preserves gradients
    template <>
    struct numeric_limits<autodiff::real> : public numeric_limits<double> 
    {
        static constexpr bool is_iec559 = true; 
    };
}




#endif


