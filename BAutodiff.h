/* BAutodiff 20/02/2026

 $$$$$$$$$$$$$$$$$$$
 $   BAutodiff.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 BAutodiff.h is a wrapper for autodiff; a c++17 library that facilitates automatic differentiation (AD).
 To enable AD ensure that the autodiff lib is installed, and that this file isincluded in BSpatialTypes.h
 
 https://en.wikipedia.org/wiki/Automatic_differentiation

 see https://autodiff.github.io
 see https://github.com/autodiff/autodiff

*/


#ifndef __BAUTODIFF_H__
#define __BAUTODIFF_H__



#include <autodiff/forward/real.hpp>

//#define GLM_FORCE_PURE          // disable SIMD (AutoDiff doesn't support it)
//#define GLM_FORCE_XYZW_ONLY     // remove unions (Prevents internal layout errors)
#define GLM_FORCE_UNRESTRICTED_GENTYPE
#define GLM_FORCE_CTOR_INIT


// my stream operators need operator>>
inline std::istream& 
operator>>(std::istream& istr, autodiff::real& val) 
{
    istr >> val.val();
    return istr;
}

// glm stream operators also need operator>>
namespace  glm {
    using ::operator>>;
}

namespace std {
    
    inline bool 
    isnan(const autodiff::real &v) { return std::isnan(v.val()); }
    
    inline constexpr autodiff::real 
    clamp(const autodiff::real &d, const autodiff::real &min, const autodiff::real &max) 
    {
        const autodiff::real t = d < min ? min : d;
        return t > max ? max : t;
    }
    
    // glm::dot and glm::cross have static asserts 
    // static_assert(std::numeric_limits<T>::is_iec559, "'dot/cross' accepts only floating-point inputs");
    // this gets round that
    template <>
    struct numeric_limits<autodiff::real> : public numeric_limits<double> 
    {
        static constexpr bool is_iec559 = true; 
    };
}


typedef  autodiff::real BScalar;

#endif


