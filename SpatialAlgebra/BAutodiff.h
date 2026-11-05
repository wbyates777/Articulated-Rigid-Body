/*!

         $$$$$$$$$$$$$$$$$$$
         $   BAutodiff.h   $
         $$$$$$$$$$$$$$$$$$$

 @file BAutodiff.h
 @ingroup SpatialAlgebra
 date 20/02/2026
 
 by W.B. Yates
 
 ---
 
 @brief This header file contains the function definitions needed for autodiff to work with glm.
 
 autodiff is a c++17 library that facilitates automatic differentiation (AD).

 AD uses the chain rule to propagate analytical derivatives through the code. 
 This end-to-end differentiability facilitates the application of more advanced optimisation and machine learning techniques. 
 
 To enable AD ensure that glm and the autodiff header-only lib is present, 
 and that this file is included in BSpatialTypes.h 
 
 @see https://en.wikipedia.org/wiki/Automatic_differentiation
 @see https://github.com/autodiff/autodiff
 @see https://en.wikipedia.org/wiki/Chain_rule

 @warning Always check that any new glm function you wish to use preserves gradients.
*/


#ifndef BAUTODIFF_H
#define BAUTODIFF_H

#define ARB_USE_AUTODIFF

#include <autodiff/forward/real.hpp>

#define GLM_FORCE_UNRESTRICTED_GENTYPE  /// @note required by GLM
#define GLM_FORCE_CTOR_INIT             /// @note required by GLM

#include <glm/vec3.hpp>
#include <glm/vec4.hpp> 

///
/// inter-operability between glm and autodiff  
/// @note derivatives not preserved here
///
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

inline glm::dvec4 
operator+( const glm::vec<4,autodiff::real> &a, const glm::dvec4 &b ) 
{
    return glm::dvec4(a.x[0] + b.x, a.y[0] + b.y, a.z[0] + b.z, a.w[0] + b.w); 
}

inline glm::dvec4 
operator+( const glm::dvec4 &a, const glm::vec<4,autodiff::real> &b ) 
{
    return glm::dvec4(a.x + b.x[0], a.y + b.y[0], a.z + b.z[0], a.w + b.w[0]); 
}


//
// stream operators 
//

/// my stream operators need operator>>() for autodiff::real
inline std::istream& 
operator>>(std::istream& istr, autodiff::real& val) 
{
    istr >> val.val();
    return istr;
}

namespace glm { 
    
    /// glm stream operators also need operator>>() for autodiff::real
    using ::operator>>; 
}

//
// trigonometry functions 
//
namespace glm { 
    
    inline autodiff::real 
    sin(const autodiff::real& a) 
    {
        using std::sin;
        return sin(a);
    }

    inline autodiff::real 
    cos(const autodiff::real& a) 
    {
        using std::cos;
        return cos(a);
    }
    
    inline autodiff::real 
    tan(const autodiff::real& a) 
    {
        using std::tan;
        return tan(a);
    }
}

namespace std {
    
    // v[0] or autodiff::val(v) is value and v[1] or autodiff::derivative(v) is gradient 
    inline bool 
    isnan( const autodiff::real &v ) { return std::isnan(autodiff::val(v)) || std::isnan(autodiff::derivative(v)); }
    
    inline constexpr autodiff::real 
    clamp( const autodiff::real &d, const autodiff::real &lo, const autodiff::real &hi ) 
    {
        const autodiff::real t = d < lo ? lo : d;
        return t > hi ? hi : t;
    }
    
    /// some GLM functions i.e. glm::dot and glm::cross have static asserts 
    /// static_assert(std::numeric_limits<T>::is_iec559, "this function accepts only floating-point inputs");
    /// this ensures that autodiff::real is considered as a floating-point input
    /// @warning always check that the glm function you are using preserves gradients
    template <>
    struct numeric_limits<autodiff::real> : public numeric_limits<double> 
    {
        static constexpr bool is_iec559 = true; 
    };
}


#endif


