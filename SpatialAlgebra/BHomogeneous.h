/*!

         $$$$$$$$$$$$$$$$$$$$$$
         $   BHomogeneous.h   $
         $$$$$$$$$$$$$$$$$$$$$$

 @file BHomogeneous.h 
 @ingroup SpatialAlgebra
 date 13/10/2026
 
 by W.B. Yates
 
 ---
 
 @brief Conversion functions between spatial transforms X (BTransform) and 4x4-homogeneous matrices M (BMatrix4).

 These functions provide conversion utilities between spatial rigid-body transforms 
 BTransforms $X$ and standard 4x4 homogeneous transformation matrices BMatrix4 $M$.
 
 Given a spatial transform BTransform X and any BVector3 p we have:
 
 @code
   X.applyTranspose(p) == arb::toHomogeneous(X) * BVector4(p, 1.0);
   X.apply(p)          == arb::toHomogeneousInverse(X) * BVector4(p, 1.0);
 @endcode
 
 with
 
        toHomogeneous(X)        = | E    r  |
                                  | 0    1  |

        toHomogeneousInverse(X) = | E^T  -E^T r |
                                  |  0     1    |
 
 @note M must be a rigid body transform (orthonormal rotation, no scale or shear)

*/


#ifndef BHOMOGENEOUS_H
#define BHOMOGENEOUS_H

#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

namespace arb
{

    /// Converts the spatial transform $X$ to a body-to-base homogeneous matrix.
    inline BMatrix4 
    toHomogeneous( const BTransform &X ) 
    {
        BMatrix4 res(X.E());
        res[3] = BVector4(X.r(), 1.0);
        return res;
    }
    
    
    /// Converts the spatial transform $X$ to a base-to-body homogeneous matrix. 
    inline BMatrix4 
    toHomogeneousInverse( const BTransform &X ) 
    {
        const BMatrix3 ET = arb::transpose(X.E());
        BMatrix4 res(ET);
        res[3] = BVector4(-(ET * X.r()), 1.0);
        return res;
    }
    

    /// Converts a body-to-base homogeneous matrix $M$ to a spatial transform $X$.
    inline BTransform
    fromHomogeneous( const BMatrix4 &M ) 
    {
        return BTransform(BMatrix3(M), BVector3(M[3]));
    }

    /// Converts a base-to-body homogeneous matrix $M$ to a spatial transform $X$.
    inline BTransform 
    fromHomogeneousInverse( const BMatrix4 &M ) 
    {
        const BMatrix3 E = arb::transpose(BMatrix3(M));  // rotation: base-to-body
        const BVector3 t = BVector3(M[3]);               // translation of the base-to-body matrix (-E * r)
        return BTransform(E, -(E * t));
    }
    
}

#endif


