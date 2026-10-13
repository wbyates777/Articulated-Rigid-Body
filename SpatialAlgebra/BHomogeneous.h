/* BHomogeneous 13/10/2026

 $$$$$$$$$$$$$$$$$$$$$$
 $   BHomogeneous.h   $
 $$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Conversion functions between spatial transforms X and 4x4-homogeneous matricies M

 Given a spatial transform BTransform X and any BVector3 pos then
 
       X.applyTranspose(pos) == arb::toHomogeneous(X) * BVector4(pos, 1.0);
 
*/


#ifndef __BHOMOGENEOUS_H__
#define __BHOMOGENEOUS_H__


#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

namespace arb
{

    /**
     * converts a body-to-base spatial transform X to a body-to-base homogeneous matrix M (for graphics)
     */
    inline constexpr BMatrix4 
    toHomogeneous( const BTransform &X ) 
    {
        BMatrix4 res(glm::transpose(X.E()));
        res[3] = BVector4(X.r(), 1.0);
        return res;
    }
    
    /**
     * converts a body-to-base spatial transform directly to a base-to-body homogeneous matrix M (for raycasting/collisions)
     * avoids arb::inverse() by constructing matrix analytically
     */
    inline constexpr BMatrix4 
    toHomogeneousInverse( const BTransform &X ) 
    // this is Featherstone's pluho - Plücker-to-homogeneous
    {
        BMatrix4 res(X.E());
        res[3] = BVector4(-(X.E() * X.r()), 1.0);
        return res;
    }
    
    /**
     * converts a body-to-base homogeneous matrix M back into a spatial transform X(E, r)
     */
    inline constexpr BTransform
    fromHomogeneous( const BMatrix4 &M ) 
    {
        const BMatrix3 R = glm::mat3(M);          // body-to-base rotation
        const BVector3 r = glm::dvec3(M[3]);      // base-to-body position
        const BMatrix3 E = glm::transpose(R);     // base-to-body rotation
        return BTransform(E, r);
    }

    /**
     * converts a base-to-body homogeneous matrix M back into a spatial transform X(E, r)
     * avoids arb::inverse() by constructing matrix analytically
     */
    inline constexpr BTransform 
    fromHomogeneousInverse( const BMatrix4 &M ) 
    {
        const BMatrix3 E = glm::mat3(M);             // base-to-body rotation
        const BVector3 t = glm::dvec3(M[3]);         // translation component (-E * r)
        const BVector3 r = -(glm::transpose(E) * t); // recover base-to-body position
        return BTransform(E, r);
    }
    
}

#endif


