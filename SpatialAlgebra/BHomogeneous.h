/*!
 @file BHomogeneous.h 
 @date 13/10/2026

 $$$$$$$$$$$$$$$$$$$$$$
 $   BHomogeneous.h   $
 $$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Conversion functions between spatial transforms X and 4x4-homogeneous matricies M

 Given a spatial transform BTransform X and any BVector3 p then
 
       X.applyTranspose(p) == arb::toHomogeneous(X) * BVector4(p, 1.0);
 
 and
 
       X.apply(p) == arb::toHomogeneousInverse(X) * BVector4(p, 1.0);

 Note:
 
 1) M must be a rigid transform (orthonormal rotation, no scale or shear)
 
*/


#ifndef BHOMOGENEOUS_H
#define BHOMOGENEOUS_H

#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

namespace arb
{

    /// converts a body-to-base spatial transform X to a body-to-base homogeneous matrix M 
    inline BMatrix4 
    toHomogeneous( const BTransform &X ) 
    {
        BMatrix4 res(arb::transpose(X.E()));
        res[3] = BVector4(X.r(), 1.0);
        return res;
    }
    
    
    /// converts a body-to-base spatial transform directly to a base-to-body homogeneous matrix M 
    /// avoids arb::inverse() by constructing matrix analytically 
    /// This is Featherstone's pluho - Plücker-to-homogeneous
    inline BMatrix4 
    toHomogeneousInverse( const BTransform &X ) 
    {
        BMatrix4 res(X.E());
        res[3] = BVector4(-(X.E() * X.r()), 1.0);
        return res;
    }
    
    /// converts a body-to-base homogeneous matrix M back into a spatial transform X(E, r) 
    inline BTransform
    fromHomogeneous( const BMatrix4 &M ) 
    {
        const BMatrix3 R = BMatrix3(M);           // rotation: body axes in base coordinates (body-to-base)
        const BVector3 r = BVector3(M[3]);        // body origin expressed in base coordinates
        const BMatrix3 E = arb::transpose(R);     // rotation: base-to-body
        return BTransform(E, r);
    }

    /// converts a base-to-body homogeneous matrix M back into a spatial transform X(E, r)
    /// avoids arb::inverse() by constructing matrix analytically
    inline BTransform 
    fromHomogeneousInverse( const BMatrix4 &M ) 
    {
        const BMatrix3 E = BMatrix3(M);                 // rotation: base-to-body
        const BVector3 t = BVector3(M[3]);              // translation of the base-to-body matrix (-E * r)
        const BVector3 r = -(arb::transpose(E) * t);    // body origin expressed in base coordinates
        return BTransform(E, r);
    }
    
}

#endif


