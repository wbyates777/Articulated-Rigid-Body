
/* BAdjoint 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Adjoints are derrived from a BSpatialTransform X - they map between world and body coords.
 
 Ad_X maps body velocity coordinates to world velocity coordinates.
 
 toAdjoint() tested against RBDL function SpatialTransform::toMatrixAdjoint()
 
 The following definitions are taken from the RBDL code. 
 
 
 Ad_X       =  | E   -E * rx |
               | 0       E   |
 
 Ad_X^{-1}  =  | E^T   rx * E^T |
               | 0       E^T    |
 
 Ad_X^{T}   =  |    E^T      0  |
               | rx * E^T   E^T |
 
 Ad_X^{-T}  =  |    E      0 |
               | -E * rx   E |
 
 where rx = arb::cross(r)
 
 Note AdjointDual <=> AdjointInverseTranspose
 
*/


#ifndef __BADJOINT_H__
#define __BADJOINT_H__


#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif


namespace arb
{

    inline const BSpatialMatrix 
    toAdjoint( const BSpatialTransform &X )  
    // Ad_X - forward motion
    {
        const BMatrix3 Erx = arb::cross(-X.r()) * X.E();
        return BSpatialMatrix( X.E(), Erx, B_ZERO_3x3, X.E() );
    }

    inline const BSpatialVector
    applyAdjoint(const BSpatialTransform &X, const BSpatialVector &f)   
    // Ad_X - forward motion
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * f.lin();
        const BVector3 ang = ET * (f.ang() - arb::cross(X.r(), f.lin()));
        return BSpatialVector( ang,  lin );
    }


    inline const BSpatialMatrix 
    toAdjointTranspose( const BSpatialTransform &X )  
    // Ad_X^{T} - forward force
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BSpatialMatrix( ET, B_ZERO_3x3,  ET * arb::cross(X.r()), ET );
    }

    inline const BSpatialVector 
    applyAdjointTranspose( const BSpatialTransform &X, const BSpatialVector &f ) 
    // Ad_X^{T} - forward force
    {
        const BVector3 lin  = arb::cross(-X.r()) *  X.E() * f.ang() + X.E() * f.lin();
        const BVector3 ang = X.E() * f.ang();
        return BSpatialVector(ang, lin);
    }


    inline const BSpatialMatrix 
    toAdjointInverse( const BSpatialTransform &X )  
    // Ad_X^{-1} - inverse motion
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BSpatialMatrix( ET, ET * arb::cross(X.r()), B_ZERO_3x3 , ET );
    }

    inline const BSpatialVector 
    applyAdjointInverse( const BSpatialTransform &X, const BSpatialVector &v ) 
    // Ad_X^{-1} - inverse motion
    {
        const BVector3 lin  = X.E() * v.lin();
        const BVector3 ang = (X.E() * v.ang()) + (arb::cross(-X.r()) * X.E() * v.lin());
        return BSpatialVector(ang, lin);
    }


    inline const BSpatialMatrix 
    toAdjointDual( const BSpatialTransform &X )  
    // Ad_X^{-T} or Ad_X^* - inverse force
    {
        return BSpatialMatrix( X.E(), B_ZERO_3x3,  arb::cross(-X.r()) * X.E() , X.E() );
    }

    inline const BSpatialVector 
    applyAdjointDual( const BSpatialTransform &X, const BSpatialVector &f )  
    // Ad_X^{-T} or Ad_X^* - inverse force
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * (f.lin() - arb::cross(X.r(), f.ang()));
        const BVector3 ang = ET * f.ang();
        return BSpatialVector(ang, lin);
    }

}

#endif
