
/* BAdjoint 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Adjoints are derrived from a BSpatialTransform X - they map between world and body coords.
 
 The adjoint of X, denoted Ad_X maps body velocity coordinates to world velocity coordinates.
 
 arb::toAdjoint() has been tested against the RBDL function SpatialTransform::toMatrixAdjoint() (using Eigen3)
 
 SpatialMatrix toMatrixAdjoint () const {
   Matrix3d _Erx =
     E * Matrix3d (
         0., -r[2], r[1],
         r[2], 0., -r[0],
         -r[1], r[0], 0.
         );
   SpatialMatrix result;
   result.block<3,3>(0,0) = E;
   result.block<3,3>(0,3) = -_Erx;
   result.block<3,3>(3,0) = Matrix3d::Zero();
   result.block<3,3>(3,3) = E;

   return result;
 }

 The remaining adjoint functions are constructed from this basic definition.
 
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
