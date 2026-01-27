
/* BAdjoint 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Adjoints are derrived from a BTransform X - they map between world and body coords.
 See pages 55, 56 "A Mathematical Introduction to Robotic Manipulation",  
 Murray, R.M., Li, Z., Shankar Sastry S. 1994. 
 Note Murray et al. uses (lin, ang) vector representation.
 
 
 The adjoint of a spatial transform X(E,r), denoted Ad_X maps body velocity 
 coordinates to world/spatial velocity coordinates.
 
 The arb::toAdjoint() presented here has been tested against and matches the Eigen3 RBDL function:
 
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

 The remaining adjoints are constructed from this basic definition.
 
 The adjoint matricies are: 
 
 Ad_X       =  | E   -rx * E |
               | 0       E   |
 
 Ad_X^{-1}  =  | E^T   E^T * rx |
               | 0       E^T    |
 
 Ad_X^T     =  |    E^T      0  |
               | E^T * rx   E^T |
 
 Ad_X^{-T}  =  |    E      0 |
               | -rx * E   E |
 
 where rx = arb::cross(r)
 
 Notes: 
 
 1) Ad_X^{-T} == Ad_X^*
 2) Ad_X ==  X^{-T}  (compare BTransform.h)
 3) AdjointDual <=> AdjointInverseTranspose
 
*/


#ifndef __BADJOINT_H__
#define __BADJOINT_H__


#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif


namespace arb
{

    inline BMatrix6 
    toAdjoint( const BTransform &X )  
    // Ad_X - forward motion
    {
        const BMatrix3 Erx = arb::cross(-X.r()) * X.E();
        return BMatrix6( X.E(), Erx, B_ZERO_3x3, X.E() );
    }

    inline BVector6
    applyAdjoint(const BTransform &X, const BVector6 &f)   
    // Ad_X - forward motion
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * f.lin();
        const BVector3 ang = ET * (f.ang() - arb::cross(X.r(), f.lin()));
        return BVector6( ang,  lin );
    }


    inline BMatrix6 
    toAdjointTranspose( const BTransform &X )  
    // Ad_X^T - forward force
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, B_ZERO_3x3,  ET * arb::cross(X.r()), ET );
    }

    inline BVector6 
    applyAdjointTranspose( const BTransform &X, const BVector6 &f ) 
    // Ad_X^T - forward force
    {
        const BVector3 lin  = arb::cross(-X.r()) *  X.E() * f.ang() + X.E() * f.lin();
        const BVector3 ang = X.E() * f.ang();
        return BVector6(ang, lin);
    }


    inline BMatrix6 
    toAdjointInverse( const BTransform &X )  
    // Ad_X^{-1} - inverse motion
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, ET * arb::cross(X.r()), B_ZERO_3x3 , ET );
    }

    inline BVector6 
    applyAdjointInverse( const BTransform &X, const BVector6 &v ) 
    // Ad_X^{-1} - inverse motion
    {
        const BVector3 lin  = X.E() * v.lin();
        const BVector3 ang = (X.E() * v.ang()) + (arb::cross(-X.r()) * X.E() * v.lin());
        return BVector6(ang, lin);
    }


    inline BMatrix6 
    toAdjointDual( const BTransform &X )  
    // Ad_X^{-T} or Ad_X^* - inverse force
    {
        return BMatrix6( X.E(), B_ZERO_3x3,  arb::cross(-X.r()) * X.E() , X.E() );
    }

    inline BVector6 
    applyAdjointDual( const BTransform &X, const BVector6 &f )  
    // Ad_X^{-T} or Ad_X^* - inverse force
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * (f.lin() - arb::cross(X.r(), f.ang()));
        const BVector3 ang = ET * f.ang();
        return BVector6(ang, lin);
    }

}

#endif
