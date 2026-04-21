
/* BAdjoint 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Adjoints are derrived from BTransforms X - they map between base (parent) and body (child) coords.
 
 
 The arb::toMotionMatrix() presented here has been tested against and matches the Eigen3 RBDL function
 
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

 This function follows the RBDL/Featherstone convention, where the primary 
 transform is 'Force-centric'.
 
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
 
 
 For more background see "A Mathematical Introduction to Robotic Manipulation",  
 Murray, R.M., Li, Z., Shankar Sastry S. 1994. 
 Note we employ (ang,lin) vectors (the Feathersone and RDBL convention) 
 while Murray et al. uses a (lin, ang) vector representation.
 
 
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
    toMotionMatrix( const BTransform &X )  
    // transforms motion vectors (twists/velocities) - motion_body = X * motion_base
    {
        const BMatrix3 Erx = arb::cross(-X.r()) * X.E();
        return BMatrix6( X.E(), Erx, B_ZERO_3x3, X.E() );
    }

    inline BVector6
    applyMotionMatrix(const BTransform &X, const BVector6 &f)   
    // apply motion vector transform (twists/velocities)
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * f.lin();
        const BVector3 ang = ET * (f.ang() - arb::cross(X.r(), f.lin()));
        return BVector6( ang,  lin );
    }


    inline BMatrix6 
    toForceMatrix( const BTransform &X )  
    // transforms force vectors (wrenches) - force_base = X^T * force_body
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, B_ZERO_3x3,  ET * arb::cross(X.r()), ET );
    }

    inline BVector6 
    applyForceMatrix( const BTransform &X, const BVector6 &f ) 
    // apply force vector transform (wrenches)
    {
        const BVector3 lin  = arb::cross(-X.r()) *  X.E() * f.ang() + X.E() * f.lin();
        const BVector3 ang = X.E() * f.ang();
        return BVector6(ang, lin);
    }


    inline BMatrix6 
    toMotionMatrixInv( const BTransform &X )  
    // inverse motion trransform - motion_base = X^{-1} * motion_body
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, ET * arb::cross(X.r()), B_ZERO_3x3 , ET );
    }

    inline BVector6 
    applyMotionMatrixInv( const BTransform &X, const BVector6 &v ) 
    // apply inverse motion trransform
    {
        const BVector3 lin  = X.E() * v.lin();
        const BVector3 ang = (X.E() * v.ang()) + (arb::cross(-X.r()) * X.E() * v.lin());
        return BVector6(ang, lin);
    }


    inline BMatrix6 
    toForceMatrixInv( const BTransform &X )  
    // inverse force transform - force_body = X^{-T} * force_base
    {
        return BMatrix6( X.E(), B_ZERO_3x3,  arb::cross(-X.r()) * X.E() , X.E() );
    }

    inline BVector6 
    applyForceMatrixInv( const BTransform &X, const BVector6 &f )  
    // apply inverse force transform 
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * (f.lin() - arb::cross(X.r(), f.ang()));
        const BVector3 ang = ET * f.ang();
        return BVector6(ang, lin);
    }

}

#endif

