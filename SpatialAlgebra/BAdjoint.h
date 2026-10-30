/*!
 @file BAdjoint.h
 @date 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Converts compact BTransforms $X(E,r)$ into 6x6 spatial transformation matrices.

 The function arb::toForce() presented here has been tested against and matches 
 the Eigen3 RBDL function:
 
 @code{.cpp}
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
 @endcode
 
 
 The remaining matricies are constructed from this basic definition.
 
 The matricies are: 
 
 X : A -> B
 toMotion(X)         =  |    E      0 |
                        | -rx * E   E |
 
 X^{-1} : B -> A 
 toMotionInverse(X)  =  |    E^T      0  |
                        | E^T * rx   E^T |
 
 X^{T} : A -> B 
 toForce(X)          =  | E   -rx * E |
                        | 0       E   |
 
 X^{-T} : B -> A 
 toForceInverse(X)   =  | E^T   E^T * rx |
                        | 0        E^T   |
 
 where rx = arb::cross(r)
 
*/


#ifndef BADJOINT_H
#define BADJOINT_H


#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif


namespace arb
{
    
    inline BMatrix6 
    toMotion( const BTransform &X )  
    // matches Featherstone's spatial motion transform X
    // transforms a twist from parent to child frame (forward pass)
    {
        return BMatrix6( X.E(), B_ZERO_3x3,  arb::cross(-X.r()) * X.E(), X.E() );
    }

    inline BVector6 
    applyMotion( const BTransform &X, const BVector6 &v )  
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * (v.lin() - arb::cross(X.r(), v.ang()));
        const BVector3 ang = ET * v.ang();
        return BVector6(ang, lin);
    }

    
    inline BMatrix6 
    toMotionInverse( const BTransform &X )  
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, B_ZERO_3x3,  ET * arb::cross(X.r()), ET );
    }

    inline BVector6 
    applyMotionInverse( const BTransform &X, const BVector6 &v ) 
    {
        const BVector3 lin  = arb::cross(-X.r()) *  X.E() * v.ang() + X.E() * v.lin();
        const BVector3 ang = X.E() * v.ang();
        return BVector6(ang, lin);
    }

    //
    //
    //
    
    inline BMatrix6 
    toForce( const BTransform &X )  
    // matches Featherstone's spatial force transform X^{T} 
    // transforms a wrench from parent to child frame
    {
        const BMatrix3 Erx = arb::cross(-X.r()) * X.E();
        return BMatrix6( X.E(), Erx, B_ZERO_3x3, X.E() );
    }

    inline BVector6
    applyForce(const BTransform &X, const BVector6 &v)   
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * v.lin();
        const BVector3 ang = ET * (v.ang() - arb::cross(X.r(), v.lin()));
        return BVector6(ang,  lin);
    }

    
    inline BMatrix6 
    toForceInverse( const BTransform &X )  
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, ET * arb::cross(X.r()), B_ZERO_3x3, ET );
    }

    inline BVector6 
    applyForceInverse( const BTransform &X, const BVector6 &v ) 
    {
        const BVector3 lin  = X.E() * v.lin();
        const BVector3 ang = (X.E() * v.ang()) + (arb::cross(-X.r()) * X.E() * v.lin());
        return BVector6(ang, lin);
    }

}


#endif

