
/* BAdjoint 19/11/2025

 $$$$$$$$$$$$$$$$$$
 $   BAdjoint.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Converts compact BTransforms into 6x6 spatial transformation matrices.


 The function arb::toForce() presented here has been tested against and matches 
 the Eigen3 RBDL function:
 
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

 
 
 The remaining matricies are constructed from this basic definition.
 
 The matricies are: 
 
 toMotion(X)        =  | E   -rx * E |
  X                    | 0       E   |
 
 toMotionInverse(X) =  |    E^T      0  |
  X^{-1}               | E^T * rx   E^T |
 
 toForce(X)         =  |    E      0 |
  X^{-T}               | -rx * E   E |
 
 toForceInverse(X)  =  | E^T   E^T * rx |
  X^{-1}               | 0        E^T   |
 
 where rx = arb::cross(r)
 
*/


#ifndef __BADJOINT_H__
#define __BADJOINT_H__


#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif


namespace arb
{
    
    inline BMatrix6 
    toMotion( const BTransform &X )  
    // motion A -> B
    // matches Featherstone's X (spatial motion transform)
    // use: transform twist from parent to child (forward pass)
    {
        return BMatrix6( X.E(), B_ZERO_3x3,  arb::cross(-X.r()) * X.E(), X.E() );
    }

    inline BVector6 
    applyMotion( const BTransform &X, const BVector6 &f )  
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * (f.lin() - arb::cross(X.r(), f.ang()));
        const BVector3 ang = ET * f.ang();
        return BVector6(ang, lin);
    }

    
    inline BMatrix6 
    toMotionInverse( const BTransform &X )  
    // motion B -> A
    {
        const BMatrix3 ET = arb::transpose(X.E());
        return BMatrix6( ET, B_ZERO_3x3,  ET * arb::cross(X.r()), ET );
    }

    inline BVector6 
    applyMotionInverse( const BTransform &X, const BVector6 &f ) 
    {
        const BVector3 lin  = arb::cross(-X.r()) *  X.E() * f.ang() + X.E() * f.lin();
        const BVector3 ang = X.E() * f.ang();
        return BVector6(ang, lin);
    }

    //
    //
    //
    
    inline BMatrix6 
    toForce( const BTransform &X )  
    // force A -> B
    // matches Featherstone's X* (spatial force transform)
    // use: transform wrench from parent to child
    {
        const BMatrix3 Erx = arb::cross(-X.r()) * X.E();
        return BMatrix6( X.E(), Erx, B_ZERO_3x3, X.E() );
    }

    inline BVector6
    applyForce(const BTransform &X, const BVector6 &f)   
    {
        const BMatrix3 ET = arb::transpose(X.E());
        const BVector3 lin = ET * f.lin();
        const BVector3 ang = ET * (f.ang() - arb::cross(X.r(), f.lin()));
        return BVector6(ang,  lin);
    }

    
    inline BMatrix6 
    toForceInverse( const BTransform &X )  
    // force B -> A
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

