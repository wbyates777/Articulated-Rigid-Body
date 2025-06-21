/* BCrossProducts 11/06/2025

 $$$$$$$$$$$$$$$$$$$$$$$$
 $   BCrossProducts.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Spatial Cross Products (see Featherstone, RBDA, Section 2.9, page 23). 

 
 All cross product operators in one handy file - 
 These get used a lot -- good candidate for SIMD optimisation

*/

#ifndef __BCROSSPRODUCTS_H__
#define __BCROSSPRODUCTS_H__

#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif


namespace arb
{

    // standard Euclidian cross product - replaces glm::cross 
    //inline const BVector3
    //cross( const BVector3 &x, const BVector3 &y )
    //{
    //    return BVector3( x[1] * y[2] - y[1] * x[2],
    //                     x[2] * y[0] - y[2] * x[0],
    //                     x[0] * y[1] - y[0] * x[1] );
    //}
  

    // The Euclidian cross product can be written as an equivalent matrix multiplication
    // The operator $v\times$ (pronounced ‘v-cross’) constructs a skew-symmetric matrix $m$ from vector $v$  
    // such that $m * a = v \times a$, see RBDA, Section 2.8, eqn 2.23, page 21
    // note arb::cross(-v) = arb::transpose(arb::cross(v))
    // implemented in RDBL as VectorCrossMatrix in SpatialAlgebraOperators.h
    // see https://en.wikipedia.org/wiki/Skew-symmetric_matrix
    inline const BMatrix3 
    cross( const BVector3 &v ) 
    {
        return BMatrix3(  0.0, -v[2],  v[1],
                         v[2],   0.0, -v[0],
                        -v[1],  v[0],   0.0 );
    }
 
  
    // spatial cross product for 'motion cross motion' vectors $v$ and $m$
    // such that $\dot{m} = v \times m$, see RBDA, Section 2.9, eqn 2.29, page 23
    inline const BSpatialVector 
    crossm( const BSpatialVector &v, const BSpatialVector &m ) 
    {
        return BSpatialVector( -v[2] * m[1] + v[1] * m[2],
                                v[2] * m[0] - v[0] * m[2],
                               -v[1] * m[0] + v[0] * m[1],
                               -v[5] * m[1] + v[4] * m[2] - v[2] * m[4] + v[1] * m[5],
                                v[5] * m[0] - v[3] * m[2] + v[2] * m[3] - v[0] * m[5],
                               -v[4] * m[0] + v[3] * m[1] - v[1] * m[3] + v[0] * m[4] );
    }

    // spatial cross product for 'motion cross force' vectors $v$ and $f$
    // such that $\dot{f} = v \times^* f$, see RBDA, Section 2.9, eqn 2.30, page 23
    inline const BSpatialVector 
    crossf( const BSpatialVector &v, const BSpatialVector &f ) 
    {
        return BSpatialVector( -v[2] * f[1] + v[1] * f[2] - v[5] * f[4] + v[4] * f[5],
                                v[2] * f[0] - v[0] * f[2] + v[5] * f[3] - v[3] * f[5],
                               -v[1] * f[0] + v[0] * f[1] - v[4] * f[3] + v[3] * f[4],
                               -v[2] * f[4] + v[1] * f[5],
                                v[2] * f[3] - v[0] * f[5],
                               -v[1] * f[3] + v[0] * f[4] );
    }


    // motion vector operator $v\times$, see RBDA, Section 2.9, eqn 2.31, page 25
    inline const BSpatialMatrix 
    crossm( const BSpatialVector &v ) 
    {
        //return BSpatialMatrix(arb::cross(v.ang()), B_ZERO_3x3, arb::cross(v.lin()), arb::cross(v.ang()));
        return BSpatialMatrix( 0.0,  -v[2],   v[1],   0.0,   0.0,    0.0, 
                              v[2],    0.0,  -v[0],   0.0,   0.0,    0.0, 
                             -v[1],   v[0],    0.0,   0.0,   0.0,    0.0,        
                               0.0,  -v[5],   v[4],   0.0, -v[2],   v[1],
                              v[5],    0.0,  -v[3],   v[2],  0.0,  -v[0],   
                             -v[4],   v[3],    0.0,  -v[1], v[0],    0.0 );
    }

    // force vector operator $v\times^*$, see RBDA, Section 2.9, eqn 2.32, page 25
    inline const BSpatialMatrix 
    crossf( const BSpatialVector &v ) 
    {
        //return BSpatialMatrix(arb::cross(v.ang()), arb::cross(v.lin()), B_ZERO_3x3, arb::cross(v.ang()));
        return BSpatialMatrix( 0.0, -v[2],  v[1],    0.0,  -v[5],   v[4],
                              v[2],   0.0, -v[0],   v[5],    0.0,  -v[3],
                             -v[1],  v[0],   0.0,  -v[4],   v[3],    0.0, 
                               0.0,   0.0,   0.0,    0.0,  -v[2],   v[1],
                               0.0,   0.0,   0.0,   v[2],    0.0,  -v[0],
                               0.0,   0.0,   0.0,  -v[1],   v[0],    0.0 );
    }


};


#endif


