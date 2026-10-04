/* BProducts 20/02/2026

 $$$$$$$$$$$$$$$$$$$
 $   BProducts.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Spatial Cross Products (see Featherstone, RBDA, Section 2.9, page 23). 

 
 All cross product operators in one handy file 
 These get used a lot -- good candidate for SIMD optimisation

 
 The operator arb::cross (which constructs a skew-symmetric matrix $v\times$)
 is implemented in RDBL as VectorCrossMatrix in SpatialAlgebraOperators.h
 See Tensor section  in https://en.wikipedia.org/wiki/Angular_velocity
 
*/


#ifndef BPRODUCTS_H
#define BPRODUCTS_H


#ifndef BMATRIX6_H
#include "BMatrix6.h"
#endif


namespace arb
{
    
    // standard dot or inner product a·b 
    inline BScalar 
    dot( const BVector3 &v1, const BVector3 &v2 ) 
    {
         return glm::dot(v1, v2);
    }
    
    // spatial dot or inner product a·b
    inline BScalar 
    dot( const BVector6 &v1, const BVector6 &v2 )
    {
        return arb::dot(v1.ang(), v2.ang()) + arb::dot(v1.lin(), v2.lin());
    }

    
    // standard Euclidian cross product a⨉b
    inline constexpr BVector3   
    cross( const BVector3 &x, const BVector3 &y )
    {
        return glm::cross(x, y);
    }
    
    // The Euclidian cross product can be written as an equivalent matrix multiplication
    // The operator $v\times$ (pronounced ‘v-cross’) constructs a skew-symmetric matrix $m$ from vector $v$  
    // such that $m * a = v \times a$, (see RBDA, Section 2.8, eqn 2.23, page 21 and table 2.1, page 22).
    // note arb::cross(-v) = arb::transpose(arb::cross(v))
    inline constexpr BMatrix3 
    cross( const BVector3 &v ) 
    {
        return BMatrix3(  0.0, -v[2],  v[1],
                         v[2],   0.0, -v[0],
                        -v[1],  v[0],   0.0 );
    }

    // extracts the 3D vector from a skew-symmetric matrix; the 'vee' operator in Lie algebra
    inline constexpr BVector3 
    uncross( const BMatrix3 &m ) 
    { 
        return BVector3(m[2][1], m[0][2], m[1][0]);
    }
    
    inline BVector6 
    uncross( const BMatrix6 &m ) 
    {
        return BVector6(uncross(m.topLeft()), uncross(m.botLeft()));
    }

    // return arb::cross(v) * arb::cross(-v)
    inline  BMatrix3 
    crosst( const BVector3 &v ) 
    {
        const BScalar xx =  v[0] * v[0];
        const BScalar yy =  v[1] * v[1];
        const BScalar zz =  v[2] * v[2];
        const BScalar xy =  v[0] * v[1];
        const BScalar xz =  v[0] * v[2];
        const BScalar yz =  v[1] * v[2];

        return BMatrix3( yy + zz,    -xy,      -xz,
                           -xy,    xx + zz,    -yz,
                           -xz,      -yz,    xx + yy );

    }
    
    // Euclidian outer product a⨂b
    inline constexpr BMatrix3  
    outer( const BVector3 &a, const BVector3 &b )
    { 
        return glm::outerProduct(b,a);
    }

    // spatial outer product a⨂b
    inline constexpr BMatrix6
    outer( const BVector6 &a, const BVector6 &b )
    { 
        return BMatrix6( arb::outer(a.ang(), b.ang()), arb::outer(a.ang(), b.lin()),
                         arb::outer(a.lin(), b.ang()), arb::outer(a.lin(), b.lin()) );
    }

   
    // spatial cross product for 'motion cross motion' vectors $v$ and $m$
    // such that $\dot{m} = v \times m$, see RBDA, Section 2.9, eqn 2.29, page 23
    inline constexpr BVector6 
    crossm( const BVector6 &v, const BVector6 &m ) 
    {
        const BVector3 v_ang = v.ang(); 
        const BVector3 v_lin = v.lin(); 
        const BVector3 m_ang = m.ang(); 
        const BVector3 m_lin = m.lin(); 
        
        return BVector6(arb::cross(v_ang, m_ang), arb::cross(v_ang, m_lin) + arb::cross(v_lin, m_ang));
    }

    // spatial cross product for 'motion cross force' vectors $v$ and $f$
    // such that $\dot{f} = v \times^* f$, see RBDA, Section 2.9, eqn 2.30, page 23
    inline constexpr BVector6 
    crossf( const BVector6 &v, const BVector6 &f ) 
    {
         const BVector3 v_ang = v.ang(); 
         const BVector3 v_lin = v.lin(); 
         const BVector3 f_ang = f.ang(); 
         const BVector3 f_lin = f.lin(); 

         return BVector6(arb::cross(v_ang, f_ang) + arb::cross(v_lin, f_lin), arb::cross(v_ang, f_lin));
    }


    // motion vector operator $v\times$, see RBDA, Section 2.9, eqn 2.31, page 25
    inline constexpr BMatrix6 
    crossm( const BVector6 &v ) 
    {
         return BMatrix6(arb::cross(v.ang()), B_ZERO_3x3, arb::cross(v.lin()), arb::cross(v.ang()));
    }

    // force vector operator $v\times^*$, see RBDA, Section 2.9, eqn 2.32, page 25
    inline constexpr BMatrix6 
    crossf( const BVector6 &v ) 
    {
        return BMatrix6(arb::cross(v.ang()), arb::cross(v.lin()), B_ZERO_3x3, arb::cross(v.ang()));
    }

}


#endif



