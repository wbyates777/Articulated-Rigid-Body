/* BTransform 20/02/2024

 $$$$$$$$$$$$$$$$$$$$
 $   BTransform.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial or Plücker Transforms (see Featherstone, RBDA, Section 2.8, page 20, and Appendix A3, page 245).
 
 Consider a tranform $X$ that maps points from a coordinate frame $A$ to a coordinate frame $B$ 
 $$ B^X_A : A \rightarrow B. $$
 The inverse $X^{-1}$ is defined by
 $$ B^{X^{-1}}_A = A^X_B. $$ 

 We require diﬀerent coordinate transformation matrices for motions and forces. 
 For motion vectors $v \in M^6$, 
 $$ B^{X}_A : A \rightarrow B $$
 For force vectors $v \in F^6$, the dual or 'star' version of the transform must be used 
 $$ B^{X^*}_A : A \rightarrow B $$
 If the matrix $X$ performs a coordinate transformation on motion vectors, and $X^*$ performs the same
 transformation on force vectors, then the two are related by $X^* = X^{-T}$ 
 (see Featherstone, RBDA, Section 2.6, eqn 2.13, page 18).

 This class employs a compact representation of spatial transformations.
 Instead of using a full 6x6 matrix, the class only stores a 3x3  rotation matrix $E$
 and a 3D translation vector $r$ to represent a spatial transformation. 
 It also provides some efficient matrix operations such as matrix concatenation 
 and transformation of spatial vectors.
 
 
 Notes:
 
 1) All angles in radians
 2) BMatrix3 E is a rotation  - the transpose of a rotation E^T is also its inverse E^{-1}
 3) BVector3 r is a translation - inverse translation is -r
 4) Note X^* = X^{-T} (RBDA, eqn 2.13)

 5) X(E,r) in block 6x6 matrix form:
 
 |     E      0  |
 | -rx * E    E  |
 
 where rx = arb::cross(r)
 
 6) The remaining transforms have analytic block definitions (which differ from 
   the usual matrix definitions) of inverse and transpose (see RBDA, page 22).
 
 X^{-1}  =  |    E^T      0  |
            | E^T * rx   E^T |
 
 X^T     =  | E^T    E^T * rx |
            |  0         E^T  |
 
 X^*  =     | E  -rx * E |
            | 0      E   |
 
 Note  X^* = X^{-T} and  X^* == Ad_X (compare BAdjoint.h)
 
 7) Transform of inertia see RBDA, eqn 2.66, page 34.
    When when transforming a spatial inertia I by a spatial transform X note:
    X.apply(I) returns I' = X^* I X^{-1} 
    X.applyTranspose(I) returns I' = X^T I X  
 
 See also:
 
 https://en.wikipedia.org/wiki/Rotation_matrix
 https://en.wikipedia.org/wiki/Frame_of_reference
 
*/



#ifndef __BTRANSFORM_H__
#define __BTRANSFORM_H__


#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif

#ifndef __BABINERTIA_H__
#include "BABInertia.h"
#endif

class BTransform
{
    
public:
    
    BTransform( void )=default;
    
    // BTransform = Xrot(glm::radians(theta), axis) * Xtrans(trans); // translate then rotate
    constexpr BTransform( const BMatrix3 &rot, const BVector3 &trans = B_ZERO_3 ): m_E(rot), m_r(trans) {}
    constexpr BTransform( const BVector3 &trans ): m_E(B_IDENTITY_3x3), m_r(trans) {}
    
    ~BTransform( void )=default;
    
    void
    clear( void ) {  m_E = B_IDENTITY_3x3; m_r = B_ZERO_3; }

    operator BMatrix6( void ) const 
    {
        return BMatrix6( m_E, B_ZERO_3x3, arb::cross(-m_r) * m_E, m_E );
    }
    
    
    void
    set( const BMatrix3 &E, const BVector3 &r = B_ZERO_3 ) 
    {
        m_E = E; m_r = r;
    }
    
    void
    set( const BVector3 &r ) 
    {
        m_E = B_IDENTITY_3x3; m_r = r;
    }
    
    // rotation, preserves coordinate frame origin
    const BMatrix3&
    E( void ) const { return m_E; }

    BMatrix3&
    E( void ) { return m_E; }

    void
    E( const BMatrix3 &m ) { m_E = m; }
    
    // translation, moves coordinate frame origin
    const BVector3&
    r( void ) const { return m_r; }

    BVector3&
    r( void ) { return m_r; }
    
    void
    r( const BVector3 &v ) { m_r = v; }
    
 
    BTransform 
    operator*( const BTransform &rhs ) const 
    {
        return BTransform(rhs.m_E * m_E, rhs.m_r + (rhs.m_E * m_r));
    }
    
    BTransform& 
    operator*=( const BTransform &rhs ) 
    {
        m_E *= rhs.m_E;
        m_r = rhs.m_r + (rhs.m_E * m_r);
        return *this;
    }
    
    BMatrix6 
    operator*( const BMatrix6 &rhs ) const 
    {
        return BMatrix6(*this) * rhs;
    }
    
    // In RBDL called SpatialRigidBodyInertia::apply(const SpatialVector &v_sp)
    BVector6 
    operator*( const BVector6 &v ) const 
    // return X * v
    {
        // v_rxw = v.lin() - arb::cross(m_r, v.ang())
        const BVector3 v_rxw( v[3] - m_r[1] * v[2] + m_r[2] * v[1],
                              v[4] - m_r[2] * v[0] + m_r[0] * v[2],
                              v[5] - m_r[0] * v[1] + m_r[1] * v[0] );
        
        return BVector6(m_E[0][0] * v[0]  + m_E[0][1] * v[1]  + m_E[0][2] * v[2],
                        m_E[1][0] * v[0]  + m_E[1][1] * v[1]  + m_E[1][2] * v[2],
                        m_E[2][0] * v[0]  + m_E[2][1] * v[1]  + m_E[2][2] * v[2],
                      
                        m_E[0][0] * v_rxw[0] + m_E[0][1] * v_rxw[1] + m_E[0][2] * v_rxw[2],
                        m_E[1][0] * v_rxw[0] + m_E[1][1] * v_rxw[1] + m_E[1][2] * v_rxw[2],
                        m_E[2][0] * v_rxw[0] + m_E[2][1] * v_rxw[1] + m_E[2][2] * v_rxw[2] );
        
        // const BMatrix3 ET = arb::transpose(m_E);
        // const BVector3 ang = ET * v.ang();
        // const BVector3 lin = ET * v_rxw;
        // return BVector6(ang, lin);
    }

    BVector6 
    apply( const BVector6 &v ) const { return operator*(v); }

    BVector6 
    applyTranspose( const BVector6 &f ) const
    // returns X^T * f 
    {
        const BVector3 ETf( m_E[0][0] * f[3] + m_E[1][0] * f[4] + m_E[2][0] * f[5],
                            m_E[0][1] * f[3] + m_E[1][1] * f[4] + m_E[2][1] * f[5],
                            m_E[0][2] * f[3] + m_E[1][2] * f[4] + m_E[2][2] * f[5] );
        
        return BVector6( m_E[0][0] * f[0] + m_E[1][0] * f[1] + m_E[2][0] * f[2] - m_r[2] * ETf[1] + m_r[1] * ETf[2],
                         m_E[0][1] * f[0] + m_E[1][1] * f[1] + m_E[2][1] * f[2] + m_r[2] * ETf[0] - m_r[0] * ETf[2],
                         m_E[0][2] * f[0] + m_E[1][2] * f[1] + m_E[2][2] * f[2] - m_r[1] * ETf[0] + m_r[0] * ETf[1],
                         ETf[0],
                         ETf[1],
                         ETf[2] );
        
        // const BVector3 ETf = m_E * f.lin();
        // const BVector3 aux = m_E * f.ang();
        // return BVector6(aux[0] - m_r[2] * ETf[1] + m_r[1] * ETf[2],
        //                 aux[1] + m_r[2] * ETf[0] - m_r[0] * ETf[2],
        //                 aux[2] - m_r[1] * ETf[0] + m_r[0] * ETf[1],
        //                 ETf[0],
        //                 ETf[1],
        //                 ETf[2]);
    }

    // transform 3D point p to/from coordinate frame 
    BVector3 
    apply( const BVector3 &p ) const { return m_E * (p - m_r); }
    
    BVector3 
    applyTranspose( const BVector3 &p ) const { return m_r + (arb::transpose(m_E) * p); }

    
    BRBInertia 
    apply( const BRBInertia &rbi ) const
    // returns  X^* I X^{-1}
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 rx = arb::cross(m_r);
        const BVector3 h  = ET * (rbi.h() - (rbi.mass() * m_r)); 
        const BMatrix3 aux = (rx * arb::cross(rbi.h())) + ((arb::cross(rbi.h() - rbi.mass() * m_r) * rx));
        const BMatrix3 I =  ET * (rbi.I() + aux) * m_E;
        return BRBInertia( rbi.mass(), h, I );
    }
    
    BRBInertia 
    applyTranspose( const BRBInertia &rbi ) const 
    // returns X^T I X 
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 rx = arb::cross(m_r);
        const BVector3 h = (m_E * rbi.h()) + (rbi.mass() * m_r); 
        const BMatrix3 aux = rx * arb::cross(m_E * rbi.h()) + arb::cross(h) * rx;
        const BMatrix3 I = (m_E * rbi.I() * ET) - aux;
        return BRBInertia( rbi.mass(), h, I );
    }
    
    
    BABInertia 
    apply( const BABInertia &abi ) const  
    // returns  X^* I X^{-1} 
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 rx = arb::cross(m_r);
        const BMatrix3 H = abi.H() - (rx * abi.M());
        const BMatrix3 I = abi.I() - (rx * arb::transpose(abi.H()) ) + (H * rx);
        return BABInertia( ET * abi.M() * m_E,  ET * H * m_E,  ET * I * m_E );
    }
    
    BABInertia
    applyTranspose( const BABInertia &abi ) const   
    // returns X^T I X
    {
        const BMatrix3 ET = arb::transpose(m_E);
        const BMatrix3 rx = arb::cross(m_r);
        const BMatrix3 M = m_E * abi.M() * ET;
        const BMatrix3 H = m_E * abi.H() * ET;
        const BMatrix3 rxM = (rx * M);
        const BMatrix3 I = (m_E * abi.I() * ET) - (rx * H) + (arb::transpose(H) - rxM) * rx; 
        return BABInertia( M, H + -arb::transpose(rxM), I ); 
    }
    
    bool 
    operator==( const BTransform &v ) const { return (m_r == v.m_r) && (m_E == v.m_E); }
    
    bool 
    operator!=( const BTransform &v ) const { return (m_r != v.m_r) || (m_E != v.m_E); }
    
    friend std::istream& 
    operator>>( std::istream &istr, BTransform &m );
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BTransform &m );
    
private:
    
    BMatrix3 m_E; // rotation - note the transpose of a rotation is also its inverse
    BVector3 m_r; // translation - inverse translation is -m_r
};

 
#ifndef GLM_FORCE_INTRINSICS
constexpr BTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#else
const BTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#endif


//
// Articulated Rigid Body
//

namespace arb
{
    // X^T - style choice - I prefer arb::transpose(m) to m.transpose()  
    inline constexpr BMatrix6 
    transpose( const BTransform &m ) 
    { 
        const BMatrix3 ET = arb::transpose(m.E());
        return BMatrix6( ET, ET * arb::cross(m.r()), B_ZERO_3x3, ET );
    }

    // X^{-1}
    inline constexpr BTransform 
    inverse( const BTransform &m )  
    { 
        const BMatrix3 ET = arb::transpose(m.E()); 
        return BTransform(ET, -ET * m.r()); 
    }

    // dual is $X^* = X^{-T}$ 
    inline constexpr BMatrix6 
    dual( const BTransform &m ) 
    { 
        return BMatrix6( m.E(), arb::cross(-m.r()) * m.E(), B_ZERO_3x3, m.E() );
    }

    // all angles in radians
    inline BTransform 
    Xrot( BScalar angle, const BVector3 &axis ) // WARNING - axis *must* be normalized
    {
        const BScalar s = std::sin(angle);
        const BScalar c = std::cos(angle);
  
        return BTransform(BMatrix3( axis[0] * axis[0] * (1.0 - c) + c,
                                    axis[1] * axis[0] * (1.0 - c) + axis[2] * s,
                                    axis[0] * axis[2] * (1.0 - c) - axis[1] * s,
                                  
                                    axis[0] * axis[1] * (1.0 - c) - axis[2] * s,
                                    axis[1] * axis[1] * (1.0 - c) + c,
                                    axis[1] * axis[2] * (1.0 - c) + axis[0] * s,
                                  
                                    axis[0] * axis[2] * (1.0 - c) + axis[1] * s,
                                    axis[1] * axis[2] * (1.0 - c) - axis[0] * s,
                                    axis[2] * axis[2] * (1.0 - c) + c) );
    }

    // RBDA, Section 2.8, table, 2.2, page 23
    inline BTransform 
    Xrotx( BScalar xrot ) 
    {
        const BScalar s = std::sin(xrot);
        const BScalar c = std::cos(xrot);
        return BTransform ( BMatrix3( 1.0, 0.0, 0.0,
                                      0.0,   c,   s,
                                      0.0,  -s,   c ) );
    }

    inline BTransform 
    Xroty( BScalar yrot ) 
    {
        const BScalar s = std::sin(yrot);
        const BScalar c = std::cos(yrot);
        return BTransform( BMatrix3(   c, 0.0,  -s,
                                     0.0, 1.0, 0.0,
                                       s, 0.0,   c ) ); 
    }

    inline BTransform 
    Xrotz( BScalar zrot ) 
    {
        const BScalar s = std::sin(zrot);
        const BScalar c = std::cos(zrot);
        return BTransform( BMatrix3(  c,   s, 0.0,
                                     -s,   c, 0.0,
                                    0.0, 0.0, 1.0 ) );
    }

    inline BTransform 
    Xtrans( const BVector3 &r ) 
    {
        return BTransform( B_IDENTITY_3x3, r ); 
    }
}


inline std::ostream&
operator<<( std::ostream &ostr, const BTransform &m )
{
    ostr << m.m_E << ' ' << m.m_r  << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BTransform &m )
{
    istr >> m.m_E >> m.m_r;
    return istr;
}


#endif


