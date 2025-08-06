/* BSpatialTransform 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialTransform.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Spatial or Plücker Transforms (see Featherstone, RBDA, Section 2.8, page 20, and Appendix A3, page 245).
 
 Consider a tranform $X$ that maps points from a coordinate frame $A$ to coordinate a frame $B$ 
 $$ B^X_A : A \rightarrow B. $$
 The inverse $X^{-1}$ is defined by
 $$ B^{X^{-1}}_A = A^X_B. $$ 

 We require diﬀerent coordinate transformation matrices for motions and forces. 
 For motion vectors $v \in M^6$, 
 $$ B^{X}_A : A \rightarrow B $$
 For force vectors $v \in F^6$, the 'star' version of the transform must be used 
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
 2) SMatrix3 E is a rotation  - the transpose of a rotation is also its inverse
 3) SVector3 r is a translation - inverse translation is -r
 
 https://en.wikipedia.org/wiki/Rotation_matrix
*/



#ifndef __BSPATIALTRANSFORM_H__
#define __BSPATIALTRANSFORM_H__


#ifndef __BSPATIALINERTIA_H__
#include "BSpatialInertia.h"
#endif

class BSpatialTransform
{
    
public:
    
    BSpatialTransform( void )=default;
    
    // BSpatialTransform = Xrot(glm::radians(theta), axis) * Xtrans(trans);
    constexpr explicit BSpatialTransform( const BMatrix3 &rot, const BVector3 &trans = B_ZERO_3 ): m_E(rot), m_r(trans) {}
    constexpr explicit BSpatialTransform( const BVector3 &trans ): m_E(B_IDENTITY_3x3), m_r(trans) {}
    
    ~BSpatialTransform( void )=default;
    
    void
    clear( void ) {  m_E = B_IDENTITY_3x3; m_r = B_ZERO_3; }

    operator BSpatialMatrix( void ) const 
    {
        const BMatrix3 ETrx =  glm::transpose(-m_E) * arb::cross(m_r);
        return BSpatialMatrix( m_E, B_ZERO_3x3, -glm::transpose(ETrx), m_E );
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
    
 
    const BSpatialTransform 
    operator*( const BSpatialTransform &rhs ) const 
    {
        return BSpatialTransform(rhs.m_E * m_E, rhs.m_r + (rhs.m_E * m_r));
    }
    
    const BSpatialTransform& 
    operator*= (const BSpatialTransform &rhs) 
    {
        m_E *= rhs.m_E;
        m_r = rhs.m_r + (rhs.m_E * m_r);
        return *this;
    }
    
    // In RBDL called SpatialRigidBodyInertia::apply(const SpatialVector &v_sp)
    const BSpatialVector 
    operator*( const BSpatialVector &v ) const 
    // return X * v
    {
        // v_rxw = v.lin() - arb::cross(m_r, v.ang())
        const BVector3 v_rxw( v[3] - m_r[1] * v[2] + m_r[2] * v[1],
                              v[4] - m_r[2] * v[0] + m_r[0] * v[2],
                              v[5] - m_r[0] * v[1] + m_r[1] * v[0] );
        
        return BSpatialVector(m_E[0][0] * v[0]  + m_E[0][1] * v[1]  + m_E[0][2] * v[2],
                              m_E[1][0] * v[0]  + m_E[1][1] * v[1]  + m_E[1][2] * v[2],
                              m_E[2][0] * v[0]  + m_E[2][1] * v[1]  + m_E[2][2] * v[2],
                              
                              m_E[0][0] * v_rxw[0] + m_E[0][1] * v_rxw[1] + m_E[0][2] * v_rxw[2],
                              m_E[1][0] * v_rxw[0] + m_E[1][1] * v_rxw[1] + m_E[1][2] * v_rxw[2],
                              m_E[2][0] * v_rxw[0] + m_E[2][1] * v_rxw[1] + m_E[2][2] * v_rxw[2] );
        
        
        // const BMatrix3 ET = arb::transpose(m_E);
        // const BVector3 ang = ET * v.ang();
        // const BVector3 lin = ET * v_rxw;
        // return BSpatialVector(ang, lin);
    }
    
    const BSpatialVector 
    applyTranspose( const BSpatialVector &f ) const
    // returns X^T * f 
    {
        const BVector3 ETf( m_E[0][0] * f[3] + m_E[1][0] * f[4] + m_E[2][0] * f[5],
                            m_E[0][1] * f[3] + m_E[1][1] * f[4] + m_E[2][1] * f[5],
                            m_E[0][2] * f[3] + m_E[1][2] * f[4] + m_E[2][2] * f[5]);
        
        return BSpatialVector(m_E[0][0] * f[0] + m_E[1][0] * f[1] + m_E[2][0] * f[2] - m_r[2] * ETf[1] + m_r[1] * ETf[2],
                              m_E[0][1] * f[0] + m_E[1][1] * f[1] + m_E[2][1] * f[2] + m_r[2] * ETf[0] - m_r[0] * ETf[2],
                              m_E[0][2] * f[0] + m_E[1][2] * f[1] + m_E[2][2] * f[2] - m_r[1] * ETf[0] + m_r[0] * ETf[1],
                              ETf[0],
                              ETf[1],
                              ETf[2] );
        
        // const BVector3 ETf = m_E * f.lin();
        // const BVector3 aux = m_E * f.ang();
        // return BSpatialVector(aux[0] - m_r[2] * ETf[1] + m_r[1] * ETf[2],
        //                       aux[1] + m_r[2] * ETf[0] - m_r[0] * ETf[2],
        //                       aux[2] - m_r[1] * ETf[0] + m_r[0] * ETf[1],
        //                       ETf[0],
        //                       ETf[1],
        //                       ETf[2]);
    }

    // transform 3D point p to/from coordinate frame 
    const BVector3 
    apply( const BVector3 &p ) const { return m_E * (p - m_r); }
    
    const BVector3 
    applyTranspose( const BVector3 &p ) const { return m_r + (glm::transpose(m_E) * p); }

    const  BSpatialInertia 
    apply( const BSpatialInertia &I ) const
    // returns  X^* I X^{-1}
    {
        const BMatrix3 ET   = glm::transpose(m_E);
        const BVector3 com_mass  = ET * (I.h() - (I.mass() * m_r)); // h
        const BMatrix3 skewr = arb::cross(m_r);
        const BMatrix3 aux = (skewr * arb::cross(I.h())) + ((arb::cross(I.h() - I.mass() * m_r) * skewr));
        return BSpatialInertia(I.mass(), com_mass, ET * (I.inertia() + aux) * m_E);
    }
    
    const BSpatialInertia 
    applyTranspose(const BSpatialInertia &I) const 
    // returns X^T I X 
    {
        const BVector3 com_mass  = (m_E * I.h()) + (I.mass() * m_r); // h
        const BMatrix3 skewr = arb::cross(m_r);
        const BMatrix3 aux = skewr * arb::cross(m_E * I.h()) + arb::cross(com_mass) * skewr;
        return BSpatialInertia(I.mass(), com_mass, (m_E * I.inertia() * glm::transpose(m_E)) - aux);
    }
    
    const BSpatialVector
    applyAdjoint(const BSpatialVector &f_sp) const  
    {
        const BMatrix3 ET = glm::transpose(m_E);
        const BVector3 ETrxf = ET * (f_sp.ang() - glm::cross(m_r, f_sp.lin()));
        const BVector3 aux = ET * f_sp.lin();
        return BSpatialVector( ETrxf, aux );
    }
    
    const BSpatialMatrix 
    toAdjoint( void ) const 
    {
        const BMatrix3 ETrx = glm::transpose(m_E) * arb::cross(m_r);
        return BSpatialMatrix( m_E, glm::transpose(ETrx), B_ZERO_3x3, m_E );
    }
    
    
    bool 
    operator==( const BSpatialTransform &v ) const { return (m_r == v.m_r) && (m_E == v.m_E); }
    
    bool 
    operator!=( const BSpatialTransform &v ) const { return (m_r != v.m_r) || (m_E != v.m_E); }
    
    friend std::istream& 
    operator>>( std::istream &istr, BSpatialTransform &m );
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BSpatialTransform &m );
    
private:
    
    BMatrix3 m_E; // rotation - note the transpose of a rotation is also its inverse
    BVector3 m_r; // translation - inverse translation is -m_r
};

 
#ifndef GLM_FORCE_INTRINSICS
constexpr BSpatialTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#else
const BSpatialTransform B_IDENTITY_TRANS(B_IDENTITY_3x3, B_ZERO_3);
#endif


inline std::ostream&
operator<<( std::ostream &ostr, const BSpatialTransform &m )
{
    ostr << m.m_E << ' ' << m.m_r  << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BSpatialTransform &m )
{
    istr >> m.m_E >> m.m_r;
    return istr;
}

//
// Articulated Rigid Body
//

namespace arb
{
    // $X^* = X^{-T}$  where X^{-T} = arb::transpose(arb::inverse(X));
 
    // style choice - I prefer arb::transpose(m) to m.transpose()  
    inline const BSpatialMatrix 
    transpose( const BSpatialTransform &m ) 
    { 
        const BMatrix3 ET = glm::transpose(m.E());
        const BMatrix3 Erx = ET * arb::cross(m.r()); 
        return BSpatialMatrix( ET, Erx, B_ZERO_3x3, ET );
    }

    inline const BSpatialTransform 
    inverse( const BSpatialTransform &m )  
    { 
        const BMatrix3 ET = glm::transpose(m.E()); 
        return BSpatialTransform(ET, -ET * m.r()); 
    }


    // all angles in radians
    inline const BSpatialTransform 
    Xrot( BScalar angle, const BVector3 &axis ) 
    {
        const BScalar s = std::sin(angle);
        const BScalar c = std::cos(angle);
  
        return BSpatialTransform(BMatrix3( axis[0] * axis[0] * (1.0 - c) + c,
                                           axis[1] * axis[0] * (1.0 - c) + axis[2] * s,
                                           axis[0] * axis[2] * (1.0 - c) - axis[1] * s,
                                          
                                           axis[0] * axis[1] * (1.0 - c) - axis[2] * s,
                                           axis[1] * axis[1] * (1.0 - c) + c,
                                           axis[1] * axis[2] * (1.0 - c) + axis[0] * s,
                                          
                                           axis[0] * axis[2] * (1.0 - c) + axis[1] * s,
                                           axis[1] * axis[2] * (1.0 - c) - axis[0] * s,
                                           axis[2] * axis[2] * (1.0 - c) + c ));
    }

    // RBDA, Section 2.8, table, 2.2, page 23
    inline const BSpatialTransform 
    Xrotx( BScalar xrot ) 
    {
        const BScalar s = std::sin(xrot);
        const BScalar c = std::cos(xrot);
        return BSpatialTransform ( BMatrix3( 1.0, 0.0, 0.0,
                                             0.0,   c,   s,
                                             0.0,  -s,   c ) );
    }

    inline const BSpatialTransform 
    Xroty( BScalar yrot ) 
    {
        const BScalar s = std::sin(yrot);
        const BScalar c = std::cos(yrot);
        return BSpatialTransform( BMatrix3(   c, 0.0,  -s,
                                            0.0, 1.0, 0.0,
                                              s, 0.0,   c ) );
    }

    inline const BSpatialTransform 
    Xrotz( BScalar zrot ) 
    {
        const BScalar s = std::sin(zrot);
        const BScalar c = std::cos(zrot);
        return BSpatialTransform( BMatrix3(  c,   s, 0.0,
                                            -s,   c, 0.0,
                                           0.0, 0.0, 1.0 ) );
    }

    inline const BSpatialTransform 
    Xtrans( const BVector3 &r ) 
    {
        return BSpatialTransform( B_IDENTITY_3x3, r ); 
    }
};

#endif


