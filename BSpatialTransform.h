/* BSpatialTransform 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialTransform.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Compact representation of spatial transformations.
 
 Instead of using a verbose 6x6 matrix, this structure only stores a 3x3
 matrix and a 3-d vector to store spatial transformations. It also
 encapsulates efficient operations such as concatenations and
 transformation of spatial vectors.
 
*/



#ifndef __BSPATIALTRANSFORM_H__
#define __BSPATIALTRANSFORM_H__



#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif


class BSpatialTransform
{
    
public:
    
    BSpatialTransform( void )=default;
    explicit BSpatialTransform( const BMatrix3 &rot, const BVector3 &trans = BZERO_3 ): m_E(rot), m_r(trans) {}
    ~BSpatialTransform( void )=default;
    
    
    const BMatrix3&
    E( void ) const { return m_E; }
    
    void
    E( const BMatrix3 &m ) { m_E = m; }
    
    const BVector3&
    r( void ) const { return m_r; }
    
    void
    r( const BVector3 &v ) { m_r = v; }
    
    
    const BSpatialTransform 
    operator*( const BSpatialTransform &rhs ) const 
    {
        return BSpatialTransform(rhs.m_E * m_E, rhs.m_r + (rhs.m_E * m_r));
        //return SpatialTransform (E * XT.E, XT.r + XT.E.transpose() * r);
    }
    
    
    const BSpatialTransform 
    inverse( void ) const 
    { 
        BMatrix3 ET = glm::transpose(m_E);
        return BSpatialTransform(ET, -ET * m_r); 
        // return SpatialTransform ( E.transpose(), -E * r  );
    }
    

    const BSpatialVector 
    apply( const BSpatialVector &v ) const 
    {
        BVector3 v_rxw( v[3] - m_r[1] * v[2] + m_r[2] * v[1],
                        v[4] - m_r[2] * v[0] + m_r[0] * v[2],
                        v[5] - m_r[0] * v[1] + m_r[1] * v[0] );

        
        return BSpatialVector(m_E[0][0] * v[0]  + m_E[0][1] * v[1]  + m_E[0][2] * v[2],
                              m_E[1][0] * v[0]  + m_E[1][1] * v[1]  + m_E[1][2] * v[2],
                              m_E[2][0] * v[0]  + m_E[2][1] * v[1]  + m_E[2][2] * v[2],
                              
                              m_E[0][0] * v_rxw[0] + m_E[0][1] * v_rxw[1] + m_E[0][2] * v_rxw[2],
                              m_E[1][0] * v_rxw[0] + m_E[1][1] * v_rxw[1] + m_E[1][2] * v_rxw[2],
                              m_E[2][0] * v_rxw[0] + m_E[2][1] * v_rxw[1] + m_E[2][2] * v_rxw[2] );
    }
    

    const BSpatialVector 
    applyTranspose( const BSpatialVector &f ) const
    // X^T * f 
    {
        BVector3 ETf( m_E[0][0] * f[3] + m_E[1][0] * f[4] + m_E[2][0] * f[5],
                      m_E[0][1] * f[3] + m_E[1][1] * f[4] + m_E[2][1] * f[5],
                      m_E[0][2] * f[3] + m_E[1][2] * f[4] + m_E[2][2] * f[5]);
        
        return BSpatialVector(m_E[0][0] * f[0] + m_E[1][0] * f[1] + m_E[2][0] * f[2] - m_r[2] * ETf[1] + m_r[1] * ETf[2],
                              m_E[0][1] * f[0] + m_E[1][1] * f[1] + m_E[2][1] * f[2] + m_r[2] * ETf[0] - m_r[0] * ETf[2],
                              m_E[0][2] * f[0] + m_E[1][2] * f[1] + m_E[2][2] * f[2] - m_r[1] * ETf[0] + m_r[0] * ETf[1],
                              ETf[0],
                              ETf[1],
                              ETf[2] );
    }
    
    
    const BSpatialMatrix 
    toTranspose( void ) const 
    {
        BMatrix3 TE = glm::transpose(m_E);
        BMatrix3 _Erx = TE * arb::cross_matrix(m_r); 
        return BSpatialMatrix( TE, _Erx, BZERO_3x3, TE );
    }
    
    const BSpatialMatrix 
    toMatrix( void ) const 
    {
        BMatrix3 _Erx =  glm::transpose(-m_E) * arb::cross_matrix(m_r);
        return BSpatialMatrix( m_E, BZERO_3x3, -glm::transpose(_Erx), m_E );
    }
    
    const BSpatialMatrix 
    toAdjoint( void ) const 
    {
        BMatrix3 _Erx = glm::transpose(m_E) * arb::cross_matrix(m_r);
        return BSpatialMatrix( m_E, glm::transpose(_Erx), BZERO_3x3, m_E );
    }
    
    
    friend std::istream& 
    operator>>( std::istream& istr, BSpatialTransform& m );
    
    friend std::ostream&
    operator<<( std::ostream& ostr, const BSpatialTransform& m );
    
private:
    
    BMatrix3 m_E;
    BVector3 m_r;
};


inline std::ostream&
operator<<( std::ostream& ostr, const BSpatialTransform& m )
{
    ostr << m.m_E[0][0] << ' ' << m.m_E[0][1] << ' ' << m.m_E[0][2] << '\n'
         << m.m_E[1][0] << ' ' << m.m_E[1][1] << ' ' << m.m_E[1][2] << '\n'
         << m.m_E[2][0] << ' ' << m.m_E[2][1] << ' ' << m.m_E[2][2] << '\n'
         << m.m_r[0]    << ' ' << m.m_r[1]    << ' ' << m.m_r[2]    << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream& istr, BSpatialTransform& m )
{
    istr >> m.m_E[0][0] >> m.m_E[0][1] >> m.m_E[0][2]
         >> m.m_E[1][0] >> m.m_E[1][1] >> m.m_E[1][2]
         >> m.m_E[2][0] >> m.m_E[2][1] >> m.m_E[2][2]
         >> m.m_r[0] >> m.m_r[1] >> m.m_r[2];
    return istr;
}

//
// Articulated Rigid Body
//

namespace arb
{
    inline const BSpatialTransform 
    Xrot( BScalar angle_rad, const BVector3 &axis ) 
    {
        BScalar s = std::sin(angle_rad);
        BScalar c = std::cos(angle_rad);
  
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

    inline const BSpatialTransform 
    Xrotx( BScalar xrot ) 
    {
        BScalar s = std::sin(xrot);
        BScalar c = std::cos(xrot);
        return BSpatialTransform ( BMatrix3( 1.0, 0.0, 0.0,
                                             0.0,   c,   s,
                                             0.0,  -s,   c ) );
    }

    inline const BSpatialTransform 
    Xroty( BScalar yrot ) 
    {
        BScalar s = std::sin(yrot);
        BScalar c = std::cos(yrot);
        return BSpatialTransform( BMatrix3(   c, 0.0,  -s,
                                            0.0, 1.0, 0.0,
                                              s, 0.0,   c ) );
    }

    inline const BSpatialTransform 
    Xrotz( BScalar zrot ) 
    {
        BScalar s = std::sin(zrot);
        BScalar c = std::cos(zrot);
        return BSpatialTransform( BMatrix3(  c,   s, 0.0,
                                            -s,   c, 0.0,
                                           0.0, 0.0, 1.0 ) );
    }

    inline const BSpatialTransform 
    Xtrans( const BVector3 &r ) 
    {
        return BSpatialTransform( BIDENTITY_3x3, r ); 
    }
};

#endif


