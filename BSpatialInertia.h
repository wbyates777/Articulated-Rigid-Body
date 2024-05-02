/* BSpatialInertia 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialInertia.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$
 
 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/


#ifndef __BSPATIALINERTIA_H__
#define __BSPATIALINERTIA_H__


#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif


class BBody;

class BSpatialInertia
{

public:

    BSpatialInertia( void )=default;

    BSpatialInertia( BScalar mass, const BVector3 &com, const BMatrix3 &inertia ) : m_h(com), m_mass(mass)
    {
        m_Ixx = inertia[0][0];
        m_Iyx = inertia[1][0]; m_Iyy = inertia[1][1];
        m_Izx = inertia[2][0]; m_Izy = inertia[2][1]; m_Izz = inertia[2][2];   
    }
    
    explicit BSpatialInertia( const BBody &b );
    
    BSpatialInertia( BScalar mass, const BVector3 &com_mass, 
                     BScalar xx, BScalar yx, BScalar yy, BScalar zx, BScalar zy, BScalar zz ) 
                    : m_h(com_mass), m_mass(mass),  m_Ixx(xx), m_Iyx(yx), m_Iyy(yy), m_Izx(zx), m_Izy(zy), m_Izz(zz) 
    {}

    ~BSpatialInertia( void )=default;
    

    
    void 
    setSpatialMatrix( BSpatialMatrix &m ) const 
    // update m with our values  
    {
        m[0][0] = m_Ixx; m[0][1] = m_Iyx; m[0][2] = m_Izx;
        m[1][0] = m_Iyx; m[1][1] = m_Iyy; m[1][2] = m_Izy;
        m[2][0] = m_Izx; m[2][1] = m_Izy; m[2][2] = m_Izz;
        
        m[3][0] =   0.0;   m[3][1] =  m_h[2]; m[3][2]  = -m_h[1];
        m[4][0] = -m_h[2]; m[4][1] =     0.0; m[4][2]  =  m_h[0];
        m[5][0] =  m_h[1]; m[5][1] = -m_h[0]; m[5][2]  =     0.0;
        
        m[0][3] =   0.0;   m[0][4] = -m_h[2]; m[0][5]  =  m_h[1];
        m[1][3] =  m_h[2]; m[1][4] =     0.0; m[1][5]  = -m_h[0];
        m[2][3] = -m_h[1]; m[2][4] =  m_h[0]; m[2][5]  =     0.0;
        
        m[3][3] = m_mass;  m[3][4] =     0.0; m[3][5] =      0.0;
        m[4][3] =   0.0;   m[4][4] =  m_mass; m[4][5] =      0.0;
        m[5][3] =   0.0;   m[5][4] =     0.0; m[5][5] =   m_mass;
    }
    
    const BSpatialMatrix 
    toMatrix( void ) const 
    {
        // note BMatrix3(m_mass) is BIDENTITY_3x3 * m_mass
        return BSpatialMatrix( BMatrix3(m_Ixx, m_Iyx, m_Izx, m_Iyx, m_Iyy, m_Izy, m_Izx, m_Izy, m_Izz),  
                               arb::cross_matrix(m_h), 
                               arb::cross_matrix(-m_h), 
                               BMatrix3(m_mass) );
 
    }
    
    const BSpatialInertia 
    operator+( const BSpatialInertia &rhs ) const
    {
        return BSpatialInertia( m_mass + rhs.m_mass, m_h + rhs.m_h,
                                m_Ixx + rhs.m_Ixx,
                                m_Iyx + rhs.m_Iyx, 
                                m_Iyy + rhs.m_Iyy,
                                m_Izx + rhs.m_Izx, 
                                m_Izy + rhs.m_Izy, 
                                m_Izz + rhs.m_Izz );
    }
    
    const BSpatialInertia 
    operator-( const BSpatialInertia &rhs ) const
    {
        return BSpatialInertia( m_mass - rhs.m_mass, m_h - rhs.m_h,
                                m_Ixx - rhs.m_Ixx,
                                m_Iyx - rhs.m_Iyx, 
                                m_Iyy - rhs.m_Iyy,
                                m_Izx - rhs.m_Izx, 
                                m_Izy - rhs.m_Izy, 
                                m_Izz - rhs.m_Izz );
    }
    
    const BSpatialVector 
    operator*( const BSpatialVector &mv ) const
    {
        BVector3 res_upper(glm::cross(m_h, mv.tail()) + 
                                 BVector3( m_Ixx * mv[0] + m_Iyx * mv[1] + m_Izx * mv[2],
                                           m_Iyx * mv[0] + m_Iyy * mv[1] + m_Izy * mv[2],
                                           m_Izx * mv[0] + m_Izy * mv[1] + m_Izz * mv[2] ));
      
        BVector3 res_lower(m_mass * mv.tail() - glm::cross(m_h, mv.head()));

        return BSpatialVector( res_upper, res_lower );
    }
    
    friend  std::ostream&
    operator<<( std::ostream& ostr, const BSpatialInertia& m );
    
    friend  std::istream& 
    operator>>( std::istream& istr, BSpatialInertia& m );
    
private:

    BVector3 m_h;    // coordinates of the center of mass
    BScalar m_mass;
    // inertia expressed at the origin
    BScalar m_Ixx;
    BScalar m_Iyx;
    BScalar m_Iyy;
    BScalar m_Izx;
    BScalar m_Izy;
    BScalar m_Izz;

};


inline std::ostream&
operator<<( std::ostream& ostr, const BSpatialInertia& m )
{
    ostr << m.m_mass << '\n' << m.m_h[0] << ' ' << m.m_h[1] << ' ' << m.m_h[2] << '\n'
         << m.m_Ixx << ' ' << m.m_Iyx << ' ' << m.m_Iyy << ' ' << m.m_Izx << ' ' << m.m_Izy << ' ' << m.m_Izz << ' ';

    return ostr;
}


inline std::istream& 
operator>>( std::istream& istr, BSpatialInertia& m )
{
    istr >> m.m_mass >> m.m_h[0] >> m.m_h[1] >> m.m_h[2] >> m.m_Ixx >> m.m_Iyx >> m.m_Iyy >> m.m_Izx >> m.m_Izy >> m.m_Izz;
    return istr;
}

#endif


