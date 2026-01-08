/* BRBInertia 20/02/2024

 $$$$$$$$$$$$$$$$$$$$
 $   BRBInertia.h   $
 $$$$$$$$$$$$$$$$$$$$
 
 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 
 Spatial Inertia (see Featherstone, RBDA, Section 2.13, page 32).
 
 A compact represention of the distribution of mass in a rigid body 
 Consists of three moments of mass:
 
 0) the zeroth moment - total mass m,
 1) the first moment  - h = com * mass (where com is centre of mass), and
 2) the second moment - 3D rotational inertia I_o at body coordinate frame origin.

 $h$ is the magnitude and direction of the body’s linear momentum (see RBDA, Section 2.12, page 31).
 
 3D Rotational inertia can be specified at the centre of mass (com), denoted I_com, or
 at the origin of a coordinate frame, denoted I_o. If com is B_ZERO_3 then com and the frame origin coincide and 
 I_com == I_o. If com is non-zero I_com can be translated to the coordinate frame origin, and I_o.
 
 Spatial Inertia can also be represented as a 6D spatial matrix defined by
 
 |  I_o    hx  |
 |  -hx     M  |
 
 where
 
     I_o is the rotational inertia at body frame origin
     hx = arb::cross(h) is the generalised inertia matrix
     M = m * B_IDENTITY_3x3 is the generalised mass 
 
 Note:
 
 1) when com = B_ZERO_3, I_o and I_com are identical as 
    arb::cross(m_com) and arb::cross(-m_com) are zero.
 2) BVector3(m_mass) == (B_IDENTITY_3x3 * m_mass)
 3) arb::cross(-m_h) == arb::transpose(arb::cross(m_h)) - see table 2.1, page 22,
 4) arb::cross(m_h) ==  m_mass * arb::cross(m_com)
 5) arb::cross(m_h) * arb::cross(-m_h) != m_mass * arb::cross(m_com) * arb::cross(-m_com)

 
 https://en.wikipedia.org/wiki/Moment_(physics)
 
 https://en.wikipedia.org/wiki/List_of_moments_of_inertia
 

 
*/


#ifndef __BRBINERTIA_H__
#define __BRBINERTIA_H__


#ifndef __BINERTIA_H__
#include "BInertia.h"
#endif

#ifndef __BMATRIX6_H__
#include "BMatrix6.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif


class BRBInertia
{

public:

    BRBInertia( void )=default;
    // the moments of mass; zero, one, and two - note mass can be 0, I_o inertia at body frame origin
    constexpr BRBInertia( BScalar mass, const BVector3 &h, const BMatrix3 &I_o ): m_mass(mass), m_h(h), m_I(I_o) {}
    BRBInertia( const BInertia &I ): m_mass(I.mass()), m_h(I.h()), m_I(I.I()) {} 
    BRBInertia( const BMatrix6 &I ) { set(I); }     
    ~BRBInertia( void )=default;
    
    void
    clear( void ) {  m_mass = 0.0; m_h = B_ZERO_3; m_I = B_ZERO_3x3; }
    
    void 
    set( const BInertia &I ) { m_mass = I.mass(); m_h = I.h(); m_I = I.I(); }
    
    void 
    set( const BMatrix6 &I )     
    {
        m_mass = I[3][3]; 
        m_h    = BVector3(-I[1][5], I[0][5], -I[0][4]); 
        m_I    = I.topLeft();
    }

  
    // RBDA, Section 2.13, eqn 2.63, page 33.
    operator BMatrix6( void ) const 
    { 
        return BMatrix6( m_I, arb::cross(m_h), arb::cross(-m_h),  BMatrix3(m_mass) );
    }
    
    // zeroth moment of mass
    BScalar
    mass( void ) const { return m_mass; }
  
    // magnitude and direction of linear momentum; first moment of mass 
    const BVector3
    h( void ) const {  return m_h; }
    
    // rotational inertia at body frame origin; second moment of mass
    const BMatrix3& 
    I( void ) const { return m_I; } 
    
    // rotational inertia at com; second moment of mass
    const BMatrix3 
    Icom( void ) const { return m_I - m_mass * arb::crosst(com()); } 
 
    
    // centre of mass in body coordinates
    const BVector3
    com( void ) const { assert(m_mass != 0); return  m_h / m_mass; }
    
   // const BVector3 // some virtual bodies have mass 0
   // com( void ) const { return (m_mass) ? (m_h / m_mass) : B_ZERO_3; }
      
    
    const BRBInertia
    operator-( void ) const { return BRBInertia(-m_mass, -m_h, -m_I); }
    
    const BRBInertia 
    operator+( const BRBInertia &rhs ) const
    {
        return BRBInertia( m_mass + rhs.m_mass, m_h + rhs.m_h, m_I + rhs.m_I );
    }
    
    const BRBInertia 
    operator-( const BRBInertia &rhs ) const
    {
        return BRBInertia( m_mass - rhs.m_mass, m_h - rhs.m_h, m_I - rhs.m_I );
    }
    
    const BRBInertia&
    operator+=( const BRBInertia &rhs )
    {
        m_mass += rhs.m_mass; m_h += rhs.m_h; m_I += rhs.m_I;
        return *this; 
    }
    
    const BRBInertia& 
    operator-=( const BRBInertia &rhs )
    {
        m_mass -= rhs.m_mass; m_h -= rhs.m_h; m_I -= rhs.m_I;
        return *this; 
    }
    
    const BRBInertia
    operator*( BScalar s ) const { return  BRBInertia( s * m_mass, s * m_h, s * m_I ); }
    
    const BRBInertia&
    operator*=( const BScalar s )
    {
        m_mass *= s; m_h *= s; m_I *= s;
        return *this; 
    }
    
    const BVector6 
    operator*( const BVector6 &v ) const
    {
        //const BVector3 ang(arb::cross(m_h, v.lin()) + (m_I * v.ang()));
        //const BVector3 lin(m_mass * v.lin() - arb::cross(m_h, v.ang()));
        // return BVector6( ang, lin );
    
        return BVector6((m_h[1] * v[5] - v[4] * m_h[2]) + (m_I[0][0] * v[0])  +  (m_I[1][0] * v[1]) + (m_I[2][0] * v[2]),
                              (m_h[2] * v[3] - v[5] * m_h[0]) + (m_I[0][1] * v[0])  +  (m_I[1][1] * v[1]) + (m_I[2][1] * v[2]),
                              (m_h[0] * v[4] - v[3] * m_h[1]) + (m_I[0][2] * v[0])  +  (m_I[1][2] * v[1]) + (m_I[2][2] * v[2]),
                           
                              (m_mass * v[3])  -  (m_h[1] * v[2] - v[1] * m_h[2]),
                              (m_mass * v[4])  -  (m_h[2] * v[0] - v[2] * m_h[0]),
                              (m_mass * v[5])  -  (m_h[0] * v[1] - v[0] * m_h[1]) );
        
    }
    
    
    bool 
    operator==( const BRBInertia &v ) const 
    { 
        return (m_mass == v.m_mass) && (m_h == v.m_h) && (m_I == v.m_I);
    }
    
    bool 
    operator!=( const BRBInertia &v ) const 
    { 
        return (m_mass != v.m_mass) || (m_h != v.m_h) || (m_I != v.m_I);
    }
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BRBInertia &m );
    
    friend std::istream& 
    operator>>( std::istream &istr, BRBInertia &m );
    
private:


    BScalar  m_mass; // total mass (kg) - zeroth moment of mass 
    BVector3 m_h;    // magnitude and direction of linear momentum h = m_com * m_mass; first moment of mass 
    BMatrix3 m_I;    // rotational inertia $I$ at body frame origin (0,0); second moment of mass 
    
};


// scalar multiplication
inline const BRBInertia 
operator*( BScalar s, const BRBInertia &m ) { return m * s; }


#ifndef GLM_FORCE_INTRINSICS
constexpr BRBInertia B_ZERO_RBI(0.0, B_ZERO_3, B_ZERO_3x3);
#else
const BRBInertia B_ZERO_RBI(0.0, B_ZERO_3, B_ZERO_3x3);
#endif


namespace arb
{
    inline const BMatrix6 
    inverse( const BRBInertia &I ) 
    // Schur complement - analytical inverse - https://en.wikipedia.org/wiki/Schur_complement
    // see RBDA, Section 2.15, eqn 2.74,  page 36
    {  
        assert(I.mass() != 0.0);
        const BMatrix3 invI(arb::inverse(I.Icom()));
        const BMatrix3 invM(1.0 / I.mass());
        const BVector3 com(I.com());

        return BMatrix6(           invI,                       arb::cross(-com) * invI,  
                        invI * arb::cross(com),  invM + arb::cross(-com) * invI * arb::cross(com) );
        
    } 
}


inline std::ostream&
operator<<( std::ostream &ostr, const BRBInertia &m )
{
    ostr << m.m_mass << '\n' << m.m_h << '\n'  << m.m_I << '\n';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BRBInertia &m )
{
    istr >> m.m_mass >> m.m_h >> m.m_I;
    return istr;
}

#endif


