/* BRBInertia 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BRBInertia.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$
 
 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 
 Spatial Inertia (see Featherstone, RBDA, Section 2.13, page 32).
 
 A represention of the distribution of mass in a rigid body 
 Consists of three moments of mass:
 
 0) the zeroth moment - total mass m,
 1) the first moment  - h = com * mass (where com is centre of mass), and
 2) the second moment - inertia I.

 $h$ is the magnitude and direction of the body’s linear momentum (see RBDA, Section 2.12, page 31).
 
 Inertia is represented as 3D rotational inertia, and 6D spatial inertia.
 
 Note:
 
 1) when com = B_ZERO_3, inertia() and inertiaCom() are identical as 
    arb::cross(m_com) and arb::cross(-m_com) are zero.
 2) BVector3(m_mass) == (B_IDENTITY_3x3 * m_mass)
 3) arb::cross(-m_h) == arb::transpose(arb::cross(m_h)) - see table 2.1, page 22,
 4) arb::cross(m_h) ==  m_mass * arb::cross(m_com)
 5) arb::cross(m_h) * arb::cross(-m_h) != m_mass * arb::cross(m_com) * arb::cross(-m_com)

 
 https://en.wikipedia.org/wiki/Moment_(physics)
 
 https://en.wikipedia.org/wiki/List_of_moments_of_inertia
 
 TODO: separate 3D stuff into BMass(mass,h,I^3) and I^6 stuff BRBInertia
 
*/


#ifndef __BRBINERTIA_H__
#define __BRBINERTIA_H__


#ifndef __BSPATIALMATRIX_H__
#include "BSpatialMatrix.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif


class BRBInertia
{

public:

    BRBInertia( void )=default;
    // the moments of mass; zero, one, and two - note mass can be 0
    constexpr BRBInertia( BScalar mass, const BVector3 &h, const BMatrix3 &inertia ): m_mass(mass), m_h(h), m_I(inertia) {}
    BRBInertia( const BSpatialMatrix &I ) { setInertia(I); }     
    ~BRBInertia( void )=default;
    
    void
    clear( void ) {  m_mass = 0.0; m_h = B_ZERO_3; m_I = B_ZERO_3x3; }
    
    // the moments of mass; zero, one, and two - note mass can be 0
    // called by BSpatialTransform::apply/applyTransform
    void 
    setMoments( BScalar mass, const BVector3 &h, const BMatrix3 &inertia )
    {
        m_mass = mass; m_h = h; m_I = inertia;
    }
    
    void 
    setInertia( const BSpatialMatrix &I )     
    {
        m_mass = I[3][3]; 
        m_h    = BVector3(-I[1][5], I[0][5], -I[0][4]); 
        m_I    = I.topLeft();
    }

    // inertia_com matrix is defined with respect to com  
    // if com is non zero the inertia will be translated to the body frame origin
    // RBDA, Section 2.13, eqn 2.63, page 33.
    // called from BBody::BBody(); this was RBDL function createFromMassComInertiaC(Body) 
    void 
    setInertia( BScalar mass, const BVector3 &com, const BMatrix3 &inertia_com  )
    {
        m_mass = mass;  // note mass can be 0 
        m_h    = com * m_mass;
        m_I    = inertia_com + m_mass * arb::crosst(com); 
    }

    void
    setMass( BScalar mass ) { setInertia(mass, com(), inertiaCom()); }

    void
    setCom(const BVector3 &com ) { setInertia(m_mass, com, inertiaCom()); }
    
    void
    setInertia( const BMatrix3 &inertia_com ) { setInertia(m_mass, com(), inertia_com); }
 
    // RBDA, Section 2.13, eqn 2.63, page 33.
    operator BSpatialMatrix( void ) const 
    { 
        return BSpatialMatrix( topLeft(), topRight(), botLeft(), botRight() );
    }
    
    // zeroth moment of mass
    BScalar
    mass( void ) const { return m_mass; }
    
    // centre of mass in body coordinates
    const BVector3
    com( void ) const { assert(m_mass != 0); return  m_h / m_mass; }
    
   // const BVector3 // some virtual bodies have mass 0
   // com( void ) const { return (m_mass) ? (m_h / m_mass) : B_ZERO_3; }
        
    // magnitude and direction of linear momentum; first moment of mass 
    const BVector3
    h( void ) const {  return m_h; }
    
    // rotational inertia at body frame origin; second moment of mass
    const BMatrix3& 
    inertia( void ) const { return m_I; } 
    
    // rotational inertia at com; second moment of mass
    const BMatrix3 
    inertiaCom( void ) const { return m_I - m_mass * arb::crosst(com()); } 
 
 
    const BMatrix3&
    topLeft( void ) const { return m_I; } // inertia at origin
    
    const BMatrix3
    topRight( void ) const { return arb::cross(m_h); }
    
    const BMatrix3
    botLeft( void ) const { return arb::cross(-m_h); }
    
    const BMatrix3
    botRight( void ) const  { return BMatrix3(m_mass); }
    

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
    
    const BSpatialVector 
    operator*( const BSpatialVector &mv ) const
    {
        const BVector3 ang(glm::cross(m_h, mv.lin()) + (m_I * mv.ang()));
        const BVector3 lin(m_mass * mv.lin() - glm::cross(m_h, mv.ang()));
        return BSpatialVector( ang, lin );
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

namespace arb
{
    // you should generally try to avoid taking the inverse of a spatial matrix
    // for articulated rigid bodies use the ABA to calculate accelerations
    inline const BSpatialMatrix 
    inverse( const BRBInertia &I ) 
    // Schur complement - analytical inverse, see RBDA, Section 2.15, eqn 2.74,  page 36
    {  
        assert(I.mass() != 0.0);
        const BMatrix3 iI(glm::inverse(I.inertiaCom()));
        const BMatrix3 iM(1.0 / I.mass());
        const BVector3 com(I.com());
        
        return BSpatialMatrix(          iI,                    iI * arb::cross(-com),  
                               arb::cross(com) * iI,  iM + arb::cross(com) * iI * arb::cross(-com));
        
    } 

};

#ifndef GLM_FORCE_INTRINSICS
constexpr BRBInertia B_ZERO_RBI(0.0, B_ZERO_3, B_ZERO_3x3);
#else
const BRBInertia B_ZERO_RBI(0.0, B_ZERO_3, B_ZERO_3x3);
#endif

// scalar multiplication
inline const BRBInertia 
operator*( BScalar s, const BRBInertia &m ) { return m * s; }

inline std::ostream&
operator<<( std::ostream &ostr, const BRBInertia &m )
{
    ostr << m.m_mass << '\n' << m.m_h << '\n'  << m.m_I << ' ';
    return ostr;
}


inline std::istream& 
operator>>( std::istream &istr, BRBInertia &m )
{
    istr >> m.m_mass >> m.m_h >> m.m_I;
    return istr;
}

#endif


