/* BInertia 06/01/2026

 $$$$$$$$$$$$$$$$$$
 $   BInertia.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Standard 3D rotational inertia at com (centre of mass)
 
 Interface class to help with setup of BBody, BRBInertia and BABInertia
 
 
 https://en.wikipedia.org/wiki/Moment_of_inertia
 
 https://en.wikipedia.org/wiki/Parallel_axis_theorem
 
 
*/


#ifndef __BINERTIA_H__
#define __BINERTIA_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

class BInertia
{
    
public:
    
    BInertia( void )=default;
    BInertia( BScalar m, const BMatrix3 &I_o ) : m_mass(m), m_h(0.0), m_I(I_o) {} 
    constexpr BInertia( BScalar m, const BVector3 &com, const BMatrix3 &Icom )  : m_mass(m), 
                                                                                  m_h(m * com), 
                                                                                  m_I(Icom + m * arb::crosst(com)) {}
    BInertia( BScalar m, const BVector3 &com, const BVector3 &diag ) : m_mass(m), m_h(m * com), m_I(B_ZERO_3x3)
    {
        m_I[0][0] = diag[0];  m_I[1][1] = diag[1];  m_I[2][2] = diag[2]; 
        m_I += m_mass * arb::crosst(com);
    }
    ~BInertia( void )=default;
    
    
    void
    clear( void ) { m_mass = 0.0;  m_h = B_ZERO_3;  m_I = B_IDENTITY_3x3; }
    
    void
    set( BScalar m, const BMatrix3 &I_o ) { m_mass = m;  m_h = B_ZERO_3;  m_I = I_o; }
    
    void
    set( BScalar m, const BVector3 &com, const BMatrix3 &Icom  ) 
    {
        m_mass = m;  m_h = m * com;  m_I = Icom + m * arb::crosst(com);  
    }
    
    
    operator const BMatrix3&( void ) const { return m_I; } // inertia at origin

    
    BScalar 
    mass( void ) const { return m_mass; } 
    
    const BVector3&
    h( void ) const { return m_h; } 
    
    const BMatrix3& 
    I( void ) const { return m_I; }  // inertia at origin

    const BMatrix3 
    Icom( void ) const { return (m_mass < 1E-6) ? m_I : m_I - (arb::crosst(m_h) / m_mass); }  // inertia at com
    
    const BVector3
    com( void ) const { return (m_mass > 1E-6) ? m_h / m_mass : B_ZERO_3; }  
 
    void
    rotate( const BMatrix3 &R ) 
    {
        m_h = R * m_h;  
        m_I = R * m_I * arb::transpose(R);
    }
    
    void 
    shift( const BVector3 &p )  
    {
        const BVector3 h_new = m_h + (m_mass * -p);
        
        const BMatrix3 px = arb::cross(-p); 
        const BMatrix3 hx = arb::cross(m_h);   
        const BMatrix3 hx_new = arb::cross(h_new);
        
        //m_I = m_I - (px * hx) - (hx_new * px);
        m_I -= (px * hx) + (hx_new * px);
        m_h = h_new;
    }
    
    void 
    transform( const BMatrix3& E, const BVector3 &r ) 
    {
        // matches spatial transform X(E,r)
        shift(r);
        rotate(glm::transpose(E));
    }

    
    const BInertia
    operator*( BScalar s ) const 
    { 
        BInertia retVal(*this);
        retVal *= s;
        return retVal; 
    }
    
    const BInertia&
    operator*=( BScalar s )
    {
        m_mass *= s; m_h *= s; m_I *= s;
        return *this; 
    }
    
    const BInertia
    operator/( BScalar s ) const 
    {        
        BInertia retVal(*this);
        retVal /= s;
        return retVal; 
    }
    
    const BInertia&
    operator/=( BScalar s )
    {
        m_mass /= s; m_h /= s; m_I /= s;
        return *this; 
    }
    
    const BInertia
    operator-( void ) const 
    { 
        BInertia retVal;
        retVal.m_mass = -m_mass; retVal.m_h = -m_h; retVal.m_I = -m_I;
        return retVal; 
    }
    
    const BInertia 
    operator-( const BInertia &rhs ) const
    {
        BInertia retVal(*this);
        retVal -= rhs;
        return retVal;
    }
    
    const BInertia& 
    operator-=( const BInertia &rhs )
    {
        m_mass -= rhs.m_mass; m_h -= rhs.m_h; m_I -= rhs.m_I;
        return *this; 
    }
    
    const BInertia 
    operator+( const BInertia &rhs ) const
    {
        BInertia retVal(*this);
        retVal += rhs;
        return retVal;
    }
    
    const BInertia&
    operator+=( const BInertia &rhs )
    {
        m_mass += rhs.m_mass; m_h += rhs.m_h; m_I += rhs.m_I;
        return *this; 
    }
    
    
    bool 
    operator==( const BInertia &v ) const 
    { 
        return (m_mass == v.m_mass) && (m_h == v.m_h) && (m_I == v.m_I);
    }
    
    bool 
    operator!=( const BInertia &v ) const 
    { 
        return (m_mass != v.m_mass) || (m_h != v.m_h) || (m_I != v.m_I);
    }
    
    
    bool 
    valid( void ) const 
    {
        if (m_mass < 0.0) 
            return false;
        
        // handle the massless case (valid for pure coordinate frames)
        if (m_mass == 0.0) 
            return (glm::length2(m_h) == 0.0 && m_I == B_ZERO_3x3);
        
        // triangle inequality only strictly applies at the COM.
        const BMatrix3 I_c = Icom();
       
        if (I_c[0][0] <= 0 || I_c[1][1] <= 0 || I_c[2][2] <= 0) // ensure diagonal positivity
            return false;
        
        // triangle inequality check 
        const BScalar xx = I_c[0][0];
        const BScalar yy = I_c[1][1];
        const BScalar zz = I_c[2][2];
        const BScalar eps = BScalar(1E-9); 
        if ((xx + yy + eps < zz) || (xx + zz + eps < yy) || (yy + zz + eps < xx)) 
            return false;
        
        return true;
    }
    
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BInertia &m );
    
    friend std::istream& 
    operator>>( std::istream &istr, BInertia &m );
    
private:
    
    BScalar  m_mass; // total mass (kg) - zeroth moment of mass 
    BVector3 m_h;    // first moment of mass h = m_com * m_mass - (see RBDA, Section 2.12, page 31)
    BMatrix3 m_I;    // rotational inertia $I$ at body frame origin (0,0,0); second moment of mass 
 
};



// scalar multiplication
inline const BInertia 
operator*( BScalar s, const BInertia &m ) { return m * s; }


inline std::ostream&
operator<<( std::ostream &ostr, const BInertia &I )
{
    ostr << I.m_mass << '\n' << I.m_h << '\n' << I.m_I << '\n';
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BInertia &I )
{
    istr >> I.m_mass >> I.m_h >> I.m_I;
    return istr;
}


#endif

