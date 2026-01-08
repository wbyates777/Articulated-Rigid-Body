/* BBody 20/02/2024

 $$$$$$$$$$$$$$$
 $   BBody.h   $
 $$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Describes all properties of a single body
 
 
 Gyration radii used to set the main diagonal of inertia tensor 
 see https://en.wikipedia.org/wiki/List_of_moments_of_inertia

 

 Examples.
 
 BInertia 
 sphere( BScalar mass, BScalar radius )
 {
     return BInertia(mass, glm::dvec3((2.0/5.0) * mass * radius * radius));
 }
 
 BInertia 
 rectangle( BScalar mass, BScalar height, BScalar width, BScalar depth )
 {
     glm::dvec3 diag;
     diag[0] = (1.0/12.0) * mass * (depth * depth + height * height); // x-width
     diag[1] = (1.0/12.0) * mass * (depth * depth + width * width);   // y-height
     diag[2] = (1.0/12.0) * mass * (width * width + height * height); // z-depth
     return BInertia(mass, diag);
 }
 
 
*/


#ifndef __BBODY_H__
#define __BBODY_H__

#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif


class BBody
{
    
public:

    BBody( void )=default; 
    
    BBody( const BInertia &I, bool isVirtual = false): m_id(0), 
                                                       m_v(B_ZERO_6), 
                                                       m_a(B_ZERO_6),
                                                       m_c(B_ZERO_6),
                                                       m_I(I),
                                                       m_X_base(B_IDENTITY_TRANS), 
                                                       m_isVirtual(isVirtual) {}

    explicit BBody( const BRBInertia &I, bool isVirtual = false ) : m_id(0), 
                                                                    m_v(B_ZERO_6), 
                                                                    m_a(B_ZERO_6), 
                                                                    m_c(B_ZERO_6),
                                                                    m_I(I),
                                                                    m_X_base(B_IDENTITY_TRANS), 
                                                                    m_isVirtual(isVirtual) {}
        
    ~BBody( void )=default;
    
    void
    clear( void )
    {
        m_v = m_a = m_c = B_ZERO_6;
        m_I.clear();
        m_X_base.clear(); 
        m_isVirtual = false;
    }

    void
    setId( BBodyId bid ) { m_id = bid; }
    
    BBodyId
    getId( void ) const { return m_id; }
    
    
    void 
    setBody( const BInertia &inertia, bool isVirtual = false )
    {
        m_v = m_a = m_c = B_ZERO_6; 
        m_I.set(inertia);
        m_X_base.clear();
        m_isVirtual = isVirtual;
    }
 

    // spatial interia of body at the origin $I_i$ (see RBDA, Section 7.1) 
    const BRBInertia&
    I( void ) const { return m_I; }
    
    BRBInertia&
    I( void ) { return m_I; }

    void  
    I( const BRBInertia &si ) { m_I = si; }
    
    
    // X is a transform from this frame to other body frame
    // transform other body inertia into this frame and add
    void 
    join( const BTransform &X, const BBody &other_body )
    {
        if (other_body.I().mass() == 0.0 && other_body.I().Icom() == B_IDENTITY_3x3) 
            return;
        
        assert(m_I.mass() + other_body.I().mass() != 0.0);
        m_I += X.applyTranspose(other_body.I()); 
    }

    // X is a transform from this frame to other body frame
    // transform other body inertia into this frame and subtract
    void 
    separate( const BTransform &X, const BBody &other_body )
    {
        if (other_body.I().mass() == 0.0 && other_body.I().Icom() == B_IDENTITY_3x3) 
            return;
        
        assert(m_I.mass() - other_body.I().mass() != 0.0);
        m_I -= X.applyTranspose(other_body.I()); 
    }
    
    // transformation from the base (world) frame to this body's coordinate frame
    const BTransform& 
    X_base( void ) const { return m_X_base; }
    
    void
    X_base( const BTransform &b )  { m_X_base = b; }
    
    
    // spatial velocity $v_i$ of body $B_i$ (see RBDA, Section 7.3, equation 7.34)
    const BVector6&
    v( void ) const { return m_v; } 

    BVector6&
    v( void ) { return m_v; } 
    
    void
    v( const BVector6 &sv ) { m_v = sv; } 
    
    
    // spatial acceleration $a_i$ of body $B_i$ (see RBDA, Section 7.3, equation 7.31)
    const BVector6&
    a( void ) const { return m_a; } 
   
    BVector6&
    a( void ) { return m_a; } 
    
    void
    a( const BVector6 &sv ) { m_a = sv; } 
    
    
    // velocity-dependent spatial acceleration term $c_i$ (see RBDA, Section 7.3, equation 7.35)
    // $c_i = c_J + v_i \cross v_J$
    const BVector6&
    c( void ) const { return m_c; } 
   
    BVector6&
    c( void ) { return m_c; } 
    
    void
    c( const BVector6 &sv ) { m_c = sv; } 
    
    
    bool
    isVirtual( void ) const { return m_isVirtual; }
    
    
    bool 
    operator==( const BBody &v ) const { return (m_id == v.m_id); }
    
    bool 
    operator!=( const BBody &v ) const { return (m_id != v.m_id); }
    
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BBody &b );
    
    friend std::istream& 
    operator>>( std::istream &istr, BBody &b );
    
private:

    BBodyId m_id;
    
    BVector6  m_v;    // spatial velocity of the body
    BVector6  m_a;    // spatial acceleration of the body
    BVector6  m_c;    // spatial velocity-dependent acceleration term

    BRBInertia m_I;   // spatial inertia at origin of the body (mass, com, rotational inertia)
   
    // transformation from the base frame to this body's coordinate frame  - set dynamically
    BTransform m_X_base;
    
    bool m_isVirtual;
};


inline std::ostream&
operator<<( std::ostream &ostr, const BBody &b )
{
    ostr << b.m_id << '\n';
    ostr << b.m_v << '\n';
    ostr << b.m_a << '\n';
    ostr << b.m_c << '\n';
    ostr << b.m_I << '\n';
    ostr << b.m_X_base << '\n';
    ostr << b.m_isVirtual << '\n';
    
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BBody &b )
{
    istr >> b.m_id;
    istr >> b.m_v;
    istr >> b.m_a;
    istr >> b.m_c;
    istr >> b.m_I;
    istr >> b.m_X_base;
    istr >> b.m_isVirtual;
    
    return istr;
}

#endif


