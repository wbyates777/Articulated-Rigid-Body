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

 

 Example 1.
 
 double mass = 99.0;   // kg
 double radius = 0.25; // m
 BVector3 gyration_radii = BVector3((2.0 / 5.0) * mass * (radius * radius));
 BVector3 com = BVector3(0.0); // centre of mass in body coordinates
 
 BBody sphere = BBody(mass,  com, gyration_radii);
 
 
*/


#ifndef __BBODY_H__
#define __BBODY_H__

#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif


class BBody
{
    
public:

    BBody( void )=default; 
    
    // construct a body from mass, center of mass (com) and radii of gyration
    // gyration radii used to set the main diagonal of inertia tensor 
    BBody( BScalar mass,
           const BVector3 &com,
           const BVector3 &gyration_radii = B_ONE_3,
           bool isVirtual = false) : m_id(0), 
                                     m_v(B_ZERO_6), 
                                     m_a(B_ZERO_6),
                                     m_c(B_ZERO_6),
                                     m_f(B_ZERO_6),
                                     m_I(B_ZERO_RBI),
                                     m_X_base(B_IDENTITY_TRANS), 
                                     m_isVirtual(isVirtual)
    {
        BMatrix3 I_com = BMatrix3( gyration_radii[0], 0.0, 0.0,
                                   0.0, gyration_radii[1], 0.0,
                                   0.0, 0.0, gyration_radii[2] );
        
        m_I.setInertiaCom(mass, com, I_com);
    }

    // construct a body from mass, center of mass (com) and inertia $I_com$
    // $I_com$ is defined with respect to the centre of mass $com$ 
    // if $c = com$ is non-zero then $I_com$ will be translated to position $-com$ - the origin of the rigid body.
    BBody( BScalar mass,
           const BVector3 &com,
           const BMatrix3 &I_com,
           bool isVirtual = false) : m_id(0), 
                                     m_v(B_ZERO_6), 
                                     m_a(B_ZERO_6), 
                                     m_c(B_ZERO_6),
                                     m_f(B_ZERO_6),
                                     m_I(B_ZERO_RBI),
                                     m_X_base(B_IDENTITY_TRANS), 
                                     m_isVirtual(isVirtual) 
    {
        // WARNING: do not use m_I(mass, com, I_com) as this is incorrect
        m_I.setInertiaCom(mass, com, I_com); 
    }
    
    explicit BBody( const BRBInertia &I, bool isVirtual = false ) : m_id(0), 
                                                                    m_v(B_ZERO_6), 
                                                                    m_a(B_ZERO_6), 
                                                                    m_c(B_ZERO_6),
                                                                    m_f(B_ZERO_6),
                                                                    m_I(I),
                                                                    m_X_base(B_IDENTITY_TRANS), 
                                                                    m_isVirtual(isVirtual) {}
        
    ~BBody( void )=default;
    
    void
    clear( void )
    {
        m_v = m_a = m_c = m_f = B_ZERO_6;
        m_I.clear();
        m_X_base.clear(); 
        m_isVirtual = false;
    }

    void
    setId( BBodyId bid ) { m_id = bid; }
    
    BBodyId
    getId( void ) const { return m_id; }
    
    
    void 
    setBody( BScalar mass, const BVector3 &com, const BMatrix3 &I_com, bool isVirtual = false )
    {
        m_v = m_a = m_c = m_f = B_ZERO_6; 
        m_I.setInertiaCom(mass, com, I_com);
        m_X_base.clear();
        m_isVirtual = isVirtual;
    }
    
    void 
    setMass( BScalar m ) { m_I.setMass(m); }
    
    BScalar 
    mass( void ) const { return m_I.mass(); }

    // centre of mass in body coordinates
    void
    setCom( const BVector3 &c ) { m_I.setCom(c); }
    
    const BVector3 
    com( void ) const { return  m_I.com(); }
    
    //  magnitude and direction of linear momentum; first moment of mass 
    const BVector3 
    h( void ) const { return m_I.h(); }
    
    // set the inertia at the centre of mass $com$ (which may be non-zero)
    void 
    setInertiaCom( const BMatrix3 &I_com ) { m_I.setInertiaCom(I_com); }

    // get the inertia at the centre of mass $com$ (which may be non-zero)
    const BMatrix3 //used in Fixed body constructor
    inertiaCom( void ) const { return m_I.inertiaCom();  }
    
    // get the inertia at the the origin of this body's coordinate frame
    const BMatrix3& 
    inertia( void ) const { return m_I.inertia();  } 
    

    // spatial interia of body at the origin $I_i$ (see RBDA, Section 7.1) 
    const BRBInertia&
    I( void ) const { return m_I; }

    void  
    I( const BRBInertia &si ) { m_I = si; }
    
    
    // X is a transform from this frame to other body frame
    // transform other body inertia into this frame and add
    void 
    join( const BSpatialTransform &X, const BBody &other_body )
    {
        if (other_body.mass() == 0.0 && other_body.I().inertiaCom() == B_IDENTITY_3x3) 
            return;
        
        assert(m_I.mass() + other_body.mass() != 0.0);
        m_I += X.applyTranspose(other_body.I()); 
    }

    // X is a transform from this frame to other body frame
    // transform other body inertia into this frame and subtract
    void 
    separate( const BSpatialTransform &X, const BBody &other_body )
    {
        if (other_body.mass() == 0.0 && other_body.I().inertiaCom() == B_IDENTITY_3x3) 
            return;
        
        assert(m_I.mass() - other_body.mass() != 0.0);
        m_I -= X.applyTranspose(other_body.I()); 
    }
    
    // transformation from the base to this body's $B_i$ coordinate frame $F^i$ - set in  BDynmaics::forward()
    const BSpatialTransform& 
    X_base( void ) const { return m_X_base; }
    
    void
    X_base( const BSpatialTransform &b )  { m_X_base = b; }
    
    
    // spatial velocity $v_i$ of body $B_i$ (see RBDA, Section 7.3, equation 7.34)
    const BSpatialVector&
    v( void ) const { return m_v; } 

    BSpatialVector&
    v( void ) { return m_v; } 
    
    void
    v( const BSpatialVector &sv ) { m_v = sv; } 
    
    
    // spatial acceleration $a_i$ of body $B_i$ (see RBDA, Section 7.3, equation 7.31)
    const BSpatialVector&
    a( void ) const { return m_a; } 
   
    BSpatialVector&
    a( void ) { return m_a; } 
    
    void
    a( const BSpatialVector &sv ) { m_a = sv; } 
    
    
    // velocity-dependent spatial acceleration term $c_i$ (see RBDA, Section 7.3, equation 7.35)
    // $c_i = c_J + v_i \cross v_J$
    const BSpatialVector&
    c( void ) const { return m_c; } 
   
    BSpatialVector&
    c( void ) { return m_c; } 
    
    void
    c( const BSpatialVector &sv ) { m_c = sv; } 
    
    
    // internal forces on the body (used only BDynamics::inverse())
    // $f_i$ the net force acting on body $B_i$ (see RBDA, equation 5.9)
    const BSpatialVector& 
    f( void ) const { return m_f; }
    
    BSpatialVector& 
    f( void ) { return m_f; }
    
    void  
    f( const BSpatialVector& v ) { m_f = v; }
   
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
    
    BSpatialVector  m_v;    // spatial velocity of the body
    BSpatialVector  m_a;    // spatial acceleration of the body
    BSpatialVector  m_c;    // spatial velocity-dependent acceleration term
    BSpatialVector  m_f;    // internal forces on the body (used only BDynamics::inverse())

    BRBInertia m_I;         // spatial inertia at origin of the body (mass, com, rotational inertia)
   
    // transformation from the base frame to this body's coordinate frame - set dynamically
    BSpatialTransform m_X_base;
    
    bool m_isVirtual;
};


inline std::ostream&
operator<<( std::ostream &ostr, const BBody &b )
{
    ostr << b.m_id << '\n';
    ostr << b.m_v << '\n';
    ostr << b.m_a << '\n';
    ostr << b.m_c << '\n';
    ostr << b.m_f << '\n';
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
    istr >> b.m_f;
    istr >> b.m_I;
    istr >> b.m_X_base;
    istr >> b.m_isVirtual;
    
    return istr;
}

#endif


