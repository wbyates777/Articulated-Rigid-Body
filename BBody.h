/* BBody 20/02/2024

 $$$$$$$$$$$$$$$
 $   BBody.h   $
 $$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Describes all properties of a single body
 
 A BBody contains information about mass, the location of its center of
 mass, and the ineria tensor at the center of mass. This class is
 designed to use the given information and transform it such that it can
 directly be used by the spatial algebra.
 
 gyration_radii used to set the main diagonal of inertia tensor 
 see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
 also see https://en.wikipedia.org/wiki/Radius_of_gyration
 
 Observe body i (Bi) is in A_i but joint i (J_i) is not
 
*/


#ifndef __BBODY_H__
#define __BBODY_H__

#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif

#ifndef __BSPATIALINERTIA_H__
#include "BSpatialInertia.h"
#endif


class BBody
{
    
public:
    
    BBody( void )=default; 
    
    BBody( BScalar mass,
           const BVector3 &com,
           const BVector3 &gyration_radii, 
           bool isVirtual = false );

    BBody( BScalar mass,
           const BVector3 &com,
           const BMatrix3 &inertia_C,
           bool isVirtual = false );
    
    ~BBody( void )=default;
    
    void
    init( void );
    
    void
    set( BScalar mass, const BVector3 &com, const BMatrix3 &inertia_C, bool isVirtual = false );
    
    void 
    mass( BScalar m ) { m_mass = m; }
    
    BScalar 
    mass( void ) const { return m_mass; }

    // com - centre of mass
    void
    com( const BVector3& c ) { m_com = c; }
    
    const BVector3& 
    com( void ) const { return m_com; }
    
    void 
    inertia( const BMatrix3& I ) { m_inertia = I; }
    
    const BMatrix3& 
    inertia( void ) const { return m_inertia; } 
    
    bool
    isVirtual( void ) const { return m_isVirtual; }
    
    // transformation from the base to this body's reference frame - set dynamically
    const BSpatialTransform& 
    X_base( void ) const { return m_X_base; }
    
    void
    X_base( const BSpatialTransform &b )  { m_X_base = b; }
    
    void 
    join( const BSpatialTransform &transform, const BBody &other_body );
    
    void 
    separate( const BSpatialTransform &transform, const BBody &other_body );
    
    
    // spatial velocity $v_i$ of body $i (see RBDA, Section 7.3, equation 7.34)
    const BSpatialVector&
    v( void ) const { return m_vel; } 

    BSpatialVector&
    v( void ) { return m_vel; } 
    
    void
    v( const BSpatialVector &sv ) { m_vel = sv; } 
    
    
    // spatial acceleration $a_i$ of body $i$ (see RBDA, Section 7.3, equation 7.31)
    const BSpatialVector&
    a( void ) const { return m_acc; } 
   
    BSpatialVector&
    a( void ) { return m_acc; } 
    
    void
    a( const BSpatialVector &sv ) { m_acc = sv; } 
    
    
    // spatial interia of body $I_i$ (see RBDA, Section 7.1) 
    const BSpatialInertia& 
    I( void ) const { return m_I; }
    
    void  
    I( const BSpatialInertia& i ) { m_I = i; }

    
    // internal forces on the body (used only BDynamics::inverse())
    // $f_i$ the net force acting on body $i$ (see RBDA, equation 5.9)
    const BSpatialVector& 
    f( void ) const { return m_f; }
    
    BSpatialVector& 
    f( void ) { return m_f; }
    
    void  
    f( const BSpatialVector& v ) { m_f = v; }
   
    
    //  friend std::ostream&
    //  operator<<( std::ostream& ostr, const BBody &b );
    
private:

    /**
     * @brief inertiaToBodyFrame transform the inertia of initial_body to the frame of a target_body
     * This is done in 3 steps:
     *  1. Transform the inertia from the initial body origin to it's COM
     *  2. Rotate the inertia of the given transform
     *  3. Transform inertia of initial_body of the given transform
     * @param transform the transformation from tje target body to the initial body 
     * @return the inertia of initial body transformed
     */
    const BMatrix3 
    inertiaToBodyFrame(const BSpatialTransform &transform, const BBody &initial_body) const;
    
    inline const BMatrix3 
    parallelAxis( const BMatrix3 &inertia, BScalar mass, const BVector3 &com ) const;

    // transformation from the base to this body's reference frame - set dynamically
    BSpatialTransform m_X_base;

    BSpatialInertia m_I;    // spatial inertia of the body 
    BSpatialVector  m_vel;  // spatial velocity of the body
    BSpatialVector  m_acc;  // spatial acceleration of the body

    BSpatialVector m_f;     // internal forces on the body (used only BDynamics::inverse())

    
    BMatrix3 m_inertia;  // inertia matrix at the center of mass 
    BVector3 m_com;      // position of the center of mass in body coordinates
    
    BScalar  m_mass;

    bool m_isVirtual;
};


inline std::ostream&
operator<<( std::ostream& ostr, const BBody &b )
{
    ostr << b.X_base() << '\n';

    ostr << b.v() << '\n';
    ostr << b.a() << '\n';
    
    ostr << b.inertia() << '\n';
    ostr << b.com() << '\n';
    ostr << b.mass() << '\n';

    return ostr;
}




#endif


