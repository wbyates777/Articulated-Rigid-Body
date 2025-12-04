/* BFixedBody 16/04/2024

 $$$$$$$$$$$$$$$$$$$$
 $   BFixedBody.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 \brief Keeps the information of a body and how it is attached to another body.

 When using fixed bodies, i.e. a body that is attached to another via a
 fixed joint, the attached body is merged onto its parent. By doing so
 adding fixed joints do not have an impact on runtime.
 
*/


#ifndef __BFIXEDBODY_H__
#define __BFIXEDBODY_H__


#ifndef __BSPATIALTRANSFORM_H__
#include "BSpatialTransform.h"
#endif

#ifndef __BBODY_H__
#include "BBody.h"
#endif


class BFixedBody 
{
public:
    
    BFixedBody( void )=default;
    BFixedBody( const BBody &body ):  m_id(0), m_movableParentId(0), m_mass(body.mass()), m_com(body.com()), m_inertia_com(body.inertiaCom()), m_parentTransform(B_IDENTITY_TRANS), m_baseTransform(B_IDENTITY_TRANS) {} 
    ~BFixedBody( void )=default;
  
    
    void
    setId( BBodyId bid ) { m_id = bid; }
    
    BBodyId
    getId( void ) const { return m_id; }
    
    void 
    setMass( BScalar m ) { m_mass = m; }
    
    BScalar 
    mass( void ) const { return m_mass; }

    // com - centre of mass
    void
    setCom( const BVector3 &c ) { m_com = c; }
    
    const BVector3& 
    com( void ) const { return m_com; }
    
    void 
    setInertiaCom( const BMatrix3& I_com ) { m_inertia_com = I_com; }
    
    const BMatrix3& 
    inertiaCom( void ) const { return m_inertia_com; } 
    
    void 
    setBody( BScalar mass, const BVector3 &com, const BMatrix3 &I_com ) 
    { 
        m_mass = mass;  m_com = com; m_inertia_com = I_com;
    }
    
    BBody 
    toBody( void ) const { return BBody(m_mass, m_com, m_inertia_com); }
    
    
    const BSpatialTransform& 
    parentTrans( void ) const { return m_parentTransform; }
    
    void 
    parentTrans( const BSpatialTransform &pt )  { m_parentTransform = pt; }
    
    const BSpatialTransform& 
    baseTrans( void ) const { return m_baseTransform; }
    
    void 
    baseTrans( const BSpatialTransform &bt ) { m_baseTransform = bt; }
    
    
    BBodyId
    movableParent( void ) const { return m_movableParentId; }
    
    void
    movableParent( BBodyId bid ) { m_movableParentId = bid; }
    
    bool 
    operator==( const BFixedBody &v ) const { return (m_id == v.m_id); }
    
    bool 
    operator!=( const BFixedBody &v ) const { return (m_id != v.m_id); }
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BFixedBody &b );
    
    friend std::istream& 
    operator>>( std::istream &istr, BFixedBody &b );
    
private: 

    BBodyId m_id;
    // bid of the movable body that this fixed body is attached to.
    BBodyId m_movableParentId;
    
    BScalar  m_mass;
    BVector3 m_com;      // position of the center of mass in body coordinates
    BMatrix3 m_inertia_com;  // inertia matrix at the center of mass 
 

    // transforms spatial quantities expressed for the parent to the fixed body.
    
    BSpatialTransform m_parentTransform; // m_X_lambda - the transformation from the parent body frame $\lambda(i)$ to body $i$ 
    BSpatialTransform m_baseTransform; // m_X_base - transformation from the base  to this body's coordinate
};

inline std::ostream&
operator<<( std::ostream &ostr, const BFixedBody &b )
{
    ostr << b.m_id << ' ';
    ostr << b.m_movableParentId << '\n';
    
    ostr << b.m_mass << '\n';
    ostr << b.m_com << '\n';
    ostr << b.m_inertia_com << '\n';
    
    ostr << b.m_parentTransform << '\n';
    ostr << b.m_baseTransform << '\n';
 
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BFixedBody &b )
{
    istr >> b.m_id;
    istr >> b.m_movableParentId;
    
    istr >> b.m_mass;
    istr >> b.m_com;
    istr >> b.m_inertia_com ;
    
    istr >> b.m_parentTransform;
    istr >> b.m_baseTransform;
    
    return istr;
}

#endif


