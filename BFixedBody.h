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
    BFixedBody( const BBody &body ):  m_mass(body.mass()), m_com(body.com()), m_inertia(body.inertia()), m_movableParent(0), m_parentTransform(BIDENTITY_3x3, BZERO_3), m_baseTransform(BIDENTITY_3x3, BZERO_3) {} 
    ~BFixedBody( void )=default;
    
    
    
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
    
    BBody 
    toBody( void ) const { return BBody(m_mass, m_com, m_inertia); }
    
    
    const BSpatialTransform& 
    parentTrans( void ) const { return m_parentTransform; }
    
    void 
    parentTrans( const BSpatialTransform& pt )  { m_parentTransform = pt; }
    
    const BSpatialTransform& 
    baseTrans( void ) const { return m_baseTransform; }
    
    void 
    baseTrans( const BSpatialTransform& bt ) { m_baseTransform = bt; }
    
    
    BBodyID
    movableParent( void ) const { return m_movableParent; }
    
    void
    movableParent( BBodyID bid ) { m_movableParent = bid; }
    
 
private: 

    BScalar  m_mass;
    BVector3 m_com;      // position of the center of mass in body coordinates
    BMatrix3 m_inertia;  // inertia matrix at the center of mass 
 
    // bid of the movable body that this fixed body is attached to.
    BBodyID m_movableParent;
    // transforms spatial quantities expressed for the parent to the fixed body.
    BSpatialTransform m_parentTransform;
    BSpatialTransform m_baseTransform;
};



#endif


