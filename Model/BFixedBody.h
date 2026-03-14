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


#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif

#ifndef __BBODY_H__
#include "BBody.h"
#endif


class BFixedBody 
{
public:
    
    BFixedBody( void )=default;
    BFixedBody( const BBody &b ):  m_id(0), m_parentId(0), 
                                   m_inertia(b.I().mass(), b.I().com(), b.I().Icom()), 
                                   m_parent(B_IDENTITY_TRANS), m_base(B_IDENTITY_TRANS) {} 
    ~BFixedBody( void )=default;
  
    
    void
    setId( BBodyId bid ) { m_id = bid; }
    
    BBodyId
    getId( void ) const { return m_id; }
    
    
    BBodyId
    movableParent( void ) const { return m_parentId; }
    
    void
    movableParent( BBodyId bid ) { m_parentId = bid; }
    

    BScalar 
    mass( void ) const { return m_inertia.mass(); }
    
    BVector3 
    com( void ) const { return m_inertia.com(); }
    
 
    BMatrix3 
    inertiaCom( void ) const { return m_inertia.Icom(); } 
    
    void 
    set(const BInertia &I) { m_inertia = I; }

    
    BBody 
    toBody( void ) const { return BBody(m_inertia); }
    
    
    const BTransform& 
    parentTrans( void ) const { return m_parent; }
    
    void 
    parentTrans( const BTransform &pt )  { m_parent = pt; }
    
    const BTransform& 
    baseTrans( void ) const { return m_base; }
    
    void 
    baseTrans( const BTransform &bt ) { m_base = bt; }
    
    

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
    BBodyId m_parentId;    // bid of the movable body that this fixed body is attached to.
    
    BInertia  m_inertia;

    // transforms spatial quantities expressed for the parent to the fixed body.
    BTransform m_parent; // m_X_lambda - the transformation from the parent body frame $\lambda(i)$ to body $i$ 
    BTransform m_base;   // m_X_base   - transformation from the base  to this body's coordinate
};

inline std::ostream&
operator<<( std::ostream &ostr, const BFixedBody &b )
{
    ostr << b.m_id << ' ';
    ostr << b.m_parentId << '\n';
    
    ostr << b.m_inertia << '\n';

    
    ostr << b.m_parent << '\n';
    ostr << b.m_base << '\n';
 
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BFixedBody &b )
{
    istr >> b.m_id;
    istr >> b.m_parentId;
    
    istr >> b.m_inertia;
   
    istr >> b.m_parent;
    istr >> b.m_base;
    
    return istr;
}

#endif


