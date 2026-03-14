/* CBody 10/02/2026

 $$$$$$$$$$$$$$$
 $   CBody.h   $
 $$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A demo 'collidable body' class.
 
 Its a demo because of the way the BCollider/Polytope and the CBody 'this' pointer are handled. 
 This implementation works, but I think it could be improved upon. 
 
 
*/


#ifndef __CBODY_H__
#define __CBODY_H__


#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BBODY_H__
#include "BBody.h"
#endif


class CBody : public ABody
{

public:

    CBody( void );
    CBody( BBodyId bid, const BBody &body, const BBox &box );
    ~CBody( void )=default;

    CBody( const CBody &rhs ); 

    CBody&
    operator=( const CBody &rhs );
    
    void
    set( BBodyId bid, const BBody &body, const BBox &box );
    
    
    BBodyId
    objId( void ) const override { return m_id; }
    
    
    //
    // object position and orientation (for a BModel use m_body[2].X_base())
    //
    const BVector3&
    pos( void ) const override { return m_body.X_base().r(); }
    
    BVector3&
    pos( void ) override { return m_body.X_base().r(); }
    
    void
    pos( const BVector3 &p ) override { m_body.X_base().r(p); }
    
    
    const BMatrix3&
    orient( void ) const override { return m_body.X_base().E(); }
   
    BMatrix3&
    orient( void ) override { return m_body.X_base().E(); }
    
    void
    orient( const BMatrix3 &q ) override { m_body.X_base().E(q); }
    
 
    BBody&
    body( void ) { return m_body; }
    
    const BBody&
    body( void ) const { return m_body; }
    
    //
    // spatial velocity interface 
    //
    const BVector6&
    v( void ) const override { return m_body.v(); }
    
    BVector6&
    v( void ) override { return m_body.v(); } 
    
    void
    v( const BVector6 &v ) override { m_body.v(v); }  
    
    //
    // spatial inertia
    //
    const BRBInertia& 
    I( void ) const override { return m_body.I(); }
    
    virtual const BRBInertia&
    I_base( void) const override { return m_I_base; } 
    
    const BMatrix6& 
    invI( void ) const override { return m_invI; }
 
    const BMatrix6&
    invI_base( void) const override  { return m_invI_base; }  

    //
    // update state
    //
    void
    updateVelocity( double dt );
      
    void
    updatePosition( double dt );
    
    //
    // OBB - bounding box in model coords
    //
    const BBox&  
    box( void ) const override  { return m_box; }  
     
    BBox& 
    box( void ) override { return m_box; }  
    
    void
    setBox( const BBox &b ) override { m_box = b; }  
    
    //
    // force inerface -- apply torque/force to this rigid body
    //
    void
    allStop( void );
    
    void
    allStopLin( void );
    
    void
    allStopAng( void );
    
    BVector6&
    force( void ) { return m_force; } 
    
    const BVector6&
    force( void ) const { return m_force; } 
    
    void
    force( const BVector6 &f ) { m_force = f; } 
    
    BScalar&
    force( int i ) { return m_force[i]; } 
    
    BScalar
    force( int i ) const { return m_force[i]; } 
    
    BScalar&
    delta( void ) { return m_delta; } 
    
    BScalar
    delta( void ) const { return m_delta; } 
    //
    
    
    //
    // collsions
    //
    BCollider& 
    collider( void ) override { return m_collider; };
    
    const BCollider& 
    collider( void ) const override { return m_collider; };

    
private:

    void 
    updateBaseInertia( void );

    void 
    updateOrient(double dt, glm::dquat &q, const glm::dvec3 &w) const;
    
    BBodyId     m_id; 
    BBox        m_box;
    BBody       m_body;

    BRBInertia  m_I_base;     // inertia in base coordinates; changes as body rotates
    BMatrix6    m_invI_base;  // inverse inertia in base coordinates; changes as body rotates
    BMatrix6    m_invI;       // inverse inertia
    
    glm::dquat  m_orient;     // orientation is stored as a quaternion (for integration)
    
    BVector6    m_force;
    BScalar     m_delta;
    
    BSphereCollider m_collider;
};


#endif


