/* CBody 10/02/2026

 $$$$$$$$$$$$$$$$$
 $   CBody.cpp   $
 $$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 For more details on use of quaternion exponential maps for integration see Solà, J. (2017),
 "Quaternion kinematics for the error-state Kalman filter", Section 2.3.1, eqn 69.
 
 More accurate than the usual linear integration model:
    m_orient += (m_orient * glm::dquat(0.0, angVel)) * (0.5 * dt); 
    m_orient = glm::normalize(m_orient); 
 
 
*/


#ifndef __CBODY_H__
#include "CBody.h"
#endif

#ifndef __BADJOINT_H__
#include "BAdjoint.h"
#endif

#include <iostream>

CBody::CBody( void )  : ABody(), 
                        m_id(0), 
                        m_box(),
                        m_body(),
                        m_I_base(B_ZERO_RBI),
                        m_invI_base(B_IDENTITY_6x6),
                        m_invI(B_IDENTITY_6x6),
                        m_orient(B_IDENTITY_QUAT),     
                        m_force(B_ZERO_6),
                        m_delta(0.25),
                        m_collider(this) {}

CBody::CBody( BBodyId bid, const BBody &body, const BBox &box )  :  ABody(), 
                                                                    m_id(0), 
                                                                    m_box(),
                                                                    m_body(),
                                                                    m_I_base(B_ZERO_RBI),
                                                                    m_invI_base(B_IDENTITY_6x6),
                                                                    m_invI(B_IDENTITY_6x6),
                                                                    m_orient(B_IDENTITY_QUAT),     
                                                                    m_force(B_ZERO_6),
                                                                    m_delta(0.25),
                                                                    m_collider(this)
{
    set( bid, body, box );
}

// NB we provdie copy contructors because m_collider has a copy to 'this' object

CBody::CBody( const CBody &rhs ) :  ABody(rhs), 
                                    m_id(rhs.m_id), 
                                    m_box(rhs.m_box),
                                    m_body(rhs.m_body),
                                    m_I_base(rhs.m_I_base),
                                    m_invI_base(rhs.m_invI_base),
                                    m_invI(rhs.m_invI),
                                    m_orient(rhs.m_orient),     
                                    m_force(rhs.m_force),
                                    m_delta(rhs.m_delta),
                                    m_collider() 
{
    m_collider.setPoints(rhs.collider().getPoints());
    m_collider.setBody(this);
}

CBody&
CBody::operator=( const CBody& rhs )
{
    if (&rhs == this)
        return *this;
    
    m_id        = rhs.m_id; 
    m_box       = rhs.m_box;
    m_body      = rhs.m_body;
    m_I_base    = rhs.m_I_base;
    m_invI_base = rhs.m_invI_base;
    m_invI      = rhs.m_invI;
    m_orient    = rhs.m_orient;     
    m_force     = rhs.m_force;
    m_delta     = rhs.m_delta;

    m_collider.setPoints(rhs.collider().getPoints());
    //m_collider.setBody(this);
    
    return *this;
}

void
CBody::set( BBodyId bid, const BBody &body, const BBox &box )
{
    m_id     = bid;
    m_body   = body;
    m_box    = box;
    
    m_orient = glm::quat_cast(m_body.X_base().E()); // sync m_orient and X_base
    m_invI   = arb::inverse(m_body.I()); 
    
    updateBaseInertia();
}

void 
CBody::updateBaseInertia( void )  
// convert 'body coordinate' spatial inertia I to 'base coordinates' I_base, and 
// convert 'body coordinate' inverse spatial inertia I^{-1} to 'base coordinates' I^{-1}_base 
// Note in a rotating body I is constant while I_base is changing 
// see RBDA, Table 2.5 and eqn 2.66, page 34

{ 
    const BTransform X(m_body.X_base().E()); // world to body
    m_I_base = X.applyTranspose(m_body.I());
    m_invI_base = arb::inverse(X) * m_invI * arb::toForce(X);
}

void 
CBody::updateOrient(double dt, glm::dquat &q, const glm::dvec3 &w) const
// integrate angular velocity ω over Δt to update the orientation quaternion q 
// quat exponential map more accurate than the usual linear integration (Solà, 2017)
// employs Taylor expansion for small angles to maintain numerical stability
{
    // w - angular velocity
    const glm::dvec3 v   = w * dt;
    const double theta_sq = glm::dot(v, v);

    glm::dquat dq;

    if (theta_sq < 1.0E-8) 
    {
        // if theta -> 0, then sin(theta*0.5)/theta -> 0.5 (Taylor expansion)
        dq = glm::dquat(1.0, v * 0.5);
    }
    else 
    {
        // dq = exp(v/2)
        const double theta = std::sqrt(theta_sq);
        const double half_theta = theta * 0.5;
        const double s = std::sin(half_theta) / theta;
        dq = glm::dquat(std::cos(half_theta), v * s);
    }

    // normalize just to be sure
    //q = glm::normalize(q * dq); // in body coords
    q = glm::normalize(dq * q);   // in base/world coords
}

void
CBody::updateVelocity(double dt) 
// convert user input force (in base coords) to body coordinate frame
// trasform direction of *each* component separately 
// X(E) transforms both together which produces torques we do not want.
{
    const BVector6 force( m_body.X_base().E() *  m_force.ang(), m_body.X_base().E() * m_force.lin());
    m_body.a() = m_invI_base * ((force * (m_delta + 1.0)) - arb::crossf(m_body.v(), m_I_base * m_body.v()));
    m_body.v() += m_body.a() * dt;
}

void 
CBody::updatePosition(double dt) 
// update m_orient and then X_base
{
    // applying a small multiplier (e.g., 0.99) after collision resolution
    // helps simulate air resistance/numerical drag - NB std::pow is expensive
    // m_v.ang( m_v.ang() * std::pow(0.95, dt) ); 
    // m_v.lin( m_v.lin() * std::pow(0.99, dt) );
    
    m_body.X_base().r() += m_body.v().lin() * BScalar(dt);
    updateOrient( dt, m_orient, m_body.v().ang() );
    m_body.X_base().E( glm::mat3_cast(m_orient) );  // sync m_orient and X_base
    updateBaseInertia(); // ensure m_X_base set first
    m_force = B_ZERO_6;
}


void
CBody::allStop( void )   
{ 
    m_force  = B_ZERO_6; 
    m_body.a(B_ZERO_6);
    m_body.v(B_ZERO_6);
    m_body.c(B_ZERO_6);
}

void
CBody::allStopLin( void ) 
{ 
    m_force.lin(B_ZERO_3);
    m_body.a().lin(B_ZERO_3);
    m_body.v().lin(B_ZERO_3);
    m_body.c().lin(B_ZERO_3);
}

void
CBody::allStopAng( void ) 
{ 
    m_force.ang(B_ZERO_3);
    m_body.a().ang(B_ZERO_3);
    m_body.v().ang(B_ZERO_3);
    m_body.c().ang(B_ZERO_3);
}

//


