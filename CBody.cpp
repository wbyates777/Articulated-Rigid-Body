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


#ifndef CBODY_H
#include "CBody.h"
#endif

#ifndef BADJOINT_H
#include "BAdjoint.h"
#endif

#ifndef BSPATIALTYPES_H
#include "BSpatialTypes.h"
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
                        m_collider(),
                        m_integrate() {}


CBody::CBody( BBodyId bid, const BBody &body, const BBox &box ) : ABody(), 
                                                                  m_id(bid), 
                                                                  m_box(box),
                                                                  m_body(body),
                                                                  m_I_base(B_ZERO_RBI),
                                                                  m_invI_base(B_IDENTITY_6x6),
                                                                  m_invI(B_IDENTITY_6x6),
                                                                  m_orient(B_IDENTITY_QUAT),     
                                                                  m_force(B_ZERO_6),
                                                                  m_delta(0.25),
                                                                  m_collider(),
                                                                  m_integrate()
{
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
CBody::updateVelocity(double dt) 
// convert user input force (in body coords) to world coordinate frame
// trasform direction of *each* component separately
// X(E) transforms both together which produces torques we do not want.
{
    // forward - calculate accelerations in world/base coordinates
    const BVector6 force( m_body.X_base().E() *  m_force.ang(), m_body.X_base().E() * m_force.lin());
    m_body.a() = m_invI_base * ((force * (m_delta + 1.0)) - arb::crossf(m_body.v(), m_I_base * m_body.v()));
   
    m_integrate.updateVel(m_body, dt);
}

void 
CBody::updatePosition(double dt) 
// update m_orient and then X_base
{
    // applying a small multiplier (e.g., 0.99) after collision resolution
    // helps simulate air resistance/numerical drag - NB std::pow is expensive
    // m_v.ang( m_v.ang() * std::pow(0.95, dt) ); 
    // m_v.lin( m_v.lin() * std::pow(0.99, dt) );
    
    m_integrate.updatePos(m_body, dt);
    
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


