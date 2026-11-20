/* BIntegrator 22/10/2026

         $$$$$$$$$$$$$$$$$$$$$$$
         $   BIntegrator.cpp   $
         $$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates


*/


#ifndef BINTEGRATOR_H
#include "BIntegrator.h"
#endif

#ifndef BMODEL_H
#include "BModel.h"
#endif

#ifndef BMODELSTATE_H
#include "BModelState.h"
#endif



#ifndef BEXPONENTIAL_H
#include "BExponential.h"
#endif


#include <iostream>

//
// Single body - assume usser does own limit checks
//

void
BIntegrator::updateVel( BBody &body, double dt ) const
// single body update velocity in *world/base coords*
{
    body.v() += body.a() * dt;
}

void
BIntegrator::updatePos( BBody &body, double dt ) const
// single body update position (X_base) in *world/base coords*
{
    const BVector3 w_dt = body.v().ang() * BScalar(dt);
    body.X_base().E() = arb::exp(-w_dt) * body.X_base().E();
    body.X_base().r() += body.v().lin() * BScalar(dt);
}

//
// Multibody models
//

void
BIntegrator::updateVel( const BModel &model, BModelState &qstate, double dt ) const
// multibody update velocity in *local/body coords*
{
    const int N = (int) model.DoFCount();
    const std::vector<BJoint> &joints = model.joint();
    const std::vector<BScalar> &qddot = qstate.qddot();
    std::vector<BScalar> &qdot = qstate.qdot();

    for (int i = 0; i < N; ++i)
    {
        qdot[i] += qddot[i] * dt;
        
        if (m_limits)
        {
            BJointId jid = model.getJointId(i);
            const BScalar vlim = joints[jid].params().limit.velocity_limit;
            qdot[i] = arb::clamp(qdot[i], -vlim, vlim);
        }
    }
}

void
BIntegrator::updatePos( const BModel &model, BModelState &qstate, double dt ) const
// multibody update position  in *local/body coords*
{
    for (int i = 1; i < model.jointNum(); ++i)
    {
        updatePos( model.joint(i), qstate, dt );
    }
}

void
BIntegrator::updatePos( const BJoint &joint, BModelState &qstate, double dt ) const
// integrate variables and update qstate.q() for a multibody model in *local/body coords*
{
    using std::atan2;
    using std::sin;
    using std::cos;
    
    std::vector<BScalar> &q = qstate.q();             // position
    const std::vector<BScalar> &qdot = qstate.qdot(); // velocity

    const BScalar lo = joint.params().limit.lower_limit;
    const BScalar hi = joint.params().limit.upper_limit;
    
    int idx = joint.qindex();
    
    switch(joint.jtype())
    {
        // joints with 1-DoF
        case BJoint::Helical:
        case BJoint::Revolute:
        case BJoint::RevoluteX:
        case BJoint::RevoluteY:
        case BJoint::RevoluteZ:
        case BJoint::Prismatic:
        {
            q[idx] += qdot[idx] * BScalar(dt);
            if (m_limits)
                q[idx] = arb::clamp(q[idx], lo, hi);
            break;
        }
        // joints with 3-DoF
        case BJoint::EulerZYX:
        case BJoint::EulerXYZ:
        case BJoint::EulerYXZ:
        case BJoint::EulerZXY:
        {
            for (int i = 0; i < 3; ++i)
                q[idx + i] += qdot[idx + i] * BScalar(dt);
            
            if (m_limits)
                for (int i = 0; i < 3; ++i)
                    q[idx + i] = arb::clamp(q[idx + i], lo, hi);
            
            break;
        }
        // joints with 3-DoF with no limits (used for bases)
        case BJoint::Planar:
        case BJoint::TransXYZ:
        {
            for (int i = 0; i < 3; ++i)
                q[idx + i] += qdot[idx + i] * BScalar(dt);
            
            break;
        }
        case BJoint::Spherical:
        {
            // angular velocity ω
            BVector3 omega(qdot[idx], qdot[idx+1], qdot[idx+2]);
            
            // update
            BQuat orient = joint.getQuat(q);
            orient = orient * glm::quat_cast(arb::exp(-omega * BScalar(dt))); // body coordinates
            orient = arb::normalize(orient);
            joint.setQuat(orient, q);
            
            break;
        }

        case BJoint::Fixed1:
        case BJoint::Fixed2:
        case BJoint::FloatBase:
        case BJoint::UNDEFINED:
            /* do nothing */
            break;
        default:
            std::cout << "BIntegrator::Bad Joint" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }
}


