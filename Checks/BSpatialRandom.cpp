/* BSpatialRandom 07/04/2026

         $$$$$$$$$$$$$$$$$$$$$$$$$$
         $   BSpatialRandom.cpp   $
         $$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 
 ---
 
*/


#ifndef BSPATIALRANDOM_H
#include "BSpatialRandom.h"
#endif


#ifndef URAND_H
#include "URand.h"
#endif

#ifndef BDYNAMICS_H
#include "BDynamics.h"
#endif

namespace arb 
{
    
    const double RANGE     = 10.0;
    const double ZERO_PROB = 0.95;

    
    URand global_ran(22);
    
    // set the seed and reinitialise the sequence
    void
    rndSeed( unsigned int s )  { return global_ran.seed(s); }
    
    int
    rndInt( int N ) { return global_ran.rndInt(N); }
    
    int
    rndInt( int min, int max ) { return global_ran.rndInt(min, max); }
    
    double 
    rndFloat( void ) { return global_ran.rndFloat(); }
    
    double 
    rndFloat( double min, double max ) { return global_ran.rndFloat(min, max); }
    

    
    //
    //
    //
    
    BVector3 
    rndVec3( void ) 
    {
        BVector3 R;
        for(int i = 0; i < 3; ++i)
        {
            if (rndFloat() < ZERO_PROB)
                R[i] = rndFloat(-RANGE, RANGE);
            else R[i] = 0.0;
        }
        return R;
    }
    
    BVector3 
    rndVec3( double min, double max ) 
    {
        BVector3 R;
        for(int i = 0; i < 3; ++i)
        {
            R[i] = rndFloat((double) min, (double) max);
        }
        return R;
    }
        
    BVector3
    rndAxis( void )
    {
        int axis = rndInt(3);
        if (axis == 0) 
            return B_XAXIS;
        else if (axis == 1)
            return B_YAXIS;
        else return B_ZAXIS;
    }
    
    BVector3
    rndNormal3( void )
    {
        BVector3 R = rndVec3();
        BScalar len = arb::length(R);
        return (len < B_EPS) ? rndAxis() : R / len;
    }

    BVector6
    rndVec6( void ) 
    {
        BVector6 R;
        for(int i = 0; i < 6; ++i)
        {
            if (rndFloat() < ZERO_PROB)
                R[i] = rndFloat(-RANGE, RANGE);
            else R[i] = 0.0;
        }
        return R;
    }
    
    BMatrix3 
    rndMat3( void ) 
    {
        BMatrix3 R;
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                if (rndFloat() < ZERO_PROB)
                    R[i][j] = rndFloat(-RANGE, RANGE);
                else R[i][j] = 0.0;
            }
        }
        return R;
    }
    
    BMatrix6  
    rndMat6( void ) 
    {
        BMatrix6 R;
        for(int i = 0; i < 6; ++i)
        {
            for(int j = 0; j < 6; ++j)
            {
                if (rndFloat() < ZERO_PROB)
                    R[i][j] = rndFloat(-RANGE, RANGE);
                else R[i][j] = 0.0;
            }
        }
        return R;
    }

    BMatrix63  
    rndMat63( void ) 
    {
        return BMatrix63(arb::rndMat3(), arb::rndMat3());
    }

    BMatrix3 
    rndRot( void ) 
    {
        BScalar theta = rndFloat(-M_PI, M_PI);
        BVector3 axis = rndNormal3();
        return glm::mat3_cast(glm::angleAxis(theta, axis));
    }
    
    BQuat 
    rndQuat( void )  { return glm::quat_cast(rndRot()); }
    
    
    BTransform
    rndTransform( void )
    {
        BMatrix3 E = rndRot();
        BVector3 r = rndVec3();
        BTransform X(E,r);
        
        return X;
    }

    BInertia 
    rndInertia( void )
    // more work needed here
    {
        BMatrix3 I(B_ZERO_3x3);
        
        BScalar mass = arb::rndFloat(1.0, 100.0);
        BVector3 diag = rndVec3(0.0, RANGE);
        
        I[0][0] = diag[1] + diag[2];
        I[1][1] = diag[0] + diag[2];
        I[2][2] = diag[0] + diag[1];
  
        BInertia inertia(mass, I);
    
        inertia.transform(rndRot(), rndVec3());
 
        assert(inertia.valid());
        return inertia;
    }
    
    BRBInertia 
    rndRBInertia( void )
    {
        BInertia I = rndInertia();
        BRBInertia rbi = BRBInertia(I); 
        
        return rbi;
    }
    
    BABInertia 
    rndABInertia( void )
    {
        BInertia I = rndInertia();
        BABInertia abi(I);
        
        return abi;
    }


    // BModelState generation parameters for positions, velocities, accelerations, and forces
    struct BModelStateParameters
    {
        double LOW_p = -5.0;
        double HIGH_p = 5.0;
        
        double LOW_v = -5.0;
        double HIGH_v = 5.0;
        
        double LOW_a = -5.0;
        double HIGH_a = 5.0;
        
        double LOW_t = -5.0;
        double HIGH_t = 5.0;
    };

    // Generate a non-zero random initial state for model
    BModelState
    rndModelState( const BModel &m )
    {
        BModelState qstate(m);
        BModelStateParameters params;

        for (int i = 0; i <  m.jointNum(); ++i)
        {
            const BJoint &joint = m.joint(i);
            int idx = joint.qindex();
            
            // recover parameter values from joint; override any default values.
            double lo_p =  joint.params().limit.lower_limit;
            double hi_p =  joint.params().limit.upper_limit;
            if (lo_p == 0) lo_p = params.LOW_p;
            if (hi_p == 0) hi_p = params.HIGH_p;
    
            double lo_v =  -joint.params().limit.velocity_limit;
            double hi_v =   joint.params().limit.velocity_limit;
            if (lo_v == -1.0) lo_v = params.LOW_v;
            if (hi_v == 1.0) hi_v  = params.HIGH_v;
    
            double lo_a = params.LOW_a;
            double hi_a = params.HIGH_a;
        
            double lo_t =  -joint.params().limit.effort_limit;
            double hi_t =   joint.params().limit.effort_limit;
            if (lo_t == -1.0) lo_t = params.LOW_t;
            if (hi_t == 1.0) hi_t  = params.HIGH_t;
            
            switch(joint.jtype())
            {
                case BJoint::Helical:    // 1-Dof joints
                case BJoint::Revolute:
                case BJoint::RevoluteX:
                case BJoint::RevoluteY:
                case BJoint::RevoluteZ:
                case BJoint::Prismatic:
                {
                    qstate.q()[idx]     = arb::rndFloat(lo_p, hi_p);
                    qstate.qdot()[idx]  = arb::rndFloat(lo_v, hi_v);
                    qstate.qddot()[idx] = arb::rndFloat(lo_a, hi_a);
                    qstate.tau()[idx]   = arb::rndFloat(lo_t, hi_t);
                    break;
                }
                case BJoint::Planar:     // 3-Dof joints
                {
                    qstate.q()[idx]      = arb::rndFloat(lo_p, hi_p);  // x
                    qstate.q()[idx+1]    = arb::rndFloat(lo_p, hi_p);  // y
                    qstate.q()[idx+2]    = arb::rndFloat(-M_PI, M_PI); // theta
                    
                    qstate.qdot()[idx]   = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+1] = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+2] = arb::rndFloat(lo_v, hi_v);
                    
                    qstate.qddot()[idx]   = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+1] = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+2] = arb::rndFloat(lo_a, hi_a);
                    
                    qstate.tau()[idx]     = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+1]   = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+2]   = arb::rndFloat(lo_t, hi_t);
                    break;
                }
                case BJoint::EulerXYZ:
                case BJoint::EulerZYX:
                case BJoint::EulerZXY:
                case BJoint::EulerYXZ:
                {
                    qstate.q()[idx]   = arb::rndFloat(-M_PI, M_PI);
                    qstate.q()[idx+1] = arb::rndFloat(-M_PI, M_PI);
                    qstate.q()[idx+2] = arb::rndFloat(-M_PI, M_PI);
                    
                    qstate.qdot()[idx]   = arb::rndFloat(-M_PI, M_PI);
                    qstate.qdot()[idx+1] = arb::rndFloat(-M_PI, M_PI);
                    qstate.qdot()[idx+2] = arb::rndFloat(-M_PI, M_PI);
                    
                    qstate.qddot()[idx]   = arb::rndFloat(-M_PI, M_PI);
                    qstate.qddot()[idx+1] = arb::rndFloat(-M_PI, M_PI);
                    qstate.qddot()[idx+2] = arb::rndFloat(-M_PI, M_PI);
                    
                    qstate.tau()[idx]     = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+1]   = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+2]   = arb::rndFloat(lo_t, hi_t);
                    break;
                }
                case BJoint::Spherical:
                {
                    joint.setQuat(arb::rndQuat(), qstate.q());
                    
                    qstate.qdot()[idx]   = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+1] = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+2] = arb::rndFloat(lo_v, hi_v);
                    
                    qstate.qddot()[idx]   = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+1] = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+2] = arb::rndFloat(lo_a, hi_a);
                    
                    qstate.tau()[idx]     = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+1]   = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+2]   = arb::rndFloat(lo_t, hi_t);
                    
                    break;
                }
                case BJoint::TransXYZ:
                {
                    qstate.q()[idx]   = arb::rndFloat(lo_p, hi_p); // x
                    qstate.q()[idx+1] = arb::rndFloat(lo_p, hi_p); // y
                    qstate.q()[idx+2] = arb::rndFloat(lo_p, hi_p); // z
                    
                    qstate.qdot()[idx]   = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+1] = arb::rndFloat(lo_v, hi_v);
                    qstate.qdot()[idx+2] = arb::rndFloat(lo_v, hi_v);
                    
                    qstate.qddot()[idx]   = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+1] = arb::rndFloat(lo_a, hi_a);
                    qstate.qddot()[idx+2] = arb::rndFloat(lo_a, hi_a);
                    
                    qstate.tau()[idx]     = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+1]   = arb::rndFloat(lo_t, hi_t);
                    qstate.tau()[idx+2]   = arb::rndFloat(lo_t, hi_t);
                    
                    break;
                }
                case BJoint::J1DoF:
                case BJoint::J2DoF:
                case BJoint::J3DoF:
                case BJoint::J4DoF:
                case BJoint::J5DoF:
                case BJoint::J6DoF:
                case BJoint::FloatBase:
                case BJoint::Fixed1:
                case BJoint::Fixed2:
                case BJoint::UNDEFINED:
                case BJoint::MAXJOINT:
                    /* do nothing */
                    break;
            }
        }
        return qstate;
    }

    // Generate a random tree-structured model of bodyNum bodies and joints.
    BModel
    rndModel( int bodyNum )
    {
        BModel model;
        
        // gravity ON/OFF
        if (arb::rndFloat() > 0.5)
        {
            model.gravity( BVector3(0.0, -9.81, 0.0));
        }

        BBodyId parent_id = 0;
        
        // add a floating base
        if (arb::rndFloat() > 0.75)
        {
            // base
            BInertia I = arb::rndInertia();
            I.set(arb::rndFloat(50.0,80.0), I.I()); // make the base heavy
            BBody body(I);
            parent_id = model.addBody(0, B_IDENTITY_TRANS, BJoint::FloatBase, body);
        }

        while (model.bodyNum() <  bodyNum)
        {
      
            if (!model.body(parent_id).isVirtual())
            {
                // inertias with com far from the frame origin can cause problems
                BInertia I = arb::rndInertia();
                // large mass ratios between bodies can causes problems
                I.set(arb::rndFloat(8.0, 12.0), I.I());
                BBody body( I );
                
                BJoint joint;
                int jtype = arb::rndInt(17);
                switch (jtype)
                {
                    case 0 :  joint = BJoint(BJoint::Prismatic, arb::rndNormal3()); break;
                    case 1 :  joint = BJoint(BJoint::Revolute,  arb::rndNormal3()); break;
                    case 2 :  joint = BJoint(BJoint::Prismatic, arb::rndAxis());    break;
                    case 3 :  joint = BJoint(BJoint::Revolute,  arb::rndAxis());    break;
                    case 4 :  joint = BJoint(BJoint::RevoluteX); break;
                    case 5 :  joint = BJoint(BJoint::RevoluteY); break;
                    case 6 :  joint = BJoint(BJoint::RevoluteZ); break;
                    case 7 :  joint = BJoint(BJoint::Fixed1);    break;
                    case 8 :  joint = BJoint(BJoint::Fixed2);    break;
                       
                    case 9 :  joint = BJoint(BJoint::Planar);    break;
                    case 10 : joint = BJoint(BJoint::Spherical); break;
                    case 11 : joint = BJoint(BJoint::TransXYZ);  break;
          
                        
                    case 12 : joint = BJoint(BJoint::EulerXYZ);  break;
                    case 13 : joint = BJoint(BJoint::EulerZYX);  break;
                    case 14 : joint = BJoint(BJoint::EulerYXZ);  break;
                    case 15 : joint = BJoint(BJoint::EulerZXY);  break;
                        
                    case 16 : joint = BJoint(BVector6(arb::rndAxis(), arb::rndAxis())); break; // Helix
                        
                    default:  joint = BJoint(BJoint::Revolute,  arb::rndAxis()); break;
                }
                
                // make sure the first body is not a Fixed1 joint
                if (model.bodyNum() == 1 && joint.jtype() == BJoint::Fixed1)
                    joint = BJoint(BJoint::Fixed2);
                
                model.addBody(parent_id,  arb::rndTransform(), joint, body);
            }
            parent_id = arb::rndInt(1, (int)model.bodyNum());
        }
       
        return model;
    }



    // for some accelerations qddot, test whether the identity
    // ABA(q, qdot, RNEA(q, qdot, qddot)) ≈ qddot is true.
    bool
    isvalid( BModel &model, int Ntrials, double tol )
    {
        using std::abs;

        BDynamics dynamics;
        bool match = true;

        for (int trial = 0; trial < Ntrials; ++trial) // 40 trials to be (more) sure
        {
            // create an initial qstate and save the accelerations
            BModelState qstate = arb::rndModelState(model);
            std::vector<BScalar> qddot_target = qstate.qddot();

            // inverse dynamics (RNEA) - calculate forces tau
            dynamics.inverse( model, qstate );
            
            // keep the calculated tau, set output qstate.qddot to zero - just to be sure
            std::vector<BScalar> tau_calculated = qstate.tau();
            qstate.qddot().assign(model.qdotsize(), 0.0);
            
            // forward dynamics (ABA) - calculate accelerations 'qddot' using 'tau'
            dynamics.forward( model, qstate );
            
            // compare qstate.m_qddot with qddot_target
            for (int i = 0; i < qstate.qddot().size(); ++i)
            {
                BScalar err = abs(qstate.qddot()[i] - qddot_target[i]);
                
                if (err > tol)
                {
                    match = false;
                    break;
                }
            }
            
            // cut short the test
            if (match == false)
                break;
        }
        
        return match;
    }
}



