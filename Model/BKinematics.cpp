/* BKinematics 13/09/2026

 $$$$$$$$$$$$$$$$$$$$$$$
 $   BKinematics.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Kinematics is the study of the geometrical aspects of motion independent of forces. 
 Kinematics differs from dynamics (also known as kinetics) 
 which studies the effect of forces on bodies.
 
 The BKinematics class operates on the kinematic variables: (position, 
 velocity, and acceleration) of a given model.
 
*/


#ifndef __BKINEMATICS_H__
#include "BKinematics.h"
#endif


void 
BKinematics::update_X_base( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates positions 
// based on Kinematics::UpdateKinematicsCustom in RBDL
{
    m_qdot_zero.resize(qstate.qdot().size(), 0.0);
    
    for (int i = 1; i < m.bodyNum(); ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(qstate.q(), m_qdot_zero);
        
        const BTransform &X_lambda = joint.X_lambda(); 
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            body.X_base( X_lambda * m.body(lambda).X_base() );
        else  body.X_base( X_lambda );
    }
}

void 
BKinematics::update_velocity( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates velocities
// based on Kinematics::UpdateKinematicsCustom in RBDL
{
    for (int i = 1; i < m.bodyNum(); ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(qstate.q(), qstate.qdot());
        
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            body.v() = (joint.X_lambda() * m.body(lambda).v()) + joint.v_J();
        else body.v() = joint.v_J();

        body.c() = joint.c_J() + arb::crossm( body.v(), joint.v_J() );
    }
}



BVector6 
BKinematics::v( BModel &m, BBodyId bid, const BVector3 &body_pos ) 
// RBDL Kinematics::CalcPointVelocity6D
{
    BBodyId  ref_bid = bid;
    BVector3 ref_pos = body_pos;
    
    if (m.isFixedBodyId(bid)) 
    {
        BVector3 base_pos = toBasePos(m, bid, body_pos);
        ref_bid = m.fixedBody(bid).parentId();
        ref_pos = toBodyPos(m, ref_bid, base_pos);
    }
    
    BTransform trans( arb::transpose(orient(m, ref_bid)), ref_pos );
    
    return trans.apply(m.body(ref_bid).v());
}

BVector6 
BKinematics::a( BModel &m, BBodyId bid,  const BVector3 &body_pos ) 
// RBDL Kinematics::CalcPointAcceleration6D
{
    BBodyId  ref_bid = bid;
    BVector3 ref_pos = body_pos;
    
    if (m.isFixedBodyId(bid)) 
    {
        BVector3 base_pos = toBasePos(m, bid, body_pos);
        ref_bid = m.fixedBody(bid).parentId();
        ref_pos = toBodyPos(m, ref_bid, base_pos); 
    }
    
    BTransform p_X(arb::transpose(orient(m, ref_bid)), ref_pos);
    BVector6 p_v = p_X.apply(m.body(ref_bid).v());
    BVector3 a_dash = arb::cross(p_v.ang(), p_v.lin());

    return p_X.apply(m.body(ref_bid).a()) + BVector6(B_ZERO_3, a_dash);
}


BVector3  
BKinematics::toBasePos( BModel &m, BBodyId bid,  const BVector3 &body_pos ) 
// returns the base coordinates of a position given in body coordinates.
// see kinematics.cc - Kinematics::CalcBodyToBaseCoordinates
{
    BVector3 pos;
    
    if (m.isFixedBodyId(bid))
    {
        BBodyId parent_id = m.fixedBody(bid).parentId();
        
        const BTransform &X_parent = m.fixedBody(bid).parentTrans();
        const BMatrix3 &fixed_rot  = arb::transpose(X_parent.E());
        const BVector3 &fixed_pos  = X_parent.r();
        
        const BTransform &X_base   = m.body(parent_id).X_base();
        const BMatrix3 &parent_rot = arb::transpose(X_base.E());
        const BVector3 &parent_pos = X_base.r();
             
        // pos = (X_base * X_parent).applyTranspose(body_pos);
        pos = parent_pos + (parent_rot * (fixed_pos + (fixed_rot * body_pos)));
    }
    else
    {
        // pos = m_body[bid].X_base().applyTranspose(body_pos);
        const BTransform &X_base = m.body(bid).X_base();
        pos = X_base.r() + X_base.E() * body_pos;
    }
 
    return pos;
}

BVector3
BKinematics::toBodyPos( BModel &m, BBodyId bid, const BVector3 &base_pos ) 
{
    BVector3 pos;
    
    if (m.isFixedBodyId(bid)) 
    {
        BBodyId parent_id = m.fixedBody(bid).parentId();
        
        const BTransform &X_parent = m.fixedBody(bid).parentTrans();
        const BMatrix3 &fixed_rot = X_parent.E();
        const BVector3 &fixed_pos = X_parent.r();
        
        const BTransform &X_base = m.body(parent_id).X_base();
        const BMatrix3 &parent_rot = X_base.E();
        const BVector3 &parent_pos = X_base.r();
        
        // pos = (X_base * X_parent).apply(base_pos);
        pos = (fixed_rot  * (-fixed_pos - parent_rot * (parent_pos - base_pos)));
    }
    else
    {
        //pos = m_body[bid].X_base().apply(base_pos);
        const BTransform &X_base = m.body(bid).X_base();
        pos = arb::transpose(X_base.E()) * (base_pos - X_base.r());
    }
    
    return pos;
}


BMatrix3
BKinematics::orient(BModel &m, const BBodyId bid)   
//  an orthonormal 3x3 matrix that rotates vectors from base to body coordinates.
{
    if (m.isFixedBodyId(bid)) 
    {
        BBodyId parent_id = m.fixedBody(bid).parentId();
        BTransform baseTrans =  m.fixedBody(bid).parentTrans() * m.body(parent_id).X_base();
        //  m.fixedBody(bid).baseTrans( m.fixedBody(bid).parentTrans() * m.body(parent_id).X_base() );
        return baseTrans.E();
    }
 
    return m.body(bid).X_base().E();
}



void 
BKinematics::calcPointJacobian(  BModel &m,
                                 const BModelState &qstate,
                                 BBodyId bid,
                                 const BVector3 &point_pos,
                                 BMatrix &G,
                                 bool update_kinematics ) 
{
    assert(G.rows() == 6 && G.cols() == m.qdotsize());
    
    if (update_kinematics) // update the kinematics if necessary
    {
        update_X_base(m, qstate);
    }
    
    // deal with Fixed1 joints
    BBodyId ref_id = bid;
    BVector3 ref_point = point_pos;
    if (m.isFixedBodyId(bid)) 
    {
        ref_id = m.fixedBody(bid).parentId();
        const BTransform &X = m.fixedBody(bid).parentTrans();
        ref_point = X.r() + (X.E() * point_pos);
    }
    BTransform point_trans(toBasePos(m, ref_id, ref_point)); 
    BBodyId j = ref_id;
    
    while (j != 0)
    {
        const BJoint &joint = m.joint(j);
        
        int qidx     = joint.qindex();    
        int dofCount = joint.DoFCount();  
        
        if (dofCount == 1) 
        {
            //G.block(0,q_index, 6, 1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block<6, 1>(0,0);
            const BVector6 S(joint.S());
            
            BVector6 val = point_trans * (arb::inverse(m.body(j).X_base()) * S);
            block(G, 0, qidx, val); 
        } 
        else if (dofCount == 3) 
        {
            //G.block(0, q_index, 6, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block<6, 3>(0,0);
            const BMatrix63 S(joint.S());
            
            BMatrix63 val = (point_trans * arb::inverse(m.body(j).X_base()) * S);
            block(G, 0, qidx, val);
        }
        
        j = m.parentId(j); 
    }
}

