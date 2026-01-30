/* BModel 31/01/2024

 $$$$$$$$$$$$$$$$$$
 $   BModel.cpp   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/


#ifndef __BMODEL_H__
#include "BModel.h"
#endif


#include <set>


BModel::BModel( int expected_dof ) :  m_dof_count(0), m_q_size(0), m_qdot_size(0), m_gravity(B_ZERO_3)
{

    m_joint.reserve(expected_dof);
    m_body.reserve(expected_dof);
    m_lambda.reserve(expected_dof);
    m_fixed.reserve(expected_dof);
    
    m_fbd = std::numeric_limits<BBodyId>::max() / 2;
    
    init();
}

void 
BModel::clear( void )
{
    m_dof_count = m_q_size = m_qdot_size = 0; 
    m_gravity = B_ZERO_3;
    
    m_lambda.clear();
    m_body.clear();
    m_joint.clear();
    m_fixed.clear(); 

    m_bodyNameMap.clear();
    
    init();
}

void 
BModel::init( void )
// setup and initialise
{
    m_lambda.push_back(0);
    
    m_body.resize(1);
    m_body[0].clear();
       
    m_joint.resize(1);
    m_joint[0].clear();
    
    m_bodyNameMap["ROOT"] = 0;
}


bool 
BModel::isBodyId( BBodyId bid ) const
{
    if (bid > 0 && bid < m_body.size()) 
        return true;
    
    if (bid >= m_fbd && bid - m_fbd < m_fixed.size()) 
        return true;
    
    return false;
}

bool 
BModel::isFixedBodyId( BBodyId bid ) const
{
    return (bid >= m_fbd && bid - m_fbd < m_fixed.size()) ? true : false;
}

BBodyId 
BModel::getBodyId( const std::string &body_name ) const
{
    auto iter = m_bodyNameMap.find(body_name);
    if (iter == m_bodyNameMap.end()) 
        return std::numeric_limits<BBodyId>::max();
 
    return iter->second;
}


std::string 
BModel::getBodyName( BBodyId bid ) const
// the name of a body for a given body id 
{
    auto iter = m_bodyNameMap.begin();
    
    while (iter != m_bodyNameMap.end()) 
    {
        if (iter->second == bid) 
            return iter->first;

        ++iter;
    }
    
    return std::string();
}

/** Determines id the actual parent body.
 *
 * When adding bodies using joints with multiple degrees of
 * freedom, additional virtual bodies are added for each degree of
 * freedom. This function returns the id of the actual
 * non-virtual parent body.
 */
BBodyId 
BModel::getParentBodyId( BBodyId bid ) const
{
    if (isFixedBodyId(bid)) 
        return fixedBody(bid).movableParent();
    
    BBodyId parent_id = m_lambda[bid];
    
    while (m_body[parent_id].isVirtual()) 
        parent_id = m_lambda[parent_id];
    
    return parent_id;
}

BTransform 
BModel::getJointFrame( BBodyId bid ) const
// returns the joint frame transformtion, i.e. the second argument to BModel::addBody()
{
    if (isFixedBodyId(bid)) 
        return fixedBody(bid).parentTrans();
 
    BBodyId parent_id = m_lambda[bid];
    BBodyId child_id = bid;
    
    if (m_body[parent_id].isVirtual()) 
    {
        while (m_body[parent_id].isVirtual()) 
        {
            child_id = parent_id;
            parent_id = m_lambda[child_id];
        }
    } 

    return m_joint[child_id].X_T();
}

void 
BModel::setJointFrame( BBodyId bid, const BTransform &transform )
// sets the joint frame transformtion, i.e. the second argument to BModel::addBody()
{
    if (isFixedBodyId(bid)) 
    {
        std::cout << "Error: setting of parent transform not supported for fixed bodies!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    BBodyId child_id = bid;
    BBodyId parent_id = m_lambda[bid];
    
    if (m_body[parent_id].isVirtual()) 
    {
        while (m_body[parent_id].isVirtual()) 
        {
            child_id = parent_id;
            parent_id = m_lambda[child_id];
        }
    } 

    m_joint[child_id].X_T( transform );
}

BVector3
BModel::toBasePos( BBodyId bid,  const BVector3 &body_pos ) const
// returns the base coordinates of a position given in body coordinates.
// see kinematics.cc - CalcBodyToBaseCoordinates
{
    BVector3 pos;
    
    if (isFixedBodyId(bid))
    {
        BBodyId fbody_id = bid - m_fbd;
        BBodyId parent_id = m_fixed[fbody_id].movableParent();
        
        const BTransform &X_parent = m_fixed[fbody_id].parentTrans();
        const BMatrix3 &fixed_rot  = arb::transpose(X_parent.E());
        const BVector3 &fixed_pos  = X_parent.r();
        
        const BTransform &X_base = m_body[parent_id].X_base();
        const BMatrix3 &parent_rot = arb::transpose(X_base.E());
        const BVector3 &parent_pos = X_base.r();
        
        pos = parent_pos + (parent_rot * (fixed_pos + (fixed_rot * body_pos)));
        // pos = (X_base * X_parent).applyTranspose(body_pos);
    }
    else
    {
        //pos = m_body[bid].X_base().applyTranspose(body_pos);
        const BTransform &X_base = m_body[bid].X_base();
        pos = X_base.r() + arb::transpose(X_base.E()) * body_pos;
    }
 
    return pos;
}


BVector3
BModel::toBodyPos( BBodyId bid, const BVector3 &base_pos ) const
{
    BVector3 pos;
    
    if (isFixedBodyId(bid)) 
    {
        BBodyId fbody_id = bid - m_fbd;
        BBodyId parent_id = m_fixed[fbody_id].movableParent();
        
        const BTransform &X_parent = m_fixed[fbody_id].parentTrans();
        const BMatrix3 &fixed_rot = X_parent.E();
        const BVector3 &fixed_pos = X_parent.r();
        
        const BTransform &X_base = m_body[parent_id].X_base();
        const BMatrix3 &parent_rot = X_base.E();
        const BVector3 &parent_pos = X_base.r();
        
        pos = (fixed_rot  * (-fixed_pos - parent_rot * (parent_pos - base_pos)));
        // pos = (X_base * X_parent).apply(base_pos);
    }
    else
    {
        //pos = m_body[bid].X_base().apply(base_pos);
        const BTransform &X_base = m_body[bid].X_base();
        pos = X_base.E() * (base_pos - X_base.r());
    }
    
    return pos;
}

BMatrix3
BModel::orient(const BBodyId bid)  const 
//  an orthonormal 3x3 matrix that rotates vectors from base to body coordinates.
{
    if (isFixedBodyId(bid)) 
    {
        BBodyId fbody_id = bid - m_fbd;
        BBodyId parent_id    = m_fixed[fbody_id].movableParent();
        BTransform baseTrans = m_fixed[fbody_id].parentTrans() * m_body[parent_id].X_base();
        
        //  m_fixed[fbody_id].baseTrans( m_fixed[fbody_id].parentTrans() * m_body[parent_id].X_base() );
        return baseTrans.E();
    }
 
    return m_body[bid].X_base().E();
}

BVector6 
BModel::pointVel( BBodyId bid, const BVector3 &body_pos ) 
// RBDL Kinematics::CalcPointVelocity6D
{
    BBodyId  ref_bid = bid;
    BVector3 ref_pos = body_pos;
    
    if (isFixedBodyId(bid)) 
    {
        BVector3 base_pos = toBasePos(bid, body_pos);
        
        ref_bid = m_fixed[bid - m_fbd].movableParent();
        ref_pos = toBodyPos(ref_bid, base_pos);
    }
    
    BTransform trans( arb::transpose(orient(ref_bid)), ref_pos );
    
    return trans.apply(m_body[ref_bid].v());
}

BVector6 
BModel::pointAcc( BBodyId bid,  const BVector3 &body_pos ) 
// RBDL Kinematics::CalcPointAcceleration6D
{
    BBodyId  ref_bid = bid;
    BVector3 ref_pos = body_pos;
    
    if (isFixedBodyId(bid)) 
    {
        BVector3 base_pos = toBasePos(bid, body_pos);
        
        ref_bid = m_fixed[bid - m_fbd].movableParent();
        ref_pos = toBodyPos(ref_bid, base_pos); 
    }
    
    BTransform p_X(arb::transpose(orient(ref_bid)), ref_pos);
    
    BVector6 p_v = p_X.apply(m_body[ref_bid].v());
    BVector3 a_dash = arb::cross(p_v.ang(), p_v.lin());
    
    return p_X.apply(m_body[ref_bid].a()) + BVector6(B_ZERO_3, a_dash);
}


BScalar
BModel::mass( BBodyId bid ) const 
{
    if (isFixedBodyId(bid)) 
        bid = m_fixed[bid - m_fbd].movableParent();
    
    BScalar mass = 0.0;
    
    std::set<BBodyId> children = {bid};
    for (int i = 1; i < m_body.size(); ++i)
    {
        if (children.contains(m_lambda[i]))
        {
            mass += m_body[i].I().mass();
            children.insert(i);
        }
    }

    return m_body[bid].I().mass() + mass;
}

BVector3 
BModel::com( BBodyId bid ) const 
{
    if (isFixedBodyId(bid)) 
        bid = m_fixed[bid - m_fbd].movableParent();

    assert(m_body[bid].I().mass() > 0);
    
    BScalar m = m_body[bid].I().mass();
    BVector3 sum = m_body[bid].I().mass() * m_body[bid].X_base().applyTranspose(m_body[bid].I().com());

    std::set<BBodyId> children = {bid};
    for (int i = 1; i < m_body.size(); ++i)
    {
        if (children.contains(m_lambda[i]) && m_body[i].I().mass() > 0.0)
        {
            m += m_body[i].I().mass();
            sum  += m_body[i].I().mass() * m_body[i].X_base().applyTranspose(m_body[i].I().com());
            children.insert(i);
        }
    }

    return m_body[bid].X_base().apply(sum / m) ;
}

BRBInertia  
BModel::inertia( BBodyId bid ) const
{
    if (isFixedBodyId(bid)) 
        bid = m_fixed[bid - m_fbd].movableParent();
    
    BRBInertia I(B_ZERO_RBI);
 
    std::set<BBodyId> children = {bid};
    for (int i = 1; i < m_lambda.size(); ++i)
    {
        if (children.contains(m_lambda[i]))
        {       
            I += m_body[i].X_base().applyTranspose(m_body[i].I());
            children.insert(i);
        }
    }
    
    I = m_body[bid].I() + m_body[bid].X_base().apply(I);
    
    return I; 
}

void
BModel::addName(  BBodyId bid, const std::string &body_name )
{
    // we could use body_name = std::to_string(bid)
    if (!body_name.empty()) 
    {
        if (m_bodyNameMap.find(body_name) != m_bodyNameMap.end()) 
        {
            std::cout << "Error: Body with name '" << body_name << "' already exists!\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_bodyNameMap[body_name] = bid;
    }
}

BBodyId 
BModel::addFixedJoint( const BBodyId parent_id,
                       const BTransform &joint_frame,
                       const BJoint &joint,
                       const BBody &body,
                       const std::string &body_name )
{
    BFixedBody fbody(body);
    
    fbody.movableParent(parent_id);
    fbody.parentTrans(joint_frame); // like X_lambda = X_T * X_J -- only X_J = 1 fixed
    
    if (isFixedBodyId(parent_id)) 
    {
        const BFixedBody &fparent = m_fixed[parent_id - m_fbd];
        fbody.movableParent( fparent.movableParent() );
        fbody.parentTrans( joint_frame * fparent.parentTrans() );
    }
    
    // add the fixed body mass tp moveable parent  body
    BBody parent_body = m_body[fbody.movableParent()];
    parent_body.join( fbody.parentTrans(), body );
    m_body[fbody.movableParent()] = parent_body;

    BBodyId bid = (BBodyId) m_fixed.size() + m_fbd;
    fbody.setId(bid);
    
    m_fixed.push_back(fbody);

    addName(bid, body_name);
    
    return bid;
}

BBodyId
BModel::addFloatingBaseJoint( BBodyId parent_id,
                              const BTransform &joint_frame,
                              const BJoint &joint,
                              const BBody &body,
                              const std::string &body_name )
{
    
    BBody null_body(BInertia(0.0, B_ZERO_3, B_ZERO_3x3), true);
    
    BBodyId null_parent = addBody(parent_id,
                          joint_frame,
                          BJoint::TranslationXYZ,
                          null_body);
    
    return addBody(null_parent,
                   BTransform(B_IDENTITY_TRANS),
                   BJoint::Spherical,
                   body,
                   body_name);
}

BBodyId 
BModel::addMultiDofJoint( BBodyId parent_id,
                          const BTransform &joint_frame,
                          const BJoint &joint,
                          const BBody &body,
                          const std::string &body_name ) 
{
    // Here we emulate multi DoF joints by simply adding nullbodies. This
    // allows us to use fixed size elements for S,v,a, etc. which is very
    // fast in Eigen.
    assert (joint.jtype() >= BJoint::J1DoF && joint.jtype() <= BJoint::J6DoF);
    
    int joint_count =  joint.jtype(); // 1-6
    
    BBodyId null_parent_id = parent_id;

    BJoint single_dof_joint;
    BTransform joint_frame_transf;
    
    // Here we add multiple virtual bodies that have no mass or inertia for
    // which each is attached to the model with a single degree of freedom joint.
    for (int j = 0; j < joint_count; ++j) 
    {
        single_dof_joint = BJoint(joint.axis(j));
        
        if (single_dof_joint.jtype() == BJoint::J1DoF) 
        {
            const BVector3 &rot = joint.axis(j).ang();
            const BVector3 &trans = joint.axis(j).lin();

            if (rot == B_ZERO_3) 
                single_dof_joint = BJoint(BJoint::Prismatic, trans);
            else if (trans == B_ZERO_3) 
                single_dof_joint = BJoint(BJoint::Revolute, rot);
        }
        
        // the first joint has to be transformed by joint_frame, all the
        // others must have a null transformation
        if (j == 0) 
            joint_frame_transf = joint_frame;
        else joint_frame_transf = BTransform(B_IDENTITY_TRANS);

        if (j < joint_count - 1)
        {
            // add an intermediate body
            BBody null_body(BInertia(0.0, B_ZERO_3, B_ZERO_3x3), true);
            null_parent_id = addBody( null_parent_id, joint_frame_transf, single_dof_joint, null_body );
        } 
        else 
        {
            break; // if we are at the last...
        }
    }
    
    //  ...we must add the real body
    return addBody( null_parent_id,
                    joint_frame_transf,
                    single_dof_joint,
                    body,
                    body_name );
}

BBodyId 
BModel::addBody( BBodyId parent_id,
                 const BTransform &joint_frame,
                 const BJoint &joint,
                 const BBody  &body,
                 const std::string &body_name )
{
    assert(m_body.size() < m_fbd);
    assert(m_lambda.size() > 0);
    assert(joint.jtype() != BJoint::UNDEFINED);
    
    if (joint.jtype() == BJoint::Fixed1) 
        return addFixedJoint( parent_id, joint_frame, joint, body, body_name );
    
    if (joint.jtype() == BJoint::FloatingBase) 
        return addFloatingBaseJoint(parent_id, joint_frame, joint, body, body_name);
    
    if ((joint.jtype() >= BJoint::J1DoF) && (joint.jtype() <= BJoint::J6DoF)) 
        return addMultiDofJoint( parent_id, joint_frame, joint, body, body_name );
    
    
    // If we add the body to a fixed body we have to make sure that we
    // actually add it to its movable parent.
    BBodyId movable_parent_id = parent_id;
    BTransform movable_parent_transform(B_IDENTITY_TRANS);
    
    if (isFixedBodyId(parent_id)) 
    {
        const BFixedBody &fbody = m_fixed[parent_id - m_fbd];
        movable_parent_id = fbody.movableParent();
        movable_parent_transform = fbody.parentTrans();
    }
    
    // structural information
    m_lambda.push_back(movable_parent_id);

    //
    // BBody 
    //
    BBodyId bid = (int) m_body.size();
    m_body.push_back(body);
    m_body.back().setId( bid );
    addName(bid, body_name);
    
    //
    // BJoint
    //
    BJointId prev_joint_id = (int) m_joint.size() - 1;
    m_joint.push_back(joint);
    m_joint.back().setId( bid ); // joint has the same Id as body
    m_joint.back().qindex( m_joint[prev_joint_id].qindex() + m_joint[prev_joint_id].DoFCount() );
    // we have to invert the transformation as it is later always used from the child bodies perspective.
    m_joint.back().X_T( joint_frame * movable_parent_transform );
    
    // set body transform X_base - base to body frame origin - assume initial joint position is 'closed'
    m_joint[bid].jcalc();  
    const BTransform &X_lambda = m_joint[bid].X_lambda(); 
    if (m_lambda[bid] != 0)
        m_body[bid].X_base( X_lambda * m_body[m_lambda[bid]].X_base() );
    else m_body[bid].X_base( X_lambda );
    
    // calculate DoF-count, w-indexes, and q-sizes
    calcDoF();


    return bid; 
}

void
BModel::calcDoF( void )
// joint parameter space - calculate Degrees-of-Freedom count, w-indexes, and q-sizes
{
    m_dof_count = 0;
    for (int i = 1; i < m_joint.size(); ++i) 
        m_dof_count += m_joint[i].DoFCount();
        
    int multdof3 = 0;
    for (int i = 1; i < m_joint.size(); ++i) 
    {
        if (m_joint[i].jtype() == BJoint::Spherical) 
        {
            // update the w-components of quaternion - they are stored at end of q-vector
            m_joint[i].windex( m_dof_count + multdof3 );
            multdof3++;
        }
    }
    m_q_size    = m_dof_count + multdof3;
    m_qdot_size = m_dof_count;
}


void 
BModel::setBody(BBodyId bid, const BInertia &inertia)
{
    if (isFixedBodyId(bid))
    {
        BFixedBody& fbody = m_fixed[bid - m_fbd];
        m_body[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.set( inertia );
        m_body[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_body[bid].I().set( inertia );
    }
}




