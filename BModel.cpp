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


BModel::BModel( int expected_dof ) :  m_dof_count(0), q_size(0), qdot_size(0), m_gravity(B_ZERO_3)
{

    m_IA.reserve(expected_dof);
    m_pA.reserve(expected_dof);
    m_joint.reserve(expected_dof);
    m_body.reserve(expected_dof);
    m_lambda.reserve(expected_dof);
    m_fixed.reserve(expected_dof);
    
    //
    // structural information
    //
    
    // root joint
    m_joint.push_back(BJoint());
    m_joint.back().clear();
    
    // root body
    m_body.push_back(BBody());
    m_body.back().clear();
       
    // parent
    m_lambda.push_back(0);
    
    m_bodyNameMap["ROOT"] = 0;
    
    // dynamic variables
    m_IA.push_back(B_IDENTITY_6x6);
    m_pA.push_back(B_ZERO_6);
    
    m_fbd = std::numeric_limits<BBodyId>::max() / 2;
}

void 
BModel::clear( void )
{
    m_dof_count = q_size = qdot_size = 0; 
    m_gravity = B_ZERO_3;
    m_lambda.clear();
    m_joint.clear();
    m_body.clear();
    m_fixed.clear();  
    m_IA.clear(); 
    m_pA.clear(); 
    m_bodyNameMap.clear();
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
    {
        return std::numeric_limits<BBodyId>::max();
    }
    
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
        {
            return iter->first;
        }
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
    {
        parent_id = m_lambda[parent_id];
    }
    
    return parent_id;
}


BSpatialTransform 
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
        return m_joint[child_id].X_T();
    } 
    else 
    {
        return m_joint[bid].X_T();
    }
}


void 
BModel::setJointFrame( BBodyId bid, const BSpatialTransform &transform )
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

    m_joint[bid].X_T( transform );
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
        
        const BSpatialTransform &X_parent = m_fixed[fbody_id].parentTrans();
        BMatrix3 fixed_rot         = glm::transpose(X_parent.E());
        const BVector3 &fixed_pos  = X_parent.r();
        
        const BSpatialTransform &X_base = m_body[parent_id].X_base();
        BMatrix3 parent_rot        = glm::transpose(X_base.E());
        const BVector3 &parent_pos = X_base.r();
        
        pos = parent_pos + (parent_rot * (fixed_pos + (fixed_rot * body_pos)));
    }
    else
    {
        const BSpatialTransform &X_base = m_body[bid].X_base();
        pos = X_base.r() + glm::transpose(X_base.E()) * body_pos;
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
        
        const BSpatialTransform &X_parent = m_fixed[fbody_id].parentTrans();
        BMatrix3 fixed_rotation = X_parent.E();
        const BVector3 &fixed_position = X_parent.r();
        
        const BSpatialTransform &X_base = m_body[parent_id].X_base();
        BMatrix3 parent_rotation = X_base.E();
        const BVector3 &parent_position = X_base.r();
        
        pos = (fixed_rotation  * ( -fixed_position - parent_rotation * (parent_position - base_pos)));
    }
    else
    {
        const BSpatialTransform &X_base = m_body[bid].X_base();
        pos = X_base.E() * (base_pos - X_base.r());
    }
    
    return pos;
}

BMatrix3
BModel::orient(const BBodyId bid)  const 
//  an orthonormal 3x3 matrix that rotates vectors from base to body coordinates.
{
    BMatrix3 rot; 
    
    if (isFixedBodyId(bid)) 
    {
        BBodyId fbody_id = bid - m_fbd;
        BBodyId parent_id    = m_fixed[fbody_id].movableParent();
        BSpatialTransform baseTrans = m_fixed[fbody_id].parentTrans() * m_body[parent_id].X_base();
        
        //  m_fixed[fbody_id].baseTrans( m_fixed[fbody_id].parentTrans() * m_body[parent_id].X_base() );
        rot = baseTrans.E();
    }
    else
    {
        rot = m_body[bid].X_base().E();
    }
    
    return rot;
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
                       const BSpatialTransform &joint_frame,
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
                              const BSpatialTransform &joint_frame,
                              const BJoint &joint,
                              const BBody &body,
                              const std::string &body_name )
{
    
    BBody null_body(0.0, B_ZERO_3, B_ZERO_3, true);
    
    BBodyId null_parent = parent_id;

    null_parent = addBody(parent_id,
                          joint_frame,
                          BJoint::BTranslationXYZ,
                          null_body);
    
    return addBody(null_parent,
                   BSpatialTransform(B_IDENTITY_3x3, B_ZERO_3),
                   BJoint::BSpherical,
                   body,
                   body_name);
}

BBodyId 
BModel::addMultiDofJoint( BBodyId parent_id,
                          const BSpatialTransform &joint_frame,
                          const BJoint &joint,
                          const BBody &body,
                          const std::string &body_name ) 
{
    // Here we emulate multi DoF joints by simply adding nullbodies. This
    // allows us to use fixed size elements for S,v,a, etc. which is very
    // fast in Eigen.
    assert (joint.jtype() >= BJoint::B1DoF && joint.jtype() <= BJoint::B6DoF);
    
    int joint_count =  joint.jtype(); // 1-6
    
    BBodyId null_parent_id = parent_id;

    BJoint single_dof_joint;
    BSpatialTransform joint_frame_transf;
    
    // Here we add multiple virtual bodies that have no mass or inertia for
    // which each is attached to the model with a single degree of freedom joint.
    for (int j = 0; j < joint_count; ++j) 
    {
        single_dof_joint = BJoint(joint.axis(j));
        
        if (single_dof_joint.jtype() == BJoint::B1DoF) 
        {
            const BVector3 &rot = joint.axis(j).ang();
            const BVector3 &trans = joint.axis(j).lin();

            if (rot == B_ZERO_3) 
                single_dof_joint = BJoint(BJoint::BPrismatic, trans);
            else if (trans == B_ZERO_3) 
                single_dof_joint = BJoint(BJoint::BRevolute, rot);
        }
        
        // the first joint has to be transformed by joint_frame, all the
        // others must have a null transformation
        if (j == 0) 
            joint_frame_transf = joint_frame;
        else joint_frame_transf = BSpatialTransform(B_IDENTITY_3x3, B_ZERO_3);

        if (j < joint_count - 1)
        {
            // add an intermediate body
            BBody null_body(0.0, B_ZERO_3, B_ZERO_3, true);
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
                 const BSpatialTransform &joint_frame,
                 const BJoint &joint,
                 const BBody  &body,
                 const std::string &body_name )
{
    assert(m_lambda.size() > 0);
    assert(joint.jtype() != BJoint::BUNDEFINED);
    
    if (joint.jtype() == BJoint::BFixed) 
    {
        return addFixedJoint( parent_id, joint_frame, joint, body, body_name );
    } 
    
    if (joint.jtype() == BJoint::BFloatingBase) 
    {
        return addFloatingBaseJoint(parent_id, joint_frame, joint, body, body_name);
    }
    
    if ((joint.jtype() >= BJoint::B1DoF) && (joint.jtype() <= BJoint::B6DoF)) 
    {
        return addMultiDofJoint( parent_id, joint_frame, joint, body, body_name );
    }
    
  
    // If we add the body to a fixed body we have to make sure that we
    // actually add it to its movable parent.
    BBodyId movable_parent_id = parent_id;
    BSpatialTransform movable_parent_transform(B_IDENTITY_3x3, B_ZERO_3);
    
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
    int bid = (int) m_body.size();
    assert(m_body.size() < m_fbd);
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
    
    // counts and sizes
    m_dof_count += joint.DoFCount();
    
    // update the w components of the quaternions. They are stored at the end of the q vector
    int multdof3 = 0;
    for (int i = 1; i < m_joint.size(); ++i) 
    {
        if (m_joint[i].jtype() == BJoint::BSpherical) 
        {
            m_joint[i].dof3_windex( m_dof_count + multdof3 );
            multdof3++;
        }
    }
    
    q_size = m_dof_count + multdof3;
    qdot_size += joint.DoFCount();
    
    // update X_base - assume initial joint position is 'closed'
    m_joint[bid].jcalc();  
    
    const BSpatialTransform &X_lambda = m_joint[bid].X_lambda(); 
    
    if (m_lambda[bid] != 0)
        m_body[bid].X_base( X_lambda * m_body[m_lambda[bid]].X_base() );
    else m_body[bid].X_base( X_lambda );
    
    // dynamic variables
    m_IA.push_back(B_ZERO_6x6);
    m_pA.push_back(B_ZERO_6);

    return bid; 
}



void 
BModel::setMass(BBodyId bid, BScalar mass)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody = m_fixed[bid - m_fbd];
        m_body[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.mass( mass );
        m_body[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_body[bid].mass( mass );
    }
}


void 
BModel::setInertia(BBodyId bid, const BMatrix3 &inertia)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody = m_fixed[bid - m_fbd];
        m_body[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.inertiaCom( inertia );
        m_body[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_body[bid].inertiaCom( inertia ); // calls my code now -- it does update
    }
}


void 
BModel::setCom(BBodyId bid, const BVector3 &com)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody = m_fixed[bid - m_fbd];
        m_body[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.com( com );
        m_body[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_body[bid].com( com );
    }
}


void 
BModel::setParameters( BBodyId bid, BScalar mass, const BMatrix3 &inertia, const BVector3 &com )
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody = m_fixed[bid - m_fbd];
        m_body[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.mass( mass );
        fbody.inertiaCom( inertia );
        fbody.com( com );
        
        //fbody.setBody( mass, com, inertia );
        
        m_body[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_body[bid].setBody( mass, com, inertia );
    }
}


