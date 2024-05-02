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


BModel::BModel( int expected_dof )
{
    m_dof_count = 0;
    q_size      = 0;
    qdot_size   = 0;
    m_gravity   = BZERO_3;

    m_IA.reserve(expected_dof);
    m_pA.reserve(expected_dof);
    m_joints.reserve(expected_dof);
    m_bodies.reserve(expected_dof);
    m_lambda.reserve(expected_dof);
    m_fixedBodies.reserve(expected_dof);
    
    //
    // structural information
    //
    
    // root joint
    m_joints.push_back(BJoint());
    m_joints.back().init();
    
    // root body
    m_bodies.push_back(BBody());
    m_bodies.back().init();
       
    // parent
    m_lambda.push_back(0);
    
    m_bodyNameMap["ROOT"] = 0;
    
    // dynamic variables
    m_IA.push_back(BIDENTITY_6x6);
    m_pA.push_back(BZERO_6);
    
    m_fbd = std::numeric_limits<BBodyID>::max() / 2;
}


bool 
BModel::isFixedBodyId( BBodyID bid ) const
{
    if (bid >= m_fbd && bid < std::numeric_limits<BBodyID>::max()
        && bid - m_fbd < m_fixedBodies.size()) 
    {
        return true;
    }
    return false;
}


BBodyID 
BModel::getBodyId( const std::string &body_name ) const
{
    auto iter = m_bodyNameMap.find(body_name);
    if (iter == m_bodyNameMap.end()) 
    {
        return std::numeric_limits<BBodyID>::max();
    }
    
    return iter->second;
}


std::string 
BModel::getBodyName( BBodyID bid ) const
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


bool 
BModel::isBodyId( BBodyID bid ) const
{
    if (bid > 0 && bid < m_bodies.size()) 
    {
        return true;
    }
    if (bid >= m_fbd && bid < std::numeric_limits<BBodyID>::max()) 
    {
        if (bid - m_fbd < m_fixedBodies.size()) 
        {
            return true;
        }
    }
    return false;
}

/** Determines id the actual parent body.
 *
 * When adding bodies using joints with multiple degrees of
 * freedom, additional virtual bodies are added for each degree of
 * freedom. This function returns the id of the actual
 * non-virtual parent body.
 */
BBodyID 
BModel::getParentBodyId( BBodyID bid ) const
{
    if (bid >= m_fbd) 
    {
        return m_fixedBodies[bid - m_fbd].movableParent();
    }
    
    BBodyID parent_id = m_lambda[bid];
    
    while (m_bodies[parent_id].isVirtual()) 
    {
        parent_id = m_lambda[parent_id];
    }
    
    return parent_id;
}


BSpatialTransform 
BModel::getJointFrame( BBodyID bid ) const
// returns the joint frame transformtion, i.e. the second argument to BModel::addBody()
{
    if (bid >= m_fbd) 
    {
        return m_fixedBodies[bid - m_fbd].parentTrans();
    }
    
    BBodyID child_id = bid;
    BBodyID parent_id = m_lambda[bid];
    
    if (m_bodies[parent_id].isVirtual()) 
    {
        while (m_bodies[parent_id].isVirtual()) 
        {
            child_id = parent_id;
            parent_id = m_lambda[child_id];
        }
        return m_joints[child_id].X_T();
    } 
    else 
    {
        return m_joints[bid].X_T();
    }
}


void 
BModel::setJointFrame( BBodyID bid, const BSpatialTransform &transform )
// sets the joint frame transformtion, i.e. the second argument to BModel::addBody()
{
    if (bid >= m_fbd) 
    {
        std::cout << "Error: setting of parent transform not supported for fixed bodies!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    BBodyID child_id = bid;
    BBodyID parent_id = m_lambda[bid];
    
    if (m_bodies[parent_id].isVirtual()) 
    {
        while (m_bodies[parent_id].isVirtual()) 
        {
            child_id = parent_id;
            parent_id = m_lambda[child_id];
        }
        m_joints[child_id].X_T( transform );
    } 
    else if (bid > 0) 
    {
        m_joints[bid].X_T( transform );
    }
}


BBodyID 
BModel::addBodyFixedJoint( const BBodyID parent_id,
                           const BSpatialTransform &joint_frame,
                           const BJoint &joint,
                           const BBody &body,
                           const std::string &body_name )
{
    BFixedBody fbody(body);
    
    fbody.movableParent(parent_id);
    fbody.parentTrans(joint_frame);
    
    if (isFixedBodyId(parent_id)) 
    {
        BFixedBody fixed_parent = m_fixedBodies[parent_id - m_fbd];

        fbody.movableParent( fixed_parent.movableParent() );
        fbody.parentTrans( joint_frame * fixed_parent.parentTrans() );
    }
    
    // merge the two bodies
    BBody parent_body = m_bodies[fbody.movableParent()];
    parent_body.join( fbody.parentTrans(), body );
    m_bodies[fbody.movableParent()] = parent_body;
    m_bodies[fbody.movableParent()].I(BSpatialInertia( parent_body ));
    m_fixedBodies.push_back(fbody);
    
    if (m_fixedBodies.size() > std::numeric_limits<BBodyID>::max() - m_fbd) 
    {
        std::cout << "Error: cannot add more than "
                    << std::numeric_limits<BBodyID>::max() - m_fixedBodies.size()
                    << " fixed bodies. You need to modify Model::m_fbd for this.\n" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    if (body_name.size() != 0) 
    {
        if (m_bodyNameMap.find(body_name) != m_bodyNameMap.end()) 
        {
            std::cout << "Error: Body with name '" << body_name << "' already exists!\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_bodyNameMap[body_name] = (BBodyID) m_fixedBodies.size() + m_fbd - 1;
    }
    
    return (BBodyID) m_fixedBodies.size() + m_fbd - 1;
}

BBodyID 
BModel::addBodyMultiDofJoint( BBodyID parent_id,
                              const BSpatialTransform &joint_frame,
                              const BJoint &joint,
                              const BBody &body,
                              const std::string &body_name ) 
{
    // Here we emulate multi DoF joints by simply adding nullbodies. This
    // allows us to use fixed size elements for S,v,a, etc. which is very
    // fast in Eigen.
    int joint_count = 0;
    switch (joint.jtype()) 
    {
        case B1DoF:
            joint_count = 1;
            break;
        case B2DoF:
            joint_count = 2;
            break;
        case B3DoF:
            joint_count = 3;
            break;
        case B4DoF:
            joint_count = 4;
            break;
        case B5DoF:
            joint_count = 5;
            break;
        case B6DoF:
            joint_count = 6;
            break;
        case BFloatingBase:
            // no action required
            break;
        default:
            std::cout << "Error: Invalid joint type: " << joint.jtype() << "\n" << std::endl;
            exit(EXIT_FAILURE);
            break;
    }
    
    BBody null_body(0.0, BZERO_3, BZERO_3, true);
    
    BBodyID null_parent = parent_id;

    if (joint.jtype() == BFloatingBase) 
    {
        null_parent = addBody(parent_id,
                              joint_frame,
                              BTranslationXYZ,
                              null_body);
        
        return addBody(null_parent,
                       BSpatialTransform(BIDENTITY_3x3, BZERO_3),
                       BSpherical,
                       body,
                       body_name);
    }
    
    BJoint single_dof_joint;
    BSpatialTransform joint_frame_transform(BIDENTITY_3x3, BZERO_3);
    
    // Here we add multiple virtual bodies that have no mass or inertia for
    // which each is attached to the model with a single degree of freedom joint.
    for (int j = 0; j < joint_count; ++j) 
    {
        single_dof_joint = BJoint(joint.axis(j));
        
        if (single_dof_joint.jtype() == B1DoF) 
        {
            BVector3 rot(joint.axis(j).head());
            BVector3 trans(joint.axis(j).tail());

            if (rot == BZERO_3) 
            {
                single_dof_joint = BJoint(BPrismatic, trans);
            }
            else if (trans == BZERO_3) 
            {
                single_dof_joint = BJoint(BRevolute, rot);
            }
        }
        
        // the first joint has to be transformed by joint_frame, all the
        // others must have a null transformation
        if (j == 0) 
            joint_frame_transform = joint_frame;
        else joint_frame_transform = BSpatialTransform(BIDENTITY_3x3, BZERO_3);

        if (j == joint_count - 1)
        // if we are at the last we must add the real body
        {
            break;
        } 
        else 
        {
            // otherwise we just add an intermediate body
            null_parent = addBody( null_parent,
                                   joint_frame_transform,
                                   single_dof_joint,
                                   null_body );
        }
    }
    
    return addBody( null_parent,
                    joint_frame_transform,
                    single_dof_joint,
                    body,
                    body_name );
}

BBodyID 
BModel::addBody( BBodyID parent_id,
                 const BSpatialTransform &joint_frame,
                 const BJoint &joint,
                 const BBody  &body,
                 const std::string &body_name )
{
    assert(m_lambda.size() > 0);
    assert(joint.jtype() != BUndefined);
    
    if (joint.jtype() == BFixed) 
    {
        return addBodyFixedJoint( parent_id,
                                  joint_frame,
                                  joint,
                                  body,
                                  body_name );
        
    } 
    
    if ( (joint.jtype() == BSpherical)
             || (joint.jtype() == BEulerZYX) 
             || (joint.jtype() == BEulerXYZ) 
             || (joint.jtype() == BEulerYXZ) 
             || (joint.jtype() == BEulerZXY) 
             || (joint.jtype() == BTranslationXYZ) ) 
    {
        // no action required
    } 
    else if ( joint.jtype() != BPrismatic
             && joint.jtype() != BRevolute
             && joint.jtype() != BRevoluteX
             && joint.jtype() != BRevoluteY
             && joint.jtype() != BRevoluteZ
             && joint.jtype() != BHelical ) 
    {
        return addBodyMultiDofJoint( parent_id,
                                     joint_frame,
                                     joint,
                                     body,
                                     body_name );
    }
    
    // If we add the body to a fixed body we have to make sure that we
    // actually add it to its movable parent.
    BBodyID movable_parent_id = parent_id;
    BSpatialTransform movable_parent_transform(BIDENTITY_3x3, BZERO_3);
    
    if (isFixedBodyId(parent_id)) 
    {
        BBodyID fbody_id = parent_id - m_fbd;
        movable_parent_id = m_fixedBodies[fbody_id].movableParent();
        movable_parent_transform = m_fixedBodies[fbody_id].parentTrans();
    }
    
    // structural information
    m_lambda.push_back(movable_parent_id);

    // BBodies
    m_bodies.push_back(body);
    m_bodies.back().I( BSpatialInertia(body) );

    if (!body_name.empty()) 
    {
        if (m_bodyNameMap.find(body_name) != m_bodyNameMap.end()) 
        {
            std::cout << "Error: Body with name '" << body_name << "' already exists!\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_bodyNameMap[body_name] = (int) m_bodies.size() - 1;
    }
  
    if (m_bodies.size() == m_fbd) 
    {
        std::cout << "Error: cannot add more than " << m_fbd << " movable bodies. You need to modify Model::m_fbd for this.\n" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // BJoints
    BJointID prev_joint = (int) m_joints.size() - 1;
    m_joints.push_back(joint);
    
    m_joints.back().qindex( m_joints[prev_joint].qindex() + m_joints[prev_joint].DoFCount() );
    // we have to invert the transformation as it is later always used from the child bodies perspective.
    m_joints.back().X_T( joint_frame * movable_parent_transform );
    
    // counts and sizes
    m_dof_count += joint.DoFCount();
    
    // update the w components of the quaternions. They are stored at the end of the q vector
    int multdof3_joint_counter = 0;
    for (int i = 1; i < m_joints.size(); ++i) 
    {
        if (m_joints[i].jtype() == BSpherical) 
        {
            m_joints[i].dof3_windex( m_dof_count + multdof3_joint_counter );
            multdof3_joint_counter++;
        }
    }
    
    q_size = m_dof_count + multdof3_joint_counter;
    qdot_size += joint.DoFCount();
    
    // dynamic variables
    m_IA.push_back(BZERO_6x6);
    m_pA.push_back(BZERO_6);

    return (int) m_bodies.size() - 1;
}

void 
BModel::updateInertia(BBodyID bid)
{
    if (isFixedBodyId(bid))
    {
        bid = m_fixedBodies[bid - m_fbd].movableParent();
    }
    
    BSpatialInertia rbi(m_bodies[bid]);
    //m_bodies[body_id].Ic(rbi);
    m_bodies[bid].I(rbi);
}


void 
BModel::setMass(BBodyID bid, BScalar mass)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody(m_fixedBodies[bid - m_fbd]);
        m_bodies[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.mass( mass );
        m_bodies[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_bodies[bid].mass( mass );
    }
    updateInertia(bid);
}


void 
BModel::setInertia(BBodyID bid, const BMatrix3 &inertia)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody(m_fixedBodies[bid - m_fbd]);
        m_bodies[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.inertia( inertia );
        m_bodies[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_bodies[bid].inertia( inertia );
    }
    updateInertia(bid);
}


void 
BModel::setCOM(BBodyID bid, const BVector3 &com)
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody(m_fixedBodies[bid - m_fbd]);
        m_bodies[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.com( com );
        m_bodies[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_bodies[bid].com( com );
    }
    updateInertia(bid);
}


void 
BModel::setParameters( BBodyID bid, BScalar mass, const BMatrix3 &inertia, const BVector3 &com )
{
    if (isFixedBodyId(bid))
    {
        // The inertial parameters of the fixed body have already been merged in the parent body,
        // in order to update correctly the parent body we need to first
        // remove the inertial effects of the fixed body from his parent body
        // Then update the fixed body inertia
        // Finally, merge it again in the parent body
        BFixedBody& fbody(m_fixedBodies[bid - m_fbd]);
        m_bodies[fbody.movableParent()].separate(fbody.parentTrans(), fbody.toBody());
        fbody.mass( mass );
        fbody.inertia( inertia );
        fbody.com( com );
        m_bodies[fbody.movableParent()].join(fbody.parentTrans(), fbody.toBody());
    }
    else
    {
        m_bodies[bid].mass( mass );
        m_bodies[bid].inertia( inertia );
        m_bodies[bid].com( com );
    }
    updateInertia(bid);
}


