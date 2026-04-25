/* URDFManager 11/04/2026

 $$$$$$$$$$$$$$$$$$$$$$$
 $   URDFManager.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 URDF, Unified Robot Description Format is an XML format
 
 This class implements a subset of the URDF XML format - see  https://wiki.ros.org/urdf/XML
 Used for loading/saving articulated bodies (BModel).
 
 Depends on tinyxml2 https://github.com/leethomason/tinyxml2
 
 I have included the tinyxml2 files here directly as there are only two of them.
 
*/


#ifndef __URDFMANAGER_H__
#include "URDFManager.h"
#endif



#include <iostream>
#include <queue>
#include <map>
#include <set>


bool
URDFManager::save( const std::string &path, const BModel &model, bool add_base ) const
{
    URDFModel dst = toURDFModel(model, add_base); 
    
    return m_writer.save(path, dst);
}

BModel
URDFManager::load( const std::string &path ) const
{
    URDFModel urdfmodel;
    m_reader.load(path, urdfmodel); 
    
    return toBModel(urdfmodel);
}


URDFLink
URDFManager::toURDFLink( const BBody &body ) const
// tensor[6] -- ixx, ixy, ixz, iyy, iyz, izz
{
    URDFInertial I;
    I.mass = (double) body.I().mass();
    I.Icom = body.I().Icom();
    
    if (I.mass != 0.0)
        I.com = body.I().com();
    else I.com = B_ZERO_3;

    URDFLink link;
    
    link.visuals.push_back( body.params().visual ); 
    link.collisions.push_back( body.params().collider ); 
    link.inertial = I;
    
    return link;
}

URDFJointType
URDFManager::toURDFJointType( BJoint::JType jtype ) const
{
    URDFJointType mytype;
    switch (jtype)
    {
        case BJoint::Revolute   : mytype = URDFJointType::Revolute; break;
        case BJoint::Prismatic  : mytype = URDFJointType::Prismatic; break;
        case BJoint::Fixed2     : mytype = URDFJointType::Fixed; break;
        case BJoint::Fixed1     : mytype = URDFJointType::Fixed; break;
        case BJoint::Helical    : mytype = URDFJointType::Helical; break;
        case BJoint::Spherical  : mytype = URDFJointType::Spherical; break;
        case BJoint::Planar     : mytype = URDFJointType::Planar; break;
        default : mytype = URDFJointType::UNDEFINED; break;
    }
    return mytype;
}

URDFOrigin
URDFManager::toURDFOrigin( const BTransform &X ) const
{
    URDFOrigin origin;
    origin.xyz = X.r();
    origin.rpy = glm::eulerAngles(glm::dquat(X.E()));
    
    return origin;
}

URDFJoint
URDFManager::toURDFJoint( const BJoint &j ) const
{
    URDFJoint joint;
    
    joint.type = toURDFJointType(j.jtype());
    
    if (joint.type == URDFJointType::Helical)
    {
        std::cout << "URDFManager::Error unsupported joint type " << toString(joint.type) << std::endl;
        exit(EXIT_FAILURE);
    }
  
    joint.origin = toURDFOrigin(j.X_T());

    // axis/limits - only needed for prismatic and revolute
    if (j.jtype() == BJoint::Revolute) 
    {
        joint.axis = j.axis(0).ang();
        joint.limit = j.params().limit;

    } 
    else if (j.jtype() == BJoint::Prismatic) 
    {
        joint.axis =  j.axis(0).lin();
        joint.limit = j.params().limit;
    } 
    
    joint.dynamics = j.params().dynamics; 
    
    return joint;
}


URDFModel
URDFManager::toURDFModel( BModel model, bool add_base ) const
{
    URDFModel dst;
    
    dst.m_name = model.name();
    
    int N = (int) model.numBody();
   
    // separate the fixed bodies from their movebale parents (undo addBody())
    for (int i = 0; i < model.fixedBody().size(); ++i)
    {
        const BFixedBody &fbody = model.fixedBody()[i];
   
        // subtract the mass/inertia
        BBody &parent_body = model.body(fbody.parentId());
        parent_body.separate( fbody.parentTrans(), fbody.toBody() );
    }
  
    // check for floating base here  (undo addBody())
    int start = 1;
    if (model.joint(1).jtype() == BJoint::TransXYZ && model.joint(2).jtype() == BJoint::Spherical)
    {
        start = 3;
        
        // these values hand chosen; must match up with addBody()
        // we drop body[1]/joint[1] the XYZ component, keep body[2] which has mass
        URDFLink link1 = toURDFLink( model.body(1) );
        link1.name = model.getBodyName( model.body(1).getId() );
        dst.m_links.push_back( link1 );
        
        URDFLink link2 = toURDFLink( model.body(2) );
        link2.name = model.getBodyName( model.body(2).getId() );
        dst.m_links.push_back( link2 );
        
        URDFJoint joint;
        joint.type   = URDFJointType::Floating;
        joint.origin = toURDFOrigin( model.joint(1).X_T() ); // frame transform is stored in joint[1]
        joint.name   = "joint_" + std::to_string( model.joint(2).getId() );
        joint.parent = model.getBodyName( model.parentId( model.body(2).getId() ));
        joint.child  = model.getBodyName( model.body(2).getId() );
        
        // check which joint here; needs documentation
        joint.dynamics = model.joint(2).params().dynamics; 
        
        dst.m_joints.push_back( joint );
    }

    // skip body[0]/joint[0] ROOT header
    for (int i = start; i < N; ++i)
    {
        // BBody
        const BBody &body = model.body(i);
        URDFLink link = toURDFLink( body );
        link.name = model.getBodyName( body.getId() );

        dst.m_links.push_back( link );
        //
    
        // BJoint
        URDFJoint joint = toURDFJoint( model.joint(i) );
        joint.name = "joint_" + std::to_string(model.joint(i).getId());
        joint.parent = model.getBodyName( model.parentId( body.getId() ));
        joint.child = model.getBodyName( body.getId() );
       
        dst.m_joints.push_back( joint );
    }
    
    // write out the fixed bodies and joints
    for (int i = 0; i < model.fixedBody().size(); ++i)
    {
        const BFixedBody &fbody = model.fixedBody()[i];
        
        URDFLink link = toURDFLink( fbody.toBody() );
        link.name     = model.getBodyName( fbody.getId() );
    
        dst.m_links.push_back( link );
        
        // Fixed bodies - we assume BJoint::Fixed1 (compressing)
        URDFJoint joint; 
        
        joint.type       = URDFJointType::Fixed;
        joint.origin     = toURDFOrigin(fbody.parentTrans());
        joint.name       = "fixed_joint_" + std::to_string(i);
        joint.parent     = model.getBodyName( fbody.parentId() );
        joint.child      = model.getBodyName( fbody.getId() );
       
        dst.m_joints.push_back( joint );
    }
    
    // remove references to BModel::ROOT() and add a base or 
    // world link (so RDBL-URDFReader can load this)
    // we assume that __ARB_BASE__ does not already exist
    if (add_base)
    {
        bool add_link = false;
        for (URDFJoint &joint : dst.m_joints)
        {
            if (joint.parent == BModel::ROOT())
            {
                joint.parent = "__ARB_BASE__";
                add_link =  true;
            }
        }
 
        if (add_link)
        {
            URDFLink base;
            base.name = "__ARB_BASE__";
            dst.m_links.push_back(base);
        }
    }
    
    return dst;
}

//
//
//

BTransform 
URDFManager::toBTransform( const URDFOrigin &origin ) const
{
    // URDF coord frames - X-axis is forward, Y-axis is left, Z-axis is up
    // remap URDF coords to 'our/your' coordinate frame here
    glm::dvec3 transform;

    transform[0] = origin.xyz[0];
    transform[1] = origin.xyz[1];
    transform[2] = origin.xyz[2];
   
    // URDF roll , pitch, yaw
    glm::dvec3 euler;

    euler[0] = origin.rpy[0];
    euler[1] = origin.rpy[1];
    euler[2] = origin.rpy[2]; 
    
    glm::dmat3 rot;
    if (arb::nearZero(euler))
        rot = B_IDENTITY_3x3;
    else rot = glm::mat3_cast( glm::dquat(euler) );
    
    return BTransform(rot, transform);
}

BInertia
URDFManager::toBInertia( const URDFInertial &inertial ) const
{
    return BInertia(inertial.mass, inertial.com, inertial.Icom);
}

BJoint::JType
URDFManager::toBJointType( const URDFJoint &joint ) const
{
    BJoint::JType mytype;
    switch (joint.type)
    {
        case URDFJointType::Revolute   : mytype = BJoint::Revolute; break; // limits
        case URDFJointType::Continuous : mytype = BJoint::Revolute; break; // no limits
        case URDFJointType::Prismatic  : mytype = BJoint::Prismatic; break;
        case URDFJointType::Fixed      : mytype = BJoint::Fixed1; break;
        case URDFJointType::Floating   : mytype = BJoint::FloatBase; break;
        case URDFJointType::Helical    : mytype = BJoint::Helical; break;
        case URDFJointType::Planar     : mytype = BJoint::Planar ; break; 
        default : mytype = BJoint::UNDEFINED; break;
    }
    return mytype;
}

BBody
URDFManager::toBBody( const URDFLink &link ) const
{
    BBody body( toBInertia(link.inertial) );
    
    if (!link.visuals.empty())
        body.params().visual = link.visuals[0]; 
    
    if (!link.collisions.empty())
        body.params().collider = link.collisions[0]; 
    
    return body;
}

BJoint
URDFManager::toBJoint( const URDFJoint &joint ) const
{
    BJoint retVal; 
    
    BJoint::JType mytype = toBJointType(joint);
    
    if (joint.type == URDFJointType::Revolute || joint.type == URDFJointType::Continuous) 
    {
        retVal = BJoint(mytype, joint.axis);
    } 
    else if (joint.type == URDFJointType::Prismatic) 
    {
        retVal = BJoint(mytype, joint.axis);
    } 
    else if (joint.type == URDFJointType::Fixed) 
    {
        retVal = BJoint(BJoint::Fixed1); // we use compressing fixed joint type
    } 
    else if (joint.type == URDFJointType::Floating) 
    {
        retVal = BJoint(BJoint::FloatBase);
    } 
    else if (joint.type == URDFJointType::Spherical) 
    {
        retVal = BJoint(BJoint::Spherical);
    }
    else if (joint.type == URDFJointType::Planar) 
    {
        retVal = BJoint(BJoint::Planar);
    }
    else if (joint.type == URDFJointType::Helical) 
    {
        std::cout << "URDFManager::Error while processing joint: Helical joints not yet supported!" << std::endl;
        exit(EXIT_FAILURE);
        //retVal = BJoint(BVector6(axis1,axis2)); // Helical - axis normalized
    } 
    
    retVal.X_T( toBTransform(joint.origin) );    
    retVal.params().limit = joint.limit;
    retVal.params().dynamics = joint.dynamics;
    
    return retVal;
}

std::string
URDFManager::root( const URDFModel &urdfmodel ) const
{
    std::set<std::string> linkset;
    
    for ( int i = 0; i < urdfmodel.m_joints.size(); ++i )
    {
        bool found = false;
        
        for ( int j = 0; j < urdfmodel.m_joints.size(); ++j )
        {
            if (urdfmodel.m_joints[i].parent == urdfmodel.m_joints[j].child)
            {
                found = true;
                break;
            }
        }
        
        if ( found == false )
            linkset.insert( urdfmodel.m_joints[i].parent );
    }
    
    std::vector<std::string> root_names(linkset.begin(), linkset.end());
    assert(root_names.size() == 1); // this is supposed to be a tree
    
    return root_names[0];
}


BModel 
URDFManager::toBModel(const URDFModel &urdfmodel) const
{
    BModel bmodel;
    
    // find the unique root of the tree; a link with no parents
    std::string root_name = root(urdfmodel);
    std::queue<std::string> queue( {root_name} );
    std::map<std::string, BBodyId> link_to_bid;
    
    while (!queue.empty()) 
    {
        std::string current_parent = queue.front();
        queue.pop();

        for (const URDFJoint& joint : urdfmodel.m_joints) 
        {
            if (joint.parent == current_parent) 
            {
                URDFLink mylink;
                for (const URDFLink& link : urdfmodel.m_links) 
                {
                    if (link.name == joint.child)
                    {
                        mylink = link;
                        break;
                    }
                }
                
                BBody mybody   = toBBody(mylink);
                BJoint myjoint = toBJoint(joint);
                BBodyId p_id   = link_to_bid[current_parent];
                
                BBodyId new_id = bmodel.addBody(p_id, toBTransform(joint.origin), myjoint, mybody, mylink.name);
        
                link_to_bid[joint.child] = new_id;
                
                queue.push(joint.child);
            }
        }
    }
    return bmodel;
}
