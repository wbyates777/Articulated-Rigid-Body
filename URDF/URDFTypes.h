/* URDFTypes 28/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   URDFTypes.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 URDF, Unified Robot Description Format is an XML format
 
 see  https://wiki.ros.org/urdf/XML
 
 Simple  intermediate layer of structs that hold URDF data for reading and writing
 uses glm::dvec3, glm::dvec4, glm::dmat3 data types - no spatial algebra stuff
 

 
 Supports URDF Joint Types
 ---------------------------
 revolute   — a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
 continuous — a continuous hinge joint that rotates around the axis and has no upper and lower limits.
 prismatic  — a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
 fixed      — this is not really a joint because it cannot move. All degrees of freedom are locked. 
              This type of joint does not require the <axis>, <calibration>, <dynamics>, <limits> or <safety_controller>.
 floating   — this joint allows motion for all 6 degrees of freedom. Must be first joint
 
 Planar, and Helical are not supported as Helical joint not implemented in URDF, 
 and Planar joint not implemented in BModel.
 
 
 Notes:
 
*/

#ifndef __URDFTYPES_H__
#define __URDFTYPES_H__


#include <string>
#include <vector>
#include <iostream>


#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>


//
// URDFLink
//

struct URDFOrigin 
{
    glm::dvec3 xyz = {0,0,0};
    glm::dvec3 rpy = {0,0,0}; 
};

struct URDFMaterial
{
    glm::dvec4 rgba = {1,1,1,1}; // optional
    std::string name; // not optional
    std::string texture; // optional
};

enum  URDFGeometryType { Box, Cylinder, Sphere, Mesh, NONE };

std::string
toString(URDFGeometryType type);

URDFGeometryType 
toGeometryType(const std::string &type);

struct URDFGeometry
{
    glm::dvec3 size = {0,0,0};  // sphere(r), box(lx,ly,lz), cylinder(r,h) 
    glm::dvec3 scale = {1,1,1};
    std::string filename;       // for meshes (.stl, .obj)
    URDFGeometryType type = URDFGeometryType::NONE;
};

struct URDFInertial 
{
    double mass = 0;
    glm::dvec3 com = {0,0,0};
    glm::dmat3 Icom = glm::dmat3(0.0);
};

struct URDFVisual 
{
    std::string  name; // optional
    URDFGeometry geometry;
    URDFOrigin   origin;
    URDFMaterial material;
};

struct URDFCollision 
{
    std::string  name; // optional
    URDFGeometry geometry;
    URDFOrigin   origin;
};

struct URDFLink 
{
    std::string name;
    URDFInertial inertial;
    std::vector<URDFVisual> visuals; 
    std::vector<URDFCollision> collisions; 
};



//
// URDFJoint
//

struct URDFLimit 
{
    double lower_limit    = 0.0;   //  (optional, defaults to 0)
    double upper_limit    = 0.0;   //  (optional, defaults to 0)
    double effort_limit   = 1.0;   //  (requied, defaults to 1)  max force [N] or [N∙m]
    double velocity_limit = 1.0;   //  (requied, defaults to 1)
};

struct URDFDynamics 
{
    double damping  = 0.0;  // (optional, defaults to 0) [N∙s/m] or  [N∙m∙s/rad] 
    double friction = 0.0;  // (optional, defaults to 0) static friction value  [N] or [N∙m]
};

enum URDFJointType { Revolute, Continuous, Prismatic, Fixed, Floating, Planar, Helical, Spherical, UNDEFINED };

std::string
toString(URDFJointType type);

URDFJointType 
toURDFJointType(const std::string &type);

struct URDFJoint 
{
    std::string name;
    std::string parent;
    std::string child;
    
    URDFJointType type = URDFJointType::UNDEFINED;
    URDFOrigin origin;
    glm::dvec3 axis = {1,0,0};
    
    URDFLimit limit;       // limits - only needed for prismatic and revolute
    URDFDynamics dynamics; // optional
};

//
// URDFModel
//

struct URDFModel 
{
    std::string m_name;
    std::vector<URDFLink> m_links;
    std::vector<URDFJoint> m_joints;
};

//
// BModel BBodyParams and BJointParams component parameters 1,...,N_B
//

struct BBodyParams
{
    URDFVisual visual;
    URDFCollision collider;
};

std::ostream&
operator<<( std::ostream &ostr, const BBodyParams &bp );

std::istream& 
operator>>( std::istream &istr, BBodyParams &bp );


struct BJointParams
{
    URDFLimit limit;
    URDFDynamics dynamics;
};


std::ostream&
operator<<( std::ostream &ostr, const BJointParams &jp );

std::istream& 
operator>>( std::istream &istr, BJointParams &jp );


#endif


