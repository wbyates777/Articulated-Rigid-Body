/* URDFReader 29/03/2026

 $$$$$$$$$$$$$$$$$$$$$$
 $   URDFReader.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 URDF, Unified Robot Description Format is an XML format
 
 This class implements a subset of the URDF XML format 
 Used for loading/saving articulated bodies (BMultibody).
 
 Depends on tinyxml2 https://github.com/leethomason/tinyxml2
 
 see  https://wiki.ros.org/urdf/XML
 
*/


#ifndef __URDFREADER_H__
#include "URDFReader.h"
#endif

#include <iostream>

#include "tinyxml2.h"
using namespace tinyxml2;



// we want to keep this class decoupled from the spatial algebra implemenation


void
URDFReader::readOrigin( const XMLElement* xml, URDFOrigin &orig ) const 
{
    const char* xyz = xml->Attribute("xyz");
    if (xyz) 
        std::sscanf(xyz, "%lf %lf %lf", &orig.xyz[0], &orig.xyz[1], &orig.xyz[2]);
    
    const char* rpy = xml->Attribute("rpy");
    if (rpy) 
        std::sscanf(rpy, "%lf %lf %lf", &orig.rpy[0], &orig.rpy[1], &orig.rpy[2]);
}

void
URDFReader::readMaterial( const XMLElement* xml, URDFMaterial &mat ) const 
{
    if (xml->Attribute("name")) 
    {
        mat.name = xml->Attribute("name");
    }
    
    if (const XMLElement* color = xml->FirstChildElement("color")) 
    {
        const char* rgba = color->Attribute("rgba");
        if (rgba) 
            std::sscanf(rgba, "%lf %lf %lf %lf", &mat.rgba[0], &mat.rgba[1], &mat.rgba[2], &mat.rgba[3]);
    }
    
    if (const XMLElement* texture = xml->FirstChildElement("texture")) 
    {
        const char* filename = texture->Attribute("filename");
        if (filename) 
            mat.texture = filename;
    }
}

void
URDFReader::readGeometry( const XMLElement* xml, URDFGeometry &geom ) const 
{
    if (const XMLElement* box = xml->FirstChildElement("box")) 
    {
        geom.type = URDFGeometryType::Box;
        
        const char* size = box->Attribute("size");
        if (size)
            std::sscanf(size, "%lf %lf %lf", &geom.size[0], &geom.size[1], &geom.size[2]);
    } 
    else if (const XMLElement* cyl = xml->FirstChildElement("cylinder")) 
    {
        geom.type = URDFGeometryType::Cylinder;
        geom.size[0] = cyl->DoubleAttribute("radius");
        geom.size[1] = cyl->DoubleAttribute("length");
    }
    else if (const XMLElement* sph = xml->FirstChildElement("sphere"))
    {
        geom.type = URDFGeometryType::Sphere;
        geom.size[0] = sph->DoubleAttribute("radius");
    }
    else if (const XMLElement* mesh = xml->FirstChildElement("mesh"))
    {
        geom.type = URDFGeometryType::Mesh;
        geom.filename = mesh->Attribute("filename");
     
        // optional
        const char* scale = mesh->Attribute("scale");
        if (scale)
            std::sscanf(scale, "%lf %lf %lf", &geom.scale[0], &geom.scale[1], &geom.scale[2]);
    }
}


void
URDFReader::readVisual( const XMLElement* xml, URDFVisual &vis ) const 
{
    if (xml->Attribute("name")) 
    {
        vis.name = xml->Attribute("name");
    }
    
    // read visual origin
    if (const XMLElement* origin = xml->FirstChildElement("origin")) 
    {
        readOrigin(origin, vis.origin);
    }
    
    // read geometry
    if (const XMLElement* geom = xml->FirstChildElement("geometry"))
    {
        readGeometry(geom, vis.geometry);
    }
    
    // optional
    if (const XMLElement* mat = xml->FirstChildElement("material"))
    {
        readMaterial( mat, vis.material );
    }
}

void
URDFReader::readCollision( const XMLElement* xml, URDFCollision &col ) const 
{
    if (xml->Attribute("name")) 
        col.name = xml->Attribute("name");

    // read visual origin
    if (const XMLElement* origin = xml->FirstChildElement("origin")) 
    {
        readOrigin(origin, col.origin);
    }
    
    // read geometry
    if (const XMLElement* geom = xml->FirstChildElement("geometry"))
    {
        readGeometry(geom, col.geometry);
    }
}

void
URDFReader::readInertial( const XMLElement* xml, URDFInertial &I ) const 
{
    if (const XMLElement *mass = xml->FirstChildElement("mass")) 
    {
        I.mass = mass->DoubleAttribute("value");
    }
    
    if (const XMLElement *origin = xml->FirstChildElement("origin")) 
    {
        const char *xyz = origin->Attribute("xyz");
        if (xyz) 
            std::sscanf(xyz, "%lf %lf %lf", &I.com[0], &I.com[1], &I.com[2]);
    }
    
    if (const XMLElement *inertia = xml->FirstChildElement("inertia"))
    {
        I.Icom = glm::dmat3(0.0);
        I.Icom[0][0] = inertia->DoubleAttribute("ixx");
        I.Icom[0][1] = inertia->DoubleAttribute("ixy");
        I.Icom[0][2] = inertia->DoubleAttribute("ixz");
        I.Icom[1][1] = inertia->DoubleAttribute("iyy");
        I.Icom[1][2] = inertia->DoubleAttribute("iyz");
        I.Icom[2][2] = inertia->DoubleAttribute("izz");
        
        I.Icom[1][0] = I.Icom[0][1]; // iyx = ixy
        I.Icom[2][0] = I.Icom[0][2]; // izx = ixz
        I.Icom[2][1] = I.Icom[1][2]; // izy = iyz
    }
}


//
// Links
//

void
URDFReader::readLink( const XMLElement* xml, URDFLink &link ) const
{
    if (xml->Attribute("name")) 
        link.name = xml->Attribute("name");
    
    // read inertia
    if (const XMLElement* inertial = xml->FirstChildElement("inertial"))
    {
        readInertial( inertial, link.inertial );
    }
    
    // read visuals (you can have more than one visual per link)
    const XMLElement* vis = xml->FirstChildElement("visual"); 
    while (vis != nullptr) 
    {
        URDFVisual visual;
        readVisual( vis, visual );
        link.visuals.push_back(visual);
        
        vis = vis->NextSiblingElement("visual");
    }
        
    // read collisions (you can have more than one collision per link)
    const XMLElement* col = xml->FirstChildElement("collision"); 
    while (col != nullptr) 
    {
        URDFCollision collision;
        readCollision( col, collision );
        link.collisions.push_back(collision);
        
        col = col->NextSiblingElement("collision");
    }
}

//
// Joints
//
void
URDFReader::readLimit( const tinyxml2::XMLElement *xml, URDFLimit &limit ) const
{
    xml->QueryDoubleAttribute("lower", &limit.lower_limit);
    xml->QueryDoubleAttribute("upper", &limit.upper_limit);
    xml->QueryDoubleAttribute("effort", &limit.effort_limit);
    xml->QueryDoubleAttribute("velocity", &limit.velocity_limit);
}

void
URDFReader::readDynamics( const tinyxml2::XMLElement *xml, URDFDynamics &dynamics ) const
{
    xml->QueryDoubleAttribute("damping", &dynamics.damping);
    xml->QueryDoubleAttribute("friction", &dynamics.friction);
}

void
URDFReader::readJoint( const XMLElement* xml, URDFJoint &joint ) const
{
    if (xml->Attribute("name")) 
        joint.name = xml->Attribute("name");
    
    joint.type = toURDFJointType(xml->Attribute("type"));

    joint.parent = xml->FirstChildElement("parent")->Attribute("link");
    joint.child  = xml->FirstChildElement("child")->Attribute("link");
    
    if (const XMLElement* origin = xml->FirstChildElement("origin")) 
    {
        readOrigin(origin, joint.origin);
    }
    
    if (const XMLElement* axis = xml->FirstChildElement("axis")) 
    {
        const char* xyz = axis->Attribute("xyz");
        if (xyz) 
            std::sscanf(xyz, "%lf %lf %lf", &joint.axis[0], &joint.axis[1], &joint.axis[2]);
    }
  
    if (const XMLElement* limit = xml->FirstChildElement("limit")) 
    {
        readLimit(limit, joint.limit);
    }
    
    if (const XMLElement* limit = xml->FirstChildElement("dynamics")) 
    {
        readDynamics(limit, joint.dynamics);
    }
    
}


bool 
URDFReader::load(const std::string &path, URDFModel &model) const
{
    model.m_links.clear();
    model.m_joints.clear();
    
    XMLDocument doc;
    
    if (doc.LoadFile(path.c_str()) != XML_SUCCESS) 
    {
        std::cout << "URDF Error: Could not open file " << path << std::endl;
        std::cout << "Error ID: " << doc.ErrorID() << std::endl;
        std::cout << "Error Name: " << doc.ErrorName() << std::endl;
        std::cout << "Error Details: " << doc.ErrorStr() << std::endl;
        std::cout << "Error Line: " << doc.ErrorLineNum() << std::endl;
        return false;
    }
    
    XMLElement* robot = doc.FirstChildElement("robot");
    
    if (!robot) 
    {
        std::cout << "URDF Error: Could not find robot tag in " << path << std::endl;
        return false;
    }

    model.m_name = robot->Attribute("name");
    
    // read Links
    XMLElement* l = robot->FirstChildElement("link");
    while (l != nullptr) 
    {
        URDFLink link;
        readLink( l, link );
        model.m_links.push_back(link);
        l = l->NextSiblingElement("link");
    }
    
    // read Joints
    XMLElement* j = robot->FirstChildElement("joint"); 
    while (j != nullptr) 
    {
        URDFJoint joint;
        readJoint( j, joint );
        model.m_joints.push_back(joint);
        j = j->NextSiblingElement("joint");
    }
    
    return true;
}
