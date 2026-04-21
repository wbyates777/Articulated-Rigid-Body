/* URDFWriter 29/03/2026

 $$$$$$$$$$$$$$$$$$$$$$
 $   URDFWriter.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 URDF, Unified Robot Description Format is an XML format
 
 This class implements a subset of the URDF XML format 
 Used for loading/saving articulated bodies (BMultibody).
 
 Depends on tinyxml2 https://github.com/leethomason/tinyxml2
 
 see  https://wiki.ros.org/urdf/XML
 
 
 For c++17 the following code:
 
    return std::format("{:.{}f} {:.{}f} {:.{}f}", v[0], m_dp, v[1], m_dp, v[2], m_dp);
 
 can be replaced with:
 
     char buf[128];
     std::snprintf(buf, sizeof(buf), "%.*f %.*f %.*f", m_dp, v[0], m_dp, v[1], m_dp, v[2]);
     return buf;
 
*/


#ifndef __URDFWRITER_H__
#include "URDFWriter.h"
#endif


#include "tinyxml2.h"
#include <iostream>

#include <format> 
//#include <cstdio> // c++17 for snprintf

using namespace tinyxml2;


std::string 
URDFWriter::format( const glm::dvec3 &v ) const 
{
    return std::format("{:.{}f} {:.{}f} {:.{}f}", v[0], m_dp, v[1], m_dp, v[2], m_dp);
}

std::string 
URDFWriter::format( const glm::dvec4 &v ) const 
{
    return std::format("{:.{}f} {:.{}f} {:.{}f} {:.{}f}", v[0], m_dp, v[1], m_dp, v[2], m_dp, v[3], m_dp);
}

std::string
URDFWriter::format( double v ) const
{
    return std::format("{:.{}f}", v, m_dp);
}

//

void 
URDFWriter::writeOrigin(XMLDocument &doc, XMLElement *parent, const URDFOrigin &orig) const 
{
    XMLElement *xml = doc.NewElement("origin");
    xml->SetAttribute("xyz", format(orig.xyz).c_str());
    xml->SetAttribute("rpy", format(orig.rpy).c_str());
    parent->InsertEndChild(xml);
}

void 
URDFWriter::writeMaterial(XMLDocument &doc, XMLElement *parent, const URDFMaterial &mat) const 
{
    // URDF expects a name for a material; no name implies no material 
    if (!mat.name.empty()) 
    {
        XMLElement *xml = doc.NewElement("material");
        
        xml->SetAttribute("name", mat.name.c_str());
        
        XMLElement *color = doc.NewElement("color");
        color->SetAttribute("rgba", format(mat.rgba).c_str());
        xml->InsertEndChild(color);
        
        if (!mat.texture.empty()) 
        {
            XMLElement *tex = doc.NewElement("texture");
            tex->SetAttribute("filename", mat.texture.c_str());
            xml->InsertEndChild(tex);
        }
        
        parent->InsertEndChild(xml);
    }
}

void 
URDFWriter::writeGeometry(XMLDocument &doc, XMLElement *parent, const URDFGeometry &geom) const 
{

    XMLElement *xml = doc.NewElement("geometry");
    
    if (geom.type == URDFGeometryType::Box) 
    {
        XMLElement *box = doc.NewElement("box");
        box->SetAttribute("size", format(geom.size).c_str());
        xml->InsertEndChild(box);
    } 
    else if (geom.type == URDFGeometryType::Cylinder) 
    {
        XMLElement *cyl = doc.NewElement("cylinder");
        cyl->SetAttribute("radius", format(geom.size[0]).c_str());
        cyl->SetAttribute("length", format(geom.size[1]).c_str());
        xml->InsertEndChild(cyl);
    } 
    else if (geom.type == URDFGeometryType::Sphere) 
    {
        XMLElement *sph = doc.NewElement("sphere");
        sph->SetAttribute("radius", format(geom.size[0]).c_str());
        xml->InsertEndChild(sph);
    } 
    else if (geom.type == URDFGeometryType::Mesh) 
    {
        XMLElement *mesh = doc.NewElement("mesh");
        mesh->SetAttribute("filename", geom.filename.c_str());
        if (geom.scale != glm::dvec3(1,1,1)) 
        {
            mesh->SetAttribute("scale", format(geom.scale).c_str());
        }
        xml->InsertEndChild(mesh);
    }
    
    parent->InsertEndChild(xml);

}

void 
URDFWriter::writeVisual(XMLDocument &doc, XMLElement *parent, const URDFVisual &vis) const 
{
    if (vis.geometry.type != URDFGeometryType::NONE)
    {
        XMLElement *xml = doc.NewElement("visual");
        if (!vis.name.empty()) 
            xml->SetAttribute("name", vis.name.c_str());
        
        writeOrigin(doc, xml, vis.origin);
        writeGeometry(doc, xml, vis.geometry);
        writeMaterial(doc, xml, vis.material);
        
        parent->InsertEndChild(xml);
    }
}

void 
URDFWriter::writeCollision(XMLDocument &doc, XMLElement *parent, const URDFCollision &col) const 
{
    if (col.geometry.type != URDFGeometryType::NONE)
    {
        XMLElement *xml = doc.NewElement("collision");
        if (!col.name.empty()) 
            xml->SetAttribute("name", col.name.c_str());
        
        writeOrigin(doc, xml, col.origin);
        writeGeometry(doc, xml, col.geometry);
        
        parent->InsertEndChild(xml);
    }
}

void 
URDFWriter::writeInertial(XMLDocument &doc, XMLElement *parent, const URDFInertial &inertial) const 
{
    XMLElement *xml = doc.NewElement("inertial");
    
    if (inertial.mass != 0.0)
    {
        XMLElement *mass = doc.NewElement("mass");
        mass->SetAttribute("value", format(inertial.mass).c_str());
        xml->InsertEndChild(mass);
        
        XMLElement *origin = doc.NewElement("origin");
        origin->SetAttribute("xyz", format(inertial.com).c_str());
        xml->InsertEndChild(origin);
        
        XMLElement *tensor = doc.NewElement("inertia");
        tensor->SetAttribute("ixx", format(inertial.Icom[0][0]).c_str());
        tensor->SetAttribute("ixy", format(inertial.Icom[0][1]).c_str());
        tensor->SetAttribute("ixz", format(inertial.Icom[0][2]).c_str());
        tensor->SetAttribute("iyy", format(inertial.Icom[1][1]).c_str());
        tensor->SetAttribute("iyz", format(inertial.Icom[1][2]).c_str());
        tensor->SetAttribute("izz", format(inertial.Icom[2][2]).c_str());
        xml->InsertEndChild(tensor);
        
        parent->InsertEndChild(xml);
    }
}

void 
URDFWriter::writeLink(XMLDocument &doc, XMLElement *parent, const URDFLink &link) const 
{
    XMLElement *xml = doc.NewElement("link");
    xml->SetAttribute("name", link.name.c_str());

    writeInertial(doc, xml, link.inertial);

    if (!link.visuals.empty())
    {
        for (const URDFVisual &vis : link.visuals) 
        {
            writeVisual(doc, xml, vis);
        }
    }

    if (!link.collisions.empty())
    {
        for (const URDFCollision &col : link.collisions) 
        {
            writeCollision(doc, xml, col);
        }
    }

    parent->InsertEndChild(xml);
}




void 
URDFWriter::writeLimit(XMLDocument &doc, XMLElement *parent, const URDFLimit &limit) const 
{
    XMLElement *xml = doc.NewElement("limit");
    xml->SetAttribute("lower", format(limit.lower_limit).c_str());
    xml->SetAttribute("upper", format(limit.upper_limit).c_str());
    xml->SetAttribute("effort", format(limit.effort_limit).c_str());
    xml->SetAttribute("velocity", format(limit.velocity_limit).c_str());
    parent->InsertEndChild(xml);
}


void 
URDFWriter::writeDynamics(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFDynamics &dynamics) const
{
    XMLElement *xml = doc.NewElement("dynamics");
    xml->SetAttribute("damping", format(dynamics.damping).c_str());
    xml->SetAttribute("friction", format(dynamics.friction).c_str());
    parent->InsertEndChild(xml);
}
    
    
void 
URDFWriter::writeJoint(XMLDocument &doc, XMLElement *parent, const URDFJoint &joint) const 
{
    XMLElement *xml = doc.NewElement("joint");
    xml->SetAttribute("name", joint.name.c_str());
    
    xml->SetAttribute("type", toString(joint.type).c_str());

    writeOrigin(doc, xml, joint.origin);

    XMLElement *parent_xml = doc.NewElement("parent");
    parent_xml->SetAttribute("link", joint.parent.c_str());
    xml->InsertEndChild(parent_xml);

    XMLElement *child_xml = doc.NewElement("child");
    child_xml->SetAttribute("link", joint.child.c_str());
    xml->InsertEndChild(child_xml);

    XMLElement *axis_xml = doc.NewElement("axis");
    axis_xml->SetAttribute("xyz", format(joint.axis).c_str());
    xml->InsertEndChild(axis_xml);

    // only write limits for joints that require them:Revolute and Prismatic
    if (joint.type == URDFJointType::Revolute || joint.type == URDFJointType::Prismatic) 
    {
        writeLimit(doc, xml, joint.limit); 
    }

    writeDynamics(doc, xml, joint.dynamics);
                  
    parent->InsertEndChild(xml);
}


bool 
URDFWriter::save( const std::string &path, const URDFModel &model ) const 
{
    XMLDocument doc;
    
    XMLDeclaration *decl = doc.NewDeclaration("xml version=\"1.0\" encoding=\"utf-8\"");
    
    doc.InsertFirstChild(decl);

    XMLElement *xml = doc.NewElement("robot");
    xml->SetAttribute("name", model.m_name.c_str());
    doc.InsertEndChild(xml);

    for (const URDFLink &link : model.m_links) 
    {
        writeLink(doc, xml, link);
    }

    for (const URDFJoint &joint : model.m_joints) 
    {
        writeJoint(doc, xml, joint);
    }

    XMLError eResult = doc.SaveFile(path.c_str());
    
    if (eResult != XML_SUCCESS) 
    {
        std::cout << "URDF Error: Could not save file " << path << "\n";
        std::cout << "Error ID: " << doc.ErrorID() << "\n";
        std::cout << "Error Name: " << doc.ErrorName() << std::endl;
        std::cout << "Error Details: " << doc.ErrorStr() << std::endl;
        std::cout << "Error Line: " << doc.ErrorLineNum() << std::endl;
        return false;
    }

    return true;
}
