/* URDFTypes 28/03/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   URDFTypes.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/

#ifndef __URDFTYPES_H__
#include "URDFTypes.h" 
#endif

#include <iomanip>

std::string
toString(URDFGeometryType type) 
{
    switch (type)
    {
        case URDFGeometryType::Box      : return "box"; break;
        case URDFGeometryType::Cylinder : return "cylinder"; break;
        case URDFGeometryType::Sphere   : return "sphere"; break;
        case URDFGeometryType::Mesh     : return "mesh"; break;
        default : return "NONE"; break;
    }
}

URDFGeometryType 
toGeometryType(const std::string &type) 
{
    if (type == "box")      return URDFGeometryType::Box;
    if (type == "cylinder") return URDFGeometryType::Cylinder;
    if (type == "sphere")   return URDFGeometryType::Sphere;
    if (type == "mesh")     return URDFGeometryType::Mesh;
    return URDFGeometryType::NONE;
}

std::string
toString(URDFJointType type) 
{
    switch (type)
    {
        case URDFJointType::Revolute   : return "revolute"; break;
        case URDFJointType::Continuous : return "continuous"; break;
        case URDFJointType::Prismatic  : return "prismatic"; break;
        case URDFJointType::Fixed      : return "fixed"; break;
        case URDFJointType::Floating   : return "floating"; break;
        case URDFJointType::Planar     : return "planar"; break; 
        case URDFJointType::Helical    : return "helical"; break;
        case URDFJointType::Spherical  : return "spherical"; break;
        default : return "UNDEFINED"; break;
    }
}

URDFJointType 
toURDFJointType(const std::string &type) 
{
    if (type == "revolute")   return URDFJointType::Revolute;
    if (type == "continuous") return URDFJointType::Continuous;
    if (type == "prismatic")  return URDFJointType::Prismatic;
    if (type == "fixed")      return URDFJointType::Fixed;
    if (type == "floating")   return URDFJointType::Floating;
    if (type == "planar")     return URDFJointType::Planar;
    if (type == "helical")    return URDFJointType::Helical;
    if (type == "spherical")  return URDFJointType::Spherical;
    return URDFJointType::UNDEFINED;
}

//
//
//


// we want to keep this class decoupled from the spatial algebra implemenation

template<typename T> inline std::ostream&
operator<<( std::ostream &ostr, const  glm::vec<3,T> &v )
{
    ostr << v.x << ',' << v.y << ',' << v.z  << ' ';
    return ostr;
}

template<typename T> inline std::istream&
operator>>( std::istream &istr, glm::vec<3,T> &v )
{
    char delim;
    istr >> v.x >> delim >> v.y >> delim >> v.z;
    return istr;
}


template<typename T> inline std::ostream&
operator<<( std::ostream &ostr, const  glm::vec<4,T> &v )
{
    ostr << v.x << ',' << v.y << ',' << v.z  << ',' << v.w << ' ';
    return ostr;
}

template<typename T> inline std::istream&
operator>>( std::istream &istr, glm::vec<4,T> &v )
{
    char delim;
    istr >> v.x >> delim >> v.y >> delim >> v.z >> delim >> v.w;
    return istr;
}

template<typename T> inline std::ostream&
operator<<( std::ostream &ostr, const glm::mat<3,3,T> &m )
{
    ostr << m[0][0] << ' ' << m[0][1] << ' ' << m[0][2] << '\n'
         << m[1][0] << ' ' << m[1][1] << ' ' << m[1][2] << '\n'
         << m[2][0] << ' ' << m[2][1] << ' ' << m[2][2] << '\n';
    return ostr;
}

template<typename T> inline std::istream&
operator>>( std::istream &istr, glm::mat<3,3,T> &m )
{
    istr >> m[0][0] >> m[0][1] >> m[0][2] 
         >> m[1][0] >> m[1][1] >> m[1][2]
         >> m[2][0] >> m[2][1] >> m[2][2];
    return istr;
}

//
//
//

std::ostream&
operator<<( std::ostream &ostr, const URDFOrigin &origin )
{
    ostr << origin.xyz << ' ' << origin.rpy << ' ';    
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFOrigin &origin )
{
    istr >> origin.xyz >> origin.rpy;
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFMaterial &mat )
{
    ostr << std::quoted(mat.name) << ' ';
    ostr << mat.rgba << ' ';
    ostr << std::quoted(mat.texture) << ' ';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFMaterial &mat )
{
    istr >> std::quoted(mat.name);
    istr >> mat.rgba;
    istr >> std::quoted(mat.texture);    
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFGeometry &geom )
{
    ostr << geom.size << ' ';
    ostr << geom.scale << ' ';
    ostr << toString(geom.type) << ' ';
    ostr << std::quoted(geom.filename) << ' ';    
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFGeometry &geom )
{
    istr >> geom.size;
    istr >> geom.scale;
    std::string type;
    istr >> type;
    geom.type = toGeometryType(type);
    istr >> std::quoted(geom.filename);
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFVisual &visual )
{
    ostr << std::quoted(visual.name) << ' ';
    ostr << visual.geometry << ' ';
    ostr << visual.origin << ' ';
    ostr << visual.material << ' ';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFVisual &visual )
{
    istr >> std::quoted(visual.name);
    istr >> visual.geometry;
    istr >> visual.origin;
    istr >> visual.material;
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFCollision &collision )
{
    ostr << std::quoted(collision.name) << ' ';
    ostr << collision.geometry << ' ';
    ostr << collision.origin << ' ';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFCollision &collision )
{
    istr >> std::quoted(collision.name);
    istr >> collision.geometry;
    istr >> collision.origin;
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFLimit &limit )
{
    ostr << limit.lower_limit << ' ';
    ostr << limit.upper_limit << ' ';
    ostr << limit.effort_limit << ' ';
    ostr << limit.velocity_limit << ' ';    
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFLimit &limit )
{
    istr >> limit.lower_limit;
    istr >> limit.upper_limit;
    istr >> limit.effort_limit;
    istr >> limit.velocity_limit;
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const URDFDynamics &dynamics )
{
    ostr << dynamics.damping << ' ' << dynamics.friction << ' ';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, URDFDynamics &dynamics )
{
    istr >> dynamics.damping >> dynamics.friction;
    return istr;
}

//
//
//

std::ostream&
operator<<( std::ostream &ostr, const BBodyParams &bp )
{
    ostr << bp.visual << '\n' << bp.collider << '\n';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, BBodyParams &bp )
{
    istr >> bp.visual >> bp.collider;
    return istr;
}

std::ostream&
operator<<( std::ostream &ostr, const BJointParams &jp )
{
    ostr << jp.limit << '\n' << jp.dynamics << '\n';
    return ostr;
}

std::istream& 
operator>>( std::istream &istr, BJointParams &jp )
{
    istr >> jp.limit >> jp.dynamics;
    return istr;
}

