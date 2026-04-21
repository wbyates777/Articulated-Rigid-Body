/* URDFWriter 29/03/2026

 $$$$$$$$$$$$$$$$$$$$
 $   URDFWriter.h   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 URDF, Unified Robot Description Format is an XML format
 
 This class implements a subset of the URDF XML format 
 Used for loading/saving articulated bodies (BMultibody).
 
 Depends on tinyxml2 https://github.com/leethomason/tinyxml2
 
 see  https://wiki.ros.org/urdf/XML
 
 TODO: improve error handling 
 
*/


#ifndef __URDFWRITER_H__
#define __URDFWRITER_H__

#include <string>

#ifndef __URDFTYPES_H__
#include "URDFTypes.h" 
#endif


class URDFModel;

namespace tinyxml2
{
    class XMLElement;
    class XMLDocument;
};



class URDFWriter
{

public:

    URDFWriter( void ) : m_dp(8) {};
    ~URDFWriter( void )=default;

    
    void
    setDP( int dp ) { m_dp = dp; }
    
    int
    getDP( void ) const { return m_dp; }  
    
    
    bool 
    save( const std::string &path, const URDFModel &model ) const; 
    
    
private:

    // helper to format vectors to space-separated strings
    std::string  
    format( const glm::dvec3 &v ) const;
    
    std::string 
    format( const glm::dvec4 &v ) const;
    
    std::string
    format( double v ) const;
    
    // write methods
    void 
    writeOrigin(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFOrigin &orig) const;
    
    void 
    writeLimit(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFLimit &limit) const;
    
    void 
    writeDynamics(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFDynamics &dynamics) const;

    void 
    writeMaterial(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFMaterial &mat) const;
    
    void 
    writeGeometry(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFGeometry &geom) const;
    
    void 
    writeVisual(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFVisual &vis) const;
   
    void 
    writeCollision(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFCollision &col) const;
    
    void 
    writeInertial(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFInertial &inertial) const;
    
    void 
    writeLink(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFLink &link) const;
    
    void 
    writeJoint(tinyxml2::XMLDocument &doc, tinyxml2::XMLElement *parent, const URDFJoint &joint) const;
    
    int m_dp;

};

#endif


