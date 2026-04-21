/* URDFReader 29/03/2026

 $$$$$$$$$$$$$$$$$$$$
 $   URDFReader.h   $
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


#ifndef __URDFREADER_H__
#define __URDFREADER_H__

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



class URDFReader
{

public:

    URDFReader( void )=default;
    ~URDFReader( void )=default;

    bool 
    load( const std::string &path, URDFModel &model ) const;
    
private:

    void
    readInertial( const tinyxml2::XMLElement *xml, URDFInertial &I ) const;
    
    void
    readOrigin( const tinyxml2::XMLElement *xml, URDFOrigin &orig ) const;
    
    void
    readDynamics( const tinyxml2::XMLElement *xml, URDFDynamics &dynamics ) const;
    
    void
    readLimit( const tinyxml2::XMLElement *xml, URDFLimit &limit ) const;
    
    void
    readGeometry( const tinyxml2::XMLElement *xml, URDFGeometry &geom ) const;
    
    void
    readMaterial( const tinyxml2::XMLElement *xml, URDFMaterial &mat ) const;
    
    void
    readVisual( const tinyxml2::XMLElement *xml, URDFVisual &vis ) const;
    
    void
    readCollision( const tinyxml2::XMLElement *xml, URDFCollision &col ) const;
    
    
    void
    readLink( const tinyxml2::XMLElement *xml, URDFLink &link ) const;
    
    void
    readJoint( const tinyxml2::XMLElement *xml, URDFJoint &joint ) const;

};

#endif


