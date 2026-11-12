/*!
 * @defgroup URDF URDF Manager
 * @brief Unified Robot Description Format (URDF) load and save.
 */

/*! 
         $$$$$$$$$$$$$$$$$$$$$
         $   URDFManager.h   $
         $$$$$$$$$$$$$$$$$$$$$

 @file URDFManager.h
 @ingroup URDF
 date 11/04/2026
 
 by W.B. Yates
 
 ---
 
 @brief Manager class for URDF reading and writing.
 
 URDF, Unified Robot Description Format is an XML format.
 
 This class implements a subset of the URDF XML format
 used for loading/saving articulated body models (BModel).
 
 ARB URDF models that use FloatBase can be read directly 
 by the RBDL-URDFReader addon, and thus cross verified.
 
 However, the RDBL-URDFReader is quite strict/picky, and in general only ARB URDF models 
 that are saved with 'add_base=true' can be read by the RBDL-URDFReader addon. 
 
 This project depends on tinyxml2. I have included the tinyxml2 files here directly as there are only two of them.
 

  Supports URDF Joint Types

 - revolute    a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.
 - continuous  a continuous hinge joint that rotates around the axis and has no upper and lower limits.
 - prismatic   a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
 - fixed       this is not really a joint because it cannot move. All degrees of freedom are locked. 
               This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
 - floating    this joint allows motion for all 6 degrees of freedom. Must be first joint
 - planar      3-DoF joint allows motion x,y translation and z rotation
 
 Supports non-URDF Joint Types
  - spherical  3-DoF spherical or 'ball and socket' joint
 
 @see https://wiki.ros.org/urdf/XML
 @see https://github.com/leethomason/tinyxml2 

@warning when loading or saving a model make sure that the model only contains joint types supported in the URDF standard.
*/


#ifndef URDFMANAGER_H
#define URDFMANAGER_H


#include <string>

#ifndef BMODEL_H
#include "BModel.h"
#endif

#ifndef URDFREADER_H
#include "URDFReader.h"
#endif

#ifndef URDFWRITER_H
#include "URDFWriter.h"
#endif


class URDFManager
{

public:

    URDFManager( void ) : m_reader(), m_writer() {}
    ~URDFManager( void )=default;

    /// save to a URDF file; set add_base to true to generate RDBL readable XML
    bool
    save( const std::string &path, const BModel &model, bool add_base = false ) const;
    
    /// load a URDF file from filename 
    BModel
    load( const std::string &filename ) const;
    
    /// number of decimal places for save
    void
    setDP( int dp ) { m_writer.setDP(dp); }
    
    int
    getDP( void ) const { return  m_writer.getDP(); } 
    
private:

    URDFJointType
    toURDFJointType( BJoint::JType jtype ) const;
    
    URDFOrigin
    toURDFOrigin( const BTransform &X ) const;
    
    URDFJoint
    toURDFJoint( const BJoint &j )const;
    
    URDFLink
    toURDFLink( const BBody &body )const;
    
    URDFModel
    toURDFModel( BModel model, bool add_base ) const;

    
    //
    //
    //

    BJoint::JType
    toBJointType( const URDFJoint &joint ) const;
    
    BTransform 
    toBTransform( const URDFOrigin &origin ) const;
    
    BInertia
    toBInertia( const URDFInertial &inertial ) const;
    
    BBody
    toBBody(const URDFLink &link) const;
    
    BJoint
    toBJoint( const URDFJoint &joint ) const;
  
    std::string
    root( const URDFModel &urdfmodel ) const;
    
    BModel
    toBModel( const URDFModel &urdfmodel ) const;
    
    //
    //
    //
    
    URDFReader m_reader;
    URDFWriter m_writer;
    
};

#endif


