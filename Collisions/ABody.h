/* ABody 25/11/2025

 $$$$$$$$$$$$$$$
 $   ABody.h   $
 $$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 The abstract-base interface used by the BCollisionManager
 
*/


#ifndef __ABODY_H__
#define __ABODY_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif


class BVector6;
class BMatrix6;
class BRBInertia;
class BTransform;
class BBox;
class BCollider;

class ABody 
{

public:

    ABody( void )=default;   
    virtual ~ABody( void )=default;
    
    
    //
    // object identity, position and orientation
    //
    virtual BBodyId
    objId( void ) const = 0;     // must be unique
    
    virtual const BVector3&
    pos( void ) const = 0;
    
    virtual  BVector3&
    pos( void )  = 0;
    
    virtual void
    pos( const BVector3 &p ) = 0;
    
    virtual const BMatrix3&
    orient( void ) const = 0;
    
    virtual BMatrix3&
    orient( void ) = 0;
    
    virtual void
    orient( const BMatrix3 &q ) = 0;
    
    //
    // spatial interface 
    //
    virtual const BTransform& 
    X_base( void ) const = 0;

    virtual BTransform& 
    X_base( void ) = 0;

    virtual void
    X_base( const BTransform &X ) = 0;
    

    virtual const BVector6&
    v( void ) const = 0;
    
    virtual BVector6&
    v( void ) = 0; 
    
    virtual void
    v( const BVector6 &v ) = 0; 
    

    virtual const BRBInertia& 
    I( void ) const = 0;
    
    virtual const BMatrix6& 
    invI( void ) const = 0;
    
    virtual const BRBInertia&
    I_base( void) const = 0;
    
    virtual const BMatrix6&
    invI_base( void) const = 0;
    
    

    //
    // bounding box in model coords
    //
    virtual const BBox& 
    box( void ) const = 0;
     
    virtual BBox& 
    box( void ) = 0;
    
    virtual void
    setBox( const BBox &b ) = 0;
    
    //
    // collsions
    //
    virtual BCollider& 
    collider( void ) = 0;
    
    virtual const BCollider& 
    collider( void ) const = 0;
        
protected:


};


#endif



