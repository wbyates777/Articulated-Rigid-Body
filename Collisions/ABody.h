/* ABody 25/11/2025

 $$$$$$$$$$$$$$$
 $   ABody.h   $
 $$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 The abstract-base interface to the BCollisionManager
 
*/


#ifndef __ABODY_H__
#define __ABODY_H__


#include <string>
#include <vector>

#ifndef __BMATRIX6_H__
#include "BMatrix6.h"
#endif

#ifndef __BBOX_H__
#include "BBox.h"
#endif

#ifndef __BRBINERTIA_H__
#include "BRBInertia.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif


class ABody 
{

public:

    ABody( void )=default;   
    virtual ~ABody( void )=default;
    
    // must be unique
    virtual BBodyId
    objId( void ) const = 0;
    
    
    //
    // object position  and orientation i.e. X_base.r() and X_base.E()
    //
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
    // spatial velocity interface 
    //
    virtual const BVector6&
    v( void ) const = 0;
    
    virtual BVector6&
    v( void ) = 0; 
    
    virtual void
    v( const BVector6 &v ) = 0; 
    
    //
    // spatial inertia
    //
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



