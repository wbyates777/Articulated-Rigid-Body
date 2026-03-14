/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BCollider.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Support functions for various types/shapes of colliders

 See for example https://github.com/jrl-umi3218/sch-core/tree/master/src/S_Object 
   
 
 Note we use double here (see libccd build options) and not spatial algebra types
 
*/


#ifndef __BCOLLIDER_H__
#define __BCOLLIDER_H__


#include <vector>
#include <glm/vec3.hpp>


#ifndef __BPOLYTOPE_H__
#include "BPolytope.h"
#endif


class ABody;


class BCollider
{

public:

    BCollider( void ) : m_body(nullptr), m_polytopes() {}
    BCollider( ABody *b ) : m_body(b), m_polytopes() {}
    virtual ~BCollider( void ) { m_body = nullptr; }

    void
    setBody( ABody *b ) { m_body = b; }
    
    
    const std::vector<BPolytope>&
    getPoints( void ) const { return m_polytopes; }
    
    std::vector<BPolytope>&
    getPoints( void ) { return m_polytopes; }
    
    void
    setPoints( const std::vector<BPolytope> &p ) { m_polytopes = p; }
    
    //
    // support functions
    //
    virtual glm::dvec3  
    first_point( void ) const = 0;
    
    virtual glm::dvec3
    max_point( const glm::dvec3 &dir ) const = 0;
    
protected:

    ABody *m_body;                      // take care when copying - ensure that m_body points to correct object
    std::vector<BPolytope> m_polytopes; // not always used by support functions

};



//
// mesh - max_point is a linear vertex search
// relatively slow but bullet proof. Ok for small meshes or testing
//
class BBasicCollider : public BCollider
{
    
public:
    
    BBasicCollider( void )=default;
    BBasicCollider( ABody *b ) : BCollider(b) {}
    ~BBasicCollider( void ) override=default;

    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
};


//
// adjacency mesh; fastest method - requires triangular mesh
//
class BBodyCollider : public BCollider
{
    
public:
    
    BBodyCollider( void )=default;
    BBodyCollider( ABody *b ) : BCollider(b) {}
    ~BBodyCollider( void ) override=default;
    
    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
    
};

//
// Box
//
class BBoxCollider : public BCollider
{
    
public:
    
    BBoxCollider( void )=default;
    BBoxCollider( ABody *b ) : BCollider(b) {}
    ~BBoxCollider( void ) override=default;
    
    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
    
};


//
// Sphere
//
class BSphereCollider : public BCollider
{
    
public:
    
    BSphereCollider( void )=default;
    BSphereCollider( ABody *b ) : BCollider(b) {}
    ~BSphereCollider( void ) override=default;
    
    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
    
};

#endif


