/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BCollider.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Support functions for various types/shapes of colliders

 Note all points in coordinates local to model.
 
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

#ifndef __BBOX_H__
#include "BBox.h"
#endif


class BCollider
{

public:

    BCollider( void ) : m_lastVert(0), m_lastPoly(0), m_polytope() {}
    
    BCollider( const std::vector<BPolytope>& p ) : m_lastVert(0), m_lastPoly(0), m_polytope(p) {}
    BCollider( const BPolytope& p ) : m_lastVert(0), m_lastPoly(0), m_polytope(std::vector<BPolytope>(1,p)) {}
    
    virtual ~BCollider( void )=default;
    
    //
    // support functions
    //
    
    virtual glm::dvec3  
    first_point( void ) const = 0;
    
    virtual glm::dvec3
    max_point( const glm::dvec3 &dir ) const = 0;
    
protected:

    mutable int m_lastVert;
    mutable int m_lastPoly;
    std::vector<BPolytope> m_polytope; 

};



//
// mesh - max_point is a linear vertex search
// relatively slow but bullet proof. Ok for small meshes or testing
//
class BBasicCollider : public BCollider
{
    
public:
    
    BBasicCollider( void )=default;
    BBasicCollider( const std::vector<BPolytope>& p ) : BCollider(p) {}
    BBasicCollider( const BPolytope& p ) : BCollider(p) {}
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
    BBodyCollider( const std::vector<BPolytope>& p ) : BCollider(p) {}
    BBodyCollider( const BPolytope& p ) : BCollider(p) {}
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
    BBoxCollider( const BBox& box ) : BCollider(), m_box(box)  {}
    ~BBoxCollider( void ) override=default;
    
    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
   
private:
    
    BBox m_box;
};


//
// Sphere
//
class BSphereCollider : public BCollider
{
    
public:
    
    BSphereCollider( void )=default;
    BSphereCollider( double r ) : BCollider() 
    {
        m_polytope.push_back( BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }
    ~BSphereCollider( void ) override=default;
    
    glm::dvec3  
    first_point( void ) const override;

    glm::dvec3
    max_point( const glm::dvec3 &dir ) const override;
    
};

#endif


