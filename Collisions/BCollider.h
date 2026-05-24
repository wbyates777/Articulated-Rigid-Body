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

    enum ColliderType { Sphere, Box, Linear, Adjacency, MAXCOLLIDER };
    
    BCollider( void ) : m_type(Linear), m_lastVert(0), m_lastPoly(0), m_polytope(), m_box() {}
    BCollider( const std::vector<BPolytope>& p ) : m_type(Linear), m_lastVert(0), m_lastPoly(0), m_polytope(p), m_box() {}
    BCollider( const BPolytope& p ) : BCollider(std::vector<BPolytope>(1,p)) {}
    BCollider( const BBox& box ) : m_type(Box), m_lastVert(0), m_lastPoly(0), m_polytope(), m_box(box)  {}
    BCollider( double r ) : m_type(Sphere), m_lastVert(0), m_lastPoly(0), m_polytope(), m_box() 
    {
        m_polytope.push_back( BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }
    ~BCollider( void )=default;
    
    
    ColliderType 
    getType( void ) const { return m_type; }
    
    void 
    setType( ColliderType c ) { m_type = c; }
    
    void 
    setType( const BBox &box ) { m_type = BCollider::Box; m_box = box; }
    
    void 
    setType( double r ) 
    { 
        m_type = BCollider::Sphere; 
        m_polytope.push_back( BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }
    
    
    void
    setPoints(  const std::vector<BPolytope>& p, ColliderType type = Linear ) { m_polytope = p; m_type = type; }
    
    void
    setPoints( const BPolytope& p, ColliderType type = Linear ) { m_polytope = std::vector<BPolytope>(1,p); m_type = type; }
    
    //
    // support functions
    //
    
    virtual glm::dvec3  
    first_point( void ) const;
    
    
    virtual glm::dvec3
    max_point( const glm::dvec3 &dir ) const;
    

    
private:

    glm::dvec3 
    linear_max_point( const glm::dvec3 &dir ) const;
    
    glm::dvec3
    adjacency_max_point( const glm::dvec3 &dir ) const;
    
    ColliderType m_type;
    
    mutable int m_lastVert;
    mutable int m_lastPoly;
    std::vector<BPolytope> m_polytope; 
    BBox m_box;
    
};



#endif


