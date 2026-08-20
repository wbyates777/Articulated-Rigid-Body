/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BCollider.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 CCD Collider
 
 
 Support functions for various types/shapes of colliders

 Note all points in coordinates local to model.
 
 See for example https://github.com/jrl-umi3218/sch-core/tree/master/src/S_Object 
   
 
 Note we use double here (see libccd build options) and not spatial algebra types
 
*/


#ifndef __BCOLLIDER_H__
#define __BCOLLIDER_H__



#ifndef __BGLM_H__
#include "BGLM.h"
#endif

#ifndef __BPOLYTOPE_H__
#include "BPolytope.h"
#endif

#ifndef __BBOX_H__
#include "BBox.h"
#endif

#include <vector>
#include <glm/vec3.hpp>



class BCollider
{

public:

    enum ColliderType { Sphere, Box, Linear, Adjacency, MAXCOLLIDER };
    
    BCollider( void ) : m_lastVert(0), m_lastPoly(0), m_box(), m_polytope(), m_type(Linear) {}
    explicit BCollider( const std::vector<BPolytope>& p ) : m_lastVert(0), m_lastPoly(0), m_box(), m_polytope(p), m_type(Linear) {}
    explicit BCollider( const BPolytope& p ) : BCollider(std::vector<BPolytope>(1,p)) {}
    explicit BCollider( const BBox& box ) : m_lastVert(0), m_lastPoly(0), m_box(box), m_polytope(), m_type(Box)  {}
    explicit BCollider( double r ) : m_lastVert(0), m_lastPoly(0), m_box() , m_polytope(), m_type(Sphere)
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
        m_polytope = std::vector<BPolytope>(1,BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }

    void
    setPoints( const std::vector<BPolytope>& p, ColliderType type = Linear ) 
    { 
        m_type = type; m_polytope = p;
    }
    
    void
    setPoints( const BPolytope& p, ColliderType type = Linear ) 
    { 
        m_type = type; m_polytope = std::vector<BPolytope>(1,p);
    }
    
    //
    // support functions
    //
    
    glm::dvec3  
    first_point( void ) const;
    
    
    glm::dvec3
    max_point( const glm::dvec3 &dir ) const;
    
private:

    glm::dvec3 
    linear_max_point( const glm::dvec3 &dir ) const;
    
    glm::dvec3
    adjacency_max_point( const glm::dvec3 &dir ) const;
    
  
    mutable int m_lastVert;
    mutable int m_lastPoly;
    BBox m_box;
    std::vector<BPolytope> m_polytope; 
    ColliderType m_type;

};


#endif

//
