/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BCollider.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 OpenGJK Collider
 
 
 This class lives in a game object or CBody class - it is the collidable shape for the object.
 
 Technically, these are the 'support functions' for openEPA. 
 
 Only handles (vector of) polytope colliders. Does not support some standard shapes i.e sphere
 Boxes can be represent as polytope with 8-verteces.

 
*/


#ifndef __BCOLLIDER_H__
#define __BCOLLIDER_H__

#ifndef __BGLM_H__
#include "BGLM.h"
#endif

#ifndef __BPOLYTOPE_H__
#include "BPolytope.h"
#endif

#include <vector>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp> 



class BCollider 
{

public:

    enum ColliderType { Linear, Adjacency, MAXCOLLIDER };
    
    BCollider( void ): m_type(Adjacency), m_lastVert(0), m_lastPoly(0), m_polytope() {}
    explicit BCollider( const std::vector<BPolytope>& p ): m_type(Adjacency), m_lastVert(0), m_lastPoly(0), m_polytope(p) {}
    explicit BCollider( const BPolytope& poly): BCollider( std::vector<BPolytope>(1, poly) ) {}

    ~BCollider( void )=default;
    
    ColliderType 
    getType( void ) const { return m_type; }
    
    void 
    setType( ColliderType c ) { m_type = c; }
    
    void
    setPoints( const BPolytope &p, ColliderType c  ) 
    { 
        m_type = c; 
        m_polytope.push_back(p); 
    }
    
    void
    setPoints( const std::vector<BPolytope> &p, ColliderType c ) 
    { 
        m_type = c; 
        m_polytope = p; 
    }
    
    const std::vector<BPolytope>&
    getPoints( void ) const { return m_polytope; }
    
    std::vector<BPolytope>&
    getPoints( void ) { return m_polytope; }
    
    BPolytope&
    getPoints( int i ) { return m_polytope[i]; }
    
    
    const BPolytope&
    getPoints( int i ) const { return m_polytope[i]; }
    
    int
    size( void ) const { return (int) m_polytope.size(); }
    
    

    //
    // support functions
    //
    
    /*! The point in the polytope at index idx. */
    glm::dvec3
    point( const glm::ivec2& idx ) const { return m_polytope[idx[1]].m_coord[idx[0]]; }

    /*! The first point returned by the support function. */
    glm::dvec3  
    first_point( void ) const { return m_polytope[m_lastPoly].m_coord[m_lastVert]; }

    /*! The furthest point returned by the support function. */
    glm::dvec3 
    max_point( const glm::dvec3 &dir ) const
    {
        return (m_type == BCollider::Adjacency) ? adjacency_max_point(dir) : linear_max_point(dir); 
    }

    /*! Index of the furthest point returned by the support function. */
    glm::ivec2
    max_index( void ) const { return glm::ivec2(m_lastVert, m_lastPoly); } 
    
private:

    glm::dvec3 
    linear_max_point( const glm::dvec3 &dir ) const;
    
    glm::dvec3
    adjacency_max_point( const glm::dvec3 &dir ) const;
    
    ColliderType m_type;
    
    mutable int m_lastVert;
    mutable int m_lastPoly;
    
    std::vector<BPolytope> m_polytope; // not always used by support functions
};

//
// 
//

#endif


