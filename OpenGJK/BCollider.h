/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BCollider.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 OpenGJK Collider
 
 
 This class lives in a game object or ABody class - it is the collidable shape for the object.
 
 Technically, these are the 'support functions' for openEPA. 
 
 Only handles (vector of) polytope colliders. Does not support some standard shapes i.e sphere
 Boxes can be represent as polytope with 8-verteces.

 
*/


#ifndef BCOLLIDER_H
#define BCOLLIDER_H


#ifndef BGLM_H
#include "BGLM.h"
#endif


#ifndef BPOLYTOPE_H
#include "BPolytope.h"
#endif


#include <vector>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp> 

class BCollider 
{

public:

    enum CType { Linear, Adjacency, MAXCOLLIDER };
    
    BCollider( void ): m_lastVert(0), m_lastPoly(0), m_polyIdx(0), m_polytope(), m_type(Adjacency)  {}
    BCollider( const std::vector<BPolytope>& p ): m_lastVert(0), m_lastPoly(0), m_polyIdx(0), m_polytope(p), m_type(Adjacency) {}
    BCollider( const BPolytope& poly ): BCollider( std::vector<BPolytope>(1, poly) ) {}

    ~BCollider( void )=default;
    
    CType 
    getType( void ) const { return m_type; }
    
    void 
    setType( CType c ) { m_type = c; }
    
    /*! The polytope index polyIdx in (vertIdx, polyIdx). */
    void 
    setPolyIdx( int p ) { m_polyIdx = p; } 
    
    int 
    polyIdx( void ) const { return m_polyIdx; }
    
    
    void
    set( const BPolytope &p, CType type = Linear ) { m_polytope = std::vector<BPolytope>(1, p); m_type = type; }
    
    void
    set( const std::vector<BPolytope> &p, CType type = Linear ) { m_polytope = p; m_type = type; }
    
    
    const std::vector<BPolytope>&
    polytopes( void ) const { return m_polytope; }
    
    std::vector<BPolytope>&
    polytopes( void ) { return m_polytope; }
 
    BPolytope&
    polytope( int i ) { return m_polytope[i]; }
     
    const BPolytope&
    polytope( int i ) const { return m_polytope[i]; }
    
    BPolytope&
    polytope( void ) { return m_polytope[m_polyIdx]; }

    const BPolytope&
    polytope( void ) const { return m_polytope[m_polyIdx]; }
    

    int
    size( void ) const { return (int) m_polytope.size(); }
    
    
    ///
    /// Support functions called by openGJK
    ///
    
    /*! The point in the polytope at index idx (vertIdx, polyIdx). */
    glm::dvec3
    point( const glm::ivec2& idx ) const { return m_polytope[idx[1]].m_coord[idx[0]]; }

    /*! The first point returned by the support function for polytope polyIdx. */
    glm::dvec3  
    first_point( void ) const { return m_polytope[m_polyIdx].m_coord[0]; }

    /*! The furthest point returned by the support function for polytope polyIdx. */
    glm::dvec3 
    max_point( const glm::dvec3 &dir) const
    {
        return (m_type == BCollider::Adjacency) ? adjacency_max_point(dir, m_polyIdx) : linear_max_point(dir, m_polyIdx); 
    }

    /*! Index of the furthest point returned by the support function for polytope polyIdx. */
    glm::ivec2
    max_index( void ) const { return glm::ivec2(m_polytope[m_polyIdx].m_lastVertex, m_polyIdx); } 
    
private:

    glm::dvec3 
    linear_max_point( const glm::dvec3 &dir, int polyIdx ) const;
    
    glm::dvec3
    adjacency_max_point( const glm::dvec3 &dir, int polyIdx ) const;
    

    mutable int m_lastVert;
    mutable int m_lastPoly;
    int m_polyIdx;
    
    std::vector<BPolytope> m_polytope; // not always used by support functions
    CType m_type;
};

//
// 
//

#endif


