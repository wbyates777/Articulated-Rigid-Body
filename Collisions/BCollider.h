/*! 
 @file BCollider.h
 @date 01/03/2026

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


#ifndef BCOLLIDER_H
#define BCOLLIDER_H


#ifndef BPOLYTOPE_H
#include "BPolytope.h"
#endif

#ifndef BBOX_H
#include "BBox.h"
#endif

#include <vector>
#include <glm/vec3.hpp>

class BCollider
{

public:

    enum CType { Sphere, Box, Linear, Adjacency, MAXCOLLIDER };
    
    BCollider( void ) : m_type(Linear), m_lastVert(0), m_lastPoly(0), m_polyIdx(0), m_polytope(), m_box() {}
    BCollider( const std::vector<BPolytope>& p ) : m_type(Linear), m_lastVert(0), m_polyIdx(0), m_lastPoly(0), m_polytope(p), m_box() {}
    BCollider( const BPolytope& p ) : BCollider(std::vector<BPolytope>(1,p)) {}
    BCollider( const BBox& box ) : m_type(Box), m_lastVert(0), m_lastPoly(0), m_polyIdx(0), m_polytope(), m_box(box)  {}
    BCollider( double r ) : m_type(Sphere), m_lastVert(0), m_lastPoly(0), m_polyIdx(0), m_polytope(), m_box() 
    {
        m_polytope.push_back( BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }
    ~BCollider( void )=default;
    
    
    CType 
    getType( void ) const { return m_type; }
    
    void 
    setType( CType c ) { m_type = c; }
    
    void 
    setType( const BBox &box ) { m_type = BCollider::Box; m_box = box; }
    
    void 
    setType( double r ) 
    { 
        m_type = BCollider::Sphere; 
        m_polytope.push_back( BPolytope( std::vector<glm::vec3>(1, glm::vec3(0.0, r, 0.0)) ) );
    }
    
    void 
    setPolyIdx( int p ) { m_polyIdx = p; } // set polytope index
    
    //int 
    //polyIdx( void ) const { return m_polyIdx; }
    
    
    void
    set( const std::vector<BPolytope>& p, CType type = Linear ) { m_polytope = p; m_type = type; }
    
    void
    set( const BPolytope& p, CType type = Linear ) { m_polytope = std::vector<BPolytope>(1,p); m_type = type; }
    
    
    std::vector<BPolytope>&
    polytopes( void ) { return m_polytope; }
    
    const std::vector<BPolytope>&
    polytopes( void ) const { return m_polytope; }
    
    BPolytope&
    polytope( int i ) { return m_polytope[i]; }
    
    const BPolytope&
    polytope( int i ) const { return m_polytope[i]; }
    
    //BPolytope&
    //polytope( void ) { return m_polytope[m_polyIdx]; }

    //const BPolytope&
    //polytope( void ) const { return m_polytope[m_polyIdx]; }
    
    int
    size( void ) const { return (int) m_polytope.size(); }

    //
    // Support functions called by ccd
    //

    glm::dvec3  
    first_point( void ) const;
    
    glm::dvec3
    max_point( const glm::dvec3 &dir ) const;

private:

    glm::dvec3 
    linear_max_point( const glm::dvec3 &dir, int polyIdx ) const;
    
    glm::dvec3
    adjacency_max_point( const glm::dvec3 &dir, int polyIdx ) const;
    
    CType m_type;
    
    mutable int m_lastVert;
    mutable int m_lastPoly;
    int m_polyIdx;
    std::vector<BPolytope> m_polytope; 
    BBox m_box;
    
};



#endif


