/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   BCollider.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 CCD Collider
 
 Note all points in coordinates local to model.
 
 Note we use double/float and glm types here not spatial algebra types
 

 
*/


#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // for glm::distance2/glm::length2


constexpr  double B_SMALL_NUM = 1E-12;
constexpr  double B_MIN_NUM =  -std::numeric_limits<double>::max();



//
// Box
//
glm::dvec3  
BCollider::first_point( void ) const
{
    if (m_type == BCollider::Box)
    {
        m_box.top();
    }
    
    return glm::dvec3(m_polytope[0].m_coord[0]);
}

glm::dvec3
BCollider::max_point( const glm::dvec3 &dir ) const
{
    if (m_type == BCollider::Sphere)
    {
        return  (glm::normalize(dir) * double(m_polytope[0].m_coord[0][1])); //.y
    }
    
    if (m_type == BCollider::Box)
    {
        const glm::dvec3 &ext = m_box.extent();
        
        return glm::dvec3( (dir.x >= 0) ? ext.x : -ext.x,
                           (dir.y >= 0) ? ext.y : -ext.y,
                           (dir.z >= 0) ? ext.z : -ext.z );
    }
    
    if (m_type == BCollider::Linear)
    {
        return linear_max_point( dir );
    }
    
    if (m_type == BCollider::Adjacency)
    {
        return adjacency_max_point( dir );
    }
    
    return glm::dvec3(0.0);
}



glm::dvec3 
BCollider::linear_max_point( const glm::dvec3 &dir ) const
// basic, robust, brute force linear search - tested 
{
    assert(!m_polytope.empty());
    
    // degenerate direction
    if (glm::length2(dir) < B_SMALL_NUM)
    {
        // any/this value will do
        m_lastVert = 0;
        m_lastPoly = 0;
        
        return m_polytope[0].m_coord[0];
    }

    m_lastVert = -1;
    m_lastPoly = -1;
    
    double maxDot = B_MIN_NUM; 
    for (int j = 0; j < m_polytope.size(); ++j) 
    {
        const BPolytope &poly = m_polytope[j];
        
        for (int i = 0; i < poly.m_coord.size(); ++i) 
        {
            double d = glm::dot(glm::dvec3(poly.m_coord[i]), dir);
            
            if (d > maxDot) 
            {
                maxDot = d;
                m_lastVert = i;
                m_lastPoly = j;
                
                poly.m_lastVertex = i;
            }
        }
    }
    
    assert(m_lastPoly != -1 && m_lastVert != -1);
    return m_polytope[m_lastPoly].m_coord[m_lastVert];
}


glm::dvec3
BCollider::adjacency_max_point( const glm::dvec3 &dir ) const
{
    // degenerate direction
    if (glm::length2(dir) < B_SMALL_NUM)
    {
        // any/this value will do
        m_lastVert = 0;
        m_lastPoly = 0;
        return m_polytope[0].m_coord[0];
    }

    double globalMaxDot = B_MIN_NUM;
    glm::dvec3 bestVertexLocal(0.0);

    for (int j = 0; j < m_polytope.size(); ++j) 
    {
        const BPolytope &poly = m_polytope[j];
        
        // warm start
        int currentIdx = poly.m_lastVertex; 
        
        double currentDot = glm::dot(glm::dvec3(poly.m_coord[currentIdx]), dir);
        
        bool improved = true;
        while (improved)
        {
            improved = false;
            for (int neighbourIdx : poly.m_adjacency[currentIdx])
            {
                double nDot = glm::dot(glm::dvec3(poly.m_coord[neighbourIdx]), dir);
                if (nDot > currentDot)
                {
                    currentDot = nDot;
                    currentIdx = neighbourIdx;
                    improved = true;
                    break; // keep walking
                }
            }
        }
        
        // track the best 
        if (currentDot > globalMaxDot)
        {
            globalMaxDot = currentDot;
            bestVertexLocal = poly.m_coord[currentIdx];
            
            m_lastVert = currentIdx;
            m_lastPoly = j;
            m_polytope[j].m_lastVertex = currentIdx;
        }
    }

    return bestVertexLocal;
}

//
