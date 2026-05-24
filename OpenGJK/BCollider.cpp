/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   BCollider.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


 A body orientation and position is given (in world/base coordinates) by
 the rotation BMatrix3 m_body->orient() and the BVector3 m_body->pos()
 respectively.
 
 Note we use  spatial algebra types
 
*/


#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif


constexpr  double B_SMALL_NUM = 1E-12;
constexpr  double B_MIN = -std::numeric_limits<double>::max();

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // For glm::distance2/glm::length2

glm::dvec3
BCollider::linear_max_point( const glm::dvec3 &dir ) const
// basic, robust, brute force linear search 
{
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
    
    double maxDot = B_MIN; 
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

    double globalMaxDot = B_MIN;
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


