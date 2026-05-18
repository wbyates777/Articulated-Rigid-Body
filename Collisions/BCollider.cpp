/* BCollider 01/03/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   BCollider.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Note all points in coordinates local to model.
 
 Note we use double/float and glm types here not spatial algebra types
 

 
*/


#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // for glm::distance2/glm::length2


constexpr  double B_SMALL_NUM = 1E-12;
constexpr  double B_MIN =  -std::numeric_limits<double>::max();

//
// Linear 
//
glm::dvec3  
BBasicCollider::first_point( void ) const
{

    return glm::dvec3(m_polytope[0].m_coord[0]);
}


glm::dvec3 
BBasicCollider::max_point( const glm::dvec3 &dir ) const
// basic, robust, brute force linear search - tested 
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


//
// Adjacency
//
glm::dvec3  
BBodyCollider::first_point( void ) const
{
    return m_polytope[0].m_coord[0];
}

glm::dvec3
BBodyCollider::max_point( const glm::dvec3 &dir ) const
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


//
// Box
//
glm::dvec3  
BBoxCollider::first_point( void ) const
{
    return m_box.top();
}

glm::dvec3 
BBoxCollider::max_point( const glm::dvec3 &dir ) const
{
    const glm::dvec3 &extent = m_box.extent();
    
    glm::dvec3 v( (dir.x >= 0) ? extent.x : -extent.x,
                  (dir.y >= 0) ? extent.y : -extent.y,
                  (dir.z >= 0) ? extent.z : -extent.z );

    
    return v;
}


//
// Sphere
//
glm::dvec3  
BSphereCollider::first_point( void ) const
{
    return glm::dvec3(m_polytope[0][0]);
}

glm::dvec3 
BSphereCollider::max_point( const glm::dvec3 &dir ) const
{
    return  (glm::normalize(dir) * double(m_polytope[0][0][1])); //.y
}

