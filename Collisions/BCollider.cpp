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
 
 Note we use double/float and glm types here not spatial algebra types
 
*/


#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // for glm::distance2/glm::length2


constexpr  double B_SMALL_NUM = 1E-12;
constexpr  double B_MIN =  -std::numeric_limits<BScalar>::max();

//
// Linear 
//
glm::dvec3  
BBasicCollider::first_point( void ) const
{

    return (m_body->orient() * glm::dvec3(m_polytopes[0].m_coord[0])) + m_body->pos();
}

glm::dvec3 
BBasicCollider::max_point( const glm::dvec3 &dir ) const
// basic, robust, brute force linear search - tested 
{
    // degenerate direction
    if (glm::length2(dir) < B_SMALL_NUM)
    {
        // any/this value will do
        return m_polytopes[0].m_coord[0];
    }

    // convert from world to body/model coords
    const glm::dvec3 mydir = (glm::transpose(m_body->orient()) * dir);
    
    int vertId = -1;
    int polyId = -1;
    double maxDot = B_MIN; 
    for (int j = 0; j < m_polytopes.size(); ++j) 
    {
        const BPolytope &poly = m_polytopes[j];
        
        for (int i = 0; i < poly.m_coord.size(); ++i) 
        {
            double d = glm::dot(glm::dvec3(poly.m_coord[i]), mydir);
            
            if (d > maxDot) 
            {
                maxDot = d;
                vertId = i;
                polyId = j;
            }
        }
    }

    glm::dvec3 retVal = (m_body->orient() * glm::dvec3(m_polytopes[polyId].m_coord[vertId]));
    retVal += m_body->pos();
    
    return retVal;
}


//
// Adjacency
//
glm::dvec3  
BBodyCollider::first_point( void ) const
{
    return (m_body->orient() * glm::dvec3(m_polytopes[0].m_coord[0])) + m_body->pos();
}

glm::dvec3
BBodyCollider::max_point( const glm::dvec3 &dir ) const
{
    // degenerate direction
    if (glm::length2(dir) < B_SMALL_NUM)
    {
        // any/this value will do
        m_polytopes[0].m_lastVertex = 0;
        return m_polytopes[0].m_coord[0];
    }
    
    const glm::dvec3 localDir = (glm::transpose(m_body->orient()) * dir);
    
    double globalMaxDot = B_MIN;
    glm::dvec3 bestVertexLocal(B_ZERO_3);

    for (int j = 0; j < m_polytopes.size(); ++j) 
    {
        const BPolytope &poly = m_polytopes[j];
        
        // warm start
        int currentIdx = poly.m_lastVertex; 
        
        double currentDot = glm::dot(glm::dvec3(poly.m_coord[currentIdx]), localDir);
        
        bool improved = true;
        while (improved)
        {
            improved = false;
            for (int neighborIdx : poly.m_adjacency[currentIdx])
            {
                double nDot = glm::dot(glm::dvec3(poly.m_coord[neighborIdx]), localDir);
                if (nDot > currentDot)
                {
                    currentDot = nDot;
                    currentIdx = neighborIdx;
                    improved = true;
                    break; // keep walking
                }
            }
        }
        
        poly.m_lastVertex = currentIdx;

        // track the best 
        if (currentDot > globalMaxDot)
        {
            globalMaxDot = currentDot;
            bestVertexLocal = poly.m_coord[currentIdx];
        }
    }

    return (m_body->orient() * bestVertexLocal) + m_body->pos();
}


//
// Box
//
glm::dvec3  
BBoxCollider::first_point( void ) const
{
    return ((m_body->orient() * m_body->box().top()) + m_body->pos());
}

glm::dvec3 
BBoxCollider::max_point( const glm::dvec3 &dir ) const
{
    glm::dvec3 mydir = glm::transpose(m_body->orient()) * dir;
    const glm::dvec3 &extent = m_body->box().extent();
    
    glm::dvec3 v( glm::sign(mydir.x) * extent.x, 
                  glm::sign(mydir.y) * extent.y, 
                  glm::sign(mydir.z) * extent.z );
    
    return (m_body->orient() * v) + m_body->pos();
 
}


//
// Sphere
//
glm::dvec3  
BSphereCollider::first_point( void ) const
{
    return m_body->pos() + glm::dvec3(0.0, m_body->box().extent().y, 0.0);
}

glm::dvec3 
BSphereCollider::max_point( const glm::dvec3 &dir ) const
{
    return m_body->pos() + (glm::normalize(dir) * m_body->box().extent().y);
}

