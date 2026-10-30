/*! 
 @file BPolytope.h 
 @date 01/03/2026

 $$$$$$$$$$$$$$$$$$$
 $   BPolytope.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


 A polytope is a geometric object with flat sides (faces). 
 
 Used for collision detection.
 
 Polytopes are the generalization of three-dimensional polyhedra to any number of dimensions. 
 (see https://en.wikipedia.org/wiki/Polytope). The GJK algorithm works directly
 on their convex-hull. However the convex-hull is never computed explicitly,
 instead each GJK-iteration employs a support function that has a cost
 linearly dependent on the number of points defining the polytope.
 
 
 Note:
 
 1) I use float and short here (to save memory) - no spatial algebra/diffentiable types
 2) I use the Python executable CoACD for converting meshes to sequences of polytopes/convex hulls. 
 
 See https://github.com/SarahWeiii/CoACD
 
      pip install trimesh ; pip install coacd;
 
      mycoacd.py -i mymesh.obj -o mypolytope.obj
 
 i.e. mycoacd.py -mi 1000 -md 5 -mn 30  -i ncc1701A.obj -o ncc1701A_convexhull.obj
 
*/


#ifndef BPOLYTOPE_H
#define BPOLYTOPE_H


#ifndef BGLM_H
#include "BGLM.h"
#endif


#include <vector>
#include <glm/common.hpp>
#include <glm/vec3.hpp> 

struct BPolytope 
{
    BPolytope( void ) : m_min(-std::numeric_limits<float>::max()), 
                        m_max(std::numeric_limits<float>::max()), 
                        m_coord(), 
                        m_adjacency(), 
                        m_lastVertex(0)   {}
    
    explicit BPolytope( const std::vector<glm::vec3> &points ): m_min(-std::numeric_limits<float>::max()), 
                                                                m_max(std::numeric_limits<float>::max()), 
                                                                m_coord(points), 
                                                                m_adjacency(), 
                                                                m_lastVertex(0)
    {
        for (const glm::vec3 &p : m_coord)
        {
            m_min = glm::min(p, m_min);
            m_max = glm::max(p, m_max);
        }
    }
    ~BPolytope( void )=default;

    glm::vec3 m_min;
    glm::vec3 m_max;
    
    // NB polytope verteces and indexes are float and short respectively
    std::vector<glm::vec3> m_coord;  
    std::vector<std::vector<short>> m_adjacency;  

    mutable int m_lastVertex;
};


#endif

//
