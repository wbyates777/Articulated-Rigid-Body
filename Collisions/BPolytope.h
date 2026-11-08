/*! 

         $$$$$$$$$$$$$$$$$$$
         $   BPolytope.h   $
         $$$$$$$$$$$$$$$$$$$

 @file BPolytope.h 
 @ingroup Collisions
 date 01/03/2026
 
 by W.B. Yates
 
 ---

 @brief A polytope is a geometric object with flat sides (faces) which is used for collision detection.
 
 Polytopes are the generalization of three-dimensional polyhedra to any number of dimensions. 
 The GJK algorithm works directly on their convex-hull. However the convex-hull is never 
 computed explicitly, instead each GJK-iteration employs a support function that has a cost
 linearly dependent on the number of points defining the polytope.
 
 Note:
 - use float and short here to save memory - no spatial algebra or diffentiable types.
 - use the Python executable CoACD for converting meshes to sequences of polytopes/convex hulls. 
 
 @code{.py}
      pip install trimesh ; pip install coacd;
      mycoacd.py -mi 1000 -md 5 -mn 30  -i ncc1701A.obj -o ncc1701A_convexhull.obj
 @endcode

 @see https://en.wikipedia.org/wiki/Polytope
 @see https://github.com/SarahWeiii/CoACD
 
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
    BPolytope( void ) : m_min(std::numeric_limits<float>::max()),
                        m_max(std::numeric_limits<float>::lowest()),
                        m_coord(), 
                        m_adjacency(), 
                        m_lastVertex(0)   {}
    
    explicit BPolytope( const std::vector<glm::vec3> &points ): m_min(std::numeric_limits<float>::max()), 
                                                                m_max(std::numeric_limits<float>::lowest()),
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
    
    // polytope verteces and indexes are float and short respectively
    std::vector<glm::vec3> m_coord;  
    std::vector<std::vector<short>> m_adjacency;  

    mutable int m_lastVertex;
};


#endif

//
