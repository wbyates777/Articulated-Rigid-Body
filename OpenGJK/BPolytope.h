/* BPolytope 01/03/2026

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
 
 i) I use float ans short here (to save memory) - no spatial algebra/diffentiable types
 ii) I use the Python executable CoACD for converting meshes to sequences of polytopes/convex hulls. 
 
 See https://github.com/SarahWeiii/CoACD
 
      pip install trimesh ; pip install coacd;
 
      mycoacd.py -i mymesh.obj -o mypolytope.obj
 
*/


#ifndef __BPOLYTOPE_H__
#define __BPOLYTOPE_H__


#include <vector>
#include <glm/vec3.hpp> 

struct BPolytope 
{
    BPolytope( void ): m_coord(), m_adjacency(), m_lastVertex(0)  {}
    explicit BPolytope( const std::vector<glm::vec3> &points ): m_coord(points), m_adjacency(), m_lastVertex(0) {}
    ~BPolytope( void )=default;

    glm::vec3& 
    operator[]( int idx ) { return m_coord[idx]; }
    
    const glm::vec3& 
    operator[]( int idx ) const { return m_coord[idx]; }
    
    size_t 
    size( void ) const { return m_coord.size(); }
    
    // NB polytope verteces and indexes are float and short respectively
    std::vector<glm::vec3> m_coord;  
    std::vector<std::vector<short>> m_adjacency;  

    mutable int m_lastVertex;
};




#endif


