/* BPolytope 17/12/2025

 $$$$$$$$$$$$$$$$$$$
 $   BPolytope.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A polytope is a geometric object with flat sides (faces). 
 Polytopes are the generalization of three-dimensional polyhedra to any number of dimensions. 
 
 https://en.wikipedia.org/wiki/Polytope
 
 Used for collision detection.
 
 I use the Python executable CoACD for converting meshes to polytopes/convex hulls. 
 
 See https://github.com/SarahWeiii/CoACD
 
      pip install trimesh ; pip install coacd;
 
      mycoacd.py -i mymesh.obj -o mypolytope.obj
 
 
*/


#ifndef __BPOLYTOPE_H__
#define __BPOLYTOPE_H__

#include <vector>
#include <glm/vec3.hpp>

//
// actually a sub-polytope - a mesh polytope is a vector of these sub-polytopes
//
struct BPolytope 
{
    BPolytope( void ): m_coord(), m_adjacency(), m_lastVertex(0)  {}
    explicit BPolytope( const std::vector<glm::vec3> &points ): m_coord(points), m_adjacency(), m_lastVertex(0)  {}
    ~BPolytope( void )=default;

    // NB polytope coords and indexes are float and short respectively
    std::vector<glm::vec3> m_coord;  
    std::vector<std::vector<short>> m_adjacency;  

    mutable short m_lastVertex;
};


#endif


