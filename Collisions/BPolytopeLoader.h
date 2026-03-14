/* BPolytopeLoader 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BPolytopeLoader.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Load polytope for a mesh using the ASSIMP mesh loader.
 Computes the vertex edge adjacency vector used to optimise collision detection.
 Requires a triangular mesh - ASSIMP does this triangulation so you don't have to.

 You will need to install the ASSIMP library.
 Problems can occur due to zlib - if you already have zlib installed on your system, 
 you must disable its inclusion in the build -- see ASSIMP  CMakeLists.txt
 
 https://github.com/assimp/assimp
 
 
 I use the Python executable CoACD for converting meshes to polytopes/convex hulls. 
 
 See https://github.com/SarahWeiii/CoACD
 
      pip install trimesh ; pip install coacd;
 
      mycoacd.py -i mymesh.obj -o mypolytope.obj
 
 
 Note 
 
 1) scale of mesh and scale of polytope must be identical.

 
*/


#ifndef __BPOLYTOPELOADER_H__
#define __BPOLYTOPELOADER_H__


#include <string>
#include <vector>


#ifndef __BPOLYTOPE_H__
#include "BPolytope.h"
#endif


class aiMaterial;
class aiNode;
class aiScene;
class aiMesh;


class BPolytopeLoader
{

public:

    BPolytopeLoader( void )=default;
    ~BPolytopeLoader( void )=default;

    static void 
    loadPolytope( std::vector<BPolytope> &poly, const std::string& path, float scale = 1.0 );
    
private:

    static void 
    processPoly( std::vector<BPolytope> &poly, aiNode *node, const aiScene *scene, float scale );

    static void 
    processSubPoly( BPolytope &poly, aiMesh *mesh, const aiScene *scene, float scale );

};

#endif


