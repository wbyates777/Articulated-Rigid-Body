/*! 

         $$$$$$$$$$$$$$$$$$$$$$$$$
         $   BPolytopeLoader.h   $
         $$$$$$$$$$$$$$$$$$$$$$$$$

 @file BPolytopeLoader.h 
 @ingroup Collisions
 date 17/12/2025
 
 by W.B. Yates
 
 ---
 
 @brief Load a polytope for a mesh using the ASSIMP mesh loader.
 
 Computes the vertex edge adjacency vector used to optimise collision detection.
 Requires a triangular mesh - ASSIMP does this triangulation so you don't have to.

 You will need to install the ASSIMP library.
 Problems can occur due to zlib - if you already have zlib installed on your system, 
 you must disable its inclusion in the build -- see ASSIMP  CMakeLists.txt
 
 @note scale of mesh and scale of polytope must be identical.
 
 @see https://github.com/assimp/assimp
 
*/


#ifndef BPOLYTOPELOADER_H
#define BPOLYTOPELOADER_H


#include <string>
#include <vector>


#ifndef BPOLYTOPE_H
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


