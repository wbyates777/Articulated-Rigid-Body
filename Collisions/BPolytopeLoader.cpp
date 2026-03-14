/* BPolytopeLoader 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BPolytopeLoader.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


 Load polytope for a mesh using the ASSIMP mesh loader.
 Computes the vertex edge adjancency vector used to optimise collision detection.
 Requires a triangular mesh - ASSIMP does this triangulation so you don't have to.

 Note 
 
 1) scale of mesh and scale of polytope must be identical.
 
*/


#ifndef __BPOLYTOPELOADER_H__
#include "BPolytopeLoader.h"
#endif

#include <iostream>
#include <unordered_set>

#include <assimp/scene.h> 
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>




// loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
void
BPolytopeLoader::loadPolytope(std::vector<BPolytope>  &poly, const std::string &path, float scale)
{
    // read file via ASSIMP 
    Assimp::Importer importer; 
    importer.SetExtraVerbose(true);
    
    // don't load these elements
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, 
                             aiComponent_NORMALS        |
                             aiComponent_MATERIALS      |
                             aiComponent_TEXTURES       |
                             aiComponent_LIGHTS         |
                             aiComponent_CAMERAS        |
                             aiComponent_ANIMATIONS     |
                             aiComponent_BONEWEIGHTS    |
                             aiComponent_COLORS );
    

    // you should let ASSIMP do the triangulaion - use aiProcess_Triangulate here
    unsigned int loadFlags =   aiProcess_Triangulate           | 
                               aiProcess_JoinIdenticalVertices | 
                               aiProcess_RemoveComponent       |
                               aiProcess_OptimizeMeshes        | 
                               aiProcess_ImproveCacheLocality  | 
                               aiProcess_FindDegenerates       |   
                               aiProcess_SortByPType;

    
    const aiScene* scene = nullptr;

    // we usually use a .obj file; ASSIMP supports many file extensions
    scene = importer.ReadFile(path, loadFlags);  

    // check for errors
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
        std::cout <<"ERROR::ASSIMP:: " << importer.GetErrorString() << " [" << path << "]" << std::endl;
        return;
    }
    
    // process ASSIMP's root node recursively
    processPoly(poly, scene->mRootNode, scene, scale);
  
    std::cout << "Loaded polytope [" << path << "] with " << poly.size() << " meshes" << std::endl;
}

void 
BPolytopeLoader::processPoly( std::vector<BPolytope> &poly, aiNode *node, const aiScene *scene, float scale )
{
    // process each mesh located at the current node
    for (int i = 0; i < node->mNumMeshes; ++i)
    {
        // the node object only contains indices to index the actual objects in the scene. 
        // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        
        poly.push_back(BPolytope());
        processSubPoly(poly.back(), mesh, scene, scale);
    }
    
    // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for (int i = 0; i < node->mNumChildren; ++i)
    {
        processPoly( poly, node->mChildren[i], scene, scale );
    }
}

void 
BPolytopeLoader::processSubPoly( BPolytope &poly, aiMesh *mesh, const aiScene *scene, float scale )
{
    // add this subpoly to polytope

    //
    // process vetexes
    //
    int N = mesh->mNumVertices;
    
    poly.m_coord.reserve( N );
   
    glm::vec3 vertex;
    for (int i = 0; i < N; ++i)
    {
        // pos
        vertex.x = mesh->mVertices[i].x * scale;
        vertex.y = mesh->mVertices[i].y * scale;
        vertex.z = mesh->mVertices[i].z * scale;
 
        poly.m_coord.push_back( vertex );
    }
    
    //
    // now walk through each of the mesh's faces and retrieve the vertex indices
    //
    std::vector<std::unordered_set<int>> adjSet( N );
 
    for (int i = 0; i < mesh->mNumFaces; ++i)
    {
        const aiFace &face = mesh->mFaces[i];
 
        assert(face.mNumIndices == 3);
        
        int a = face.mIndices[0];
        int b = face.mIndices[1];
        int c = face.mIndices[2];
   
         // edge adjacency; we assume triangulated mesh
        adjSet[a].insert(b);
        adjSet[b].insert(a);
         
        adjSet[a].insert(c);
        adjSet[c].insert(a);
         
        adjSet[b].insert(c);
        adjSet[c].insert(b);
    }
    
    poly.m_adjacency.resize( N ); 
    for (int i = 0; i < N; ++i)
    {
        poly.m_adjacency[i].assign(adjSet[i].begin(), adjSet[i].end());
    }
}


