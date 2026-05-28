/* BOpenEPA 17/03/2026

 $$$$$$$$$$$$$$$$$$$$
 $   BOpenEPA.cpp   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


*/

/*
 *                          _____      _ _  __
 *                         / ____|    | | |/ /
 *   ___  _ __   ___ _ __ | |  __     | | ' /
 *  / _ \| '_ \ / _ \ '_ \| | |_ |_   | |  <
 * | (_) | |_) |  __/ | | | |__| | |__| | . \
 *  \___/| .__/ \___|_| |_|\_____|\____/|_|\_\
 *       | |
 *       |_|
 *
 * Copyright 2022-2026 Mattia Montanari, University of Oxford
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. See https://www.gnu.org/licenses/
 */

#ifndef __BOPENEPA_H__
#include "BOpenEPA.h"
#endif

#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif

#ifndef __BFUNCTIONS_H__
#include "BFunctions.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif



// Each face is a triangle with 3 vertex indices
struct BEPAFace 
{
    BEPAVertex &operator[]( int idx ) { return vertex[idx]; }
    const BEPAVertex &operator[]( int idx ) const { return vertex[idx]; }
    
    BVector3 normal;                  // Face normal (pointing outward from origin)
    BScalar distance = 0.0;           // Distance from origin to face plane  
    std::array<BEPAVertex, 3> vertex; // Face has three vertexes
    bool valid = false;               // Whether this face is still valid (not removed)
};

// Polytope structure for EPA
struct BEPAPolytope
{
    BGJKVertex &operator[]( int idx ) { return points[idx]; }
    const BGJKVertex &operator[]( int idx ) const { return points[idx]; }

    int num_vertices = 0;
    int max_face_index = 0;  // Highest face index in use (for iteration bounds)
    std::array<BEPAFace, B_MAX_EPA_FACES> faces;  
    std::array<BGJKVertex, B_MAX_EPA_FACES + 4> points;  // Vertex coordinates in the Minkowski difference
};

// 
//
//

void 
BOpenEPA::set_contact_normal(const BVector3& w1, const BVector3& w2, BVector3& contact_normal) const
{
    BVector3 d = w2 - w1;
    BScalar n = arb::length(d);
    contact_normal = (n > B_EPS) ? d / n : B_XAXIS;
}


// Compute face normal and distance of face from origin.
// Winding is already fixed at face creation time, so the cross product
// direction is trusted directly — no centroid-based orientation check needed.
void 
BOpenEPA::compute_face_normal_distance(BEPAPolytope& poly) const
{
    using std::sqrt;
    
    for (int i = 0; i < poly.max_face_index; ++i) 
    {
        if (poly.faces[i].valid) 
        {
            BEPAFace& face = poly.faces[i];
            
            const BVector3& v0 = poly[face[0]];
            const BVector3& v1 = poly[face[1]];
            const BVector3& v2 = poly[face[2]];
            
            const BVector3 e0 = v1 - v0;
            const BVector3 e1 = v2 - v0;
            
            face.normal =  arb::cross(e0, e1);
            const BScalar norm_sq = arb::length2(face.normal);
            
            if (norm_sq > B_EPS2) 
            {
                BScalar norm = sqrt(norm_sq); 
                assert(!arb::isnan(norm)); // also checks derivative for NaN 
                
                face.normal /= norm;
                face.distance = arb::dot(face.normal, v0);
                
                // Safety: origin should be inside polytope so distance must be positive.
                // If negative, the winding was wrong — flip to recover.
                if (face.distance < 0) 
                {
                    face.normal = -face.normal;
                    face.distance = -face.distance;
                }
            }
            else 
            {
                face.valid = false;
                face.distance = B_SCALAR_MAX;
            }
            
        
        }
    }
}

// Check if a face is visible from a point (point is on positive side of face) Needed to determine which faces to restructure when vertex is added
bool 
BOpenEPA::face_visible( const BEPAPolytope& poly, int face_idx, const BVector3& point ) const
{
    const BEPAFace& face = poly.faces[face_idx];
    
    if (!face.valid) 
        return false;
    
    const BVector3 diff = point - poly[face[0]].vertex;
    return (arb::dot(face.normal, diff) > B_EPS);
}


void
BOpenEPA::init_epa_polytope( BEPAPolytope &poly,  const BSimplex& smp, BVector3& centroid ) const
{
    poly.num_vertices = 4;
    poly.max_face_index = 4;

    // Copy vertices from simplex and centroid of the tetrahedron
    centroid = B_ZERO_3;
    for (int i = 0; i < 4; ++i) 
    {
        poly[i] = smp[i];
        centroid += poly[i].vertex;
    }
    centroid *= BScalar(0.25);
    
    // Create 4 faces of tetrahedron
    // set up the faces and then fix winding based on normal direction
    // Face 0: vertices 0, 1, 2
    poly.faces[0][0].vid = 0;
    poly.faces[0][1].vid = 1;
    poly.faces[0][2].vid = 2;
    poly.faces[0].valid = true;
    
    // Face 1: vertices 0, 3, 1
    poly.faces[1][0].vid = 0;
    poly.faces[1][1].vid = 3;
    poly.faces[1][2].vid = 1;
    poly.faces[1].valid = true;
    
    // Face 2: vertices 0, 2, 3
    poly.faces[2][0].vid = 0;
    poly.faces[2][1].vid = 2;
    poly.faces[2][2].vid = 3;
    poly.faces[2].valid = true;
    
    // Face 3: vertices 1, 3, 2
    poly.faces[3][0].vid = 1;
    poly.faces[3][1].vid = 3;
    poly.faces[3][2].vid = 2;
    poly.faces[3].valid = true;
    
    // Copy vertex indices for witness point computation
    for (int f = 0; f < 4; ++f) // face f
    {
        for (int v = 0; v < 3; ++v) // vertex v
        {
            // don't overwrite whole object
            int vid = poly.faces[f][v].vid;
            poly.faces[f][v].index = smp[vid].index; 
        }
    }
    
    // Compute normals and fix winding
    for (int f = 0; f < 4; ++f) 
    {
        BEPAFace &face = poly.faces[f];

        const BVector3& v0 = poly[face[0]];
        const BVector3& v1 = poly[face[1]];
        const BVector3& v2 = poly[face[2]];
        
        const BVector3 e0 = v1 - v0;
        const BVector3 e1 = v2 - v0;
        
        const BVector3 normal = arb::cross(e0, e1);
        
        // If normal points toward centroid need to flip the winding
        const BVector3 to_centroid = centroid - v0;
        
        if (arb::dot(normal, to_centroid) > 0) 
        {
            // Swap v[1] and v[2]
            std::swap(face[1], face[2] );
        }
    }
}

// barycentric coordinate compute closest point on triangle to origin
void 
BOpenEPA::barycentric_origin(const BVector3& v0, const BVector3& v1, const BVector3& v2, BVector3 &a) const
{
    using std::abs;
    using std::clamp;

    // Compute vectors
    const BVector3 e0 = v1 - v0;
    const BVector3 e1 = v2 - v0;
    
    // Compute dot products for barycentric coords
    const BScalar d00 =  arb::dot(e0, e0);
    const BScalar d01 =  arb::dot(e0, e1);
    const BScalar d11 =  arb::dot(e1, e1);
    const BScalar d20 = -arb::dot(v0, e0);
    const BScalar d21 = -arb::dot(v0, e1);
    
    const BScalar denom = d00 * d11 - d01 * d01;
    
    if (abs(denom) < B_EPS) 
    {
        // Degenerate
        a[0] = a[1] = a[2] = BScalar(1.0) / BScalar(3.0);
        return;
    }
    
    const BScalar inv_denom = BScalar(1.0) / denom;
    const BScalar u = (d11 * d20 - d01 * d21) * inv_denom;
    const BScalar v = (d00 * d21 - d01 * d20) * inv_denom;
    const BScalar w = 1.0 - u - v;
    
    // Clamp to triangle
    if (w < 0.0) 
    {
        // Origin projects outside edge v1-v2
        // Project onto edge v1-v2
        BVector3 e12 = v2 - v1;
        
        BScalar t = -arb::dot(v1, e12) / arb::dot(e12, e12);
        t = clamp(t, BScalar(0.0), BScalar(1.0));
        a = {0.0, BScalar(1.0) - t, t};
    }
    else if (u < 0.0) 
    {
        // Origin projects outside edge v0-v2
        BScalar t = -arb::dot(v0, e1) / arb::dot(e1, e1);
        t = clamp(t, BScalar(0.0), BScalar(1.0));
        a = {BScalar(1.0) - t, 0.0, t};
    }
    else if (v < 0) 
    {
        // Origin projects outside edge v0-v1
        BScalar t = -arb::dot(v0, e0) / arb::dot(e0, e0);
        t = clamp(t, BScalar(0.0), BScalar(1.0));
        a = {BScalar(1.0) - t, t, 0.0};
    }
    else 
    {
        // Inside triangle
        a = {w, u, v};
    }
}

BVector3
BOpenEPA::point(const ABody *body, const  BIndex &idx) const
{
    // collider works in local coords; convert to 'world' coords using body pos/orientation
    // derivatives are injected here by body->orient() and body->pos()
    const BVector3 &pt = body->collider().point(idx);
    return (body->orient() * pt) + body->pos(); 
}

void
BOpenEPA::support_epa(const ABody* body1, const ABody* body2, const BVector3& dir, BGJKVertex& result) const
// support function for EPA but only care about Minkowski difference point
{
    // search body1
    const glm::dvec3 localDir1 = glm::transpose(body1->orient()) * dir;
    BScalar local_max1 = B_SCALAR_MIN;
    BIndex local_best1(-1,-1); 
    for (int i = 0; i < body1->collider().size(); ++i) 
    {
        for (int j = 0; j < body1->collider().getPoints(i).size(); ++j) 
        {
            BIndex idx(j,i); // (vertIdx, polyIdx)
            BScalar s = arb::dot(point(body1, idx), localDir1);
            
            if (s > local_max1) 
            {
                local_max1 = s;
                local_best1 = idx;
            }
        }
    }
    
    // search body2 in opposite direction
    const glm::dvec3 localDir2 = glm::transpose(body2->orient()) * dir;
    BScalar local_max2 = B_SCALAR_MIN;
    BIndex local_best2(-1,-1);
    for (int i = 0; i < body2->collider().size(); ++i) 
    {
        for (int j = 0; j < body2->collider().getPoints(i).size(); ++j) 
        {
            BIndex idx(j,i);  //  (vertIdx, polyIdx)
            BScalar s = arb::dot(point(body2, idx), localDir2);
            
            if (-s > local_max2) 
            {
                local_max2 = -s;
                local_best2 = idx;
            }
        }
    }
    
    // compute Minkowski difference point
    if (local_best1[0] >= 0 && local_best2[0] >= 0) 
    {
        result.vertex = point(body1, local_best1) - point(body2, local_best2);
        result.index = { local_best1, local_best2 }; 
    }
}


int
BOpenEPA::find_closest_face( const BEPAPolytope &poly ) const
{
    int closest_face = -1;
    BScalar closest_distance = B_SCALAR_MAX;
    
    for (int i = 0; i < poly.max_face_index; ++i) 
    {
        if (!poly.faces[i].valid) 
            continue;
        
        const BScalar distance = poly.faces[i].distance;
        if (distance >= 0.0 && distance < closest_distance) 
        {
            closest_distance = distance;
            closest_face = i;
        }
    }
    
    return closest_face;
}

bool
BOpenEPA::expand_simplex_to_tetrahedron( const ABody *body1, const ABody *body2, BSimplex &smp ) const
{
    // If GJK returned a degenerate simplex, rebuild it properly for EPA
    if (smp.vertexNum() != 4) 
    {
        // Need to get it up to 4 vertices
        if (smp.vertexNum() == 1) 
        {
            // Grow simplex from a single point: fire a support in some direction.
            // We use current simplex point for new direction for the
            // support; if this does not produce a new point, treat penetration as 0.
      
            // Parallel EPA support in that direction.
            BGJKVertex new_point;
            support_epa(body1, body2, smp[0], new_point);
            
            // Check if this is a new point relative to the existing simplex vertex.
            bool is_new = smp.is_new(new_point);
            
            if (is_new)
            {
                int idx = smp.vertexNum();
                smp[idx] = new_point;
                smp.vertexNum(2);
            }
            else 
            {
                // No new support point means penetration depth effectively zero.
                smp.witness(B1) = point(body1, new_point[B1]);
                smp.witness(B2) = point(body2, new_point[B2]);

                return false;
            }
        }
        if (smp.vertexNum() == 2) 
        {
            // Grow simplex from an edge: fire a support in a direction perpendicular
            // to the edge. If this does not produce a new point, treat penetration as 0.
            
            BVector3 edge = smp[1].vertex - smp[0].vertex;
            
            // Build a perpindicular 
            BVector3 axis = B_XAXIS;
            BScalar edge_norm = arb::length(edge);
            if (edge_norm > B_EPS && abs(edge[0]) > 0.9 * edge_norm) 
            {
                axis = B_YAXIS;
            }
            
            // dir = edge x axis
            BVector3 dir = arb::cross(edge, axis);
            BScalar nrm2 = arb::length2(dir);
            if (nrm2 < B_EPS) 
            {
                // Fallback axis
                axis = B_ZAXIS;
                dir = arb::cross(edge, axis);
            }
            //
            
            // Parallel EPA support in that direction.
            BGJKVertex new_point;
            support_epa(body1, body2, dir, new_point);
            
            // Check if this is a new point relative to both existing simplex vertices.
            if (smp.is_new(new_point)) 
            {
                int idx = smp.vertexNum();
                smp[idx] = new_point;
                smp.vertexNum(3);
            }
            else 
            {
                smp.witness(B1) = point(body1, new_point[B1]);
                smp.witness(B2) = point(body2, new_point[B2]);
          
                return false;
            }
        }
        if (smp.vertexNum() == 3) 
        {
            // Grow simplex from a triangle: fire a support in the direction of the
            // triangle normal. If this does not produce a new point, treat penetration as 0.

            const BVector3  e0 = smp[1].vertex - smp[0].vertex;
            const BVector3  e1 = smp[2].vertex - smp[0].vertex;
            
            // dir = e0 x e1 (normal to the triangle)
            BVector3 dir = arb::cross(e0, e1);
            
            // Parallel EPA support in that direction.
            BGJKVertex new_point;
            support_epa(body1, body2, dir, new_point);
            
            // Check if this is a new point relative to all three existing simplex vertices.
            if (smp.is_new(new_point)) 
            {
                int idx = smp.vertexNum();
                smp[idx] = new_point;
                smp.vertexNum(4);
            }
            else 
            {
                // Try opposite direction
                dir = -dir;
            }
            
            // If first direction didn't work, try opposite
            if (smp.vertexNum() == 3) 
            {
                support_epa(body1, body2, dir, new_point);
                
                if (smp.is_new(new_point)) 
                {
                    int idx = smp.vertexNum();
                    smp[idx] = new_point;
                    smp.vertexNum(4);
                }
                else 
                {
                    smp.witness(B1) = point(body1, new_point[B1]);
                    smp.witness(B2) = point(body2, new_point[B2]);
            
                    return false;
                }
            }
        }
        
        // If we still don't have 4 vertices, abort -- differs from gpu version (see below)
        if (smp.vertexNum() != 4) 
        {
            // Set witness points from best available simplex vertex
            int best = smp.vertexNum() > 0 ? smp.vertexNum() - 1 : 0;
            smp.witness(B1) = point(body1, smp[best][B1]);
            smp.witness(B2) = point(body2, smp[best][B2]);

            return false;
        }
    }
    
    return true;
}

void
BOpenEPA::tessellate_polytope( BEPAPolytope &poly, BGJKVertex &new_point, BVector3 &centroid )
{
    // Add new vertex to polytope
    int new_vertex_id = poly.num_vertices;
    poly[new_vertex_id] = new_point;
    poly.num_vertices++;
    
    // Update centroid incrementally (running mean)
    centroid += (new_point.vertex - centroid) / BScalar(poly.num_vertices);
    
    // Find horizon edges: collect edges from faces being removed this iteration
    // only, then mark them invalid. Collecting from ALL invalid faces (including
    // ones from previous iterations) would pull in stale interior edges.
    // BEPAEdge m_edges[B_MAX_EPA_FACES * 3]; // could be thread_local
    // std::array<BEPAEdge, B_MAX_EPA_FACES * 3> m_edges;
    
    int num_edges = 0;
    
    for (int f = 0; f < poly.max_face_index; ++f) 
    {
        if (!poly.faces[f].valid) 
            continue;
        
        if (!face_visible(poly, f, new_point)) 
            continue;
        
        BEPAFace &face = poly.faces[f];
        
        // Collect edges before invalidating the face
        if (num_edges < m_edges.size()) 
        {
            BEPAEdge &edge = m_edges[num_edges];
            edge.v1     = face[0];
            edge.v2     = face[1];
            edge.valid  = true;
            num_edges++;
        }
        if (num_edges < m_edges.size()) 
        {
            BEPAEdge &edge = m_edges[num_edges];
            edge.v1     = face[1];
            edge.v2     = face[2];
            edge.valid  = true;
            num_edges++;
        }
        if (num_edges < m_edges.size()) 
        {
            BEPAEdge &edge = m_edges[num_edges];
            edge.v1     = face[2];
            edge.v2     = face[0];
            edge.valid  = true;
            num_edges++;
        }
        
        face.valid = false;
    }
    
    // Remove duplicate edges (edges shared by two removed faces)
    for (int i = 0; i < num_edges; ++i) 
    {
        if (!m_edges[i].valid) 
            continue;
        
        for (int j = i + 1; j < num_edges; ++j)
        {
            if (!m_edges[j].valid) 
                continue;
            
            // check if same edge (both directions)
            if (m_edges[i] == m_edges[j])
                m_edges[i].valid = m_edges[j].valid = false;
        }
    }
    
    // Create new faces from horizon edges
    for (int i = 0; i < num_edges; ++i)
    {
        if (!m_edges[i].valid) 
            continue;
        
        // Find next available face slot; linear search could be improved using free list
        int new_face_idx = -1;
        for (int j = 0; j < B_MAX_EPA_FACES; ++j) 
        {
            if (!poly.faces[j].valid)
            {
                new_face_idx = j;
                break;
            }
        }
        
        if (new_face_idx < 0 || new_face_idx >= B_MAX_EPA_FACES) 
            break;
        
        BEPAFace &face = poly.faces[new_face_idx];
        
        //
        // Create new face: edge horizon vertices + new vertex
        face[0]       = m_edges[i].v1;
        face[1]       = m_edges[i].v2;
        face[2].vid   = new_vertex_id; 
        face[2].index = new_point.index;
        face.valid    = true;
        //
        //
        
        // Check winding and fix if necessary
        const BVector3& fv0 = poly[face[0]];
        const BVector3& fv1 = poly[face[1]];
        const BVector3& fv2 = poly[face[2]];
        
        const BVector3 fe0 = fv1 - fv0;
        const BVector3 fe1 = fv2 - fv0;
        const BVector3 fnormal = arb::cross(fe0, fe1);
        
        // If normal points toward centroid flip winding
        const BVector3 to_cent = centroid - fv0;
        
        if (arb::dot(fnormal, to_cent) > 0)
        {
            // Swap v[1] and v[2]
            std::swap(face[1], face[2]);
        }
        
        // Update max face index
        if (new_face_idx >= poly.max_face_index) 
        {
            poly.max_face_index = new_face_idx + 1;
        }
    }
}

//*******************************************************************************************
// Entry point to the EPA Implementation
//*******************************************************************************************

BScalar
BOpenEPA::collision( ABody* body1, ABody* body2, BSimplex &smp, BScalar distance, BVector3 &cnormal, BVector3& cpoint ) 
{
    using std::abs;

    if (distance > B_EPS) 
    {
        //  no penetration has been detected
        set_contact_normal(smp.witness(B1), smp.witness(B2), cnormal);
        return 0.0;
    }

    // ensure we have a volume (tetrahedron) to start EPA.
    // Degenerate cases handled inside expand_simplex_to_tetrahedron
    if (expand_simplex_to_tetrahedron(body1, body2, smp) == false)
    {
        // No new support point means penetration depth effectively zero.
        set_contact_normal(smp.witness(B1), smp.witness(B2), cnormal);
        //cpoint = (smp.witness(B1) + smp.witness(B2)) * BScalar(0.5);  
        return 0.0; 
    }   
    
    // initialization
    BEPAPolytope poly;
    BVector3 centroid;
    
    init_epa_polytope(poly, smp, centroid);
    
    int iters = 0;
    bool calc_witness = false;
    
    // main EPA loop - finds the 'right' vertices - the closest face to the origin
    for ( ; iters < m_max_iters; ++iters) 
    {
        // recompute geometric data for all current faces
        compute_face_normal_distance(poly);
      
        // find closest face
        int closest_face_idx = find_closest_face(poly);
        if (closest_face_idx < 0) 
        {
            calc_witness = false; // just exit
            break;
        }
        const BEPAFace& closest = poly.faces[closest_face_idx];
        
        // get new support point in direction of closest face normal
        BGJKVertex new_point;
        support_epa(body1, body2, closest.normal, new_point);
        
        // check for termination 
        BScalar dist_to_new = arb::dot(closest.normal, new_point.vertex);
        BScalar improvement = dist_to_new - closest.distance;
        if (static_cast<double>(improvement) < static_cast<double>(B_EPS_ABS)) 
        {
            calc_witness = true; // found boundary 
            break; 
        }
        
        // check for duplicate vertex to prevent infinite loops
        if (!smp.is_new(new_point)) 
        {
            calc_witness = true; // found duplicate vertex
            break; 
        }
        
        if (poly.num_vertices >= B_MAX_EPA_VERTICES - 1)  
        {
            calc_witness = false; // just exit
            break;
        }
        
        // still not at the boundary? expand polytope by adding new vertex 
        tessellate_polytope(poly, new_point, centroid);
    }
    // end main loop

    if (iters >= m_max_iters)        
    {
        // only if we run out of iterations - recompute geometric data  
        compute_face_normal_distance(poly);
        calc_witness = true;
    }
    
    BScalar cdepth(0.0);
    
    if (calc_witness)
    {
        int closest_face_idx = find_closest_face(poly);
        
        if (closest_face_idx >= 0) 
        {
            const BEPAFace &face = poly.faces[closest_face_idx];
       
            BVector3 bary;
            
            // barycentric coords to find where the origin projects onto face
            barycentric_origin(poly[face[0]], poly[face[1]], poly[face[2]], bary);
            
            // calc witness points
            smp.witness(B1) =  point(body1, face[0][B1]) * bary[0] 
                             + point(body1, face[1][B1]) * bary[1]  
                             + point(body1, face[2][B1]) * bary[2];
            
            smp.witness(B2) =  point(body2, face[0][B2]) * bary[0]  
                             + point(body2, face[1][B2]) * bary[1] 
                             + point(body2, face[2][B2]) * bary[2];
            
            //
            // set contact parameters
            //
            //cpoint = smp.witness(B1) - (cnormal * (depth * BScalar(0.5)));
            cpoint   = (smp.witness(B1) + smp.witness(B2)) * BScalar(0.5); // matches libccd
            cnormal  = face.normal;
            cdepth   = face.distance; // collision and penetration  results in poitive depth/distance
        }
    }
    
    return cdepth;
}
