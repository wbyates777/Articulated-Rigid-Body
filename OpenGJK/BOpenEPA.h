/* BOpenEPA 17/03/2026

 $$$$$$$$$$$$$$$$$$
 $   BOpenEPA.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 My c++ rewrite of the EPA in openGJK 
 
 The main changes are:
 
 i)   glm based Vector3 types which can be made differentiable,
 ii)  the structs EPAVertex and BIndex with operators that simplify the syntax
 iii) broke up the large collisions function into more managable logical chunks,
      by adding find_closest_face(), expand_simplex_to_tetrahedron(),
      tessellate_polytope(), and compute_contact()

 
 see https://github.com/MattiaMontanari/openGJK 
 
 
 Note: OpenGJK is released under a GPL3 license. As a result if this code is used 
 then *all* the ARB code base is also bound by the GPL3 license.
 
 
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
#define __BOPENEPA_H__

#include <string>
#include <vector>
#include <map>

#ifndef __BPSIMPLEX_H__
#include "BSimplex.h"
#endif

#ifndef __ABODY_H__
#include "ABody.h"
#endif

class BEPAPolytope;
class BEPAFace;


//
// local EPA types
//

constexpr  int B_MAX_EPA_FACES  = 128;
constexpr  int B_MAX_EPA_VERTICES  = (B_MAX_EPA_FACES + 4);



struct BEPAVertex 
{
    operator int&( void ) { return vid; }
    operator int( void ) const { return vid; }
    
    BIndex &operator[]( int body_idx ) { return index[body_idx]; }
    const BIndex &operator[]( int body_idx ) const { return index[body_idx]; }
    
    // (vertexId,polyId) for body1, (vertexId,polyId) for body2, 
    std::array<BIndex, 2> index; 
    int vid = -1; // vertex index into EPAPolytope 
};


// Structure for horizon edge collection
struct BEPAEdge
{
    bool 
    operator==( const BEPAEdge &e ) const
    { 
        return (v1.vid == e.v1.vid && v2.vid == e.v2.vid) ||
               (v1.vid == e.v2.vid && v2.vid == e.v1.vid); 
    }

    BEPAVertex v1;
    BEPAVertex v2;
    bool valid = false;
};


class BOpenEPA
{

public:

    BOpenEPA( void ): m_max_iters(25) {}
    ~BOpenEPA( void )=default;
    

    BScalar 
    collision( ABody* body1, ABody* body2, BSimplex &simplex, BScalar distance, 
               BVector3 &cnormal, BVector3& cpoint );
    
    void 
    max_iters( int iters ) { m_max_iters = iters; }
    
    int 
    max_iters( void ) const { return m_max_iters; }
    

private:

    void 
    compute_face_normal_distance( BEPAPolytope& poly ) const;
    
    bool 
    face_visible( const BEPAPolytope &poly, int face_idx, const BVector3 &point ) const;
    
    void
    init_epa_polytope( BEPAPolytope &poly, const BSimplex &smp, BVector3 &centroid ) const;
    
    void 
    barycentric_origin( const BVector3 &v0, const BVector3 &v1, const BVector3 &v2, BVector3 &a ) const;
    
    void
    support_epa( const ABody *body1, const ABody *body2, const BVector3 &dir, BGJKVertex &result ) const;
    
    void 
    set_contact_normal( const BVector3 &w1, const BVector3 &w2, BVector3 &cnormal ) const;

    int 
    find_closest_face( const BEPAPolytope &poly ) const;
    
    bool        
    expand_simplex_to_tetrahedron( const ABody *body1, const ABody *body2, BSimplex &smp ) const;
    
    void
    tessellate_polytope( BEPAPolytope &poly, BGJKVertex &new_point, BVector3 &centroid );
    
    
    BVector3
    point(const ABody *body, const BIndex &idx) const;
    
    int m_max_iters;
    std::array<BEPAEdge, B_MAX_EPA_FACES * 3> m_edges;
    
};

#endif


