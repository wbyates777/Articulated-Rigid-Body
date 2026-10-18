/* BOpenGJK 15/12/2025

 $$$$$$$$$$$$$$$$$$
 $   BOpenGJK.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 My c++ rewrite of openGJK/openEPA so we can use glm functions and therefore autodiff
 
 openEPA provides us with the indices into the two polytopes; libccd did not provide this.
 
 We have added an extra level of indexing so that each body can have a number of polytopes.
 
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


#ifndef BOPENGJK_H
#define BOPENGJK_H

#include <vector>

#ifndef ABODY_H
#include "ABody.h"
#endif

#ifndef BPSIMPLEX_H
#include "BSimplex.h"
#endif


#ifndef BPRODUCTS_H
#include "BProducts.h"
#endif


class BCollider;

class BOpenGJK
{
    
public:

    BOpenGJK(void) : m_max_iters(25) {}
    ~BOpenGJK(void)=default;
    
    /*! @brief Invoke the GJK algorithm to compute the minimum distance between two
     * polytopes.
     *
     * @param[in]     bdy1  First polytope (passed by value, modified internally).
     * @param[in]     bdy2  Second polytope (passed by value, modified internally).
     * @param[in,out] smp   Simplex structure. Must be initialized (set nvrtx = 0)
     *                       before first call. After return, contains the final
     *                       simplex and witness points in s->witnesses.
     * @return The minimum Euclidean distance between the two polytopes.
     *         Returns 0 if the polytopes are intersecting or touching.
     *
     * @note The simplex has to be initialised prior the call to this function.
     *       Witness points are automatically computed and stored in s.witnesses.
     */
    BScalar 
    min_distance( ABody *bdy1, int polyIdx1, ABody *bdy2, int polyIdx2, BSimplex &smp );
  
    
    void 
    max_iters( int iters ) { m_max_iters = iters; }
    
    int 
    max_iters( void ) const { return m_max_iters; }
    
private:
    

    void 
    projectOnLine( const BVector3 &p, const BVector3 &q, BVector3 &v ) const
    {
        const BVector3 pq = p - q;
        v = p - (pq * (arb::dot(p, pq) / arb::dot(pq, pq)));
    }

    void 
    projectOnPlane( const BVector3 &p, const BVector3 &q, const BVector3 &r, BVector3 &v ) const
    {
        const BVector3 n = arb::cross(p - q, p - r);
        v = n * (arb::dot(n, p) / arb::dot(n, n));
    }
    
    int 
    hff1( const BVector3 &p, const BVector3 &q ) const 
    {
        return (arb::dot(p, p) > arb::dot(p, q));
    }

    int 
    hff2( const BVector3 &p, const BVector3 &q,  const BVector3 &r ) const 
    {
        const BVector3 pq = q - p;
        const BVector3 pr = r - p;
        return (arb::dot(p, arb::cross(pq, arb::cross(pq, pr))) < 0); // discard r if true
    }

    int 
    hff3( const BVector3 &p, const BVector3 &q, const BVector3 &r ) const 
    {
        return (arb::dot(p, arb::cross(q, r)) <= 0);  // discard s if true
    }
    
 
    void 
    S1D( BSimplex &smp, BVector3 &v ) const;
    
    void 
    S2D( BSimplex &smp, BVector3 &v ) const;
    
    void 
    S3D( BSimplex &smp, BVector3 &v ) const;
    
    void 
    subalgorithm( BSimplex &smp, BVector3 &v ) const;
    
    
    void 
    W0D( const ABody *body1, const ABody *body2, BSimplex &smp ) const; 
    
    void 
    W1D( const ABody *body1, const ABody *body2, BSimplex &smp ) const;
    
    void 
    W2D( const ABody *body1, const ABody *body2, BSimplex &smp ) const;
    
    void 
    W3D( const ABody *body1, const ABody *body2, BSimplex &smp ) const;
    
    void
    calc_witness( const ABody *body1, const ABody *body2, BSimplex &smp ) const;
    
    BVector3
    first_point( ABody *body, int polyIdx ) const;
    
    BVector3
    max_point( ABody *body, const BVector3& dir, int polyIdx ) const; 
    
    BVector3
    point(const ABody *body, const  BIndex &idx) const;
    
    int m_max_iters;
};


#endif  // __BOPENGJK_H__


