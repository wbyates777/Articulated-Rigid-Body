/* BSimplex 17/12/2025

 $$$$$$$$$$$$$$$$$$
 $   BSimplex.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A generalization of the notion of a triangle or tetrahedron to  
 arbitrary dimensions (see https://en.wikipedia.org/wiki/Simplex).
  
 After calling BOpenGJK::min_distance():
 
    - witnesses(0) contains the closest point on body1
    - witnesses(1) contains the closest point on body2
 
 These are computed using barycentric coordinates
 from the final simplex vertices.
 
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


#ifndef __BSIMPLEX_H__
#define __BSIMPLEX_H__


#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BFUNCTIONS_H__
#include "BFunctions.h"
#endif

// namespace opengjk

/** Relative tolerance for convergence (scaled machine epsilon) */
const BScalar B_EPS_REL  = B_EPS * BScalar(1E4);
const BScalar B_EPS_REL2 = B_EPS_REL * B_EPS_REL;

/** Absolute tolerance for convergence (scaled machine epsilon) */
const BScalar B_EPS_ABS  = B_EPS * BScalar(1E2);
const BScalar B_EPS_ABS2 = B_EPS_ABS * B_EPS_ABS;


// body1, body2
constexpr  int B1 = 0;
constexpr  int B2 = 1;


typedef glm::vec<2,short>   BIndex; 


struct BGJKVertex 
{
    operator BVector3&( void ) { return vertex; }
    operator const BVector3&( void ) const { return vertex; }
    
    BIndex& operator[]( int body_idx ) { return index[body_idx]; }
    const BIndex& operator[]( int body_idx ) const { return index[body_idx]; }
    
    BVector3 vertex;// = BVector3(0.0);
    
    // (vertId, polyId) for body1, (verIdt, polyId) for body2,
    std::array<BIndex, 2> index;// = {BIndex(-1),BIndex(-1)};   
};

inline std::ostream&
operator<<( std::ostream &ostr, const BGJKVertex &v )
{
    ostr << v.vertex << '\n';
    for (int i = 0; i < 2; ++i)
        ostr << v.index[i] << ' ';
 
    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BGJKVertex &v )
{
    istr >> v.vertex;
    for (int i = 0; i < 2; ++i)
        istr >> v.index[i];

    return istr;
}




class BSimplex 
{
public:
    
    BSimplex(void) : m_nvrtx(0), m_point(), m_witness() {}
    ~BSimplex(void)=default;
    
    void
    init( const BVector3 &v )
    {
        m_nvrtx = 1;
        m_point[0].vertex = v;
        m_point[0][0] = BIndex(0); // point 0, body 1
        m_point[0][1] = BIndex(0); // point 0, body 2
    }

    void 
    vertexNum( int n ) { m_nvrtx = n; }
    
    int 
    vertexNum( void ) const { return m_nvrtx; }
    
    void
    vertexMove( int i, int j ) { m_point[i] = m_point[j]; }
    

    BGJKVertex&
    operator[]( int i ) { return m_point[i]; }
    
    const BGJKVertex&
    operator[]( int i ) const { return m_point[i]; }
 
    int
    size( void ) const { return m_nvrtx; }

    BVector3&
    witness( int i ) { return m_witness[i]; }
    
    const BVector3&
    witness( int i ) const { return m_witness[i]; }
    
    
    void
    select(const BGJKVertex &s1,  const BGJKVertex &s2)
    {
        m_nvrtx = 3;                
        m_point[2] = m_point[3];
        m_point[1] = s1;     
        m_point[0] = s2;  
    }

    void 
    select(const BGJKVertex &s)
    {
        m_nvrtx =  2;
        m_point[1] = m_point[3];                 
        m_point[0]  = s;
    }
    
    bool
    is_new(const BGJKVertex &new_point) const
    {
        for (int vtx = 0; vtx < m_nvrtx; ++vtx) 
        {
            const BScalar d2 = arb::length2(new_point.vertex - m_point[vtx].vertex);
            if (d2 < B_EPS2) 
                return false;
        }
        return true;
    }
    
    friend std::ostream&
    operator<<( std::ostream &ostr, const BSimplex &smp );
    
    friend std::istream& 
    operator>>( std::istream &istr, BSimplex &smp );
    
private:
    
    std::array<BGJKVertex, 4> m_point;   /*!< Coordinates and Indices of the points of the simplex. */
    std::array<BVector3, 2> m_witness;   /*!< Coordinates of the witness points. */
    int m_nvrtx;                         /*!< Number of points defining the simplex.  */
};



inline std::ostream&
operator<<( std::ostream &ostr, const BSimplex &smp )
{
    ostr << smp.m_nvrtx << '\n';
    
    for (int i = 0; i < smp.m_point.size(); ++i)
        ostr << smp.m_point[i] << '\n';

    for (int i = 0; i < smp.m_witness.size(); ++i)
        ostr << smp.m_witness[i] << '\n';

    return ostr;
}

inline std::istream& 
operator>>( std::istream &istr, BSimplex &smp )
{
    istr >> smp.m_nvrtx;
    
    for (int i = 0; i < smp.m_point.size(); ++i)
        istr >> smp.m_point[i];

    for (int i = 0; i < smp.m_witness.size(); ++i)
        istr >> smp.m_witness[i];

    return istr;
}

#endif


