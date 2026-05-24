/* BGJK 17/05/2026

 $$$$$$$$$$$$$$
 $   BGJK.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Narrow-phase collision detection.
 
 My c++ wrapper for external c++ class openGJK/openEPA, see https://github.com/MattiaMontanari/openGJK

 An implemetation of: 
 
 i) the Gilbert–Johnson–Keerthi (GJK) distance algorithm, and 
 ii) the Expanding-Polytope-Algorithm (EPA). 

 Computes penetration of body2 into body1. 
 Returns distance/depth of penetration, contact normal direction, and contact position. 

 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
 This code is differentiable. 
 

 Notes 
 
 i) OpenGJK is released under a GPL3 license. As a result if OpenGJK code is used 
    then *all* the code in the release/product is also bound by the GPL3 license.
 
 ii) EPA returns positive depth for penetration; 
 
 
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


#ifndef __BGJK_H__
#define __BGJK_H__



#include <string>
#include <vector>
#include <map>

#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BOPENGJK_H__
#include "BOpenGJK.h"
#endif

#ifndef __BOPENEPA_H__
#include "BOpenEPA.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif



class BGJK  
{

public:

    BGJK( void )=default;
    ~BGJK( void )=default;
    
    //
    // if a collision has taken place fill in the contact depth, normal and point, and return true 
    //
    bool
    collision( ABody *b1, ABody *b2, BScalar &depth, BVector3 &cnormal, BVector3 &cpoint )
    {
        BSimplex simplex;
        BScalar distance = m_gjk.min_distance( b1, b2, simplex );
        depth = m_epa.collision( b1, b2, simplex, distance, cnormal, cpoint );
        return (depth > 0) ? true : false;      // a positive depth indicates a collision
    }
    
private:

    BOpenGJK                     m_gjk;  
    BOpenEPA                     m_epa;

};

#endif


