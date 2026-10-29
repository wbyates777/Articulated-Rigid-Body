/*! 
 @file BGJK.h
 @date 17/05/2026

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
 
 Returns true if collision with depth of penetration, contact normal direction, and contact position. 
 Note  penetration depth is always positive.
 
 
 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
 This code is differentiable. 
 

 Notes 
 
 i) OpenGJK is released under a GPL3 license. As a result if OpenGJK code is used 
    then *all* the code in the release/product is also bound by the GPL3 license.
 
 ii) EPA returns positive depth for penetration; 
 
 iii) GJK is not great when working with large flat surfaces or boxes 
 
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


#ifndef BGJK_H
#define BGJK_H



#include <string>
#include <vector>
#include <map>

#ifndef ABODY_H
#include "ABody.h"
#endif

#ifndef BMANIFOLD_H
#include "BManifold.h"
#endif

#ifndef BBOX_H
#include "BBox.h"
#endif

#ifndef BOPENGJK_H
#include "BOpenGJK.h"
#endif

#ifndef BOPENEPA_H
#include "BOpenEPA.h"
#endif

#ifndef BCOLLIDER_H
#include "BCollider.h"
#endif



class BGJK  
{

public:

    BGJK( void )=default;
    ~BGJK( void )=default;
    
 
    /*! @brief Detect if a collision has taken place and fill in the BContactInfo (depth, normal and point).
     *
     * @param[in] body1       pointer to body1
     * @param[in] body2       pointer to body2
     * @param[out] cinfo      contact infomation (depth, normal and point)
     */
    void
    collision( ABody *body1, ABody *body2, std::vector<BContactInfo> &cinfo );
    
private:


    BBox 
    toBaseBox( const BMatrix4 &trans, const BVector3 &lo, const BVector3 &hi ) const;
    
    BOpenGJK m_gjk;  
    BOpenEPA m_epa;
};

#endif


