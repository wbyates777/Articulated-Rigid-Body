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
 
 Returns positive depth of penetration, contact normal direction, and contact position. 

 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
 This code is differentiable using autodiff 
 

 Notes: 
 
 i) OpenGJK is released under a GPL3 license. As a result if this code is used 
 then *all* the ARB code base is also bound by the GPL3 license.
 ii) GJK/EPA not good with boxes - big flat collision surfaces can cause problems computing collision point,
     this is why proffesional systems use (faster) specialised box/box collisions.
 
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



#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BBOX_H__
#include "BBox.h"
#endif

#ifndef __BOPENGJK_H__
#include "BOpenGJK.h"
#endif

#ifndef __BOPENEPA_H__
#include "BOpenEPA.h"
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
    collision( ABody *b1, ABody *b2, BScalar &cdepth, BVector3 &cnormal, BVector3 &cpoint )
    {
        BSimplex simplex;
        
        // a positive distance indicated the bodies are separated
        BScalar distance = m_gjk.min_distance( b1, b2, simplex );
        // a (near) zero distance indicates contact; a penetration or 'just touching'
        cdepth = m_epa.collision( b1, b2, simplex, distance, cnormal, cpoint );

        // large flat surfaces/boxes can cause a cpoint dimension fall outside the object 
        // keep cpoint within objects - note no orientation taken into considertion
        const BVector3 top1 = b1->box().top() + b1->pos();
        const BVector3 bot1 = b1->box().bot() + b1->pos();
        const BVector3 top2 = b2->box().top() + b2->pos();
        const BVector3 bot2 = b2->box().bot() + b2->pos();
        
        const BVector3 mymin = arb::max(bot1, bot2);
        const BVector3 mymax = arb::min(top1, top2);
    
        for (int i = 0; i < 3; ++i)
        {
            if (mymin[i] < mymax[i])
                cpoint[i] = arb::clamp(cpoint[i], mymin[i], mymax[i]);
            else cpoint[i] = arb::clamp(cpoint[i], mymax[i], mymin[i]);
        }
        //
        
        // a positive depth indicates a penetration
        return (cdepth > 0) ? true : false; 
    }
    
private:

    BOpenGJK                     m_gjk;  
    BOpenEPA                     m_epa;

};

#endif


