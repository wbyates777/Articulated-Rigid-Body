/* BGJK 20/12/2025

 $$$$$$$$$$$$$$
 $   BGJK.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Used for collision detection.
 
 My c++ wrapper for external c library libcdd, see https://github.com/danfis/libccd

 An implemetation of: 
 
 i) the Gilbert–Johnson–Keerthi (GJK) distance algorithm, and 
 ii) the Expanding-Polytope-Algorithm (EPA). 

 Computes penetration of body2 into body1. Returns distance/depth of penetration, contact normal direction, 
 and contact position. 

 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
 It is not the most recent implementation of GJK but it is well tested, fairly efficient and robust.
 Also it does not depend directly on Eigen.
 
*/

/***
 * libccd
 * ---------------------------------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef __BGJK_H__
#define __BGJK_H__



#ifndef __ABODY_H__
#include "ABody.h"
#endif

#include <ccd/ccd.h>
#include <glm/vec3.hpp>

class BGJK
{

public:

    BGJK( void );
    ~BGJK( void )=default;

    // if a collision has taken place fill in the contact distance, normal and point, and return true 
    bool
    collision( ABody* body1, ABody* body2, BScalar &distance, BVector3 &cnormal, BVector3 &cpoint ); 
    
private:

    
    static glm::dvec3 
    toGLM(const ccd_vec3_t *v) { return glm::dvec3(ccdVec3X(v), ccdVec3Y(v), ccdVec3Z(v)); }

    static void
    setVec3(ccd_vec3_t *v1, const glm::dvec3 &v2) { ccdVec3Set(v1, v2.x, v2.y, v2.z); }
    
    static void 
    first_dir_fn(const void *obj1, const void *obj2, ccd_vec3_t *dir);
    
    static void
    support_fn(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);
    
    static BVector3
    first_point( const ABody *body );
    
    static BVector3
    max_point( const ABody *body, const BVector3& dir ); 
    
    ccd_t m_ccd;

};

#endif


