/* BGJK 20/12/2025

 $$$$$$$$$$$$$$
 $   BGJK.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 My c++ wrapper for external c library libcdd, see https://github.com/danfis/libccd

 An implemetation of: 
 
 i) the Gilbert–Johnson–Keerthi (GJK) distance algorithm, and 
 ii) the Expanding-Polytope-Algorithm (EPA). 

 Computes penetration of body2 into body1. Returns depth of penetration, contact position, 
 and contact normal direction. 

 Used for collision detection.
 
 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
 It is not the most recent implementation of GJK but it is well tested, fairly efficient and robust.
 Also it does not depend directly on Eigen.
 

*/

#ifndef __BGJK_H__
#define __BGJK_H__


#ifndef __BCONTACT_H__
#include "BContact.h"
#endif

#include <ccd/ccd.h>
#include <glm/vec3.hpp>

class BGJK
{

public:

    BGJK( void );
    ~BGJK( void )=default;

    bool
    collision( BContact &contact ); // if true, fill out details in contact
    
private:

    
    static glm::dvec3 
    toGLM(const ccd_vec3_t *v) { return glm::dvec3(ccdVec3X(v), ccdVec3Y(v), ccdVec3Z(v)); }

    static void
    setVec3(ccd_vec3_t *v1, const glm::dvec3 &v2) { ccdVec3Set(v1, v2.x, v2.y, v2.z); }
    
    static void 
    first_dir_fn(const void *obj1, const void *obj2, ccd_vec3_t *dir);
    
    static void
    support_fn(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);
    
    
    ccd_t m_ccd;

};

#endif


