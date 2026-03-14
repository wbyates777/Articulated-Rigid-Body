/* BGJK 20/12/2025

 $$$$$$$$$$$$$$$$
 $   BGJK.cpp   $
 $$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 My c++ wrapper for libcdd, see https://github.com/danfis/libccd
 
 An implemetation of GFK and EPA; the  Gilbert–Johnson–Keerthi distance algorithm
 and the Expanding-Polytope-Algorithm, respectively. 

 Computes penetration of body2 into body1. Returns depth of penetration, contact position, 
 and contact normal direction.  

 see https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 
*/


#ifndef __BGJK_H__
#include "BGJK.h"
#endif

#include <iostream>



#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif




BGJK::BGJK( void ) : m_ccd{} 
{
    CCD_INIT(&m_ccd);
    
    // these defaults suggested by libccd
    m_ccd.max_iterations  = 100;     
    m_ccd.epa_tolerance   = 0.0001;
    m_ccd.dist_tolerance  = 1E-6;   
    
    m_ccd.first_dir = first_dir_fn; 
    
    m_ccd.support1  = support_fn;   
    m_ccd.support2  = support_fn;  
}


//
// support functions for BCollider shape i.e. BPolytope, BBox or BSpheres
//

void 
BGJK::first_dir_fn(const void *obj1, const void *obj2, ccd_vec3_t *dir)
{
    const ABody *body1 = static_cast<const ABody*>(obj1);
    const ABody *body2 = static_cast<const ABody*>(obj2);
    const glm::dvec3 v1 = body1->collider().first_point();
    const glm::dvec3 v2 = body2->collider().first_point();
    setVec3(dir, v1 - v2);
}

void
BGJK::support_fn(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec)
{
    const ABody *body = static_cast<const ABody*>(obj);
    const glm::dvec3 mydir(toGLM(dir)); 
    const glm::dvec3 v = body->collider().max_point(mydir);
    setVec3(vec, v); 
}


/**
 * Computes penetration of obj2 into obj1.
 * Depth of penetration, direction and position is returned. It means that
 * if obj2 is translated by distance depth in direction dir objects will
 * have touching contact, pos should be position in global coordinates
 * where force should take a place.
 *
 * GJK+EPA algorithm is used.
 *
 * Returns 0 if obj1 and obj2 intersect and depth, dir and pos are filled
 * if given non-NULL pointers.
 * If obj1 and obj2 don't intersect -1 is returned.
 * If memory allocation fails -2 is returned.
 */
bool
BGJK::collision( BContact &c )
{
    double depth = -1.0;
    
    ccd_vec3_t dir, pos;
    
    ccdVec3Set(&dir, CCD_ZERO, CCD_ZERO, CCD_ZERO);
    ccdVec3Set(&pos, CCD_ZERO, CCD_ZERO, CCD_ZERO);

    //
    int res = ccdGJKPenetration(c.body1, c.body2, &m_ccd, &depth, &dir, &pos);
    //
    
    bool retVal = false;
    
    if (res == 0)
    {
        c.norm      = toGLM(&dir);
        c.pos       = toGLM(&pos);
        c.depth     = depth;
        
        //assert(arb::isnan(c.norm));
        
        retVal  = true;
    }
    else if (res == -1)
    {
        c.depth = -1;
        retVal = false;
    }
    else if (res == -2)
    {
        // if memory allocation fails -2 
        std::cout << "Error, GJK memory allocation fail" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    return retVal;
}


//



