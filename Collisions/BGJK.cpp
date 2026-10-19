/* BGJK 20/12/2025

 $$$$$$$$$$$$$$$$$
 $   BGJK.cpp   $
 $$$$$$$$$$$$$$$$$

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


#ifndef BGJK_H
#include "BGJK.h"
#endif

#include <iostream>

#ifndef ABODY_H
#include "ABody.h"
#endif

#ifndef BCOLLIDER_H
#include "BCollider.h"
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>


BGJK::BGJK( void ) : m_ccd{} 
{
    CCD_INIT(&m_ccd);
    
    // these defaults suggested by libccd
    m_ccd.max_iterations  = 100;     
    m_ccd.epa_tolerance   = 0.001; // was 0.0001
    m_ccd.dist_tolerance  = 1E-6;   
    
    m_ccd.first_dir = first_dir_fn; 
    
    m_ccd.support1  = support_fn;   
    m_ccd.support2  = support_fn;  
}


//
// support functions for BCollider shape i.e. BPolytope, BBox or BSpheres
//
BVector3
BGJK::first_point( const ABody *body ) 
{ 
    // convert body/model vertex to base/world vertex; 
    const BVector3 vert =  body->collider().first_point();
    return (body->orient() * vert) + body->pos(); 
}

BVector3
BGJK::max_point( const ABody *body, const BVector3 &dir ) 
{
    // convert from world to body/model coords
    const BVector3 mydir = (glm::transpose(body->orient()) * dir);
  
    const BVector3 vert =  body->collider().max_point(mydir);
    // convert body/model vertex to base/world vertex; 
    return (body->orient() * vert)  + body->pos();
}

//
// these are passed to ccd as pointers to functions -- they call the functions above
//
void 
BGJK::first_dir_fn( const void *obj1, const void *obj2, ccd_vec3_t *dir )
{
    const ABody *b1 = static_cast<const ABody*>(obj1);
    const ABody *b2 = static_cast<const ABody*>(obj2);
    const glm::dvec3 v1 = first_point(b1);
    const glm::dvec3 v2 = first_point(b2);
    setVec3(dir, v1 - v2);
}

void
BGJK::support_fn( const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec )
{
    const ABody *body = static_cast<const ABody*>(obj);
    const glm::dvec3 mydir(toGLM(dir)); 
    const glm::dvec3 v = max_point(body, mydir);
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
BGJK::collision( ABody* b1, ABody* b2, BScalar &cdepth, BVector3 &cnormal, BVector3 &cpoint )
{
    using std::abs;
    
    ccd_vec3_t dir, pos;
    ccdVec3Set(&dir, CCD_ZERO, CCD_ZERO, CCD_ZERO);
    ccdVec3Set(&pos, CCD_ZERO, CCD_ZERO, CCD_ZERO);

    // perform the test
    double depth = -1.0;
    int res = ccdGJKPenetration(b1, b2, &m_ccd, &depth, &dir, &pos);
    //
    
    bool retVal = false;
    cdepth = 0.0; 
    
    using std::abs;
    
    if (res == 0)
    {
        // GJK - a positive depth indicates a penetration
        assert(depth > 0.0);
        
        cdepth = BScalar(depth);
        
        cnormal = toGLM(&dir);
        assert(!arb::isnan(cnormal)); 
        assert((abs(arb::length(cnormal) - 1.0) < 1E-8));
    
        cpoint = toGLM(&pos);
        
        // CCD has known issue with large flat faces
        // this fix attempts to keep cpoint within an object's bounds
        // note no orientation taken into considertion
        const BVector3 top1 = b1->box().top() + b1->pos();
        const BVector3 bot1 = b1->box().bot() + b1->pos();
        
        const BVector3 top2 = b2->box().top() + b2->pos();
        const BVector3 bot2 = b2->box().bot() + b2->pos();
        
        const BVector3 mymin = arb::max(bot1, bot2);
        const BVector3 mymax = arb::min(top1, top2);
    
        for (int i = 0; i < 3; ++i)
        {
            if (mymin[i] < mymax[i])
                cpoint[i] = std::clamp(cpoint[i], mymin[i], mymax[i]);
            else cpoint[i] = std::clamp(cpoint[i], mymax[i], mymin[i]);
        }
        //
        
        cpoint += (cnormal * abs(cdepth)); 
      
        retVal = true;
    }
    else if (res == -1)
    {
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

BBox 
BGJK::toBaseBox( const BMatrix4 &trans, const BVector3 &lo, const BVector3 &hi ) const
// the resulting box is 'generous' - it contains the orientated bounding box
{
    const BVector4 corners[8] = 
    {
        {lo.x, lo.y, lo.z, 1.0},
        {hi.x, lo.y, lo.z, 1.0},
        {lo.x, hi.y, lo.z, 1.0},
        {hi.x, hi.y, lo.z, 1.0},
        {lo.x, lo.y, hi.z, 1.0},
        {hi.x, lo.y, hi.z, 1.0},
        {lo.x, hi.y, hi.z, 1.0},
        {hi.x, hi.y, hi.z, 1.0}
    };
    
    BVector3 top(B_SCALAR_MIN), bot(B_SCALAR_MAX);
    
    for (int c = 0; c < 8; ++c) 
    {
        BVector3 p(trans * corners[c]);
        
        top = arb::max(p, top);
        bot = arb::min(p, bot);
        
    }
    return BBox(top, bot);
}

void
BGJK::collision( ABody *b1, ABody *b2, std::vector<BContactInfo> &cinfo ) 
{
    const BCollider &c1 = b1->collider();
    const BCollider &c2 = b2->collider();
    
    // model transform body (local) to base (world)
    const BMatrix4 m1 = glm::translate(b1->pos()) * BMatrix4(b1->orient());
    const BMatrix4 m2 = glm::translate(b2->pos()) * BMatrix4(b2->orient()); 
    
    // broad-phase - test all sub-polytopes pairs between b1 and b2
    for (int i = 0; i < c1.polytopes().size(); ++i) 
    {
        const BPolytope &poly1 = c1.polytope(i);
        BBox box1 = toBaseBox(m1, poly1.m_min, poly1.m_max);
        
        for (int j = 0; j < c2.polytopes().size(); ++j) 
        {
            const BPolytope &poly2 = c2.polytope(j);
            BBox box2 = toBaseBox(m2, poly2.m_min, poly2.m_max);
            
            // filter out sub-polytopes that don't intersect
            if (!arb::intersect(box1, box2)) 
                continue; 
 
            // narrow-phase: run GJK/EPA on this specific pair of polytopes
            // a positive distance indicated the bodies are separated -- index i is poly1, index j is poly2
            // a (near) zero distance indicates contact; a penetration or 'just touching'
            // a positive cdepth indicates a penetration 
            BVector3 cnormal, cpoint;
            BScalar cdepth;
        
            b1->collider().setPolyIdx(i);
            b2->collider().setPolyIdx(j);
            
            bool penetration = collision( b1, b2, cdepth, cnormal, cpoint ); 
                      
            if (penetration) 
                cinfo.emplace_back(cdepth, cnormal, cpoint);
        }
    }
}



//



