
/* BGJK 17/05/2026

 $$$$$$$$$$$$$$
 $   BGJK.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
*/



#ifndef BGJK_H
#include "BGJK.h"
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

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
            
            // filter out polytopes that don't intersect
            if (!arb::intersect(box1, box2)) 
                continue; 
 
            // narrow-phase: run GJK/EPA on this specific pair of polytopes
            // a positive distance indicated the bodies are separated -- index i is poly1, index j is poly2
            // a (near) zero distance indicates contact; a penetration or 'just touching'
            // a positive cdepth indicates a penetration 
            BSimplex smp;
            BVector3 cnormal, cpoint;
            BScalar distance = m_gjk.min_distance( b1, i, b2, j, smp);
            BScalar cdepth = m_epa.collision(b1, i, b2, j, smp, distance, cnormal, cpoint);
            
            if (cdepth > 0) 
                cinfo.emplace_back(cdepth, cnormal, cpoint);
        }
    }
}

