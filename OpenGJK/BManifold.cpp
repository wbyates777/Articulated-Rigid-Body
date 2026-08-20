/* BManifold 23/07/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   BManifold.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Assume depth > 0 implies penetration
 
 Compare bullet3 class btPersistentManifold
 
 see https://github.com/bulletphysics/bullet3 
 
*/


#ifndef __BMANIFOLD_H__
#include "BManifold.h"
#endif

#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // for glm::distance2/glm::length2

constexpr int BManifold::m_index[4][3] = 
{
    {1, 2, 3},
    {0, 2, 3},
    {0, 1, 3},
    {0, 1, 2}
};

void 
BManifold::refresh( BScalar max_dist, BScalar max_tan_dist2 ) 
// drop points if bodies separated beyond tolerance (max_dist or max_tan_dist)
// c.depth > 0 implies penetration
// c.normal is from body2 to body1
{
    for (int i = 0; i < m_points.size(); ) 
    {
        BContact &c = m_points[i];

        // convert body coords to world coords
        const BVector3 w1 = m_body1->X_base().applyTranspose(c.localPos1);
        const BVector3 w2 = m_body2->X_base().applyTranspose(c.localPos2);
 
        // check separation along the current contact normal
        // i)   positive (+): objects are separated
        // ii)  zero     (0): objects are exactly touching
        // iii) negative (-): objects are intersecting (penetrating)
        const BVector3 diff = w2 - w1;
        BScalar separation = arb::dot(diff, c.normal);

        if (separation > max_dist) 
        {
            // drop stale contact i.e. m_points.erase(m_points.begin() + i); 
            m_points[i] = m_points.back();
            m_points.pop_back(); 
        } 
        else 
        {
            // tangential drift check - has the contact slid too far sideways?
            const BVector3 tanDiff = diff - (separation * c.normal);
            BScalar tanDistSq = arb::length2(tanDiff);
  
            if (tanDistSq > max_tan_dist2)  
            {
                // drop stale contact i.e. m_points.erase(m_points.begin() + i); 
                m_points[i] = m_points.back();
                m_points.pop_back(); 
              
            }
            else
            {
                // Note we can get positive and negative separations here
                c.pos = (w1 + w2) * BScalar(0.5);
                c.depth = separation;
                ++i;
            }
        }
    }
}

void 
BManifold::addPoint( const BVector3 &cpoint, const BVector3 &cnormal, BScalar cdepth, BScalar threshold2 ) 
{
    // convert world to body coords
    const BVector3 local1 = m_body1->X_base().apply(cpoint);
    const BVector3 local2 = m_body2->X_base().apply(cpoint);

    // check for matching existing point (within say ~5mm)
    for (BContact &c : m_points) 
    {
        BScalar d1 = glm::distance2(c.localPos1, local1);
        if (d1 < threshold2) 
        {
            BScalar d2 = glm::distance2(c.localPos2, local2);
            if (d2 < threshold2)
            {
                // update geometric data while keeping accumulated impulses (accJ, accJx, accJy)
                c.localPos1 = local1;
                c.localPos2 = local2;
                c.pos       = cpoint;
                c.normal    = cnormal;
                c.depth     = cdepth;
    
                return;
            }
        }
    }

    // reduce to MAX_POINTS if needed
    if (m_points.size() < MAX_POINTS) 
    {
        m_points.emplace_back( local1, local2, cpoint, cnormal, cdepth );
    } 
    else 
    {
        reducePoints(BContact(local1, local2, cpoint, cnormal, cdepth));
    }
}


BScalar 
BManifold::calcArea(const BVector3& p0, const BVector3& p1, const BVector3& p2, const BVector3& p3) const
// calculate the area of four points -- adapted from persistent manifold class in Bullet
{
    // pack the vectors into matrices
    const BMatrix3 A(p0 - p1, p0 - p2, p0 - p3);
    const BMatrix3 B(p2 - p3, p1 - p3, p1 - p2);

    const BMatrix3 AT = glm::transpose(A);
    const BMatrix3 BT = glm::transpose(B);

    // compute the cross products for all 3 configurations simultaneously
    const BVector3 crossX = AT[1] * BT[2] - AT[2] * BT[1];
    const BVector3 crossY = AT[2] * BT[0] - AT[0] * BT[2];
    const BVector3 crossZ = AT[0] * BT[1] - AT[1] * BT[0];

    // compute the squared lengths of the 3 resulting vectors
    const BVector3 sq_lengths = (crossX * crossX) + (crossY * crossY) + (crossZ * crossZ);

    // return the maximum component
    return std::max({ sq_lengths[0], sq_lengths[1], sq_lengths[2] });
}

void
BManifold::reducePoints( const BContact& pt ) 
{
    // find and keep the point i0 with 'deepest' penetration 
    int maxDepthIdx = 0; 
    BScalar maxDepth = m_points[0].depth;
    for (int i = 1; i < m_points.size(); ++i) 
    {
        if (m_points[i].depth > maxDepth)  
        {
            maxDepth = m_points[i].depth;
            maxDepthIdx = i;
        }
    }
    
    int maxAreaIdx = -1;
    BScalar maxArea = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        BScalar res = 0.0;
        
        if (maxDepthIdx != i)
        {
            res = calcArea(pt.localPos1, 
                           m_points[m_index[i][0]].localPos1, 
                           m_points[m_index[i][1]].localPos1, 
                           m_points[m_index[i][2]].localPos1);
        }
        if (res > maxArea)
        {
            maxArea = res;
            maxAreaIdx = i;
        }
    }
    
    assert(maxAreaIdx != -1);
    
    // keep only the 4 chosen points (preserves impulse cache)
    std::vector<BContact> keptPoints(4);
    
    keptPoints[0] = pt;
    keptPoints[1] = m_points[m_index[maxAreaIdx][0]];
    keptPoints[2] = m_points[m_index[maxAreaIdx][1]];
    keptPoints[3] = m_points[m_index[maxAreaIdx][2]];
 
    m_points = keptPoints;
}

//

