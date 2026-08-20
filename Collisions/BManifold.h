/* BManifold 23/07/2026

 $$$$$$$$$$$$$$$$$$$
 $   BManifold.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Assume depth > 0 implies penetration
 
 Compare bullet3 class btPersistentManifold
 see https://github.com/bulletphysics/bullet3 
 
*/


#ifndef __BMANIFOLD_H__
#define __BMANIFOLD_H__


#ifndef __BVECTOR6_H__
#include "BVector6.h"
#endif

#include <vector>


class ABody;

// a single point of contact
struct BContact 
{
    BContact( void )=default;
    BContact( const BVector3 &lp1, const BVector3 &lp2,
              const BVector3 &p, const BVector3 &n, BScalar d) : localPos1(lp1), localPos2(lp2), 
                                                                 pos(p), normal(n), depth(d), 
                                                                 accJ(0.0), accJx(0.0), accJy(0.0) {}
    ~BContact( void )=default;
    
    BVector3 localPos1; // position relative to body1 frame
    BVector3 localPos2; // position relative to body2 frame
    BVector3 pos;       // world contact position
    BVector3 normal;    // world contact normal
    BScalar  depth;     // penetration depth
    
    // persistent solver state (for warm starting)
    BScalar accJ, accJx, accJy;

    // kinematic helper properties calculated during prepare()
    BScalar velBias, invK, invK_x, invK_y;
    BVector6 n_1, n_2, dv_1, dv_2;
    BVector6 nx_1, nx_2, dvx_1, dvx_2;
    BVector6 ny_1, ny_2, dvy_1, dvy_2;
};

// a collection of upto 4 points of contact
class BManifold 
{
    
public:
    
    BManifold( void )=default;
    BManifold( ABody *a, ABody *b, uint64_t idx ): m_body1(a), m_body2(b), m_manifoldId(idx), m_warmstart(false)
    {
        m_points.reserve(MAX_POINTS+1);
    }
    ~BManifold( void )=default;

    
    uint64_t
    manifoldId( void ) const { return  m_manifoldId; }
    
    ABody*
    body1( void ) { return m_body1; }

    ABody*
    body2( void ) { return m_body2; }
    
    const std::vector<BContact>&
    points( void ) const { return m_points; }
    
    std::vector<BContact>&
    points( void )  { return m_points; }
    
    bool
    warmstart( void ) const { return m_warmstart; }
    
    void
    warmstart( bool b )  { m_warmstart = b; }  
    
    // updates world positions and removes stale contacts that have drifted apart
    void 
    refresh( BScalar max_dist, BScalar max_tan_dist2 );
    
    // add a new point and reduces the set to the 4 best points
    void 
    addPoint( const BVector3 &cpoint, const BVector3 &cnormal, BScalar cdepth, BScalar threshold2 );


private:
    
    BScalar 
    calcArea(const BVector3& p0, const BVector3& p1, const BVector3& p2, const BVector3& p3) const;
    
    // remove excess points from the manifold
    void
    reducePoints( const BContact& pt );
    
    ABody* m_body1;
    ABody* m_body2;
    uint64_t m_manifoldId;
    std::vector<BContact> m_points;
    bool m_warmstart;
    
    static constexpr int MAX_POINTS = 4; // 4 points form a stable 3D polygon
    static const int m_index[4][3];
};



#endif


