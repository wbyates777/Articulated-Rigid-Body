/* BContactManager 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BContactManager.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A simple impulse based collision detection and resolution system.
 This code is not a full blown, production ready, collision resolution system 
 such as Jolt or Bullet. Insetad it is intended as a demonstration of spatial impulse. 
 
 
 The BContactManager detects collisions between pairs of objects and resolves any 'contacts' by employing 
 the physical concept of impulse to vary an object's velocity following a collision
 See https://en.wikipedia.org/wiki/Physics_engine
 
 The BContactManager performs _broad phase_ and _narrow phase_ collison detection.
 
 Broad-phase consists of detecting intersections between orientated bounding boxes (OBB) using 
 the Separating Axis Theorem (SAT). Computationally, this is relatively efficient.
 See https://en.wikipedia.org/wiki/Hyperplane_separation_theorem
 
 Narrow-phase consists of detecting intersections between mesh polytopes/convex hulls 
 using the GJK and EPA algorithms. This is very precise but computationally expensive.
 See https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm

 Collision resolution consists of varying the colliding objects velocities by calculating and applying the 
 spatial impulse resulting from the collision (see RBDA, Section 11.7 Impulsive Dynamics, 
 Subsections: Two-Body Collisions, and Friction, pages 232-235).
 The force of a collision, that is the impulse, is transmitted from Body1 to Body2, and can be
 expressed in the form $nλ$  where $n$ is the (spatial) contact normal from body2 to body1, 
 and λ a scalar impulse magnitude.
 
 Friction is represented by Coulomb's Friction Model which is the simplest physical model for dry friction. 
 Coulomb's friction law relates the (spatial) impulse to two (spatial) tangential, frictional forces. 

 Multiple point contacts are resolved simultaneously by a Projected Gauss-Seidel (PGS) solver. 
 PGS is an iterative method used to solve Linear Complementary Problems (LCPs) (see RBDA, Section 11.5). 
 It extends the traditional Gauss-Seidel method by incorporating projections to ensure that 
 specific constraints (such as 'no penetration between rigid bodies') are met.
 
 
 Coefficient of Restitution
 ------------------------------
 The coefficient of restitution (COR) is denoted e and when:
 e = 0     : a perfectly inelastic collision; objects do not rebound at all and end up touching.
 0 < e < 1 : an inelastic collision, in which some kinetic energy is dissipated. 
             The objects rebound with a lower separation speed than the speed of approach.
 e = 1     : a perfectly elastic collision, in which no kinetic energy is dissipated. 
             The objects rebound with the same relative speed with which they approached.
 See https://en.wikipedia.org/wiki/Coefficient_of_restitution 
 

 Coefficient of Friction
 ------------------------------
 The coefficient of friction is denoted μ and when:
 μ ≈ 0     : near-frictionless (like ice on ice),
 μ ≈ 0.5   : typical wood or plastic contact,
 μ ≈ 1.0   : very grippy (like rubber on dry asphalt),
 μ > 1.0   : extremely high friction (like racing tires or adhesives).
 See  https://en.wikipedia.org/wiki/Friction 
 
 
 Other Constants
 ------------------------------
 i)   m_iters - PGS solver iterations - 20 is good, 25 is plenty 
 ii)  m_baumgarte - Baumgarte stabilization factor [0.15, 0.2] are reasonable values
 ii)  m_max_dist and m_max_tan_dist2 - used by BManifold::refresh() to 'drop' contact points if the contact bodies are 
      separated beyond the given tolerance.
 iii) m_threshold2 - used by BManifold::addPoint() to determine if two contact points are equal

 
 Notes: 
 1) Consider a 3D force f applied to a rigid body at some 3D point p, then the corresponding 
    spatial force (wrench) is $F = (p \times f, f)$, Modern Robotics, page 108, eqn 3.93 
 2) we work in base/world coordinates (positions, velocities, inertias) as we are dealing with multiple bodies 
 3) contacts are differentiable (AD).
 4) see https://github.com/jslee02/awesome-collision-detection for a detailed list of useful resources 

 
*/


#ifndef __BCONTACTMANAGER_H__
#define __BCONTACTMANAGER_H__

#ifndef __ABODY_H__
#include "ABody.h"
#endif

#ifndef __BBOX_H__
#include "BBox.h"
#endif

#ifndef __BMANIFOLD_H__
#include "BManifold.h"
#endif

#ifndef __BGJK_H__
#include "BGJK.h"
#endif

#include <unordered_map>


class BContactManager
{

public:

    // number of expected bodies N is used to reserve space
    explicit BContactManager( int N = 32 );  
    ~BContactManager( void );
    
    void
    clear( void ) { m_active.clear(); m_history.clear(); }
        
    // return number of collisions; update velocities of bodies accordingly
    int
    resolve( double dt, const std::vector<ABody*> &body, const std::vector<ABody*> &fixed = std::vector<ABody*>() );

    
    void
    setIters( int num ) { m_iters = num; }
    
    int
    getIters( void ) const { return (int) m_iters; }

    // 
    // coefficients  μ, e, and Baumgarte
    //
    void
    frictionOn( bool b ) { m_frictionOn = b; }
    
    bool
    frictionOn( void ) const { return m_frictionOn; }
    
    
    void
    setFriction( BScalar m ) { assert(m_mu >= 0); m_mu = m; }

    BScalar
    getFriction( void ) const { return m_mu; }
    
    void
    setRestitution( BScalar e ) { assert(e >= 0 && e <= 1.0); m_e = e; }
    
    BScalar
    getRestitution( void ) const { return m_e; }
    
    void
    setBaumgarte( BScalar b ) { m_baumgarte = b; }
    
    BScalar
    getBaumgarte( void ) const { return m_baumgarte; }
    

private:  
   
    // used in sat_obb
    static int next(uint64_t i) { return (i + 1) % 3; }
    static int prev(uint64_t i) { return (i + 2) % 3; }
    
    static uint64_t
    myhash( BBodyId bid1, BBodyId bid2 )
    { 
        // WARNING: this depends on BIdType being uint32_t and unique
        return ((uint64_t(std::min(bid1, bid2)) << 32) | uint64_t(std::max(bid1, bid2))); 
        //return ((uint64_t(bid1) << 32) | uint64_t(bid2)); // if order is important
    }
    
    static void 
    compute_basis(const BVector3 &n, BVector3 &b1, BVector3 &b2 );
    
    static bool 
    sat_obb( const glm::dmat3 &rotA, const glm::dvec3 &posA, const glm::dvec3 &extA,
             const glm::dmat3 &rotB, const glm::dvec3 &posB, const glm::dvec3 &extB);
    
    bool
    broad_check( const ABody *b1, const ABody *b2 ) const // SAT
    {
        return sat_obb(b1->orient(), b1->pos() + b1->box().pos(), b1->box().extent(), 
                       b2->orient(), b2->pos() + b2->box().pos(), b2->box().extent());
    }
    
    void
    narrow_check(ABody *b1, ABody *b2, long now); // GJK/EPA
    
    int
    detect( const std::vector<ABody*> &body, const std::vector<ABody*> &fixed  );
    
    void 
    prepare( BScalar dt );
    
    void 
    solve( void );
    
    void 
    cache( void );
    

    uint64_t m_iters;        // PGS solver iterations 25 is plenty - 20 is OK
    uint64_t m_cachehits;
    
    BScalar m_e;             // coefficient of restitution, denoted e
    BScalar m_mu;            // coefficient of friction, denoted μ
    BScalar m_baumgarte;     // coefficient of Baumgarte stabilization 
 
    BScalar m_max_dist;      // see BManifold::refresh 
    BScalar m_max_tan_dist2; // see BManifold::refresh
    BScalar m_threshold2;    // see BManifold::addPoint
    
    BGJK  m_detector; 
    
    std::vector<BManifold> m_active;
    std::unordered_map<uint64_t, BManifold> m_history; 
    
    bool m_frictionOn;
};

#endif


