/* BContactManager 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$
 $   BContactManager.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A simple impulse based collision detection and resolution system.
 
 Detect collisions between pairs of objects and resolve any 'contacts' by employing 
 the physical concept of impulse to vary an object's velocity following a collision
 See https://en.wikipedia.org/wiki/Physics_engine
 
 
 Performs _broad phase_ and _narrow phase_ collison detection.
 
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
 expressed in the form $nλ$  where $n$ is the (spatial) contact normal, and λ a scalar impulse magnitude.
 
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
    
    // return number of collisions; update velocities of bodies accordingly
    int
    resolve( double dt, const std::vector<ABody*> &body_list );
    
    
    void
    setIters( int num ) { m_iters = num; }
    
    int
    getIters( void ) const { return (int) m_iters; }

    // 
    // coefficients  μ and e
    //
    void
    setFriction( bool state ) { m_frictionOn = state; }
    
    void
    setFriction( BScalar m ) { m_mu = m; }

    BScalar
    getFriction( void ) const { return m_mu; }
    
    void
    setRestitution( BScalar e ) { m_e = e; }
    
    BScalar
    getRestitution( void ) const { return m_e; }
    
    
private:  
   
    // used in sat_obb
    static int next(uint64_t i) { return (i + 1) % 3; }
    static int prev(uint64_t i) { return (i + 2) % 3; }
    
    static uint64_t
    myhash( BBodyId bid1, BBodyId bid2 )
    { 
        // WARNING: this depends on BIdType being uint32_t
        return ((uint64_t(std::min(bid1, bid2)) << 32) | uint64_t(std::max(bid1, bid2))); 
    }
    
    static void 
    compute_basis(const BVector3 &n, BVector3 &b1, BVector3 &b2 );
    
    bool
    intersect( const ABody *b1, const ABody *b2 ) const
    // check if orientated bounding boxes intersect
    {
        return sat_obb(b1->orient(), b1->pos() + b1->box().pos(), b1->box().extent(), 
                       b2->orient(), b2->pos() + b2->box().pos(), b2->box().extent());
    }
    
    bool 
    sat_obb( const glm::dmat3 &rotA, const glm::dvec3 &posA, const glm::dvec3 &extA,
             const glm::dmat3 &rotB, const glm::dvec3 &posB, const glm::dvec3 &extB) const;
    
    int
    detect( const std::vector<ABody*> &body );
    
    void 
    prepare( BScalar dt );
    
    void 
    solve( void );
    
    void 
    cache( void );
    

    uint64_t m_timestamp;    // keep track of different frames
    uint64_t m_iters;        // PGS solver iterations 20 is plenty - 15 is OK
    
    BScalar m_e;             // coefficient of restitution, denoted e
    BScalar m_mu;            // coefficient of friction, denoted μ

    int     m_cachehits;
    bool    m_frictionOn;    // test code - remove for production

    std::vector<BContact>        m_active;
    std::unordered_map<uint64_t, BContact> m_history; 
    BGJK                         m_gjk;  

};

#endif


