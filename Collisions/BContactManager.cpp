/* BContactManager 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BContactManager.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 Two-Body Collisions, Friction
 
 Collision resolution consists of varying the colliding objects velocities by calculating and applying the 
 spatial impulse resulting from the collision (see RBDA, Section 11.7 Impulsive Dynamics, 
 Subsections: Two-Body Collisions, and Friction, pages 232-235).

 Consider two rigid bodies, body1 and body2, that collide at a contact point c.
 Their velocities before the impact are v1 and v2, and their velocities afterwards
 are v1 + ∆v1 and v2 + ∆v2. 
 The force of a collision, that is the impulse, is transmitted from Body1 to Body2, and can be
 expressed in the form $nλ$  where $n$ is the (spatial) contact normal from body2 to body1, 
 and λ a scalar impulse magnitude.
 

 
 Notes

 i)  an Orientated Bounding Box (OBB) is an Axis Aligned Bounding Box (AABB) with
     an accompanying 3D orientation matrix/quaternion.
 ii) For the special case where all the collider objects under consideration are perfect spheres
     you can replace:  
   
 iscollision = m_detector1.collision( b1, b2, c.depth, c.normal, c.pos ); // openGJHK 
 
 with
 
 {
    BVector3 diff = b2->pos() - b1->pos();
    BScalar dist  = arb::length(diff);
    c.depth       = (b1->box().extent().y + b2->box().extent().y) - dist;
    c.normal      = diff / dist;
    c.pos         = b1->pos() + (c.normal * BScalar(b1->box().extent().y)); // extent here is radius
 
    if (c.depth > 0)
    {
        iscollision = true;
    }
 }

 This also calculates the correct derivatives.
*/


#ifndef __BCONTACTMANAGER_H__
#include "BContactManager.h"
#endif


#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif

#ifndef __BPRODUCTS_H__
#include "BProducts.h"
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>          // for glm::distance2/glm::length2
#include <glm/gtc/matrix_access.hpp> // for glm::column


// All params have been set assuming 1 = 1m 
// if your scale differes significantly you may need to recalibrate/tune these parameters
BContactManager::BContactManager( int N ) : m_iters(25), 
                                            m_cachehits(0),
                                            m_e(0.75),
                                            m_mu(0.5), 
                                            m_baumgarte(0.2), 
                                            m_max_dist(0.01),        // manifold refresh - 1cm
                                            m_max_tan_dist2(2.0),    // manifold refresh
                                            m_threshold2(0.000025),  // manifold addPoint - 5mm  
                                            m_detector(),
                                            m_active(), 
                                            m_history(), 
                                            m_frictionOn(true)
                                           
{
    m_active.reserve(N);  
    m_history.reserve(N);
}

BContactManager::~BContactManager( void )
{
    //std::cout << "BContactManager::m_cachehits " << m_cachehits << std::endl;
}
 
// 
// Broad-phase - 3D algbra usimg float/double, no AD here
//

bool 
BContactManager::sat_obb( const glm::dmat3 &rotA, const glm::dvec3 &posA, const glm::dvec3 &extA,
                          const glm::dmat3 &rotB, const glm::dvec3 &posB, const glm::dvec3 &extB )
// Separating Axis Theorem (SAT) check for axis Orientated Bounding Boxes (OBB)
// in this case there are 15 axes to check. This is a glm impemenation of Randy Gaul's q3BoxtoBox 
{
    // B's frame in A's space
    const glm::dmat3 rotAT = glm::transpose(rotA);
    const glm::dmat3 C = rotAT * rotB;

    glm::dmat3 absC;
    bool parallel = false;
    const double kCosToll = 1.0E-6;
    
    for ( int i = 0; i < 3; ++i )
    {
        for ( int j = 0; j < 3; ++j )
        {
            absC[i][j] = std::fabs(C[i][j]);
            parallel = parallel || ((absC[i][j] + kCosToll) >= 1.0);
        }
    }

    // vector from center A to center B in A's space
    const glm::dvec3 t(rotAT * (posB - posA)); 

    // face normal axis checks
    for (int i = 0; i < 3; ++i) // A's axes 
    {
        //double s = std::fabs(t[i]) - (extA[i] + glm::dot(glm::row(absC, i), extB)); // slower
        const glm::dvec3 row(absC[0][i], absC[1][i], absC[2][i]);
        double s = std::fabs(t[i]) - (extA[i] + glm::dot(row, extB));
        
        if (s > 0.0)
            return false;
    }
    
    for (int i = 0; i < 3; ++i) // B's axes 
    {
        //double s = std::fabs(glm::dot(t, glm::column(C, i))) - (extB[i] + glm::dot(glm::column(absC, i), extA)); // slower
        double s = std::fabs(glm::dot(t, C[i])) - (extB[i] + glm::dot(absC[i], extA));

        if (s > 0.0)
            return false;
    }

    // edge cross products axis checks
    if ( !parallel )
    {
        for (int i = 0; i < 3; ++i) // A edge
        {
            int i1 = next(i);
            int i2 = prev(i);

            for (int j = 0; j < 3; ++j) // B edge
            {
                int j1 = next(j);
                int j2 = prev(j);

                // projected radii
                double rA = extA[i1] * absC[j][i2] + extA[i2] * absC[j][i1];
                double rB = extB[j1] * absC[j2][i] + extB[j2] * absC[j1][i];
                
                // separation
                double s = std::fabs(t[i2] * C[j][i1] - t[i1] * C[j][i2]) - (rA + rB);
                
                if (s > 0.0)
                    return false;
            }
        }
    }
    
    return true;
}


//
// Narrow-phase -- here we use spatial algebra (which is differentiable)
//
void 
BContactManager::compute_basis(const BVector3 &n, BVector3 &b1, BVector3 &b2)
// improved version of Frisvad's method in: 
// "Building an Orthonormal Basis, Revisted", Duff et al. JCGT, 2017. 
// the tangents remain geometrically consistent as the normal changes/body rotates
{ 
    //const BScalar sign = std::copysign(BScalar(1.0), n.z); // does not work with autodiff
    const BScalar sign = (n.z >= 0.0) ? 1.0 : -1.0;
    const BScalar a = -1.0 / (sign + n.z);
    const BScalar b = n.x * n.y * a;
    b1 = BVector3(1.0 + sign * n.x * n.x * a,  sign * b,  -sign * n.x);
    b2 = BVector3(b, sign + n.y * n.y * a, -n.y);
}

void 
BContactManager::prepare( BScalar dt ) 
// Impulse Dynamics, RBDA, Section 11.7, pages 232-235
// employs Baumgarte Stabilization Technique (BST) 
{
    const BScalar beta = (m_baumgarte / dt);
    const BScalar slop = 0.002; // 2mm to allow resting/stacking objects some 'space'

    for (BManifold &m : m_active)
    {
        m.refresh(m_max_dist, m_max_tan_dist2); // remove broken/stale points

        ABody *b1 = m.body1();
        ABody *b2 = m.body2();
        
        for (BContact &c : m.points()) 
        {
            // Two-Body Collisions, RBDA, page 232
            BVector3 rel_pos1 = c.pos - b1->pos();
            BVector3 rel_pos2 = c.pos - b2->pos();
            
            // the unit spatial impulse (force) transmitted from b1 to b2 along the contact normal
            c.n_1  = BVector6(arb::cross(rel_pos1, -c.normal), -c.normal);
            c.n_2  = BVector6(arb::cross(rel_pos2,  c.normal),  c.normal);
            
            // Δv_1, Δv_2 unit spatial motion vectors in world coords (eqns 11.60, 11.61)
            c.dv_1 = (b1->invI_base() * c.n_1); 
            c.dv_2 = (b2->invI_base() * c.n_2); 
            
            // effective mass - how 'heavy' the collision 'feels' (denominator of eqn 11.65)
            c.invK = BScalar(1.0) / (arb::dot( c.n_1, c.dv_1) + arb::dot(c.n_2, c.dv_2));
            
            // zeta ζ initial relative or separation velocity at contact c (eqn 11.62)
            // however unlike eqn 11.62 here n_1 != -n_2
            const BScalar relvel = arb::dot(c.n_2, b2->v()) + arb::dot(c.n_1, b1->v());
            
            // restitution bias: only apply if moving fast enough (prevents jitter)
            const BScalar restitution = (relvel < -0.5) ? m_e : 0.0; // 0.5 is a restitution threshold
            
            // arb::max because Baumgarte expects positive depth
            const BScalar baumgarte_bias = beta * arb::max(0.0, c.depth - slop); 
            
            // this is the total velocity change we want to achieve (see numerator of eqn 11.65)
            //c.velBias = arb::max((-restitution * relvel), baumgarte_bias);
            c.velBias = (-restitution * relvel) + baumgarte_bias;
            

            // Coulomb friction, RBDA, page 233
            if (m_frictionOn) 
            {
                BVector3 normx, normy;
                
                // create a coordinate frame from the normal
                // note c.n_1 and c.n_2 are considered to be nz_1 and nz_2
                compute_basis(c.normal, normx, normy );  
                
                // unit spatial impulse (force) vector - tangent plane (x-y axis) of contact space 
                c.nx_1 = BVector6(arb::cross(rel_pos1, -normx), -normx);
                c.nx_2 = BVector6(arb::cross(rel_pos2,  normx),  normx);
                
                c.ny_1 = BVector6(arb::cross(rel_pos1, -normy), -normy);
                c.ny_2 = BVector6(arb::cross(rel_pos2,  normy),  normy);
                
                // Δv_1, Δv_2 unit spatial motion vectors in world coords (x-y axis)
                c.dvx_1 = b1->invI_base() * c.nx_1;                 
                c.dvx_2 = b2->invI_base() * c.nx_2;  
                
                c.dvy_1 = b1->invI_base() * c.ny_1;                 
                c.dvy_2 = b2->invI_base() * c.ny_2;                 
                
                // compute effective mass for x-y axis (K_x and K_y)
                c.invK_x = BScalar(1.0) / (arb::dot(c.nx_1, c.dvx_1) + arb::dot(c.nx_2, c.dvx_2));
                c.invK_y = BScalar(1.0) / (arb::dot(c.ny_1, c.dvy_1) + arb::dot(c.ny_2, c.dvy_2)); 
            }

            //
            // warm start - apply the impulse from the previous frame
            // this is applied once, before solve iterations begin.
            //
            if (m.warmstart()) 
            {
                // apply old solution impulse 
                b1->v() += c.dv_1 * c.accJ;
                b2->v() += c.dv_2 * c.accJ;
                
                if (m_frictionOn) 
                {
                    // apply friction impulses (warm start)
                    b1->v() += c.dvx_1 * c.accJx;
                    b2->v() += c.dvx_2 * c.accJx;
                    
                    b1->v() += c.dvy_1 * c.accJy;
                    b2->v() += c.dvy_2 * c.accJy;
                }
                ++m_cachehits;
            }
        }
    }
}


void 
BContactManager::solve( void ) 
// Projected Gauss-Seidel (PGS) solver 
{
    using std::sqrt;
    
    for (int i = 0; i < m_iters; ++i) 
    {
        for (BManifold &m : m_active) 
        {
            ABody *b1 = m.body1();
            ABody *b2 = m.body2();
            
            for (BContact &c : m.points()) 
            {
                // current relative velocity (separation velocity at contact i)
                const BScalar relvel = arb::dot(c.n_2, b2->v()) + arb::dot(c.n_1, b1->v());

                // how much more impulse j do we need to reach the target bias?
                const BScalar j = (c.velBias - relvel) * c.invK;
                
                // clamping - Projected Gauss-Seidel (PGS)
                const BScalar oldJ = c.accJ;
                c.accJ = arb::max(0.0,  oldJ + j);
                const BScalar applyJ = c.accJ - oldJ; 
                
                // apply spatial impulse
                b1->v() += c.dv_1 * applyJ;
                b2->v() += c.dv_2 * applyJ;
                
                if (m_frictionOn)
                {
                    // Coulomb friction, RBDA, page 233 
                    // solve a few "normal-only" iterations before starting friction iterations
                    if (i > 3)
                    {
                        // calculate desired incremental impulses for both tangents
                        const BScalar dvx = arb::dot(c.nx_2, b2->v()) + arb::dot(c.nx_1, b1->v());
                        const BScalar dvy = arb::dot(c.ny_2, b2->v()) + arb::dot(c.ny_1, b1->v());
                        
                        // calculate the new total accumulated friction vector
                        BScalar next_accJx = c.accJx + (-dvx * c.invK_x);
                        BScalar next_accJy = c.accJy + (-dvy * c.invK_y);
                        
                        // circular clamping (friction cone)
                        const BScalar maxFriction   = m_mu * c.accJ;
                        const BScalar maxFrictionSq = maxFriction * maxFriction;
                        const BScalar magnitudeSq   = (next_accJx * next_accJx) + (next_accJy * next_accJy);
                        
                        if (magnitudeSq > maxFrictionSq) 
                        {
                            // we only do this for contacts that are 'sliding'
                            // as we wish to avoid expensive sqrt()
                            assert(magnitudeSq > 0);
                            
                            const BScalar scale = maxFriction / sqrt(magnitudeSq);
                            next_accJx *= scale;
                            next_accJy *= scale;
                        }
                        
                        // calculate the actual impulse to apply this iteration
                        const BScalar applyJx = next_accJx - c.accJx;
                        const BScalar applyJy = next_accJy - c.accJy;
                        
                        // update accumulated state
                        c.accJx = next_accJx;
                        c.accJy = next_accJy;
                        
                        // apply spatial impulses
                        b1->v() += c.dvx_1 * applyJx + c.dvy_1 * applyJy;
                        b2->v() += c.dvx_2 * applyJx + c.dvy_2 * applyJy;
                    }
                }
            }
        }
    }
}


void
BContactManager::narrow_check(ABody *b1, ABody *b2, long now)
// narrow-phase GJK check
{
    BVector3 cpos;       // world contact position
    BVector3 cnormal;    // world contact normal
    BScalar  cdepth;     // penetration contact depth

    // GJK - a positive depth indicates a penetration
    bool iscollision = m_detector.collision( b1, b2, cdepth, cnormal, cpos ); 
    //
    
    if (iscollision)
    {
        //b1->addMsg(BMsg(BMsg::COLLISION, b2, b1, now));
        //b2->addMsg(BMsg(BMsg::COLLISION, b1, b2, now ));
        
        // find or create a manifold for this contact
        uint64_t hash =  myhash(b1->objId(), b2->objId());
        auto fidx = m_history.find(hash);
        
        if (fidx != m_history.end())
        {
            m_active.push_back(fidx->second);
            m_active.back().warmstart(true);
        }
        else
        {
            m_active.emplace_back(b1, b2, hash);
            //m_active.back().warmstart(false);
        }

        m_active.back().addPoint(cpos, cnormal, cdepth, m_threshold2);
    }
}

int
BContactManager::detect( const std::vector<ABody*> &body, const std::vector<ABody*> &fixed )
// collision detection: broad-phase then narrow-phase
// check body and fixed objects against one another for collisions
{
    using std::abs;
    
    const BScalar pad = 0.1;
    const long now = 0 ;// theClock->seconds();

    m_active.clear();
    
    // dynamic objects
    for (int i = 0; i < body.size(); ++i)
    {
        ABody *b1 = body[i];
        BScalar radius1 = b1->box().radius(); 

        for (int j = i+1; j < body.size(); ++j)    
        {
            ABody *b2 = body[j];
            BScalar radius2 = b2->box().radius(); 
            BScalar d2 = glm::distance2(b1->pos(), b2->pos());
            
            // proximity check
            // if (d2 < range1_sq)  b1->addMsg(BMsg(BMsg::NEAR, b2));
            // if (d2 < range2_sq)  b2->addMsg(BMsg(BMsg::NEAR, b1));
            
            // sphere intersection check
            // bool sphere_check = d < (pad + (radius1 + radius2)); 
            bool sphere_check = d2 < ((pad + radius1 + radius2) * (pad + radius1 + radius2));
           
            if (sphere_check) 
            {
                // broad-phase SAT for OBB check 
                bool box_check = broad_check(b1, b2);
             
                if (box_check) 
                {
                    // narrow-phase GJK check 
                    narrow_check(b1, b2, now);
                }
            } 
        }
    } 
    
    // fixed/static objects 
    for (int i = 0; i < body.size(); ++i)
    {
        ABody* b1 = body[i];
        BScalar radius1 = b1->box().radius(); 

        for (int j = 0; j < fixed.size(); ++j)
        {
            ABody* b2 = fixed[j];
            BScalar radius2 = b2->box().radius(); 
            BScalar d2 = glm::distance2(b1->pos(), b2->pos());
            
            // proximity check
            // if (d2 < range1_sq)  b1->addMsg(BMsg(BMsg::NEAR, b2));

            // sphere intersection check
            // bool sphere_check = d < (pad + (radius1 + radius2)); 
            bool sphere_check = d2 < ((pad + radius1 + radius2) * (pad + radius1 + radius2));
           
            if (sphere_check) 
            {
                // broad-phase SAT for OBB check 
                bool box_check = broad_check(b1, b2);
             
                if (box_check) 
                {
                    // narrow-phase GJK check 
                    narrow_check(b1, b2, now);
                }
            }
        }
    }
    
    return (int) m_active.size(); // the number of collisions detected
}

void 
BContactManager::cache( void )
{
    m_history.clear();
    for (const BManifold &m : m_active)
    {
        m_history[m.manifoldId()] = m;
    }
}

//
// main entry point to BContactManager -- return number of collisions, update body_list
//

int
BContactManager::resolve( double dt, const std::vector<ABody*> &body, const std::vector<ABody*> &fixed)
{
    if (body.empty())
        return 0;
 
    if (body.size() == 1 && fixed.empty())
        return 0;
    
    // broad-phase then narrow-phase detection
    int num_contacts = detect(body, fixed);
    
    if (num_contacts) 
    {
        // solver phase - prepare contacts, solve them, store useful results
        prepare(dt);
        solve();
    }
    
    cache(); 

    
    return num_contacts;
}

//


