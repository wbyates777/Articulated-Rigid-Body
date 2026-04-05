/* BContactManager 17/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BContactManager.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 RBDA, chapter 11, Contact and Impact. 
 Section 11.7 - Two-Body Collisions, Friction

 Consider two rigid bodies, body1 and body2, that collide at a single contact point c.
 Their velocities before the impact are v1 and v2, and their velocities afterwards
 are v1 + ∆v1 and v2 + ∆v2. 
 
 
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
 
 Note an Orientated Bounding Box (OBB) is an Axis Aligned Bounding Box (AABB) with
 an accompanying 3D orientation matrix/quaternion.
 
*/


#ifndef __BCONTACTMANAGER_H__
#include "BContactManager.h"
#endif


#include <glm/gtc/matrix_access.hpp> // for glm::column

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>  // for glm::distance2/glm::length2



BContactManager::BContactManager( int N ) : m_timestamp(0), 
                                            m_iters(20),
                                            m_e(0.75),
                                            m_mu(0.5), 
                                            m_cachehits(0), 
                                            m_frictionOn(true), 
                                            m_active(), 
                                            m_history(), 
                                            m_gjk() 
{
    m_active.reserve(N);  
    m_history.reserve(N);
}

BContactManager::~BContactManager( void )
{
    std::cout << "BContactManager::m_cachehits " << m_cachehits << std::endl;
}

//
// Broad-phase - 3D algbra usimg float/double, no AD here
//

bool 
BContactManager::sat_obb( const glm::dmat3 &rotA, const glm::dvec3 &posA, const glm::dvec3 &extA,
                          const glm::dmat3 &rotB, const glm::dvec3 &posB, const glm::dvec3 &extB) const
// Separating Axis Theorem (SAT) check for axis Orientated Bounding Boxes (OBB)
// in this case there are 15 axes to check
{
    // B's frame in A's space
    glm::dmat3 C = glm::transpose(rotA) * rotB;

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
    glm::dvec3 t(rotB * (posB - posA));

    // face normal axis checks
    for (int i = 0; i < 3; ++i) // A's axes 
    {
        double s = std::fabs(t[i]) - (extA[i] + glm::dot(glm::column(absC, i), extB));
        if (s > 0.0)
            return false;
    }
    
    for (int i = 0; i < 3; ++i) // B's axes 
    {
        double s = std::fabs(glm::dot(t, glm::row(C, i))) -  (extB[i] + glm::dot(glm::row(absC, i), extA));
        if (s > 0.0)
            return false;
    }

    if ( !parallel )
    {
        // edge cross products axis checks
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
    const BScalar beta = (0.2 / dt);
    const BScalar slop = 0.001;

    for (BContact &c : m_active)
    {
        ABody *b1 = c.body1;
        ABody *b2 = c.body2;
        
        // 
        // Two-Body Collisions, RBDA, page 232
        //
        BVector3 rel_pos1 = c.pos - b1->pos();
        BVector3 rel_pos2 = c.pos - b2->pos();
        
        // the unit spatial impulse (force) transmitted from b1 to b2 along the contact normal
        c.n_1 = BVector6(arb::cross(rel_pos1, -c.norm), -c.norm);
        c.n_2 = BVector6(arb::cross(rel_pos2,  c.norm),  c.norm);
        
        // Δv_1, Δv_2 unit spatial motion vectors in world coords (eqns 11.60, 11.61)
        c.dv_1 = (b1->invI_base() * c.n_1); 
        c.dv_2 = (b2->invI_base() * c.n_2); 
        
        // effective mass - how 'heavy' the collision 'feels' (denominator of (11.65))
        c.invK  = BScalar(1.0) / (arb::dot( c.n_1, c.dv_1) + arb::dot(c.n_2, c.dv_2));
 
        // ζ initial relative or separation velocity at contact c (eqn 11.62)
        const BScalar n_dot_relvel = arb::dot(c.n_2, b2->v() - b1->v()); 
        
        // restitution bias: only apply if moving fast enough (prevents jitter)
        const BScalar e = (n_dot_relvel < -0.5) ? m_e : 0.0; // 0.5 is a restitution threshold
        
        const BScalar baumgarte_bias = beta * arb::max(0.0, c.depth - slop); 
        
        // this is the total velocity change we want to achieve (numerator of (11.65))
        c.velBias = (-(1.0 + e) * n_dot_relvel) + baumgarte_bias;
     
        // prevent 'explosive ejection' 
        c.velBias = arb::clamp(c.velBias, -5.0, 5.0);
    
        //
        // Coulomb friction, RBDA, page 233
        //
        if (m_frictionOn) // unnecessary branching
        {
            BVector3 normx, normy;
            
            // create a coordinate frame from the normal
            // note c.n_1 and c.n_2 are considered to be nz_1 and nz_2
            compute_basis(c.norm, normy, normx );  // sic. order is correct
            
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
            c.invK_x = BScalar(1.0) / (arb::dot(c.nx_1, c.dvx_1) +  arb::dot(c.nx_2, c.dvx_2));
            c.invK_y = BScalar(1.0) / (arb::dot(c.ny_1, c.dvy_1) +  arb::dot(c.ny_2, c.dvy_2)); 
        }
        //
        
        //
        // warm start - apply the impulse from the previous frame
        // this is applied once, before solve iterations begin.
        //
        auto fidx = m_history.find(c.contactId);
        if (fidx != m_history.end()) 
        {
            BContact &oldc = fidx->second;

            BScalar d2 = glm::distance2(c.pos, oldc.pos); 
            if (d2 < 0.005)  // if same point - note distance2
            {
                // apply old solution impulse 
                b1->v() += c.dv_1 * oldc.accJ;
                b2->v() += c.dv_2 * oldc.accJ;
                
                if (m_frictionOn)
                {
                    // apply friction impulses (warm start)
                    b1->v() += c.dvx_1 * oldc.accJx;
                    b2->v() += c.dvx_2 * oldc.accJx;
                    
                    b1->v() += c.dvy_1 * oldc.accJy;
                    b2->v() += c.dvy_2 * oldc.accJy;
                }
                m_cachehits++;
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
        for (BContact &c : m_active) 
        {
            ABody *b1 = c.body1;
            ABody *b2 = c.body2;
            
            // current relative velocity (separation velocity at contact i)
            const BScalar n_dot_dvel = arb::dot(c.n_2, c.body2->v()) + arb::dot(c.n_1, c.body1->v());
            
            // how much more impulse j do we need to reach the target bias?
            const BScalar j = (c.velBias - n_dot_dvel) * c.invK;
            
            // clamping - Projected Gauss-Seidel (PGS)
            BScalar oldJ = c.accJ;
            c.accJ = arb::max(0.0,  oldJ + j);
            BScalar applyJ = c.accJ - oldJ; 
            
            // apply spatial impulse
            b1->v() += c.dv_1 * applyJ;
            b2->v() += c.dv_2 * applyJ;
            
            //
            // Coulomb friction, RBDA, page 233 
            //
            if (m_frictionOn) // unnecessary branching
            {
                // solve a few "normal-only" iterations or ensure the normal is 
                // already solved before starting friction iterations.
                if (i > 3 || arb::nearZero(applyJ, 1E-2)) 
                {
                    // calculate desired incremental impulses for both tangents
                    const BScalar dvx = arb::dot(c.nx_2, b2->v()) + arb::dot(c.nx_1, b1->v());
                    const BScalar dvy = arb::dot(c.ny_2, b2->v()) + arb::dot(c.ny_1, b1->v());
                    
                    // calculate the new total accumulated friction vector
                    BScalar next_accJx = c.accJx + (-dvx * c.invK_x);
                    BScalar next_accJy = c.accJy + (-dvy * c.invK_y);
                    
                    // circular clamping (friction cone)
                    BScalar maxFriction   = m_mu * c.accJ;
                    BScalar maxFrictionSq = maxFriction * maxFriction;
                    BScalar magnitudeSq   = (next_accJx * next_accJx) + (next_accJy * next_accJy);
                    
                    if (magnitudeSq > maxFrictionSq) 
                    {
                        // we only do this for contacts that are 'sliding'
                        // as we wish to avoid expensive sqrt()
                        assert(magnitudeSq > 0);
                        
                        BScalar scale = maxFriction / sqrt(magnitudeSq);
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

int
BContactManager::detect( const std::vector<ABody*> &body )
// collision detection: broad-phase then narrow-phase
{
    m_active.clear();
    
    for (int i = 0; i < body.size(); ++i)
    {
        ABody *b1 = body[i];

        //BScalar  range1_sq  = 20.0 * 20.0; // b1->range2();
        BScalar radius1    = b1->box().radius(); 

        for (int j = i+1; j < body.size(); ++j)    
        {
           ABody *b2 = body[j];
            
           if (b1 == b2) 
               continue;
            
            //BScalar  range2_sq  = 20.0 * 20.0; //b2->range2();
            BScalar radius2    = b2->box().radius(); 
              
            BScalar d2 = glm::distance2(b1->pos(), b2->pos());
            
            // proximity check
            // if (d2 < range1_sq)
            //    b1->addMsg(BMsg(BMsg::NEAR, b2));

            // if (d2 < range2_sq)
            //    b2->addMsg(BMsg(BMsg::NEAR, b1));
            
            // sphere intersection check
            // bool sphere_check = d < (1.0 + (radius1 + radius2)); 
            bool sphere_check = d2 < ((1.0 + radius1 + radius2) * (1.0 + radius1 + radius2));
           
            if (sphere_check) 
            {
                // broad-phase SAT for OBB check 
                bool box_check = intersect(b1, b2);
       
                if (box_check) 
                {
                    // narrow-phase GJK check 
                    BContact c(b1, b2);
        
                    if (m_gjk.collision( c ))
                    {
                        // if bodies collide
                        //b1->addMsg(BMsg(BMsg::COLLISON, b2));
                        //b2->addMsg(BMsg(BMsg::COLLISON, b1));
                        
                        c.contactId = myhash(b1->objId(), b2->objId()); // must be done here
                        m_active.push_back(c);
                    }
                }
            } 
        }
    } 
    
    return (int) m_active.size();
}

void 
BContactManager::cache( void )
{
    // persist for 3 frames
    int next_time = (m_timestamp + 1) % 3;
    int prev_time = (m_timestamp + 2) % 3;
    
    for (BContact &c : m_active)
    {
        if (c.accJ > 0.0)
        {
            c.timestamp = next_time;  // with new/next time stamp 
            m_history[c.contactId] = c;
        }
    }

    for ( auto i = m_history.begin(); i != m_history.end(); )
    {
        i = (i->second.timestamp == prev_time) ? m_history.erase(i) : ++i;
    }
    
    ++m_timestamp;
}

int
BContactManager::resolve( double dt, const std::vector<ABody*> &body_list )
{
    if (body_list.empty())
        return 0;
 
    // broad-phase then narrow-phase detection
    int num_contacts = detect(body_list);
    
    if (num_contacts) 
    {
        // solver phase - prepare contacts, solve them, store useful results
        prepare(dt);
        solve();
        cache(); 
    }
    
    return num_contacts;
}

