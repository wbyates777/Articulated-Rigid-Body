/* BContact 26/12/2025

 $$$$$$$$$$$$$$$$$$
 $   BContact.h   $
 $$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Record the details of a contact or collision between two objects: body1 and body2
 The contact information (position, normal and depth) is computed by the GJK/EPA algorithms.
 
 The remaining variables are the results needed by the solver that can either be calculated 
 once at contact creation or need to be preserved between frames.
 
 Note all varaibles are differentiable using AD
 
 
*/


#ifndef __BCONTACT_H__
#define __BCONTACT_H__

#ifndef __BVECTOR6_H__
#include "BVector6.h"
#endif


class ABody;


struct BContact
{
    BContact( void ) = default;
 
    BContact( ABody *a, ABody *b ): body1(a), 
                                    body2(b),
                                    contactId(0), 
                                    timestamp(0) {}
    
    ~BContact(void) = default;
    
    
    ABody    *body1;
    ABody    *body2;
    
    uint64_t contactId;    
    uint64_t timestamp;          // m_history pruning   

    BVector3 pos;                // computed 3D contact position c in world coords
    BVector3 norm;               // computed 3D contact normal n
    
    //
    // spatial impulse (norm is z-axis)
    //
    BVector6 n_1;                // unit spatial impulse - force vector along contact normal
    BVector6 n_2;                // unit spatial impulse - force vector along contact -normal
    
    BVector6 dv_1;               // Δv_1 unit change in spatial velocity - motion vector
    BVector6 dv_2;               // Δv_2 unit change in spatial velocity - motion vector
    
    BScalar  accJ;               // accumulated impulse 
    BScalar  K;                  // 'effective' mass 
    BScalar  depth;              // computed  penetration depth (always positive)
    BScalar  velBias;            // total "target" change in velocity
    
    //
    // Coulomb friction  (tangents x-y axis)
    //
    BVector6 nx_1;   // unit spatial impulses x-axis
    BVector6 nx_2; 
    
    BVector6 ny_1;   // unit spatial impulses y-axis
    BVector6 ny_2; 
    
    BVector6 dvx_1;  // unit change in spatial velocity x-axis
    BVector6 dvx_2;  
    
    BVector6 dvy_1;  // unit change in spatial velocity y-axis
    BVector6 dvy_2; 
    
    BScalar  accJx;  // accumulated impulses (x-y axis)
    BScalar  accJy;
    
    BScalar  K_x;    // 'effective' masses (x-y axis) 
    BScalar  K_y;
    
};


#endif

