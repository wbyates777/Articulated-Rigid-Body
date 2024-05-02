/* BSpatialInertia 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialInertia.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$$
 
 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/

#include <stdio.h>


#ifndef __BSPATIALINERTIA_H__
#include "BSpatialInertia.h"
#endif


#ifndef __BBODY_H__
#include "BBody.h"
#endif


BSpatialInertia::BSpatialInertia( const BBody &b ): m_h(b.com() * b.mass()), m_mass(b.mass()) 
// this was the RBDL function static createFromMassComInertiaC()
{
    BMatrix3 ccom = arb::cross_matrix(b.com());
    BMatrix3 inertia = b.inertia() + ccom * glm::transpose(ccom) * b.mass();

    m_Ixx = inertia[0][0];
    m_Iyx = inertia[1][0]; m_Iyy = inertia[1][1];
    m_Izx = inertia[2][0]; m_Izy = inertia[2][1]; m_Izz = inertia[2][2]; 
}


