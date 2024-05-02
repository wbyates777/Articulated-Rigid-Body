/* BBody 20/02/2024

 $$$$$$$$$$$$$$$$$
 $   BBody.cpp   $
 $$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/


#ifndef __BBODY_H__
#include "BBody.h"
#endif


#ifndef __BSPATIALINERTIA_H__
#include "BSpatialInertia.h"
#endif

#include <iostream>

#include <sstream>

/** \brief Describes all properties of a single body
 *
 * A Body contains information about mass, the location of its center of
 * mass, and the ineria tensor in the center of mass. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 */


/** \brief Constructs a body from mass, center of mass and radii of gyration
 *
 * This constructor eases the construction of a new body as all the
 * required parameters can be specified as parameters to the
 * constructor. These are then used to generate the spatial inertia
 * matrix which is expressed at the origin.
 *
 * \param mass the mass of the body
 * \param com  the position of the center of mass in the bodies coordinates
 * \param gyration_radii the radii of gyration at the center of mass of the body
 */
BBody::BBody( BScalar mass,
              const BVector3 &com,
              const BVector3 &gyration_radii,
              bool isVirtual ) : m_X_base(BIDENTITY_3x3, BZERO_3), 
                                 m_I(0.0, BZERO_3, BIDENTITY_3x3),
                                 m_vel(BZERO_6), 
                                 m_acc(BZERO_6),
                                 m_inertia(BIDENTITY_3x3),
                                 m_com(com),
                                 m_mass(mass),
                                 m_isVirtual(isVirtual)
{
    m_inertia = BMatrix3( gyration_radii[0], 0.0, 0.0,
                         0.0, gyration_radii[1], 0.0,
                         0.0, 0.0, gyration_radii[2] );
}

/** \brief Constructs a body from mass, center of mass, and a 3x3 inertia matrix
 *
 * This constructor eases the construction of a new body as all the
 * required parameters can simply be specified as parameters to the
 * constructor. These are then used to generate the spatial inertia
 * matrix which is expressed at the origin.
 *
 * \param mass the mass of the body
 * \param com  the position of the center of mass in the bodies coordinates
 * \param inertia_C the inertia at the center of mass
 */
BBody::BBody( BScalar mass,
              const BVector3 &com,
              const BMatrix3 &inertia_C,
              bool isVirtual) : m_X_base(BIDENTITY_3x3, BZERO_3), 
                                 m_I(0.0, BZERO_3, BIDENTITY_3x3),
                                 m_vel(BZERO_6), 
                                 m_acc(BZERO_6), 
                                 m_inertia(inertia_C),
                                 m_com(com),
                                 m_mass(mass),
                                 m_isVirtual(isVirtual) {}


void
BBody::init( void )
{
    m_X_base = BSpatialTransform(BIDENTITY_3x3, BZERO_3); 
    m_I = BSpatialInertia(0.0, BZERO_3, BIDENTITY_3x3);
    m_vel = BSpatialVector(BZERO_6); 
    m_acc = BSpatialVector(BZERO_6);
    m_inertia = BMatrix3(BIDENTITY_3x3);
    m_com = BVector3(BZERO_3);
    m_mass = 0.0;
    m_isVirtual = false;
}

void
BBody::set( BScalar mass, const BVector3 &com, const BMatrix3 &inertia_C, bool isVirtual )
{
    m_X_base   = BSpatialTransform(BIDENTITY_3x3, BZERO_3); 
    m_I        = BSpatialInertia(0.0, BZERO_3, BIDENTITY_3x3);
    m_vel      = BZERO_6; 
    m_acc      = BZERO_6; 
    m_inertia  = inertia_C;
    m_com      = com;
    m_mass     = mass;
    m_isVirtual = isVirtual;
}


const BMatrix3 
BBody::parallelAxis( const BMatrix3 &inertia, BScalar mass, const BVector3 &com ) const 
{
    BMatrix3 com_cross(arb::cross_matrix(com));
    return inertia + mass * com_cross * glm::transpose(com_cross);
}


/** \brief Joins inertial parameters of two bodies to create a composite
 * body.
 *
 * This function can be used to joint inertial parameters of two bodies
 * to create a composite body that has the inertial properties as if the
 * two bodies were joined by a fixed joint.
 *
 * \param transform The frame transformation from the current body to the
 * other body.
 * \param other_body The other body that will be merged with *this.
 */
void 
BBody::join(const BSpatialTransform &transform, const BBody &other_body)
{
    // nothing to do if we join a massles body to the current.
    if (other_body.m_mass == 0.0 && other_body.m_inertia == BZERO_3x3)
    {
        return;
    }

    BScalar other_mass = other_body.m_mass;
    BScalar new_mass = m_mass + other_mass;
    
    if (new_mass == 0.0) 
    {
        std::cout << "Error join: cannot join bodies as both have zero mass!\n" << std::endl;
        exit(EXIT_FAILURE); 
    }

    BVector3 other_com(glm::transpose(transform.E()) * other_body.m_com + transform.r());
    BVector3 new_com((BScalar(1.0) / new_mass) * (m_mass * m_com + other_mass * other_com));
    
    // We have to transform the inertia of other_body to the new COM. This
    // is done in 4 steps:
    //
    // 1. Transform the inertia from other origin to other COM
    // 2. Rotate the inertia that it is aligned to the frame of this body
    // 3. Transform inertia of other_body to the origin of the frame of
    // this body
    // 4. Sum the two inertias
    // 5. Transform the summed inertia to the new COM
    
    BSpatialInertia rbi(*this);

    
    // inertia_other_com_rotated_this_origin
    BMatrix3 aux = inertiaToBodyFrame(transform, other_body);
    
    // 4. Sum the two inertias
    BMatrix3 inertia_summed = rbi.toMatrix().topLeft() + aux;

    // 5. Transform the summed inertia to the new COM
    BMatrix3 new_inertia = inertia_summed - new_mass * arb::cross_matrix(new_com) * glm::transpose(arb::cross_matrix(new_com));

    set(new_mass, new_com, new_inertia);
}


const BMatrix3 
BBody::inertiaToBodyFrame( const BSpatialTransform &transform, const BBody &body ) const
{
    BVector3 com = glm::transpose(transform.E()) * body.m_com + transform.r();
    
    BSpatialInertia rbi(body);    
    
    BMatrix3 inertia_initial = rbi.toMatrix().topLeft();
 
    // 1. Transform the inertia from initial origin to initial COM
    BMatrix3 com_cross = arb::cross_matrix(body.m_com);
    BMatrix3 inertia_com = inertia_initial - body.m_mass * com_cross * glm::transpose(com_cross);
 
    // 2. Rotate the inertia that it is aligned to the frame of this body
    BMatrix3 inertia_com_rot = glm::transpose(transform.E()) * inertia_com * transform.E();

    // 3. Transform inertia of body to the origin of the frame of the target body
    BMatrix3 inertia_com_rot_this_origin = parallelAxis(inertia_com_rot, body.m_mass, com);

    return inertia_com_rot_this_origin;
}


/**
 * @brief Separate inertial parameters of two bodies that were creating a composite
 * body.
 *
 * This function can be used to Separate inertial parameters of two bodies
 * that were previously joined as a composite body.
 *
 * \param trans The frame transformation from the current body to the
 * other body.
 * \param other_body The other body that will be separated with *this.
 */
void 
BBody::separate(const BSpatialTransform &trans, const BBody &other_body)
// untested
{
    // nothing to do if we join a massles body to the current.
    if (other_body.m_mass == 0.0 && other_body.m_inertia == BZERO_3x3)
    {
        return;
    }
    
    BScalar other_mass = other_body.m_mass;
    BScalar new_mass = m_mass - other_mass;
    
    if (new_mass == 0.0)
    {
        std::cout << "Error: cannot separate bodies as both have zero mass!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    BVector3 other_com = glm::transpose(trans.E()) * other_body.m_com + trans.r();
    BVector3 new_com = (BScalar(1.0) / new_mass) * (m_mass * m_com - other_mass * other_com);
  
    // We have to transform the inertia of other_body to current COM (before separation).
    // This is done in 4 steps:
    // 1. Transform the inertia from other origin to other COM
    // 2. Rotate the inertia that it is aligned to the frame of this body
    // 3. Transform inertia of other_body to the origin of the frame of
    // this body
    // 4. Substract the two inertias
    // 5. Transform the new inertia to the new COM
    
    BSpatialInertia rbi(*this);
    
    BMatrix3 inertia_other_com_rot_this_origin = inertiaToBodyFrame(trans, other_body);
    
    // 4. Substract the two inertias
    BMatrix3 inertia_substracted = BMatrix3(rbi.toMatrix().topLeft()) - inertia_other_com_rot_this_origin;

    // 5. Transform the summed inertia to the new COM
    BMatrix3 new_inertia = inertia_substracted - new_mass * arb::cross_matrix(new_com) * glm::transpose(arb::cross_matrix(new_com));
    
    set(new_mass, new_com, new_inertia);
}

