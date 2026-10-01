/* BKinematics 13/09/2026

 $$$$$$$$$$$$$$$$$$$$$
 $   BKinematics.h   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Kinematics is the study of the geometrical aspects of motion independent of forces. 
 Kinematics differs from dynamics (also known as kinetics) 
 which studies the effect of forces on bodies.
 
 The BKinematics class operates on the kinematic variables: (position, 
 velocity, and acceleration) of a given model.
 
 Methods taken from RBDL Kinematics.cc
 


*/


#ifndef BKINEMATICS_H
#define BKINEMATICS_H


#ifndef BMODELSTATE_H
#include "BModelState.h"
#endif

#ifndef BMODEL_H
#include "BModel.h"
#endif

#ifndef AMATRIX_H
#include "AMatrix.h"
#endif

#include <vector>


class BKinematics
{

public:

    BKinematics( void )=default;
    explicit BKinematics( int expected_dof ) { m_qdot_zero.reserve(expected_dof); }
    ~BKinematics( void )=default;

    // update kinematics - calculates positions (X_base, pos, orient)
    void 
    update_X_base( BModel &m, const BModelState &qstate );

    // update kinematics - calculates velocities (v and c)
    void 
    update_velocity( BModel &m, const BModelState &qstate );
    
    
    // velocity at point
    BVector6  
    v( const BModel &m, BBodyId bid, const BVector3 &body_pos ) const;
    
    // acceleration at point 
    BVector6
    a( const BModel &m, BBodyId bid, const BVector3 &body_pos ) const;

    
    // return base coordinates of body_pos where body_pos is expressed in body $bid$ coordinates 
    BVector3 
    toBasePos( const BModel &m, BBodyId bid, const BVector3 &body_pos = B_ZERO_3 ) const;
    
    // return body $bid$ coordinates of base_pos where base_pos is expressed in base/world coordinates 
    BVector3  
    toBodyPos( const BModel &m, BBodyId bid,  const BVector3 &base_pos ) const;

    // return orientation of body $bid$
    BMatrix3 
    orient( const BModel &m, BBodyId bid ) const;

    /** @brief Computes a 6-D Jacobian for a point on a body
     *
     * Computes the 6-D Jacobian \f$G(q)\f$ that when multiplied with
     * \f$\dot{q}\f$ gives a 6-D vector that has the angular velocity as the
     * first three entries and the linear velocity as the last three entries.
     *
     * @param m   rigid body model
     * @param qstate state of the internal joints (positions, velocities, accelerations)
     * @param bid the id of the body
     * @param point_pos the position of the point in body-local data
     * @param G       a matrix of dimensions 6 x \#qdot_size where the result will be stored in
     * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
     *
     * The result will be returned via the G argument.
     *
     * @note This function only evaluates the entries of G that are non-zero. One
     * Before calling this function one has to ensure that all other values
     * have been set to zero, e.g. by calling G.setZero().
     *
     */
    void 
    calcPointJacobian( BModel &m, const BModelState &qstate, BBodyId bid,
                       const BVector3 &point_pos, BMatrix &G, bool update_kinematics );

private:


    void
    block_6_1(BMatrix& G, int dof_index_i, int dof_index_j, const BVector6 &val) const
    {
        for ( int i = 0; i < 6; ++i )
            G[dof_index_i + i][dof_index_j] = val[i];
    }
    
    void
    block_6_3(BMatrix& G, int dof_index_i, int dof_index_j, const BMatrix63 &val) const
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                G[i + dof_index_i][j + dof_index_j] = val[i][j];
    }


    std::vector<BScalar> m_qdot_zero;
};

#endif


