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


#ifndef __BKINEMATICS_H__
#define __BKINEMATICS_H__


#ifndef __BMODELSTATE_H__
#include "BModelState.h"
#endif

#ifndef __BMODEL_H__
#include "BModel.h"
#endif

#ifndef __AMATRIX_H__
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
    v( BModel &m, BBodyId bid, const BVector3 &body_pos );
    
    // acceleration at point 
    BVector6
    a( BModel &m, BBodyId bid, const BVector3 &body_pos );

    
    // return base coordinates of body_pos where body_pos is expressed in body $bid$ coordinates 
    BVector3 
    toBasePos( BModel &m, BBodyId bid, const BVector3 &body_pos = B_ZERO_3 );
    
    // return body $bid$ coordinates of base_pos where base_pos is expressed in base/world coordinates 
    BVector3  
    toBodyPos( BModel &m, BBodyId bid,  const BVector3 &base_pos );

    // return orientation of body $bid$
    BMatrix3 
    orient( BModel &m, BBodyId bid );
 
    void 
    calcPointJacobian( BModel &m, const BModelState &qstate, BBodyId bid,
                       const BVector3 &point_pos, BMatrix &G, bool update_kinematics );

private:


    void
    block(BMatrix& G, int dof_index_i, int dof_index_j, const BVector6 &val)
    {
        for ( int i = 0; i < 6; ++i )
            G[dof_index_i + i][dof_index_j] = val[i];
    }
    
    void
    block(BMatrix& G, int dof_index_i, int dof_index_j, const BMatrix63 &val)
    {
        for ( int i = 0; i < 6; ++i )
            for ( int j = 0; j < 3; ++j )
                G[i + dof_index_i][j + dof_index_j] = val[i][j];
    }


    std::vector<BScalar> m_qdot_zero;
};

#endif


