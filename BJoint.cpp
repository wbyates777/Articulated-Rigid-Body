/* BJoint 31/01/2024
 
 $$$$$$$$$$$$$$$$$$
 $   BJoint.cpp   $
 $$$$$$$$$$$$$$$$$$
 
 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 */


#ifndef __BJOINT_H__
#include "BJoint.h"
#endif




#define GLM_ENABLE_EXPERIMENTAL
#include <glm/geometric.hpp>       // for glm::length 
#include <glm/gtx/quaternion.hpp>  // for toMat3


const std::vector<std::vector<BScalar>> BJoint::m_ZERO_6x3(6, {0.0, 0.0, 0.0});
const std::vector<std::vector<BScalar>> BJoint::m_ZERO_1x6(1, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
const std::vector<std::vector<BScalar>> BJoint::m_ZERO_6x6(6, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});


BJoint::BJoint(BJointType joint_type) : m_id(0),
                                        m_qidx(0),
                                        m_widx(0),
                                        m_jtype(joint_type),
                                        m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                        m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                        m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                        m_v_J(B_ZERO_6),
                                        m_c_J(B_ZERO_6),
                                        m_S(m_ZERO_1x6),
                                        m_jointAxes(1)
{
    
    if (m_jtype == BJointType::BRevoluteX) 
    {
        m_jointAxes[0] = BSpatialVector(B_XAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BRevoluteY) 
    {
        m_jointAxes[0] = BSpatialVector(B_YAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BRevoluteZ) 
    {
        m_jointAxes[0] = BSpatialVector(B_ZAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BSpherical) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_ZAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_YAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_XAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BEulerZYX) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_ZAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_YAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_XAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BEulerXYZ) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_XAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_YAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_ZAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BEulerYXZ) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_YAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_XAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_ZAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BEulerZXY) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_ZAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_XAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_YAXIS, B_ZERO_3);
    } 
    else if (m_jtype == BJointType::BTranslationXYZ) 
    {
        m_jointAxes.resize(3);
        m_jointAxes[0] = BSpatialVector(B_XAXIS, B_ZERO_3);
        m_jointAxes[1] = BSpatialVector(B_YAXIS, B_ZERO_3);
        m_jointAxes[2] = BSpatialVector(B_ZAXIS, B_ZERO_3);
    } 
    else if (m_jtype >= BJointType::B1DoF && m_jtype <= BJointType::B6DoF) 
    {
        // create a joint and allocate memory for it.
        int DoF_count = joint_type - B1DoF + 1; 
        m_jointAxes.resize(DoF_count, B_ZERO_6); 
        std::cout << "Warning: zero vector " << m_jointAxes[0] << std::endl;
    }  
    else if (m_jtype != BJointType::BFixed && m_jtype != BJointType::BFloatingBase) 
    {
        std::cout << "Error: Invalid use of Joint constructor Joint(BJointType type). Only allowed when type == BFixed or BSpherical." << std::endl;
        exit(EXIT_FAILURE);
    }
    
    if (!m_jointAxes.empty())
    {
        setJointSpace(m_jointAxes[0]);
        m_v_J = m_jointAxes[0];
    }
}


// only joint_type  BRevolute or BPrismatic
BJoint::BJoint( BJointType joint_type, const BVector3 &joint_axis ) : m_id(0),
                                                                      m_qidx(0),
                                                                      m_widx(0),
                                                                      m_jtype(joint_type),
                                                                      m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                                      m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                                      m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                                      m_v_J(B_ZERO_6),
                                                                      m_c_J(B_ZERO_6),
                                                                      m_S(m_ZERO_1x6),
                                                                      m_jointAxes(1)
{
    // Only rotation around the Z-axis
    assert( joint_type == BJointType::BRevolute || joint_type == BJointType::BPrismatic );
    
    if (m_jtype == BJointType::BRevolute) 
    {
        // make sure we have a unit axis
        // assert (joint_axis.length() - 1.0 >  BSMALL_VALUE);
        m_jointAxes[0].set( joint_axis, B_ZERO_3  );
    } 
    else if (m_jtype == BJointType::BPrismatic) 
    {
        // make sure we have a unit axis
        // assert (joint_axis.length() - 1.0 >  BSMALL_VALUE);
        m_jointAxes[0].set( B_ZERO_3, joint_axis );
    }
    
    if (!m_jointAxes.empty())
    {
        setJointSpace(m_jointAxes[0]);
        m_v_J = m_jointAxes[0];
    }
}

/** \brief Constructs a 1 DoF joint with the given motion subspaces.
 *
 * The motion subspaces are of the format:
 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
 *
 * \param axis_0 Motion subspace for axis 0
 */
BJoint::BJoint( const BSpatialVector &axis_0 ): m_id(0),
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes(1, axis_0)
{
    setJointSpace(m_jointAxes[0]);
    m_v_J = m_jointAxes[0];
    
    // TODO: this has to be properly determined AND test case. Try Matt's dot product idea
    if (axis_0 == BSpatialVector(B_XAXIS, B_ZERO_3)) 
    {
        m_jtype = BJointType::BRevoluteX;
    } 
    else if (axis_0 == BSpatialVector(B_YAXIS, B_ZERO_3)) 
    {
        m_jtype = BJointType::BRevoluteY;
    } 
    else if (axis_0 == BSpatialVector(B_ZAXIS, B_ZERO_3)) 
    {
        m_jtype = BJointType::BRevoluteZ;
    } 
    else if (axis_0.ang() == B_ZERO_3) 
    {
        m_jtype = BJointType::BPrismatic;
    } 
    else 
    {
        m_jtype = BJointType::BHelical;
    }
    
    validate_spatial_axis(m_jointAxes[0]);  
}

BJoint::BJoint( const BSpatialVector &axis_0, 
                const BSpatialVector &axis_1) : m_id(0),
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes()
{
    setJoint({axis_0, axis_1}); 
}

BJoint::BJoint( const BSpatialVector &axis_0,
                const BSpatialVector &axis_1,
                const BSpatialVector &axis_2 ): m_id(0), 
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes()
{
    setJoint({axis_0, axis_1, axis_2}); 
}

BJoint::BJoint( const BSpatialVector &axis_0,
                const BSpatialVector &axis_1,
                const BSpatialVector &axis_2,
                const BSpatialVector &axis_3 ): m_id(0),
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes()
{
    setJoint({axis_0, axis_1, axis_2, axis_3}); 
}

BJoint::BJoint( const BSpatialVector &axis_0,
                const BSpatialVector &axis_1,
                const BSpatialVector &axis_2,
                const BSpatialVector &axis_3,
                const BSpatialVector &axis_4 ): m_id(0),
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes()
{
    setJoint({axis_0, axis_1, axis_2, axis_3, axis_4}); 
}

BJoint::BJoint( const BSpatialVector &axis_0,
                const BSpatialVector &axis_1,
                const BSpatialVector &axis_2,
                const BSpatialVector &axis_3,
                const BSpatialVector &axis_4,
                const BSpatialVector &axis_5 ): m_id(0),
                                                m_qidx(0),
                                                m_widx(0),
                                                m_jtype(BJointType::BUNDEFINED),
                                                m_X_lambda(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_J(B_IDENTITY_3x3, B_ZERO_3),
                                                m_X_T(B_IDENTITY_3x3, B_ZERO_3),
                                                m_v_J(B_ZERO_6),
                                                m_c_J(B_ZERO_6),
                                                m_S(m_ZERO_1x6),
                                                m_jointAxes()
{
    setJoint({axis_0, axis_1, axis_2, axis_3, axis_4, axis_5}); 
}

void
BJoint::setJoint( const std::vector<BSpatialVector> &axes )
// Constructs a N=2,..,6 DoF joint with the given motion subspaces.
{
    int DoF_count = (int) axes.size();
    
    m_jointAxes = axes;
    
    // emulated 2-6 DoF joint.
    switch (DoF_count)
    {
        case 2 : m_jtype = BJointType::B2DoF; break;
        case 3 : m_jtype = BJointType::B3DoF; break; 
        case 4 : m_jtype = BJointType::B4DoF; break; 
        case 5 : m_jtype = BJointType::B5DoF; break; 
        case 6 : m_jtype = BJointType::B6DoF; break; 
        default: exit(EXIT_FAILURE); break;
    }

    for (int i = 0; i < DoF_count; ++i)
    {
        validate_spatial_axis(m_jointAxes[i]);
    }
}


/** \brief Checks whether we have pure rotational or translational axis.
 *
 * This function is mainly used to print out warnings when specifying an
 * axis that might not be intended.
 */
bool 
BJoint::validate_spatial_axis( const BSpatialVector &axis )
{
    bool axis_rot = false;
    bool axis_trans = false;
    
    BVector3 rot(axis.ang());
    BVector3 trans(axis.lin());

    const BScalar SMALL_VALUE = 1E-8;
    
    BScalar rot_len   = glm::length(rot);
    BScalar trans_len = glm::length(trans);
    
    if (rot_len > SMALL_VALUE) 
    {
        axis_rot = true;
    }
    
    if (trans_len > SMALL_VALUE) 
    {
        axis_trans = true;
    }
    
    if (axis_rot && rot_len - 1.0 > SMALL_VALUE) 
    {
        std::cout << "Warning: joint rotation axis is not unit!" << std::endl;
    }
    
    if (axis_trans && trans_len - 1.0 > SMALL_VALUE) 
    {
        std::cout << "Warning: joint translation axis is not unit! " << std::endl;
    }
    
    return axis_rot != axis_trans; 
}

void
BJoint::clear( void )
{
    m_qidx = 0;
    m_widx = 0;
    m_jtype = BJointType::BUNDEFINED;
    m_X_lambda.clear();
    m_X_J.clear();
    m_X_T.clear();
    m_v_J = B_ZERO_6;
    m_c_J = B_ZERO_6;
    m_S   = m_ZERO_1x6; // 1 DoF
    
    m_jointAxes.clear();
}    

BQuat
BJoint::getQuat(const std::vector<BScalar> &q) const
{
    assert(m_jtype == BJointType::BSpherical);
    
    return BQuat(q[m_widx], q[m_qidx], q[m_qidx + 1], q[m_qidx + 2]);
}

void 
BJoint::SetQuat(const BQuat &quat, std::vector<BScalar> &q) const
{
    assert(m_jtype == BJointType::BSpherical);

    q[m_qidx]     = quat[1];
    q[m_qidx + 1] = quat[2];
    q[m_qidx + 2] = quat[3];
    q[m_widx]     = quat[0];
}

void
BJoint::setJointSpace( const BSpatialVector &v )
{
    m_S = m_ZERO_1x6;
    for (int i = 0; i < 6; ++i)
        m_S[0][i] = v[i];
}

void
BJoint::setJointSpace( const BMatrix63 &m )
{
    m_S = m_ZERO_6x3;
    const std::array<std::array<BScalar, 3>, 6>& d(m.data());
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j)
            m_S[i][j] = d[i][j];
}

void
BJoint::setJointSpace( const BSpatialMatrix &m )
{
    m_S = m_ZERO_6x6;
    const std::array<std::array<BScalar, 6>, 6>& d(m.data());
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            m_S[i][j] = d[i][j];
}


/** \brief Computes all variables for a joint model
 *
 *  By appropriate modification of this function all types of joints can be
 *  modeled. See RBDA Section 4.4 for details.
 *
 * \param q        joint state variables
 * \param qdot     joint velocity variables
 */
void 
BJoint::jcalc( const std::vector<BScalar> &q, const std::vector<BScalar> &qdot ) 
// calculate  $\[ X_lambda, X_J, S_i, v_J, c_J \] = jcalc(jtype(i), q, qdot, i)$ 
{
    if (m_jtype == BJointType::BRevoluteX)  // 1 DoF 
    {
        BScalar s = std::sin(q[m_qidx]);
        BScalar c = std::cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( E[0],
                                c * E[1] + s * E[2],
                               -s * E[1] + c * E[2] ));
        
        m_X_lambda.r( m_X_T.r() );
        
        m_v_J[0] = qdot[m_qidx];
    } 
    else if (m_jtype == BJointType::BRevoluteY) // 1 DoF 
    {
        BScalar s = std::sin(q[m_qidx]);
        BScalar c = std::cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( c * E[0] + -s * E[2],
                                E[1],
                                s * E[0] + c *  E[2] ));
        
        m_X_lambda.r( m_X_T.r() );
        
        m_v_J[1] = qdot[m_qidx];
    } 
    else if (m_jtype == BJointType::BRevoluteZ) // 1 DoF
    {
        BScalar s = std::sin(q[m_qidx]);
        BScalar c = std::cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( c * E[0] + s * E[1],
                               -s * E[0] + c * E[1],
                                E[2] ));

        m_X_lambda.r( m_X_T.r() );
        m_v_J[2] = qdot[m_qidx];
    } 
    else if (m_jtype == BJointType::BRevolute) // 1 DoF
    {
        m_X_J = arb::Xrot(q[m_qidx], m_jointAxes[0].ang());
   
        // Note m_S should be non-zero here
        BSpatialVector MS(m_S);
        
        m_v_J = MS * qdot[m_qidx]; 
        m_X_lambda = m_X_J * m_X_T;
    }
    else if (m_jtype == BJointType::BPrismatic) // 1 DoF
    {
        m_X_J = arb::Xtrans(m_jointAxes[0].lin() * q[m_qidx]);

        // Note m_S should be non-zero here
        BSpatialVector MS(m_S);
        
        m_v_J = MS * qdot[m_qidx]; 
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == BJointType::BHelical) // 1 DoF
    {
        BVector3 ang(m_jointAxes[0].ang());
        BVector3 lin(m_jointAxes[0].lin());
        
        BSpatialTransform jrot   = arb::Xrot(q[m_qidx], ang);
        BSpatialTransform jtrans = arb::Xtrans(lin * q[m_qidx]);
        m_X_J = jrot * jtrans;
        
        BVector3 strans = glm::transpose(m_X_J.E()) * lin; 
        BSpatialVector MS(ang, strans);
        
        BScalar Jqd = qdot[m_qidx];
        m_v_J = MS * Jqd;

        BVector3 c = glm::cross(ang, strans); 
        c *= -Jqd * Jqd;
        m_c_J = BSpatialVector(B_ZERO_3, c);
        
        m_X_lambda = m_X_J * m_X_T;

        setJointSpace(MS);
    } 
    else if (m_jtype == BJointType::BSpherical) // 3 DoF
    {
        m_X_J = BSpatialTransform(glm::toMat3(getQuat(q)));
        m_v_J = BSpatialVector( BVector3(qdot[m_qidx], qdot[m_qidx+1], qdot[m_qidx+2]), B_ZERO_3 );
       
        m_X_lambda = m_X_J * m_X_T;
        
        m_S = m_ZERO_6x3;
        
        m_S[0][0] = 1.0;
        m_S[1][1] = 1.0;
        m_S[2][2] = 1.0;
    } 
    else if (m_jtype == BJointType::BEulerZYX)  // 3 DoF
    {
        BScalar q0 = q[m_qidx];
        BScalar q1 = q[m_qidx + 1];
        BScalar q2 = q[m_qidx + 2];
        
        BScalar s0 = std::sin(q0);
        BScalar c0 = std::cos(q0);
        BScalar s1 = std::sin(q1);
        BScalar c1 = std::cos(q1);
        BScalar s2 = std::sin(q2);
        BScalar c2 = std::cos(q2);
        
        m_X_J = BSpatialTransform(BMatrix3( c0 * c1, s0 * c1, -s1,
                                            c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
                                            c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2 ));
        
        m_S = m_ZERO_6x3;
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0][0] = MS[0][0] = -s1;
        m_S[0][2] = MS[0][2] = 1.0;
        
        m_S[1][0] = MS[1][0] = c1 * s2;
        m_S[1][1] = MS[1][1] = c2;
        
        m_S[2][0] = MS[2][0] = c1 * c2;
        m_S[2][1] = MS[2][1] = - s2;
        
        BScalar qdot0 = qdot[m_qidx];
        BScalar qdot1 = qdot[m_qidx + 1];
        BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);

        m_c_J.set(-c1*qdot0*qdot1,
                  -s1*s2*qdot0*qdot1 + c1*c2*qdot0*qdot2 - s2*qdot1*qdot2,
                  -s1*c2*qdot0*qdot1 - c1*s2*qdot0*qdot2 - c2*qdot1*qdot2,
                   0.0, 0.0, 0.0);
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == BJointType::BEulerXYZ) // 3 DoF
    {
        BScalar q0 = q[m_qidx];
        BScalar q1 = q[m_qidx + 1];
        BScalar q2 = q[m_qidx + 2];
        
        BScalar s0 = std::sin(q0);
        BScalar c0 = std::cos(q0);
        BScalar s1 = std::sin(q1);
        BScalar c1 = std::cos(q1);
        BScalar s2 = std::sin(q2);
        BScalar c2 = std::cos(q2);
        
        m_X_J = BSpatialTransform(BMatrix3( c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
                                           -s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
                                            s1, -c1 * s0, c1 * c0 ));
        
        m_S = m_ZERO_6x3;
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0][0] = MS[0][0] = c2 * c1;
        m_S[0][1] = MS[0][1] = s2;
        
        m_S[1][0] = MS[1][0] = -s2 * c1;
        m_S[1][1] = MS[1][1] = c2;
        
        m_S[2][0] = MS[2][0] = s1;
        m_S[2][2] = MS[2][2] = 1.0;
        
        BScalar qdot0 = qdot[m_qidx];
        BScalar qdot1 = qdot[m_qidx + 1];
        BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set(-s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 + c2*qdot2*qdot1,
                  -c2*c1*qdot2*qdot0 + s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
                   c1*qdot1*qdot0,
                   0.0, 0.0, 0.0);
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == BJointType::BEulerYXZ) // 3 DoF
    {
        BScalar q0 = q[m_qidx];
        BScalar q1 = q[m_qidx + 1];
        BScalar q2 = q[m_qidx + 2];

        BScalar s0 = std::sin(q0);
        BScalar c0 = std::cos(q0);
        BScalar s1 = std::sin(q1);
        BScalar c1 = std::cos(q1);
        BScalar s2 = std::sin(q2);
        BScalar c2 = std::cos(q2);
        
        m_X_J = BSpatialTransform(BMatrix3( c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
                                           -s2 * c0 + c2 * s1 * s0, c2 * c1,  s2 * s0 + c2 * s1 * c0,
                                            c1 * s0, -s1, c1 * c0 )); 
        
        m_S = m_ZERO_6x3;
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0][0] = MS[0][0] = s2 * c1;
        m_S[0][1] = MS[0][1] = c2;
        
        m_S[1][0] = MS[1][0] = c2 * c1;
        m_S[1][1] = MS[1][1] = -s2;
        
        m_S[2][0] = MS[2][0] = -s1;
        m_S[2][2] = MS[2][2] = 1.0;
        
        BScalar qdot0 = qdot[m_qidx];
        BScalar qdot1 = qdot[m_qidx + 1];
        BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set( c2*c1*qdot2*qdot0 - s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
                 -s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 - c2*qdot2*qdot1,
                 -c1*qdot1*qdot0,
                 0.0, 0.0, 0.0 );
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == BJointType::BEulerZXY) // 3 DoF 
    {
        BScalar q0 = q[m_qidx];
        BScalar q1 = q[m_qidx + 1];
        BScalar q2 = q[m_qidx + 2];
        
        BScalar s0 = std::sin(q0);
        BScalar c0 = std::cos(q0);
        BScalar s1 = std::sin(q1);
        BScalar c1 = std::cos(q1);
        BScalar s2 = std::sin(q2);
        BScalar c2 = std::cos(q2);
        
        m_X_lambda = BSpatialTransform( BMatrix3( -s0 * s1 * s2 + c0 * c2, s0 * c2 + s1 * s2 * c0, -s2 * c1,
                                                -s0 * c1, c0 * c1, s1,
                                                s0 * s1 * c2 + s2 * c0, s0 * s2 - s1 * c0 * c2, c1 * c2 )) * m_X_T;
        
        m_S = m_ZERO_6x3;
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0][0] = MS[0][0] = -s2 * c1;
        m_S[0][1] = MS[0][1] = c2;
        
        m_S[1][0] = MS[1][0] = s1;
        m_S[1][2] = MS[1][2] = 1;
        
        m_S[2][0] = MS[2][0] = c1 * c2;
        m_S[2][1] = MS[2][1] = s2;
        
        BScalar qdot0 = qdot[m_qidx];
        BScalar qdot1 = qdot[m_qidx + 1];
        BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set((-c1 * c2 * qdot2 + s1 * s2 * qdot1) * qdot0 - s2 * qdot1 * qdot2,
                  c1 * qdot1 * qdot0,
                  (-s1 * c2 * qdot1 - c1 * s2 * qdot2) * qdot0 + c2 * qdot2 * qdot1,
                  0.0, 0.0, 0.0);
    } 
    else if (m_jtype == BJointType::BTranslationXYZ)  // 3 DoF
    {
        m_S = m_ZERO_6x3;
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[3][0] = MS[3][0] = 1.0;
        m_S[4][1] = MS[4][1] = 1.0;
        m_S[5][2] = MS[5][2] = 1.0;
        
        m_v_J = MS * BVector3(qdot[m_qidx], qdot[m_qidx + 1], qdot[m_qidx + 2]);
        
        m_c_J = B_ZERO_6;
        
        m_X_lambda.E( m_X_T.E() );
        m_X_lambda.r( m_X_T.r() + glm::transpose(m_X_T.E()) * BVector3(q[m_qidx], q[m_qidx + 1], q[m_qidx + 2]));
    } 
    else 
    {
        std::cout << "Error jcalc: invalid joint type " << m_jtype << std::endl;
        exit(EXIT_FAILURE);
    }
}


std::string 
BJoint::toString( BJointType jt )
{
    switch (jt)
    {
        case BJointType::BUNDEFINED:      return "Undefined"; break;
            
        case BJointType::BPrismatic:      return "Prismatic"; break;
        case BJointType::BRevolute:       return "Revolute"; break;
        case BJointType::BRevoluteX:      return "RevoluteX"; break;
        case BJointType::BRevoluteY:      return "RevoluteY"; break;
        case BJointType::BRevoluteZ:      return "RevoluteZ"; break;
 
        case BJointType::BSpherical:      return "Spherical"; break;
        case BJointType::BEulerZYX:       return "EulerZYX"; break;
        case BJointType::BEulerXYZ:       return "EulerXYZ"; break;
        case BJointType::BEulerYXZ:       return "EulerYXZ"; break;
        case BJointType::BEulerZXY:       return "EulerZXY"; break;
        case BJointType::BTranslationXYZ: return "TranslationXYZ"; break;
            
        case BJointType::B1DoF:           return "1DoF"; break;
        case BJointType::B2DoF:           return "2DoF"; break;
        case BJointType::B3DoF:           return "3DoF"; break;
        case BJointType::B4DoF:           return "4DoF"; break;
        case BJointType::B5DoF:           return "5DoF"; break;
        case BJointType::B6DoF:           return "6DoF"; break;
            
        case BJointType::BFloatingBase:   return "FloatingBase"; break;
        case BJointType::BFixed:          return "Fixed"; break;
        case BJointType::BHelical:        return "Helical"; break;
        case BJointType::BMAXJOINT:       return "MaxJoint"; break;
  
        default: exit(EXIT_FAILURE); break;
    }
}

