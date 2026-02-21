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



constexpr std::array<std::array<BScalar, 3>, 6> B_ONE_ZERO_6x3
{
     1.0, 0.0, 0.0, 
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0, 
     0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 
     0.0, 0.0, 0.0 
};

constexpr std::array<std::array<BScalar, 3>, 6> B_ZERO_ONE_6x3
{
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 
    0.0, 0.0, 1.0 
};


// constructs a 1-DoF joint with the given motion subspaces
BJoint::BJoint( const BVector6 &axis_0 )  : m_id(0),
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis(1, axis_0)
{
    setMotionSpace(m_axis[0]);
    m_v_J = m_axis[0];
    
    // TODO: this has to be properly determined AND test case. Try Matt's dot product idea
    if (axis_0 == BVector6(B_XAXIS, B_ZERO_3)) 
    {
        m_jtype = JType::RevoluteX;
    } 
    else if (axis_0 == BVector6(B_YAXIS, B_ZERO_3)) 
    {
        m_jtype = JType::RevoluteY;
    } 
    else if (axis_0 == BVector6(B_ZAXIS, B_ZERO_3)) 
    {
        m_jtype = JType::RevoluteZ;
    } 
    else if (axis_0.ang() == B_ZERO_3) 
    {
        m_jtype = JType::Prismatic;
    } 
    else 
    {
        m_jtype = JType::Helical;
    }
    
    validate_spatial_axis(m_axis[0]);  
}

BJoint::BJoint( const BVector6 &axis_0, 
                const BVector6 &axis_1 ) :  m_id(0),
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis()
{
    setJoint({axis_0, axis_1}); 
}

BJoint::BJoint( const BVector6 &axis_0,
                const BVector6 &axis_1,
                const BVector6 &axis_2 )  : m_id(0), 
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis()
{
    setJoint({axis_0, axis_1, axis_2}); 
}

BJoint::BJoint( const BVector6 &axis_0,
                const BVector6 &axis_1,
                const BVector6 &axis_2,
                const BVector6 &axis_3 ) :  m_id(0),
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis()
{
    setJoint({axis_0, axis_1, axis_2, axis_3}); 
}

BJoint::BJoint( const BVector6 &axis_0,
                const BVector6 &axis_1,
                const BVector6 &axis_2,
                const BVector6 &axis_3,
                const BVector6 &axis_4 ) :  m_id(0),
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis()
{
    setJoint({axis_0, axis_1, axis_2, axis_3, axis_4}); 
}

BJoint::BJoint( const BVector6 &axis_0,
                const BVector6 &axis_1,
                const BVector6 &axis_2,
                const BVector6 &axis_3,
                const BVector6 &axis_4,
                const BVector6 &axis_5 )  : m_id(0),
                                            m_qidx(0),
                                            m_widx(0),
                                            m_jtype(JType::UNDEFINED),
                                            m_X_lambda(B_IDENTITY_TRANS),
                                            m_X_J(B_IDENTITY_TRANS),
                                            m_X_T(B_IDENTITY_TRANS),
                                            m_v_J(B_ZERO_6),
                                            m_c_J(B_ZERO_6),
                                            m_S(),
                                            m_axis()
{
    setJoint({axis_0, axis_1, axis_2, axis_3, axis_4, axis_5}); 
}

void
BJoint::setJoint( const std::vector<BVector6> &axes )
// Constructs a N=2,..,6-DoF joint with the given motion subspaces.
{
    int DoF_count = (int) axes.size();
    
    m_axis = axes;
    
    // emulated 2-6 DoF joint.
    switch (DoF_count)
    {
        case 2 : m_jtype = JType::J2DoF; break;
        case 3 : m_jtype = JType::J3DoF; break; 
        case 4 : m_jtype = JType::J4DoF; break; 
        case 5 : m_jtype = JType::J5DoF; break; 
        case 6 : m_jtype = JType::J6DoF; break; 
        default: exit(EXIT_FAILURE); break;
    }

    for (int i = 0; i < DoF_count; ++i)
    {
        validate_spatial_axis(m_axis[i]);
    }
}


/** \brief Checks whether we have pure rotational or translational axis.
 *
 * This function is mainly used to print out warnings when specifying an
 * axis that might not be intended.
 */
bool 
BJoint::validate_spatial_axis( const BVector6 &axis )
{
    bool axis_rot = false;
    bool axis_trans = false;
    
    BVector3 rot(axis.ang());
    BVector3 trans(axis.lin());

    const BScalar SMALL_VALUE = 1E-8;
    
    BScalar rot_len   = arb::length(rot);
    BScalar trans_len = arb::length(trans);
    
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

BJoint::BJoint(JType jtype ) :  m_id(0),
                                m_qidx(0),
                                m_widx(0),
                                m_jtype(jtype),
                                m_X_lambda(B_IDENTITY_TRANS),
                                m_X_J(B_IDENTITY_TRANS),
                                m_X_T(B_IDENTITY_TRANS),
                                m_v_J(B_ZERO_6),
                                m_c_J(B_ZERO_6),
                                m_S(),
                                m_axis(1)
{
    if (m_jtype == JType::RevoluteX) 
    {
        m_axis[0] = BVector6(B_XAXIS, B_ZERO_3);
        
        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::RevoluteY) 
    {
        m_axis[0] = BVector6(B_YAXIS, B_ZERO_3);
        
        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::RevoluteZ) 
    {
        m_axis[0] = BVector6(B_ZAXIS, B_ZERO_3);
        
        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::EulerZYX) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_ZAXIS, B_ZERO_3);
        m_axis[1] = BVector6(B_YAXIS, B_ZERO_3);
        m_axis[2] = BVector6(B_XAXIS, B_ZERO_3);
        
        setMotionSpace(B_ZERO_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::EulerXYZ) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_XAXIS, B_ZERO_3);
        m_axis[1] = BVector6(B_YAXIS, B_ZERO_3);
        m_axis[2] = BVector6(B_ZAXIS, B_ZERO_3);
        
        setMotionSpace(B_ZERO_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::EulerYXZ) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_YAXIS, B_ZERO_3);
        m_axis[1] = BVector6(B_XAXIS, B_ZERO_3);
        m_axis[2] = BVector6(B_ZAXIS, B_ZERO_3);
        
        setMotionSpace(B_ZERO_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::EulerZXY) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_ZAXIS, B_ZERO_3);
        m_axis[1] = BVector6(B_XAXIS, B_ZERO_3);
        m_axis[2] = BVector6(B_YAXIS, B_ZERO_3);

        setMotionSpace(B_ZERO_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::Spherical) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_ZAXIS, B_ZERO_3);
        m_axis[1] = BVector6(B_YAXIS, B_ZERO_3);
        m_axis[2] = BVector6(B_XAXIS, B_ZERO_3);
        
        setMotionSpace(B_ONE_ZERO_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::TransXYZ) 
    {
        m_axis.resize(3);
        m_axis[0] = BVector6(B_ZERO_3, B_XAXIS);
        m_axis[1] = BVector6(B_ZERO_3, B_YAXIS);
        m_axis[2] = BVector6(B_ZERO_3, B_ZAXIS); 
        
        setMotionSpace(B_ZERO_ONE_6x3);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype >= JType::J1DoF && m_jtype <= JType::J6DoF) 
    {
        // create a joint and allocate memory for it.
        std::cout << "Warning: zero vector " << m_axis[0] << std::endl;
        int DoF_count = m_jtype - J1DoF + 1; 
        m_axis.resize(DoF_count, B_ZERO_6); 

        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::Fixed2)
    {
        m_axis.clear(); // ensure DoF = 0
    }
    else if (m_jtype != JType::FloatBase && m_jtype != JType::Fixed1) 
    {
        std::cout << "Error: Invalid constructor Joint(" << toString(m_jtype) << ")." << std::endl;
        exit(EXIT_FAILURE);
    }
}


// only joint_type  BRevolute or BPrismatic
BJoint::BJoint( JType jtype, const BVector3 &jaxis ) :  m_id(0),
                                                        m_qidx(0),
                                                        m_widx(0),
                                                        m_jtype(jtype),
                                                        m_X_lambda(B_IDENTITY_TRANS),
                                                        m_X_J(B_IDENTITY_TRANS),
                                                        m_X_T(B_IDENTITY_TRANS),
                                                        m_v_J(B_ZERO_6),
                                                        m_c_J(B_ZERO_6),
                                                        m_S(),
                                                        m_axis(1)
{
    assert( m_jtype == JType::Revolute || jtype == JType::Prismatic );
    
    if (m_jtype == JType::Revolute) 
    {
        // make sure we have a unit axis
        // assert (joint_axis.length() - 1.0 >  BSMALL_VALUE);
        m_axis[0].set( jaxis, B_ZERO_3  );
        
        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    } 
    else if (m_jtype == JType::Prismatic) 
    {
        // make sure we have a unit axis
        // assert (joint_axis.length() - 1.0 >  BSMALL_VALUE);
        m_axis[0].set( B_ZERO_3, jaxis );
        
        setMotionSpace(m_axis[0]);
        m_v_J = m_axis[0];
    }
}

void
BJoint::clear( void )
{
    m_qidx = 0;
    m_widx = 0;
    m_jtype = JType::UNDEFINED;
    m_X_lambda.clear();
    m_X_J.clear();
    m_X_T.clear();
    m_v_J = B_ZERO_6;
    m_c_J = B_ZERO_6;
    m_S.clear();  
    m_axis.clear();
}    

BQuat
BJoint::getQuat(const std::vector<BScalar> &q) const
{
    assert(m_jtype == JType::Spherical);
 
    BQuat quat;
    
    quat.w = q[m_widx];
    quat.x = q[m_qidx];
    quat.y = q[m_qidx+1];
    quat.z = q[m_qidx+2];
    
    return quat;
}

void 
BJoint::setQuat(const BQuat &quat, std::vector<BScalar> &q) const
{
    assert(m_jtype == JType::Spherical);

    q[m_widx]   = quat.w;
    q[m_qidx]   = quat.x;
    q[m_qidx+1] = quat.y;
    q[m_qidx+2] = quat.z;
}

void
BJoint::setMotionSpace( const BMatrix63 &m )
{
    m_S.resize(18);
    // WARNING: assumes std::array<std::array<BScalar, 3>, 6> is contiguous memory - no padding
    const BScalar* src_ptr = reinterpret_cast<const BScalar*>(m.data().data());
    std::copy_n(src_ptr, 18, m_S.begin());
}

void
BJoint::setMotionSpace( const BMatrix6 &m )
{
    m_S.resize(36);
    // WARNING: assumes std::array<std::array<BScalar, 6>, 6> is contiguous memory - no padding
    const BScalar* src_ptr = reinterpret_cast<const BScalar*>(m.data().data());
    std::copy_n(src_ptr, 36, m_S.begin());
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
    using std::sin;
    using std::cos;
    
    if (m_jtype == JType::Fixed2)  // 0-DoF 
    {
        m_X_lambda = m_X_T; 
    } 
    else if (m_jtype == JType::Spherical) // 3-DoF
    {
        m_v_J = BVector6( qdot[m_qidx], qdot[m_qidx+1], qdot[m_qidx+2], B_ZERO_3 );
        
        m_X_J = BTransform(glm::mat3_cast(getQuat(q)));
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == JType::TransXYZ)  // 3-DoF
    {
        m_v_J = BVector6( B_ZERO_3, qdot[m_qidx], qdot[m_qidx + 1], qdot[m_qidx + 2] );
 
        m_X_lambda.E( m_X_T.E() );
        m_X_lambda.r( m_X_T.r() + arb::transpose(m_X_T.E()) * BVector3(q[m_qidx], q[m_qidx + 1], q[m_qidx + 2]));
    } 
    else if (m_jtype == JType::RevoluteX)  // 1-DoF 
    {
        const BScalar s = sin(q[m_qidx]);
        const BScalar c = cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( E[0],
                                c * E[1] + s * E[2],
                               -s * E[1] + c * E[2] ));
        
        m_X_lambda.r( m_X_T.r() );
        
        m_v_J[0] = qdot[m_qidx];
    } 
    else if (m_jtype == JType::RevoluteY) // 1-DoF 
    {
        const BScalar s = sin(q[m_qidx]);
        const BScalar c = cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( c * E[0] + -s * E[2],
                                E[1],
                                s * E[0] + c *  E[2] ));
        
        m_X_lambda.r( m_X_T.r() );
        
        m_v_J[1] = qdot[m_qidx];
    } 
    else if (m_jtype == JType::RevoluteZ) // 1-DoF
    {
        const BScalar s = sin(q[m_qidx]);
        const BScalar c = cos(q[m_qidx]);
        
        const BMatrix3 &E = m_X_T.E();
        
        m_X_lambda.E( BMatrix3( c * E[0] + s * E[1],
                               -s * E[0] + c * E[1],
                                E[2] ));

        m_X_lambda.r( m_X_T.r() );
        m_v_J[2] = qdot[m_qidx];
    } 
    else if (m_jtype == JType::Revolute) // 1-DoF
    {
        m_X_J = arb::Xrot(q[m_qidx], m_axis[0].ang());
   
        // Note m_S should be non-zero here
        const BVector6 MS(m_S);
        
        m_v_J = MS * qdot[m_qidx]; 
        m_X_lambda = m_X_J * m_X_T;
    }
    else if (m_jtype == JType::Prismatic) // 1-DoF
    {
        m_X_J = arb::Xtrans(m_axis[0].lin() * q[m_qidx]);

        // Note m_S should be non-zero here
        const BVector6 MS(m_S);
        
        m_v_J = MS * qdot[m_qidx]; 
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == JType::Helical) // 1-DoF
    {
        const BVector3 ang(m_axis[0].ang());
        const BVector3 lin(m_axis[0].lin());
        
        const BTransform jrot   = arb::Xrot(q[m_qidx], ang);
        const BTransform jtrans = arb::Xtrans(lin * q[m_qidx]);
        m_X_J = jrot * jtrans;
        
        const BVector3 strans = arb::transpose(m_X_J.E()) * lin; 
        const BVector6 MS(ang, strans);
        
        const BScalar Jqd = qdot[m_qidx];
        m_v_J = MS * Jqd;

        const BVector3 c = arb::cross(ang, strans) * (-Jqd * Jqd); 

        m_c_J = BVector6(B_ZERO_3, c);
        
        m_X_lambda = m_X_J * m_X_T;

        setMotionSpace(MS);
    } 
    else if (m_jtype == JType::EulerZYX)  // 3-DoF
    {
        const BScalar q0 = q[m_qidx];
        const BScalar q1 = q[m_qidx + 1];
        const BScalar q2 = q[m_qidx + 2];
        
        const BScalar s0 = sin(q0);
        const BScalar c0 = cos(q0);
        const BScalar s1 = sin(q1);
        const BScalar c1 = cos(q1);
        const BScalar s2 = sin(q2);
        const BScalar c2 = cos(q2);
        
        m_X_J = BTransform(BMatrix3( c0 * c1, s0 * c1, -s1,
                                     c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
                                     c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2 ));
        
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0] = MS[0][0] = -s1;       
        m_S[2] = MS[0][2] = 1.0;      
        
        m_S[3] = MS[1][0] = c1 * s2;   
        m_S[4] = MS[1][1] = c2;        
        
        m_S[6] = MS[2][0] = c1 * c2;   
        m_S[7] = MS[2][1] = -s2;      
        
        const BScalar qdot0 = qdot[m_qidx];
        const BScalar qdot1 = qdot[m_qidx + 1];
        const BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);

        m_c_J.set(-c1*qdot0*qdot1,
                  -s1*s2*qdot0*qdot1 + c1*c2*qdot0*qdot2 - s2*qdot1*qdot2,
                  -s1*c2*qdot0*qdot1 - c1*s2*qdot0*qdot2 - c2*qdot1*qdot2,
                   0.0, 0.0, 0.0);
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == JType::EulerXYZ) // 3-DoF
    {
        const BScalar q0 = q[m_qidx];
        const BScalar q1 = q[m_qidx + 1];
        const BScalar q2 = q[m_qidx + 2];
        
        const BScalar s0 = sin(q0);
        const BScalar c0 = cos(q0);
        const BScalar s1 = sin(q1);
        const BScalar c1 = cos(q1);
        const BScalar s2 = sin(q2);
        const BScalar c2 = cos(q2);
        
        m_X_J = BTransform(BMatrix3( c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
                                    -s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
                                     s1, -c1 * s0, c1 * c0 ));
        
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0] = MS[0][0] = c2 * c1;          
        m_S[1] = MS[0][1] = s2;               
        
        m_S[3] = MS[1][0] = -s2 * c1;         
        m_S[4] = MS[1][1] = c2;              
        
        m_S[6] = MS[2][0] = s1;              
        m_S[8] = MS[2][2] = 1.0;            
        
        const BScalar qdot0 = qdot[m_qidx];
        const BScalar qdot1 = qdot[m_qidx + 1];
        const BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set(-s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 + c2*qdot2*qdot1,
                  -c2*c1*qdot2*qdot0 + s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
                   c1*qdot1*qdot0,
                   0.0, 0.0, 0.0);
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == JType::EulerYXZ) // 3-DoF
    {
        const BScalar q0 = q[m_qidx];
        const BScalar q1 = q[m_qidx + 1];
        const BScalar q2 = q[m_qidx + 2];

        const BScalar s0 = sin(q0);
        const BScalar c0 = cos(q0);
        const BScalar s1 = sin(q1);
        const BScalar c1 = cos(q1);
        const BScalar s2 = sin(q2);
        const BScalar c2 = cos(q2);
        
        m_X_J = BTransform(BMatrix3( c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
                                    -s2 * c0 + c2 * s1 * s0, c2 * c1,  s2 * s0 + c2 * s1 * c0,
                                     c1 * s0, -s1, c1 * c0 )); 
        
        BMatrix63 MS(B_ZERO_6x3);
   
        m_S[0] = MS[0][0] = s2 * c1;       
        m_S[1] = MS[0][1] = c2;            
        
        m_S[3] = MS[1][0] = c2 * c1;       
        m_S[4] = MS[1][1] = -s2;           
        
        m_S[6] = MS[2][0] = -s1;           
        m_S[8] = MS[2][2] = 1.0;           
        
        const BScalar qdot0 = qdot[m_qidx];
        const BScalar qdot1 = qdot[m_qidx + 1];
        const BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set( c2*c1*qdot2*qdot0 - s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
                  -s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 - c2*qdot2*qdot1,
                  -c1*qdot1*qdot0,
                   0.0, 0.0, 0.0 );
        
        m_X_lambda = m_X_J * m_X_T;
    } 
    else if (m_jtype == JType::EulerZXY) // 3-DoF 
    {
        const BScalar q0 = q[m_qidx];
        const BScalar q1 = q[m_qidx + 1];
        const BScalar q2 = q[m_qidx + 2];
        
        const BScalar s0 = sin(q0);
        const BScalar c0 = cos(q0);
        const BScalar s1 = sin(q1);
        const BScalar c1 = cos(q1);
        const BScalar s2 = sin(q2);
        const BScalar c2 = cos(q2);
        
        m_X_J = BTransform( BMatrix3( -s0 * s1 * s2 + c0 * c2,  s0 * c2 + s1 * s2 * c0,  -s2 * c1,
                                      -s0 * c1,                        c0 * c1,             s1,
                                       s0 * s1 * c2 + s2 * c0,  s0 * s2 - s1 * c0 * c2,  c1 * c2   ));
        m_X_lambda = m_X_J * m_X_T;
        
        BMatrix63 MS(B_ZERO_6x3);
        
        m_S[0] = MS[0][0] = -s2 * c1;     
        m_S[1] = MS[0][1] = c2;           
        
        m_S[3] = MS[1][0] = s1;           
        m_S[5] = MS[1][2] = 1;             
         
        m_S[6] = MS[2][0] = c1 * c2;       
        m_S[7] = MS[2][1] = s2;            
        
        const BScalar qdot0 = qdot[m_qidx];
        const BScalar qdot1 = qdot[m_qidx + 1];
        const BScalar qdot2 = qdot[m_qidx + 2];
        
        m_v_J = MS * BVector3(qdot0, qdot1, qdot2);
        
        m_c_J.set((-c1 * c2 * qdot2 + s1 * s2 * qdot1) * qdot0 - s2 * qdot1 * qdot2,
                   c1 * qdot1 * qdot0,
                   (-s1 * c2 * qdot1 - c1 * s2 * qdot2) * qdot0 + c2 * qdot2 * qdot1,
                   0.0, 0.0, 0.0);
    } 
    else 
    {
        std::cout << "Error jcalc: invalid joint type " << m_jtype << std::endl;
        exit(EXIT_FAILURE);
    }
}


std::string 
BJoint::toString( JType jt )
{
    switch (jt)
    {
        case JType::UNDEFINED:      return "UNDEFINED"; break;
            
        case JType::Prismatic:      return "Prismatic"; break;
        case JType::Revolute:       return "Revolute"; break;
        case JType::RevoluteX:      return "RevoluteX"; break;
        case JType::RevoluteY:      return "RevoluteY"; break;
        case JType::RevoluteZ:      return "RevoluteZ"; break;
 
        case JType::Spherical:      return "Spherical"; break;
        case JType::EulerZYX:       return "EulerZYX"; break;
        case JType::EulerXYZ:       return "EulerXYZ"; break;
        case JType::EulerYXZ:       return "EulerYXZ"; break;
        case JType::EulerZXY:       return "EulerZXY"; break;
        case JType::TransXYZ:       return "TransXYZ"; break;
            
        case JType::J1DoF:          return "1DoF"; break;
        case JType::J2DoF:          return "2DoF"; break;
        case JType::J3DoF:          return "3DoF"; break;
        case JType::J4DoF:          return "4DoF"; break;
        case JType::J5DoF:          return "5DoF"; break;
        case JType::J6DoF:          return "6DoF"; break;
            
        case JType::FloatBase:      return "FloatBase"; break;
        case JType::Fixed1:         return "Fixed1"; break;
        case JType::Fixed2:         return "Fixed2"; break;
        case JType::Helical:        return "Helical"; break;
            
        case JType::MAXJOINT:       return "MAXJOINT"; break;
  
        default: exit(EXIT_FAILURE); break;
    }
}

