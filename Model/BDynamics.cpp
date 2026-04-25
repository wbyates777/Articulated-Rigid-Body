/* BDynamics 20/02/2024

 $$$$$$$$$$$$$$$$$$$$$
 $   BDynamics.cpp   $
 $$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 
 The articulated-body algorithm (ABA) is an example of a propagation algorithm, and it is the fastest known algorithm
 for calculating the forward dynamics of a kinematic tree with a computational complexity of $O(N_B)$,
 where $N_B$ is the number of bodies/joints. 
 This is the theoretical minimum for solving the forward dynamics problem (see RBDA, Section 7.3). 
 
 The recursive Newton-Euler algorithm (RNEA) calculate the inverse dynamics of a kinematic tree. 
 It is the simplest, most efficient known algorithm for trees, and also has a computational complexity 
 of $O(N_B)$ (see RBDA, Section 5.3). 
 
 These implemenations depend soly on the GLM linear algebra library ( see https://github.com/g-truc/glm ).
 
 //
 // Implememtation Details
 //
 
 The implementations presented here are based on those in the RBDL library ( see https://github.com/rbdl/rbdl ).
 We use similar variable names and the same object hierarchy. This facilitates numerical coparison testing. 
 Some variables have been moved to apprpriate classes and accessor methods have been added throughout. 
 This improves encapsulation and readability.

 RBDL depends on the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
 fixed-size matrices to arbitrarily large dense matrices, and even sparse matrices.
 This code does not depend on Eigen3, and instead relies on the lighter-weight GLM library 
 for simple 3D-linear algebra types and operations. 
 
 This code depends on the 3D GLM types: glm::dvec3, glm::dmat3, glm::dquat, 
 and functions: glm::cross(v1, v2), glm::dot(v1, v2), glm::length(v1), glm::inverse(m1), glm::mat3_cast(q).
 
 It should be relatively straightforward to convert back to Eigen3 (although see 
 the note below on Eigen3 and GLM row-major/column-major differences), or  replace GLM with some other simple
 linear algebra library.

 
 //
 // Note the difference in syntax between GLM and Eigen3
 //

 std::cout.precision(4);
 std::cout.setf( std::ios::fixed, std::ios::floatfield );
 
 // GLM
 glm::dvec3 v1(1.0, 2.0, 3.0);
 glm::dmat3 m1(1.0, 2.0, 3.0,  4.0, 5.0, 6.0,  7.0, 8.0, 9.0);
 std::cout << arb::transpose(m1) * v1 << std::endl << std::endl;
 
 // Eigen3
 Vector3d v2(1.0, 2.0, 3.0);
 Matrix3d m2(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
 std::cout << m2 * v2  << std::endl; 
 
 see https://en.wikipedia.org/wiki/Row-_and_column-major_order
 
 
 
 //
 // LaTeX
 //
 
 The psuedo LaTeX used to express a leading superscipt can be converted to working LaTeX using the following mapping
 ${\lambda(i)}^X_i$ --> ${}^{\lambda(i)}\!X_i$
 This will typeset the leading superscript $\lambda(i)$ properly i.e $\mysup{B}{X}{A}$
 
 \makeatletter
 \newcommand*\mysup[3]{%
 {}^{#1}\!{#2}_{#3}
 }
 \makeatother
  
*/


#ifndef __BDYNAMICS_H__
#include "BDynamics.h"
#endif

#ifndef __BADJOINT_H__
#include "BAdjoint.h"
#endif

BDynamics::BDynamics( int expected_dof ): m_dof1_U(),
                                          m_dof1_d(),
                                          m_dof1_u(), 
                                          m_dof3_U(),
                                          m_dof3_Dinv(),
                                          m_dof3_u() 
{
    m_dof1_U.reserve(expected_dof);
    m_dof1_d.reserve(expected_dof);
    m_dof1_u.reserve(expected_dof);

    m_dof3_U.reserve(expected_dof);  
    m_dof3_Dinv.reserve(expected_dof);
    m_dof3_u.reserve(expected_dof);
    
    m_IA.reserve(expected_dof);
    m_pA.reserve(expected_dof);
}


void 
BDynamics::update_X_base( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates positions 
// based on UpdateKinematicsCustomin RBDL
{
    const std::vector<BScalar> qdot_zero(qstate.qdot.size(), 0.0);
    
    for (int i = 1; i < m.numBody(); ++i) 
    {
        m.joint(i).jcalc(qstate.q, qdot_zero);
        
        const BTransform &X_lambda = m.joint(i).X_lambda(); 
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            m.body(i).X_base( X_lambda * m.body(lambda).X_base() );
        else  m.body(i).X_base( X_lambda );
    }
}

void 
BDynamics::update_velocity( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates velocities
// based on UpdateKinematicsCustomin RBDL
{
    for (int i = 1; i < m.numBody(); ++i) 
    {
        m.joint(i).jcalc(qstate.q, qstate.qdot);
        
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            m.body(i).v() = (m.joint(i).X_lambda() * m.body(lambda).v()) + m.joint(i).v_J();
        else m.body(i).v() = m.joint(i).v_J();

        m.body(i).c() = m.joint(i).c_J() + arb::crossm( m.body(i).v(), m.joint(i).v_J() );
    }
}


// compute qddot -- accelerations
void  
BDynamics::forward( BModel &m, BModelState &qstate, const BExtForce &f_ext ) // f_ext is f^x_i
// Computes forward dynamics with the Articulated Body algorithm (ABA)
// forward dynamics refers to the computation of the position (in our case accelerations) of 
// an end-effector, such as a jointed robotic arm, from specified values for the joint forces.
// see RBDA, Table 7.1
{
    
    const std::vector<BScalar> &q    = qstate.q;    // pos
    const std::vector<BScalar> &qdot = qstate.qdot; // vel 
    const std::vector<BScalar> &tau  = qstate.tau;  // force
    
    const int N_B = (int) m.numBody();
    
    m_dof1_U.resize(N_B);
    m_dof1_d.resize(N_B);
    m_dof1_u.resize(N_B);
    
    m_dof3_U.resize(N_B);
    m_dof3_Dinv.resize(N_B);
    m_dof3_u.resize(N_B);
    
    m_IA.resize(N_B);
    m_pA.resize(N_B);
    
    // reset the velocity of the root body
    // $v_0 = 0$
    // $a_0 = -a_g$
    m.body(0).v(B_ZERO_6);
    m.body(0).a().set(B_ZERO_3, -m.gravity());
    
    m_IA[0] = B_ZERO_ABI;
    m_pA[0] = B_ZERO_6;
    
    // first pass (root to leaves) to calculate velocity and bias terms 
    // $v_i   = {i}^X_{\lambda(i)}  v_{\lambda(i)} + v_J$ (RBDA, equation 7.34)
    // $c_i   = c_J + v_i \cross v_J$ (RBDA, equation 7.35)
    // $I_i^A = I_i$
    // $p_i^A = v_i \cross^{*} I_i v_i - {i}^X^{*}_0 f^x_i$
    
    for (int i = 1; i < N_B; ++i) 
    {
        m.joint(i).jcalc(q, qdot);  // calculate [X_lambda, X_J, S_i, c_J, v_J]  for joint i
        
        int lambda = m.parentId(i); 
        
        // X_lambda is transformation from the parent body frame to this body frame
        const BTransform& X_lambda = m.joint(i).X_lambda(); 
        
        // set spatial transform X_base in this body 
        if (lambda != 0)
        {
            m.body(i).X_base( X_lambda * m.body(lambda).X_base() );
            m.body(i).v() = (X_lambda * m.body(lambda).v()) + m.joint(i).v_J();
        }
        else 
        {
            m.body(i).X_base( X_lambda );
            m.body(i).v() = m.joint(i).v_J();
        }
   
        m.body(i).c()  = m.joint(i).c_J() + arb::crossm( m.body(i).v(), m.joint(i).v_J() );
        
        m_IA[i] = m.body(i).I(); // initialise articulated inertia
        m_pA[i] = arb::crossf( m.body(i).v(), m.body(i).I() * m.body(i).v() );
        
        if (!f_ext.empty() && f_ext[i] != B_ZERO_6) 
        { 
            // external forces are assumed to be in world coordinates
            // forces must be applied in body coordinates to each body.
            m_pA[i] -= arb::applyForce(m.body(i).X_base(), f_ext[i]);
        }
    }
     
    // second  pass (leaves to root) to calculate  articulated intertia of bodies   
    // 'm_IA' and the spatial bias force 'm_pA' using intermediate results U_i, D_i, and u_i
    // see RBDA, Section 7.3, equations 7.43, 7.44, 7.45, 7.47, 7.48
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i); 
        
        const BTransform& X_lambda = m.joint(i).X_lambda(); 

        if (dofCount == 0)  // body attached with Fixed2 joint (not merged)
        {
            if (lambda != 0) 
            {
                m_IA[lambda] += X_lambda.applyTranspose(m_IA[i]); 
                m_pA[lambda] += X_lambda.applyTranspose(m_pA[i]);
            }
        }
        else if (dofCount == 1) 
        {
            const BVector6 S(m.joint(i).S());
            
            // S^T * I * S,
            m_dof1_U[i] = m_IA[i] * S;                       
            m_dof1_d[i] = arb::dot(S, m_dof1_U[i]);
            m_dof1_u[i] = tau[qidx] - arb::dot(S, m_pA[i]); 
            
            if (lambda != 0) 
            {
                BScalar Dinv = 1.0 / m_dof1_d[i];
                BABInertia Ia = m_IA[i] - BABInertia(m_dof1_U[i], (m_dof1_U[i] * Dinv)); 
                m_IA[lambda] += X_lambda.applyTranspose(Ia); 
                BVector6 pa(m_pA[i] + (Ia * m.body(i).c()) + (m_dof1_U[i] * (m_dof1_u[i] * Dinv))); 
                m_pA[lambda] += X_lambda.applyTranspose(pa);
            }
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(m.joint(i).S());
            const BVector3 res(tau[qidx], tau[qidx + 1], tau[qidx + 2]); 
            
            // S^T * I * S,
            m_dof3_U[i] = m_IA[i] * S;
            //m_dof3_Dinv[i] = arb::inverse(arb::transpose(S) * m_dof3_U[i]);
            //m_dof3_u[i]    = res - (arb::transpose(S) * m_pA[i]);
            const BMatrix3 aux = m_dof3_U[i].top() * arb::transpose(S.top()) 
                                 + m_dof3_U[i].bot() * arb::transpose(S.bot());
            m_dof3_Dinv[i] = arb::inverse(aux);
            m_dof3_u[i] = res - (S.top() * m_pA[i].ang() + S.bot() * m_pA[i].lin());
            
            if (lambda != 0) 
            {
                const BMatrix63 UDinv_tmp(m_dof3_U[i] * m_dof3_Dinv[i]);
                //const BABInertia Ia = m_IA[i] - BABInertia(UDinv_tmp * arb::transpose(m_dof3_U[i])); 
                const BABInertia Ia = m_IA[i] - BABInertia(m_dof3_U[i], m_dof3_Dinv[i]); 
                m_IA[lambda] += X_lambda.applyTranspose(Ia); 
                const BVector6 pa(m_pA[i] + Ia * m.body(i).c() + UDinv_tmp * m_dof3_u[i]);
                m_pA[lambda] += X_lambda.applyTranspose(pa);
            }
        } 
    }

    
    // third (and final) pass (root to leaves) to calculate the acceleration $a_i$ for each body $i$ and joint qddot
    // $a^{'} = {i}^X_{\lambda(i)} a_{\lambda(i)} + c_i$
    // $\ddot{q}_i = D^{-1}_i (u_i - U_i^T a^{'})$
    // $a_i += S_i \ddot{q}_i$ 
    
    std::vector<BScalar> &qddot = qstate.qddot;
    qddot.resize(tau.size()); // output accelerations -- one for each force
    
    for (int i = 1; i < N_B; ++i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i);
        
        const BTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c();
        
        if (dofCount == 1) 
        {
            qddot[qidx] = (1.0 / m_dof1_d[i]) * (m_dof1_u[i] - arb::dot(m_dof1_U[i], m.body(i).a()));
            
            const BVector6 S(m.joint(i).S());
            
            m.body(i).a() += S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            const BVector3 acc(m_dof3_Dinv[i] * (m_dof3_u[i] - (arb::transpose(m_dof3_U[i]) * m.body(i).a())));
    
            qddot[qidx]     = acc[0];
            qddot[qidx + 1] = acc[1];
            qddot[qidx + 2] = acc[2];
            
            const BMatrix63 S(m.joint(i).S());
            
            m.body(i).a() += S * acc;
        } 
    }
}


void // compute tau -- forces
BDynamics::inverse( BModel &m, BModelState &qstate, const BExtForce &f_ext)  // f_ext is f^x_i
// Computes inverse dynamics with the recursive Newton-Euler algorithm (RNEA)
// The reverse calculation, that computes the joint forces that achieve a specified arm position, 
// see RBDA, Table 5.1
{
    const std::vector<BScalar> &q     = qstate.q;     // pos
    const std::vector<BScalar> &qdot  = qstate.qdot;  // vel 
    const std::vector<BScalar> &qddot = qstate.qddot; // acc 
   
    
    // reset the velocity of the root body
    // $v_0 = 0$
    // $a_0 = -a_g$
    m.body(0).v(B_ZERO_6);
    m.body(0).a().set(B_ZERO_3, -m.gravity());
    
    const int N_B = (int) m.numBody();
    
    m_f.assign(N_B, B_ZERO_6);
    
    // $v_i = {i}^X_{\lambda(i)} v_{\lambda(i)} + v_J$
    // $c_i = c_J + v_i \cross v_J$
    // $a_i = {i}^X_{\lambda(i)} a_{\lambda(i)} + S_i \ddot{q} + c_i$
    // $f_i = I_i a_i + v_i \cross^{*} I_i v_i$ (equation 5.9)
    
    for (int i = 1; i < N_B; ++i) 
    {
        m.joint(i).jcalc(q, qdot); // calculate $\[X_lambda, X_J, S_i, c_J, v_J\]$  for joint i
        
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i);
        
        const BTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).v() = (X_lambda * m.body(lambda).v()) + m.joint(i).v_J();
        m.body(i).c() = m.joint(i).c_J() + arb::crossm(m.body(i).v(), m.joint(i).v_J());
        
        if (dofCount == 0)  // body attached with fixed joint (not merged)
        {
            m.body(i).a() = (X_lambda * m.body(lambda).a());
        } 
        else if (dofCount == 1) 
        {
            const BVector6 S(m.joint(i).S());
            m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c() + S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(m.joint(i).S());
            const BVector3 acc(qddot[qidx], qddot[qidx + 1], qddot[qidx + 2]);
            m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c() + S * acc;
        }

        if (!m.body(i).isVirtual()) 
        {
            BBody &b(m.body(i));
            m_f[i] = b.I() * b.a() + arb::crossf(b.v(), b.I() * b.v());
        } 
        else 
        {
            m_f[i] = B_ZERO_6;
        } 
    }
    
    if (!f_ext.empty()) 
    {
        // $f_i -= {i}^X_0^{*} f^x_i$ where f^x are the external forces
        for (int i = 1; i < N_B; ++i) 
        {
            BBodyId lambda = m.parentId(i);
            m.body(i).X_base( m.joint(i).X_lambda() * m.body(lambda).X_base() );
            m_f[i] -= arb::applyForce(m.body(i).X_base(), f_ext[i]);
        }
    }
    
    // $\tau_i = S_i^T f_i$ (equation 5.11)
    // $f_{\lambda(i)} += {\lambda(i)}^X_i^{*} f_i$
    
    std::vector<BScalar> &tau  = qstate.tau; // tau
    tau.resize(qddot.size()); // output forces -- one for each acceleration
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i);
        
        if (dofCount == 1) 
        {
            const BVector6 S(m.joint(i).S());
            tau[qidx] = arb::dot(S, m_f[i]);
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(m.joint(i).S());
            //const BVector3 res = arb::transpose(S) * m_f[i];
            const BVector3 res = S.top() * m_f[i].ang() + S.bot() * m_f[i].lin();
        
            tau[qidx]     = res[0];
            tau[qidx + 1] = res[1];
            tau[qidx + 2] = res[2];
        }

        if (lambda != 0) 
        {
            m_f[lambda] += m.joint(i).X_lambda().applyTranspose(m_f[i]);
        }
    }
}


typedef  std::vector<std::vector<BScalar>> BMatrix;

/*
void 
BDynamics::calculateJSIM(BModel &m, const std::vector<BScalar> &q, BMatrix &H) 
{
    const int N_B = (int)m.numBody();
    const int N_Q = (int)q.size();
    
    // Initialize JSIM to zero
    //H.setZero(N_Q, N_Q);
    H.resize(N_Q, std::vector<BScalar>(N_Q,0.0));

    m_composite_I.resize(N_B); // std::vector<BRBInertia> m_composite_I
    
    // 1. Forward Pass: Update kinematics and initialize composite inertias
    // We only need the joint transforms X_lambda and the motion subspace S.
    for (int i = 1; i < N_B; ++i) 
    {
        m.joint(i).jcalc_position(q); // Only need position-dependent terms
        m_composite_I[i] = m.body(i).I(); 
    }

    // 2. Backward Pass: Calculate composite inertias and populate H
    // See RBDA Table 6.2 (The Composite-Rigid-Body Algorithm)
    for (int i = N_B - 1; i > 0; --i) 
    {
        int lambda = m.parentId(i);
        const BTransform& X_lambda = m.joint(i).X_lambda();

        // Add this body's composite inertia to its parent
        if (lambda != 0) 
        {
            m_composite_I[lambda] += X_lambda.applyTranspose(m_composite_I[i]);
        }

        int qidx_i = m.joint(i).qindex();
        int dof_i  = m.joint(i).DoFCount();

        if (dof_i == 0) continue; // Skip fixed joints

        // F is the spatial force required to produce unit acceleration at joint i
        // F = I_composite * S_i
        if (dof_i == 1) {
            BVector6 Si = m.joint(i).S();
            BVector6 Fi = m_composite_I[i] * Si;
            
            // Self-inertia: H(i,i) = S_i^T * F_i
            H(qidx_i, qidx_i) = arb::dot(Si, Fi);

            // Off-diagonal terms: H(i,j) and H(j,i)
            int j = i;
            while (m.parentId(j) != 0) {
                const BTransform& X_j = m.joint(j).X_lambda();
                Fi = X_j.applyTranspose(Fi); // Shift force to parent frame
                j = m.parentId(j);
                
                int qidx_j = m.joint(j).qindex();
                int dof_j  = m.joint(j).DoFCount();
                if (dof_j == 1) {
                    H(qidx_i, qidx_j) = H(qidx_j, qidx_i) = arb::dot(m.joint(j).S(), Fi);
                } else if (dof_j == 3) {
                    BVector3 H_row = arb::transpose(m.joint(j).S()) * Fi;
                    for (int k = 0; k < 3; ++k) {
                        H(qidx_i, qidx_j + k) = H(qidx_j + k, qidx_i) = H_row[k];
                    }
                }
            }
        } 
        else if (dof_i == 3)
        {
            BMatrix63 Si = m.joint(i).S();
            BMatrix63 Fi = m_composite_I[i] * Si;

            // Self-inertia matrix (3x3 block): H(i,i) = Si^T * Fi
            BMatrix3 H_block = arb::transpose(Si) * Fi;
            for (int r = 0; r < 3; ++r) 
            {
                for (int c = 0; c < 3; ++c) 
                {
                    H(qidx_i + r, qidx_i + c) = H_block(r, c);
                }
            }

            // Off-diagonal terms
            int j = i;
            BMatrix63 Fi_recursive = Fi;
            while (m.parentId(j) != 0) {
                Fi_recursive = m.joint(j).X_lambda().applyTranspose(Fi_recursive);
                j = m.parentId(j);
                
                int qidx_j = m.joint(j).qindex();
                int dof_j  = m.joint(j).DoFCount();
                
                if (dof_j == 1) {
                    BVector3 H_col = arb::transpose(Fi_recursive) * m.joint(j).S();
                    for (int k = 0; k < 3; ++k) {
                        H(qidx_i + k, qidx_j) = H(qidx_j, qidx_i + k) = H_col[k];
                    }
                } else if (dof_j == 3) {
                    BMatrix3 H_sub = arb::transpose(m.joint(j).S()) * Fi_recursive;
                    for (int r = 0; r < 3; ++r) {
                        for (int c = 0; c < 3; ++c) {
                            H(qidx_j + r, qidx_i + c) = H(qidx_i + c, qidx_j + r) = H_sub(r, c);
                        }
                    }
                }
            }
        }
    }
}







void 
BDynamics::compositeRigidBodyAlgorithm( BModel& m,
                                        const BModelState &Q,
                                        BMatrix &H,
                                        bool update_kinematics ) 
{
    
    const int N_B = (int)m.numBody();
    const int N_Q = (int)Q.q.size();
    
    // Initialize JSIM to zero
    //H.setZero(N_Q, N_Q);
    H.resize(N_Q, std::vector<BScalar>(N_Q,0.0));
    
    // 1. Forward Pass: Update kinematics and initialize composite inertias
    // We only need the joint transforms X_lambda and the motion subspace S.

    for (int i = 1; i < N_B; ++i) 
    { 
        m.joint(i).jcalc(Q.q, Q.qdot); // Only need q position-dependent terms
        m_Ic[i] = m.body(i).I(); 
    }

    
    for (int i = N_B - 1; i > 0; --i) 
    {
 
        const BTransform& X_lambda = m.joint(i).X_lambda();
        
        int lambda   = m.parentId(i);
        
        // Add this body's composite inertia to its parent
        if (lambda != 0) 
        {
            m_Ic[lambda] += X_lambda.applyTranspose(m_Ic[i]);
        }
        
        int qidx     = m.joint(i).qindex();    
        int dof_index_i = m.joint(i).DoFCount();   

        if (dof_index_i == 0) 
            continue; // Skip fixed joints
        
        if (dof_index_i == 1) 
        {
            const BVector6 S(m.joint(i).S());
            
            BVector6 F  = m_Ic[i] * S;
            H[dof_index_i][dof_index_i] = arb::dot(F,S);
            
            int j = i;
            int dof_index_j = dof_index_i;
       
            while (m.parentId(j) != 0) 
            {
                const BTransform& X_j = m.joint(j).X_lambda();
                F = X_j.applyTranspose(F); // Shift force to parent frame
                j = m.parentId(j);
                
    
                dof_index_j =  m.joint(j).qindex();
                
                if (m.joints(j).mDoFCount == 1) 
                {
                    H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
                    H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);
                } 
                else if (model.mJoints[j].mDoFCount == 3) 
                {
                    // WBY is this outer?
                    BVector3 H_temp2 =  (arb::transpose(F) * arb::transpose(m.multdof3_S[j]));
                    
                    H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
                    H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;
                }
          
            }
        } 
        else if (dof_index_i == 3)
        {
            const BMatrix63 S(m.joint(i).S());
            
            BMatrix63 F_63 = m_Ic[i] * S;
            H.block<3,3>(dof_index_i, dof_index_i) = arb::transpose(S) * F_63;
            
            int j = i;
            int dof_index_j = dof_index_i;
            
            while (model.lambda[j] != 0) 
            {
                const BTransform& X_j = m.joint(j).X_lambda();
                F_63 = X_j.applyTranspose(F_63);
                j = model.lambda[j];
                dof_index_j = model.mJoints[j].q_index;
                
                if (model.mJoints[j].mDoFCount == 1) 
                {
                    BVector3 H_temp2 = F_63.transpose() * (model.S[j]);
                    
                    H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
                    H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
                } 
                else if (model.mJoints[j].mDoFCount == 3) 
                {
                    BMatrix H_temp2 = F_63.transpose() * (model.multdof3_S[j]);
                    
                    H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
                    H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
                }
            }
        } 
    }
}

 */
