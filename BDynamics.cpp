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
 We use similar variable names and the same object hierarchy. Some variables have been moved to apprpriate classes and 
 accessor methods have been added throughout. This improves encapsulation and readability.

 RBDL depends on the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
 fixed-size matrices to arbitrarily large dense matrices, and even sparse matrices.
 This code does not depend on Eigen3, and instead relies on the lighter-weight GLM library 
 for simple 3D-linear algebra types and operations. As the GLM library does not support 
 6D vectors and matricies, the code for custom joint types is not implemented.

 This code depends on the 3D GLM types: glm::dvec3, glm::dmat3, glm::dquat, 
 and functions: glm::cross(v1, v2), glm::dot(v1, v2), glm::length(v1), glm::inverse(m1), glm::toMat3(q).
 
 It should be relatively straightforward to convert back to Eigen3 (although see 
 the note below on Eigen3 and GLM differences), or  replace GLM with some other simple
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
 
 
 //
 // LaTeX
 //
 
 The psuedo LaTeX used to express a leading superscipt can be converted to working LaTeX using the following mapping
 ${\lambda(i)}^X_i$ --> ${}^{\lambda(i)}\!X_i$
 This will typeset the leading superscript $\lambda(i)$ properly
 
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

BDynamics::BDynamics( int expected_dof ): m_U(),
                                          m_d(),
                                          m_u(), 
                                          m_dof3_U(),
                                          m_dof3_Dinv(),
                                          m_dof3_u() 
{
    m_U.reserve(expected_dof);
    m_d.reserve(expected_dof);
    m_u.reserve(expected_dof);

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
    
    for (int i = 1; i < m.bodies(); ++i) 
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
    for (int i = 1; i < m.bodies(); ++i) 
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
    
    const int N_B = (int) m.bodies();
    
    m_U.resize(N_B);
    m_d.resize(N_B);
    m_u.resize(N_B);
    
    m_dof3_U.resize(N_B);
    m_dof3_Dinv.resize(N_B);
    m_dof3_u.resize(N_B);
    
    m_IA.resize(N_B, B_ZERO_ABI);
    m_pA.resize(N_B, B_ZERO_6);
    
    // reset the velocity of the root body
    // $v_0 = 0$
    // $a_0 = -a_g$
    m.body(0).v(B_ZERO_6);
    m.body(0).a().set(B_ZERO_3, -m.gravity());
    
    
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
            // we use adjoint here - it is equivalent to arb::dual(X_base)
            m_pA[i] -= arb::applyAdjoint(m.body(i).X_base(), f_ext[i]);
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
            m_U[i] = m_IA[i] * S;                       
            m_d[i] = arb::dot(S, m_U[i]);
            m_u[i] = tau[qidx] - arb::dot(S, m_pA[i]); 
            
            if (lambda != 0) 
            {
                BScalar Dinv = 1.0 / m_d[i];
                BABInertia Ia = m_IA[i] - BABInertia(m_U[i], (m_U[i] * Dinv)); 
                m_IA[lambda] += X_lambda.applyTranspose(Ia); 
                BVector6 pa(m_pA[i] + (Ia * m.body(i).c()) + (m_U[i] * (m_u[i] * Dinv))); 
                m_pA[lambda] += X_lambda.applyTranspose(pa);
            }
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(m.joint(i).S());
            const BVector3 res(tau[qidx], tau[qidx + 1], tau[qidx + 2]); 
            
            // S^T * I * S,
            m_dof3_U[i] = m_IA[i] * S;
            const BMatrix3 aux = m_dof3_U[i].top() * arb::transpose(S.top()) + m_dof3_U[i].bot() * arb::transpose(S.bot());
            m_dof3_Dinv[i] = arb::inverse(aux);
            m_dof3_u[i] = res - (S.top() * m_pA[i].ang() + S.bot() * m_pA[i].lin());
            
            if (lambda != 0) 
            {
                const BMatrix63 UDinv_tmp(m_dof3_U[i] * m_dof3_Dinv[i]);
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
            qddot[qidx] = (1.0 / m_d[i]) * (m_u[i] - arb::dot(m_U[i], m.body(i).a()));
            
            const BVector6 S(m.joint(i).S());
            
            m.body(i).a() += S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            const BVector3 res(m_dof3_Dinv[i] * (m_dof3_u[i] - (arb::transpose(m_dof3_U[i]) * m.body(i).a())));
    
            qddot[qidx]     = res[0];
            qddot[qidx + 1] = res[1];
            qddot[qidx + 2] = res[2];
            
            const BMatrix63 S(m.joint(i).S());
            
            m.body(i).a() += S * res;
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
    
    const int N_B = (int) m.bodies();
    
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
            m_f[i] -= arb::applyAdjoint(m.body(i).X_base(), f_ext[i]);
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
