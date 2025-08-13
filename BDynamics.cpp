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
 
 These implemenations run a bit faster (around 10%) than those of RBDL on simple test cases without instrinsics.
 This is due to the omission of the custom joint code, and removing some redundant calculations and unnecessary copying.
 
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
}


// compute qddot -- accelerations
void  
BDynamics::forward( BModel &m, BModelState &qstate, const BExtForce &f_ext ) // f_ext is f^x_i
// Computes forward dynamics with the Articulated Body algorithm (ABA)
// forward dynamics refers to the computation of the position (in our case accelerations) of 
// an end-effector, such as a jointed robotic arm, from specified values for the joint forces.
// see RBDA, Table 7.1
{   
    const std::vector<BScalar>  &q   = qstate.q;    // pos
    const std::vector<BScalar> &qdot = qstate.qdot; // vel 
    const std::vector<BScalar> &tau  = qstate.tau;  // force
    
    const int N_B = (int) m.bodies();
    
    m_U.assign(N_B, B_ZERO_6);
    m_d.assign(N_B, 0.0);
    m_u.assign(N_B, 0.0);
    
    m_dof3_U.assign(N_B, B_ZERO_6x3);
    m_dof3_Dinv.assign(N_B, B_ZERO_3x3);
    m_dof3_u.assign(N_B, B_ZERO_3);
    
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
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        // set spatial transform X_base in this body 
        if (lambda != 0)
            m.body(i).X_base( X_lambda * m.body(lambda).X_base() );
        else m.body(i).X_base( X_lambda );

        m.body(i).v() = (X_lambda * m.body(lambda).v()) + m.joint(i).v_J();
        m.body(i).c() = m.joint(i).c_J() + arb::crossm( m.body(i).v(), m.joint(i).v_J() );
        m.IA(i) = m.body(i).I(); // initialise articulated inertia
        m.pA(i) = arb::crossf( m.body(i).v(), m.body(i).I() * m.body(i).v() );
        
        if (!f_ext.empty() && f_ext[i] != B_ZERO_6) 
        { 
            m.pA(i) -= m.body(i).X_base().toAdjoint() * f_ext[i];
        }
    }
     
    // second  pass (leaves to root) to calculate articulated body spatial intertia of bodies   
    // 'm.IA' and the spatial bias force 'm.pA' using intermediate results U_i, D_i, and u_i
    // see RBDA, Section 7.3, equations 7.43, 7.44, 7.45, 7.47, 7.48
    // $U_i = I_i^A S_i$ 
    // $D_i = S_i^T U_i$ 
    // $u_i = \tau_i - S_i^T p^a_i$  
    // $I^a = I^A_i - (U_i D_i^{-1} U_i^T)$
    // $p^a = p^A_i + (I^a c_i) + (U_i D_i^{-1} u_i)$
    // $I^A_{\lambda(i)} += {\lambda(i)}^X^{*}_i I^a {i}^X_{\lambda(i)}$
    // $p^A_{\lambda(i)} += {\lambda(i)}^X^{*}_i p^a$
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i); 
        
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 

        if (dofCount == 1) 
        {
            BSpatialVector S(m.joint(i).S());
            
            m_U[i] = m.IA(i) * S;                       
            m_d[i] = arb::dot(S, m_U[i]);  // $S_i^T U_i$                    
            m_u[i] = tau[qidx] - arb::dot(S, m.pA(i)); 
            
            if (lambda != 0) 
            {
                BScalar Dinv = 1.0 / m_d[i];
                BABInertia Ia = m.IA(i) - BABInertia(m_U[i], (m_U[i] * Dinv)); 
                m.IA(lambda) += X_lambda.applyTranspose(Ia); 
                //BSpatialMatrix Ia(m.IA(i) - arb::outer(m_U[i], (m_U[i] * Dinv))); 
                //m.IA(lambda) += arb::transpose(X_lambda) * Ia * X_lambda;
                BSpatialVector pa(m.pA(i) + (Ia * m.body(i).c()) + (m_U[i] * (m_u[i] * Dinv))); 
                m.pA(lambda) += X_lambda.applyTranspose(pa);
            }
        } 
        else if (dofCount == 3) 
        {
            BMatrix63 S(m.joint(i).S());
        
            BVector3 tau_tmp(tau[qidx], tau[qidx + 1], tau[qidx + 2]); 
            
            m_dof3_U[i]    = m.IA(i) * S;
            m_dof3_Dinv[i] = glm::inverse(arb::transpose(S) * m_dof3_U[i]);
            m_dof3_u[i]    = tau_tmp - (arb::transpose(S) * m.pA(i));
     
            if (lambda != 0) 
            {
                BMatrix63 UDinv_tmp(m_dof3_U[i] * m_dof3_Dinv[i]);
                
                BABInertia Ia = m.IA(i) - BABInertia(UDinv_tmp * arb::transpose(m_dof3_U[i])); 
                m.IA(lambda) += X_lambda.applyTranspose(Ia); 
                //BSpatialMatrix Ia( m.IA(i) - UDinv_tmp * arb::transpose(m_dof3_U[i]));
                //m.IA(lambda) += arb::transpose(X_lambda) * Ia * X_lambda;
                BSpatialVector pa(m.pA(i) + Ia * m.body(i).c() + UDinv_tmp * m_dof3_u[i]);
                m.pA(lambda) += X_lambda.applyTranspose(pa);
            }
        } 
    }
    
    // third (and final) pass (root to leaves) to calculate the acceleration $a_i$ for each body $i$ and joint QDDot
    // $a^{'} = {i}^X_{\lambda(i)} a_{\lambda(i)} + c_i$
    // $\ddot{q}_i = D^{-1}_i (u_i - U_i^T a^{'})$
    // $a_i += S_i \ddot{q}_i$ 
    
    std::vector<BScalar> &QDDot = qstate.qddot;
    QDDot.resize(tau.size()); // output accelerations -- one for each force
    
    for (int i = 1; i < N_B; ++i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i);
        
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c();
        
        if (dofCount == 1) 
        {
            QDDot[qidx] = (1.0 / m_d[i]) * (m_u[i] - arb::dot(m_U[i], m.body(i).a()));
            
            BSpatialVector S(m.joint(i).S());
            
            m.body(i).a() += S * QDDot[qidx];
        } 
        else if (dofCount == 3) 
        {
            BVector3 qdd_tmp(m_dof3_Dinv[i] * (m_dof3_u[i] - (arb::transpose(m_dof3_U[i]) * m.body(i).a())));
    
            QDDot[qidx]     = qdd_tmp[0];
            QDDot[qidx + 1] = qdd_tmp[1];
            QDDot[qidx + 2] = qdd_tmp[2];
            
            BMatrix63 S(m.joint(i).S());
            
            m.body(i).a() += S * qdd_tmp;
        } 
    }
}


void // compute tau -- forces
BDynamics::inverse( BModel &m, BModelState &qstate, const BExtForce &f_ext)  // f_ext is f^x_i
// Computes inverse dynamics with the recursive Newton-Euler algorithm (RNEA)
// The reverse calculation, that computes the joint forces that achieve a specified arm position, 
// see RBDA, Table 5.1
{
    const std::vector<BScalar> &q     = qstate.q; // pos
    const std::vector<BScalar> &qdot  = qstate.qdot; // vel 
    const std::vector<BScalar> &qddot = qstate.qddot; // vel 
   
    
    // reset the velocity of the root body
    // $v_0 = 0$
    // $a_0 = -a_g$
    m.body(0).v(B_ZERO_6);
    m.body(0).a().set(B_ZERO_3, -m.gravity());
    
    const int N_B = (int) m.bodies();
    
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
        
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).v() = (X_lambda * m.body(lambda).v()) + m.joint(i).v_J();
        m.body(i).c() = m.joint(i).c_J() + arb::crossm(m.body(i).v(), m.joint(i).v_J());
        
        if (dofCount == 1) 
        {
            BSpatialVector S(m.joint(i).S());
            m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c() + S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            BMatrix63 S(m.joint(i).S());
            BVector3 qdd_tmp(qddot[qidx], qddot[qidx + 1], qddot[qidx + 2]);
            m.body(i).a() = (X_lambda * m.body(lambda).a()) + m.body(i).c() + S * qdd_tmp;
        }

        if (!m.body(i).isVirtual()) 
        {
            BBody &b(m.body(i));
            b.f() = b.I() * b.a() + arb::crossf(b.v(), b.I() * b.v());
        } 
        else 
        {
            m.body(i).f() = B_ZERO_6;
        } 
    }
    
    if (!f_ext.empty()) 
    {
        // $f_i -= {i}^X_0^{*} f^x_i$ where f^x are the external forces
        for (int i = 1; i < N_B; ++i) 
        {
            BBodyId lambda = m.parentId(i);
            m.body(i).X_base( m.joint(i).X_lambda() * m.body(lambda).X_base() );
            m.body(i).f() -= m.body(i).X_base().toAdjoint() * f_ext[i];
        }
    }
    
    // $\tau_i = S_i^T f_i$ (equation 5.11)
    // $f_{\lambda(i)} += {\lambda(i)}^X_i^{*} f_i$
    
    std::vector<BScalar> &Tau  = qstate.tau; // tau
    Tau.resize(qddot.size()); // output forces -- one for each acceleration
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentId(i);
        
        if (dofCount == 1) 
        {
            BSpatialVector S(m.joint(i).S());
            Tau[qidx] = arb::dot(S, m.body(i).f());
        } 
        else if (dofCount == 3) 
        {
            BMatrix63 S(m.joint(i).S());
            BVector3 tau_tmp = arb::transpose(S) * m.body(i).f();

            Tau[qidx]     = tau_tmp[0];
            Tau[qidx + 1] = tau_tmp[1];
            Tau[qidx + 2] = tau_tmp[2];
        }

        if (lambda != 0) 
        {
            m.body(lambda).f() += m.joint(i).X_lambda().applyTranspose(m.body(i).f());
        }
    }
}
