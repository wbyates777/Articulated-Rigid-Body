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
 std::cout << glm::transpose(m1) * v1 << std::endl << std::endl;
 
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
 
*/


#ifndef __BDYNAMICS_H__
#include "BDynamics.h"
#endif



BDynamics::BDynamics( void ): m_U(),
                              m_d(),
                              m_u(),
                              m_c(),  
                              m_dof3_U(),
                              m_dof3_Dinv(),
                              m_dof3_u() 
{
    int expected_bodies = 5;
    m_U.reserve(expected_bodies);
    m_d.reserve(expected_bodies);
    m_u.reserve(expected_bodies);
    m_c.reserve(expected_bodies);
    m_dof3_U.reserve(expected_bodies);  
    m_dof3_Dinv.reserve(expected_bodies);
    m_dof3_u.reserve(expected_bodies);
}


const BSpatialMatrix
BDynamics::mulT( const BSpatialVector& a, const BSpatialVector& b ) const
// original code using Eigen3 is 
//      model.U[i] * (model.U[i] / model.d[i]).transpose())
// where 
//      a = model.U[i] is column vector, and 
//      b^T = (model.U[i] / model.d[i]).transpose() is a row vector
{ 
    return BSpatialMatrix(a[0] * b[0], a[0] * b[1], a[0] * b[2], a[0] * b[3], a[0] * b[4], a[0] * b[5], 
                          a[1] * b[0], a[1] * b[1], a[1] * b[2], a[1] * b[3], a[1] * b[4], a[1] * b[5], 
                          a[2] * b[0], a[2] * b[1], a[2] * b[2], a[2] * b[3], a[2] * b[4], a[2] * b[5], 
                          a[3] * b[0], a[3] * b[1], a[3] * b[2], a[3] * b[3], a[3] * b[4], a[3] * b[5], 
                          a[4] * b[0], a[4] * b[1], a[4] * b[2], a[4] * b[3], a[4] * b[4], a[4] * b[5], 
                          a[5] * b[0], a[5] * b[1], a[5] * b[2], a[5] * b[3], a[5] * b[4], a[5] * b[5]);
}


void 
BDynamics::forward( BModel &m, 
                    const std::vector<BScalar> &q,
                    const std::vector<BScalar> &qdot,
                    const std::vector<BScalar> &tau,
                    std::vector<BScalar> &QDDot, // output
                    const std::vector<BSpatialVector> &f_ext ) // f_ext is f^x_i

// see RBDA, Table 7.1
{
    const int N_B = (int) m.bodies();
    
    m_U.assign(N_B, BZERO_6);
    m_d.assign(N_B, 0.0);
    m_u.assign(N_B, 0.0);
    
    m_c.assign(N_B, BZERO_6);
    
    m_dof3_U.assign(N_B, BZERO_6x3);
    m_dof3_Dinv.assign(N_B, BZERO_3x3);
    m_dof3_u.assign(N_B, BZERO_3);
    
    BSpatialVector spatial_gravity(BZERO_3, m.gravity());
    
    // reset the velocity of the root body
    m.body(0).v(BZERO_6);
    
    // first pass (root to leaves) to calculate velocity and bias terms 
    // $v_i   = {i}^X_{\lambda(i)}  v_{\lambda(i)} + v_J$ (RBDA, equation 7.34)
    // $c_i   = c_J + v_i \cross v_J$ (RBDA, equation 7.35)
    // $I_i^A = I_i$
    // $p_i^A = v_i \cross^{*} I_i v_i - {i}^X^{*}_0 f^x_i$
    
    for (int i = 1; i < N_B; ++i) 
    {
        m.joint(i).jcalc(q, qdot);  // calculate [X_lambda, X_J, S_i, c_J, v_J]  for joint i
        
        int lambda = m.parentID(i); 
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        if (lambda != 0)
            m.body(i).X_base( X_lambda * m.body(lambda).X_base() );
        else m.body(i).X_base( X_lambda );

        m.body(i).v() = X_lambda.apply( m.body(lambda).v() ) + m.joint(i).v_J();
        m_c[i] = m.joint(i).c_J() + arb::crossm( m.body(i).v(), m.joint(i).v_J() );
        m.body(i).I().setSpatialMatrix( m.IA(i) ); // this sets the values of m.IA(i)
        m.pA(i) = arb::crossf( m.body(i).v(), m.body(i).I() * m.body(i).v() );
        
        if (!f_ext.empty() && f_ext[i] != BZERO_6) 
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
        int lambda   = m.parentID(i); 
        
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
                BSpatialMatrix Ia(m.IA(i) - mulT(m_U[i], (m_U[i] * Dinv))); 
                BSpatialVector pa(m.pA(i) + (Ia * m_c[i]) + (m_U[i] * (m_u[i] * Dinv)));
                
                m.IA(lambda) += X_lambda.toTranspose() * Ia * X_lambda.toMatrix();
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
                BMatrix63 UDinv_tmp = m_dof3_U[i] * m_dof3_Dinv[i];
                BSpatialMatrix Ia(m.IA(i) - UDinv_tmp * arb::transpose(m_dof3_U[i]));
                BSpatialVector pa(m.pA(i) + Ia * m_c[i] + UDinv_tmp * m_dof3_u[i]);
                
                m.IA(lambda) += X_lambda.toTranspose() * Ia * X_lambda.toMatrix();
                m.pA(lambda) += X_lambda.applyTranspose(pa);
            }
        } 
    }
    
    // third (and final) pass (root to leaves) to calculate the acceleration $a_i$ for each body $i$ and joint QDDot
    // $a^{'} = {i}^X_{\lambda(i)} a_{\lambda(i)} + c_i$
    // $\ddot{q}_i = D^{-1}_i (u_i - U_i^T a^{'})$
    // $a_i += S_i \ddot{q}_i$ 
    
    // set acceleration due to gravity
    m.body(0).a( spatial_gravity * -1.0 );
    
    for (int i = 1; i < N_B; ++i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentID(i);
        
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).a() = X_lambda.apply(m.body(lambda).a()) + m_c[i];
        
        if (dofCount == 1) 
        {
            QDDot[qidx] = (1.0 / m_d[i]) * (m_u[i] - arb::dot(m_U[i], m.body(i).a()) );
            
            BSpatialVector S(m.joint(i).S());
            
            m.body(i).a() += S * QDDot[qidx];
        } 
        else if (dofCount == 3) 
        {
            BVector3 qdd_tmp(m_dof3_Dinv[i] * (m_dof3_u[i] - arb::transpose(m_dof3_U[i]) * m.body(i).a()));
            
            QDDot[qidx]     = qdd_tmp[0];
            QDDot[qidx + 1] = qdd_tmp[1];
            QDDot[qidx + 2] = qdd_tmp[2];
            
            BMatrix63 S(m.joint(i).S());
            
            m.body(i).a() += S * qdd_tmp;
        } 
    }
}


void 
BDynamics::inverse( BModel &m,
                    const std::vector<BScalar> &q,
                    const std::vector<BScalar> &qdot,
                    const std::vector<BScalar> &qddot,
                    std::vector<BScalar> &Tau,
                    const std::vector<BSpatialVector> &f_ext)  // f_ext is f^x_i
// see RBDA, Table 5.1
{
    // reset the velocity of the root body
    // $v_0 = 0$
    // $a_0 = -a_g$
    
    m.body(0).v(BZERO_6);
    m.body(0).a().set(BZERO_3, -m.gravity());
    
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
        int lambda   = m.parentID(i);
        
        const BSpatialTransform& X_lambda = m.joint(i).X_lambda(); 
        
        m.body(i).v() = X_lambda.apply(m.body(lambda).v()) + m.joint(i).v_J();
        m_c[i] = m.joint(i).c_J() + arb::crossm(m.body(i).v(), m.joint(i).v_J());
        
        if (dofCount == 1) 
        {
            BSpatialVector S(m.joint(i).S());
            m.body(i).a() = X_lambda.apply(m.body(lambda).a()) + m_c[i] + S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            BMatrix63 S(m.joint(i).S());
            BVector3 qdd_tmp(qddot[qidx], qddot[qidx + 1], qddot[qidx + 2]);
            m.body(i).a() = X_lambda.apply(m.body(lambda).a()) + m_c[i] + S * qdd_tmp;
        }

        if (!m.body(i).isVirtual()) 
        {
            BBody &b(m.body(i));
            b.f() = b.I() * b.a() + arb::crossf(b.v(), b.I() * b.v());
        } 
        else 
        {
            m.body(i).f() = BZERO_6;
        } 
    }
    
    if (!f_ext.empty()) 
    {
        // $f_i -= {i}^X_0^{*} f^x_i$ where f^x are the external forces
        for (int i = 1; i < N_B; ++i) 
        {
            BBodyID lambda = m.parentID(i);
            m.body(i).X_base( m.joint(i).X_lambda() * m.body(lambda).X_base() );
            m.body(i).f() -= m.body(i).X_base().toAdjoint() * f_ext[i];
        }
    }
    
    // $\tau_i = S_i^T f_i$ (equation 5.11)
    // $f_{\lambda(i)} += {\lambda(i)}^X_i^{*} f_i$
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        int qidx     = m.joint(i).qindex();    
        int dofCount = m.joint(i).DoFCount();   
        int lambda   = m.parentID(i);
        
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
