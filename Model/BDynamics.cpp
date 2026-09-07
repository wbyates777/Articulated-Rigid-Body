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
    
    m_Ic.reserve(expected_dof);
}


void 
BDynamics::update_X_base( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates positions 
// based on UpdateKinematicsCustomin RBDL
{
    const std::vector<BScalar> qdot_zero(qstate.qdot.size(), 0.0);
    
    for (int i = 1; i < m.numBody(); ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(qstate.q, qdot_zero);
        
        const BTransform &X_lambda = joint.X_lambda(); 
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            body.X_base( X_lambda * m.body(lambda).X_base() );
        else  body.X_base( X_lambda );
    }
}

void 
BDynamics::update_velocity( BModel &m, const BModelState &qstate ) 
// update kinematics - calculates velocities
// based on UpdateKinematicsCustomin RBDL
{
    for (int i = 1; i < m.numBody(); ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(qstate.q, qstate.qdot);
        
        const int lambda = m.parentId(i); 
        
        if (lambda != 0) 
            body.v() = (joint.X_lambda() * m.body(lambda).v()) + joint.v_J();
        else body.v() = joint.v_J();

        body.c() = joint.c_J() + arb::crossm( body.v(), joint.v_J() );
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
    m.body(0).a().lin(-m.gravity());
 
    m_IA[0] = B_ZERO_ABI;
    m_pA[0] = B_ZERO_6;
    
    // first pass (root to leaves) to calculate velocity and bias terms 
    // $v_i   = {i}^X_{\lambda(i)}  v_{\lambda(i)} + v_J$ (RBDA, equation 7.34)
    // $c_i   = c_J + v_i \cross v_J$ (RBDA, equation 7.35)
    // $I_i^A = I_i$
    // $p_i^A = v_i \cross^{*} I_i v_i - {i}^X^{*}_0 f^x_i$
    
    for (int i = 1; i < N_B; ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(q, qdot);  // calculate [X_lambda, X_J, S_i, c_J, v_J]  for joint i
        
        int lambda = m.parentId(i); 
        
        // X_lambda is transformation from the parent body frame to this body frame
        const BTransform& X_lambda = joint.X_lambda(); 
        
        body.v() = joint.v_J();
        
        // set spatial transform X_base in this body 
        if (lambda != 0)
        {
            body.X_base( X_lambda * m.body(lambda).X_base() );
            body.v() += X_lambda * m.body(lambda).v();
        }
        else 
        {
            body.X_base( X_lambda );
        }
   
        body.c()  = joint.c_J() + arb::crossm( body.v(), joint.v_J() );
        
        m_IA[i] = body.I(); // initialise articulated inertia
        m_pA[i] = arb::crossf( body.v(), body.I() * body.v() );
        
        if (!f_ext.empty() && f_ext[i] != B_ZERO_6) 
        { 
            // external forces are assumed to be in world coordinates
            // forces must be applied in body coordinates to each body.
            m_pA[i] -= arb::applyForce(body.X_base(), f_ext[i]);
        }
    }
 
    // second  pass (leaves to root) to calculate  articulated intertia of bodies   
    // 'm_IA' and the spatial bias force 'm_pA' using intermediate results U_i, D_i, and u_i
    // see RBDA, Section 7.3, equations 7.43, 7.44, 7.45, 7.47, 7.48
    
    for (int i = N_B - 1; i > 0; --i) 
    {
        const BBody &body = m.body(i);
        const BJoint &joint = m.joint(i);
        
        int qidx     = joint.qindex();    
        int dofCount = joint.DoFCount();   
        int lambda   = m.parentId(i); 
        
        const BTransform& X_lambda = joint.X_lambda(); 

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
            const BVector6 S(joint.S());
            
            // S^T * I * S,
            m_dof1_U[i] = m_IA[i] * S;                       
            m_dof1_d[i] = arb::dot(S, m_dof1_U[i]);
            m_dof1_u[i] = tau[qidx] - arb::dot(S, m_pA[i]); 
            
            if (lambda != 0) 
            {
                BScalar Dinv = 1.0 / m_dof1_d[i];
                BABInertia Ia = m_IA[i] - BABInertia(m_dof1_U[i], (m_dof1_U[i] * Dinv)); 
                m_IA[lambda] += X_lambda.applyTranspose(Ia); 
                BVector6 pa(m_pA[i] + (Ia * body.c()) + (m_dof1_U[i] * (m_dof1_u[i] * Dinv))); 
                m_pA[lambda] += X_lambda.applyTranspose(pa);
            }
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(joint.S());
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
                const BVector6 pa(m_pA[i] + Ia * body.c() + UDinv_tmp * m_dof3_u[i]);
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
        BBody &body = m.body(i);
        const BJoint &joint = m.joint(i);
        
        int qidx     = joint.qindex();    
        int dofCount = joint.DoFCount();   
        int lambda   = m.parentId(i);
        
        const BTransform& X_lambda = joint.X_lambda(); 
        
        body.a() = (X_lambda * m.body(lambda).a()) + body.c();
      
        if (dofCount == 1) 
        {
            qddot[qidx] = (1.0 / m_dof1_d[i]) * (m_dof1_u[i] - arb::dot(m_dof1_U[i], body.a()));
            
            const BVector6 S(joint.S());
            
            body.a() += S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            const BVector3 acc(m_dof3_Dinv[i] * (m_dof3_u[i] - (arb::transpose(m_dof3_U[i]) * body.a())));
    
            qddot[qidx]     = acc[0];
            qddot[qidx + 1] = acc[1];
            qddot[qidx + 2] = acc[2];
            
            const BMatrix63 S(joint.S());
            
            body.a() += S * acc;
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
    m.body(0).a().lin(-m.gravity());
    
    const int N_B = (int) m.numBody();
    
    m_f.resize(N_B);
    m_f[0] = B_ZERO_6;
    
    // $v_i = {i}^X_{\lambda(i)} v_{\lambda(i)} + v_J$
    // $c_i = c_J + v_i \cross v_J$
    // $a_i = {i}^X_{\lambda(i)} a_{\lambda(i)} + S_i \ddot{q} + c_i$
    // $f_i = I_i a_i + v_i \cross^{*} I_i v_i$ (equation 5.9)
    
    for (int i = 1; i < N_B; ++i) 
    {
        BBody &body = m.body(i);
        BJoint &joint = m.joint(i);
        
        joint.jcalc(q, qdot); // calculate $\[X_lambda, X_J, S_i, c_J, v_J\]$  for joint i
        
        int qidx     = joint.qindex();    
        int dofCount = joint.DoFCount();   
        int lambda   = m.parentId(i);
        
        const BTransform &X_lambda = joint.X_lambda(); 
        
        body.a() = (X_lambda * m.body(lambda).a());
        body.v() = (X_lambda * m.body(lambda).v()) + joint.v_J();

        body.c() = joint.c_J() + arb::crossm(body.v(), joint.v_J());
        
        if (dofCount == 1) 
        {
            const BVector6 S(joint.S());
            body.a() += body.c() + S * qddot[qidx];
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(joint.S());
            const BVector3 acc(qddot[qidx], qddot[qidx + 1], qddot[qidx + 2]);
            body.a() += body.c() + S * acc;
        }

        if (!body.isVirtual()) 
        {
            m_f[i] = body.I() * body.a() + arb::crossf(body.v(), body.I() * body.v());
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
        const BJoint &joint = m.joint(i);
        
        int qidx     = joint.qindex();    
        int dofCount = joint.DoFCount();   
        int lambda   = m.parentId(i);
        
        if (dofCount == 1) 
        {
            const BVector6 S(joint.S());
            tau[qidx] = arb::dot(S, m_f[i]);
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(joint.S());
            //const BVector3 res = arb::transpose(S) * m_f[i];
            const BVector3 res = S.top() * m_f[i].ang() + S.bot() * m_f[i].lin();
        
            tau[qidx]     = res[0];
            tau[qidx + 1] = res[1];
            tau[qidx + 2] = res[2];
        }

        if (lambda != 0) 
        {
            m_f[lambda] += joint.X_lambda().applyTranspose(m_f[i]);
        }
    }
}

void 
BDynamics::crba( BModel &m, const BModelState &qstate, BMatrix &H, bool update_kinematics ) 
// Composite-Rigid-Body Algorithm, RBDA, Section 6.2, page 104
// Given an empty (zeroed) H matrix, fill in the elements of the 'joint space inertia matrix'
{    
    const int N_B = (int) m.numBody();
    m_Ic.resize(N_B);
    m_Ic[0] = B_ZERO_RBI;

    if (update_kinematics) 
    {
        m_qdot_zero.resize(qstate.qdot.size(), 0.0);
        for (int i = 1; i < N_B; ++i) 
        {
            m.joint(i).jcalc(qstate.q, m_qdot_zero);
            m_Ic[i] =  m.body(i).I();
        }
    }
    else
    {
        for (int i = 1; i < N_B; ++i) 
        {
            m_Ic[i] =  m.body(i).I();
        }
    }
    
    // fill in the joint space inertia matrix
    for (int i = N_B - 1; i > 0; --i) 
    {
        const BJoint &joint_i = m.joint(i);
        
        int lambda       = m.parentId(i); 
        int dof_index_i  = joint_i.qindex();
        int dofCount     = joint_i.DoFCount();  
        
        const BTransform& X_lambda = joint_i.X_lambda(); 
        
        if (lambda != 0) 
        {
            // sum all children (composite) inertias
            m_Ic[lambda] += X_lambda.applyTranspose(m_Ic[i]);
        }
        
        if (dofCount == 1) 
        {
            const BVector6 S(joint_i.S());
            BVector6 F = m_Ic[i] * S;
            
            H[dof_index_i][dof_index_i] = arb::dot(S, F);
            
            int j = i;

            while (m.parentId(j) != 0) 
            {
                const BJoint &joint_j = m.joint(j);
                
                F = joint_j.X_lambda().applyTranspose(F); 
                j = m.parentId(j);
                int dof_index_j = joint_j.qindex();
                
                if (joint_j.DoFCount() == 1) 
                {
                    const BVector6 S_j(joint_j.S());
                    
                    H[dof_index_i][dof_index_j] = H[dof_index_j][dof_index_i] = arb::dot(F, S_j);
                } 
                else if (joint_j.DoFCount() == 3) 
                {
                    const BMatrix63 S_j(joint_j.S());
                    const BVector3 val =  arb::transpose(S_j) * F;
                    
                    block_1_3(H, dof_index_i, dof_index_j, val);
                    block_3_1(H, dof_index_j, dof_index_i, val); // transpose not needed here
                }
            }
        } 
        else if (dofCount == 3) 
        {
            const BMatrix63 S(joint_i.S());
            BMatrix63 F = m_Ic[i] * S;
          
            block_3_3(H, dof_index_i, dof_index_i, arb::transpose(S) * F);
            
            int j = i;
            
            while (m.parentId(j) != 0) 
            {
                const BJoint &joint_j = m.joint(j);
                
                const BTransform  &X_lambda = joint_j.X_lambda(); 
                F = arb::toForceInverse(X_lambda) * F; 
                //F = arb::transpose(X_lambda) * F; // this also works
                j = m.parentId(j);
                int dof_index_j = joint_j.qindex();
                
                if (joint_j.DoFCount() == 1) 
                {
                    const BVector6 S_j(joint_j.S());
                    const BVector3 val = arb::transpose(F) * S_j;
                    
                    block_3_1(H, dof_index_i, dof_index_j, val);
                    block_1_3(H, dof_index_j, dof_index_i, val);
                } 
                else if (joint_j.DoFCount() == 3) 
                {
                    const BMatrix63 S_j(joint_j.S());
                    //const BMatrix3 val = arb::transpose(F) * S_j;
                    const BMatrix3 val = arb::transpose(F.top()) * S_j.top() + 
                                         arb::transpose(F.bot()) * S_j.bot();
                    
                    block_3_3(H, dof_index_i, dof_index_j, val);
                    block_3_3(H, dof_index_j, dof_index_i, arb::transpose(val));
                }
            }
        } 
    }
}


