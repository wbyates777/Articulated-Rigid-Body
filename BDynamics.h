/* BDynamics 20/02/2024

 $$$$$$$$$$$$$$$$$$$
 $   BDynamics.h   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Algorithms for calculating the forward and inverse dynamics of a kinematic tree.
 
 Trees (and chains) of kinematic equations are used in robotics, computer games, and animation.
 In robotics {\it forward kinematics} refers to the use of kinematic equations to compute the position of 
 an end-effector, such as a jointed robotic arm, from specified values for the joint parameters.
 
 https://en.wikipedia.org/wiki/Forward_kinematics
 
 The reverse calculation, that computes the joint parameters that achieve a specified arm position, 
 is known as {\it inverse kinematics}.

 https://en.wikipedia.org/wiki/Inverse_kinematics

 BDynamics is an implementation of 
 i) the articulated-body algorithm (ABA) and  
 ii) the recursive Newton-Euler algorithm (RNEA), 
 presented in: 
 "Rigid Body Dynamics Algorithms" (RBDA), R. Featherstone, Springer, 2008 (see https://royfeatherstone.org). 
 
 ABA is an example of a propagation algorithm, and it is the fastest known algorithm
 for calculating the forward dynamics of a kinematic tree with a computational complexity of $O(N_B)$,
 where $N_B$ is the number of bodies/joints. 
 This is the theoretical minimum for solving the forward dynamics problem (see RBDA, Section 7.3). 
 
 RNEA calculate the inverse dynamics of a kinematic tree. 
 It is the simplest, most efficient known algorithm for trees, and also has a computational complexity 
 of $O(N_B)$ (see RBDA, Section 5.3). 
 
 The implementations presented here are based on those in the RBDL library ( see https://github.com/rbdl/rbdl ).
 
 The implemenations here do NOT depend on Eigen3, instead they depend soly on the 
 GLM linear algebra library ( see https://github.com/g-truc/glm ).
  
 It ie straightforward to convert back to Eigen3 (although see 
 the note on Eigen3 and GLM differences), or  replace GLM with some other simple
 linear algebra library.
*/


#ifndef __BDYNAMICS_H__
#define __BDYNAMICS_H__

#ifndef __BMODEL_H__
#include "BModel.h"
#endif


struct BModelState 
{
    // joint parameters (joint space)
    std::vector<BScalar> q;     // positions
    std::vector<BScalar> qdot;  // velocities
    std::vector<BScalar> qddot; // accelerations
    std::vector<BScalar> tau;   // forces
};

typedef  std::vector<BSpatialVector> BExtForce;

class BDynamics
{
    
public:
    
    BDynamics( int expected_dof = 3 );
    ~BDynamics( void )=default;
    
    /** \brief Computes forward dynamics with the Articulated Body algorithm (ABA)
     *
     * This function computes the generalized accelerations $a$ from given
     * generalized states, velocities and forces:
     *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, \dot{q}) + \tau)\f$
     * It does this by using the recursive Articulated Body Algorithm that runs
     * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
     *
     * \param m      rigid body model
     * \param qstate state  of the internal joints (positions, velocities, accelerations)
     * \param f_ext  External forces acting on the body in base coordinates (optional, defaults to empty)
     */
    void
    forward( BModel &m, BModelState &qstate, const BExtForce &f_ext = BExtForce() ); 
    
    
    /** \brief Computes inverse dynamics with the recursive Newton-Euler algorithm (RNEA)
     *
     * This function computes the generalized forces $\tau$ from given generalized
     * states, velocities, and accelerations:
     *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}, f_\textit{ext}) \f$
     *
     * \param m      rigid body model
     * \param qstate state  of the internal joints (positions, velocities, accelerations)
     * \param f_ext  External forces acting on the body in base coordinates (optional, defaults to empty)
     */
    void  
    inverse( BModel &m, BModelState &qstate, const BExtForce &f_ext = BExtForce()); 
    
    
    
    
    // update kinematics - calculates positions
    void 
    update_X_base( BModel &m, BModelState &qstate );

    // update kinematics - calculates velocities
    void 
    update_velocity( BModel &m, BModelState &qstate );
    
private: 

    // temporary variables U_i, d_i and u_i (see  RBDA, equations 7.43, 7.44, and 7.45)
    std::vector<BSpatialVector> m_U; // $U_i = I_i^A  S_i$ 
    std::vector<BScalar> m_d;        // $d_i = S_i^T U_i$
    std::vector<BScalar> m_u;        // $u_i = \tau_i − S_i^T p_i^A$
   

    // temporary variables for joints with 3 degrees of freedom
    std::vector<BMatrix63> m_dof3_U;
    std::vector<BMatrix3>  m_dof3_Dinv;
    std::vector<BVector3>  m_dof3_u;
 
};

#endif


