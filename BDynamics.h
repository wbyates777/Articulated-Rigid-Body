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
  
 It should be relatively straightforward to convert back to Eigen3 (although see 
 the note on Eigen3 and GLM differences), or  replace GLM with some other simple
 linear algebra library.
*/


#ifndef __BDYNAMICS_H__
#define __BDYNAMICS_H__

#ifndef __BMODEL_H__
#include "BModel.h"
#endif


class BDynamics
{
    
public:
    
    BDynamics( void );
    ~BDynamics( void )=default;
    
    
    /** \brief Computes forward dynamics with the Articulated Body algorithm (ABA)
     *
     * This function computes the generalized accelerations from given
     * generalized states, velocities and forces:
     *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, \dot{q}) + \tau)\f$
     * It does this by using the recursive Articulated Body Algorithm that runs
     * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
     *
     * \param model rigid body model
     * \param q     state vector of the internal joints
     * \param qdot  velocity vector of the internal joints
     * \param tau   actuations of the internal joints
     * \param qddot accelerations of the internal joints (output)
     * \param f_ext External forces acting on the body in base coordinates (optional, defaults to empty)
     */
    void 
    forward( BModel &model,
            const std::vector<BScalar> &q,
            const std::vector<BScalar> &qdot,
            const std::vector<BScalar> &tau,
            std::vector<BScalar>       &qddot, // output 
            const std::vector<BSpatialVector> &f_ext = std::vector<BSpatialVector>() ); 
    
    
    /** \brief Computes inverse dynamics with the recursive Newton-Euler algorithm (RNEA)
     *
     * This function computes the generalized forces from given generalized
     * states, velocities, and accelerations:
     *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}, f_\textit{ext}) \f$
     *
     * \param model rigid body model
     * \param q     state vector of the internal joints
     * \param qdot  velocity vector of the internal joints
     * \param qddot accelerations of the internals joints
     * \param tau   actuations of the internal joints (output)
     * \param f_ext External forces acting on the body in base coordinates (optional, defaults to empty)
     */
    void 
    inverse( BModel &model, 
            const std::vector<BScalar> &q,
            const std::vector<BScalar> &qdot,
            const std::vector<BScalar> &qddot,
            std::vector<BScalar> &tau, // output 
            const std::vector<BSpatialVector> &f_ext = std::vector<BSpatialVector>());
    
private:
    
    // the function implemented here is $m = a * b^T$
    inline const BSpatialMatrix
    mulT( const BSpatialVector& a, const BSpatialVector& b ) const;
    
    // temporary variables U_i, d_i and u_i (see  RBDA, equations 7.43, 7.44, and 7.45)
    std::vector<BSpatialVector> m_U; // $U_i = I_i^A  S_i$ 
    std::vector<BScalar> m_d;        // $d_i = S_i^T U_i$
    std::vector<BScalar> m_u;        // $u_i = \tau_i − S_i^T p_i^A$
    // velocity-dependent spatial acceleration term $c_i$ (see RBDA, equation 7.35)
    std::vector<BSpatialVector> m_c; // $c_i = c_J + v_i \cross v_J$
    
    // temporary variables for joints with 3 degrees of freedom
    std::vector<BMatrix63> m_dof3_U;
    std::vector<BMatrix3>  m_dof3_Dinv;
    std::vector<BVector3>  m_dof3_u;
    
    // Eigen3 VectoNd stuff here for custom joints 
    // std::vector<Math::MatrixNd> m_dof6_U;
    // std::vector<Math::MatrixNd> m_dof6_Dinv;
    // std::vector<Math::VectorNd> m_dof6_u;
};

#endif


