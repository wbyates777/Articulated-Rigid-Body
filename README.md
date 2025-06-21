
 ## Forward and Inverse Dynamics of Kinematic Trees.

 Trees (and chains) of kinematic equations are used in robotics, computer games, and animation.
 In robotics _forward kinematics_ refers to the use of kinematic equations to compute the position of 
 an end-effector, such as a jointed robotic arm, from specified values for the joint parameters.
 
  https://en.wikipedia.org/wiki/Forward_kinematics
  
 The reverse calculation, that computes the joint parameters that achieve a specified arm position, 
 is known as _inverse kinematics_.
 
 https://en.wikipedia.org/wiki/Inverse_kinematics

 BDynamics is an implementation of:
 - the articulated-body algorithm (ABA) and  
 - the recursive Newton-Euler algorithm (RNEA).

 Both of these algorithms are presented in:

 "Rigid Body Dynamics Algorithms" (RBDA), R. Featherstone, Springer, 2008 (see https://royfeatherstone.org). 

 ABA is an example of a propagation algorithm, and it is the fastest known algorithm
 for calculating the forward dynamics of a kinematic tree with a computational complexity of $O(N_B)$,
 where $N_B$ is the number of bodies/joints. 
 This is the theoretical minimum for solving the forward dynamics problem (see RBDA, Section 7.3). 
 
 RNEA calculate the inverse dynamics of a kinematic tree. 
 It is the simplest, most efficient known algorithm for trees, and also has a computational
 complexity of $O(N_B)$ (see RBDA, Section 5.3). 

 These algorithms are described and implememted using _spatial algebra_. 
 Spatial algebra  employs 6D vectors that combine the 3D linear and
 3D angular aspects of rigid-body motion.
 Linear and angular velocities or accelerations are
 combined to form  _spatial motion_ vectors, while forces and moments are combined
 to form  _spatial force_ vectors.
 Spatial algebra significantly reduces  the number of computational operations 
 required when  compared with standard 3D vector approaches (see RBDA, Section 1.2). 

 The implementations presented here, are intended for use in computer games, and are 
 based on those in the RBDL library (see https://github.com/rbdl/rbdl).
 We intentionally use similar variable names and the same object structure and hierarchy. 
 Some variables have been moved to their appropriate classes and accessor methods 
 have been added throughout. This improves encapsulation and readability.

 RBDL employs the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
 fixed-size matrices to arbitrarily large dense matrices, and even sparse matrices.
 This code does not depend on Eigen3, and instead relies on the lighter-weight GLM library 
 for simple 3D-linear algebra types and operations (see https://github.com/g-truc/glm). 
 As the GLM library does not support 6D vectors and matricies, the code for RBDL custom
 joint types is not implemented.

 This code depends on the 3D GLM types and functions: glm::dvec3, glm::dmat3, glm::dquat,
 glm::cross(v1, v2), glm::dot(v1, v2), glm::length(v1), glm::inverse(m1), glm::toMat3(q).
 
 It is straightforward to convert back to Eigen3 (although see 
 the note on Eigen3 and GLM differences), or  replace GLM with some other simple
 linear algebra library.

 
