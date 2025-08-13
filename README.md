
 ## Forward and Inverse Dynamics of Kinematic Trees.

 Trees (and chains) of kinematic equations are used in robotics, computer games, and animation.
 In robotics _forward kinematics_ refers to the use of kinematic equations to compute the position of 
 an end-effector, such as a jointed robotic arm, from specified values for the joint parameters
 (see  https://en.wikipedia.org/wiki/Forward_kinematics).

 The reverse calculation, that computes the joint parameters that achieve a specified arm position, 
 is known as _inverse kinematics_ (see https://en.wikipedia.org/wiki/Inverse_kinematics).

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
 Spatial algebra significantly reduces the
"volume of algebra by at least a factor of 4 compared with standard 3D vector notation" (see RBDA, Section 1.2). 
 For example, the following code excerpt performs Newton-Euler integration (see https://en.wikipedia.org/wiki/Newton–Euler_equations):

    // set up single body - a sphere -  
    double mass = 100.0, radius = 0.5; 
    BVector3 h(0.0);
    BMatrix3 I_o((2.0/5.0) * mass * (radius * radius)); 
    BRBInertia I(mass, h,  I_o);
    BSpatialMatrix invI = arb::inverse(I);

    // set initial position, velocity and acceleration
    BSpatialVector pos(B_ZERO_3, BVector3(20.0, 50.0, 3.0)), vel(B_ZERO_6), acc(B_ZERO_6);
    
    // apply an angular force of 1.0 around Y axis and a linear force of 100.0 along Z axis
    BSpatialVector force(0.0, 1.0, 0.0,  0.0, 0.0, 100.0);

    double T = 0.0, dt = 0.01;
    std::cout << T << " position is " << pos << std::endl;
    for (int t = 0; t < 500; ++t)
    {
        // Newton-Euler (see Featherstone, Section 2.15, eqn 2.72, page 36).
        acc = invI * (force - arb::crossf(vel, I * vel)); 
        vel += acc * dt;
        pos += vel * dt;
        T += dt;
    }
    std::cout << T << " position is " << pos << std::endl; // angles in radians!

 This produces the output:

    0.0 position is 0.00000000 0.00000000 0.00000000 20.00000000 50.00000000 3.00000000 
    5.0 position is 0.00000000 1.25250000 0.00000000 15.23282410 50.00000000 13.89482750 
    
 The implementations presented here, are intended for use in computer games, and are 
 based on those in the RBDL library (see https://github.com/rbdl/rbdl).
 Alternative implementations can be found in the RBDyn library (see https://github.com/jrl-umi3218/RBDyn).
 We intentionally use similar variable names and the same object structure and hierarchy as RBDL. 
 Some variables have been moved to their appropriate classes and accessor methods 
 have been added throughout. This facillitates comparison testing, and improves encapsulation and readability.
 

 RBDL and RBDyn employ the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
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

 
