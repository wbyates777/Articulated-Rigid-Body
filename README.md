
 ## Forward and Inverse Dynamics of Kinematic Trees

 Trees (and chains) of kinematic equations are used in robotics, computer graphics, and animation.
 In robotics _forward kinematics_ refers to the use of kinematic equations to compute the position of 
 an end-effector, such as a jointed robotic arm, from specified values for the joint parameters
 (see  https://en.wikipedia.org/wiki/Forward_kinematics).

 The reverse calculation, that computes the joint parameters that achieve a specified arm position, 
 is known as _inverse kinematics_ (see https://en.wikipedia.org/wiki/Inverse_kinematics).

 BDynamics is an implementation of:
 
 - the articulated-body algorithm (ABA), and  
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

## Spatial Algebra

 The algorithms are described and implememted using _spatial algebra_ (see RBDA, Chapter 2). 
 Spatial algebra  employs 6D vectors that combine the 3D linear and
 3D angular aspects of rigid-body motion.
 Linear and angular velocities (or accelerations) are
 combined to form  _spatial_ _motion_ vectors, while linear forces and torques are combined
 to form  _spatial_ _force_ vectors.  By convention, spatial velocity vectors are called _twists_, 
 while spatial force vectors are called _wrenches_.
 
 Spatial algebra significantly reduces the
"volume of algebra by at least a factor of 4 compared with standard 3D vector notation" (see RBDA, Section 1.2). 
 For example, the following code excerpt performs Newton-Euler integration (see https://en.wikipedia.org/wiki/Newton–Euler_equations):

    // set up single body - a sphere -  
    double mass = 100.0, radius = 0.5; 
    BMatrix3 I_o((2.0/5.0) * mass * (radius * radius)); 
   
    BRBInertia I( BInertia(mass, I_o) );
    BMatrix6 invI = arb::inverse(I);

    // set initial position, velocity and acceleration
    BVector6 pos(B_ZERO_3, 20.0, 50.0, 3.0), vel(0.0), acc(0.0);
    
    // apply an angular force of 1.0 around Y axis and a linear force of 100.0 along Z axis
    BVector6 force(0.0, 1.0, 0.0,  0.0, 0.0, 100.0);

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

 The acceleration due to an actual, physical force is calculated by the term ```acc = invI * force```.
 The extra term ```arb::crossf(vel, I * vel)``` is called the bias force.
 The bias force represents an _inertial_ force; a so called fictitious force such as centrifugal, Coriolis, or Euler force. 
 The inertial force is necessary for describing motion correctly (see https://en.wikipedia.org/wiki/Fictitious_force).
 
## Implementation 

 The implementations presented here, are intended for use in computer graphics, and are 
 based on those in the RBDL library.
 Alternative implementations can be found in the RBDyn library, and the Pinocchio library (see below for links).
 We intentionally use similar variable names and the same object structure and hierarchy as RBDL. 
 This facillitates numerical comparison testing. 
 Some variables have been moved to their appropriate classes and accessor methods 
 have been added throughout. This improves encapsulation and readability.
 

 RBDL, RBDyn, and Pinocchio employ the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
 fixed-size matrices to arbitrarily large dense matrices, and even sparse matrices.
 This code does not depend on Eigen3, and instead relies on the light-weight, header-only GLM library. 
 GLM (OpenGL Mathematics) provides a highly optimized implementation of 3D linear algebra primitives. 
 
 The spatial algebra implementation (though not the algorithms) is also header only, and depends solely on STL and the 3D GLM types: 
 
 ```glm::vec3, glm::mat3, glm::quat```, 
 
 and the GLM functions:
 
 ```glm::cross(v1, v2), glm::dot(v1, v2), glm::outerProduct(v1, v2), glm::transpose(m), glm::inverse(m), glm::mat3_cast(q)```.
 
 It is straightforward to convert back to Eigen3 (although see 
 the note on Eigen3's and GLM's row-major, column-major differences), or  replace GLM with some other simple
 linear algebra library.

 ## Build Instructions

On a platform that supports cmake you can use the CMakeList.txt file included in this project. Simply cd to the directory where you have saved this project and:

 ```mkdir build```
 
 ```cd build```
 
 ```cmake ..```
 
 ```make```

 ## Libraries
 
 GLM       - https://github.com/g-truc/glm
 
 RBDL      - https://github.com/rbdl/rbdl
 
 RBDyn     - https://github.com/jrl-umi3218/RBDyn
 
 Pinocchio - https://stack-of-tasks.github.io/pinocchio/
 
