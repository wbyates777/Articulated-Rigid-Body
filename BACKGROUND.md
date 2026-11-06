
# <img width="80" height="80" alt="icon" src="https://github.com/user-attachments/assets/7e7f598c-a0b1-47de-a18c-6572527ed229" />  Articulated Rigid Body (ARB) 


# Background

 ## Forward and Inverse Dynamics of Kinematic Trees

 Trees (and chains) of kinematic equations are used in robotics, computer graphics, and animation.
 In robotics [forward kinematics] refers to the use of kinematic equations to compute the position of 
 an end-effector, such as a jointed robotic arm, from specified values for the joint parameters
 (see figure 1).
 The reverse calculation, that computes the joint parameters that achieve a specified arm position, 
 is known as [inverse kinematics].

 <p align="center">
<img width="255" height="250" alt="robot_arm" src="https://github.com/user-attachments/assets/a51a097e-1480-4c3b-a724-2e5de4086daa"    />
</p>
 <p align="center">
 Figure 1.  A simple jointed robot arm.
 </p>

 The BDynamics class contains an implementation of:
 
 - the articulated-body algorithm (ABA), 
 - the composite rigid-body algorithm (CRBA), and  
 - the recursive Newton-Euler algorithm (RNEA).

 These algorithms are presented in:

 "Rigid Body Dynamics Algorithms" (RBDA), [R. Featherstone], Springer, 2008. 

 ABA is an example of a propagation algorithm, and it is the fastest known algorithm
 for calculating the forward dynamics of a kinematic tree with a computational complexity of $O(N_B)$,
 where $N_B$ is the number of bodies/joints. 
 This is the theoretical minimum for solving the forward dynamics problem (see RBDA, Section 7.3). 
 
 
 CRBA also calculates forward dynamics. In contrast to ABA, it forms an equation of motion for the whole system, 
 and then solves it for the acceleration variables. Although CRBA is $O(N_B^2)$ it can match or slightly
exceed the speed of the $O(N_B)$ ABA on trees with only a few bodies, or that are branched enough to have a small depth (see RBDA, Section 6.2). 

 RNEA calculates the inverse dynamics of a kinematic tree. 
 It is the simplest, most efficient known algorithm for trees, and also has a computational
 complexity of $O(N_B)$ (see RBDA, Section 5.3). 


<details>

<summary>Table: ARB - Supported Joint Types </summary>

 ARB supports all of the following joints:

 | Joint Type | DoF  | Description | Representation | 
| :---       | ---: | :---        | :---           |
| Fixed1     |  0   |  Compressible rigid connection between links.  |  Identity Transform |
| Fixed2     |  0   |  Incompressible rigid connection between links.  |  Identity Transform |
| Revolute   |  1   |  Rotation about a specified axis.  |  Scalar Angle  |
| Prismatic  |  1   |  Translation along a specified axis.  |  Scalar Displacement  |
| Helical    |  1   |  A 'screw' joint with both rotational and translational motion |  Scalar Pitch  |
| Spherical  |  3   |  Full 3D rotation (ball-and-socket).  |  Quaternion  |
| Planar     |  3   |  Translation in the x,y plane and rotation about z-axis  |  Vector3  |
| Floating Base  |  6  | Unconstrained 3D motion of the root.  |  Vector3 + Quaternion |

</details>




## Spatial Algebra

 The algorithms described above are formulated and implemented using _spatial algebra_ (see RBDA, Chapter 2). 
 Spatial algebra  employs 6D vectors that combine the 3D linear and
 3D angular aspects of rigid-body motion.
 Linear and angular velocities (or accelerations) are
 combined to form  _spatial_ _motion_ vectors, while linear forces and torques are combined
 to form  _spatial_ _force_ vectors.  By convention, spatial velocity vectors are called _twists_, 
 while spatial force vectors are called _wrenches_.
 
 Spatial algebra significantly reduces the
"volume of algebra by at least a factor of 4 compared with standard 3D vector notation" (see RBDA, Section 1.2). 
 It also significantly reduces the complexity of the implementation of rigid body and articulated rigid body physics.
 For example, the following simple 'hello world' program demonstrates [Newton-Euler] integration:
 ```cpp
    #include "BSpatialAlgebra.h"
     
    int 
    main( void )
    {
       // set up single body - a sphere -  
       double mass = 100.0, radius = 0.5; 
       BMatrix3 I_o((2.0/5.0) * mass * (radius * radius)); 
      
       BRBInertia I( BInertia(mass, I_o) );
       BMatrix6 invI = arb::inverse(I);
   
       // set initial position, velocity and acceleration (angular, linear)
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
       return EXIT_SUCCESS;
    }
 ```
 This produces the output:

    0.0 position is 0.00000000 0.00000000 0.00000000 20.00000000 50.00000000 3.00000000 
    5.0 position is 0.00000000 1.25250000 0.00000000 15.23282410 50.00000000 13.89482750 

 The acceleration due to an actual, physical force is calculated by the term ```acc = invI * force```.
 The extra term ```arb::crossf(vel, I * vel)``` is called the bias force.
 The bias force represents an [inertial force]; a so called fictitious force such as centrifugal, Coriolis, or Euler force. 
 The inertial force is necessary for describing motion correctly.
 
 ## Automatic Differentiation 

 This library also supports [Automatic Differentiation] (AD).
 Adding automatic differentiation to the spatial algebra library means that 
 the algebra, ABA, CRBA, RNEA, and collision resolution are fully differentiable. 
 Unlike _numerical_ differentiation (finite differences), which is computationally expensive and prone to truncation errors,
 AD uses the [chain rule] to propagate  _analytical_ derivatives through the code at the machine level.
 This end-to-end differentiability facilitates the application of more advanced optimisation and machine learning techniques
 such as real-time [Model Predictive Control] (MPC), gradient-based trajectory optimisation, or analytical [System Identification]  (SI).
   
 
For example, consider  _System Identification_ (SI).
System Identification is the process of constructing a mathematical model of a dynamic system, 
using analytical methods, based on observed input and output data. 
While forward dynamics (ABA/CRBA) predicts how a system moves given its physical parameters, 
SI reverses this process: 
it can recover the underlying physical parameters (mass, centre-of-mass, inertia tensor) of each body 
by observing the system's motion (see ARB example 6 in main.cpp).
SI allows researchers to synchronise a simulation with the real world.
Additionally, SI can be used for the estimation of non-conservative forces such as joint friction and damping, 
which are traditionally difficult to calibrate manually.

## Collision Detection and Impulse Based Resolution

 The task of detecting collisions between rigid bodies is non-trivial. It is typically subdivided into two phases:  _broad phase_ and  _narrow phase_.
 Broad-phase consists of detecting intersections between oriented bounding boxes (OBB) and axis aligned boxes (AABB). 
 Computationally, this is relatively efficient.
 The narrow-phase detects intersections between mesh colliders represented by _polytopes_ (convex hulls) 
 using the [GJK algorithm]. This is very precise but computationally more expensive.
 

 ARB provides two options for narrow-phase collision detection: 
 - the default libccd backend,  or
 - an implementation of openGJK.
   
Libccd is a well-tested C library, with  support for standard primitives such as boxes and spheres, as well as complex arbitrary shapes represented as polytopes (convex hulls).
It is released under a permissive license, compatible with the MIT license used here. However it does not use differentiable types, and so, it is not possible to differentiate _across a contact_, that is, to compute post-collision derivatives for interacting bodies.
The openGJK C++ implementation presented here does employ differentiable types.
However it only supports polytopes; although boxes can be represented as 8-point polytopes, analytical primitives like spheres are not natively supported.
Furthermore, openGJK is released under the GPLv3 license, and using this module will subject the entire project to GPLv3 copyleft terms.

 Although the openGJK implementation  can be used to "differentiate across a contact", the contact derivatives may exhibit jump discontinuities due to discrete contact switching, friction clamping, or simplex transitions. Applications that require strictly continuous gradients must implement more advanced techniques such as soft contact or gradient-smoothing techniques.

 Collisions are resolved by calculating the _spatial impulses_ resulting from a contact and using these impulses to update the object's velocities
 so that they separate appropriately (see RBDA, Section 11.7). 

The BContactManager class utilises an iterative Projected Gauss-Seidel (PGS) solver to resolve multiple contact constraints simultaneously. It combines Baumgarte Stabilisation for overlap recovery with a circular friction cone (Coulomb friction) model, providing  more realistic sliding and sticking behaviour than a box-friction approximation.
It also uses impulse caching (_warm starting_) across frames to ensure stability and eliminate _jitter_ in resting or stacked objects.

https://youtu.be/gKXwwdfl8QE



## Unified Robot Description Format (URDF)

  The Unified Robot Description Format (URDF) is an XML-based industry standard used to define the physical configuration of a robot, 
  acting as the universal language for the [Robot Operating System] (ROS) ecosystem. It specifies  a 
  robot’s kinematic tree, including the hierarchy of links, the types of joints connecting them, and their respective physical properties such as mass, centre of mass, and inertia tensors.
  
  By supporting URDF, ARB can load real-world robot models, such as the [ur5] or tiago, directly from standard industry files. 
  The library parses these XML descriptions to automatically construct the internal spatial inertia matrices and joint transforms required 
  for the ABA, CRBA and RNEA algorithms. These models can also be exported to other URDF compliant systems for cross-validation.
 
 
 ## Libraries
 
   ### Dependencies
   
 glm       - https://github.com/g-truc/glm
 
 autodiff  - https://github.com/autodiff/autodiff
 
 libccd    - https://github.com/danfis/libccd
 
 openGJK   - https://github.com/MattiaMontanari/openGJK

 TinyXML2  - https://github.com/leethomason/tinyxml2
 
  ### External 
  
 RBDL      - https://github.com/rbdl/rbdl
 
 RBDyn     - https://github.com/jrl-umi3218/RBDyn
 
 Pinocchio - https://stack-of-tasks.github.io/pinocchio/

 [forward kinematics]: https://en.wikipedia.org/wiki/Forward_kinematics
 [inverse kinematics]: https://en.wikipedia.org/wiki/Inverse_kinematics
 [ur5]: https://www.youtube.com/watch?v=35KhfiAmE4o
 [GJK algorithm]: https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
 [R. Featherstone]:  https://royfeatherstone.org
 [Robot Operating System]: https://en.wikipedia.org/wiki/Robot_Operating_System
 [Newton-Euler]: https://en.wikipedia.org/wiki/Newton–Euler_equations
 [inertial force]:  https://en.wikipedia.org/wiki/Fictitious_force
 [Automatic Differentiation]: https://en.wikipedia.org/wiki/Automatic_differentiation 
 [chain rule]: https://en.wikipedia.org/wiki/Chain_rule
 [System Identification]:  https://en.wikipedia.org/wiki/System_identification
 [Model Predictive Control]: https://en.wikipedia.org/wiki/Model_predictive_control
 [TinyXML2]: https://github.com/leethomason/tinyxml2
 

 
