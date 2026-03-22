# <img width="80" height="80" alt="icon" src="https://github.com/user-attachments/assets/7e7f598c-a0b1-47de-a18c-6572527ed229" /> Articulated Rigid Body (ARB)

**Compact C++ articulated rigid body simulation library** implementing:

- Forward dynamics via the Articulated Body Algorithm (ABA) —  $O(N_B)$,  
- Inverse dynamics via the Recursive Newton–Euler Algorithm (RNEA) -  $O(N_B)$, 
- Collision detection and spatial impulse-based contact resolution.  

Includes **end-to-end automatic differentiation**, enabling system identification, optimization, and machine learning applications.

---

### Demo

**Rigid body simulation using collision detection and contact resolution:**

https://youtu.be/g1jMEpu1sl8

---

## Overview

ARB is a compact implementation of spatial algebra and rigid body dynamics, designed for use in computer graphics, simulation, and robotics.

It combines:
- Spatial algebra (6D motion and force vectors)
- Efficient rigid body dynamics algorithms (ABA, RNEA)
- Collision detection and physically consistent contact resolution  
- Automatic differentiation across the full simulation pipeline  

This allows simulation of articulated systems ranging from simple mechanisms to fully-jointed characters, with support for advanced applications such as system identification and model-based control.

## Introduction

ARB is a compact implementation of _spatial_ _algebra_ with end-to-end automatic differentiability (AD). 

The spatial algebra is used to implement two important rigid body dynamics algorithms: articulated-body algorithm (ABA) 
and the recursive Newton-Euler algorithm (RNEA)  (see Featherstone 2008).
The algebra is also used to implement a _spatial impulse_ based, collision resolution algorithm.
All three algorithms have beed tested extensively in a graphics environment.

The implementations of ABA and RNEA presented here are based on those in the well known robotics library RBDL, but are intended for use in computer graphics.
They allow a programmer to handle, in a physically consistent manner, rigid bodies and articulated rigid bodies.
In the context of graphics these articulated bodies could range from a fully-jointed humanoid character, 
to a simple hinged mechanism such as a door. 
 
 Spatial algebra combines the 3D-linear and 3D-angular components of rigid body physics
 and provides a compact notation that significantly reduces the 
"volume of algebra by at least a factor of 4 compared with standard 3D vector notation" (Featherstone 2008).
 Spatial algebra also significantly reduces the complexity of the implementation.
 
 Automatic differentiation is provided by the autodiff library.
 Thus the algebra, the ABA,  the RNEA, and collision resolution are completely differentiable. 
 This end-to-end differentiability facilitates the application of 
  advanced optimisation, machine learning techniques, and System Identification (SI).

  The BContactManager class can detect collisions between (pairs of) rigid bodies and can resolves any 'contacts' 
 by employing the physical concept of spatial impulse.  

 
This code is written in c++23 and depends primarily on STL and the header-only GLM. 
Additionally, the header-only autodiff library is required for automatic differentiablity, and 
 the libccd shared library for collision detection.


### Key Features

* Articulated-body algorithm (ABA) - $O(N_B)$ forward dynamics for kinematic trees,
* Recursive Newton-Euler algorithm (RNEA) - $O(N_B)$ inverse dynamics for kinematic trees,
* Spatial algebra implementation (header-only),
* End-to-end automatic differentiability using autodiff (header-only),
* Collision Resolution –  spatial impulses and GJK/EPA contact manifolds.
* Minimal dependencies STL, GLM, (autodiff and libccd optional).

## Background

 ### Forward and Inverse Dynamics of Kinematic Trees

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

### Collision Detection and Impulse Based Resolution


 Collison detection can be subdivided into _broad phase_ and  _narrow phase_.
 Broad-phase consists of detecting intersections between bounding boxes (BB) using 
 the Separating Axis Theorem (SAT). Computationally, this is relatively efficient.
 See https://en.wikipedia.org/wiki/Hyperplane_separation_theorem
 The narrow-phase detects intersections between mesh colliders represented by _polytopes_ (convex hulls) 
 using the GJK algorithm. This is very precise but computationally more expensive.
 See https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm.

 Collisons are resolved by calculating the
 _spatial impulses_ resulting from a contact and using these impulses to update the objects velocities
 so that they separate appropriately. (see RBDA, Section 11.7).

 https://youtu.be/g1jMEpu1sl8

### Spatial Algebra

 The algorithms described above are described and implemented using _spatial algebra_ (see RBDA, Chapter 2). 
 Spatial algebra  employs 6D vectors that combine the 3D linear and
 3D angular aspects of rigid-body motion.
 Linear and angular velocities (or accelerations) are
 combined to form  _spatial_ _motion_ vectors, while linear forces and torques are combined
 to form  _spatial_ _force_ vectors.  By convention, spatial velocity vectors are called _twists_, 
 while spatial force vectors are called _wrenches_.
 
 Spatial algebra significantly reduces the
"volume of algebra by at least a factor of 4 compared with standard 3D vector notation" (see RBDA, Section 1.2). 
 it also significantly reduces the complexity of the implementation of rigid body and articulated rigid body physics.
 For example, the following code excerpt performs Newton-Euler integration (see https://en.wikipedia.org/wiki/Newton–Euler_equations):

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

 This produces the output:

    0.0 position is 0.00000000 0.00000000 0.00000000 20.00000000 50.00000000 3.00000000 
    5.0 position is 0.00000000 1.25250000 0.00000000 15.23282410 50.00000000 13.89482750 

 The acceleration due to an actual, physical force is calculated by the term ```acc = invI * force```.
 The extra term ```arb::crossf(vel, I * vel)``` is called the bias force.
 The bias force represents an _inertial_ force; a so called fictitious force such as centrifugal, Coriolis, or Euler force. 
 The inertial force is necessary for describing motion correctly (see https://en.wikipedia.org/wiki/Fictitious_force).
 
 ### Automatic Differentiation 

 This library also supports _Automatic_ _Differentiation_ (AD).
 Adding automatic differentiation to the spatial algebra library means that 
 the algebra, the ABA, the RNEA, and collision resolution are completely differentiable. 
 Unlike _numerical_ differentiation (finite differences), which is computationally expensive and prone to truncation errors,
 AD uses the chain rule to propagate exact _analytical_ derivatives through the code at the machine level.
 This end-to-end differentiability facilitates the application of more advanced optimisation and machine learning techniques
 such as real-time Model Predictive Control (MPC), gradient-based trajectory optimisation, or analytical System Identification (SI)
   (see  https://en.wikipedia.org/wiki/Automatic_differentiation). 
 
For example, consider  _System Identification_ (SI).
System Identification is the process of constructing a mathematical model of a dynamic system, 
using analytical methods, based on observed input and output data. 
While forward dynamics (ABA) predicts how a system moves given its physical parameters, 
SI reverses this process: 
it can recover the underlying physical parameters (mass, center-of-mass, inertia tensor) of each body 
by observing the system's motion (see ARB Example 6 in main.cpp).
SI allows researchers to synchronise a simulation with the real world.
Additionally, SI can be used for the estimation of non-conservative forces such as joint friction and damping, 
which are traditionally difficult to calibrate manually.


  
## Implementation 

 The implementations presented here, are intended for use in computer graphics, and are 
 based on those in the RBDL library.
 Alternative implementations can be found in the RBDyn library, and the Pinocchio library (see below for links).
 We intentionally use similar variable names and the same object structure and hierarchy as RBDL. 
 This facilitates numerical comparison testing. 
 Some variables have been moved to their appropriate classes and accessor methods 
 have been added throughout. This improves encapsulation and readability.
 

 RBDL, RBDyn, and Pinocchio employ the Eigen3 linear algebra library. Eigen3 supports all matrix sizes, from small 
 fixed-size matrices to arbitrarily large dense matrices, and even sparse matrices.
 This code does not depend on Eigen3, and instead relies on the light-weight, header-only GLM library. 
 GLM (OpenGL Mathematics) is  based on the OpenGL Shading Language (GLSL) specifications, and
 provides a highly optimised implementation of 3D linear algebra primitives. 
 
 The spatial algebra implementation (though not the algorithms) is also header-only, and depends solely on STL and the 3D GLM types: 
 
 ```glm::vec3, glm::mat3, glm::quat```, 
 
 and the GLM functions:
 
 ```glm::cross(v1,v2), glm::dot(v1,v2), glm::outerProduct(v1,v2), glm::transpose(m), glm::inverse(m), glm::mat3_cast(q)```.
 
 It is straightforward to convert back to Eigen3 (although see 
 the note on Eigen3's and GLM's row-major, column-major differences), or  replace GLM with some other simple
 linear algebra library.

 In order to implement automatic differentiation it is sufficient to #include "BAutodiff.h" in the BSpatialTypes.h file.
 BAutodiff.h is a wrapper for the header-only autodiff library (see below).
 This enables the automatic computation of derivatives in an efficient and intuitive manner.
 It should be noted that the calculated derivatives are
 exact (to machine precision) and not approximated, as is the case for finite difference methods.

The collision detection component relies on the external library _libccd_ (see below).
libccd is a C library for a collision detection between two convex shapes. It implements a variant 
of the Gilbert–Johnson–Keerthi algorithm and the Expanded Polytope Algorithm (EPA). 
 
 ## Correctness and Validation
 
 This implementation has been numerically validated against RBDL v3.3.1 and tested extensively in a graphics environment. The results of test example 1 included in main.cpp and taken from the RBDL documentation are shown below. Notice that the forward and inverse dynamics results match to 8 dp:

   | Test | Value | RBDL | ARB |
| :--- | :--- | :--- | :--- |
| **Forward** | accel. | `-6.54000000 6.54000000 0.00000000` | `-6.54000000 6.54000000 0.00000000` |
| **Inverse** | tau | `2.67999109 -4.18995273 0.00000000` | `2.67999109 -4.18995273 0.00000000` |

 Furthermore, Checks/BSpatialChecks.cpp also includes 36 consistency tests that verify basic analytical relationships and identities. These checks cover spatial transforms, cross products and inertia operations, ensuring algebraic self-consistency.
 
 ## Build Instructions
 

On a platform that supports cmake you can use the CMakeList.txt file included in this project. Simply cd to the directory where you have saved this project and:

 ```mkdir build```
 
 ```cd build```
 
 ```cmake ..```
 
 ```make```

Cmake will take care of installing the GLM, autodiff, and libccd libaries. 
If you are not using cmake these libraries can be downloaded directly from github (see links below).

The minimum compiler requirement is now c++23. This is due to the improved constexpr handling in c++23. If you do not use GLM_FORCE_INTRINSICS you can use c++20.
If you remove (some) constexpr definitions, then my code will compile under c++20, and 'almost' compile under c++17. GLM and autodiff are both c++17 compliant.  See the source for details on the remaining minor c++17 changes. 

 ## Libraries
 
   ### Dependencies
   
 GLM       - https://github.com/g-truc/glm
 
 autodiff  - https://github.com/autodiff/autodiff
 
 CCD       - https://github.com/danfis/libccd
 
  ### External 
  
 RBDL      - https://github.com/rbdl/rbdl
 
 RBDyn     - https://github.com/jrl-umi3218/RBDyn
 
 Pinocchio - https://stack-of-tasks.github.io/pinocchio/
 
## Notes

Please send any questions or report any errors, omissions, or suggested extensions to the email wbyates777@gmail.com. 

