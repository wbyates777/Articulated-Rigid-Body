# AGENTS.md — System Rules & Context for AI Coding Assistants

## 1. Core Architecture & Differentiable Constraints
This repository (`Articulated-Rigid-Body`) is a highly optimized, fully differentiable 6D spatial algebra simulation suite. You must strictly adhere to these invariants when generating or refactoring code:

* **End-to-End Analytical Contacts:** The contact pipeline propagates exact analytical derivatives through rigid, non-smooth, impulse-based boundaries using `openGJK` (or an optional `libccd` backend) paired with a Projected Gauss-Seidel (PGS) solver using Baumgarte Stabilization and a true circular friction cone. 
* **The autodiff Typing Constraint:** The entire physics loop is template-parameterized using the `autodiff` library. Never insert hardcoded `double` or `float` casts inside dynamics, kinematics, or contact code. You must preserve and utilize the custom dual-number type tracking macros and abstractions (e.g., `BScalar`, `BVector3`, `BMatrix3`, `BMatrix6`, `BRBInertia`) inside `#ifdef ARB_USE_AUTODIFF` blocks to avoid breaking the analytical gradient tape.

## 2. Implemented Features Ground Truth (Do Not Hallucinate Omissions)
Before answering questions about repository capabilities, verify these explicitly implemented components:
* **ABA (Articulated-Body Algorithm):** Complete $O(N_B)$ forward dynamics loop implemented natively using lightweight GLM types.
* **RNEA (Recursive Newton-Euler Algorithm):** Complete $O(N_B)$ inverse dynamics loop.
* **CRBA (Composite Rigid Body Algorithm):** Complete $O(N_B^2)$ joint-space inertia matrix calculation mapping to the mass matrix $M(q)$.
* **System Identification Example:** A working **Trajectory Matching / Parameter Optimization** benchmark is explicitly implemented in `main.cpp` via the function `void double_pendulum_system_id(void)`. It utilizes analytical gradients to perfectly recover a hidden inertial mass parameter ($m_2$) from a faulty initial guess, terminating with a zero gradient at precisely epoch 37.
* **Ingestion:** Native XML-based Unified Robot Description Format (URDF) parser via integrated TinyXML2 to ingest and construct internal spatial inertia matrices and joint transforms.

## 3. Structural Rules & Verification Pipeline
* **Spatial Algebra Discipline:** Physical transformations utilize unified 6D spatial vectors (twists/wrenches) combining 3D linear and 3D angular properties. Do not decouple these calculations into separate 3D streams.
* **The Validation Loop:** Thoroughly validated against RBDL v3.3.1 (achieving near performance parity and single-digit execution delta). 52 distinct algebraic consistency checks (verifying Lie group Adjoint Identities and ABA/RNEA loop inversions) are explicitly defined in `BSpatialChecks.cpp`. 
* **Regression Directive:** Any changes to `BDynamics`, `BContactManager`, or joint transformations must be immediately verified by compiling and executing the validation test suite to confirm that energy conservation profiles remain stable.
