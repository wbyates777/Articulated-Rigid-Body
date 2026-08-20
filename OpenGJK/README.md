# Articulated Rigid Body and OpenGJK

 Use these files in your build and you will switch out libccd and use the openGJK 
 variant of GJK instead.
 
 See the included paper for a discussion as to how openGJK improves on the basic GJK agorithm.
 
 This implementation employs differentiable (autodiff) types and can be used to "differentiate across a contact", that is, to compute post-collision derivatives for interacting bodies. 
   However, the contact derivatives may exhibit jump discontinuities due to discrete contact switching, friction clamping, or simplex transitions. 
   Applications that require strictly continuous gradients  must implement more advanced techniques such as soft contact or gradient-smoothing techniques.

This version of openGJK only supports polytopes; although boxes can be represented as 8-point polytopes, analytical primitives such as spheres are not natively supported.

It should be noted that GJK/EPA is known to perform badly with aligned flat surfaces, such as boxes stacked on top of one another.  In these cases, professional collision management systems employ specialist box on box routines.


Note that OpenGJK is released under a GPL3 license. As a result if this code is used 
  then *all* the ARB code-base in this project is also bound by the GPL3 license.
 

---
 

 * Copyright 2022-2026 Mattia Montanari, University of Oxford
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. See https://www.gnu.org/licenses/

