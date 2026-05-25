# Articulated Rigid Body and OpenGJK

 Use these files in your build and you can switch out [libccd] and use the [openGJK] 
 variant of GJK instead.
 
  OpenGJK is a modern GJK implementation that is both faster and more numerically stable than libccd. 
  For a  discussion on how this variant improves upon the classic GJK algorithm, 
  see the  paper included in this subdirectory.
 
  OpenGJK has been rewritten with [GLM] to support [automatic differentiation] using [autodiff] types. 
  This makes it possible to differentiate across a contact boundary and calculate 
  precise post-collision  derivatives for interacting  bodies. 
 
 
This version of openGJK only supports polytopes (convex hulls); although boxes can be represented 
as 8-point polytopes, analytical primitives such as spheres are not natively supported.


Note that OpenGJK is released under a GPL3 license.
As the GPLv3 is a "strong copyleft" license, integrating these files into your build structure 
acts as a combined or derivative work. Consequently, if you use this module, 
the entire ARB codebase in your project becomes bound by the GPLv3 license, 
overriding the permissive nature of ARB's default MIT license. 

# Directory Contents

The specific OpenGJK project files are:

 BSimplex.h
 
 BCollider.h/.cpp
 
 BOpenGJK.h/.cpp
 
 BOpenEPA.h/.cpp
 
 BGJK.h
  
The files common to both collision managment modules are:
 
 ABody.h
 
 BBox.h

 BPolytope.h
 
 BContact.h
 
 BContactManager.h/.cpp
 

 ```
/*
 *                          _____      _ _  __
 *                         / ____|    | | |/ /
 *   ___  _ __   ___ _ __ | |  __     | | ' /
 *  / _ \| '_ \ / _ \ '_ \| | |_ |_   | |  <
 * | (_) | |_) |  __/ | | | |__| | |__| | . \
 *  \___/| .__/ \___|_| |_|\_____|\____/|_|\_\
 *       | |
 *       |_|
 *
 * Copyright 2022-2026 Mattia Montanari, University of Oxford
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. See https://www.gnu.org/licenses/
 */
 ```



[automatic differentiation]: https://en.wikipedia.org/wiki/Automatic_differentiation
[autodiff]: https://github.com/autodiff/autodiff
[GJK]: https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm
[openGJK]: https://github.com/MattiaMontanari/openGJK
[GLM]: https://github.com/g-truc/glm
[libccd]:  https://github.com/danfis/libccd








