/*! 

         $$$$$$$$$$$$$$$$$
         $   BStdExp.h   $
         $$$$$$$$$$$$$$$$$

 @file BStdExp.h
 @ingroup Checks
 date 11/11/2026
 
 by W.B. Yates
 
 ---
 
 @brief Reference implementations of the standard SE(3) exponential and logarithmic maps for testing.
 
  These two functions are used to test the functions of BExponential.h.
  The std_exp function is very slow.
 
  Reference implementation of the standard SE(3) exponential.
  Sums the series
 
           $$ sum_k T^k / k!  for T = [[ [w]x, v ],[0, 0]] $$
 
  where
  w is the rotational velocity (an Euler vector),
  v is the linear velocity (translation component).
  The symbol [w]x is the usual skew-symmetric matrix (so [w]x u == w x u).
  Returns the 4x4 homogeneous matrix [R p; 0 1].
 
  Intended for testing only: it is deliberately independent of arb::cross(), arb::exp() and
  arb::leftJacobian(). Note that arb::cross(w) == -[w]x, hence
 
        toHomogeneous(arb::exp(v))  == std_exp(-v.ang(), -v.lin())
        toHomogeneous(arb::exp(-v)) == std_exp( v.ang(),  v.lin())
 
  Reference implementation of the standard SE(3) logarithm.
  Maps a 4x4 homogeneous transformation matrix T = [R p;  0 1]
  to its corresponding twist coordinates (w, v) in se(3).

        w = rotational vector (Euler vector),
        v = linear velocity (translation component).

  Uses closed-form analytical solutions (matrix logarithm for SO(3) and
  the inverse left Jacobian V^{-1}) with robust numerical branching
  for near-zero rotations (theta -> 0) and half-turn rotations (theta -> pi).

  Intended for testing only: serves as an exact analytical inverse to
  std_exp() for validation and  verification.
 

 @note Euler vector w - where length |w| < pi is angle of rotation, and axis = arb::normalize(w) is the axis of rotation.
 
  @see "Modern Robotics", Lynch and Park, 2017.
  @see "Lie Groups for 2D and 3D Transformations", Ethan Earle, 2017.
 
  @warning std_log produces errors at rotations close to M_PI_2.
 
*/


#ifndef BSTDEXP_H
#define BSTDEXP_H

#ifndef BSPATIALALGEBRA_H
#include "BSpatialAlgebra.h"
#endif

#include <string>
#include <vector>
#include <map>

namespace arb {

    /// Standard SE(3) exponential map.
    BMatrix4
    std_exp( const BVector6 &v, int terms = 40 );

    /// Standard SE(3) exponential map.
    BMatrix4
    std_exp( const BVector3 &w, const BVector3 &v, int terms );

    /// Standard SE(3) logarithmic map.
    BVector6
    std_log(const BMatrix4 &T);

    /// Standard SE(3) logarithmic map.
    void
    std_log(const BMatrix4 &T, BVector3 &w_out, BVector3 &v_out);
}

#endif


