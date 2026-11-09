/*!

         $$$$$$$$$$$$$$$$$$$$$$
         $   BExponential.h   $
         $$$$$$$$$$$$$$$$$$$$$$

 @file BExponential.h
 @ingroup SpatialAlgebra
 date 04/04/2026
 
 by W.B. Yates
 
 ---
 
 @brief Exponential and logarithmic maps between motion vectors (twists) and spatial transforms.

 For a motion vector $v = (w, v_lin)$, the spatial transform $X = exp(v)$ has the form

                  E = exp(w)              (Rodrigues' formula)
                  r = -J(w) v_lin         (J is the left Jacobian of SO(3))

 in the BTransform convention (see BTransform.h). The log mapping is the inverse of exp:

                  log( exp(v) ) = v       whenever the rotation angle |w| < pi

 For larger angles the rotation wraps, and log returns the equivalent vector with |w| <= pi.
 

 Lie group terminology:
    - SE(3): Lie group of rigid body transforms (BTransform),
    - se(3): its Lie algebra, the tangent space at the identity (motion vectors or twists),
    - SO(3): Lie group of rotations (BMatrix3), and so(3) its Lie algebra,
    - exp: se(3) --> SE(3)
    - log: SE(3) --> se(3).
 

 @note $arb::cross(w) * arb::cross(w) == arb::outer(w,w) - (arb::dot(w,w) * B_IDENTITY_3x3)$;
 @note Euler vector w - where length |w| < pi is angle of rotation, and direction axis = arb::normalize(w) is the axis of rotation.
 
 @see "Modern Robotics", Lynch and Park, 2017.
 @see "Lie Groups for 2D and 3D Transformations", Ethan Earle, 2017.
 @see https://en.wikipedia.org/wiki/3D_rotation_group

*/



#ifndef BEXPONENTIAL_H
#define BEXPONENTIAL_H


#ifndef BTRANSFORM_H
#include "BTransform.h"
#endif

#ifndef BADJOINT_H
#include "BAdjoint.h"
#endif

namespace arb {
    
    //
    // exp
    //
    
    /// Rotation exponential so(3) -> SO(3) via Rodrigues' formula.
    inline BMatrix3
    exp( const BVector3 &w )
    {
        using std::sin;
        using std::cos;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I(B_IDENTITY_3x3);
        const BMatrix3 W = arb::cross(w);
        
        if (theta < 1E-8)
        {
            // second-order Taylor series: exp([w]x) ~ I + [w]x + 1/2 [w]x^2
            return I + W + B_HALF * (W * W);
        }
        
        const BScalar A = sin(theta) / theta;
        const BScalar B = (1.0 - cos(theta)) / (theta * theta);
        
        return I + A * W + B * (W * W);
    }
    
    /// Left Jacobian of SO(3)
    inline BMatrix3
    leftJacobian( const BVector3 &w )
    {
        using std::sin;
        using std::cos;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I(B_IDENTITY_3x3);
        const BMatrix3 W = arb::cross(w);
        
        // Taylor series: I + 1/2 [w]x + 1/6 [w]x^2
        if (theta < 1E-8)
        {
            return I + B_HALF * W + BScalar(1.0 / 6.0) * (W * W);
        }
        
        const BScalar A = (1.0 - cos(theta)) / (theta * theta);
        const BScalar B = (theta - sin(theta)) / (theta * theta * theta);
        
        return I + A * W + B * (W * W);
    }

    /// Exponential map exp : se(3) -> SE(3) where v is a motion vector (twist)
    inline BTransform
    exp( const BVector6 &v )
    {
        const BVector3 w = v.ang();
        const BVector3 l = v.lin();
        
        // rotation
        const BMatrix3 E = exp(w);
        
        const BMatrix3 J = leftJacobian(w);
        const BVector3 r = -(J * l);
        
        return BTransform(E, r);
    }
    
    //
    // log
    //
    

    /// Logarithm log : SO(3) -> so(3). Returns the rotation vector with |w| in [0, pi].
    inline BVector3
    log( const BMatrix3 &R )
    {
        using std::atan2;
        
        const BVector3 a = arb::uncross(R - arb::transpose(R)) * B_HALF;   // sin θ * axis
        const BScalar  s = arb::length(a);                                       // sin θ
        const BScalar  c = B_HALF * (arb::trace(R) - B_ONE);               // cos θ
        const BScalar theta = atan2(s, c);                                       // accurate everywhere
        
        if (theta < 1E-8)
            return a * (B_ONE + theta * theta / BScalar(6));   // θ / sin θ ≈ 1 + θ²/6
        
        if (theta < BScalar(M_PI - 1E-3))
            return a * (theta / s);
        
        // Near pi, sin(theta) is too small to give the axis reliably. Recover the
        // axis from the symmetric part, S = cos(t) I + (1 - cos t) a a^T, take the
        // column with the largest diagonal entry for stability, and use the
        // antisymmetric part to choose the sign.
        const BMatrix3 S = B_HALF * (R + arb::transpose(R));
        const BMatrix3 N = (S - c * B_IDENTITY_3x3) * (B_ONE / (B_ONE - c)); // = a aᵀ
       
        BVector3 u;
        if (N[0][0] >= N[1][1] && N[0][0] >= N[2][2])
        {
            u = BVector3(N[0][0], N[1][0], N[2][0]);
        }
        else if (N[1][1] >= N[2][2])
        {
            u = BVector3(N[0][1], N[1][1], N[2][1]);
        }
        else
        {
            u = BVector3(N[0][2], N[1][2], N[2][2]);
        }
        
        u = arb::normalize(u);
        if (arb::dot(u, a) < 0)
            u = -u;    // use the antisymmetric part to fix the sign
        
        return u * theta;
    }

    /// Inverse of leftJacobian(w), using the closed form with cot(theta/2).
    inline BMatrix3
    leftJacobianInverse( const BVector3 &w )
    {
        using std::tan;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I = B_IDENTITY_3x3;
        const BMatrix3 W = arb::cross(w);
        
        if (theta < 1E-8)
        {
            return I - B_HALF * W + BScalar(1.0 / 12.0) * (W * W);
        }
        
        const BScalar half_theta = B_HALF * theta;
        const BScalar cot_half_theta = B_ONE / tan(half_theta);
        
        const BScalar A = (1.0 - half_theta * cot_half_theta);
        const BScalar B = B_ONE / (theta * theta);
        
        return I - B_HALF * W + (A * B) * (W * W);
    }
    
    /// Logarithm map log : SE(3) -> se(3): returns the motion vector v with exp(v) == X.
    inline BVector6
    log( const BTransform &X )
    {
        const BMatrix3 &R = X.E();
        const BVector3  p =  -X.r(); 
        
        // angular part
        const BVector3 w = log(R);
        
        // linear part - v = J^{-1}(w) * p
        const BMatrix3 Jinv = leftJacobianInverse(w);
        const BVector3 v = Jinv * p;
        
        return BVector6(w, v);
    }
   
    /// Matrix exponential of a 6x6 motion generator m = crossm(v).
    inline BMatrix6
    expm( const BMatrix6 &m )  { return arb::toMotion(exp(uncross(m))); }

    /// Matrix logarithm of a transform X. Returns the 6x6 motion generator crossm(log(X)).
    inline BMatrix6
    logm( const BTransform &X )  { return arb::crossm(log(X)); }
    
    /// Matrix logarithm of a 6x6 motion transform matrix X (as returned by BMatrix6(BTransform).
    inline BMatrix6
    logm( const BMatrix6 &X ) 
    {
        const BMatrix3 E = X.topLeft();
        const BMatrix3 Z = X.botLeft() * arb::transpose(E);
        const BVector3 r = -arb::uncross(Z);
        return arb::logm(BTransform(E, r));
    }

}

#endif





