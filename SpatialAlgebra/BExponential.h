/* BExponential 04/04/2026

 $$$$$$$$$$$$$$$$$$$$$$
 $   BExponential.h   $
 $$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History: 
 
 Spatial exponential and logarithmic maps for motion vectors.


 Given an arbitrary twist, he spatial transform $X = exp(v)$ represents 
 the integrated displacement over a unit time step Δt. 
 If $v$ is expressed in the base/world coordinate frame we have:
 
    X(t + Δt) = X(t) exp(vΔt)  
 
 or
 
    X(t + Δt) = exp(vΔt) X(t) 
 
 if $v$ is expressed in body frame coordinates.
 
 The log mapping is the inverse of exp. In symbols:
 
    log( exp(v) ) = v
 

 see:
 
 "Modern Robotics", Lynch and Park, 2017.
 "Lie Groups for 2D and 3D Transformations", Ethan Earle, 2017.
 
 Note
 
 1) we use the Featherstone convention on transforms, i.e X(E,r) as opposed to the homogenous convention X = (R,p),
    where E = R^T and r = -R^Tp
 2) arb::cross(w) * arb::cross(w) == arb::outer(w,w) - (arb::dot(w,w) * B_IDENTITY_3x3);
 3) velocity vectors v are elements of a Lie algebra, transforms X are elements of a Lie group, exp:LieAlg --> LieGrp,
    SE(3) lie group of rigid body transforms, se(3) tangent space at identity
 
*/



#ifndef __BEXPONENTIAL_H__
#define __BEXPONENTIAL_H__


#ifndef __BTRANSFORM_H__
#include "BTransform.h"
#endif



namespace arb {
    
    //
    // exp
    //

    inline BMatrix3 
    exp( const BVector3 &w )
    // best to keep angular variables in range [-M_PI_2, M_PI_2]
    {
        using std::sin;
        using std::cos;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I(B_IDENTITY_3x3);
        const BMatrix3 W = arb::cross(w);
        
        if (theta < 1E-8)
        {
            // Rodrigues series near zero: $exp(cross(w)) ≈ I + cross(w) + 1/2 cross(w)^2$
            return I + W + BScalar(0.5) * (W * W);
        }
        
        const BScalar A = sin(theta) / theta;
        const BScalar B = (1.0 - cos(theta)) / (theta * theta);
        
        return I + A * W + B * (W * W);
    }
    
    inline BMatrix3 
    leftJacobian( const BVector3 &w )
    {
        using std::sin;
        using std::cos;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I(B_IDENTITY_3x3);
        const BMatrix3 W = arb::cross(w);
        
        if (theta < 1E-8)
        {
            return I + BScalar(0.5) * W + BScalar(1.0 / 6.0) * (W * W);
        }
        
        const BScalar A = (1.0 - cos(theta)) / (theta * theta);
        const BScalar B = (theta - sin(theta)) / (theta * theta * theta);
        
        return I + A * W + B * (W * W);
    }

    inline BTransform 
    exp( const BVector6 &vec ) //  must be motion vector or twist
    {
        const BVector3 w = vec.ang();
        const BVector3 v = vec.lin();
        
        // rotation
        const BMatrix3 E = exp(w);
        
        // standard homogeneous translation
        const BMatrix3 J = leftJacobian(w);
        const BVector3 p = -(J * v); 
        
        // convert homogeneous translation p into Featherstone r
        const BVector3 r =  p;
        
        return BTransform(E, r);
    }


    //
    // log
    //
      
    inline BVector3 
    log( const BMatrix3 &R )
    {
        using std::acos;
        using std::sin;
        
        const BScalar cos_theta = arb::clamp(0.5 * (arb::trace(R) - 1.0), -1.0, 1.0);
        const BScalar theta = acos(cos_theta);
        //assert(!arb::isnan(theta));
        
        // near identity, use first-order approximation
        if (theta < 1E-10)
        {
            return arb::uncross(R - arb::transpose(R)) * BScalar(0.5);
        }
        
        // when theta near M_PI handle rotation singularity
        if (theta > BScalar(M_PI - 1E-3))
        {
            // use the column with the largest diagonal element to ensure numerical stability
            BVector3 u;
            if (R[0][0] >= R[1][1] && R[0][0] >= R[2][2]) 
            {
                u = BVector3(R[0][0] + BScalar(1.0), R[1][0], R[2][0]);
            } 
            else if (R[1][1] >= R[2][2]) 
            {
                u = BVector3(R[0][1], R[1][1] + BScalar(1.0), R[2][1]);
            } 
            else 
            {
                u = BVector3(R[0][2], R[1][2], R[2][2] + BScalar(1.0));
            }
            
            return arb::normalize(u) * theta;
        }
         
        const BMatrix3 W = (R - arb::transpose(R)) * (BScalar(0.5) * theta / sin(theta));
        return arb::uncross(W);
    }

    inline BMatrix3 
    leftJacobianInverse( const BVector3 &w )
    {
        using std::tan;
        
        const BScalar theta = arb::length(w);
        const BMatrix3 I = B_IDENTITY_3x3;
        const BMatrix3 W = arb::cross(w);
        
        if (theta < 1E-8)
        {
            return I - BScalar(0.5) * W + BScalar(1.0 / 12.0) * (W * W);
        }
        
        const BScalar half_theta = 0.5 * theta;
        const BScalar cot_half_theta = 1.0 / tan(half_theta);
        
        const BScalar A = (1.0 - half_theta * cot_half_theta);
        const BScalar B = BScalar(1.0) / (theta * theta);
        
        return I - BScalar(0.5) * W + (A * B) * (W * W);
    }
    
    inline BVector6 
    log( const BTransform &X )
    // return the spatial twist (motion vector) whose exponetial produces the given X
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
   
    
    inline BMatrix6 
    expm( const BMatrix6 &m )  { return toMotion(exp(uncross(m))); }

    inline BMatrix6 
    logm( const BTransform &X )  { return crossm(log(X)); }
    
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





