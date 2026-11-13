/* BStdExp 11/11/2026

 $$$$$$$$$$$$$$$$$$$
 $   BStdExp.cpp   $
 $$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 
 ---

*/


#ifndef BSTDEXP_H
#include "BStdExp.h"
#endif

#include <cmath>
#include <iostream>


namespace arb {

 
    BMatrix4
    std_exp( const BVector3 &w, const BVector3 &v, int terms )
    {
        // T = [[ [w]x, v ], [0, 0]]  (column j of [w]x is w x e_j)
        BMatrix4 T(B_ZERO_4x4);
        for (int j = 0; j < 3; ++j)
        {
            BVector3 e(B_ZERO_3);
            e[j] = B_ONE;
            T[j] = BVector4(arb::cross(w, e), B_ZERO);
        }
        T[3] = BVector4(v, B_ZERO);
        
        BMatrix4 result = B_IDENTITY_4x4;   // sum_k T^k / k!
        BMatrix4 term   = B_IDENTITY_4x4;
        for (int k = 1; k <= terms; ++k)
        {
            term = (term * T) * (B_ONE / BScalar(k));
            result += term;
        }
        return result;
    }

    BMatrix4
    std_exp( const BVector6 &v, int terms )
    {
        return std_exp(v.ang(), v.lin(), terms);
    }


    //
    //
    //

    inline BVector3
    skew_to_vector(const BMatrix3 &skew)
    {
        return BVector3(skew[1][2], skew[2][0], skew[0][1]);
    }


    // NB the transpose of arb::cross in BProduct.h
    inline BMatrix3
    vector_to_skew(const BVector3 &w)
    {
        return BMatrix3(
            BVector3(B_ZERO, w[2], -w[1]),   // Column 0
            BVector3(-w[2], B_ZERO, w[0]),   // Column 1
            BVector3(w[1], -w[0], B_ZERO)    // Column 2
        );
    }

    // precision log function: SE(3) -> Twist (w, v)
    // code errors at rotations close to M_PI_2 - method could be improved. 
    void
    std_log(const BMatrix4 &T, BVector3 &w_out, BVector3 &v_out)
    {
        using std::acos;
        using std::sin;
        using std::cos;
        using std::tan;
        using std::sqrt;
        using std::abs;

        const BScalar EPSILON_ZERO = BScalar(1E-8);
        const BScalar EPSILON_PI   = BScalar(1E-7);

        BMatrix3 R(T);
        BVector3 t(T[3]);

        // extract rotation angle theta from R
        BScalar cos_theta = (arb::trace(R) - B_ONE) * B_HALF;
        BScalar theta = acos(arb::clamp(cos_theta, -B_ONE, B_ONE));

        // near zero rotation (theta ≈ 0)
        if (theta < EPSILON_ZERO)
        {
            BMatrix3 skew_w = R - B_IDENTITY_3x3;
            w_out = skew_to_vector(skew_w);
            
            BMatrix3 V_inv = B_IDENTITY_3x3 - B_HALF * skew_w;
            v_out = V_inv * t;
        }
        else if (abs(theta - M_PI) < EPSILON_PI)
        {
            // half-turn rotation (theta ≈ π) — prevents division by zero in sin(theta)
            
            // find the column of (R + I) with the largest norm
            BMatrix3 M = R + B_IDENTITY_3x3;
            int max_col = 0;
            BScalar max_norm_sq = -B_ONE;
            for (int j = 0; j < 3; ++j)
            {
                BScalar norm_sq = arb::length2(M[j]);
                if (norm_sq > max_norm_sq)
                {
                    max_norm_sq = norm_sq;
                    max_col = j;
                }
            }
            // extract new axis
            BVector3 axis = M[max_col] / sqrt(max_norm_sq);
            
            w_out = axis * theta;

            // reconstruct [w]x for V_inv calculation
            BMatrix3 skew_w = vector_to_skew(w_out);
            
            // near pi, the coefficient simplifies and avoids 0/0
            BScalar coeff = (B_ONE / (theta * theta));
            BMatrix3 V_inv = B_IDENTITY_3x3 - B_HALF * skew_w + coeff * (skew_w * skew_w);
            v_out = V_inv * t;
        }
        else
        {
            // general analytical case (0 < theta < π)
            BScalar sin_theta = sin(theta);
            BScalar factor = theta / (BScalar(2.0) * sin_theta);

            BMatrix3 skew_w = factor * (R - arb::transpose(R));
            w_out = skew_to_vector(skew_w);

            // nNumerically stable evaluation of V^(-1) coefficient using tan(theta/2)
            // identity: (1 + cos_theta) / (2 * theta * sin_theta) = 1 / (2 * theta * tan(theta / 2))
            BScalar coeff = (B_ONE / (theta * theta)) - (B_ONE / (BScalar(2.0) * theta * tan(theta * B_HALF)));

            BMatrix3 v_inv = B_IDENTITY_3x3 - B_HALF * skew_w + coeff * (skew_w * skew_w);
            v_out = v_inv * t;
        }
    }

    BVector6
    std_log( const BMatrix4 &R )
    {
        BVector3 ang, lin;
        std_log(R, ang, lin);
        return BVector6(ang,lin);
    }


}
