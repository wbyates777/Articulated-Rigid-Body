/* BOpenGJK 15/12/2025

 $$$$$$$$$$$$$$$$$$$$
 $   BOpenGJK.cpp   $
 $$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:


*/

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


#ifndef __BOPENGJK_H__
#include "BOpenGJK.h"
#endif

#ifndef __BSPATIALTYPES_H__
#include "BSpatialTypes.h"
#endif

#ifndef __BCOLLIDER_H__
#include "BCollider.h"
#endif



#include <iostream>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp> 

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>



void 
BOpenGJK::S1D( BSimplex &smp, BVector3 &v ) const
{
    const BVector3 &s1p = smp[1];
    const BVector3 &s2p = smp[0];
    
    if (hff1(s1p, s2p)) 
    {
        projectOnLine(s1p, s2p, v);
    } 
    else 
    {
        v = smp[1];                        
        smp.vertexNum(1);                            
        smp.vertexMove(0, 1);
    }
}

void 
BOpenGJK::S2D( BSimplex &smp, BVector3 &v ) const
{
    const BVector3& s1p = smp[2];
    const BVector3& s2p = smp[1];
    const BVector3& s3p = smp[0];
    
    const int hff1f_s12 = hff1(s1p, s2p);
    const int hff1f_s13 = hff1(s1p, s3p);
    
    if (hff1f_s12) 
    {
        const int hff2f_23 = !hff2(s1p, s2p, s3p);
        if (hff2f_23)
        {
            if (hff1f_s13)
            {
                const int hff2f_32 = !hff2(s1p, s3p, s2p);
                if (hff2f_32) 
                {
                    projectOnPlane(s1p, s2p, s3p, v);  // update. s, no need to update c
                    return;                            // return V{1,2,3}
                } 
                else 
                {
                    projectOnLine(s1p, s3p, v);  // update. v
                    smp.vertexNum(2);                         
                    smp.vertexMove(1,2);
                    return;                      // return V{1,3}
                }
            }
            else 
            {
                projectOnPlane(s1p, s2p, s3p, v);  // update. s, no need to update c
                return;                            // return V{1,2,3}
            }
        } 
        else 
        {
            projectOnLine(s1p, s2p, v);  // update. v            
            smp.vertexNum(2);                          
            smp.vertexMove(0, 2);
            return;                      // return V{1,2}
        }
    } 
    else if (hff1f_s13) 
    {
        const int hff2f_32 = !hff2(s1p, s3p, s2p);
        if (hff2f_32) 
        {
            projectOnPlane(s1p, s2p, s3p, v);  // update. s, no need to update v
            return;                            // return V{1,2,3}
        } 
        else 
        {
            projectOnLine(s1p, s3p, v);  // update. v             
            smp.vertexNum(2);                         
            smp.vertexMove(1, 2);
            return;                      // return V{1,3}
        }
    } 
    else 
    {
        v = smp[2];                      
        smp.vertexNum(1);                          
        smp.vertexMove(0, 2);
        return;        // return V{1}
    }
}


void 
BOpenGJK::S3D( BSimplex &smp, BVector3 &v ) const
{
    const BGJKVertex& s1 = smp[3]; // order is correct - sic
    const BGJKVertex& s2 = smp[2];
    const BGJKVertex& s3 = smp[1];
    const BVector3  s4 = smp[0];
 
    // calculateEdgeVector
    const BVector3 s1s2 = s2.vertex - smp[3].vertex;
    const BVector3 s1s3 = s3.vertex - smp[3].vertex;
    const BVector3 s1s4 = s4 - smp[3].vertex;
    
    
    int hff1_tests[3];
    hff1_tests[2] = hff1(s1, s2);
    hff1_tests[1] = hff1(s1, s3);
    hff1_tests[0] = hff1(s1, s4);
    int testLineThree = hff1(s1, s3);
    int testLineFour  = hff1(s1, s4);
    
    int dotTotal = hff1(s1, s2) + testLineThree + testLineFour;
    if (dotTotal == 0) 
    {   
        v = s1;                       
        smp.vertexNum(1);                  
        smp[0] = s1;
        return;
    }
    
    const BScalar det134 = arb::determinant(BMatrix3(s1s3, s1s4, s1s2));
    int sss = (det134 <= 0);
    
    int testPlaneTwo  = hff3(s1, s3, s4) - sss;
    testPlaneTwo *= testPlaneTwo;
    
    int testPlaneThree = hff3(s1, s4, s2) - sss;
    testPlaneThree *= testPlaneThree;
    
    int testPlaneFour = hff3(s1, s2, s3) - sss;
    testPlaneFour *= testPlaneFour;
    
    int i, j, k;
    BGJKVertex si, sj, sk;
    
    switch (testPlaneTwo + testPlaneThree + testPlaneFour) 
    {
        case 3:
            v = B_ZERO_3;        
            smp.vertexNum(4);
            break;
            
        case 2:
            // Only one facing the origin
            // 1,i,j, are the indices of the points on the triangle and remove k from
            // simplex
            smp.vertexNum(3);
            if (!testPlaneTwo) 
            {  
                // k = 2;  // removes s2
                smp.vertexMove(2, 3);
            } 
            else if (!testPlaneThree) 
            {  
                // k = 1; // removes s3
                smp[1] = s2;
                smp.vertexMove(2, 3);
            } 
            else if (!testPlaneFour) 
            {  
                // k = 0; // removes s4  and no need to reorder
                smp[0] = s3;
                smp[1] = s2;
                smp.vertexMove(2, 3);
            }
            S2D(smp, v);
            break;
        case 1:
            // Two triangles face the origins:
            // The only positive hff3 is for triangle 1,i,j, therefore k must be in
            // the solution as it supports the the point of minimum norm.
            
            // 1,i,j, are the indices of the points on the triangle and remove k from
            // simplex
            smp.vertexNum(3);
            if (testPlaneTwo) 
            {
                k = 2; i = 1; j = 0; // s2
            }
            else if (testPlaneThree) 
            {
                k = 1; i = 0; j = 2; // s3
            } 
            else 
            {
                k = 0; i = 2; j = 1; // s4
            }
            
            si = smp[i]; sj = smp[j]; sk = smp[k];
            
            if (dotTotal == 1) 
            {
                if (hff1_tests[k]) 
                {
                    if (!hff2(s1, sk, si)) 
                    {
                        smp.select(si, sk);
                        projectOnPlane(s1, si, sk, v);
                    }
                    else if (!hff2(s1, sk, sj)) 
                    {
                        smp.select(sj, sk);
                        projectOnPlane(s1, sj, sk, v);
                    }
                    else 
                    {
                        smp.select(sk);
                        projectOnLine(s1, sk, v);
                    }
                } 
                else if (hff1_tests[i]) 
                {
                    if (!hff2(s1, si, sk)) 
                    {
                        smp.select(si, sk);
                        projectOnPlane(s1, si, sk, v);
                    } 
                    else 
                    {
                        smp.select(si);
                        projectOnLine(s1, si, v);
                    }
                } 
                else 
                {
                    if (!hff2(s1, sj, sk)) 
                    {
                        smp.select(sj, sk);
                        projectOnPlane(s1, sj, sk, v);
                    } 
                    else
                    {
                        smp.select(sj);
                        projectOnLine(s1, sj, v);
                    }
                }
            } 
            else if (dotTotal == 2) 
            {
                // Two edges have positive hff1, meaning that for two edges the origin's
                // project fall on the segement.
                // Certainly the edge 1,k supports the the point of minimum norm, and
                // so hff1_1k is positive
                
                if (hff1_tests[i]) 
                {
                    if (!hff2(s1, sk, si)) 
                    {
                        if (!hff2(s1, si, sk)) 
                        {
                            smp.select(si, sk);
                            projectOnPlane(s1, si, sk, v);
                        } 
                        else 
                        {
                            smp.select(sk);
                            projectOnLine(s1, sk, v);
                        }
                    } 
                    else 
                    {
                        if (!hff2(s1, sk, sj)) 
                        {
                            smp.select(sj, sk);
                            projectOnPlane(s1, sj, sk, v);
                        } 
                        else 
                        {
                            smp.select(sk);
                            projectOnLine(s1, sk, v);
                        }
                    }
                } 
                else if (hff1_tests[j]) 
                {  
                    if (!hff2(s1, sk, sj)) 
                    {
                        if (!hff2(s1, sj, sk)) 
                        {
                            smp.select(sj, sk);
                            projectOnPlane(s1, sj, sk, v);
                        } 
                        else 
                        {
                            smp.select(sj);
                            projectOnLine(s1, sj, v);
                        }
                    } 
                    else 
                    {
                        if (!hff2(s1, sk, si)) 
                        {
                            smp.select(si, sk);
                            projectOnPlane(s1, si, sk, v);
                        } 
                        else 
                        {
                            smp.select(sk);
                            projectOnLine(s1, sk, v);
                        }
                    }
                } 
            } 
            else if (dotTotal == 3) 
            {
                int hff2_ik = hff2(s1, si, sk);
                int hff2_jk = hff2(s1, sj, sk);
                int hff2_ki = hff2(s1, sk, si);
                int hff2_kj = hff2(s1, sk, sj);
                
                if (hff2_ki == 0 && hff2_kj == 0) 
                {
                    std::cout << "\n\n UNEXPECTED VALUES!!! \n\n" << std::endl;
                }
                if (hff2_ki == 1 && hff2_kj == 1)
                {
                    smp.select(sk);
                    projectOnLine(s1, sk, v);
                } 
                else if (hff2_ki) 
                {
                    if (hff2_jk) 
                    {
                        smp.select(sj);
                        projectOnLine(s1, sj, v);
                    }
                    else 
                    {
                        smp.select(sj, sk);
                        projectOnPlane(s1, sk, sj, v);
                    }
                } 
                else
                {
                    if (hff2_ik) 
                    {
                        smp.select(si);
                        projectOnLine(s1, si, v);
                    } 
                    else
                    {
                        smp.select(si, sk);
                        projectOnPlane(s1, sk, si, v);
                    }
                }
            }
            break;
            
        case 0:
            // The origin is outside all 3 triangles
            if (dotTotal == 1) 
            {
                // Here si is set such that hff(s1,si) > 0
                if (testLineThree) 
                {
                    k = 2; i = 1; j = 0;  // s3
                } 
                else if (testLineFour) 
                {
                    k = 1; i = 0; j = 2; // s3   
                } 
                else 
                {
                    k = 0; i = 2; j = 1;  // s2
                }
                
                si = smp[i]; sj = smp[j]; sk = smp[k];
                
                if (!hff2(s1, si, sj)) 
                {
                    smp.select(si, sj);
                    projectOnPlane(s1, si, sj, v);
                } 
                else if (!hff2(s1, si, sk)) 
                {
                    smp.select(si, sk);
                    projectOnPlane(s1, si, sk, v);
                } 
                else 
                {
                    smp.select(si);
                    projectOnLine(s1, si, v);
                }
            } 
            else if (dotTotal == 2) 
            {
                // Here si is set such that hff(s1,si) < 0
                smp.vertexNum(3);
                if (!testLineThree) 
                {
                    k = 2; i = 1; j = 0;  // s3  
                }
                else if (!testLineFour) 
                {
                    k = 1; i = 0;  j = 2; // s4
                }
                else 
                {
                    k = 0; i = 2; j = 1; // s2   
                }
                
                si = smp[i];  sj = smp[j]; sk = smp[k];
                
                if (!hff2(s1, sj, sk)) 
                {
                    if (!hff2(s1, sk, sj)) 
                    {
                        smp.select(sj, sk);
                        projectOnPlane(s1, sj, sk, v);
                    } 
                    else if (!hff2(s1, sk, si)) 
                    {
                        smp.select(si, sk);
                        projectOnPlane(s1, sk, si, v);
                    } 
                    else 
                    {
                        smp.select(sk);
                        projectOnLine(s1, sk, v);
                    }
                } 
                else if (!hff2(s1, sj, si))
                {
                    smp.select(si, sj);
                    projectOnPlane(s1, si, sj, v);
                } 
                else 
                {
                    smp.select(sj);
                    projectOnLine(s1, sj, v);
                }
            }
            break;
        default:
            std::cout << "\nBOpenGJK::ERROR:\tunhandled" << std::endl;
    }
}


void 
BOpenGJK::subalgorithm(BSimplex &smp, BVector3 &v) const
{
    switch (smp.vertexNum())
    {
        case 4:
            S3D(smp, v);
            break;
        case 3:
            S2D(smp, v);
            break;
        case 2:
            S1D(smp, v);
            break;
        default:
            std::cout << "\nBOpenGJK::ERROR:\t invalid simplex\n" << std::endl;
    }
}

void 
BOpenGJK::W0D(const ABody *body1, const ABody *body2, BSimplex &smp) const
{
    //smp.witness(0) = body1[smp[0][B1]]; // w00
    //smp.witness(1) = body2[smp[0][B2]]; // w01
    
    smp.witness(B1) = point(body1, smp[0][B1]);// w00
    smp.witness(B2) = point(body2, smp[0][B2]); // w01
}

void 
BOpenGJK::W1D(const ABody *body1, const ABody *body2, BSimplex &smp) const
{
    const BVector3 &p = smp[0];
    const BVector3 &q = smp[1];
    
    const BVector3 pq = q - p;
    const BVector3 po = -p;
    
    // Compute barycentric coordinates via matrix inversion
    // (in the linear case the matrix is 1x1 thus simplified)
    const BScalar det = arb::dot(pq, pq);
    
    if (arb::nearZero(det, 1E-5))
    {
        // Degenerate case
        W0D(body1, body2, smp); 
        return;
    }
    
    const BScalar a1 = arb::dot(pq, po) / det;
    const BScalar a0 = BScalar(1.0) - a1;

    // Compute witness points
    const BVector3 w00 = point(body1, smp[0][B1]);
    const BVector3 w01 = point(body2, smp[0][B2]);
    const BVector3 w10 = point(body1, smp[1][B1]);
    const BVector3 w11 = point(body2, smp[1][B2]);
    
    smp.witness(B1) = w00 * a0 + w10 * a1;
    smp.witness(B2) = w01 * a0 + w11 * a1; 
}

void 
BOpenGJK::W2D(const ABody *body1, const ABody *body2, BSimplex &smp) const
/**
 *  Compute barycentric coordinates via matrix inversion
 *  Given the points $P$, $Q$, and $R$ forming a triangle
 *  we want to find the barycentric coordinates of the origin
 *  projected onto the triangle. We can do this
 *  by inverting $\mathbf{T}$ in the linear equation below:
 *
 *  \begin{align*}
 *  \mathbf{T}
 *  \begin{bmatrix}
 *  \lambda_q \\
 *  \lambda_r
 *  \end{bmatrix} &= \begin{bmatrix}
 *  \overrightarrow{PQ}\cdot\overrightarrow{PO} \\
 *  \overrightarrow{PR}\cdot\overrightarrow{PO}
 *  \end{bmatrix} \\
 *  \lambda_p &= 1 - \lambda_q - \lambda_r \\
 *  \mathbf{T} &= \begin{bmatrix}
 *  \overrightarrow{PQ}\cdot\overrightarrow{PQ} &
 * \overrightarrow{PR}\cdot\overrightarrow{PQ} \\
 *  \overrightarrow{PR}\cdot\overrightarrow{PQ} &
 * \overrightarrow{PR}\cdot\overrightarrow{PR}
 *  \end{bmatrix}
 *  \end{align*}
 */
{
    const BVector3 &p = smp[0];
    const BVector3 &q = smp[1];
    const BVector3 &r = smp[2];

    const BVector3 pq = q - p;
    const BVector3 pr = r - p;
    const BVector3 po = -p;

    const BScalar T00 = arb::dot(pq, pq);
    const BScalar T01 = arb::dot(pq, pr);
    const BScalar T11 = arb::dot(pr, pr);
    const BScalar det = T00 * T11 - T01 * T01;
    
    if (arb::nearZero(det, 1E-5))
    {
        // Degenerate case
        W1D(body1, body2, smp);
        return;
    }

    const BScalar invDet = BScalar(1.0) / det;
    const BScalar b0 = arb::dot(pq, po);
    const BScalar b1 = arb::dot(pr, po);
    const BScalar I00 = T11 * invDet;
    const BScalar I01 = -T01 * invDet;
    const BScalar I11 = T00 * invDet;
    const BScalar a1 = I00 * b0 + I01 * b1;
    const BScalar a2 = I01 * b0 + I11 * b1;
    const BScalar a0 = BScalar(1.0) - a1 - a2;

    // check if the origin is very close to one of the edges of the
    // simplex. In this case, a 1D projection will be more accurate.
    if (a0 < B_EPS) 
    {
        smp.vertexNum(2);
        smp.vertexMove(0,2);
        W1D(body1, body2, smp);
    }
    else if (a1 < B_EPS) 
    {
        smp.vertexNum(2);
        smp.vertexMove(1,2);
        W1D(body1, body2, smp);
    } 
    else if (a2 < B_EPS) 
    {
        smp.vertexNum(2);
        W1D(body1, body2, smp);
    }

    // Compute witness points
    // This is done by blending the source points using
    // the barycentric coordinates
    const BVector3 w00 = point(body1, smp[0][B1]);
    const BVector3 w01 = point(body2, smp[0][B2]);
    const BVector3 w10 = point(body1, smp[1][B1]);
    const BVector3 w11 = point(body2, smp[1][B2]);
    const BVector3 w20 = point(body1, smp[2][B1]);
    const BVector3 w21 = point(body2, smp[2][B2]);

    smp.witness(B1) = w00 * a0 + w10 * a1 + w20 * a2;
    smp.witness(B2) = w01 * a0 + w11 * a1 + w21 * a2;
}

void 
BOpenGJK::W3D(const ABody *body1, const ABody *body2, BSimplex &smp) const
/**
 *  Compute barycentric coordinates via matrix inversion
 *  Given the points $P$, $Q$, and $R$, and $S$ forming a
 *  tetrahedron we want to find the barycentric coordinates of
 *  the origin. We can do this by inverting $\mathbf{T}$ in the
 *  linear equation below:
 *
 *  \begin{align*}
 *  \mathbf{T}
 *  \begin{bmatrix}
 *  \lambda_q \\
 *  \lambda_r \\
 *  \lambda_s
 *  \end{bmatrix} &= \begin{bmatrix}
 *  \overrightarrow{PQ}\cdot\overrightarrow{PO} \\
 *  \overrightarrow{PR}\cdot\overrightarrow{PO} \\
 *  \overrightarrow{PS}\cdot\overrightarrow{PO}
 *  \end{bmatrix} \\
 *  \lambda_p &= 1 - \lambda_q - \lambda_r - \lambda_s \\
 *  \mathbf{T} &= \begin{bmatrix}
 *  \overrightarrow{PQ}\cdot\overrightarrow{PQ} &
 * \overrightarrow{PQ}\cdot\overrightarrow{PR} &
 * \overrightarrow{PQ}\cdot\overrightarrow{PS}\\
 *  \overrightarrow{PR}\cdot\overrightarrow{PQ} & \overrightarrow{PR} \cdot
 * \overrightarrow{PR} & \overrightarrow{PR}\cdot\overrightarrow{PS} \\
 *  \overrightarrow{PS}\cdot\overrightarrow{PQ} &
 * \overrightarrow{PS}\cdot\overrightarrow{PR} &
 * \overrightarrow{PS}\cdot\overrightarrow{PS}
 *  \end{bmatrix}
 *  \end{align*}
 */
{
    const BVector3 &p = smp[0];
    const BVector3 &q = smp[1];
    const BVector3 &r = smp[2];
    const BVector3 &s = smp[3];
    
    const BVector3 pq = q - p;
    const BVector3 pr = r - p;
    const BVector3 ps = s - p;
    const BVector3 po = -p;
    
    // the equivalent compact code 
    // is slower as 9 dot products instead of 6
    // glm::dmat3 T = glm::dmat3(pq, pr, ps);
    // det = glm:: determinant(glm::transpose(T) * T);

    const BScalar T00 = arb::dot(pq, pq);
    const BScalar T01 = arb::dot(pq, pr);
    const BScalar T02 = arb::dot(pq, ps);
    const BScalar T11 = arb::dot(pr, pr);
    const BScalar T12 = arb::dot(pr, ps);
    const BScalar T22 = arb::dot(ps, ps);
    
    const BScalar det00 = T11 * T22 - T12 * T12;
    const BScalar det01 = T01 * T22 - T02 * T12;
    const BScalar det02 = T01 * T12 - T02 * T11;
    const BScalar det = T00 * det00 - T01 * det01 + T02 * det02;
    
    
    if (arb::nearZero(det, 1E-5))
    {
        // Degenerate case
        W2D(body1, body2, smp);
        return;
    }
    
    const BScalar b0 = arb::dot(pq, po);
    const BScalar b1 = arb::dot(pr, po);
    const BScalar b2 = arb::dot(ps, po);
    
    // inverse matrix
    // (the matrix is symmetric, so we can use the cofactor matrix)
    const BScalar invDet = BScalar(1.0) / det;
    const BScalar det11 = T00 * T22 - T02 * T02;
    const BScalar det12 = T00 * T12 - T01 * T02;
    const BScalar det22 = T00 * T11 - T01 * T01;
    const BScalar I00 = det00 * invDet;
    const BScalar I01 = -det01 * invDet;
    const BScalar I02 = det02 * invDet;
    const BScalar I11 = det11 * invDet;
    const BScalar I12 = -det12 * invDet;
    const BScalar I22 = det22 * invDet;
    
    const BScalar a1 = I00 * b0 + I01 * b1 + I02 * b2;
    const BScalar a2 = I01 * b0 + I11 * b1 + I12 * b2;
    const BScalar a3 = I02 * b0 + I12 * b1 + I22 * b2;
    const BScalar a0 = BScalar(1.0) - a1 - a2 - a3;
    
    // check if the origin is very close to one of the faces of the
    // simplex. In this case, a 2D projection will be more accurate.
    if (a0 < B_EPS) 
    {
        smp.vertexNum(3);
        smp.vertexMove(0, 3);
        W2D(body1, body2, smp);
    } 
    else if (a1 < B_EPS) 
    {
        smp.vertexNum(3);
        smp.vertexMove(1, 3);
        W2D(body1, body2, smp);
    } 
    else if (a2 < B_EPS) 
    {
        smp.vertexNum(3);
        smp.vertexMove(2, 3);
        W2D(body1, body2, smp);
    } 
    else if (a3 < B_EPS)
    {
        smp.vertexNum(3);
        W2D(body1, body2, smp);
    }
    
    // Compute witness points
    // This is done by blending the original points using
    // the barycentric coordinates
    const BVector3 w00 =  point(body1, smp[0][B1]);
    const BVector3 w01 =  point(body2, smp[0][B2]);
    const BVector3 w10 =  point(body1, smp[1][B1]);
    const BVector3 w11 =  point(body2, smp[1][B2]);
    const BVector3 w20 =  point(body1, smp[2][B1]);
    const BVector3 w21 =  point(body2, smp[2][B2]);
    const BVector3 w30 =  point(body1, smp[3][B1]);
    const BVector3 w31 =  point(body2, smp[3][B2]);
    
    smp.witness(B1) = w00 * a0 + w10 * a1 + w20 * a2 + w30 * a3;
    smp.witness(B2) = w01 * a0 + w11 * a1 + w21 * a2 + w31 * a3;
    
  /*  
    // more compact, possibly faster, but introduces
    // a new glm type: glm::mat4x3 (glm::mat4x2 aswell)
    glm::dvec4 bary(a0, a1, a2, a3);

    glm::dmat4x3 m0( body1[smp[0][B1]], 
                     body1[smp[1][B1]], 
                     body1[smp[2][B1]], 
                     body1[smp[3][B1]] );

    glm::dmat4x3 m1( body2[smp[0][B2]], 
                     body2[smp[1][B2]], 
                     body2[smp[2][B2]], 
                     body2[smp[3][B2]] );

    smp.witness(0) = m0 * bary;
    smp.witness(1) = m1 * bary; */
}

void
BOpenGJK::calc_witness(const ABody *body1, const ABody *body2, BSimplex &smp) const
{
    switch (smp.vertexNum()) 
    {
        case 4:
            W3D(body1, body2, smp);
            break;
        case 3:
            W2D(body1, body2, smp);
            break;
        case 2:
            W1D(body1, body2, smp);
            break;
        case 1:
            W0D(body1, body2, smp);
            break;
        default:
            std::cout << "\nBOpenGJK::ERROR:\t invalid simplex\n" << std::endl;
    }
}


BVector3
BOpenGJK::point(const ABody *body, const BIndex &idx) const
{
    return (body->orient() * body->collider().point(idx)) + body->pos(); 
}

BVector3
BOpenGJK::first_point( ABody *body ) const 
{ 
    // convert body/model vertex to world vertex; 
    return (body->orient() * body->collider().first_point()) + body->pos(); 
}

BVector3
BOpenGJK::max_point( ABody *body, const BVector3& dir ) const 
{
    // convert from world to body/model coords
    const BVector3 mydir = (arb::transpose(body->orient()) * dir);
    
    // support function 
    const BVector3 vert =  body->collider().max_point(mydir);
    
    // convert body/model vertex to world vertex; 
    return (body->orient() * vert)  + body->pos();
}

BScalar 
BOpenGJK::min_distance(ABody *bdy1, ABody *bdy2, BSimplex &smp)
{
    using std::max;
    
    const BCollider &body1 = bdy1->collider();
    const BCollider &body2 = bdy2->collider();
    
    // initialise search direction
    BVector3 v = first_point(bdy1) - first_point(bdy2); 
    
    // initialise simplex 
    smp.init(v);
    
    int k = 0;                  
    
    // begin GJK iteration 
    for ( ; k < m_max_iters; ++k) 
    { 
        // update negative search direction
        BVector3 vminus = -v;
        
        // support function 
        const BVector3 body1_vert =  max_point(bdy1, vminus);
  
        const BVector3 body2_vert =  max_point(bdy2, v);
        
        BGJKVertex w;
        w.vertex = (body1_vert - body2_vert);
        w.index  = {body1.max_index(), body2.max_index()}; // (vert,poly) body1, (vert,poly) body 2
        //

        BScalar len_sq = arb::length2(v);
        
        // test first exit condition (new point already in simplex/can't move
        BScalar exeedtol_rel = (len_sq - arb::dot(v, w.vertex));
        
        if (exeedtol_rel <= (B_EPS_REL * len_sq) || exeedtol_rel < B_EPS_ABS) 
            break;
        
        if (len_sq < B_EPS_REL2)  
            break; // it a null V

        // add new vertex to simplex 
        int i = smp.vertexNum();
        smp[i] = w;
        smp.vertexNum(i+1);
        
        // Invoke distance sub-algorithm
        subalgorithm(smp, v); // v can change here!
        
        // Test 
        BScalar norm2Wmax = 0;
        for (int j = 0; j < smp.vertexNum(); ++j) 
        {
            BScalar tesnorm = arb::length2(smp[j].vertex);
            norm2Wmax = max(tesnorm, norm2Wmax);
        }
        
        if ((arb::length2(v) <= (B_EPS_ABS2 * norm2Wmax))) 
            break;
        
        if (smp.vertexNum() == 4)
            break;
    } 
    
    if (k == m_max_iters) 
    {
        std::cout << "\n* * * GJK MAXIMUM ITERATION NUMBER REACHED!!! * * *\n" << std::endl;
    }
    
    calc_witness(bdy1, bdy2, smp);
    
    return arb::length(v);
}

