/* BSpatialChecks 08/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialChecks.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 A number of basic functions and consistency checks to help with debugging/maintanence
 You can use/improve these functions to build a more comprehensive test suite if you like
 
 This is very much a first draft.
 
 Note rndInertia() could do with a lot more thought/improvement. 
 It is not straightforward to generate completely random inertias that 'cover' 
 the space of possible inertias.
 
 
*/

#ifndef __BSPATIALCHECKS_H__
#include "BSpatialChecks.h"
#endif

#ifndef __URAND_H__
#include "URand.h"
#endif

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

namespace arb 
{
    
    const double RANGE     = 10.0;
    const double ZERO_PROB = 0.99;
    
    inline URand global_ran(22);
    
    int
    rndInt( int N ) { return global_ran.rndInt(N); }
    
    int
    rndInt( int min, int max ) { return global_ran.rndInt(min, max); }
    
    double 
    rndFloat( void ) { return global_ran.rndFloat(); }
    
    double 
    rndFloat( double min, double max ) { return global_ran.rndFloat(min, max); }
    
    // set the seed and reinitialise the sequence
    void
    rndSeed( unsigned int s )  { return global_ran.seed(s); }
    
    //
    //
    //
    
    BVector3 
    rndVec3( void ) 
    {
        BVector3 R;
        for(int i = 0; i < 3; ++i)
        {
            if (rndFloat() < ZERO_PROB)
                R[i] = rndFloat(-RANGE, RANGE);
            else R[i] = 0.0;
        }
        return R;
    }
    
    BVector3 
    rndVec3( double min, double max ) 
    {
        BVector3 R;
        for(int i = 0; i < 3; ++i)
        {
            R[i] = rndFloat((double) min, (double) max);
        }
        return R;
    }
    
    BVector3 
    rndAxis( void )
    {
        int axis = rndInt(3);
        if (axis == 0) 
            return B_XAXIS;
        else if (axis == 1)
            return B_YAXIS;
        else return B_ZAXIS;
    }
    
    BVector6 
    rndVec6( void ) 
    {
        BVector6 R;
        for(int i = 0; i < 6; ++i)
        {
            if (rndFloat() < ZERO_PROB)
                R[i] = rndFloat(-RANGE, RANGE);
            else R[i] = 0.0;
        }
        return R;
    }
    
    BMatrix3 
    rndMat3( void ) 
    {
        BMatrix3 R;
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                if (rndFloat() < ZERO_PROB)
                    R[i][j] = rndFloat(-RANGE, RANGE);
                else R[i][j] = 0.0;
            }
        }
        return R;
    }
    
    
    BMatrix6  
    rndMat6( void ) 
    {
        BMatrix6 R;
        for(int i = 0; i < 6; ++i)
        {
            for(int j = 0; j < 6; ++j)
            {
                if (rndFloat() < ZERO_PROB)
                    R[i][j] = rndFloat(-RANGE, RANGE);
                else R[i][j] = 0.0;
            }
        }
        return R;
    }
    

    BMatrix3 
    rndRot( void ) 
    {
        BScalar angle = rndFloat(-M_PI, M_PI);
        BVector3 axis = rndVec3();
        if (axis == B_ZERO_3)
            axis = rndAxis();
        
        return glm::rotate(angle, glm::normalize(axis));
    }
    
    BQuat 
    rndQuat( void )  { return glm::quat_cast(rndRot()); }
    
    
    BTransform
    rndTransform( void )
    {
        BMatrix3 E = rndRot();
        BVector3 r = rndVec3();
        BTransform X(E,r);
        
        return X;
    }

    BInertia 
    rndInertia( void )
    // more work needed here
    {
        BMatrix3 I(B_ZERO_3x3);
        
        BScalar mass = arb::rndFloat(1.0, 100.0);
        BVector3 diag = rndVec3(0.0, RANGE);
        
        I[0][0] = diag[1] + diag[2];
        I[1][1] = diag[0] + diag[2];
        I[2][2] = diag[0] + diag[1];
  
        BInertia inertia(mass, I);
        
        inertia.transform(rndRot(), rndVec3());
 
        return inertia;
    }
    
    
    BRBInertia 
    rndRBInertia( void )
    {
        BInertia I = rndInertia();
        BRBInertia rbi = BRBInertia(I); 
        
        return rbi;
    }
    
    BABInertia 
    rndABInertia( void )
    {
        BInertia I = rndInertia();
        BABInertia abi(I);
        
        return abi;
    }
    
    //
    // consistency tests
    //
}  
    
std::vector<int>
test_adjoints( bool display )
{
    if (display)
    {
        std::cout << "\nAdjoint Test\n" << std::endl; 
    }
    
    BTransform X = arb::rndTransform();
    const BVector6 force = arb::rndVec6();
    
    std::vector<int> testsPassed;
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjoint( X, force ); 
        BMatrix6 m1 =  arb::toAdjoint(X); 
        BVector6 x2 =  m1 * force;

        bool test1 = arb::nearZero(x1 - x2);
        bool test2 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjointInverse(X, force)) - force);
        bool test3 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjoint(arb::inverse(X), force) ) - force);
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);
        
        if (display)
        {
            std::cout  << "Adjoint -- Ad_X -- Test1 is " << test1 << std::endl;
            std::cout  << "Adjoint -- Ad_X -- Test2 is " << test2 << std::endl;
            std::cout  << "Adjoint -- Ad_X -- Test3 is " << test3 << std::endl;
        }
    }
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjointTranspose( X, force ); 
        BMatrix6 m1 =  arb::toAdjointTranspose(X);
        BVector6 x2 =  m1 * force;

        bool test1 = arb::nearZero(x1 - x2);
        
        testsPassed.push_back(test1);
        
        if (display)
        {
            std::cout  << "Adjoint Transpose -- Ad_X^{T} -- Test1 is " << test1 << std::endl;
        }
    }
    
    if (1)
    {
        BVector6 x1 = arb::applyAdjointInverse( X, force ); 
        BMatrix6 m1 = arb::toAdjointInverse(X);
        BVector6 x2 = m1 * force;
        BMatrix6 P  = arb::toAdjoint(X) *  arb::toAdjointInverse(X);
        BMatrix6 Q  = arb::toAdjointInverse(X) * arb::toAdjoint(X);
        
        bool test1 = arb::nearZero(x1 - x2);
        bool test2 = arb::nearZero(P - B_IDENTITY_6x6);
        bool test3 = arb::nearZero(Q - B_IDENTITY_6x6);
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);
        
        if (display)
        {
            std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test1 is " << test1 << std::endl;
            std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test2 is " << test2 << std::endl;
            std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test3 is " << test3 << std::endl;
        }
    }
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjointDual( X, force ); 
        BMatrix6 m1 =  arb::toAdjointDual(X);
        BVector6 x2 =  m1 * force;

        bool test1 = arb::nearZero(x1 - x2);
        
        testsPassed.push_back(test1);
        
        if (display)
        {
            std::cout  << "Adjoint Dual -- Ad_X^{-T} -- Test1 is " << test1 << std::endl;
        }
    }
    
    if (1)
    {
        BTransform Y(X.E(), BVector3(0.1, 2.0, 0.3));
        BTransform Z = Y * arb::inverse(Y);

        bool test1 = arb::nearZero(BMatrix6(Z) - B_IDENTITY_6x6);
        
        testsPassed.push_back(test1);
        
        if (display)
        {
            std::cout  << "Inverse -- X^{-1}-- Test1 is " << test1 << std::endl;
        }
    }
    
    if (1)
    {
        BMatrix6 A = arb::toAdjointTranspose(X);
        BMatrix6 B = arb::transpose(arb::toAdjoint(X)); 
        BMatrix6 C = arb::transpose(arb::toAdjointInverse(X)); 
        BMatrix6 D = arb::toAdjointDual(X);

        bool test1 = arb::nearZero(A - B);
        bool test2 = arb::nearZero(C - D);
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        
        if (display)
        {
            std::cout  << "Transpose -- X^{T} -- Test1 is " << test1 << std::endl;
            std::cout  << "Transpose  -- X^{T} -- Test2 is " << test2 << std::endl;
        }
    }
    
    return  testsPassed;
}

std::vector<int>
test_rbinertia( bool display )
{
    if (display)
    {
        std::cout << "\nRigid Body Inertia Test\n" << std::endl; 
    }
    
    BRBInertia I = arb::rndRBInertia();
    BTransform X = arb::rndTransform();
    
    std::vector<int> testsPassed;
    
    if (1)
    {   
        // world coords
        BMatrix6 I_w  = arb::transpose(X) * I * X; // inertia in word coords
        BMatrix6 invI_w = arb::inverse(X) * arb::inverse(I) * arb::dual(X); // inverse inertia in word coords

        // basic inverse
        bool test1 = arb::nearZero( (BMatrix6(I) * arb::inverse(I)) - B_IDENTITY_6x6 );
        bool test2 = arb::nearZero( (I_w * invI_w) - B_IDENTITY_6x6 );
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        
        if (display)
        {
            std::cout  << "RBInertia --  I_b * I_b^{-1} == IDENTITY -- Test1 is " << test1 << std::endl;
            std::cout  << "RBInertia --  I_w * I_w^{-1} == IDENTITY -- Test2 is " << test2 << std::endl;
        }
    }
    
    if (1)
    {   
        // Apply - X^* I X^{-1}
        BRBInertia I1 = X.apply( I ); 
        
        BMatrix6 A = arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(I) * BMatrix6(arb::inverse(X));
        BMatrix6 B = arb::dual(X) * BMatrix6(I) * BMatrix6(arb::inverse(X));
        BMatrix6 C = arb::dual(X) * I * arb::inverse(X);
        BMatrix6 D = arb::transpose(arb::inverse(X)) * BMatrix6(I) * arb::inverse(X);
        BMatrix6 E = BMatrix6(I1);
        
        bool test1 = arb::nearZero(A - E);
        bool test2 = arb::nearZero(B - E);
        bool test3 = arb::nearZero(C - E);
        bool test4 = arb::nearZero(D - E);
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);
        testsPassed.push_back(test4);
        
        if (display)
        {
            std::cout  << "RBInertia --  X^* I X^{-1} -- Test1 is " << test1 << std::endl;
            std::cout  << "RBInertia --  X^* I X^{-1} -- Test2 is " << test2 << std::endl;
            std::cout  << "RBInertia --  X^* I X^{-1} -- Test3 is " << test3 << std::endl;
            std::cout  << "RBInertia --  X^* I X^{-1} -- Test4 is " << test4 << std::endl;
        }
    }
    
    if (1)                                                      
    {
        // ApplyTransposw -  X^T I X
        BRBInertia I2 = X.applyTranspose( I ); 
        BMatrix6 A = arb::transpose(BMatrix6(X)) * BMatrix6(I) * BMatrix6(X);
        BMatrix6 B = arb::transpose(X) * I * X;
        BMatrix6 C = BMatrix6(I2);
        
        bool test5 =  arb::nearZero(A - C);
        bool test6 =  arb::nearZero(B - C);
        
        testsPassed.push_back(test5);
        testsPassed.push_back(test6);
        
        if (display)
        {
            std::cout  << "RBInertia --  X^T I X -- Test5 is " << test5 << std::endl;
            std::cout  << "RBInertia --  X^T I X -- Test6 is " << test6 << std::endl;
        }
    }
    
    return testsPassed;
}

std::vector<int>
test_abinertia( bool display )
{
    if (display)
    {
        std::cout << "\nArticulated Inertia Test\n" << std::endl; 
    }
    
    BABInertia abi = arb::rndABInertia(); 
    BTransform X = arb::rndTransform();  
    
    std::vector<int> testsPassed;
    
    if (1)
    {   
        // world coords
        BMatrix6 I_w  = arb::transpose(X) * abi * X; // inertia in word coords
        BMatrix6 invI_w = arb::inverse(X) * arb::inverse(abi) * arb::dual(X); // inverse inertia in word coords
        
        // basic inverse
        bool test1 = arb::nearZero( (BMatrix6(abi) * arb::inverse(abi)) - B_IDENTITY_6x6 );
        bool test2 = arb::nearZero( (I_w * invI_w) - B_IDENTITY_6x6 );
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        
        if (display)
        {
            std::cout  << "ABInertia --  I_b * I_b^{-1} == IDENTITY -- Test1 is " << test1 << std::endl;
            std::cout  << "ABInertia --  I_w * I_w^{-1} == IDENTITY -- Test2 is " << test2 << std::endl;
        }
    }
    
    if (1)
    {
        // Apply - X^* I X^{-1}
        BABInertia I1 = X.apply( abi ); 
        BMatrix6 A = (arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(abi)) * BMatrix6(arb::inverse(X));
        BMatrix6 B = (arb::transpose(arb::inverse(X)) * abi) * arb::inverse(X);
        BMatrix6 C = (arb::dual(X) * abi) * arb::inverse(X);
 
        BMatrix6 D = BMatrix6(I1);
        
        bool test3 = arb::nearZero(A - D);
        bool test4 = arb::nearZero(B - D);
        bool test5 = arb::nearZero(C - D);
        
        testsPassed.push_back(test3);
        testsPassed.push_back(test4);
        testsPassed.push_back(test5);
        
        if (display)
        {
            std::cout  << "ABInertia -- X^* I X^{-1} -- Test3 is " << test3 << std::endl;
            std::cout  << "ABInertia -- X^* I X^{-1} -- Test4 is " << test4 << std::endl;
            std::cout  << "ABInertia -- X^* I X^{-1} -- Test5 is " << test5 << std::endl;
        }
    }
    if (1)
    {
        // ApplyTransposw -  X^T I X
        BABInertia I2 = X.applyTranspose( abi ); 
        BMatrix6 A = arb::transpose(BMatrix6(X)) * BMatrix6(abi) * BMatrix6(X);
        BMatrix6 B = arb::transpose(X) * abi * X;
        BMatrix6 C = BMatrix6(I2);
        
        bool test6 = arb::nearZero(A - C);
        bool test7 = arb::nearZero(B - C);
        
        testsPassed.push_back(test6);
        testsPassed.push_back(test7);
        
        if (display)
        {
            std::cout  << "ABInertia -- X^T I X -- Test6 is " << test6 << std::endl;
            std::cout  << "ABInertia -- X^T I X -- Test7 is " << test7 << std::endl;
        }
    }
    
    return testsPassed;
}

std::vector<int>
test_inverse( bool display )
{
    if (display)
    {
        std::cout << "\nInverse Inertia Test\n" << std::endl; 
    }
    
    std::vector<int> testsPassed;
    
    // setup a body
    
    BRBInertia I = arb::rndRBInertia();
    // world to body transform
    BTransform X_wb = arb::rndTransform(); 
    
    
    if (1)
    {
        // BTransform Identities 1,2,3
        BMatrix6 A(X_wb * arb::inverse(X_wb)); 
        BMatrix6 B(arb::inverse(X_wb) * X_wb);
        BMatrix6 C = arb::transpose(X_wb) * arb::dual(X_wb);
        
        bool test1 = arb::nearZero(A - B_IDENTITY_6x6);
        bool test2 = arb::nearZero(B - B_IDENTITY_6x6);
        bool test3 = arb::nearZero(C - B_IDENTITY_6x6);
        
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);
        
        if (display)
        {
            std::cout << "BTransform -- b^X_w * arb::inverse(b^X_w) == Identity -- Test1 is " << test1  << std::endl;
            std::cout << "BTransform -- arb::inverse(b^X_w) * b^X_w == Identity -- Test2 is  " << test2  << std::endl;
            std::cout << "BTransform -- arb::transpose(b^X_w) * arb::dual(b^X_w) == Identity -- Test3 is " << test3  << std::endl;
        }
    }
    
    
    if (1)
    {
        BVector6 h_w(0.0, 0.0, 1.0,  2.0, 0.0, 0.0);  // momentum is a force vector
        
        // body coords momentum and velocity
        BVector6 h_b = arb::dual(X_wb) * h_w; 
        BVector6 v_b = arb::inverse(I) * h_b;  // RBDA, eqn 2.61, page 32.
        
        // world coords velocity
        BVector6 v_w    = arb::inverse(X_wb) * v_b;
        BMatrix6 invI_w = arb::inverse(X_wb) * arb::inverse(I) * arb::dual(X_wb);  // RBDA, Table 2.5, page 34
        BVector6 my_v_w = invI_w * h_w;  
   
        BMatrix6 invI_b = (X_wb * invI_w * arb::transpose(X_wb));  
        
        bool test1 = arb::nearZero(my_v_w - v_w);
        // If a force f acts on a rigid body having a velocity v, then
        // the power delivered by the force is f \dot v (RDBA, Scalar Products, page 19). 
        bool test2 = arb::nearZero(arb::dot(h_w, v_w) - arb::dot(h_b, v_b));
        bool test3 = arb::nearZero(invI_b - arb::inverse(I));
       
        testsPassed.push_back(test1);
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);

        if (display)
        {
            std::cout << "Inverse Inertia -- Velocity -- Test1 is " << test1 << std::endl;
            std::cout << "Power -- Test2 is " << test2 << std::endl;
            std::cout << "Inverse Inertia -- Identity -- Test3 is " << test3 << std::endl;
        }
    }
    
    return testsPassed;
}


std::vector<int>
test_inertia( bool display )
{
    if (display)
    {
        std::cout << "\nRotational Inertia Test\n" << std::endl; 
    }
    
    std::vector<int> testsPassed;
    
    BInertia I3 = arb::rndInertia();
    BTransform X = arb::rndTransform();  
    
    if (1)
    {
  
        // check inertia at origin matches spatial inertia at origin
        BRBInertia I6(I3.mass(), I3.h(), I3.I());
       
        
        bool test2 = arb::nearZero(I6.I() - BMatrix3(I3.I()));
        
        // check inetia transform matches spatial transform
        I6 = X.apply(I6);
        I3.transform(X.E(),X.r());
        
        bool test3 = arb::nearZero(I6.Icom() - BMatrix3(I3.Icom()));
        
        // shift test - shift to some r and then back again
        BVector3 r = arb::rndVec3();
        BInertia tmp = I3;
   
        I3.shift( r );  
        I3.shift( -r ); 

        bool test4 = arb::nearZero(I3 - tmp);
       
        
        testsPassed.push_back(test2);
        testsPassed.push_back(test3);
        testsPassed.push_back(test4);
        
        if (display)
        {
            std::cout  << "BInertia -- inertial is valid -- Test2 is " << test2 << std::endl;     
            std::cout  << "BInertia -- agreement with spatial inertia -- Test3 is " << test3 << std::endl; 
            std::cout  << "BInertia -- transform inertia -- Test4 is " << test4 << std::endl; 
        }

    }
    
    return testsPassed;
}

std::vector<int>
test_misc( bool display )
// your tests go here!
{
    if (display)
    {
        std::cout << "\nMiscellaneous Tests\n" << std::endl; 
    }
    
    std::vector<int> testsPassed;
    
    BVector3 v = arb::rndVec3();

    if (1)
    {
        
        bool test1  = (arb::cross(v) == -arb::transpose(arb::cross(v)));
        
        testsPassed.push_back(test1);
        
        if (display)
        {
            std::cout  << "BProduct -- cross is valid -- Test1 is " << test1 << std::endl;     
        }

    }
    return testsPassed;
}

int
check( int count, int seed )
// consistency checks 
{
    std::cout << "ARB: Performing consistency checks..." << std::endl;
    arb::global_ran.seed(seed);
    
    int errs = 0;
    
    int numTest = 0;
    
    bool display = false;
    if (count == 1)
        display = true;
    
    for (int i = 0; i < count; ++i)
    {
        std::vector<int> aux, tests;
        
        aux =  test_adjoints(display);
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        aux = test_rbinertia(display);
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        aux = test_abinertia(display);
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        aux = test_inverse(display);
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        aux =  test_inertia( display );
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        aux =  test_misc( display );
        tests.insert(tests.end(), aux.begin(), aux.end());
        
        int correct = std::accumulate(tests.begin(), tests.end(), 0);
        
        errs = (int) tests.size() - correct;
        
        numTest = (int) tests.size();
        
        if (errs) 
            break;
    }
    
    if (errs)
    {
        std::cout << "\nARB: Fatal Error - " <<  errs << " internal consistency checks failed!!!\n" << std::endl;
    
        exit(EXIT_FAILURE);
       // return errs;
    }
    
    std::cout << "\nARB: " << numTest << " internal consistency checks passed on " << count << " examples\n" << std::endl;
    return 0;
}


