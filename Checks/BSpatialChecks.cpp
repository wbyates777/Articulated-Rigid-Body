
/* BSpatialChecks 07/04/2026

 $$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialChecks.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:
 
 add test index number i.e 1,2,...

*/


#ifndef __BSPATIALCHECKS_H__
#include "BSpatialChecks.h"
#endif

#ifndef __BSPATIALRANDOM_H__
#include "BSpatialRandom.h"
#endif

#include <string>
#include <vector>
#include <map>
#include <iostream>



class BSpatialChecks
{
public:

    BSpatialChecks( void ) : totalTests(0), failedTests(0), verbose(false) {} 
    BSpatialChecks( bool display ) : totalTests(0), failedTests(0), verbose(display) {}
    ~BSpatialChecks( void )=default;
    
    void 
    expect( bool condition, const std::string& category, const std::string& testName ) 
    {
        if (!condition) 
        {
            std::cout << "  [FAIL] " << category << " :: " << testName << std::endl;
            ++failedTests;
        } 
        else if (verbose) 
        {
            std::cout << "  [PASS] " << category << " :: " << testName << std::endl;
        }
        ++totalTests;
    }

    void 
    section( const std::string& name ) 
    {
        if (verbose) 
            std::cout << "\n--- " << name << " ---" << std::endl;
    }

    bool
    summary( void )
    {
        if (failedTests > 0) 
        {
            std::cout << "\nFATAL: " << failedTests << "/" << totalTests << " checks failed!" << std::endl;
            return false;
        }
        std::cout << "\nSUCCESS: All " << totalTests << " consistency checks passed." << std::endl;
        return true;
    }
    
    int totalTests;
    int failedTests;
    bool verbose;
};


//
// Specific Test Suites --- BSpatialChecks
//


void 
run_adjoint_tests(BSpatialChecks &st) 
{
    st.section("Adjoint Identities");
    
    BTransform X = arb::rndTransform();
    BVector6 f = arb::rndVec6();
    BVector6 v = arb::rndVec6();
    
    // test apply toMatrix
    bool A = arb::nearZero(arb::applyAdjoint(X, f) - (arb::toAdjoint(X) * f));
    bool B = arb::nearZero(arb::applyAdjointDual(X, f) - (arb::toAdjointDual(X) * f));
    bool C = arb::nearZero(arb::applyAdjointInverse(X, f) - (arb::toAdjointInverse(X) * f));
    bool D = arb::nearZero(arb::applyAdjointTranspose(X, f) - (arb::toAdjointTranspose(X) * f));
    
    st.expect(A, "Adjoint", "adjoint - apply vs toMatrix");
    st.expect(B, "Adjoint", "dual  - apply vs toMatrix");
    st.expect(C, "Adjoint", "inverse  -apply vs toMatrix");
    st.expect(D, "Adjoint", "transpose  - apply vs toMatrix");
    
    // inverse
    bool E = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjointInverse(X, f)) - f);
    bool F = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjoint(arb::inverse(X), f) ) - f);
    bool G = arb::nearZero(arb::toAdjoint(X) *  arb::toAdjointInverse(X) - B_IDENTITY_6x6);
    bool H = arb::nearZero(arb::toAdjointInverse(X) * arb::toAdjoint(X) - B_IDENTITY_6x6);
    
    st.expect(E, "Adjoint", "inverse - Ad_X(Ad_invX) == Id");
    st.expect(F, "Adjoint", "inverse - Ad_X(Ad_X(invX)) == Id" );
    st.expect(G, "Adjoint", "inverse - Ad_X * Ad_invX == Id"); 
    st.expect(H, "Adjoint", "inverse - Ad_invX * Ad_X == Id");
    
    // transpose 
    bool I = arb::nearZero(arb::toAdjointTranspose(X) - arb::transpose(arb::toAdjoint(X)));
    bool J = arb::nearZero(arb::transpose(arb::toAdjointInverse(X)) - arb::toAdjointDual(X));
    
    st.expect(I, "Adjoint", "transose - Ad_XT == (Ad_X)^T");
    st.expect(J, "Adjoint", "transpose - (Ad_invX)^T == Ad_X^*");
    
    // Modern Robotics Prop 3.21, pg 100 : Ad[T1]*Ad[T2] == Ad[T1*T2]
    BTransform T1 = arb::rndTransform();
    BTransform T2 = arb::rndTransform();
    
    bool K = arb::nearZero((arb::toAdjoint(T1) * arb::toAdjoint(T2)) - arb::toAdjoint(T1 * T2));
    bool L = arb::nearZero((arb::toAdjointDual(T1) * arb::toAdjointDual(T2)) - arb::toAdjointDual(T1 * T2)); 
    
    st.expect(K, "Adjoint", "Prop 3.21 - composition chain");
    st.expect(L, "Adjoint", "Prop 3.21 - dual composition chain");
    
    // power invariance (RBDA Section 2.6, page 17)
    BScalar p_orig = arb::dot(f, v);
    BScalar p_tran = arb::dot(arb::applyAdjointDual(X, f), arb::applyAdjoint(X, v));
    
    st.expect(arb::nearZero(p_orig - p_tran), "Adjoint", "physics - power invariance under X");
}

void 
run_inertia_tests(BSpatialChecks &st) 
{
    st.section("Rotational Inertia");
    
    BInertia I3 = arb::rndInertia();
    BTransform X = arb::rndTransform(); 
    
    // check inertia at origin matches spatial inertia at origin
    BRBInertia I6(I3.mass(), I3.h(), I3.I());
   
    st.expect(arb::nearZero(I6.I() - BMatrix3(I3.I())), "BInertia", "rotational and spatial inertia at origin match");
   
    // check inetia transform matches spatial transform
    I6 = X.apply(I6);
    I3.transform(X.E(),X.r());
    
    st.expect(arb::nearZero(I6.Icom() - BMatrix3(I3.Icom())), "BInertia", "rotational and spatial transform match");
    
    // shift test - shift to some r and then back again
    BVector3 r = arb::rndVec3();
    BInertia tmp = I3;

    I3.shift( r );  
    I3.shift( -r ); 

    st.expect(arb::nearZero(I3 - tmp), "BInertia", "shift test");
}

void 
run_rbinertia_tests(BSpatialChecks &st) 
{
    st.section("Rigid Body Inertia");
    
    BRBInertia I = arb::rndInertia();
    BTransform X = arb::rndTransform();
    
    // basic inverse
    BMatrix6 I_mat(I);
    
    bool R = arb::nearZero((I_mat * arb::inverse(I)) - B_IDENTITY_6x6);
    bool S = arb::nearZero((arb::inverse(I) * I_mat) - B_IDENTITY_6x6);
    
    st.expect(R, "BRBInertia", "I * I^-1 == Id");
    st.expect(S, "BRBInertia", "I^-1 * I == Id");
    
    // world inverse

    BMatrix6 I_w  = arb::transpose(X) * I * X; // inertia in word coords
    BMatrix6 invI_w = arb::inverse(X) * arb::inverse(I) * arb::dual(X); // inverse inertia in word coords
    
    bool T = arb::nearZero((I_w * invI_w) - B_IDENTITY_6x6);
    
    st.expect(T, "BRBInertia", "I_w * I_w^-1 == Id");
    
    
    // apply: I' = X^* * I * X^-1
    BRBInertia I_apply = X.apply(I);
    BMatrix6 I_expected = arb::dual(X) * BMatrix6(I) * arb::inverse(X);
    st.expect(arb::nearZero(BMatrix6(I_apply) - I_expected), "BRBInertia", "I' = X^* * I * X^-1");
    
    // variants : I' = X^* * I * X^-1
    BMatrix6 A = arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(I) * BMatrix6(arb::inverse(X));
    BMatrix6 B = arb::dual(X) * BMatrix6(I) * BMatrix6(arb::inverse(X));
    BMatrix6 C = arb::dual(X) * I * arb::inverse(X);
    BMatrix6 D = arb::transpose(arb::inverse(X)) * BMatrix6(I) * arb::inverse(X);
    BMatrix6 E = BMatrix6(I_apply);
    
    st.expect(arb::nearZero(A - E), "BRBInertia", "I' = X^* * I * X^-1 (a)");
    st.expect(arb::nearZero(B - E), "BRBInertia", "I' = X^* * I * X^-1 (b)");
    st.expect(arb::nearZero(C - E), "BRBInertia", "I' = X^* * I * X^-1 (c)");
    st.expect(arb::nearZero(D - E), "BRBInertia", "I' = X^* * I * X^-1 (d)");
    
    
    // apply transpose : X^T * I * X
    BRBInertia I_trans = X.applyTranspose(I);
    BMatrix6 I_expected2 = arb::transpose(X) * BMatrix6(I) * X;
    st.expect(arb::nearZero(BMatrix6(I_trans) - I_expected2), "BRBInertia", "I' = X^T * I * X");
    
    // variants : X^T * I * X
    BMatrix6 F = arb::transpose(BMatrix6(X)) * BMatrix6(I) * BMatrix6(X);
    BMatrix6 G = arb::transpose(X) * I * X;
    BMatrix6 H = BMatrix6(I_trans);
    
    st.expect(arb::nearZero(F - H), "BRBInertia", "I' = X^T * I * X (a)");
    st.expect(arb::nearZero(G - H), "BRBInertia", "I' = X^T * I * X (b)");
}

void 
run_abinertia_tests(BSpatialChecks &st) 
{
    st.section("Articulated Body Inertia");
    
    // Symmetry of ABI Projection
    BABInertia I = arb::rndABInertia();
    BTransform X = arb::rndTransform();
    BVector6 S = arb::rndVec6();
    BVector6 U = BMatrix6(I) * S;
    BScalar  d = arb::dot(S, U);
    
    if (abs(d) > 1E-6) 
    {
        BMatrix6 I_next = BMatrix6(I) - (arb::outer(U, U) / d);
        st.expect(arb::nearZero(I_next - arb::transpose(I_next)), "BABInertia", "Projection Symmetry");
    }
    
    // basic inverse
    BMatrix6 I_mat(I);
    st.expect(arb::nearZero((I_mat * arb::inverse(I)) - B_IDENTITY_6x6), "BABInertia", "I * I^-1 == Id");
    st.expect(arb::nearZero((arb::inverse(I) * I_mat) - B_IDENTITY_6x6), "BABInertia", "I^-1 * I == Id");
    
    // world inverse
    BMatrix6 I_w  = arb::transpose(X) * I * X; // inertia in word coords
    BMatrix6 invI_w = arb::inverse(X) * arb::inverse(I) * arb::dual(X); // inverse inertia in word coords
    st.expect(arb::nearZero((I_w * invI_w) - B_IDENTITY_6x6), "BABInertia", "I_w * I_w^-1 == Id");
    
    // apply - X^* * I * X^{-1}
    BABInertia I1 = X.apply( I ); 

    BMatrix6 A = (arb::dual(X) * I) * arb::inverse(X);
    BMatrix6 B = (arb::transpose(arb::inverse(X)) * I) * arb::inverse(X);
    BMatrix6 C = (arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(I)) * BMatrix6(arb::inverse(X));
    BMatrix6 D = BMatrix6(I1);
    
    st.expect(arb::nearZero(A - D), "BABInertia", "I' = X^* * I * X^-1");
    st.expect(arb::nearZero(B - D), "BABInertia", "I' = X^* * I * X^-1 (a)");
    st.expect(arb::nearZero(C - D), "BABInertia", "I' = X^* * I * X^-1 (b)");
    
    
    // apply transposw -  X^T * I * X
    BABInertia I2 = X.applyTranspose( I ); 
    BMatrix6 E = arb::transpose(X) * I * X;
    BMatrix6 F = arb::transpose(BMatrix6(X)) * BMatrix6(I) * BMatrix6(X);
    BMatrix6 G = arb::transpose(X) * BMatrix6(I) * BMatrix6(X);
    BMatrix6 H = BMatrix6(I2);
    
    st.expect(arb::nearZero(E - H), "BABInertia", "I' = X^T * I * X");
    st.expect(arb::nearZero(F - H), "BABInertia", "I' = X^T * I * X (a)");
    st.expect(arb::nearZero(G - H), "BABInertia", "I' = X^T * I * X (b)");

}

void 
run_transform_tests(BSpatialChecks &st) 
{
    st.section("Transform Test");
    
    BRBInertia I = arb::rndRBInertia();
    BTransform X_wb = arb::rndTransform(); 

    // BTransform Identities 1,2,3
    BMatrix6 A = X_wb * arb::inverse(X_wb); 
    BMatrix6 B = arb::inverse(X_wb) * X_wb;
    BMatrix6 C = arb::transpose(X_wb) * arb::dual(X_wb);
        
    st.expect(arb::nearZero(A - B_IDENTITY_6x6), "Transform", "inverse - X * X^{-1} == Identity");
    st.expect(arb::nearZero(B - B_IDENTITY_6x6), "Transform", "inverse - X^{-1} * X == Identity");
    st.expect(arb::nearZero(C - B_IDENTITY_6x6), "Transform", "dual identity - X^T * X^*) == Identity");
    
    BVector6 h_w = arb::rndVec6();  // momentum is a force vector
    
    // body coords momentum and velocity
    BVector6 h_b = arb::dual(X_wb) * h_w; 
    BVector6 v_b = arb::inverse(I) * h_b;  // RBDA, eqn 2.61, page 32.
    
    // world coords velocity
    BVector6 v_w    = arb::inverse(X_wb) * v_b;
    BMatrix6 invI_w = arb::inverse(X_wb) * arb::inverse(I) * arb::dual(X_wb);  // RBDA, Table 2.5, page 34
    BVector6 my_v_w = invI_w * h_w;  
    BMatrix6 invI_b = (X_wb * invI_w * arb::transpose(X_wb));  
    st.expect(arb::nearZero(my_v_w - v_w), "Transform", "world/base coords and velocity");
    
    // If a force f acts on a rigid body having a velocity v, then
    // the power delivered by the force is f \dot v (RDBA, Scalar Products, page 19).
    bool power_test = arb::nearZero(arb::dot(h_w, v_w) - arb::dot(h_b, v_b));
    st.expect(power_test, "Transform", "Power Identity - arb::dot(h_w, v_w) == arb::dot(h_b, v_b)");
    
    st.expect(arb::nearZero(invI_b - arb::inverse(I)), "Transform", "inverse inertia transform");
   
}

void 
run_lie_group_tests(BSpatialChecks &st) 
{
    st.section("Lie Theory (exp/log)");
    
    BVector3 axis = arb::rndVec3();
    if (axis == B_ZERO_3)
        axis = arb::rndAxis();
    BVector3 w = glm::normalize(axis);
    
    BScalar theta = arb::rndFloat();
    BVector3 x = w * theta;
    BMatrix3 R = arb::exp( x );
    BVector3 y = arb::log(R);
    
    st.expect(arb::nearZero(x - y), "Lie", "3D - log(exp(v)) == v"); // || arb::nearZero(v1 + v2)

    
    BVector6 v = arb::rndVec6();
    v.ang( arb::rndVec3(-M_PI_2, M_PI_2));  // keep rotation small to avoid wrap-around issues in Log
    v.lin( arb::rndVec3());
    
    // exp(log(v)) ==v identity
    BTransform X = arb::exp(v);
    BVector6 v_recovered = arb::log(X);
    st.expect(arb::nearZero(v - v_recovered), "Lie", "6D - log(exp(v)) == v");
    
    // exp(-v) == inv(exp(v))
    st.expect(arb::nearZero(BMatrix6(arb::exp(-v)) - arb::inverse((arb::exp(v)))), "Lie", "exp mapping inverse");
    
    
    BVector6 v1 = arb::log(X);
    // The Adjoint Identity is a fundamental properties of Lie theory
    BMatrix6 x1 = BMatrix6(arb::exp(X * v1)) - X * arb::exp(v1) * arb::inverse(X);
    st.expect(arb::nearZero( x1 ), "Lie", "Adjoint Identity");
    
}

void 
run_cross_tests(BSpatialChecks &st) 
{
    st.section("Cross Products");
    
    BVector6 u = arb::rndVec6();
    BVector6 v = arb::rndVec6();
    BVector6 w = arb::rndVec6();
    
    // (v \cross v) == 0 (use [u \cross v = - v \cross u] - Table 2.3, page 25, RBDA)
    st.expect(arb::nearZero(arb::crossm(v, v)), "Cross", "crossm(v, v) == 0"); 
    
    // vx* == -vx^T (Table 2.3, page 25, RBDA)
    BMatrix6 vx = BMatrix6(arb::crossm(v));
    BMatrix6 vxf = BMatrix6(arb::crossf(v));
    st.expect(arb::nearZero(vxf + arb::transpose(vx)), "Cross", "vx* == -vx^T");
    
    // scalar product derivative identity: d/dt(dot(f, v))
    // arb::dot(vxf * f, v) + dot(f, vx * v) == 0 (since vx * v is 0)
    st.expect(arb::nearZero(arb::dot(arb::crossf(v, w), v)), "Cross", "derivative identity"); 

    // spatial triple product (the Jacobi identity)
    // test u×(v×w) + v×(w×u) + w×(u×v) == 0
    // https://en.wikipedia.org/wiki/Jacobi_identity
    BVector6 res = arb::crossm(u, arb::crossm(v, w)) + 
                   arb::crossm(v, arb::crossm(w, u)) + 
                   arb::crossm(w, arb::crossm(u, v));
                   
    st.expect(arb::nearZero(res), "Cross", "spatial triple product (the Jacobi identity)");    
}


//
//
//


int 
check(int count, int seed)
{
    std::cout << "ARB: Performing consistency checks... (seed " << seed << ")\n" << std::endl;
    arb::rndSeed(seed);
    
    bool verbose = (count == 1);
    BSpatialChecks runner(verbose);
    
    for (int i = 0; i < count; ++i) 
    {
        run_adjoint_tests(runner);
        run_inertia_tests(runner);
        run_rbinertia_tests(runner);
        run_abinertia_tests(runner);
        run_transform_tests(runner);
        run_lie_group_tests(runner);
        run_cross_tests(runner);
        
        if (runner.failedTests > 0) 
            break; 
    }
    
    if (!runner.summary()) 
    {
        exit(EXIT_FAILURE);
    }
    return 0;
}
