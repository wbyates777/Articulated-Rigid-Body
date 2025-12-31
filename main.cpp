/* Articulated Rigid Body Dynamics 20/02/2024

 $$$$$$$$$$$$$$
 $   BARB.h   $
 $$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/


#include <iostream>
#include <sstream>
#include <fstream>

#define GLM_ENABLE_EXPERIMENTAL

// either
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
// or
//#define GLM_FORCE_INTRINSICS

#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/transform.hpp> 

#ifndef __BDYNAMICS_H__
#include "BDynamics.h"
#endif

#ifndef __BADJOINT_H__
#include "BAdjoint.h"
#endif

int
test_adjoints( void )
{
    std::cout << "\nAdjoint Test\n" << std::endl; 
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(46.0)), glm::normalize(BVector3(-1.1,-2.2,1.3)));
    BTransform X(rot, BVector3(1.0, 2.0, 3.0));
    
    const BVector6 force(-1.1, 1.2, -1.3, 4.0, 0.0, 6.0);
    
    // const BTransform X = arb::Xtrans(glm::dvec3(1.0,2.0,3.0)); // pure translation
    //const BTransform X = arb::Xrot(0.3, glm::dvec3(-1.1,-2.2,1.3)); // pure rotation
    
    int testsPassed = 0;
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjoint( X, force ); 
        BMatrix6 m1 =  arb::toAdjoint(X); 
        BVector6 x2 =  m1 * force;
        //std::cout << x1 << std::endl;
        //std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        bool test2 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjointInverse(X, force) ) - force);
        bool test3 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjoint(arb::inverse(X), force) ) - force);
        testsPassed += test1 + test2 + test3;
        std::cout  << "Adjoint -- Ad_X -- Test1 is " << test1 << std::endl;
        std::cout  << "Adjoint -- Ad_X -- Test2 is " << test2 << std::endl;
        std::cout  << "Adjoint -- Ad_X -- Test3 is " << test3 << std::endl;
    }
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjointTranspose( X, force ); 
        BMatrix6 m1 =  arb::toAdjointTranspose(X);
        BVector6 x2 =  m1 * force;
        //std::cout << x1 << std::endl;
        //std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        testsPassed += test1;
        std::cout  << "Adjoint Transpose -- Ad_X^{T} -- Test1 is " << test1 << std::endl;
    }
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjointInverse( X, force ); 
        BMatrix6 m1 =  arb::toAdjointInverse(X);
        BVector6 x2 =  m1 * force;
        //std::cout << x1 << std::endl;
        //std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        testsPassed += test1;
  
        BMatrix6 P =  arb::toAdjoint(X) *  arb::toAdjointInverse(X);
        //std::cout << P << std::endl;
        bool test2 = arb::nearZero(P - B_IDENTITY_6x6);
        testsPassed += test2;
 
        BMatrix6 Q = arb::toAdjointInverse(X) * arb::toAdjoint(X);
        //std::cout << Q << std::endl;
        bool test3 = arb::nearZero(Q - B_IDENTITY_6x6);
        testsPassed += test3;
        
        std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test1 is " << test1 << std::endl;
        std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test2 is " << test2 << std::endl;
        std::cout  << "Adjoint Inverse -- Ad_X^{-1} -- Test3 is " << test3 << std::endl;
    }
    
    if (1)
    {
        BVector6 x1 =  arb::applyAdjointDual( X, force ); 
        BMatrix6 m1 =  arb::toAdjointDual(X);
        BVector6 x2 =  m1 * force;
        //std::cout << x1 << std::endl;
        //std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        testsPassed += test1;
        std::cout  << "Adjoint Dual -- Ad_X^{-T} -- Test1 is " << test1 << std::endl;
    }
    
    if (1)
    {
        BTransform Y(X.E(), BVector3(0.1, 2.0, 0.3));
        BTransform Z = Y * arb::inverse(Y);
        //std::cout << Z << std::endl;
        bool test1 = arb::nearZero(BMatrix6(Z) - B_IDENTITY_6x6);
        testsPassed += test1;
        std::cout  << "Inverse -- X^{-1}-- Test1 is " << test1 << std::endl;
    }
    
    if (1)
    {
        BMatrix6 tmp4 = arb::toAdjointTranspose(X);
        BMatrix6 tmp5 = arb::transpose(arb::toAdjoint(X)); 
        //std::cout << tmp4 << std::endl;
        //std::cout << tmp5 << std::endl;
        bool test1 = arb::nearZero(tmp4 - tmp5);
        testsPassed += test1;

        BMatrix6 tmp6 =  arb::transpose(arb::toAdjointInverse(X)); 
        BMatrix6 tmp7 = arb::toAdjointDual(X);
        //std::cout << tmp6 << std::endl;
        //std::cout << tmp7 << std::endl;
        bool test2 = arb::nearZero(tmp6 - tmp7);
        testsPassed += test2;
        
        std::cout  << "Transpose -- X^T -- Test1 is " << test1 << std::endl;
        std::cout  << "Transpose  -- X^T -- Test2 is " << test2 << std::endl;
    }
    
    return  testsPassed;
}

int
test_rbinertia( void )
{
    std::cout << "\nRigid Body Inertia Test\n" << std::endl; 
    
    double mass = 59.0, radius = 0.5;
    BVector3 com(1.0);
    BVector3 gyration((2.0/5.0) * mass * (radius * radius));
    BBody sphere(mass, com, gyration);
    
    BRBInertia I = sphere.I();
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(46.0)), glm::normalize(BVector3(1.1, 2.2, 3.3)));
    BVector3 trans = BVector3(-0.9, 8.0, 3.0);
    
    BTransform X(rot, -trans);
    
    int testsPassed = 0;
    
    if (1)
    {   
        // Apply - X^* I X^{-1}
        BRBInertia I1 = X.apply( I ); 
        
        BMatrix6 aux1 = arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(I) * BMatrix6(arb::inverse(X));
        BMatrix6 aux2 = arb::dual(X) * BMatrix6(I) * BMatrix6(arb::inverse(X));
        BMatrix6 aux3 = arb::dual(X) * I * arb::inverse(X);
        BMatrix6 aux4 = arb::transpose(arb::inverse(X)) * BMatrix6(I) * arb::inverse(X);
        
        bool test1 = arb::nearZero(aux1 - BMatrix6(I1));
        bool test2 = arb::nearZero(aux2 - BMatrix6(I1));
        bool test3 = arb::nearZero(aux3 - BMatrix6(I1));
        bool test4 = arb::nearZero(aux4 - BMatrix6(I1));
        testsPassed += test1 + test2 + test3 + test4;
        std::cout  << "RBInertia --  X^* I X^{-1} -- Test1 is " << test1 << std::endl;
        std::cout  << "RBInertia --  X^* I X^{-1} -- Test2 is " << test2 << std::endl;
        std::cout  << "RBInertia --  X^* I X^{-1} -- Test3 is " << test3 << std::endl;
        std::cout  << "RBInertia --  X^* I X^{-1} -- Test4 is " << test4 << std::endl;
    }
  
    if (1)                                                      
    {
        // ApplyTransposw -  X^T I X
        BRBInertia I2 = X.applyTranspose( I ); 
        
        BMatrix6 aux5 = arb::transpose(BMatrix6(X)) * BMatrix6(I) * BMatrix6(X);
        BMatrix6 aux6 = arb::transpose(X) * I * X;
        
        bool test5 =  arb::nearZero(aux5 - BMatrix6(I2));
        bool test6 =  arb::nearZero(aux6 - BMatrix6(I2));
        testsPassed += test5 + test6;
        std::cout  << "RBInertia --  X^T I X -- Test5 is " << test5 << std::endl;
        std::cout  << "RBInertia --  X^T I X -- Test6 is " << test6 << std::endl;
    }
    
    return testsPassed;
}

int
test_abinertia(void)
{
    std::cout << "\nArticulated Inertia Test\n" << std::endl;
    
    // set up single body - a sphere -  
    BScalar mass = 10.0, radius = 1.5;
    double diag = ((2.0/5.0) * mass * (radius * radius)); 
    BVector3 com = BVector3(2.0,-1.0, 10.0); // centre of mass

    BMatrix3 I(diag);
    BMatrix3 M(mass);
    BMatrix3 H = arb::cross(mass * com);
    
    // check inverse 
    BMatrix6 ABI(M, H, glm::transpose(H), I);
    BMatrix6 ABIinv = arb::inverse(BABInertia(M,H,I));
    BMatrix6 I6 = ABI * ABIinv;
   // std::cout << "AB * ABinv = \n" << I6 << "\n\n";
    bool test1 = arb::nearZero(I6 - B_IDENTITY_6x6);
    std::cout << "ABInertia Inverse -- I^{-1} -- Test1 is " << test1 << "\n";
    
    // check ABI transforms
    BABInertia abi(M,H,I);
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(-46.0)), glm::normalize(BVector3(0.2,0.1,-0.8)));
    BVector3 trans(1.0,2.0,3.0);
    BTransform X(rot, trans);

    int testsPassed = 0;
    if (1)
    {
        // Apply - X^* I X^{-1}
        BABInertia I1 = X.apply( abi ); 
        //std::cout << std::endl << BMatrix6(I1) << std::endl << std::endl;
        BMatrix6 aux1 = (arb::transpose(BMatrix6(arb::inverse(X))) * BMatrix6(abi)) * BMatrix6(arb::inverse(X));
        BMatrix6 aux2 = (arb::transpose(arb::inverse(X)) * abi) * arb::inverse(X);
        BMatrix6 aux3 = (arb::dual(X) * abi) * arb::inverse(X);
        bool test2 = arb::nearZero(aux1 - BMatrix6(I1));
        bool test3 = arb::nearZero(aux2 - BMatrix6(I1));
        bool test4 = arb::nearZero(aux3 - BMatrix6(I1));
        testsPassed += test2 + test3 + test4;
        std::cout  << "ABInertia -- X^* I X^{-1} -- Test2 is " << test2 << std::endl;
        std::cout  << "ABInertia -- X^* I X^{-1} -- Test3 is " << test3 << std::endl;
        std::cout  << "ABInertia -- X^* I X^{-1} -- Test4 is " << test4 << std::endl;
    }
    if (1)
    {
        // ApplyTransposw -  X^T I X
        BABInertia I2 = X.applyTranspose( abi ); 
        // std::cout << std::endl << BMatrix6(I2) << std::endl << std::endl;
        BMatrix6 aux2 = arb::transpose(BMatrix6(X)) * BMatrix6(abi) * BMatrix6(X);
        BMatrix6 aux3 = arb::transpose(X) * abi * X;
        //std::cout  << aux2 << std::endl;
        bool test5 = arb::nearZero(aux2 - BMatrix6(I2));
        bool test6 = arb::nearZero(aux3 - BMatrix6(I2));
        testsPassed += test5 + test6;
        std::cout  << "ABInertia -- X^T I X -- Test5 is " << test5 << std::endl;
        std::cout  << "ABInertia -- X^T I X -- Test6 is " << test6 << std::endl;
    }
    return testsPassed;
}

void
check( void )
// consistency checks 
{
    std::cout << "\nARB: Performing consistency checks..." << std::endl;

    int a = test_adjoints();
    int b = test_rbinertia();
    int c = test_abinertia();
    
    int errs = 22 - (a + b + c); // 11 + 6 + 5
    
    if (errs)
    {
        std::cout << "\nARB: Fatal Error - " <<  errs << " internal consistency checks failed!!!\n" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    std::cout << "\nARB: All internal consistency checks passed\n" << std::endl;
}

//
//
//

void
example1( void ) 
// this example is taken from RBDL library https://github.com/rbdl/rbdl
// the output generated by RBDL is QDDot = { -6.54000000  6.54000000  0.00000000 }
{
    std::cout << "\nARB Example 1\n" << std::endl;
    BModel* model = NULL;
    
    unsigned int body_a_id, body_b_id, body_c_id;
    BBody body_a, body_b, body_c;
    BJoint joint_a, joint_b, joint_c;
    
    model = new BModel();
    
    model->gravity( BVector3 (0., -9.81, 0.) );
    
    body_a = BBody (1., BVector3 (0.5, 0., 0.0), BVector3 (1., 1., 1.));
    joint_a = BJoint(
                     BJoint::Revolute,
                     BVector3 (0., 0., 1.)
                     );
    
    body_a_id = model->addBody(0, arb::Xtrans(BVector3(0., 0., 0.)), joint_a, body_a);
    
    body_b = BBody (1., BVector3 (0., 0.5, 0.), BVector3 (1., 1., 1.));
    joint_b = BJoint (
                      BJoint::Revolute,
                      BVector3 (0., 0., 1.)
                      );
    
    body_b_id = model->addBody(body_a_id, arb::Xtrans(BVector3(1., 0., 0.)), joint_b, body_b);
    
    body_c = BBody (0., BVector3 (0.5, 0., 0.), BVector3 (1., 1., 1.));
    joint_c = BJoint (
                      BJoint::Revolute,
                      BVector3 (0., 0., 1.)
                      );
    
    body_c_id = model->addBody(body_b_id, arb::Xtrans(BVector3(0., 1., 0.)), joint_c, body_c);
    

    BModelState qinput;
    
    qinput.q.resize(model->qsize(),0.0);
    qinput.qdot.resize(model->qdotsize(), 0.0);
    qinput.tau.resize(model->qdotsize(), 0.0);
    
    BDynamics dyn;
    dyn.forward(*model, qinput);
    
    std::cout << qinput.qddot << std::endl;
    

    //
    //
    
    std::cout << "\nStream operator test\n" << std::endl;
    
    std::ostringstream outstr;
    outstr << *model;
    
    delete model;
    
    std::istringstream instr(outstr.str());
    BModel model2;
    instr >> model2; 
    
    qinput.q.assign(model2.qsize(),0.0);
    qinput.qdot.assign(model2.qdotsize(), 0.0);
    qinput.tau.assign(model2.qdotsize(), 0.0);
    
    dyn.forward(model2, qinput);
    
    std::cout << qinput.qddot << std::endl;
}

void
example2( void )
// Complex example to test code -- doesn't make physical sense
// the forward output generated by RBDL on this example is
// QDOT = { 0.00000621, -9.80999590, 0.00000644, 31.43004589, -0.74277671, 15.33985878, -65.28759632, 54.91808109, -40.79517561, 3.69334706, 9.57489516 }
{
    std::cout << "\nARB Example 2\n" << std::endl;
    
    double massA = 100000000.0;
    std::vector<double> com_a = { 0.0, 0.0, 0.0 };
    std::vector<double> trans_a = { 0.0, 20.0, 0.0 };

    double massC = 100.0;
    std::vector<double> com_c = { 0.0, 0.0, 0.0 };
    std::vector<double> trans_c = { 0.0, 10.0, 0.0 };
    
    BModel* model = new BModel(15);
    
    model->gravity( BVector3(0.0, -9.81, 0.0));
    
    
    BJoint joint_a = BJoint( BJoint::FloatingBase );
    BBody body_a = BBody(massA, glm::dvec3(com_a[0], com_a[1], com_a[2]), BVector3(10.0, 10.0, 10.0));
    int body_a_id = model->addBody(0, arb::Xtrans(BVector3(trans_a[0],trans_a[1],trans_a[2])), joint_a, body_a);

    BVector6 helical(0.25, 0.25, 0.25, 0.25, 0.25, 0.0); 
    BJoint joint_b = BJoint( helical );
    BBody body_b = BBody(5.0, glm::dvec3(0.0, 0.75, 0.0), BVector3(2.0, 2.0, 2.0));
    int body_b_id = model->addBody(body_a_id, arb::Xtrans(BVector3(1.0, 0.0, 0.0)), joint_b, body_b);
    
    BJoint joint_c = BJoint( BJoint::Spherical ); 
    BBody body_c  = BBody(massC, BVector3(com_c[0], com_c[1], com_c[2]), BVector3(2.0, 2.0, 2.0));
    int body_c_id = model->addBody(body_b_id, arb::Xtrans(BVector3(trans_c[0], trans_c[1], trans_c[2])), joint_c, body_c);
    
    BJoint joint_d = BJoint( BJoint::Revolute, BVector3(0.0, 0.0, 1.0));
    BBody body_d = BBody(10.0, BVector3 (0.5, 0.0, 0.0), BVector3(2.0, 2.0, 2.0));
    int   body_d_id = model->addBody(body_c_id, arb::Xtrans(BVector3(1.0, 1.0, 0.0)), joint_d, body_d);
    

    
    // one for each body (including 0 body)
    std::vector<BVector6> f_ext(11);
    f_ext[0].set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    f_ext[1].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[2].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[3].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[4].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[5].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[6].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    f_ext[7].set(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    
    f_ext[8].set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    f_ext[9].set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    f_ext[10].set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    
    BModelState qinput;
    
    qinput.q.resize(model->qsize(),1.0);
    qinput.qdot.resize(model->qdotsize(), 1.0);
    qinput.tau.resize(model->qdotsize(), 0.0);
    
    BDynamics dyn;
    dyn.forward(*model, qinput, f_ext);
    
    std::cout << qinput.qddot << std::endl;

    delete model;
}


// Example 3 
void
single_body( void ) 
{
    std::cout << "\nARB Example 3 - Single Body\n" << std::endl;
    // note spatial vectors - (angular,linear) - a force or velocity
    BModel* model = new BModel;

    model->gravity( BVector3(0.0, -9.81, 0.0) );
    //model->gravity( BVector3(0.0, 0.0, 0.0) );
  
    // set up single body -- i.e. a spaceship 
    // floating base only used for first body in model.
    BBody spaceship = BBody(10.0,  B_ZERO_3, BVector3(1.0, 1.0, 1.0));
    BJoint joint = BJoint( BJoint::FloatingBase );
    BBodyId spaceshipId = model->addBody(0, arb::Xtrans(B_ZERO_3), joint, spaceship);
    
    // control -- apply myforce to whole spaceship model -- 100 Newtons along z axis
    // body[0] is a header, body[1] is virtual, body[2] is the first, real body with mass
    // for floating base we only need apply external force to body[2]
    // external forces are assumed to be in world coordinates
    BVector6 myforce( B_ZERO_3, 0.0, 0.0, 100.0);
    BExtForce f_ext(model->bodies(), B_ZERO_6); 
    f_ext[spaceshipId] =  BVector6( myforce );
    
    BModelState qinput;
    
    qinput.q.resize(model->qsize(),0.0);
    qinput.qdot.resize(model->qdotsize(), 0.0);
    qinput.tau.resize(model->qdotsize(), 0.0);
    
    BDynamics dyn;
    dyn.forward(*model, qinput, f_ext);
    
    std::cout << "linear acceleration (" << qinput.qddot[0] << ", " << qinput.qddot[1] << ", " << qinput.qddot[2] << std::endl;
    std::cout << "angular acceleration (" << qinput.qddot[3] << ", " << qinput.qddot[4] << ", " << qinput.qddot[5] << std::endl;
    
    std::cout << std::endl;
    
    delete model;
}

void
print( double T, const BVector6 &mv )
{
    const double M_2PI = M_PI + M_PI;
    
    std::cout << T << ") ";
    
    BVector3 pos = mv.lin();
    std::cout << " pos( " <<  pos.x << ", " <<  pos.y << ", " <<  pos.z  << " ) - ";
 
    BVector3 ang = mv.ang();
  
    ang.x = std::remainder(ang.x, M_2PI);
    ang.y = std::remainder(ang.y, M_2PI);
    ang.z = std::remainder(ang.z, M_2PI);
    ang  = glm::degrees(ang);
    
    std::cout << "orient( " <<  ang.x << "°, " <<  ang.y << "°, " <<  ang.z << "° )" << std::endl;
}

BRBInertia
sphere( double mass, double radius)
// set up  a sphere   
{
    double diag = ((2.0/5.0) * mass * (radius * radius));
    BMatrix3 I_o(diag); // rotational inertia at body coordinate frame origin
    
    BVector3 h(0.0);    // linear momentum
    
    return BRBInertia(mass, h, I_o);
}

void
newton_euler( void ) 
// Spatial algebra test
// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
// https://en.wikipedia.org/wiki/Newton–Euler_equations
{
    std::cout << "\nARB Example 4 - Newton Euler\n" << std::endl;
    // set up single body - a sphere -  
    BRBInertia I = sphere(100.0, 0.5); 
    BMatrix6 invI = arb::inverse(I);
    
    BVector6 force = B_ZERO_6;
    BVector6 pos   = B_ZERO_6;
    BVector6 vel   = B_ZERO_6;
    BVector6 acc   = B_ZERO_6;
    
    // set some initial position
    pos.lin( 20.0, 50.0, 3.0 );
    
    // apply some force
    force[1] = 1.0;   // ang
    force[5] = 100.0; // lin
    
    // over time
    double dt = 0.1;
    double T = 0.0;
    int Iters = 100;
    
    for (int t = 0; t < Iters; ++t)
    {
        if (!(t % 10))
            print( T, pos );

        // Implicit Newton-Euler intergration 
        // see Featherstone, Section 2.14, page 35
        acc = invI * (force - arb::crossf(vel, I * vel)); 
        vel += acc * dt;
        pos += vel * dt;
        //
        //
        
        T += dt;
        
        if (t == 10) // switch off force after t = 10 time steps
            force = B_ZERO_6;
    }
}

BMatrix6 
toWorldInvInertia( BBody &body )
// convert 'body coordinate' rotational inertia I_o to 'world coordinates' I_w 
// Note in a rotating body I_o is constant while I_w is changing 
{
    BMatrix6 invI = arb::inverse(body.I());
    
    BMatrix6 X = arb::toAdjointDual(BTransform(body.X_base().E()));
    return X * invI * arb::transpose(X);
}

void
impulse( void )
// RBDA, chapter 11, Contact and Impact. 
// Two-Body Collisions, page 232, eqn 11.65
/*
 ''two rigid bodies, body1 and body2, that collide at a single point C.
 Their velocities before the impact are v1 and v2, and their velocities afterwards
 are v1 + ∆v1 and v2 + ∆v2. We assume initially that there is no friction at
 the contact.''
 */
{
    std::cout << "\nARB Example 5 - Spatial Impulse\n" << std::endl;
    
    // body mass, radius
    BBody body1(sphere(100.0, 1.5));
    BBody body2(sphere(85.0, 1.5));

    // body velocity
    body1.v(BVector6(B_ZERO_3, 0.0, 0.0, 2.0));   // v1 moving along z-axis towards v2
    body2.v(BVector6(B_ZERO_3, 0.0, 0.0, -3.0));  // v2 moving along -z-axis towards v1

    // details of direct contact/collision between body1 and body2
    glm::dvec3 C(0.0, 0.0, 1.5);  // contact point in world coordinates
    glm::dvec3 n(0.0, 0.0, 1.0);  // contact normal (already normalized)
  
    // coefficient of restitution (0 -> inelastic, like clay, 1.0 -> elastic, like a billiard ball)
    BScalar e = 1.0;
    
    // n_s is the unit spatial impulse
    BVector6 n_s = BVector6(arb::cross(C, n), n); 
    
    // K or effective mass
    const BMatrix6 invI1 = toWorldInvInertia(body1);
    const BMatrix6 invI2 = toWorldInvInertia(body2);
    BScalar K = arb::dot(n_s, ((invI1 + invI2) * n_s));
    
    // current relative velocity (the separation velocity at contact i, eqn 11.62)
    BScalar ns_dot_vel = arb::dot(n_s, body2.v() - body1.v());
    
    // size or magnitude of impulse 
    BScalar j = (-(1.0 + e) * ns_dot_vel) / K;
    
    // apply spatial impulse to velocity
    const BVector6 impulse = n_s * j;
    body1.v() -= invI1 * impulse;
    body2.v() += invI2 * impulse;
    
    std::cout << "Body1 " <<  body1.v() << std::endl;
    std::cout << "Body2 " <<  body2.v() << std::endl << std::endl;
    
    // general 1D Newtonian elastic collision equations derived from the 
    // conservation of momentum and kinetic energy.
    // https://en.wikipedia.org/wiki/Elastic_collision
    BScalar mA = body1.I().mass();
    BScalar uA = 2.0; // body1.v()[5];
    BScalar mB = body2.I().mass();
    BScalar uB = -3.0; // body1.v()[5];
    BScalar vA = (((mA - mB)/(mA + mB)) * uA) + (((2.0 * mB)/(mA + mB)) * uB);
    BScalar vB = (((2.0 * mA)/(mA + mB)) * uA) + (((mB - mA)/(mA + mB)) * uB);
    
    std::cout << "Should equal " << "vA " << vA  << ", vB " <<  vB << std::endl;
}

int 
main()
{
    std::cout.precision(8);
    std::cout.setf( std::ios::fixed, std::ios::floatfield );
    
    std::cout << "\nARB Demo\n" << std::endl;

    // internal consistency checks 
    check();
    
    // example1 is taken from RBDL library https://github.com/rbdl/rbdl
    // the output generated by RBDL is QDDot = { -6.54000000  6.54000000  0.00000000 }
    example1();
    
    // the ForwardDynamics output generated by RBDL on this example is
    // QDDot = { 0.00000621, -9.80999590, 0.00000644, 31.43004589, -0.74277671, 15.33985878, -65.28759632, 54.91808109, -40.79517561, 3.69334706, 9.57489516 }
    // you should notice slight differences in the last decimal places
    example2();
    
    single_body();
    
    newton_euler();
    
    impulse();
}
