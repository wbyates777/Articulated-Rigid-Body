/* BDynamics 20/02/2024

 $$$$$$$$$$$$$$$$$$$
 $   BDynamics.h   $
 $$$$$$$$$$$$$$$$$$$

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

void
test_adjoints( void )
{
    std::cout << "\nSpatial Adjoint Test\n" << std::endl; 
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(46.0)), glm::normalize(BVector3(-1.1,-2.2,1.3)));
    BSpatialTransform X(rot, BVector3(1.0, 2.0, 3.0));
    
    const BSpatialVector force(-1.1, 1.2, -1.3, 4.0, 0.0, 6.0);

  // const BTransform X = arb::Xtrans(glm::dvec3(1.0,2.0,3.0)); // pure translation
   //const BTransform X = arb::Xrot(0.3, glm::dvec3(-1.1,-2.2,1.3)); // pure rotation
    
    
    if (1)
    {
        std::cout << "Adjoint  -- Ad_X --------------------" << std::endl;
        BSpatialVector x1 =  arb::applyAdjoint( X, force ); 
        BSpatialMatrix m1 =  arb::toAdjoint(X); 
        BSpatialVector x2 =  m1 * force;
        std::cout << x1 << std::endl;
        std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        bool test2 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjointInverse(X, force) ) - force);
        bool test3 = arb::nearZero(arb::applyAdjoint(X, arb::applyAdjoint(arb::inverse(X), force) ) - force);
        std::cout  << "Test1 is " << test1 << std::endl;
        std::cout  << "Test2 is " << test2 << std::endl;
        std::cout  << "Test3 is " << test3 << std::endl;
    }
    if (1)
    {
        std::cout << "Adjoint Transpose -- Ad_X^{T} --------------------" << std::endl;
        BSpatialVector x1 =  arb::applyAdjointTranspose( X, force ); 
        BSpatialMatrix m1 =  arb::toAdjointTranspose(X);
        BSpatialVector x2 =  m1 * force;
        std::cout << x1 << std::endl;
        std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        std::cout  << "Adjoint Transpose test is " << test1 << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
    }
    if (1)
    {
        std::cout << "Adjoint Inverse -- Ad_X^{-1} -------------------------" << std::endl;
         BSpatialVector x1 =  arb::applyAdjointInverse( X, force ); 
        BSpatialMatrix m1 =  arb::toAdjointInverse(X);
        BSpatialVector x2 =  m1 * force;
        std::cout << x1 << std::endl;
        std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        std::cout  << "Adjoint Inverse test is " << test1 << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
     }
  
    if (1)
    {
        std::cout << "Adjoint Dual -- Ad_X^{-T} --------------------" << std::endl;
        BSpatialVector x1 =  arb::applyAdjointDual( X, force ); 
        BSpatialMatrix m1 =  arb::toAdjointDual(X);
        BSpatialVector x2 =  m1 * force;
        std::cout << x1 << std::endl;
        std::cout << x2 << std::endl;
        bool test1 = arb::nearZero(x1 - x2);
        std::cout  << "Adjoint Dual test is " << test1 << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
    }
    
    if (1)
    {
        std::cout << "More Inverse Checks -- X^{-1}--------------------" << std::endl;
        BSpatialTransform Y(X.E(), BVector3(0.1, 2.0, 0.3));
        BSpatialTransform Z = Y * arb::inverse(Y);
        //std::cout << Z << std::endl;
        bool test1 = arb::nearZero(BSpatialMatrix(Z) - B_IDENTITY_6x6);
        std::cout  << "Inverse test1 is " << test1 << std::endl;
 
        auto P =  arb::toAdjoint(X) *  arb::toAdjointInverse(X);
        //std::cout << P << std::endl;
        bool test2 = arb::nearZero(P - B_IDENTITY_6x6);
        std::cout  << "Inverse test2 is " << test2 << std::endl;

        auto Q = arb::toAdjointInverse(X) * arb::toAdjoint(X);
        //std::cout << Q << std::endl;
        bool test3 = arb::nearZero(Q - B_IDENTITY_6x6);
        std::cout  << "Inverse test3 is " << test3 << std::endl;
        std::cout << "-------------------------------------" << std::endl;
    }
    
    if (1)
    {
        std::cout << "More Transpose Checks -- X^{T} -----------------" << std::endl;
        BSpatialMatrix tmp4 = arb::toAdjointTranspose(X);
        //std::cout << tmp4 << std::endl;
        BSpatialMatrix tmp5 =  arb::transpose(arb::toAdjoint(X)); 
        //std::cout << tmp5 << std::endl;
        bool test1 = arb::nearZero(tmp4 - tmp5);
        std::cout  << "Transpose test1 is " << test1 << std::endl;
   
        BSpatialMatrix tmp6 =  arb::transpose(arb::toAdjointInverse(X)); 
        //std::cout << tmp6 << std::endl;
        BSpatialMatrix tmp7 = arb::toAdjointDual(X);
        //std::cout << tmp7 << std::endl;
        bool test2 = arb::nearZero(tmp6 - tmp7);
        std::cout  << "Transpose test2 is " << test2 << std::endl;
        std::cout << "-------------------------------------" << std::endl;
    }
}

void
test_rbinertia( void )
{
    std::cout << "\nSpatial Inertia Test\n" << std::endl; 
    
    double mass = 59.0, radius = 0.5;
    BVector3 com(1.0);
    BVector3 gyration((2.0/5.0) * mass * (radius * radius));
    BBody sphere(mass, com, gyration);
    
    BRBInertia I = sphere.I();
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(46.0)), glm::normalize(BVector3(1.1, 2.2, 3.3)));
    BVector3 trans = BVector3(-0.9, 8.0, 3.0);
    
    BSpatialTransform X(rot, trans);
    
    if (1)
    {   
        // Apply - X^* I X^{-1}
        BRBInertia I1 = X.apply( I ); 
        
        BSpatialMatrix aux1 = arb::transpose(BSpatialMatrix(arb::inverse(X))) * BSpatialMatrix(I) * BSpatialMatrix(arb::inverse(X));
        BSpatialMatrix aux2 = arb::dual(X) * BSpatialMatrix(I) * BSpatialMatrix(arb::inverse(X));
        BSpatialMatrix aux3 = arb::dual(X) * I * arb::inverse(X);
        BSpatialMatrix aux4 = arb::transpose(arb::inverse(X)) * BSpatialMatrix(I) * arb::inverse(X);
        
        bool test1 = arb::nearZero(aux1 - BSpatialMatrix(I1));
        bool test2 = arb::nearZero(aux2 - BSpatialMatrix(I1));
        bool test3 = arb::nearZero(aux3 - BSpatialMatrix(I1));
        bool test4 = arb::nearZero(aux4 - BSpatialMatrix(I1));
        std::cout  << "Test1 is " << test1 << std::endl;
        std::cout  << "Test2 is " << test2 << std::endl;
        std::cout  << "Test3 is " << test3 << std::endl;
        std::cout  << "Test4 is " << test4 << std::endl;
    }
  
    if (1)                                                      
    {
        // ApplyTransposw -  X^T I X
        BRBInertia I2 = X.applyTranspose( I ); 
        
        BSpatialMatrix aux5 = arb::transpose(BSpatialMatrix(X)) * BSpatialMatrix(I) * BSpatialMatrix(X);
        BSpatialMatrix aux6 = arb::transpose(X) * I * X;
        
        bool test5 =  arb::nearZero(aux5 - BSpatialMatrix(I2));
        bool test6 =  arb::nearZero(aux6 - BSpatialMatrix(I2));
        std::cout  << "Test5 is " << test5 << std::endl;
        std::cout  << "Test6 is " << test6 << std::endl;
    }
}

void
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
    BSpatialMatrix AB(M, H, glm::transpose(H), I);
    BSpatialMatrix ABinv = arb::inverse(BABInertia(M,H,I));
    
    BSpatialMatrix I6 = AB * ABinv;
   // std::cout << "AB * ABinv = \n" << I6 << "\n\n";
    bool test1 = arb::nearZero(I6 - B_IDENTITY_6x6);
    std::cout << "Inverse test is " << test1 << "\n";
    
    // check ABI transforms
    
    BABInertia abi(M,H,I);
    
    BMatrix3 rot = glm::rotate(glm::radians(BScalar(-46.0)), glm::normalize(BVector3(0.2,0.1,-0.8)));
    BVector3 trans(1.0,2.0,3.0);
    BSpatialTransform X(rot, trans);

    if (1)
    {
        // Apply - X^* I X^{-1}
        BABInertia I1 = X.apply( abi ); 
        //std::cout << std::endl << BSpatialMatrix(I1) << std::endl << std::endl;
        BSpatialMatrix aux1 = (arb::transpose(BSpatialMatrix(arb::inverse(X))) * BSpatialMatrix(abi)) * BSpatialMatrix(arb::inverse(X));
        BSpatialMatrix aux2 = (arb::transpose(arb::inverse(X)) * abi) * arb::inverse(X);
        BSpatialMatrix aux3 = (arb::dual(X) * abi) * arb::inverse(X);
        //std::cout  << aux1 << std::endl;
        bool test2 = arb::nearZero(aux1 - BSpatialMatrix(I1));
        bool test3 = arb::nearZero(aux2 - BSpatialMatrix(I1));
        bool test4 = arb::nearZero(aux3 - BSpatialMatrix(I1));
        std::cout  << "Test2 apply is " << test2 << std::endl;
        std::cout  << "Test3 apply is " << test3 << std::endl;
        std::cout  << "Test4 apply is " << test4 << std::endl;
    }
    if (1)
    {
        // ApplyTransposw -  X^T I X
        BABInertia I2 = X.applyTranspose( abi ); 
        // std::cout << std::endl << BSpatialMatrix(I2) << std::endl << std::endl;
        BSpatialMatrix aux2 = arb::transpose(BSpatialMatrix(X)) * BSpatialMatrix(abi) * BSpatialMatrix(X);
        BSpatialMatrix aux3 = arb::transpose(X) * abi * X;
        //std::cout  << aux2 << std::endl;
        bool test5 = arb::nearZero(aux2 - BSpatialMatrix(I2));
        bool test6 = arb::nearZero(aux3 - BSpatialMatrix(I2));
        std::cout  << "Test5 applyTranspose is " << test5 << std::endl;
        std::cout  << "Test6 applyTranspose is " << test6 << std::endl;
    }
}


void
example1( void ) 
// this example is taken from RBDL library https://github.com/rbdl/rbdl
// the output generated by RBDL is QDDot = { -6.54000000  6.54000000  0.00000000 }
{
    
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

    BSpatialVector helical(0.25, 0.25, 0.25, 0.25, 0.25, 0.0); 
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
    std::vector<BSpatialVector> f_ext(11);
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
    BSpatialVector myforce( B_ZERO_3, 0.0, 0.0, 100.0);
    BExtForce f_ext(model->bodies(), B_ZERO_6); 
    f_ext[spaceshipId] =  BSpatialVector( myforce );
    
    BModelState qinput;
    
    qinput.q.resize(model->qsize(),0.0);
    qinput.qdot.resize(model->qdotsize(), 0.0);
    qinput.tau.resize(model->qdotsize(), 0.0);
    
    BDynamics dyn;
    dyn.forward(*model, qinput, f_ext);
    

    std::cout << "linear acceleration (" << qinput.qddot[0] << ", " << qinput.qddot[1] << ", " << qinput.qddot[2] << std::endl;
    std::cout << "angular acceleration (" << qinput.qddot[3] << ", " << qinput.qddot[4] << ", " << qinput.qddot[5] << std::endl;
    
    std::cout  << std::endl;
    
    delete model;

}

void
print( double T, const BSpatialVector &mv )
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

void
newton_euler( void ) 
// Spatial algebra test
// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
// https://en.wikipedia.org/wiki/Newton–Euler_equations
{

    // set up single body - a sphere -  
    double mass = 100.0, radius = 0.5;
    double diag = ((2.0/5.0) * mass * (radius * radius));
    BMatrix3 I_o(diag); // rotational inertia at body coordinate frame origin
    BVector3 h(0.0); // linear momentum
 
    BRBInertia I = BRBInertia(mass, h, I_o); 
    BSpatialMatrix invI = arb::inverse(I);
    
    BSpatialVector force = B_ZERO_6;
    BSpatialVector pos   = B_ZERO_6;
    BSpatialVector vel   = B_ZERO_6;
    BSpatialVector acc   = B_ZERO_6;
    
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

        //
        // see Featherstone, Section 2.14, page 35.
        //
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


int 
main()
{
    std::cout.precision(8);
    std::cout.setf( std::ios::fixed, std::ios::floatfield );
    
    // consistency checks 
    test_adjoints();
    test_rbinertia();
    test_abinertia();
    
    // example1 is taken from RBDL library https://github.com/rbdl/rbdl
    // the output generated by RBDL is QDDot = { -6.54000000  6.54000000  0.00000000 }
    example1();
    

    // the ForwardDynamics output generated by RBDL on this example is
    // QDDot = { 0.00000621, -9.80999590, 0.00000644, 31.43004589, -0.74277671, 15.33985878, -65.28759632, 54.91808109, -40.79517561, 3.69334706, 9.57489516 }
    // you should notice slight differences in the last decimal places
    example2();
    
        
    single_body();
    
    newton_euler();

}
