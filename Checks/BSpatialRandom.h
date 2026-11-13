/*! 

         $$$$$$$$$$$$$$$$$$$$$$$$
         $   BSpatialRandom.h   $
         $$$$$$$$$$$$$$$$$$$$$$$$

 @file BSpatialRandom.h
 @ingroup Checks
 date 07/04/2026
 
 by W.B. Yates
 
 ---
 
 @brief Generate random numbers and random geometric types for testing.
 
*/


#ifndef BSPATIALRANDOM_H
#define BSPATIALRANDOM_H


#ifndef BSPATIALALGEBRA_H
#include "BSpatialAlgebra.h"
#endif

#ifndef BMODEL_H
#include "BModel.h"
#endif

#ifndef BMODELSTATE_H
#include "BModelState.h"
#endif

namespace arb
{
    
    // set the seed and reinitialise the sequence
    void
    rndSeed( unsigned int s );  
    
    //
    // Basic Random Numbers
    //
    int
    rndInt( int N ); // [0,N]
    
    int
    rndInt( int min, int max ); // [min, max]
    
    double 
    rndFloat( void ); // (0,1)
    
    double 
    rndFloat(double min, double max);  // (min, max)


    //
    // 3D Types
    //
    
    BVector3 
    rndVec3( void );
    
    BVector3 
    rndVec3( double min, double max ); 
    
    BVector3 
    rndAxis( void ); // X, Y, or Z

    BVector3
    rndNormal3( void ); // random, non-zero, normalized 3-Vector


    BMatrix3
    rndMat3( void );

    BMatrix3 
    rndRot( void );

    BQuat 
    rndQuat( void );

    
    //
    // Spatial Types
    //
    
    BVector6 
    rndVec6( void );

    BMatrix6  
    rndMat6( void );

    BMatrix63  
    rndMat63( void );

    BTransform
    rndTransform( void );

    BInertia 
    rndInertia( void );
    
    BRBInertia 
    rndRBInertia( void );
    
    BABInertia 
    rndABInertia( void );
    

    //
    // Random model and model state.
    //

    /// @brief Generate a random tree-structured model of bodyNum bodies and joints.
    ///
    /// Returns a model consisting of numBody randomly generated bodies (with mass and inertial characteristics)
    /// connected using joints randomly selected from a set of 16 BJoint::jtype types.
    /// Random models should be validated.
    /// @param[in] bodyNum  construct a random model of numBody bodies and joints
    /// @return a random tree-structured model
    /// @warning Random models cannot generally be saved or loaded via the URDFManager because they contain joint types not
    /// supported by the URDF format (see URDFManager.h).
    BModel
    rndModel( int bodyNum );


    /// @brief Test the validity of model.
    /// A models can be validated by testing it against the identity
    ///
    ///             ABA(q, qdot, RNEA(q, qdot, acc)) ≈ acc
    ///
    /// for some given accelerations acc.
    ///
    /// Models can fail this test for a number of physical or mathematical reasons.
    ///
    /// - Physical issues: Models with large mass ratios or inertias whose
    ///    centers of mass (COMs) are far from the body's frame origin can exhibit
    ///    instability when gravity is enabled.
    /// - Mathematical issues: Euler angle singularities can cause failures (gimbal lock),
    ///    i.e when EulerXYZ hits ±M_PI_2 on the middle axis.
    ///
    /// @param[in] model an articulated ridgid body model
    /// @param[in] Ntrials the number of random model states to test
    /// @param[in] tol the maximum error tolerance
    /// @return true if the model passes the test; false otherwise.
    bool
    isvalid( BModel &model, int Ntrials = 40, double tol = 1E-3 );


    /// Generate a non-zero random initial state for model m using any limits set in joint params
    BModelState
    rndModelState( const BModel &m );

}


#endif


