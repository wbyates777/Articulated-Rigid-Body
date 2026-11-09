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
    rndInt( int min, int max ); // [min,max]
    
    double 
    rndFloat( void ); // (0,1)
    
    double 
    rndFloat(double min, double max);  // (min,max)


    //
    // 3D Types
    //
    
    BVector3 
    rndVec3( void );
    
    BVector3 
    rndVec3( double min, double max ); 
    
    BVector3 
    rndAxis( void ); // X, Y, or Z

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
    
    /// Reference implementation of the STANDARD SE(3) exponential (Modern Robotics, Lynch and Park).
    BMatrix4
    exp_std( const BVector3 &w, const BVector3 &v, int terms = 40 );

    BMatrix4
    exp_std( const BVector6 &v, int terms = 40 );
    
}


#endif


