/* BSpatialChecks 08/12/2025

 $$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialChecks.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Not really a part of the Algebra or the Model.
 
 A number of basic functions and consistency checks to help with debugging/maintanence
 You can use/improve these functions to build a more comprehensive test suite
 
 This keeps tests all tests in one place.
 
*/


#ifndef __BSPATIALCHECKS_H__
#define __BSPATIALCHECKS_H__

#ifndef __BSPATIALALGEBRA_H__
#include "BSpatialAlgebra.h"
#endif



namespace arb 
{

    
    void
    rndSeed( unsigned int s );   // set the seed and reinitialise the sequence
 
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
    
    BTransform
    rndTransform( void );

    BInertia 
    rndInertia( void );
    
    BRBInertia 
    rndRBInertia( void );
    
    BABInertia 
    rndABInertia( void );
    
}

//
// randomised consistency checks - return number of errors - 0 on success
//
int
check( int iters = 1000, int seed = 123 );



#endif


