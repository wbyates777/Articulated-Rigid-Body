/* BSpatialRandom 07/04/2026

 $$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialRandom.h   $
 $$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

 Take care of generating random numbers and random geometric types for testing
 
*/


#ifndef __BSPATIALRANDOM_H__
#define __BSPATIALRANDOM_H__


#ifndef __BSPATIALALGEBRA_H__
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
    
    BTransform
    rndTransform( void );

    BInertia 
    rndInertia( void );
    
    BRBInertia 
    rndRBInertia( void );
    
    BABInertia 
    rndABInertia( void );
    
}


#endif


