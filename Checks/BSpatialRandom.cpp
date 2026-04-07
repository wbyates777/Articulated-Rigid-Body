/* BSpatialRandom 07/04/2026

 $$$$$$$$$$$$$$$$$$$$$$$$$$
 $   BSpatialRandom.cpp   $
 $$$$$$$$$$$$$$$$$$$$$$$$$$

 by W.B. Yates
 Copyright (c) W.B. Yates. All rights reserved.
 History:

*/


#ifndef __BSPATIALRANDOM_H__
#include "BSpatialRandom.h"
#endif


#ifndef __URAND_H__
#include "URand.h"
#endif


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

namespace arb 
{
    
    const double RANGE     = 10.0;
    const double ZERO_PROB = 0.95;

    
    URand global_ran(22);
    
    // set the seed and reinitialise the sequence
    void
    rndSeed( unsigned int s )  { return global_ran.seed(s); }
    
    int
    rndInt( int N ) { return global_ran.rndInt(N); }
    
    int
    rndInt( int min, int max ) { return global_ran.rndInt(min, max); }
    
    double 
    rndFloat( void ) { return global_ran.rndFloat(); }
    
    double 
    rndFloat( double min, double max ) { return global_ran.rndFloat(min, max); }
    

    
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
 
        assert(inertia.valid());
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

}  



