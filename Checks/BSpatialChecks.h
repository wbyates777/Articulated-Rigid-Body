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
 
 This keeps all tests in one place.
 
*/



#ifndef __BSPATIALCHECKS_H__
#define __BSPATIALCHECKS_H__


//
// randomised consistency checks - return number of errors - 0 on success
//

int
check( int iters = 1000, int seed = 123 );


#endif


