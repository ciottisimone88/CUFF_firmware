// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/** 
* \file 		utils.h
*
* \brief 		Definition of utility functions.
* \date 		Feb 16, 2014
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/

#include <utils.h>
#include <math.h>

//--------------------------------------------------------------     DEFINITIONS

#define ALPHA 512
#define BETA  300

#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))

//==============================================================================
//																 CURRENT FILTERS
//==============================================================================

int32 filter_i1(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

	old_value = aux;

	return aux;
}

int32 filter_i2(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

	old_value = aux;

	return aux;
}


//velocity filters

int32 filter_vel_1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}

int32 filter_vel_2(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}

int32 filter_vel_3(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}


//==============================================================================
//																	BIT CHECKSUM
//==============================================================================


uint8 BITChecksum(uint32 mydata) {
	uint8 i;
	uint8 checksum = 0;
	for(i = 0; i < 31; ++i)
	{
       	checksum = checksum ^ (mydata & 1);
		mydata = mydata >> 1;
	}
	return checksum;
}


//==============================================================================
//                                                             CHECKSUM FUNCTION
//==============================================================================

uint8 LCRChecksum(uint8 *data_array, uint8 data_length) {
    uint8 i;
    uint8 checksum = 0x00;
    for(i = 0; i < data_length; ++i)
    {
       checksum = checksum ^ data_array[i];
    }
    return checksum;
}

/* [] END OF FILE */