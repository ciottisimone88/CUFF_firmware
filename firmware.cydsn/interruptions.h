// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//=====================================================     function definitions
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);
CY_ISR_PROTO(ISR_CALIBRATE_ExInterrupt);


void measurements_int(void);
uint8 BITChecksum(uint32 mydata);

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */