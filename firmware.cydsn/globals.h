// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/** 
* \file 		globals.h
*
* \brief 		Global definitions and macros are set in this file.
* \date 		Jul 29, 2013
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "/qbmoveAPI/commands.h"

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION         "QBMMP v1.0.4"

#define NUM_OF_MOTORS   2
#define NUM_OF_SENSORS  3

//==============================================================================
//                                           CONDITIONAL COMPILATION DEFINITIONS
//==============================================================================

//#define RESET                       	// Resets the memory with standard
//                                    	// parameters

//==============================================================================
//                                                                       CONTROL
//==============================================================================

//=======================================================     control mode types

#define CONTROL_ANGLE		    0
#define CONTROL_PWM				1
#define CONTROL_CURRENT			2

//==================================================     control type definition

//#define CONTROL_MODE			CONTROL_CURRENT
//#define CONTROL_MODE			CONTROL_ANGLE
#define CONTROL_MODE            CONTROL_PWM

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE			0
#define TRUE			1

#define PWM_LIMIT       100

#define SAMPLES_FOR_MEAN 250

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

struct st_ref {
	int32 pos[NUM_OF_MOTORS];		// motor reference position
	uint8 onoff;					// enable flags
};

//=============================================================     measurements

struct st_meas {
	int32 pos[NUM_OF_SENSORS];		// sensor position

    int32 curr[NUM_OF_MOTORS];		// motor currents

    int16 rot[NUM_OF_SENSORS];		// sensor rotations
};

//==============================================================     data packet

struct st_data {
	uint8 	buffer[128];					// CMD/DATA/CHECKSUM
	int16 	length;							// length
	int16 	ind;							// index
	uint8 	ready;							// Flag
};

//============================================     settings stored on the memory 

struct st_mem {
    uint8 	flag; 						// Device has been configured 
	uint8 	id;							// device ID
	int32 	k;							// Proportional constant
    uint8   activ;     					// Activation upon startup
    uint8   mode;       				// Input mode
    uint8   res[NUM_OF_SENSORS];    	// Angle resolution
    float   filt;       				// Measurement filter
    float   dead;       				// Control deadzone
    int32   m_off[NUM_OF_SENSORS];		// Measurement offset
    float   m_mult[NUM_OF_SENSORS];		// Measurement multiplier
    uint8	pos_lim_flag;				// Position limit active/inactive
    int32	pos_lim_inf[NUM_OF_MOTORS]; // Inferior position limit for motors
    int32	pos_lim_sup[NUM_OF_MOTORS]; // Superior position limit for motors
};

//=================================================     device related variables

struct st_dev{
	int32	tension;				// Power supply tension
    float   tension_conv_factor;    // Used to calculate input tension
    uint8    tension_valid;
};

//====================================      external global variables definition



extern struct st_ref 	g_ref;			// motor variables
extern struct st_meas 	g_meas;			// measurements
extern struct st_data 	g_rx;			// income data
extern struct st_mem 	g_mem, c_mem;	// memory
extern struct st_dev	device;			//device related variables
	


// -----------------------------------------------------------------------------


#endif

//[] END OF FILE