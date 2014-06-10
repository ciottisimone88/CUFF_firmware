// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file         globals.c
*
* \brief        Global variables.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref;                  // motor variables
struct st_meas  g_meas;                 // measurements
struct st_data  g_rx;                   // income data
struct st_mem   g_mem, c_mem;           // memory
struct st_dev   device;                 // device related variables

uint8 calibration_flag;

uint16 timer_value;