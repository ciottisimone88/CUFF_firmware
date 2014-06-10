// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/** 
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         Jul 29, 2013
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "commands.h"


//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION         "QBMMP v4.0.3"

#define NUM_OF_MOTORS   2
#define NUM_OF_SENSORS  3

//==============================================================================
//                                                                       CONTROL
//==============================================================================

#define PWM_MAX_VALUE   100         // PWM is from 0 to 100, this value is used
                                    // to limit this value
#define PWM_DEAD        0           // deadband value, is directly added to the
                                    // value of PWM always limited to 100

#define ANTI_WINDUP     1000
#define MAX_CURRENT     1000        // Max current for calibration (mA)

//=======================================================     control mode types

#define CONTROL_ANGLE           0
#define CONTROL_PWM             1
#define CONTROL_CURRENT         2


//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 3000 Hz

#define ANALOG_MEASUREMENTS_DIV 1       // 3000 Hz
#define ENCODER_READ_DIV        3       // 1000 Hz
#define MOTOR_CONTROL_DIV       6       // 500 Hz
#define CALIBRATION_DIV         300     // 10 Hz

#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE           0
#define TRUE            1

#define DEFAULT_EEPROM_DISPLACEMENT 8 // in pages
   
#define SAMPLES_FOR_MEAN 100

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

struct st_ref {
    int32 pos[NUM_OF_MOTORS];       // motor reference position
    uint8 onoff;                    // enable flags
};

//=============================================================     measurements

struct st_meas {
    int32 pos[NUM_OF_SENSORS];      // sensor position

    int32 curr[NUM_OF_MOTORS];      // motor currents

    int16 rot[NUM_OF_SENSORS];      // sensor rotations
};

//==============================================================     data packet

struct st_data {
    uint8   buffer[128];                    // CMD/DATA/CHECKSUM
    int16   length;                         // length
    int16   ind;                            // index
    uint8   ready;                          // Flag
};

//============================================     settings stored on the memory 

struct st_mem {
    uint8   flag;                       // Device has been configured 
    uint8   id;                         // device ID

    int32   k_p;                        // Proportional constant
    int32   k_i;                        // Derivative constant
    int32   k_d;                        // Integrative constant

    uint8   activ;                      // Activation upon startup
    uint8   input_mode;                 // Input mode
    uint8   control_mode;               // Control mode

    uint8   res[NUM_OF_SENSORS];        // Angle resolution
    int32   m_off[NUM_OF_SENSORS];      // Measurement offset
    float   m_mult[NUM_OF_SENSORS];     // Measurement multiplier
    uint8   pos_lim_flag;               // Position limit active/inactive
    int32   pos_lim_inf[NUM_OF_MOTORS]; // Inferior position limit for motors
    int32   pos_lim_sup[NUM_OF_MOTORS]; // Superior position limit for motors

    int32   max_step_pos;               // Maximum number of step per cylce when
    int32   max_step_neg;               // using sensor 3 as input

    uint16  max_stiffness;              // Max stiffness value obtained in calibration
};

//=================================================     device related variables

struct st_dev{
    int32   tension;                // Power supply tension
    float   tension_conv_factor;    // Used to calculate input tension
    uint8   tension_valid;
    uint8   pwm_limit;
};


enum calibration_status {
    STOP        = 0,
    START       = 1,
    CONTINUE_1  = 2,
    CONTINUE_2  = 3,
    PAUSE_1     = 4,
    PAUSE_2     = 5
};

//====================================      external global variables definition



extern struct st_ref    g_ref;          // motor variables
extern struct st_meas   g_meas;         // measurements
extern struct st_data   g_rx;           // income data
extern struct st_mem    g_mem, c_mem;   // memory
extern struct st_dev    device;         //device related variables

extern uint16 timer_value;

extern uint8 calibration_flag;

// -----------------------------------------------------------------------------


#endif

//[] END OF FILE