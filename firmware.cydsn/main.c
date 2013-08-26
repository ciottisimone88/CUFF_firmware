// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/** 
* \file 		main.c
*
* \brief 		Firmware main file.
* \date 		May 16, 2012
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/

/**
* \mainpage		Firmware
* \brief		This is the firmware of the qb move.
* \version		0.1 beta 4
*
* \author		_qbrobotics_
*
* \date 		May 16, 2012
*
* \details		This is the firmware of the qb move.
*
* \copyright	(C)  qbrobotics. All rights reserved.
*
*/


// ----------------------------------------------------------------------------
// This version changes:
//		- not reported


//=================================================================     includes

#include <device.h>
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include <interruptions.h>
#include <command_processing.h>	

//==============================================================================
//                                                                 MAIN FUNCTION
//==============================================================================

int i;			//iterator


void main()
{
//====================================     initializations - psoc and components
		
	CYGlobalIntEnable; 									// enable interrupts
	
	// EEPROM
	
	EEPROM_Start();
	memRecall();                                     	// recall configuration

	// RS485
	
	UART_RS485_Start();									// start UART
	ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);		// RS485 isr function    

	// PWM
    
	PWM_MOTOR_A_Start();
	ISR_MOTORS_CONTROL_StartEx(ISR_MOTORS_CONTROL_ExInterrupt);
	PWM_MOTOR_A_WriteCompare1(0);
	PWM_MOTOR_A_WriteCompare2(0);
	CONTROL_REG_MOTORS_Write(0);	
	
	// SSI encoder initializations
	
	COUNTER_ENC_Start();
	SHIFTREG_ENC_1_Start();
	SHIFTREG_ENC_2_Start();
	SHIFTREG_ENC_3_Start();
	SHIFTREG_ENC_4_Start();
	ISR_ENCODER_StartEx(ISR_ENCODER_ExInterrupt);
		
	// ADC
	
    ADC_Start();                                     // start ADC
    ADC_StartConvert();
	// measurements isr function
    ISR_MEASUREMENTS_StartEx(ISR_MEASUREMENTS_ExInterrupt);
    AMUXSEQ_MOTORS_Start();                              // start mux
	
    RS485_CTS_Write(0);           

//========================================     initializations - clean variables


    for (i = 0; i < NUM_OF_MOTORS; i++) {
    	g_ref.pos[i] = 0;	
    }
	g_ref.onoff 		= c_mem.activ;

	for (i = 0; i < NUM_OF_SENSORS; i++) {
		g_meas.pos[i] = 0;
		g_meas.rot[i] = 0;	
	}

	g_rx.length			= 0;
	g_rx.ready			= 0;

	strcpy(g_rx.buffer,"");
	
	// Activating motors
	
    g_ref.onoff = g_rx.buffer[1];
    if (g_ref.onoff & 0x01) {				// motor 2
 		g_ref.pos[1] = g_meas.pos[1]; 				        
    	CyPins_SetPin(MOTOR_EN_B);
	} else {
 		CyPins_ClearPin(MOTOR_EN_B);
	}

    if (g_ref.onoff & 0x02) {				// motor 1
 		g_ref.pos[0] = g_meas.pos[0];         
    	CyPins_SetPin(MOTOR_EN_A);    	
	} else {
 		CyPins_ClearPin(MOTOR_EN_A);
	}

	


//=========================================================     application loop

	for(;;)
	{
	    if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER)
            UART_RS485_ClearRxBuffer();
	}
}



/* [] END OF FILE */