// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file 		interruptions.c
*
* \brief 		Interruption functions are in this file.
* \date 		Feb 06, 2012
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>
#include "globals.h"


//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
// 
// - 0: 	Waits for beggining characters
// - 1:		Waits for ID;
// - 2:		Data length;
// - 3:		Receive all bytes;
// - 4:		Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt){
	 
//===============================================     local variables definition

	static uint8 	state = 0;							// state
	static struct 	st_data data_packet;				// local data packet
	static uint8 	rx_queue[3]; 						// last 3 bytes received
                                                      
	static uint8 	rx_data;   	    				 	// RS485 UART rx data
	static uint8 	rx_data_type;					 	// my id?
	
		
//==========================================================     receive routine

// get data while rx fifo is not empty
	while (UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_FIFO_NOTEMPTY) {
		rx_data = UART_RS485_GetChar();				
		switch (state){
///////////////////////////   wait for frame start   ///////////////////////////			
			case 0:	

				rx_queue[0] = rx_queue[1];
				rx_queue[1] = rx_queue[2];
				rx_queue[2] = rx_data;					
						
				if((rx_queue[1] == ':') &&
				(rx_queue[2] == ':')){				
					rx_queue[0] = 0;
					rx_queue[1] = 0;
					rx_queue[2] = 0;
					state       = 1;
				}			
				else if(
				(rx_queue[0] == 63) &&		//ASCII - ?
				(rx_queue[1] == 13) &&		//ASCII - CR
				(rx_queue[2] == 10)){		//ASCII - LF
					infoSend();
				}				
				break;

///////////////////////////////   wait for id   ////////////////////////////////
			case  1:

				// packet is for my ID or is broadcast
				if(rx_data == c_mem.id || rx_data == 0) {
					rx_data_type = 0;
				} else {				//packet is for others
					rx_data_type = 1;
				}
				data_packet.length = -1;	
				state = 2;			
				break; 
			
//////////////////////////////   wait for length   /////////////////////////////
			case  2:
			
				data_packet.length = rx_data;				
				// check validity of pack length
				if(data_packet.length <= 1) {
					data_packet.length = -1;
					state = 0;			
				} else if(data_packet.length > 128) {
					data_packet.length = -1;
					state = 0;
				} else {
					data_packet.ind = 0;
					if(rx_data_type == 0) {
						state = 3;			// packet for me or boradcast
					} else {
						state = 4;			// packet for others
					}
				}
			break;
			
/////////////////////////////////   receving   /////////////////////////////////
			case 3:
			
			data_packet.buffer[data_packet.ind] = rx_data;
			data_packet.ind++;
			// check end of transmission				
			if(data_packet.ind >= data_packet.length){
				// verify if frame ID corresponded to the device ID
				if(rx_data_type == 0){
					// copying data from buffer to global packet
					memcpy(g_rx.buffer, data_packet.buffer, data_packet.length);
					g_rx.length = data_packet.length;
					g_rx.ready  = 1;	
					commProcess();
				}
				data_packet.ind    = 0;
				data_packet.length = -1;	
				state              = 0;				
			}
			break;

/////////////////////////   other device is receving    ////////////////////////
			case 4:
				if(!(--data_packet.length)) {
					data_packet.ind    = 0;
					data_packet.length = -1;
                    RS485_CTS_Write(1);
                    RS485_CTS_Write(0);    
					state              = 0;				
				}
			break;			
		}		
	}
}

//==============================================================================
//                                                      MOTORS CONTROL INTERRUPT
//==============================================================================
// Motors control
//==============================================================================

CY_ISR(ISR_MOTORS_CONTROL_ExInterrupt)
{	

	static int32 input_1 = 0;
	static int32 input_2 = 0;

	int32 error_1, error_2;
	static int32 sum_err_1, sum_err_2;
	
    /////////   use third encoder as input for both motors   //////////
    if( c_mem.mode == INPUT_MODE_ENCODER3 )
    {
        g_ref.pos[0] = g_meas.pos[2];
	    g_ref.pos[1] = g_meas.pos[2];
    }
	//////////////////////////////////////////////////////////////////
	
    #if (CONTROL_MODE == CONTROL_ANGLE)
		input_1 = 	(c_mem.k * (g_ref.pos[0] - g_meas.pos[0])) / 65536;
		input_2 = 	(c_mem.k * (g_ref.pos[1] - g_meas.pos[1])) / 65536;
    #endif

	#if (CONTROL_MODE == CONTROL_CURRENT)
		if(g_ref.onoff & 1)
		{
			error_1 = g_ref.pos[0] - g_meas.curr[0];
			error_2 = g_ref.pos[1] - g_meas.curr[1];

			sum_err_1 += error_1;
			sum_err_2 += error_2;

			input_1 += ((c_mem.k * (error_1)) / 65536) + sum_err_1;
			input_2 += ((c_mem.k * (error_2)) / 65536) + sum_err_2;
		} 
		else
		{
			input_1 = 0;
			input_2 = 0;
		} 
		
	#endif

	#if (CONTROL_MODE == CONTROL_PWM)
		input_1 = g_ref.pos[0];
		input_2 = g_ref.pos[1];
	#endif
		
		
    if(input_1 >  PWM_LIMIT) input_1 =  PWM_LIMIT;
    if(input_2 >  PWM_LIMIT) input_2 =  PWM_LIMIT;
    if(input_1 < -PWM_LIMIT) input_1 = -PWM_LIMIT;
    if(input_2 < -PWM_LIMIT) input_2 = -PWM_LIMIT;

	PWM_MOTOR_A_WriteCompare1(abs(input_1));
	PWM_MOTOR_A_WriteCompare2(abs(input_2));
	CONTROL_REG_MOTORS_Write((input_1 > 0) + ((input_2 > 0) << 1));
	
	/* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
	#if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
	    #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_MOTORS_CONTROL__ES2_PATCH ))      
	        ISR_MOTORS_CONTROL_ISR_PATCH();
	    #endif
	#endif
}

//==============================================================================
//                                                        MEASUREMENTS INTERRUPT
//==============================================================================
// TODO: DESCRIPTION
//==============================================================================

CY_ISR(ISR_MEASUREMENTS_ExInterrupt)
{
	static uint8 ind;
	int32 value;
	static int sign_1 = 1;
	static int sign_2 = 1;
	
	ADC_StartConvert();

	if (ADC_IsEndConversion(ADC_RETURN_STATUS)) {

		ind = AMUXSEQ_MOTORS_GetChannel();
		value = (int32) ADC_GetResult16();
		ADC_StopConvert();		
		switch(ind){
			case 0:
				device.tension = (24543*(value - 1648))/1648;
			break;
            case 1:
				g_meas.curr[0] =  (2430 * (value - 1648))/824;
				if(g_meas.curr[0] < 30)
					sign_1 = (CONTROL_REG_MOTORS_Read() & 0x01) ? 1 : -1;
				g_meas.curr[0] = g_meas.curr[0] * sign_1;
			break;			
            case 2:			
			g_meas.curr[1] = (2430 * (value - 1648))/824;
			if(g_meas.curr[1] < 30)
				sign_2 = (CONTROL_REG_MOTORS_Read() & 0x02)? 1 : -1;
			g_meas.curr[1] = g_meas.curr[1] * sign_2;
            break;
		}
		AMUXSEQ_MOTORS_Next();		
	}

	#if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
	    #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_MEASUREMENTS__ES2_PATCH ))      
	        ISR_MEASUREMENTS_ISR_PATCH();
	    #endif
	#endif
}

//==============================================================================
//                                                        	   ENCODER INTERRUPT
//==============================================================================
// TODO: DESCRIPTION
//==============================================================================


CY_ISR(ISR_ENCODER_ExInterrupt)
{
	int i;              //iterator

	int32 data_encoder[4];
	int32 value_encoder[4];
	int32 aux;

	static int32 last_value_encoder[4];
	

//==========================================================     reading sensors


	for (i = 0; i < NUM_OF_SENSORS; i++) {
		switch(i) {
			case 0: {
				data_encoder[i] = SHIFTREG_ENC_1_ReadData();
				break;
			}
			case 1: {
				data_encoder[i] = SHIFTREG_ENC_2_ReadData();
				break;
			}
			case 2: {
				data_encoder[i] = SHIFTREG_ENC_3_ReadData();
				break;
			}
			case 3: {
				data_encoder[i] = SHIFTREG_ENC_4_ReadData();
				break;
			}
		}
		aux = data_encoder[i] & 262142;
		if ((data_encoder[i] & 0x01 ) == BITChecksum(aux))
		{
			aux = data_encoder[i] & 0x3FFC0;			// reset last 6 bit
			value_encoder[i] = (aux - 0x20000) >> 2;	// subtract half of max value
													// and shift to have 16 bit val

			// take care of rotations
			aux = value_encoder[i] - last_value_encoder[i];					
			if (aux > 32768)
				g_meas.rot[i]--;
			if (aux < -32768)
			 	g_meas.rot[i]++;	

			last_value_encoder[i] = value_encoder[i];	
			
	        value_encoder[i] += g_meas.rot[i] * 65536;			                 
	        value_encoder[i] += g_mem.m_off[i];
	        value_encoder[i] *= c_mem.m_mult[i];
		}
		
	    g_meas.pos[i] = value_encoder[i];

	}


/* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
#if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
    #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_ENCODER__ES2_PATCH ))      
        ISR_ENCODER_ISR_PATCH();
    #endif
#endif

}

//==============================================================================
//																	BIT CHECKSUM
//==============================================================================


uint8 BITChecksum(uint32 mydata){
	uint8 i;
	uint8 checksum = 0;
	for(i = 0; i < 31; ++i)
	{
       	checksum = checksum ^ (mydata & 1);
		mydata = mydata >> 1;
	}
	return checksum;
}


/* [] END OF FILE */
