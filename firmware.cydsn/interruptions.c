 // ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file         interruptions.c
*
* \brief        Interruption functions are in this file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>
#include "globals.h"

//==================================================================     defines

#define TIMER_CLOCK 10000

//===================================================================     global

uint8 timer_flag = 0;

static const uint8 pwm_preload_values[36] = {100,   //0 (8000)
                                            76,
                                            71,
                                            69,
                                            67,
                                            65,     //5 (10500)
                                            63,
                                            61,
                                            60,
                                            58,
                                            57,     //10 (13000)
                                            56,
                                            55,
                                            54,
                                            53,
                                            52,     //15 (15500)
                                            51,
                                            50,
                                            49,
                                            49,
                                            48,     //20 (18000)
                                            47,
                                            47,
                                            46,
                                            45,
                                            45,     //25 (20500)
                                            45,
                                            44,
                                            44,
                                            43,
                                            43,     //30 (23000)
                                            43,
                                            43,
                                            42,
                                            42,
                                            42};    //35 (25500)

//=============================================================     decalrations

void pwm_limit_search();

void encoder_reading(void);
void motor_control(void);


//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
// 
// - 0:     Waits for beggining characters
// - 1:     Waits for ID;
// - 2:     Data length;
// - 3:     Receive all bytes;
// - 4:     Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt){
     
//===============================================     local variables definition

    static uint8    state = 0;                          // state
    static struct   st_data data_packet;                // local data packet
    static uint8    rx_queue[3];                        // last 3 bytes received
                                                      
    static uint8    rx_data;                            // RS485 UART rx data
    static uint8    rx_data_type;                       // my id?
    
        
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
                (rx_queue[0] == 63) &&      //ASCII - ?
                (rx_queue[1] == 13) &&      //ASCII - CR
                (rx_queue[2] == 10)){       //ASCII - LF
                    infoSend();
                }               
                break;

///////////////////////////////   wait for id   ////////////////////////////////
            case  1:

                // packet is for my ID or is broadcast
                if(rx_data == c_mem.id || rx_data == 0) {
                    rx_data_type = 0;
                } else {                //packet is for others
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
                        state = 3;          // packet for me or boradcast
                    } else {
                        state = 4;          // packet for others
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
//                                                                MOTORS CONTROL
//==============================================================================
// Motors control
//==============================================================================

void motor_control(void)
{
    static int32 input_1 = 0;
    static int32 input_2 = 0;


    static int32 pos_prec_1, pos_prec_2;
    int32 error_1, error_2;
    static int32 err_sum_1, err_sum_2;


    /////////   use third encoder as input for both motors   //////////
    if( c_mem.input_mode == INPUT_MODE_ENCODER3 )
    {
        //--- speed control in both directions ---//

        // motor 1
        if (((g_meas.pos[2] - g_ref.pos[0]) > c_mem.max_step_pos)   &&   (c_mem.max_step_pos != 0)) {
            g_ref.pos[0] += c_mem.max_step_pos;
        } else if (((g_meas.pos[2] - g_ref.pos[0]) < c_mem.max_step_neg)   &&   (c_mem.max_step_neg != 0)) {
            g_ref.pos[0] += c_mem.max_step_neg;
        } else {
            g_ref.pos[0] = g_meas.pos[2];
        }

        // motor 2
        if (((g_meas.pos[2] - g_ref.pos[1]) > c_mem.max_step_pos)   &&   (c_mem.max_step_pos != 0)) {
            g_ref.pos[1] += c_mem.max_step_pos;
        } else if (((g_meas.pos[2] - g_ref.pos[1]) < c_mem.max_step_neg)   &&   (c_mem.max_step_neg != 0)) {
            g_ref.pos[1] += c_mem.max_step_neg;
        } else {
            g_ref.pos[1] = g_meas.pos[2];
        }

        // position limit
        if (c_mem.pos_lim_flag) {
            if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
            if (g_ref.pos[1] < c_mem.pos_lim_inf[1]) g_ref.pos[1] = c_mem.pos_lim_inf[1];

            if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
            if (g_ref.pos[1] > c_mem.pos_lim_sup[1]) g_ref.pos[1] = c_mem.pos_lim_sup[1];
        }
    }

    //////////////////////////////////////////////////////////     CONTROL_ANGLE

    if (c_mem.control_mode == CONTROL_ANGLE) {

        error_1 = g_ref.pos[0] - g_meas.pos[0];
        error_2 = g_ref.pos[1] - g_meas.pos[1];

        err_sum_1 += error_1;
        err_sum_2 += error_2;

        // anti-windup
        if (err_sum_1 > ANTI_WINDUP) {
            err_sum_1 = ANTI_WINDUP;
        } else if (err_sum_1 < -ANTI_WINDUP) {
            err_sum_1 = -ANTI_WINDUP;
        }

        if (err_sum_2 > ANTI_WINDUP) {
            err_sum_2 = ANTI_WINDUP;
        } else if (err_sum_2 < -ANTI_WINDUP) {
            err_sum_2 = -ANTI_WINDUP;
        }

        // Proportional
        if (c_mem.k_p != 0) {
            input_1 = (int32)(c_mem.k_p * error_1) >> 16;
            input_2 = (int32)(c_mem.k_p * error_2) >> 16;
        }

        // Integrative
        if (c_mem.k_i != 0) {
            input_1 += (int32)(c_mem.k_i * err_sum_1) >> 16;
            input_2 += (int32)(c_mem.k_i * err_sum_2) >> 16;
        }

        // Derivative
        if (c_mem.k_d != 0) {
            input_1 += (int32)(c_mem.k_d * (pos_prec_1 - g_meas.pos[0])) >> 16;
            input_2 += (int32)(c_mem.k_d * (pos_prec_2 - g_meas.pos[1])) >> 16;
        }

        // Update measure
        pos_prec_1 = g_meas.pos[0];
        pos_prec_2 = g_meas.pos[1];
    }

    ////////////////////////////////////////////////////////     CONTROL_CURRENT

    if (c_mem.control_mode == CONTROL_CURRENT) {

        if(g_ref.onoff & 1)
        {
            error_1 = g_ref.pos[0] - g_meas.curr[0];
            error_2 = g_ref.pos[1] - g_meas.curr[1];

            err_sum_1 += error_1;
            err_sum_2 += error_2;

            input_1 += ((c_mem.k_p * (error_1)) / 65536) + err_sum_1;
            input_2 += ((c_mem.k_p * (error_2)) / 65536) + err_sum_2;
        } 
        else
        {
            input_1 = 0;
            input_2 = 0;
        }
    }

    ////////////////////////////////////////////////////////////     CONTROL_PWM

    if (c_mem.control_mode == CONTROL_PWM) {

        input_1 = g_ref.pos[0];
        input_2 = g_ref.pos[1];
    }
    

    #if PWM_DEAD != 0

        if (input_1 > 0) {
            input_1 += PWM_DEAD;
        } else if (input_1 < 0) {
            input_1 -= PWM_DEAD;
        }

    #endif

        
    if(input_1 >  PWM_MAX_VALUE) input_1 =  PWM_MAX_VALUE;
    if(input_2 >  PWM_MAX_VALUE) input_2 =  PWM_MAX_VALUE;
    if(input_1 < -PWM_MAX_VALUE) input_1 = -PWM_MAX_VALUE;
    if(input_2 < -PWM_MAX_VALUE) input_2 = -PWM_MAX_VALUE;

    //MOTOR_DIR_Write((input_1 >= 0) + ((input_2 >= 0) << 1));
    MOTOR_DIR_Write((input_1 < 0) + ((input_2 < 0) << 1));

    if (c_mem.control_mode != CONTROL_PWM) {
        input_1 = (((input_1 * 1024) / PWM_MAX_VALUE) * device.pwm_limit) / 1024;
        input_2 = (((input_2 * 1024) / PWM_MAX_VALUE) * device.pwm_limit) / 1024;
    }

    PWM_MOTORS_WriteCompare1(abs(input_1));
    PWM_MOTORS_WriteCompare2(abs(input_2));

}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================
// TODO: DESCRIPTION
//==============================================================================

void measurements_int(void)
{
    static uint8 ind;
    int32 value;
    static int sign_1 = 1;
    static int sign_2 = 1;
    static uint16 counter = SAMPLES_FOR_MEAN; // Used to perform calibration over
                                // the first counter values of current
    static int32 mean_value_1;
    static int32 mean_value_2;

    static uint8 counter_encoder_read = 1;
    static uint8 counter_motor_control = 1;

    static uint16 timer_counter = 1;

    if (timer_counter < 3000) {
        timer_counter++;
    } else if (timer_counter == 3000) {
        timer_value = (uint16)MY_TIMER_ReadCounter();
        MY_TIMER_WriteCounter(65535);
        timer_counter = 1;
    }


    if (counter_encoder_read == 3) {
        encoder_reading();
        counter_encoder_read = 0;
    }

    if (counter_motor_control == 6) {
        motor_control();
        counter_motor_control = 0;
    }

    counter_motor_control++;
    counter_encoder_read++;


    ADC_StartConvert();

    if (ADC_IsEndConversion(ADC_RETURN_STATUS)) {

        ind = AMUXSEQ_MOTORS_GetChannel();
        value = (int32) ADC_GetResult16();
        ADC_StopConvert();      
        switch(ind){
            // --- Input tension ---
            case 0:
                device.tension = (value - 1638) * device.tension_conv_factor;
                if (device.tension < 0) { //until there is no valid input tension repeat this measurement
                    AMUXSEQ_MOTORS_Stop();
                    AMUXSEQ_MOTORS_Next();
                    counter = SAMPLES_FOR_MEAN;
                    mean_value_1 = 0;
                    mean_value_2 = 0;
                    device.tension_valid = FALSE;
                } else {
                    device.tension_valid = TRUE;
                    pwm_limit_search();
                }
                break;

            // --- Current motor 1 ---
            case 1:
                if (counter > 0) {
                    mean_value_1 += value;
                    if (counter == 1) {
                        mean_value_1 = mean_value_1 / SAMPLES_FOR_MEAN;
                    }
                } else {
                    g_meas.curr[0] =  ((value - mean_value_1) * 5000) / mean_value_1;
                    if(g_meas.curr[0] < 60)
                        sign_1 = (MOTOR_DIR_Read() & 0x01) ? 1 : -1;
                    g_meas.curr[0] = g_meas.curr[0] * sign_1;
                }
                break;

            // --- Current motor 2 ---
            case 2:
                if (counter > 0) {
                    mean_value_2 += value;
                    if (counter == 1) {
                        mean_value_2 = mean_value_2 / SAMPLES_FOR_MEAN;
                    }
                    counter--;
                } else {    
                    g_meas.curr[1] =  ((value - mean_value_2) * 5000) / mean_value_2;
                    if(g_meas.curr[1] < 60)
                        sign_2 = (MOTOR_DIR_Read() & 0x02) ? 1 : -1;
                    g_meas.curr[1] = g_meas.curr[1] * sign_2;
                }
                break;
        }
        AMUXSEQ_MOTORS_Next();      
    }
}

//==============================================================================
//                                                               ENCODER READING
//==============================================================================
// TODO: DESCRIPTION
//==============================================================================

void encoder_reading(void)
{
    int i;              //iterator

    int32 data_encoder[NUM_OF_SENSORS];
    int32 value_encoder[NUM_OF_SENSORS];
    int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS];
    static int8 only_first_time = 1;


// Discard first reading
    if (only_first_time) {
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
        }
        only_first_time = 0;
    }

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
            aux = data_encoder[i] & 0x3FFC0;            // reset last 6 bit
            value_encoder[i] = (aux - 0x20000) >> 2;    // subtract half of max value
                                                    // and shift to have 16 bit val

            value_encoder[i] = -value_encoder[i];   //invert sign of sensor

            value_encoder[i]  = (int16)(value_encoder[i] + g_mem.m_off[i]);

            // take care of rotations
            aux = value_encoder[i] - last_value_encoder[i];                 
            if (aux > 32768)
                g_meas.rot[i]--;
            if (aux < -32768)
                g_meas.rot[i]++;    

            last_value_encoder[i] = value_encoder[i];   
            
            value_encoder[i] += g_meas.rot[i] * 65536;                           
            //value_encoder[i] += g_mem.m_off[i];
            value_encoder[i] *= c_mem.m_mult[i];
        }
        
        g_meas.pos[i] = value_encoder[i];
    }
}


//==============================================================================
//                                                           CALIBRATE INTERRUPT
//==============================================================================
// TODO: DESCRIPTION
//==============================================================================


CY_ISR(ISR_CALIBRATE_ExInterrupt)
{
    int i = 0;
    int16 mean_curr_1, mean_curr_2;

    // save old PID values
    int32 old_k_p = c_mem.k_p;
    int32 old_k_i = c_mem.k_i;
    int32 old_k_d = c_mem.k_d;

    // goto to zero position
    g_ref.pos[0] = 0;
    g_ref.pos[1] = 0;

    // // Activate motors
    // if (!(g_ref.onoff & 0x03)) {
    //     MOTOR_ON_OFF_Write(0x03);   
    // }
    
    // wait for motors to reach zero position
    CyDelay(1000);


    // XXXXXXXXX
    g_ref.pos[0] = 5000;
    g_ref.pos[1] = 5000;
    // XXXXXXXXXX

    // // set new temp values for PID parameters
    // c_mem.k_p = 0.1 * 65536;
    // c_mem.k_i = 0;
    // c_mem.k_d = 0.3 * 65536;

    // // increase stiffness until one of the two motors reach the threshold
    // while((mean_curr_1 < MAX_CURRENT) && (abs(mean_curr_2) < MAX_CURRENT)) {
    //     // increment of 0.5 degree
    //     g_ref.pos[0] += 65536 / 720;
    //     g_ref.pos[1] -= 65536 / 720;

    //     CyDelay(100);

    //     // Current measurement
    //     mean_curr_1 = 0;
    //     mean_curr_2 = 0;
    //     for (i = 0; i < 10; i++) {
    //         mean_curr_1 += g_meas.curr[0];
    //         mean_curr_2 += g_meas.curr[1];
    //         CyDelay(10);
    //     }
    //     mean_curr_1 = mean_curr_1 / 10;
    //     mean_curr_2 = mean_curr_2 / 10;
    // }

    // // save current value as MAX_STIFFNESS
    // g_mem.max_stiffness = g_ref.pos[0];

    // // reset old values for PID parameters
    // c_mem.k_p = old_k_p;
    // c_mem.k_i = old_k_i;
    // c_mem.k_d = old_k_d;

    // // go back to zero position
    // g_ref.pos[0] = 0;
    // g_ref.pos[1] = 0;

    // // wait for motors to reach zero position
    // CyDelay(3000);

    // //Deactivate motors
    // // if (!(g_ref.onoff & 0x03)) {
    // //  MOTOR_ON_OFF_Write(0x00);   
    // // }

    // // store memory to save MAX_STIFFNESS as default value
    // memStore(DEFAULT_EEPROM_DISPLACEMENT);
    // memStore(0);

/* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
#if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
    #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_CALIBRATE__ES2_PATCH ))      
        ISR_CALIBRATE_ISR_PATCH();
    #endif
#endif

}

//==============================================================================
//                                                                  BIT CHECKSUM
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

//==============================================================================
//                                                              PWM_LIMIT_SEARCH
//==============================================================================

void pwm_limit_search() {
    uint8 index;

    if (device.tension > 25500) {
        device.pwm_limit = 0;
    } else if (device.tension < 8000) {
        device.pwm_limit = 100;
    } else {
        index = (uint8)((device.tension - 8000) / 500);
        device.pwm_limit = pwm_preload_values[index];
    }
}


/* [] END OF FILE */
