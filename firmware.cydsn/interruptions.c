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
#include "utils.h"

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
//                                                            FUNCTION SCHEDULER
//==============================================================================
// Call all the function with the right frequency
//==============================================================================

void function_scheduler(void) {
    // Base frequency 1000 Hz

    static uint8 counter_analog_measurements = DIV_INIT_VALUE;
    static uint8 counter_encoder_read        = DIV_INIT_VALUE;
    static uint8 counter_motor_control       = DIV_INIT_VALUE;
    static uint16 counter_calibration        = DIV_INIT_VALUE;

    static uint16 timer_counter = 1;

    // Divider 1, freq = 1000 Hz
    if (counter_analog_measurements == ANALOG_MEASUREMENTS_DIV) {
        analog_measurements();
        counter_analog_measurements = 0;
    }
    counter_analog_measurements++;

    // Divider 1, freq = 1000 Hz
    if (counter_encoder_read == ENCODER_READ_DIV) {
        encoder_reading();
        counter_encoder_read = 0;
    }
    counter_encoder_read++;

    // Divider 1, freq = 1000 Hz
    if (counter_motor_control == MOTOR_CONTROL_DIV) {
        motor_control();
        counter_motor_control = 0;
    }
    counter_motor_control++;

    // Divider 100, freq = 10 Hz
    if (calibration_flag != STOP) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }


    // User MY_TIMER to store the execution time of 3000 executions
    // to check the base frequency
    // if (timer_counter < 3000) {
    //     timer_counter++;
    // } else if (timer_counter == 3000) {
        timer_value = (uint32)MY_TIMER_ReadCounter();
        MY_TIMER_WriteCounter(5000000);
    //     timer_counter = 1;
    // }
}


//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control(void)
{
    static int32 input_1 = 0;
    static int32 input_2 = 0;


    static int32 pos_prec_1, pos_prec_2;
    static int32 error_1, error_2;
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
            if ((error_1 > 131072) || (error_1 < -131072)) {  //if grater than 2^17
                input_1 = (int32)(c_mem.k_p * (error_1 >> 8)) >> 8;
            } else {
                input_1 = (int32)(c_mem.k_p * error_1) >> 16;
            }

            if ((error_2 > 131072) || (error_2 < -131072)) {  //if grater than 2^17
                input_2 = (int32)(c_mem.k_p * (error_2 >> 8)) >> 8;
            } else {
                input_2 = (int32)(c_mem.k_p * error_2) >> 16;
            }
            // input_1 = (int32)(c_mem.k_p * error_1) >> 16;
            // input_2 = (int32)(c_mem.k_p * error_2) >> 16;
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

        input_1 = g_ref.pos[0] >> g_mem.res[0];
        input_2 = g_ref.pos[1] >> g_mem.res[1];
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

    MOTOR_DIR_Write((input_1 >= 0) + ((input_2 >= 0) << 1));
    //MOTOR_DIR_Write((input_1 < 0) + ((input_2 < 0) << 1));

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

void analog_measurements(void)
{
    static uint8 i;
    static int32 value;

    static uint16 counter = SAMPLES_FOR_MEAN; // Used to perform calibration over
                                // the first counter values of current
    static int32 mean_value_1;
    static int32 mean_value_2;


    for (i = 0; i < NUM_OF_ANALOG_INPUTS; i++) {

        AMUX_FastSelect(i);
        ADC_StartConvert();

        if (ADC_IsEndConversion(ADC_WAIT_FOR_RESULT)) {

            value = (int32) ADC_GetResult16();
            ADC_StopConvert();

            switch(i) {

                // --- Input tension ---
                case 0:
                    device.tension = (value - 1638) * device.tension_conv_factor;
                    //until there is no valid input tension repeat this measurement
                    if (device.tension < 0) {
                        i = NUM_OF_ANALOG_INPUTS;
                        device.tension_valid = FALSE;
                    } else {
                        device.tension_valid = TRUE;
                        pwm_limit_search();
                    }
                    break;

                // --- Current motor 1 ---
                case 1:
                    g_meas.curr[0] =  filter_i1(abs(((value - 1638) * 4000) / (1638)));
                    break;

                // --- Current motor 2 ---
                case 2:
                    g_meas.curr[1] =  filter_i2(abs(((value - 1638) * 4000) / (1638)));
                    break;
            }
        }
    }
}

//==============================================================================
//                                                               ENCODER READING
//==============================================================================

void encoder_reading(void)
{
    static uint8 i;              //iterator

    static int32 data_encoder;
    static int32 value_encoder;
    static int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS];

    static int32 l_value[NUM_OF_SENSORS];   //last value for vel
    static int32 ll_value[NUM_OF_SENSORS];  //last last value for vel
    static int32 lll_value[NUM_OF_SENSORS];  //last last last value for vel
    static int8 only_first_time = 1;


// Discard first reading
    if (only_first_time) {
        for (i = 0; i < NUM_OF_SENSORS; i++) {
            last_value_encoder[i] = 0;

            switch(i) {
                case 0: {
                    data_encoder = SHIFTREG_ENC_1_ReadData();
                    break;
                }
                case 1: {
                    data_encoder = SHIFTREG_ENC_2_ReadData();
                    break;
                }
                case 2: {
                    data_encoder = SHIFTREG_ENC_3_ReadData();
                    break;
                }
                case 3: {
                    data_encoder = SHIFTREG_ENC_4_ReadData();
                    break;
                }
            }
        }
        only_first_time = 0;
        CyDelay(1); //Wait to be sure the shift register is updated with a new valid measure
    }

//==========================================================     reading sensors
    for (i = 0; i < NUM_OF_SENSORS; i++) {
        switch(i) {
            case 0: {
                data_encoder = SHIFTREG_ENC_1_ReadData();
                break;
            }
            case 1: {
                data_encoder = SHIFTREG_ENC_2_ReadData();
                break;
            }
            case 2: {
                data_encoder = SHIFTREG_ENC_3_ReadData();
                break;
            }
            case 3: {
                data_encoder = SHIFTREG_ENC_4_ReadData();
                break;
            }
        }


        if (check_enc_data(&data_encoder)) {

            aux = data_encoder & 0x3FFC0;               // reset last 6 bit
            value_encoder = 32768 - (aux >> 2);         // shift to have 16 bit val and
                                                        // subtract half of max value and
                                                        // invert sign of sensor

            value_encoder  = (int16)(value_encoder + g_mem.m_off[i]);

            // take care of rotations
            aux = value_encoder - last_value_encoder[i];
            if (aux > 32768)
                g_meas.rot[i]--;
            if (aux < -32768)
                g_meas.rot[i]++;

            last_value_encoder[i] = value_encoder;

            value_encoder += g_meas.rot[i] * 65536;

            value_encoder *= c_mem.m_mult[i];

            g_meas.pos[i] = value_encoder;
        } else {
            g_meas.pos[i] = last_value_encoder[i];
        }

        // // velocity calculation
        // switch(i) {
        //     case 0: {
        //         g_meas.vel[i] = (int16)filter_vel_1((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
        //         break;
        //     }
        //     case 1: {
        //         g_meas.vel[i] = (int16)filter_vel_2((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
        //         break;
        //     }
        //     case 2: {
        //         g_meas.vel[i] = (int16)filter_vel_3((3*value_encoder + l_value[i] - ll_value[i] - 3*lll_value[i]) / 10);
        //         break;
        //     }
        // }

        // // update old values
        // lll_value[i] = ll_value[i];
        // ll_value[i] = l_value[i];
        // l_value[i] = value_encoder;
    }
}


//==============================================================================
//                                                          CALIBRATION FUNCTION
//==============================================================================

void calibration()
{
    static int32 old_k_p;
    static int32 old_k_i;
    static int32 old_k_d;

    static uint8 pause_counter = 0;

    switch(calibration_flag) {
        case START:
        {
            ISR_RS485_RX_Disable();

            // save old PID values
            old_k_p = c_mem.k_p;
            old_k_i = c_mem.k_i;
            old_k_d = c_mem.k_d;

            // goto to zero position
            g_ref.pos[0] = 0;
            g_ref.pos[1] = 0;

            // Activate motors
            if (!(g_ref.onoff & 0x03)) {
                MOTOR_ON_OFF_Write(0x03);
            }

            // wait for motors to reach zero position
            calibration_flag = PAUSE_1;
            break;
        }

        case PAUSE_1:
            pause_counter++;

            if (pause_counter == 10) {
                pause_counter = 0;

                // set new temp values for PID parameters
                c_mem.k_p = 0.1 * 65536;
                c_mem.k_i = 0;
                c_mem.k_d = 0.3 * 65536;

                calibration_flag = CONTINUE_1;
            }
            break;

        case CONTINUE_1:
            // increment of 0.5 degree
            g_ref.pos[0] += 65536 / 720;
            g_ref.pos[1] -= 65536 / 720;

            // check if one of the motors reach the threashold
            if ((g_meas.curr[0] > MAX_CURRENT) || (g_meas.curr[1] > MAX_CURRENT)) {
                // save current value as MAX_STIFFNESS
                g_mem.max_stiffness = g_ref.pos[0];

                // reset old values for PID parameters
                c_mem.k_p = old_k_p;
                c_mem.k_i = old_k_i;
                c_mem.k_d = old_k_d;

                // go back to zero position
                g_ref.pos[0] = 0;
                g_ref.pos[1] = 0;

                // wait for motors to reach zero position
                calibration_flag = PAUSE_2;
            }
            break;

        case PAUSE_2:
            pause_counter++;

            if (pause_counter == 10) {
                pause_counter =0;

                calibration_flag = CONTINUE_2;
            }
            break;

        case CONTINUE_2:
            // Deactivate motors
            if (!(g_ref.onoff & 0x03)) {
                MOTOR_ON_OFF_Write(0x00);
            }

            // store memory to save MAX_STIFFNESS as default value
            memStore(DEFAULT_EEPROM_DISPLACEMENT);
            memStore(0);

            calibration_flag = STOP;

            ISR_RS485_RX_Enable();
            break;

        case STOP:
        default:
            break;
    }
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
