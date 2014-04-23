// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/** 
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include "commands.h"
#include <stdio.h>

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;
 
//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(void){
    int i;              //iterator
    uint8 rx_cmd;
    uint8 aux_checksum; 
    uint8 packet_data[16];
    uint8 packet_lenght;
    int32 pos_1, pos_2;
    int32  pos, stiff;
    uint32 off_1, off_2;
    uint32 mult_1, mult_2;
        
    rx_cmd = g_rx.buffer[0];
        
//==========================================================     verify checksum
    aux_checksum = LCRChecksum(g_rx.buffer,
        g_rx.length - 1);
    if (!(aux_checksum ==
    g_rx.buffer[g_rx.length-1])){
        // wrong checksum
        g_rx.ready = 0;
        return; 
    }   


    switch(rx_cmd){
//=============================================================     CMD_ACTIVATE        
        case CMD_ACTIVATE:
            g_ref.onoff = g_rx.buffer[1];

            if (g_mem.control_mode == CONTROL_ANGLE) {
                g_ref.pos[0] = g_meas.pos[0];
                g_ref.pos[1] = g_meas.pos[1];
            }

            MOTOR_ON_OFF_Write(g_ref.onoff);

            break;
//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            g_ref.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1      
            g_ref.pos[0] = g_ref.pos[0] << g_mem.res[0];

            g_ref.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
            g_ref.pos[1] = g_ref.pos[1] << g_mem.res[1];

            if (c_mem.pos_lim_flag) {                      // pos limiting
                if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
                if (g_ref.pos[1] < c_mem.pos_lim_inf[1]) g_ref.pos[1] = c_mem.pos_lim_inf[1];

                if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
                if (g_ref.pos[1] > c_mem.pos_lim_sup[1]) g_ref.pos[1] = c_mem.pos_lim_sup[1];
            }

            break;

//========================================================     CMD_SET_POS_STIFF

        case CMD_SET_POS_STIFF:
            pos = *((int16 *) &g_rx.buffer[1]);   // eq position
            stiff = *((int16 *) &g_rx.buffer[3]);   // stiffness

            // position in ticks
            pos = pos << g_mem.res[0];

            // position limit
            if (pos > (c_mem.pos_lim_sup[0] - c_mem.max_stiffness))
                pos = c_mem.pos_lim_sup[0] - c_mem.max_stiffness;

            if (pos < (c_mem.pos_lim_inf[0] + c_mem.max_stiffness))
                pos = c_mem.pos_lim_inf[0] + c_mem.max_stiffness;

            // stiffness is intended between -32768 and 32767
            // remap  stiff value between -MAX_STIFFNESS and MAX_STIFFNESS
            stiff = (int32)(((float)c_mem.max_stiffness / 32768.0) * stiff);

            // pos stiff rule
            g_ref.pos[0] = pos + stiff;
            g_ref.pos[1] = pos - stiff;

            break;

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            // Packet: header + measure(int16) + crc
            packet_lenght = 1 + (NUM_OF_SENSORS * 2) + 1;

            packet_data[0] = CMD_GET_MEASUREMENTS;   //header

            for (i = 0; i < NUM_OF_SENSORS; i++) {
                *((int16 *) &packet_data[(i*2) + 1]) = (int16)
                (g_meas.pos[i] >> g_mem.res[i]);
            }

            packet_data[packet_lenght - 1] = 
                    LCRChecksum (packet_data,packet_lenght - 1);

            commWrite(packet_data, packet_lenght);

        break;

//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            //Packt: header + measure(int16) + CRC
            packet_lenght = 6;
            
            packet_data[0] = CMD_GET_CURRENTS;              
            
            *((int16 *) &packet_data[1]) = (int16) g_meas.curr[0];
            *((int16 *) &packet_data[3]) = (int16) g_meas.curr[1];
            
            packet_data[5] = LCRChecksum (packet_data,packet_lenght - 1);

            commWrite(packet_data, packet_lenght);
        break;

//====================================================     CMD_GET_CURR_AND_MEAS

        case CMD_GET_CURR_AND_MEAS:
            //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC
            //packet_lenght = 1 + 2 * 2 + (NUM_OF_SENSORS * 2) + 1;
            packet_lenght = 6 + (NUM_OF_SENSORS * 2);

            packet_data[0] = CMD_GET_CURR_AND_MEAS;

            // Currents
            *((int16 *) &packet_data[1]) = (int16) g_meas.curr[0];
            *((int16 *) &packet_data[3]) = (int16) g_meas.curr[1];

            // Positions
            for (i = 0; i < NUM_OF_SENSORS; i++) {
                *((int16 *) &packet_data[(i*2) + 5]) = (int16)
                (g_meas.pos[i] >> g_mem.res[i]);
            }

            packet_data[packet_lenght - 1] =
                LCRChecksum (packet_data,packet_lenght - 1);

            //commWrite(packet_data, packet_lenght);
            commWrite(packet_data, packet_lenght);
        break;

//=========================================================     CMD_GET_ACTIVATE
        
        case CMD_GET_ACTIVATE:
            packet_lenght = 3;                                                     

            packet_data[0] = CMD_GET_ACTIVATE;
            packet_data[1] = g_ref.onoff;
            packet_data[2] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);

            break;
        
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            packet_lenght = 6;

            pos_1 = g_ref.pos[0]  >> g_mem.res[0];
            pos_2 = g_ref.pos[1]  >> g_mem.res[1];

            *((int16 *) &packet_data[1]) = (int16) (pos_1);
            *((int16 *) &packet_data[3]) = (int16) (pos_2);
            packet_data[5] = LCRChecksum(packet_data,packet_lenght - 1);

            commWrite(packet_data, packet_lenght);
            break;

//=============================================================     CMD_GET_INFO
        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]),
                g_rx.buffer[3]);
            break;

//============================================================     CMD_SET_PARAM
        case CMD_SET_PARAM:
            paramSet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//============================================================     CMD_GET_PARAM
        case CMD_GET_PARAM:        
            paramGet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//=================================================================     CMD_PING
        case CMD_PING:
            packet_lenght = 2;
            
            packet_data[0] = CMD_PING;
            packet_data[1] = CMD_PING;
            
            commWrite(packet_data, packet_lenght);
            break;

//=========================================================     CMD_STORE_PARAMS
        case CMD_STORE_PARAMS:
            if( c_mem.input_mode == INPUT_MODE_EXTERNAL )
            {
                off_1 = c_mem.m_off[0];
                off_2 = c_mem.m_off[1];
                mult_1 = c_mem.m_mult[0];
                mult_2 = c_mem.m_mult[1];
 
                g_ref.pos[0] /= mult_1;
                g_ref.pos[1] /= mult_2;
                g_ref.pos[0] *= g_mem.m_mult[0];
                g_ref.pos[1] *= g_mem.m_mult[1];
    
                g_ref.pos[0] +=  g_mem.m_off[0] - off_1;
                g_ref.pos[1] +=  g_mem.m_off[1] - off_2;

                if (c_mem.pos_lim_flag) {                   // position limiting
                    if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
                    if (g_ref.pos[1] < c_mem.pos_lim_inf[1]) g_ref.pos[1] = c_mem.pos_lim_inf[1];

                    if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
                    if (g_ref.pos[1] > c_mem.pos_lim_sup[1]) g_ref.pos[1] = c_mem.pos_lim_sup[1];
                }
            }

            memStore(0);
            sendAcknowledgment();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS
        case CMD_STORE_DEFAULT_PARAMS:
            memStore(DEFAULT_EEPROM_DISPLACEMENT);
            sendAcknowledgment();
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            memRestore();
            sendAcknowledgment();
            break;

//=============================================================     CMD_INIT_MEM        

        case CMD_INIT_MEM:
            memInit();
            sendAcknowledgment();
            break;
            
//===========================================================     CMD_BOOTLOADER
        case CMD_BOOTLOADER:
            sendAcknowledgment();
            Bootloadable_Load();
            break;

//============================================================     CMD_CALIBRATE
        case CMD_CALIBRATE:
            CALIB_TRIGG_Write(1);
            sendAcknowledgment();
            CALIB_TRIGG_Write(0);
            break;
    }

    g_rx.ready = 0; 
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================

void infoSend(void){
    unsigned char packet_string[1100];    
    infoPrepare(packet_string);
    UART_RS485_PutString(packet_string);
}

//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type, uint8 page){
    unsigned char packet_lenght;
    unsigned char packet_data[1300];
    static unsigned char packet_string[1100];    
    uint8 pages;
    uint32 aux_int;    
    
//======================================     choose info type and prepare string

    if(!page)       // Only process string for the first page (page = 0)
    {
      switch (info_type)
        {
        case INFO_ALL:
            infoPrepare(packet_string);
            break;
        default:
            break;    
        }        
    }
//======================================================     packet and transmit

    
    aux_int = strlen(packet_string);
    pages = aux_int / (250 - 4) + (aux_int % (250 - 4) > 0);
    if (page >= pages) return;
    
    packet_data[0] = CMD_GET_INFO;
    packet_data[1] = pages;
    
    strcpy(packet_data + 2, "");
    strncpy(packet_data + 2, packet_string + ((250 - 4) * page), (250 - 4));
    packet_lenght = strlen(packet_data + 2) + 4;
    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);    

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                        COMMAND SET PARAMETER
//==============================================================================


void paramSet(uint16 param_type)
{
    uint8 i;
    int32 aux_int;
    
    switch(param_type)
    {
        case PARAM_ID:
            g_mem.id = g_rx.buffer[3];        
            break;

        case PARAM_PID_CONTROL:
            g_mem.k_p = *((double *) &g_rx.buffer[3]) * 65536;
            g_mem.k_i = *((double *) &g_rx.buffer[3 + 4]) * 65536;
            g_mem.k_d = *((double *) &g_rx.buffer[3 + 8]) * 65536;
            break;

        case PARAM_STARTUP_ACTIVATION:
            g_mem.activ = g_rx.buffer[3];
            break;        

        case PARAM_INPUT_MODE:
            g_mem.input_mode = g_rx.buffer[3];
            break;

        case PARAM_CONTROL_MODE:
            g_mem.control_mode = g_rx.buffer[3];
            break;

        case PARAM_POS_RESOLUTION:
            for (i =0; i < NUM_OF_SENSORS; i++) {
                g_mem.res[i] = g_rx.buffer[i+3];
            }
            break;

        case PARAM_MEASUREMENT_OFFSET:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                g_mem.m_off[i] = 
                    *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] =
                        g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            break;

        case PARAM_MEASUREMENT_MULTIPLIER:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                g_mem.m_mult[i] = 
                    *((double *) &g_rx.buffer[3 + i * 4]);
            }
            break;

        case PARAM_POS_LIMIT_FLAG:
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
            break;

        case PARAM_POS_LIMIT:
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int16 *) &g_rx.buffer[3 + (i * 4)]);
                g_mem.pos_lim_sup[i] = *((int16 *) &g_rx.buffer[3 + (i * 4) + 2]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];

            }
            break;

        case PARAM_MAX_STEP_POS:
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int >= 0) {
                g_mem.max_step_pos = aux_int;    
            }
            break;

        case PARAM_MAX_STEP_NEG:
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0) {
                g_mem.max_step_neg = aux_int;    
            }
            break;

    }
    sendAcknowledgment();
}


//==============================================================================
//                                                         COMMAND GET PARAMETER
//==============================================================================

void paramGet(uint16 param_type)
{
    uint8 packet_data[20];
    uint16 packet_lenght;
    uint8 i;
    
    packet_data[0] = CMD_GET_PARAM;

    switch(param_type)
    {
        case PARAM_ID:
            packet_data[1] = c_mem.id;
            packet_lenght = 3;        
            break;

        case PARAM_PID_CONTROL:
            *((double *) (packet_data + 1)) = (double) c_mem.k_p / 65536;
            *((double *) (packet_data + 5)) = (double) c_mem.k_i / 65536;
            *((double *) (packet_data + 9)) = (double) c_mem.k_d / 65536;
            packet_lenght = 14;
            break;

        case PARAM_STARTUP_ACTIVATION:
            packet_data[1] = c_mem.activ;
            packet_lenght = 3;
            break;   
            
        case PARAM_INPUT_MODE:
            packet_data[1] = c_mem.input_mode;
            packet_lenght = 3;                
            break;

        case PARAM_CONTROL_MODE:
            packet_data[1] = c_mem.control_mode;
            packet_lenght = 3;
            break;
                        
        case PARAM_POS_RESOLUTION:
            for (i = 0; i < NUM_OF_SENSORS; i++) {
                packet_data[i+1] = c_mem.res[i];
            }
            packet_lenght = NUM_OF_SENSORS + 2;
            break;

        case PARAM_MEASUREMENT_OFFSET:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                *((int16 *) ( packet_data + 1 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
                
            }

            packet_lenght = 2 + NUM_OF_SENSORS * 2;            
            break;

        case PARAM_MEASUREMENT_MULTIPLIER:
            for(i = 0; i < NUM_OF_SENSORS; ++i)
            {
                *((double *) ( packet_data + 1 + (i * 4) )) = 
                    c_mem.m_mult[i];
            }

            packet_lenght = 2 + NUM_OF_SENSORS * 4;  
            break;

        case PARAM_POS_LIMIT_FLAG:
            packet_data[1] = c_mem.pos_lim_flag;
            packet_lenght = 3;
            break;

        case PARAM_POS_LIMIT:
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                *((int32 *)( packet_data + 1 + (i * 2 * 4) )) =
                    c_mem.pos_lim_inf[i];
                *((int32 *)( packet_data + 1 + (i * 2 * 4) + 4)) =
                    c_mem.pos_lim_sup[i];
            }
            packet_lenght = 2 + (NUM_OF_MOTORS * 2 * 4);
            break;

        case PARAM_MAX_STEP_POS:
            *((int32 *)(packet_data + 1)) = c_mem.max_step_pos;
            packet_lenght = 6;
            break;

        case PARAM_MAX_STEP_NEG:
            *((int32 *)(packet_data + 1)) = c_mem.max_step_neg;
            packet_lenght = 6;
            break;

    }
    
    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    commWrite(packet_data, packet_lenght);    
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i;
    int pages;
    
    unsigned char str[50];    
    strcpy(info_string, "");        
    strcat(info_string, "\r\n");
    strcat(info_string, "Firmware version: ");
    strcat(info_string, VERSION);
    strcat(info_string, ".\r\n\r\n");

    strcat(info_string,"DEVICE INFO\r\n");                       
    sprintf(str,"ID: %d\r\n",(int) c_mem.id);        
    strcat(info_string,str);
    sprintf(str,"Number of sensors: %d\r\n",(int) NUM_OF_SENSORS);        
    strcat(info_string,str);
    sprintf(str,"PWM Limit: %d\r\n",(int) device.pwm_limit);        
    strcat(info_string,str);
    strcat(info_string,"\r\n");

    strcat(info_string, "MOTOR INFO\r\n");
    strcat(info_string, "Motor references: ");
    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "%d ", (int)(g_ref.pos[i] >> c_mem.res[i]));
        strcat(info_string,str);
    }
    strcat(info_string,"\r\n");
    

    sprintf(str, "Motor enabled: ");
    if (g_ref.onoff & 0x03) {
        strcat(str,"YES\r\n");
    } else {
        strcat(str,"NO\r\n");
    }
    strcat(info_string, str);
    strcat(info_string,"\r\n");


    strcat(info_string,"\r\nMEASUREMENTS INFO\r\n");
    strcat(info_string, "Sensor value:\r\n");
    for (i = 0; i < NUM_OF_SENSORS; i++) {
        sprintf(str,"%d -> %d", i+1,
            (int) g_meas.pos[i] >> c_mem.res[i]);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
    }
    sprintf(str,"Voltage (mV): %ld", (int32) device.tension );
    strcat(info_string, str);
    strcat(info_string,"\r\n"); 

    sprintf(str,"Current 1 (mA): %ld", (int32) g_meas.curr[0] );
    strcat(info_string, str);
    strcat(info_string,"\r\n"); 

    sprintf(str,"Current 2 (mA): %ld", (int32) g_meas.curr[1] );
    strcat(info_string, str);
    strcat(info_string,"\r\n"); 
 

    strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");
    strcat(info_string, "PID Controller:\r\n");
    sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p / 65536));
    strcat(info_string, str);
    sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i / 65536));
    strcat(info_string, str);
    sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d / 65536));
    strcat(info_string, str); 
    strcat(info_string,"\r\n");

    if (c_mem.activ == 0x03) {
        strcat(info_string, "Startup activation: YES\r\n");
    } else {
        strcat(info_string, "Startup activation: NO\r\n");
    }

    switch(c_mem.input_mode) {
        case 0:
            strcat(info_string, "Input mode: USB\r\n");
            break;
        case 1:
            strcat(info_string, "Input mode: Sensor 3\r\n");
            break;
    }

    strcat(info_string, "Control Mode: ");
    switch(c_mem.control_mode) {
        case 0:
            strcat(info_string, "Position\r\n");
            break;
        case 1:
            strcat(info_string, "PWM\r\n");
            break;
        case 2:
            strcat(info_string, "Current\r\n");
            break;
    }
    


    strcat(info_string, "Sensor resolution:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %d", (int) (i + 1), 
            (int) c_mem.res[i]);
        strcat(info_string, str); 
        strcat(info_string,"\r\n");
    }


    strcat(info_string, "Measurement Offset:\r\n"); 
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %ld", (int) (i + 1), 
            (int32) c_mem.m_off[i] >> c_mem.res[i]);
        strcat(info_string, str); 
        strcat(info_string,"\r\n");
    }

    strcat(info_string, "Measurement Multiplier:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %f", (int)(i + 1), 
            (double) c_mem.m_mult[i]);
        strcat(info_string, str); 
        strcat(info_string,"\r\n");
    }

    sprintf(str, "Position limit active: %d", (int)g_mem.pos_lim_flag);
    strcat(info_string, str); 
    strcat(info_string,"\r\n");

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "Position limit motor %d: inf -> %ld  ", (int)(i + 1),
                (int32)g_mem.pos_lim_inf[i] >> g_mem.res[i]);
        strcat(info_string, str);

        sprintf(str, "sup -> %ld\r\n",
                (int32)g_mem.pos_lim_sup[i] >> g_mem.res[i]);
        strcat(info_string, str);
    }

    sprintf(str, "Max step pos and neg: %d %d", (int)g_mem.max_step_pos, (int)g_mem.max_step_neg);
    strcat(info_string, str); 
    strcat(info_string,"\r\n");

    sprintf(str, "Max stiffness: %d", (int)g_mem.max_stiffness >> g_mem.res[0]);
    strcat(info_string, str); 
    strcat(info_string,"\r\n");

    // pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);
    // sprintf(str,"Debug: %d",(int) pages);
    // strcat(info_string, str); 
    // strcat(info_string,"\r\n");
    
}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8 *packet_data, uint16 packet_lenght)
{
    uint16 i;
    
    // UART_RS485_LoadTxConfig();
    
    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':'); 
    // frame - ID                               
    UART_RS485_PutChar (g_mem.id);
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(i = 0; i < packet_lenght; ++i)
    {
        UART_RS485_PutChar(packet_data[i]); 
    }
    

    i = 0;
    
    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && i++ <= 1000){}
    
    RS485_CTS_Write(1);
    RS485_CTS_Write(0); 
    
        
    // UART_RS485_LoadRxConfig();
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


//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment() {
    int packet_lenght = 2;
    uint8 packet_data[2];

    packet_data[0] = 1;
    packet_data[1] = 1;

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================

/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

void memStore(int displacement)
{   
    uint8 writeStatus;
    int i;
    int pages;

    ISR_RS485_RX_Disable();
    ISR_MOTORS_CONTROL_Disable();
    ISR_ENCODER_Disable();
    ISR_MEASUREMENTS_Disable();

    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);

    // Retrieve temperature for better writing performance
    CySetTemp();
        
    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            break;
        }
    }
    
    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    ISR_RS485_RX_Enable();      
    ISR_MOTORS_CONTROL_Enable();
    ISR_ENCODER_Enable();
    ISR_MEASUREMENTS_Enable();
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================

/**
* This function loads user settings from the eeprom.
**/

void memRecall(void)
{
    uint16 i;
    
    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memRestore();   
    } else {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );    
    }
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================

/**
* This function loads default settings from the eeprom.
**/

void memRestore(void) {
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memInit();   
    } else {
        memStore(0);    
    }
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================

/**
* This function initialize memory when eeprom is compromised.
**/

void memInit(void)
{
    uint8 i;
    //initialize memory settings
    g_mem.id            =   1;
    g_mem.k_p           =   0.1 * 65536;
    g_mem.k_i           =   0 * 65536;
    g_mem.k_d           =   0.8 * 65536;
    g_mem.activ         =   0;
    g_mem.input_mode    =   0;
    g_mem.control_mode  =   0;

    g_mem.pos_lim_flag = 1;

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        g_mem.pos_lim_inf[i] = -30000;
        g_mem.pos_lim_sup[i] =  30000;
    }  
 
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        g_mem.m_mult[i] = 1;
        g_mem.res[i] = 1;
    }
    
    g_mem.m_off[0] = (int32)0 << g_mem.res[0];
    g_mem.m_off[1] = (int32)0 << g_mem.res[1];
    g_mem.m_off[2] = (int32)0 << g_mem.res[2];

    g_mem.max_step_pos = 0;
    g_mem.max_step_neg = 0;

    g_mem.max_stiffness = (int32)3000 << g_mem.res[0];
 
    //set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;

    //write that configuration to EEPROM
    memStore(0);
    memStore(DEFAULT_EEPROM_DISPLACEMENT);
}

/* [] END OF FILE */