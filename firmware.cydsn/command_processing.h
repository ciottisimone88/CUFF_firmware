// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------
// File:        data_processing.c
//
// Description: Data processing functions.
// ----------------------------------------------------------------------------
//  
// Project:             qbotFirmware
// Project Manager(s):  Fabio Bonomo and Felipe Belo
//
// Soft. Ver:           0.1b4
// Date:                2012-02-06
// 
// This version changes:
//      - not reported
//
// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.          
// ----------------------------------------------------------------------------
// Permission is granted to copy, modify and redistribute this file, provided
// header message is retained.
// ----------------------------------------------------------------------------
#ifndef COMMAND_PROCESSING_H_INCLUDED
#define COMMAND_PROCESSING_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//==============================================================================
//                                                          function definitions
//==============================================================================


void    paramSet           (uint16 param_type);
void    paramGet           (uint16 param_type);
void    infoPrepare        (unsigned char *info_string);
void    infoSend           (void);
void    infoGet            (uint16 info_typea, uint8 page);
void    commProcess        (void);
void    commWrite          (uint8 *packet_data, uint16 packet_lenght);
void    memStore           (int);
void    sendAcknowledgment (void);
void    memRecall          (void);
void    memRestore         (void);
void    memInit            (void);

#endif

/* [] END OF FILE */