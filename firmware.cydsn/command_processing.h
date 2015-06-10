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


void     paramSet           (uint16 param_type);
void     paramGet           (uint16 param_type);
void     infoPrepare        (unsigned char *info_string);
void     infoGet            (uint16 info_typea);
void     commProcess        (void);
void     commWrite          (uint8 *packet_data, uint16 packet_lenght);
uint8    memStore           (int);
void     sendAcknowledgment (uint8 value);
void     memRecall          (void);
uint8    memRestore         (void);
uint8    memInit            (void);

#endif

/* [] END OF FILE */