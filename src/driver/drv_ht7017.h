#pragma once

// #ifndef __DRV_HT7017_H__
// #define __DRV_HT7017_H__

// // Only expose the public functions
// void HT7017_Init(void);
// void HT7017_RunEverySecond(void);
// void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

// #endif

#ifndef __HT7017_H__
#define __HT7017_H__

#include "new_common.h"

// HT7017 Register Addresses (Source: Datasheet Page 16-18) [2-4]
#define HT7017_REG_I1RMS        0x06 // Current Channel 1 RMS
#define HT7017_REG_I2RMS        0x07 // Current Channel 2 RMS
#define HT7017_REG_URMS         0x08 // Voltage RMS
#define HT7017_REG_POWER_P1     0x0A // Active Power Channel 1
#define HT7017_REG_POWER_Q1     0x0B // Reactive Power Channel 1
#define HT7017_REG_EMUSR        0x19 // EMU Status Register
#define HT7017_REG_WREN         0x33 // Write Enable

// Packet Constants
#define HT7017_FRAME_HEAD       0x6A
#define HT7017_BAUD_RATE        4800

// Main Driver Functions
void HT7017_Init(void);
void HT7017_RunEverySecond(void); 

// Web UI Integration (Uncommented for Main Page Display)
// This allows the driver to write HTML directly to the main page [1]
void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

// Getters for integration with OpenBeken logic
float HT7017_GetVoltage(void);
float HT7017_GetCurrent(void);
float HT7017_GetPower(void);

#endif // __HT7017_H__
