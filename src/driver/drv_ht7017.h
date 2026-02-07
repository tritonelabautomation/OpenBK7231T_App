#pragma once

#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

// // Only expose the public functions
// void HT7017_Init(void);
// void HT7017_RunEverySecond(void);
// void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

// #endif

#include "../new_common.h"
#include "../httpserver/new_http.h" // Fixes unknown type http_request_t

// HT7017 Register Addresses (Source: Datasheet Page 16-18) [2][3]
#define HT7017_REG_RMS_I1       0x06 // Current Channel 1 RMS
#define HT7017_REG_RMS_U        0x08 // Voltage RMS
#define HT7017_REG_POWER_P1     0x0A // Active Power Channel 1
#define HT7017_REG_FREQ         0x09 // Frequency (Added)

// Packet Constants
#define HT7017_FRAME_HEAD       0x6A
#define HT7017_BAUD_RATE        4800

// Driver Interface
void HT7017_Init(void);
void HT7017_RunQuick(void);       
void HT7017_RunEverySecond(void); 
void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

// Getters
float HT7017_GetVoltage(void);
float HT7017_GetCurrent(void);
float HT7017_GetPower(void);

#endif // __HT7017_H__
