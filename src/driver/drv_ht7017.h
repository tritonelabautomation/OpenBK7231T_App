#pragma once

 //#ifndef __DRV_HT7017_H__
// #define __DRV_HT7017_H__

// // Only expose the public functions
// void HT7017_Init(void);
// void HT7017_RunEverySecond(void);
// void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

// #endif

#ifndef __HT7017_H__
#define __HT7017_H__

#include "../new_common.h"
// Fix 1: Include HTTP definition for http_request_t
#include "../httpserver/new_http.h" 

// Register Definitions [15-17]
#define HT7017_REG_RMS_I1       0x06 
#define HT7017_REG_RMS_U        0x08 
#define HT7017_REG_POWER_P1     0x0A 

void HT7017_Init(void);
void HT7017_RunEverySecond(void);
void HT7017_RunQuick(void);
void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

#endif // __HT7017_H__
