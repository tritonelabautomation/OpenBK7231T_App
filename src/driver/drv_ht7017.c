/*
 * HT7017 "Hex Dump" Diagnostic Driver
 * Purpose: DEBUGGING ONLY.
 * Actions:
 * 1. Sends the standard 0x6A Read Command.
 * 2. Prints EVERY byte received from the sensor to the logs.
 */

#include "../obk_config.h"

#if ENABLE_DRIVER_HT7017

#include "drv_ht7017.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h"
#include "../cmnds/cmd_public.h"

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// Counters
static uint32_t g_tx_count = 0;
static uint32_t g_rx_total = 0;

// Command to switch speed on the fly
static commandResult_t CMD_HT7017_Baud(const void *context, const char *cmd, const char *args, int cmdFlags) {
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int baud = atoi(args);
    UART_InitUART(baud, 2, 0); // Try Even Parity
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "DIAG: Set Baud to %d (8E1)", baud);
    return CMD_RES_OK;
}

// Command to try "No Parity" (Some clones use this)
static commandResult_t CMD_HT7017_NoParity(const void *context, const char *cmd, const char *args, int cmdFlags) {
    UART_InitUART(4800, 0, 0); // 0 = No Parity
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "DIAG: Set to 4800 (8N1 - No Parity)");
    return CMD_RES_OK;
}

void HT7017_Init(void) {
    UART_InitReceiveRingBuffer(1024); // Large buffer for dumping
    UART_InitUART(4800, 2, 0);       // Default: 4800, 8E1
    
    CMD_RegisterCommand("HT7017_Baud", CMD_HT7017_Baud, NULL);
    CMD_RegisterCommand("HT7017_NoParity", CMD_HT7017_NoParity, NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "=== HT7017 HEX DUMP MODE STARTED ===");
    
    // Check Flag 26 again to be safe
    if (CFG_HasFlag(26)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "LISTENING ON: UART2 (Pin 6/7)");
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "LISTENING ON: UART1 (Pin 15/16)");
    }
}

void HT7017_RunEverySecond(void) {
    // Send Read Voltage Command (0x6A 0x08)
    UART_ConsumeBytes(UART_GetDataSize()); // Clear buffer before sending
    
    UART_SendByte(0x6A);
    UART_SendByte(0x08);
    g_tx_count += 2;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "TX > 6A 08 (Total Sent: %u)", g_tx_count);
}

// This function runs constantly to catch ANY incoming data
void HT7017_RunQuick(void) {
    int available = UART_GetDataSize();
    
    if (available > 0) {
        g_rx_total += available;
        
        // Print the header for this batch
        char hexDump[128] = {0};
        char *p = hexDump;
        
        for (int i = 0; i < available && i < 20; i++) { // Limit to 20 bytes per log line
            byte b = UART_GetByte(i);
            sprintf(p, "%02X ", b);
            p += 3;
        }
        
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "RX RECEIVED! Data: %s", hexDump);
        
        // Consume the bytes so we can read new ones
        UART_ConsumeBytes(available);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    char tmp[128];
    sprintf(tmp, "<h5>Hex Dump Mode</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "Total RX Bytes: %u<br>", g_rx_total);
    strcat(request->reply, tmp);
}

// Stubs to prevent linking errors
float HT7017_GetVoltage(void) { return 0; }
float HT7017_GetCurrent(void) { return 0; }
float HT7017_GetPower(void)   { return 0; }

#endif
