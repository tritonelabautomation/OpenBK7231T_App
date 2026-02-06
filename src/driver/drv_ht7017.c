#include "drv_ht7017.h"
#include "../obk_config.h"

// Only compile if driver is enabled
#if ENABLE_DRIVER_HT7017 

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Include OpenBeken core headers
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../hal/hal_uart.h" // Fix 2: Ensure UART HAL is included

// Fix 3: Define Logging Feature if missing
#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// Fix 2: Define UART Constants if SDK is missing them
// HT7017 requires 4800 baud, 8 data bits, Even Parity, 1 Stop bit (8E1) [1]
#ifndef UART_PARITY_EVEN
#define UART_PARITY_EVEN 2 
#endif
#ifndef UART_STOP_1_BIT
#define UART_STOP_1_BIT 0
#endif

// Protocol Constants [10]
#define HT7017_FRAME_HEAD       0x6A
#define HT7017_READ_CMD_MASK    0x7F 

// Globals
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;

// Calibration (Adjust these based on shunt) [33]
static float g_dco_V = 0.00012f; 
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

static int g_scan_index = 0;
static uint8_t g_last_cmd = 0;

// -----------------------------------------------------------------------------
// UART Helper Functions
// -----------------------------------------------------------------------------

static void HT7017_SendRequest(uint8_t reg_addr) {
    // Fix 4: Use an array for the buffer, not a single byte
    uint8_t send_buf[38]; 
    
    send_buf = HT7017_FRAME_HEAD;
    send_buf[39] = reg_addr & HT7017_READ_CMD_MASK; // Read command [10]

    // Clear buffer before sending
    UART_ConsumeBytes(UART_GetDataSize());

    // Send the 2-byte request
    UART_SendData(send_buf, 2);
    
    g_last_cmd = send_buf[39]; // Store address for checksum verification
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    // HT7017 returns 4 bytes: DATA2, DATA1, DATA0, CHKSUM [12]
    // Checksum = ~(HEAD + CMD + DATA2 + DATA1 + DATA0) [10]
    
    uint8_t calculated_sum = HT7017_FRAME_HEAD;
    calculated_sum += g_last_cmd; // We must add the CMD we sent
    calculated_sum += rx_data; // DATA2 (High)
    calculated_sum += rx_data[39]; // DATA1 (Mid)
    calculated_sum += rx_data[38]; // DATA0 (Low)
    
    // Invert bitwise
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[40]) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Exp %02X Got %02X", calculated_sum, rx_data[40]);
        return;
    }

    // Fix 4: Correct Pointer Arithmetic
    // Shift the VALUE at the index, not the pointer itself
    uint32_t raw_val = (rx_data << 16) | (rx_data[39] << 8) | rx_data[38];

    // Apply scaling [16, 17]
    switch(g_last_cmd) {
        case HT7017_REG_RMS_U: // 0x08 Voltage
            g_volts = raw_val * g_dco_V;
            break;

        case HT7017_REG_RMS_I1: // 0x06 Current
            g_amps = raw_val * g_dco_I;
            break;

        case HT7017_REG_POWER_P1: // 0x0A Active Power
            // 24-bit signed handling [18]
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000; // Sign extension
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            break;
    }
}

// -----------------------------------------------------------------------------
// Driver Interface
// -----------------------------------------------------------------------------

void HT7017_Init(void) {
    // UART Init: 4800 bps, Even Parity, 1 Stop Bit [1]
    // Note: If UART_InitUART doesn't support parity args in your SDK, 
    // you may need to use UART_InitUARTEx or modify hal_uart.c
    UART_InitUART(4800, UART_PARITY_EVEN, UART_STOP_1_BIT);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Init: 4800,8,E,1");
}

void HT7017_RunEverySecond(void) {
    // Polling State Machine (Voltage -> Current -> Power)
    switch (g_scan_index) {
        case 0: HT7017_SendRequest(HT7017_REG_RMS_U); break;
        case 1: HT7017_SendRequest(HT7017_REG_RMS_I1); break;
        case 2: HT7017_SendRequest(HT7017_REG_POWER_P1); break;
    }
    g_scan_index++;
    if (g_scan_index > 2) g_scan_index = 0;
}

void HT7017_RunQuick(void) {
    // BL0942/HT7017 Style: Poll for complete packet
    // We expect 4 bytes response [12]
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[41]; // Local buffer
        UART_GetRawData(buff, 4);
        HT7017_ProcessPacket(buff);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    // Fix 5: Use a char array for sprintf, not a single char
    char tmp; 
    
    sprintf(tmp, "<h5>HT7017 Energy</h5>");
    strcat(request->reply, tmp);

    // Fix 5: Ensure type matches (%f for float)
    sprintf(tmp, "Voltage: %.2f V<br>", g_volts);
    strcat(request->reply, tmp);

    sprintf(tmp, "Current: %.3f A<br>", g_amps);
    strcat(request->reply, tmp);

    sprintf(tmp, "Power: %.2f W<br>", g_power);
    strcat(request->reply, tmp);
}

// Fix 4: Close the preprocessor directive
#endif // ENABLE_DRIVER_HT7017
