#include "../obk_config.h"

// Check if driver is enabled in config
#if ENABLE_DRIVER_HT7017

#include "drv_ht7017.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// OpenBeken Core Includes
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h"

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// Fix for missing UART Constants in some SDK versions
#ifndef UART_PARITY_EVEN
#define UART_PARITY_EVEN 2
#endif
#ifndef UART_STOP_1_BIT
#define UART_STOP_1_BIT 0
#endif

// Packet Constants
#define HT7017_READ_CMD_MASK    0x7F

// Globals for measurements
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;

// Calibration Scalars
static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

// State Machine
static int g_scan_index = 0;
static uint8_t g_last_cmd = 0;

// -----------------------------------------------------------------------------
// UART Helper Functions
// -----------------------------------------------------------------------------

static void HT7017_SendRequest(uint8_t reg_addr) {
    uint8_t send_buf[2];

    // FIX: Assign to the ARRAY INDEX, not the array variable
    send_buf[0] = HT7017_FRAME_HEAD;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;

    // Clear buffer
    UART_ConsumeBytes(UART_GetDataSize());

    // FIX: Use a loop with UART_SendByte (Compatible with all SDKs)
    for(int i = 0; i < 2; i++) {
        UART_SendByte(send_buf[i]);
    }

    g_last_cmd = send_buf[1];
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    // FIX: Use array indexing [0] to get the value, not the pointer address
    uint8_t calculated_sum = HT7017_FRAME_HEAD;
    calculated_sum += g_last_cmd;
    calculated_sum += rx_data[0];
    calculated_sum += rx_data[1];
    calculated_sum += rx_data[2];

    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[3]) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Exp %02X Got %02X", calculated_sum, rx_data[3]);
        return;
    }

    // FIX: Bit shift the VALUE at index 0, not the pointer
    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    // Apply Scaling
    switch(g_last_cmd) {
        case HT7017_REG_RMS_U: // Voltage
            g_volts = raw_val * g_dco_V;
            break;

        case HT7017_REG_RMS_I1: // Current
            g_amps = raw_val * g_dco_I;
            break;

        case HT7017_REG_POWER_P1: // Power
            // Sign extension for 24-bit integer
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
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
    // FIX: Use the defined constants for 8E1 (Even Parity)
    UART_InitUART(HT7017_BAUD_RATE, UART_PARITY_EVEN, UART_STOP_1_BIT);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Initialized: 4800,8,E,1");
}

void HT7017_RunEverySecond(void) {
    // Polling State Machine
    switch (g_scan_index) {
        case 0: HT7017_SendRequest(HT7017_REG_RMS_U); break;
        case 1: HT7017_SendRequest(HT7017_REG_RMS_I1); break;
        case 2: HT7017_SendRequest(HT7017_REG_POWER_P1); break;
    }
    g_scan_index++;
    if (g_scan_index > 2) g_scan_index = 0;
}

void HT7017_RunQuick(void) {
    // Read 4 bytes: DATA2, DATA1, DATA0, CHKSUM
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[4];
        // FIX: Manual loop to get bytes
        for(int i = 0; i < 4; i++) {
            buff[i] = UART_GetByte(i);
        }
        HT7017_ProcessPacket(buff);
        
        // Consume the bytes we just read
        UART_ConsumeBytes(4);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    // FIX: Ensure buffer is large enough for sprintf
    char tmp[128];
    
    sprintf(tmp, "<h5>HT7017 Energy</h5>");
    strcat(request->reply, tmp);

    sprintf(tmp, "Voltage: %.2f V<br>", g_volts);
    strcat(request->reply, tmp);

    sprintf(tmp, "Current: %.3f A<br>", g_amps);
    strcat(request->reply, tmp);

    sprintf(tmp, "Power: %.2f W<br>", g_power);
    strcat(request->reply, tmp);
}

// Getters
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
