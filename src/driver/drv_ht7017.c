#include "../obk_config.h"

// Check if driver is enabled in obk_config.h
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

// UART Constants (if missing in SDK)
#ifndef UART_PARITY_EVEN
#define UART_PARITY_EVEN 2
#endif
#ifndef UART_STOP_1_BIT
#define UART_STOP_1_BIT 0
#endif

// Register Definitions
#define HT7017_READ_CMD_MASK    0x7F
#define HT7017_REG_FREQ         0x09 // Frequency Register

// Globals for measurements
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f; 

// Calibration Scalars (Default values, tune these later)
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

    // Prepare Command: Head + Address
    send_buf[0] = HT7017_FRAME_HEAD;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;

    // DEBUG: Log what we are sending
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: TX Request for Reg 0x%02X", send_buf[1]);

    // Clear buffer to ensure we don't read old data
    UART_ConsumeBytes(UART_GetDataSize());

    // Send bytes manually (Standard Loop)
    for(int i = 0; i < 2; i++) {
        UART_SendByte(send_buf[i]);
    }

    // Store the command byte for checksum verification later
    g_last_cmd = send_buf[1];
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    // DEBUG: Print the raw 4 bytes received
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: RX Raw: %02X %02X %02X %02X", 
              rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    // Checksum Calculation: ~(Head + Cmd + Data2 + Data1 + Data0)
    uint8_t calculated_sum = HT7017_FRAME_HEAD;
    calculated_sum += g_last_cmd; // Add the hidden command byte
    calculated_sum += rx_data[0];
    calculated_sum += rx_data[1];
    calculated_sum += rx_data[2];

    calculated_sum = ~calculated_sum; // Invert

    if (calculated_sum != rx_data[3]) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Exp %02X Got %02X", calculated_sum, rx_data[3]);
        return;
    }

    // Extract 24-bit Raw Value (Big Endian)
    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    // Apply Scaling based on what we asked for
    switch(g_last_cmd) {
        case HT7017_REG_RMS_U: // Voltage
            g_volts = raw_val * g_dco_V;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Voltage: %.2f V (Raw: %u)", g_volts, raw_val);
            break;

        case HT7017_REG_RMS_I1: // Current
            g_amps = raw_val * g_dco_I;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Current: %.3f A (Raw: %u)", g_amps, raw_val);
            break;

        case HT7017_REG_POWER_P1: // Active Power
            // Handle 24-bit Signed Integer
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000; // Sign extend
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Power: %.2f W (Raw: %u)", g_power, raw_val);
            break;

        case HT7017_REG_FREQ: // Frequency
            // Frequency is usually Clock / Period. 
            // We log raw first to determine the constant.
            // g_freq = CONSTANT / raw_val; 
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Freq Raw Value: %u", raw_val);
            break;
    }
}

// -----------------------------------------------------------------------------
// Driver Interface
// -----------------------------------------------------------------------------

void HT7017_Init(void) {
    // Initialize UART: 4800 Baud, Even Parity, 1 Stop Bit (8E1)
    UART_InitUART(HT7017_BAUD_RATE, UART_PARITY_EVEN, UART_STOP_1_BIT);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Initialized: 4800,8,E,1");
}

void HT7017_RunEverySecond(void) {
    // Poll a different register every second to avoid bus congestion
    switch (g_scan_index) {
        case 0: HT7017_SendRequest(HT7017_REG_RMS_U); break;
        case 1: HT7017_SendRequest(HT7017_REG_RMS_I1); break;
        case 2: HT7017_SendRequest(HT7017_REG_POWER_P1); break;
        case 3: HT7017_SendRequest(HT7017_REG_FREQ); break; 
    }
    g_scan_index++;
    if (g_scan_index > 3) g_scan_index = 0;
}

void HT7017_RunQuick(void) {
    // Wait for complete 4-byte response packet
    // Packet: [DATA_HIGH] [DATA_MID] [DATA_LOW] [CHECKSUM]
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[4];
        
        // Manual loop to read bytes (SDK compatible)
        for(int i = 0; i < 4; i++) {
            buff[i] = UART_GetByte(i);
        }
        
        HT7017_ProcessPacket(buff);
        
        // Remove processed bytes from buffer
        UART_ConsumeBytes(4);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
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

// Getters for other modules
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
