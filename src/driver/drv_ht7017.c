#include "drv_ht7017.h"
#include "../obk_config.h"

// Only compile if enabled in obk_config.h or via compiler flag
// You can remove this check if you are manually adding the file to the build list
// #if ENABLE_DRIVER_HT7017 

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// OpenBeken Core Includes
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h" // Fixes implicit UART_SendData

// Define Logging Feature if not present in your SDK version
#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// Packet Constants
#define HT7017_READ_CMD_MASK    0x7F 

// Globals for measurements
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;

// Calibration Scalars (Calibrate these based on your hardware)
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
    // FIX 1: Use an array, not a single byte variable
    uint8_t send_buf[4]; 
    
    send_buf = HT7017_FRAME_HEAD;      // 0x6A
    send_buf[5] = reg_addr & HT7017_READ_CMD_MASK; // Read Bit (7) is 0

    // Clear buffer to remove old garbage data
    UART_ConsumeBytes(UART_GetDataSize());

    // Send the 2 bytes
    UART_SendData(send_buf, 2);
    
    // Store command for checksum verification later
    g_last_cmd = send_buf[5]; 
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    // HT7017 Read Response Structure (Datasheet Page 14) [8]:
    // Index 0: DATA2 (High Byte)
    // Index 1: DATA1 (Mid Byte)
    // Index 2: DATA0 (Low Byte)
    // Index 3: CHKSUM
    
    // 1. Verify Checksum
    // Algo: ~(HEAD + CMD + DATA2 + DATA1 + DATA0) = CHKSUM
    uint8_t calculated_sum = HT7017_FRAME_HEAD;
    calculated_sum += g_last_cmd;
    calculated_sum += rx_data; // FIX 2: Access array index
    calculated_sum += rx_data[5]; 
    calculated_sum += rx_data[4]; 
    
    // Bitwise Inversion
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[7]) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Exp %02X Got %02X", calculated_sum, rx_data[7]);
        return;
    }

    // 2. Extract 24-bit Value (Big Endian)
    // FIX 2: Shift the data value, not the pointer address
    uint32_t raw_val = (rx_data << 16) | (rx_data[5] << 8) | rx_data[4];

    // 3. Apply Scaling
    switch(g_last_cmd) {
        case HT7017_REG_RMS_U: // 0x08 Voltage
            g_volts = raw_val * g_dco_V;
            break;

        case HT7017_REG_RMS_I1: // 0x06 Current
            g_amps = raw_val * g_dco_I;
            break;

        case HT7017_REG_POWER_P1: // 0x0A Active Power
            // Power is signed 24-bit (Bit 23 is sign)
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000; // Sign extend to 32-bit
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
    // UART Init: 4800 bps, Even Parity, 1 Stop Bit
    // Source: Datasheet 4.1.1 [9]
    UART_InitUART(HT7017_BAUD_RATE, UART_PARITY_EVEN, UART_STOP_1_BIT);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Initialized: 4800,8,E,1");
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
    // Check if we have received the 4-byte response
    // Response: DATA2, DATA1, DATA0, CHKSUM
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[10]; // Local buffer
        UART_GetRawData(buff, 4);
        HT7017_ProcessPacket(buff);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    // FIX 4: Use a large buffer, not a single char
    char tmp; 
    
    sprintf(tmp, "<h5>HT7017 Energy</h5>");
    strcat(request->reply, tmp);

    sprintf(tmp, "Voltage: %.2f V<br>", g_volts);
    strcat(request->reply, tmp);

    sprintf(tmp, "Current: %.3f A<br>", g_amps);
    strcat(request->reply, tmp);

    sprintf(tmp, "Power: %.2f W<br>", g_power);
    strcat(request->reply, tmp);
}

// Getters for integration with other OBK modules (MQTT, etc.)
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

// #endif // ENABLE_DRIVER_HT7017
