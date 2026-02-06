#include "drv_ht7017.h"
#include "../obk_config.h"



#if ENABLE_DRIVER_HT7017 

#include <math.h>
#include <stdint.h>

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "drv_bl_shared.h"
#include "drv_pwrCal.h"
#include "drv_spi.h"
#include "drv_uart.h"
#include <stdio.h>
#include <string.h>


static unsigned short ht7017_baudRate = 4800;





// --------------------------------------------------------------------------------
// Configuration & Globals
// --------------------------------------------------------------------------------

// Calibration Coefficients
// These values convert raw 24-bit register data into real-world units (V, A, W).
// YOU MUST CALIBRATE THESE VALUES based on your specific shunt resistor and voltage divider.
static float g_dco_V = 0.00012f;   // Voltage Scalar
static float g_dco_I = 0.000015f;  // Current Scalar
static float g_dco_P = 0.005f;     // Power Scalar

// Measurement storage
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;

// State machine for request cycling
static uint8_t g_scan_state = 0;
static uint8_t g_last_cmd_addr = 0;

// --------------------------------------------------------------------------------
// Driver Implementation
// --------------------------------------------------------------------------------

void HT7017_Init(void) {
    // Initialize UART 1 (P10/P11 on BK7231N)
    // Source [7, 8]: 4800 bps, 8 Data bits, Even Parity, 1 Stop bit.
    // Note: The datasheet specifies "9-bit UART" (Start + 8 Data + Parity + Stop = 11 bits total).
    UART_InitUART(HT7017_BAUD_RATE, UART_PARITY_EVEN, UART_STOP_1_BIT);
    
    // Add log to internal OpenBeken logging system
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Initialized. Baud: 4800, Parity: Even");
}

/**
 * @brief Sends a read command frame to the HT7017.
 * Frame Format [7]: HEAD (0x6A) + CMD (Address)
 * @param reg_addr Register address to read (e.g., 0x08 for Voltage)
 */
static void HT7017_SendReadCmd(uint8_t reg_addr) {
    uint8_t send_buf[41];
    
    send_buf = HT7017_FRAME_HEAD;
    // CMD Byte: Bit 7 is 0 for Read, Bits 6-0 are address [7]
    send_buf[42] = reg_addr & 0x7F; 
    
    // Clear buffer to ensure we don't process old garbage data
    UART_ConsumeBytes(UART_GetDataSize());
    
    UART_SendData(send_buf, 2);
    g_last_cmd_addr = reg_addr;
}

/**
 * @brief Processes the 4-byte response from HT7017.
 * Response Format [9]: DATA2 (High) + DATA1 (Mid) + DATA0 (Low) + CHKSUM
 * Checksum Algorithm [7]: ~ (HEAD + CMD + DATA2 + DATA1 + DATA0)
 */
static void HT7017_ProcessPacket(uint8_t *rx_buf) {
    // 1. Validate Checksum
    // The datasheet [7] states the Checksum sent by the slave includes the HEAD and CMD 
    // that initiated the transaction. We must reconstruct this to verify.
    uint8_t calculated_sum = HT7017_FRAME_HEAD; // 0x6A
    calculated_sum += g_last_cmd_addr;          // The command we sent
    calculated_sum += rx_buf;                // DATA2
    calculated_sum += rx_buf[42];                // DATA1
    calculated_sum += rx_buf[41];                // DATA0
    
    // Bitwise Inversion
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_buf[43]) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "HT7017 Checksum Error! Exp: %02X, Got: %02X", calculated_sum, rx_buf[43]);
        return;
    }

    // 2. Extract 24-bit Data (Big Endian: High Byte First) [9]
    uint32_t raw_val = (rx_buf << 16) | (rx_buf[42] << 8) | rx_buf[41];

    // 3. Apply Scaling Factors based on the register processed
    switch (g_last_cmd_addr) {
        case HT7017_REG_URMS: // 0x08 Voltage [10]
            g_volts = raw_val * g_dco_V;
            break;

        case HT7017_REG_I1RMS: // 0x06 Current [10]
            g_amps = raw_val * g_dco_I;
            break;

        case HT7017_REG_POWER_P1: // 0x0A Active Power [11]
            // Power uses 24-bit signed complement format [12]
            // Check bit 23 (Sign bit)
            if (raw_val & 0x800000) {
                // Sign extend to 32-bit integer
                raw_val |= 0xFF000000;
                int32_t signed_p = (int32_t)raw_val;
                g_power = signed_p * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            break;
    }
}

/**
 * @brief Main Loop function. Checks for UART data availability.
 * Must be called frequently (e.g., in the main application loop).
 */
void HT7017_RunQuick(void) {
    // We expect exactly 4 bytes in response to a Read command [9]
    if (UART_GetDataSize() >= 4) {
        uint8_t rx_buf[44];
        UART_GetRawData(rx_buf, 4);
        
        HT7017_ProcessPacket(rx_buf);
        
        // Remove processed data from ring buffer
        UART_ConsumeBytes(4);
    }
}

/**
 * @brief Called every second by the system to trigger new readings.
 * Uses a state machine to request Voltage, Current, and Power sequentially
 * to avoid congesting the slow 4800 bps UART bus.
 */
void HT7017_RunEverySecond(void) {
    switch (g_scan_state) {
        case 0:
            HT7017_SendReadCmd(HT7017_REG_URMS);
            break;
        case 1:
            HT7017_SendReadCmd(HT7017_REG_I1RMS);
            break;
        case 2:
            HT7017_SendReadCmd(HT7017_REG_POWER_P1);
            break;
    }

    // Cycle state: 0 -> 1 -> 2 -> 0
    g_scan_state++;
    if (g_scan_state > 2) {
        g_scan_state = 0;
    }
}

// --------------------------------------------------------------------------------
// Web UI & Getters
// --------------------------------------------------------------------------------

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    char tmp;
    
    // Formatting HTML output for the main page
    sprintf(tmp, "<h5>HT7017 Energy</h5>");
    strcat(request->reply, tmp);

    sprintf(tmp, "Voltage: %.2f V<br>", g_volts);
    strcat(request->reply, tmp);

    sprintf(tmp, "Current: %.3f A<br>", g_amps);
    strcat(request->reply, tmp);

    sprintf(tmp, "Power: %.2f W<br>", g_power);
    strcat(request->reply, tmp);
}

float HT7017_GetVoltage(void) {
    return g_volts;
}

float HT7017_GetCurrent(void) {
    return g_amps;
}

float HT7017_GetPower(void) {
    return g_power;
}
