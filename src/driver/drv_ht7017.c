/*
 * HT7017 Energy Meter Driver (Clean Version)
 * * Protocol: 4800 baud, 8 data, Even parity, 1 stop
 * Frame: 6-Byte Read Request (Robust) -> 4-Byte Response
 */

#include "../obk_config.h"

#if ENABLE_DRIVER_HT7017

#include "drv_ht7017.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h"

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// Registers
#define HT7017_REG_RMS_U        0x03
#define HT7017_REG_RMS_I1       0x04
#define HT7017_REG_RMS_I2       0x07
#define HT7017_REG_FREQ         0x09
#define HT7017_REG_POWER_P1     0x05
#define HT7017_READ_CMD_MASK    0x7F

// State variables
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

// Calibration Defaults
static float g_voltage_cal = 0.00012f;
static float g_current_cal = 0.000015f;
static float g_power_cal = 0.005f;

// Diagnostics
static uint32_t g_total_tx_bytes = 0;
static uint32_t g_total_rx_bytes = 0;
static uint32_t g_total_packets = 0;
static uint32_t g_checksum_errors = 0;

static int g_scan_index = 0;
static uint8_t g_last_reg = 0;

/*
 * Send 6-Byte Read Request
 * [Head] [Reg] [00] [00] [Head] [Chk]
 */
static void HT7017_SendReadRequest(uint8_t reg_addr) {
    uint8_t send_buf[6];
    uint8_t chksum = 0;

    // Clear buffer to remove noise
    UART_ConsumeBytes(UART_GetDataSize());

    // Build Frame
    send_buf[0] = 0x6A;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x6A;
    
    // Calculate Checksum (Sum 0-4)
    for(int i=0; i<5; i++) {
        chksum += send_buf[i];
    }
    send_buf[5] = ~chksum; // Invert

    // Log TX
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
              "TX > Reg:0x%02X | Bytes: %02X %02X %02X %02X %02X %02X", 
              reg_addr, send_buf[0], send_buf[1], send_buf[2], send_buf[3], send_buf[4], send_buf[5]);

    // Send
    for(int i = 0; i < 6; i++) {
        UART_SendByte(send_buf[i]);
        g_total_tx_bytes++;
    }
    
    g_last_reg = reg_addr;
}

/*
 * Process 4-Byte Response
 * [D2] [D1] [D0] [Chk]
 */
static void HT7017_ProcessResponse(uint8_t *rx_data) {
    g_total_rx_bytes += 4;
    
    // Log Raw RX
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
              "RX < Raw: %02X %02X %02X %02X", 
              rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    // Verify Checksum: ~(0x6A + Reg + D2 + D1 + D0)
    uint8_t calc_sum = 0x6A + g_last_reg + rx_data[0] + rx_data[1] + rx_data[2];
    calc_sum = ~calc_sum;

    if(calc_sum != rx_data[3]) {
        g_checksum_errors++;
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "Checksum Error! Exp:0x%02X Got:0x%02X", calc_sum, rx_data[3]);
        // We continue anyway for debugging purposes
    } else {
        g_total_packets++;
    }

    // Combine 24-bit value
    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_reg) {
        case HT7017_REG_RMS_U:
            g_volts = raw_val * g_voltage_cal;
            CHANNEL_Set(10, (int)(g_volts * 10), 0); // Ch10: 2305 = 230.5V
            break;
        case HT7017_REG_RMS_I1:
            g_amps = raw_val * g_current_cal;
            CHANNEL_Set(11, (int)(g_amps * 1000), 0); // Ch11: 1500 = 1.500A
            break;
        case HT7017_REG_POWER_P1:
            if(raw_val & 0x800000) { // Signed 24-bit
                 raw_val |= 0xFF000000;
                 g_power = (int32_t)raw_val * g_power_cal;
            } else {
                 g_power = raw_val * g_power_cal;
            }
            CHANNEL_Set(12, (int)(g_power * 10), 0); // Ch12: 500 = 50.0W
            break;
        case HT7017_REG_FREQ:
            if(raw_val > 0) {
                g_freq = 1000000.0f / raw_val;
                CHANNEL_Set(13, (int)(g_freq * 10), 0); // Ch13: 500 = 50.0Hz
            }
            break;
    }
}

void HT7017_Init(void) {
    // Init UART: 4800, 8, Even, 1
    // Note: We rely on autoexec.bat to set the correct Port (0 vs 1)
    UART_InitUART(4800, 2, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Initialized (4800,8,E,1)");
}

void HT7017_RunEverySecond(void) {
    // Debug output every few seconds if no packets
    if(g_total_packets == 0 && g_total_tx_bytes > 0 && (g_scan_index == 0)) {
        addLogAdv(LOG_WARN, LOG_FEATURE_ENERGY, "Stats: TX=%u RX=%u (No valid packets yet)", g_total_tx_bytes, g_total_rx_bytes);
    }

    switch(g_scan_index) {
        case 0: HT7017_SendReadRequest(HT7017_REG_RMS_U); break;
        case 1: HT7017_SendReadRequest(HT7017_REG_RMS_I1); break;
        case 2: HT7017_SendReadRequest(HT7017_REG_POWER_P1); break;
        case 3: HT7017_SendReadRequest(HT7017_REG_FREQ); break;
    }
    g_scan_index++;
    if(g_scan_index > 3) g_scan_index = 0;
}

void HT7017_RunQuick(void) {
    // We need at least 4 bytes for a response
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[4];
        for(int i = 0; i < 4; i++) {
            buff[i] = UART_GetByte(i);
        }
        HT7017_ProcessResponse(buff);
        UART_ConsumeBytes(4);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    char tmp[128];
    sprintf(tmp, "<h5>HT7017 Stats</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "TX: %u | RX: %u | Pkts: %u<br>", g_total_tx_bytes, g_total_rx_bytes, g_total_packets);
    strcat(request->reply, tmp);
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW, F: %.1fHz<br>", g_volts, g_amps, g_power, g_freq);
    strcat(request->reply, tmp);
}

// Getters
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
