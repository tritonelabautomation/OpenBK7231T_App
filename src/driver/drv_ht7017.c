/*
 * HT7017 Driver - Datasheet Compliant
 * Protocol: 4800 baud, 8 data, Even parity, 1 stop bit
 * Reference: HT7017 Datasheet (P73-13-46)
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

// --- Registers (HT7017 Datasheet) ---
#define HT7017_HEAD             0x6A
#define HT7017_REG_RMS_I1       0x06  // Current Channel 1
#define HT7017_REG_RMS_U        0x08  // Voltage Channel 1
#define HT7017_REG_FREQ         0x09  // Frequency (Period)
#define HT7017_REG_POWER_P1     0x0A  // Active Power Channel 1

// State Variables
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

// Calibration Coefficients (Default)
static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

// Debug Counters
static uint32_t g_tx_count = 0;
static uint32_t g_rx_count = 0;
static uint32_t g_pkt_count = 0;
static uint32_t g_chk_err = 0;

static int g_scan_index = 0;
static uint8_t g_last_reg = 0;

/*
 * Send Read Request
 * Datasheet: 2 Bytes [0x6A] [RegAddr]
 */
static void HT7017_SendReadRequest(uint8_t reg_addr) {
    // Clear buffer to remove noise/stale data
    UART_ConsumeBytes(UART_GetDataSize());

    // Byte 1: Header
    UART_SendByte(HT7017_HEAD);
    // Byte 2: Register Address
    UART_SendByte(reg_addr);

    g_last_reg = reg_addr;
    g_tx_count += 2;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "TX > Head:%02X Reg:%02X", HT7017_HEAD, reg_addr);
}

/*
 * Process Response
 * Datasheet: 4 Bytes [DataH] [DataM] [DataL] [Checksum]
 * Checksum = ~(DataH + DataM + DataL)
 */
static void HT7017_ProcessResponse(uint8_t *rx_data) {
    g_rx_count += 4;
    g_pkt_count++;

    // 1. Checksum Calculation (Data bytes only)
    uint8_t sum = rx_data[0] + rx_data[1] + rx_data[2];
    uint8_t calc_chk = ~sum;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
        "RX < %02X %02X %02X | Chk:%02X (Calc:%02X)", 
        rx_data[0], rx_data[1], rx_data[2], rx_data[3], calc_chk);

    if (calc_chk != rx_data[3]) {
        g_chk_err++;
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "Checksum Error!");
        // We continue processing for debug visibility
    }

    // 2. Combine 24-bit Data (Big Endian)
    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_reg) {
        case HT7017_REG_RMS_U: // 0x08
            g_volts = raw_val * g_dco_V;
            CHANNEL_Set(10, (int)(g_volts * 10), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Voltage: %.2fV (Raw:%u)", g_volts, raw_val);
            break;

        case HT7017_REG_RMS_I1: // 0x06
            g_amps = raw_val * g_dco_I;
            CHANNEL_Set(11, (int)(g_amps * 1000), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Current: %.3fA (Raw:%u)", g_amps, raw_val);
            break;

        case HT7017_REG_POWER_P1: // 0x0A
            // Power is Signed 24-bit
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            CHANNEL_Set(12, (int)(g_power * 10), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Power: %.2fW (Raw:%u)", g_power, raw_val);
            break;

        case HT7017_REG_FREQ: // 0x09
             if(raw_val > 0) {
                 // HT7017 usually returns Period. Freq = Clock / Period.
                 // Assuming 1MHz clock for now based on typical implementation.
                 g_freq = 1000000.0f / raw_val;
                 CHANNEL_Set(13, (int)(g_freq * 10), 0);
             }
             addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Freq: %.2fHz (Raw:%u)", g_freq, raw_val);
             break;
    }
}

void HT7017_Init(void) {
    // 4800 baud, 8 bits, Even Parity, 1 Stop bit
    UART_InitUART(4800, 2, 0); 
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Init: 4800,8,E,1 (2-Byte Protocol)");
}

void HT7017_RunEverySecond(void) {
    // Debug warning if bus is dead
    if(g_tx_count > 0 && g_rx_count == 0 && (g_scan_index == 0)) {
        addLogAdv(LOG_WARN, LOG_FEATURE_ENERGY, "Stats: TX=%u RX=0 (Check Pins & Autoexec)", g_tx_count);
    }

    switch (g_scan_index) {
        case 0: HT7017_SendReadRequest(HT7017_REG_RMS_U); break;
        case 1: HT7017_SendReadRequest(HT7017_REG_RMS_I1); break;
        case 2: HT7017_SendReadRequest(HT7017_REG_POWER_P1); break;
        case 3: HT7017_SendReadRequest(HT7017_REG_FREQ); break;
    }
    g_scan_index++;
    if (g_scan_index > 3) g_scan_index = 0;
}

void HT7017_RunQuick(void) {
    // Datasheet says response is 4 bytes
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
    sprintf(tmp, "TX: %u | RX: %u | Pkts: %u | Err: %u<br>", g_tx_count, g_rx_count, g_pkt_count, g_chk_err);
    strcat(request->reply, tmp);
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW, F: %.1fHz<br>", g_volts, g_amps, g_power, g_freq);
    strcat(request->reply, tmp);
}

// Getters for other modules
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
