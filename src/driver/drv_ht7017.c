/*
 * HT7017 Driver - Datasheet Compliant (2-Byte Read)
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

// --- Registers per Datasheet Page 13 ---
#define HT7017_REG_RMS_I1       0x06  // Current
#define HT7017_REG_RMS_U        0x08  // Voltage
#define HT7017_REG_FREQ         0x09  // Frequency
#define HT7017_REG_POWER_P1     0x0A  // Power
#define HT7017_HEAD             0x6A

// State
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

// Calibration Defaults
static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

// Debug Stats
static uint32_t g_tx_count = 0;
static uint32_t g_rx_count = 0;
static uint32_t g_pkt_count = 0;
static uint32_t g_chk_err = 0;

static int g_scan_index = 0;
static uint8_t g_last_reg = 0;

/*
 * Send Read Request (2 Bytes ONLY)
 * Format: [0x6A] [RegAddr]
 */
static void HT7017_SendReadRequest(uint8_t reg_addr) {
    // Clear buffer to ensure we read fresh data
    UART_ConsumeBytes(UART_GetDataSize());

    // Send 2 Bytes Only
    UART_SendByte(HT7017_HEAD);
    UART_SendByte(reg_addr);

    g_last_reg = reg_addr;
    g_tx_count += 2;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "TX > Head:%02X Reg:%02X", HT7017_HEAD, reg_addr);
}

/*
 * Process Response (4 Bytes)
 * Format: [DataH] [DataM] [DataL] [Checksum]
 */
static void HT7017_ProcessResponse(uint8_t *rx_data) {
    g_rx_count += 4;
    g_pkt_count++;

    // 1. Calculate Checksum: ~(DataH + DataM + DataL)
    uint8_t sum = rx_data[0] + rx_data[1] + rx_data[2];
    uint8_t calc_chk = ~sum;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
        "RX < %02X %02X %02X | Chk:%02X (Calc:%02X)", 
        rx_data[0], rx_data[1], rx_data[2], rx_data[3], calc_chk);

    if (calc_chk != rx_data[3]) {
        g_chk_err++;
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "Checksum Error!");
    }

    // 2. Parse 24-bit Value
    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_reg) {
        case HT7017_REG_RMS_U:
            g_volts = raw_val * g_dco_V;
            CHANNEL_Set(10, (int)(g_volts * 10), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Voltage: %.2fV", g_volts);
            break;

        case HT7017_REG_RMS_I1:
            g_amps = raw_val * g_dco_I;
            CHANNEL_Set(11, (int)(g_amps * 1000), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Current: %.3fA", g_amps);
            break;

        case HT7017_REG_POWER_P1:
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            CHANNEL_Set(12, (int)(g_power * 10), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Power: %.2fW", g_power);
            break;

        case HT7017_REG_FREQ:
             if(raw_val > 0) {
                 g_freq = 1000000.0f / raw_val;
                 CHANNEL_Set(13, (int)(g_freq * 10), 0);
             }
             addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Freq: %.2fHz", g_freq);
             break;
    }
}

void HT7017_Init(void) {
    UART_InitUART(4800, 2, 0); 
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Init: 2-Byte Protocol");
}

void HT7017_RunEverySecond(void) {
    if(g_tx_count > 0 && g_rx_count == 0) {
        addLogAdv(LOG_WARN, LOG_FEATURE_ENERGY, "Stats: TX=%u RX=0 (Check Pins/Baud)", g_tx_count);
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
    sprintf(tmp, "<h5>HT7017</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW, F: %.1fHz<br>", g_volts, g_amps, g_power, g_freq);
    strcat(request->reply, tmp);
    sprintf(tmp, "Debug: TX=%u RX=%u Err=%u<br>", g_tx_count, g_rx_count, g_chk_err);
    strcat(request->reply, tmp);
}

float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
