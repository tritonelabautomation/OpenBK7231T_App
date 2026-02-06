#include "../obk_config.h"

// Check if driver is enabled
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

// UART Constants
#ifndef UART_PARITY_EVEN
#define UART_PARITY_EVEN 2
#endif
#ifndef UART_STOP_1_BIT
#define UART_STOP_1_BIT 0
#endif

#define HT7017_READ_CMD_MASK    0x7F
#define HT7017_REG_FREQ         0x09 

// --- CRITICAL HACK FOR BK7231N ---
// 0 = UART1 (P10/P11) -> Your Sensor
// 1 = UART2 (P0/P1)   -> Default/Logs
extern int g_uart_port; 
// ---------------------------------

static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;

static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

static int g_scan_index = 0;
static uint8_t g_last_cmd = 0;

static void HT7017_SendRequest(uint8_t reg_addr) {
    uint8_t send_buf[6];
    uint8_t chksum;

    // PROTOCOL: Fixed 6-Byte Read Frame per Datasheet
    send_buf[0] = 0x6A;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x6A; 

    // Calculate Checksum: Sum of bytes 0-4
    chksum = 0;
    for(int i=0; i<5; i++) {
        chksum += send_buf[i];
    }
    // Datasheet requires bitwise inversion
    send_buf[5] = ~chksum; 

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: TX 6-Byte Req 0x%02X", send_buf[1]);

    UART_ConsumeBytes(UART_GetDataSize());

    for(int i = 0; i < 6; i++) {
        UART_SendByte(send_buf[i]);
    }

    g_last_cmd = send_buf[1];
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: RX Raw %02X %02X %02X %02X", 
              rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    // Checksum verification
    uint8_t calculated_sum = 0x6A + g_last_cmd + rx_data[0] + rx_data[1] + rx_data[2];
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[3]) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Calc %02X vs RX %02X", calculated_sum, rx_data[3]);
        // We do NOT return here, so we can debug the data even if checksum fails
    }

    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_cmd) {
        case HT7017_REG_RMS_U:
            g_volts = raw_val * g_dco_V;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Voltage: %.2f", g_volts);
            break;
        case HT7017_REG_RMS_I1:
            g_amps = raw_val * g_dco_I;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Current: %.3f", g_amps);
            break;
        case HT7017_REG_POWER_P1:
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Power: %.2f", g_power);
            break;
        case HT7017_REG_FREQ:
             addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Freq Raw: %u", raw_val);
             break;
    }
}

void HT7017_Init(void) {
    // --- FORCE PORT TO UART1 (P10/P11) ---
    // This is the specific fix for your "command not found" error
    g_uart_port = 0; 
    // -------------------------------------

    UART_InitUART(HT7017_BAUD_RATE, UART_PARITY_EVEN, UART_STOP_1_BIT);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 Init on UART1 (P10/P11)");
}

void HT7017_RunEverySecond(void) {
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
    if (UART_GetDataSize() >= 4) {
        uint8_t buff[4];
        for(int i = 0; i < 4; i++) {
            buff[i] = UART_GetByte(i);
        }
        HT7017_ProcessPacket(buff);
        UART_ConsumeBytes(4);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    char tmp[128];
    sprintf(tmp, "<h5>HT7017</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW<br>", g_volts, g_amps, g_power);
    strcat(request->reply, tmp);
}

float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
