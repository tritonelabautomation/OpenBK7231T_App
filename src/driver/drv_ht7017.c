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

// --- DIRECT SDK HACK TO FORCE P10/P11 ---
// We declare the internal Beken SDK function manually.
// Port 0 = UART1 (P10/P11) - Your Sensor
// Port 1 = UART2 (P0/P1)   - Logs
// This allows us to bypass the "default" port logic.
extern void bk_uart_init(int port, int baud, int exchange);
// ----------------------------------------

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

#define HT7017_READ_CMD_MASK    0x7F
#define HT7017_REG_FREQ         0x09 

static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

// Calibration Scalars (You can tune these later)
static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

static int g_scan_index = 0;
static uint8_t g_last_cmd = 0;

static void HT7017_SendRequest(uint8_t reg_addr) {
    uint8_t send_buf[6];
    uint8_t chksum;

    // 6-Byte Frame Format
    send_buf[0] = 0x6A;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x6A; 
    
    // Checksum: Inverse of the Sum
    chksum = 0;
    for(int i=0; i<5; i++) {
        chksum += send_buf[i];
    }
    send_buf[5] = ~chksum; 

    // Debug Log
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: TX 6-Byte Req 0x%02X", send_buf[1]);

    UART_ConsumeBytes(UART_GetDataSize());

    for(int i = 0; i < 6; i++) {
        UART_SendByte(send_buf[i]);
    }

    g_last_cmd = send_buf[1];
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    // Debug Log
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: RX Raw %02X %02X %02X %02X", 
              rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    uint8_t calculated_sum = 0x6A + g_last_cmd + rx_data[0] + rx_data[1] + rx_data[2];
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[3]) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "HT7017 Checksum Fail: Calc %02X vs RX %02X", calculated_sum, rx_data[3]);
        // Proceed anyway for debugging
    }

    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_cmd) {
        case HT7017_REG_RMS_U:
            g_volts = raw_val * g_dco_V;
            // Map to Channel 10 (x10 format, e.g. 2305 = 230.5V)
            CHANNEL_Set(10, (int)(g_volts * 10), 0); 
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Voltage %.2f V -> Ch10", g_volts);
            break;

        case HT7017_REG_RMS_I1:
            g_amps = raw_val * g_dco_I;
            // Map to Channel 11 (x1000 format, e.g. 1500 = 1.500A)
            CHANNEL_Set(11, (int)(g_amps * 1000), 0); 
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Current %.3f A -> Ch11", g_amps);
            break;

        case HT7017_REG_POWER_P1:
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            // Map to Channel 12 (x10 format, e.g. 500 = 50.0W)
            CHANNEL_Set(12, (int)(g_power * 10), 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Power %.2f W -> Ch12", g_power);
            break;

        case HT7017_REG_FREQ:
             if(raw_val > 0) {
                 // Freq = 1000000 / period
                 g_freq = 1000000.0f / raw_val;
                 // Map to Channel 13 (x10 format, e.g. 500 = 50.0Hz)
                 CHANNEL_Set(13, (int)(g_freq * 10), 0);
             }
             addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Freq %.2f Hz -> Ch13", g_freq);
             break;
    }
}

void HT7017_Init(void) {
    // 1. Standard Init (sets up internal buffers)
    UART_InitUART(4800, 2, 0); 

    // 2. HARDWARE OVERRIDE -> FORCE PORT 0 (P10/P11)
    // This tells the hardware: "Ignore the default P0/P1, connect UART to P10/P11"
    bk_uart_init(0, 4800, 1); 
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Hardware Forced to UART1 (P10/P11)");
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
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW, F: %.1fHz<br>", g_volts, g_amps, g_power, g_freq);
    strcat(request->reply, tmp);
}

float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
