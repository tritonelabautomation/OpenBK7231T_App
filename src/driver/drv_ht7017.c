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

#define HT7017_READ_CMD_MASK    0x7F
#define HT7017_REG_FREQ         0x09 

static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

static float g_dco_V = 0.00012f;
static float g_dco_I = 0.000015f;
static float g_dco_P = 0.005f;

static int g_scan_index = 0;
static uint8_t g_last_cmd = 0;

// Debug Counters
static uint32_t g_total_tx_bytes = 0;
static uint32_t g_total_rx_bytes = 0;
static uint32_t g_total_packets = 0;

static void HT7017_SendRequest(uint8_t reg_addr) {
    uint8_t send_buf[6];
    uint8_t chksum;

    // Standard 6-Byte Frame
    send_buf[0] = 0x6A;
    send_buf[1] = reg_addr & HT7017_READ_CMD_MASK;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x6A; 
    
    chksum = 0;
    for(int i=0; i<5; i++) {
        chksum += send_buf[i];
    }
    send_buf[5] = ~chksum; 

    // DEBUG: Log exactly what we are sending
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "TX[#%u]: Sending 6 bytes. Cmd: 0x%02X Checksum: 0x%02X", 
              g_scan_index, send_buf[1], send_buf[5]);

    UART_ConsumeBytes(UART_GetDataSize()); // Clear any old junk

    for(int i = 0; i < 6; i++) {
        UART_SendByte(send_buf[i]);
        g_total_tx_bytes++;
    }

    g_last_cmd = send_buf[1];
}

static void HT7017_ProcessPacket(uint8_t *rx_data) {
    g_total_packets++;
    
    // DEBUG: Log the raw packet we captured
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "PKT[#%u]: %02X %02X %02X %02X", 
              g_total_packets, rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

    uint8_t calculated_sum = 0x6A + g_last_cmd + rx_data[0] + rx_data[1] + rx_data[2];
    calculated_sum = ~calculated_sum;

    if (calculated_sum != rx_data[3]) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY, "Checksum Fail! Calc: 0x%02X != Recv: 0x%02X", calculated_sum, rx_data[3]);
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Checksum OK.");
    }

    uint32_t raw_val = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    switch(g_last_cmd) {
        case HT7017_REG_RMS_U:
            g_volts = raw_val * g_dco_V;
            CHANNEL_Set(10, (int)(g_volts * 10), 0); 
            break;
        case HT7017_REG_RMS_I1:
            g_amps = raw_val * g_dco_I;
            CHANNEL_Set(11, (int)(g_amps * 1000), 0); 
            break;
        case HT7017_REG_POWER_P1:
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000;
                g_power = (int32_t)raw_val * g_dco_P;
            } else {
                g_power = raw_val * g_dco_P;
            }
            CHANNEL_Set(12, (int)(g_power * 10), 0);
            break;
        case HT7017_REG_FREQ:
             if(raw_val > 0) {
                 g_freq = 1000000.0f / raw_val;
                 CHANNEL_Set(13, (int)(g_freq * 10), 0);
             }
             break;
    }
}

void HT7017_Init(void) {
    // Normal Initialization - 4800 Baud, 8 Data, Even Parity, 1 Stop
    UART_InitUART(4800, 2, 0); 
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Driver Initialized (Normal Mode)");
}

void HT7017_RunEverySecond(void) {
    // DEBUG: Periodic Status Report
    // This will tell us if the RX line is dead (0 bytes received)
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "--- STATS: TX Bytes: %u | RX Bytes: %u | Packets: %u ---", 
              g_total_tx_bytes, g_total_rx_bytes, g_total_packets);

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
    // Check how many bytes are sitting in the hardware buffer
    int available_bytes = UART_GetDataSize();

    // If we have data, let's peek at it!
    if (available_bytes > 0) {
        // DEBUG: Only log if something is actually there to avoid spamming 0s
        // Note: This log might happen frequently if data is coming in trickles
        // addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "UART Buffer: %d bytes available", available_bytes);
    }

    if (available_bytes >= 4) {
        uint8_t buff[4];
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Reading 4 bytes from buffer...");
        
        for(int i = 0; i < 4; i++) {
            buff[i] = UART_GetByte(i);
            g_total_rx_bytes++;
        }
        HT7017_ProcessPacket(buff);
        UART_ConsumeBytes(4);
    }
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    char tmp[128];
    sprintf(tmp, "<h5>HT7017 Debug</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "TX Total: %u<br>", g_total_tx_bytes);
    strcat(request->reply, tmp);
    sprintf(tmp, "RX Total: %u<br>", g_total_rx_bytes);
    strcat(request->reply, tmp);
    sprintf(tmp, "Packets: %u<br>", g_total_packets);
    strcat(request->reply, tmp);
    
    sprintf(tmp, "<h5>Readings</h5>");
    strcat(request->reply, tmp);
    sprintf(tmp, "V: %.2fV, I: %.3fA, P: %.2fW, F: %.1fHz<br>", g_volts, g_amps, g_power, g_freq);
    strcat(request->reply, tmp);
}

float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
