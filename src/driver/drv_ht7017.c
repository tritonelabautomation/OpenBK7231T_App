/*
 * HT7017 Energy Meter Driver
 * 
 * Protocol: UART 4800 baud, 8 data bits, Even parity, 1 stop bit
 * Datasheet: HT7017 User Manual (P73-13-46) Rev1.3
 * 
 * Key Protocol Details:
 * - READ:  TX 2 bytes (0x6A + Reg), RX 4 bytes (DATA2 + DATA1 + DATA0 + CHKSUM)
 * - WRITE: TX 5 bytes (0x6A + Reg|0x80 + DATA1 + DATA0 + CHKSUM), RX 1 byte (ACK)
 * - Checksum (RX): ~(DATA2 + DATA1 + DATA0)
 * - Checksum (TX): ~(0x6A + Reg + DATA1 + DATA0)
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

// Additional register addresses
#define HT7017_REG_RMS_I2       0x07  // Current Channel 2 RMS
#define HT7017_REG_FREQ         0x09  // Frequency
#define HT7017_REG_POWER_Q1     0x0B  // Reactive Power Channel 1
#define HT7017_REG_POWER_S      0x0C  // Apparent Power

// State variables
static float g_volts = 0.0f;
static float g_amps = 0.0f;
static float g_power = 0.0f;
static float g_freq = 0.0f;

// Calibration coefficients (BL0942-style)
// These are tuned based on your hardware - adjust via commands
static float g_voltage_cal = 0.00012f;   // V = raw × this
static float g_current_cal = 0.000015f;  // I = raw × this
static float g_power_cal = 0.005f;       // P = raw × this

// Communication state
static uint8_t g_last_requested_reg = 0;
static uint8_t g_scan_index = 0;

// Statistics
static uint32_t g_total_tx_bytes = 0;
static uint32_t g_total_rx_bytes = 0;
static uint32_t g_total_packets = 0;
static uint32_t g_checksum_errors = 0;
static uint32_t g_timeouts = 0;

/*
 * Send READ request (2 bytes only - per datasheet page 13)
 * Format: [0x6A] [Register Address]
 */
static void HT7017_SendReadRequest(uint8_t reg_addr) {
    // Clear any stale RX data
    int stale = UART_GetDataSize();
    if(stale > 0) {
        UART_ConsumeBytes(stale);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
                  "HT7017: Cleared %d stale bytes before TX", stale);
    }
    
    // Send 2-byte READ command
    UART_SendByte(HT7017_FRAME_HEAD);    // 0x6A
    UART_SendByte(reg_addr & 0x7F);      // Reg (bit7=0 for READ)
    
    g_last_requested_reg = reg_addr;
    g_total_tx_bytes += 2;
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 TX: READ 0x%02X → [6A %02X]", reg_addr, reg_addr);
}

/*
 * Process READ response (4 bytes: DATA2, DATA1, DATA0, CHECKSUM)
 * Checksum = ~(DATA2 + DATA1 + DATA0)
 */
static int HT7017_ProcessReadResponse(void) {
    uint8_t rx_data[4];
    uint32_t raw_value;
    uint8_t calc_checksum;
    
    // Need exactly 4 bytes
    if(UART_GetDataSize() < 4) {
        return 0;
    }
    
    // Read 4 bytes from buffer
    for(int i = 0; i < 4; i++) {
        rx_data[i] = UART_GetByte(i);
    }
    g_total_rx_bytes += 4;
    
    // Verify checksum (CRITICAL: only sum the 3 data bytes!)
    calc_checksum = ~(rx_data[0] + rx_data[1] + rx_data[2]);
    
    if(calc_checksum != rx_data[3]) {
        g_checksum_errors++;
        addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY,
                  "HT7017 RX: CHECKSUM ERROR! Calc=0x%02X != Recv=0x%02X [%02X %02X %02X %02X]",
                  calc_checksum, rx_data[3],
                  rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
        UART_ConsumeBytes(4);
        return 0;
    }
    
    // Extract 24-bit value (big-endian: MSB first)
    raw_value = ((uint32_t)rx_data[0] << 16) | 
                ((uint32_t)rx_data[1] << 8) | 
                rx_data[2];
    
    g_total_packets++;
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 RX: Reg=0x%02X Raw=0x%06X (%u) [%02X %02X %02X] CHK OK",
              g_last_requested_reg, raw_value, raw_value,
              rx_data[0], rx_data[1], rx_data[2]);
    
    // Process register-specific data
    switch(g_last_requested_reg) {
        case HT7017_REG_RMS_U:
            // Voltage: 24-bit unsigned
            g_volts = raw_value * g_voltage_cal;
            CHANNEL_Set(10, (int)(g_volts * 10), 0);  // Channel 10 = Voltage (×0.1V)
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, 
                      "→ Voltage: %.2f V (raw=%u × %.6f)", 
                      g_volts, raw_value, g_voltage_cal);
            break;
            
        case HT7017_REG_RMS_I1:
            // Current: 24-bit unsigned
            g_amps = raw_value * g_current_cal;
            CHANNEL_Set(11, (int)(g_amps * 1000), 0);  // Channel 11 = Current (×0.001A)
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "→ Current: %.3f A (raw=%u × %.6f)",
                      g_amps, raw_value, g_current_cal);
            break;
            
        case HT7017_REG_POWER_P1:
            // Power: 24-bit SIGNED (two's complement)
            if(raw_value & 0x800000) {
                // Negative power - sign extend to 32-bit
                int32_t signed_power = raw_value | 0xFF000000;
                g_power = signed_power * g_power_cal;
            } else {
                g_power = raw_value * g_power_cal;
            }
            CHANNEL_Set(12, (int)(g_power * 10), 0);  // Channel 12 = Power (×0.1W)
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "→ Power: %.2f W (raw=%d × %.6f)",
                      g_power, (int32_t)raw_value, g_power_cal);
            break;
            
        case HT7017_REG_FREQ:
            // Frequency: femu / (2 × UFREQ)
            // With femu=1MHz: Freq = 1000000 / (2 × raw)
            if(raw_value > 0) {
                g_freq = 1000000.0f / (2.0f * raw_value);
                CHANNEL_Set(13, (int)(g_freq * 10), 0);  // Channel 13 = Frequency (×0.1Hz)
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "→ Frequency: %.2f Hz (raw=%u)",
                          g_freq, raw_value);
            }
            break;
            
        default:
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "→ Unknown register 0x%02X, raw=%u",
                      g_last_requested_reg, raw_value);
            break;
    }
    
    UART_ConsumeBytes(4);
    return 1;
}

/*
 * Calibration Commands (BL0942 style)
 */
static commandResult_t HT7017_Cal_Voltage(const void *context, const char *cmd, const char *args, int flags) {
    if(!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, 
                  "HT7017: Voltage calibration = %.6f", g_voltage_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, 
                  "Usage: ht7017_cal_voltage <multiplier>");
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
                  "Example: Actual=230V, Raw=2000000 → cal=230/2000000=0.000115");
        return CMD_RES_OK;
    }
    
    float new_cal = atof(args);
    if(new_cal <= 0.0f || new_cal > 1.0f) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, 
                  "HT7017: Invalid calibration (must be 0.0-1.0)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_voltage_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD, 
              "HT7017: Voltage calibration set to %.6f", g_voltage_cal);
    return CMD_RES_OK;
}

static commandResult_t HT7017_Cal_Current(const void *context, const char *cmd, const char *args, int flags) {
    if(!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
                  "HT7017: Current calibration = %.6f", g_current_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
                  "Usage: ht7017_cal_current <multiplier>");
        return CMD_RES_OK;
    }
    
    float new_cal = atof(args);
    if(new_cal <= 0.0f || new_cal > 1.0f) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD,
                  "HT7017: Invalid calibration (must be 0.0-1.0)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_current_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
              "HT7017: Current calibration set to %.6f", g_current_cal);
    return CMD_RES_OK;
}

static commandResult_t HT7017_Cal_Power(const void *context, const char *cmd, const char *args, int flags) {
    if(!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
                  "HT7017: Power calibration = %.6f", g_power_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
                  "Usage: ht7017_cal_power <multiplier>");
        return CMD_RES_OK;
    }
    
    float new_cal = atof(args);
    if(new_cal <= 0.0f || new_cal > 1.0f) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD,
                  "HT7017: Invalid calibration (must be 0.0-1.0)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_power_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
              "HT7017: Power calibration set to %.6f", g_power_cal);
    return CMD_RES_OK;
}

/*
 * Initialize HT7017 driver
 */
void HT7017_Init(void) {
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Initializing energy meter driver...");
    
    // Initialize UART: 4800 baud, 8 data bits, Even parity (mode 2), 1 stop bit
    UART_InitUART(HT7017_BAUD_RATE, 2, false);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART initialized - 4800 baud, 8E1 (9-bit)");
    
    // Wait for HT7017 crystal to stabilize (datasheet: 20ms cold start)
    delay_ms(50);
    
    // Register calibration commands
    CMD_RegisterCommand("ht7017_cal_voltage", HT7017_Cal_Voltage, NULL);
    CMD_RegisterCommand("ht7017_cal_current", HT7017_Cal_Current, NULL);
    CMD_RegisterCommand("ht7017_cal_power", HT7017_Cal_Power, NULL);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Ready! Calibration: V=%.6f I=%.6f P=%.6f",
              g_voltage_cal, g_current_cal, g_power_cal);
}

/*
 * Called frequently (from main loop) to process incoming data
 */
void HT7017_RunQuick(void) {
    int available = UART_GetDataSize();
    
    // Process response if we have 4+ bytes
    if(available >= 4) {
        HT7017_ProcessReadResponse();
    }
    
    // Timeout detection: clear partial data if stuck
    static int partial_byte_timeout = 0;
    if(available > 0 && available < 4) {
        partial_byte_timeout++;
        
        // After 200 quick cycles (~2 seconds), clear garbage
        if(partial_byte_timeout > 200) {
            g_timeouts++;
            addLogAdv(LOG_ERROR, LOG_FEATURE_ENERGY,
                      "HT7017: Timeout! Clearing %d partial bytes (timeout #%u)",
                      available, g_timeouts);
            UART_ConsumeBytes(available);
            partial_byte_timeout = 0;
        }
    } else {
        partial_byte_timeout = 0;
    }
}

/*
 * Called every second - sends next register read request
 */
void HT7017_RunEverySecond(void) {
    uint8_t target_reg;
    
    // Log statistics periodically
    static int stats_counter = 0;
    if(++stats_counter >= 10) {  // Every 10 seconds
        stats_counter = 0;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017 Stats: TX=%u RX=%u Pkts=%u ChkErr=%u Timeouts=%u",
                  g_total_tx_bytes, g_total_rx_bytes, g_total_packets,
                  g_checksum_errors, g_timeouts);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017 Values: V=%.2fV I=%.3fA P=%.2fW F=%.2fHz",
                  g_volts, g_amps, g_power, g_freq);
    }
    
    // Cycle through registers (one read per second)
    switch(g_scan_index) {
        case 0: target_reg = HT7017_REG_RMS_U;    break;  // Voltage
        case 1: target_reg = HT7017_REG_RMS_I1;   break;  // Current
        case 2: target_reg = HT7017_REG_POWER_P1; break;  // Power
        case 3: target_reg = HT7017_REG_FREQ;     break;  // Frequency
        default:
            target_reg = HT7017_REG_RMS_U;
            g_scan_index = 0;
            break;
    }
    
    HT7017_SendReadRequest(target_reg);
    
    g_scan_index = (g_scan_index + 1) % 4;
}

/*
 * Append measurement info to web interface
 */
void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request) {
    hprintf255(request, "<h5>HT7017 Energy Meter</h5>");
    hprintf255(request, "<table>");
    hprintf255(request, "<tr><td>Voltage:</td><td><b>%.2f V</b></td></tr>", g_volts);
    hprintf255(request, "<tr><td>Current:</td><td><b>%.3f A</b></td></tr>", g_amps);
    hprintf255(request, "<tr><td>Power:</td><td><b>%.2f W</b></td></tr>", g_power);
    hprintf255(request, "<tr><td>Frequency:</td><td><b>%.2f Hz</b></td></tr>", g_freq);
    hprintf255(request, "</table>");
    
    hprintf255(request, "<h5>Statistics</h5>");
    hprintf255(request, "TX: %u bytes | RX: %u bytes<br>", 
               g_total_tx_bytes, g_total_rx_bytes);
    hprintf255(request, "Packets: %u | Errors: %u | Timeouts: %u<br>",
               g_total_packets, g_checksum_errors, g_timeouts);
    
    hprintf255(request, "<h5>Calibration</h5>");
    hprintf255(request, "Voltage: %.6f<br>", g_voltage_cal);
    hprintf255(request, "Current: %.6f<br>", g_current_cal);
    hprintf255(request, "Power: %.6f<br>", g_power_cal);
    hprintf255(request, "<small>Commands: ht7017_cal_voltage, ht7017_cal_current, ht7017_cal_power</small>");
}

/*
 * Getter functions
 */
float HT7017_GetVoltage(void) { return g_volts; }
float HT7017_GetCurrent(void) { return g_amps; }
float HT7017_GetPower(void)   { return g_power; }

#endif // ENABLE_DRIVER_HT7017
