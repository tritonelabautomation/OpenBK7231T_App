#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * HT7017 Single-Phase Energy Metering IC Driver
 * Manufacturer: HiTrendtech (钜泉光电科技)
 * Protocol: UART 4800 baud, 9-bit (8E1), half-duplex
 *
 * Wiring (BK7231N):
 *   P11 (TX1) ---[Diode]-+---> HT7017 RX (pin 10)
 *                        |
 *   P10 (RX1) <----------+---- HT7017 TX (pin 11)
 *
 * Read frame:  Master sends  → [0x6A][REG]
 * Response:    HT7017 sends  → [DATA2][DATA1][DATA0][CHECKSUM]
 * Checksum:    ~(0x6A + REG + DATA2 + DATA1 + DATA0) & 0xFF
 * Raw value:   (DATA2 << 16) | (DATA1 << 8) | DATA0  (24-bit, MSB first)
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── UART Settings ────────────────────────────────────────────────────────────
#define HT7017_BAUD_RATE        4800
#define HT7017_PARITY_EVEN      2       // 8E1 as required by datasheet

// ─── Protocol Constants ───────────────────────────────────────────────────────
#define HT7017_FRAME_HEAD       0x6A    // Fixed frame header for all commands
#define HT7017_RESPONSE_LEN     4       // DATA2 + DATA1 + DATA0 + CHECKSUM
#define HT7017_ACK_OK           0x54    // Write operation ACK success
#define HT7017_ACK_FAIL         0x63    // Write operation ACK failure

// ─── Register Addresses (Datasheet §5.1) ─────────────────────────────────────
#define HT7017_REG_RMS_I1       0x06    // Current Channel 1 RMS
#define HT7017_REG_RMS_U        0x08    // Voltage RMS
#define HT7017_REG_FREQ         0x09    // Line Frequency
#define HT7017_REG_POWER_P1     0x0A    // Active Power Channel 1
#define HT7017_REG_POWER_Q1     0x0B    // Reactive Power Channel 1
#define HT7017_REG_POWER_S1     0x0C    // Apparent Power Channel 1
#define HT7017_REG_EMUSR        0x19    // Status register (creep detection)

// ─── Calibration Scale Factors ───────────────────────────────────────────────
// Voltage: calibrated at 244.30V actual → raw ~2,690,940
// Formula: scale = raw / actual_value
// To recalibrate: measure actual with trusted meter, note raw from log, divide.
#define HT7017_VOLTAGE_SCALE    11015.3f

// Current & Power: set these after connecting a known resistive load
// (e.g. 1000W kettle) and comparing raw values to a trusted clamp meter.
// Leave as 1.0 until calibrated — logs will show raw counts.
#define HT7017_CURRENT_SCALE    1.0f    // TODO: calibrate with known load
#define HT7017_POWER_SCALE      1.0f    // TODO: calibrate with known load
#define HT7017_FREQ_SCALE       100.0f  // Typical: raw / 100 = Hz (verify)

// ─── Public API ───────────────────────────────────────────────────────────────
void  HT7017_Init(void);
void  HT7017_RunQuick(void);
void  HT7017_RunEverySecond(void);
void  HT7017_AppendInformationToHTTPIndexPage(http_request_t *request);

// Measurement getters
float HT7017_GetVoltage(void);   // Volts RMS
float HT7017_GetCurrent(void);   // Amps RMS
float HT7017_GetPower(void);     // Watts (active)
float HT7017_GetFrequency(void); // Hz

// Diagnostic getters
uint32_t HT7017_GetGoodFrames(void);
uint32_t HT7017_GetBadFrames(void);
uint32_t HT7017_GetTxCount(void);

#endif // __DRV_HT7017_H__
