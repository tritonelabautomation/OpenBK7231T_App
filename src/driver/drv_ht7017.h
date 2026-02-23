#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║         HT7017 Energy Metering IC — Driver for KWS-303WF  v8              ║
 * ║         Manufacturer: HiTrendtech (钜泉光电科技)                             ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DEVICE: KWS-303WF Smart Energy Meter                                      ║
 * ║  Architecture: BK7231N → UART1 (4800 8E1) → HT7017 direct                ║
 * ║  No intermediate MCU. Current sensing: metal shunt strip (not CT).        ║
 * ║                                                                            ║
 * ║  WIRING:                                                                   ║
 * ║    BK7231N P11 (TX1) ──[Diode]──┬──► HT7017 RX (pin 10)                  ║
 * ║    BK7231N P10 (RX1) ◄──────────┘◄── HT7017 TX (pin 11)                  ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  PROTOCOL (Datasheet §4.1)                                                 ║
 * ║    4800 baud, 8E1, half-duplex, HT7017 is slave                           ║
 * ║    Read  → [0x6A][REG]             2 bytes                                ║
 * ║    Reply ← [D2][D1][D0][CHECKSUM]  4 bytes                                ║
 * ║    Raw 24-bit: (D2<<16)|(D1<<8)|D0  MSB first                             ║
 * ║    Checksum: ~(0x6A + REG + D2 + D1 + D0) & 0xFF                          ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  REGISTER MAP                                                              ║
 * ║    0x06  I1RMS  Current ch1 RMS    unsigned                               ║
 * ║    0x08  URMS   Voltage RMS        unsigned                               ║
 * ║    0x09  FREQ   Line frequency     unsigned                               ║
 * ║    0x0A  P1     Active power ch1   SIGNED 24-bit 2's complement           ║
 * ║    0x0C  S1     Apparent power     unsigned                               ║
 * ║    0x19  EMUSR  Status flags       unsigned                               ║
 * ║  ⚠ P1 is SIGNED: 0xFFFFFF = -1 = ~0W. Not 16,777,215W!                   ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CALIBRATION SYSTEM                                                        ║
 * ║                                                                            ║
 * ║  Scale factors are stored in flash using CFG_SetExtended / CFG_GetExtended ║
 * ║  and survive reboots. The compiled #define values below are the factory    ║
 * ║  defaults used ONLY when no flash calibration has been saved.              ║
 * ║                                                                            ║
 * ║  GUI calibration (same page as BL0937) works via:                         ║
 * ║    VoltageSet <actual_volts>   — recalculates and saves voltage scale     ║
 * ║    CurrentSet <actual_amps>    — recalculates and saves current scale     ║
 * ║    PowerSet   <actual_watts>   — recalculates and saves power scale       ║
 * ║                                                                            ║
 * ║  Formula: new_scale = last_raw_reading / entered_actual_value             ║
 * ║  (For current: new_scale = (last_raw - offset) / entered_actual_amps)    ║
 * ║                                                                            ║
 * ║  FACTORY DEFAULTS — from 10-point calibration 2026-02-23:                 ║
 * ║    Voltage: ±0.16% accuracy   Current: ±0.11%   Power: ±0.22%            ║
 * ║    Frequency: derived from log, scale = 200.0 (50Hz → raw ~10000)        ║
 * ║                                                                            ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── UART ─────────────────────────────────────────────────────────────────────
#define HT7017_BAUD_RATE            4800
#define HT7017_PARITY_EVEN          2       // 8E1 mandatory per datasheet §4.1

// ─── Protocol ─────────────────────────────────────────────────────────────────
#define HT7017_FRAME_HEAD           0x6A
#define HT7017_RESPONSE_LEN         4       // D2 + D1 + D0 + CHECKSUM

// ─── Register Addresses ───────────────────────────────────────────────────────
#define HT7017_REG_RMS_I1           0x06
#define HT7017_REG_RMS_U            0x08
#define HT7017_REG_FREQ             0x09
#define HT7017_REG_POWER_P1         0x0A    // SIGNED 24-bit 2's complement
#define HT7017_REG_POWER_S1         0x0C
#define HT7017_REG_EMUSR            0x19

// ─── Factory Default Scale Factors ────────────────────────────────────────────
// These are used only when no calibration has been saved to flash.
// Once VoltageSet/CurrentSet/PowerSet is used, flash values take priority.
//
// 10-point calibration 2026-02-23 (reference meter, resistive load ~870W):
#define HT7017_DEFAULT_VOLTAGE_SCALE    10961.98f   // ±0.16%
#define HT7017_DEFAULT_CURRENT_SCALE    2319.96f    // ±0.11%
#define HT7017_DEFAULT_POWER_SCALE      -3.1777f    // ±0.22% (negative = signed raw negative for real power)
#define HT7017_DEFAULT_FREQ_SCALE       200.0f      // derived, no equipment needed

// Shunt DC offset — raw count at zero current (measured from logs, no load)
// Subtracted before current scaling. Re-measure after any hardware change.
#define HT7017_CURRENT_OFFSET           462.0f

// ─── Flash Storage Keys ───────────────────────────────────────────────────────
// CFG_SetExtended / CFG_GetExtended slots for persisting calibration.
// Choose slot numbers that don't clash with other drivers in your build.
#define HT7017_CFG_SLOT_VSCALE         20
#define HT7017_CFG_SLOT_ISCALE         21
#define HT7017_CFG_SLOT_PSCALE         22
#define HT7017_CFG_SLOT_IOFFSET        23

// ─── OpenBeken Channel Mapping ────────────────────────────────────────────────
// Set to 0 to disable publishing for that measurement.
#define HT7017_CHANNEL_VOLTAGE          10
#define HT7017_CHANNEL_CURRENT          11
#define HT7017_CHANNEL_POWER            12
#define HT7017_CHANNEL_FREQ             13
#define HT7017_CHANNEL_PF               14

// ─── Behaviour ────────────────────────────────────────────────────────────────
#define HT7017_MAX_MISS_COUNT           3

// ─── Public API ───────────────────────────────────────────────────────────────
void     HT7017_Init(void);
void     HT7017_RunQuick(void);
void     HT7017_RunEverySecond(void);
void     HT7017_AppendInformationToHTTPIndexPage(http_request_t *request);

float    HT7017_GetVoltage(void);
float    HT7017_GetCurrent(void);
float    HT7017_GetPower(void);
float    HT7017_GetFrequency(void);
float    HT7017_GetPowerFactor(void);
float    HT7017_GetApparentPower(void);
uint32_t HT7017_GetGoodFrames(void);
uint32_t HT7017_GetBadFrames(void);
uint32_t HT7017_GetTxCount(void);
uint32_t HT7017_GetMissCount(void);

#endif // __DRV_HT7017_H__
