#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║         HT7017 Energy Metering IC — Driver for KWS-303WF  v11             ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DEVICE  : KWS-303WF Smart Energy Meter                                   ║
 * ║  SOC     : BK7231N CBU module                                              ║
 * ║  ARCH    : BK7231N → UART1 (4800 8E1) → HT7017 direct  (no MCU between)  ║
 * ║  SENSING : Metal shunt strip ~0.5–1mΩ  (not current transformer)          ║
 * ║                                                                            ║
 * ║  WIRING  : BK7231N P0(RX1) ←── HT7017 TX                                 ║
 * ║            BK7231N P1(TX1) ──[Diode]──► HT7017 RX                        ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  PROTOCOL                                                                  ║
 * ║    4800 baud, 8E1, half-duplex                                             ║
 * ║    Request  → [0x6A][REG]              2 bytes                            ║
 * ║    Response ← [D2][D1][D0][CHECKSUM]   4 bytes                            ║
 * ║    Raw 24-bit: (D2<<16)|(D1<<8)|D0  MSB first                             ║
 * ║    CS: ~(0x6A + REG + D2 + D1 + D0) & 0xFF                                ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  REGISTER MAP (polled)                                                     ║
 * ║    0x06  I1RMS    Current ch1 RMS      unsigned                           ║
 * ║    0x08  URMS     Voltage RMS          unsigned                           ║
 * ║    0x09  FREQ     Line frequency       unsigned  period counter           ║
 * ║    0x0A  P1       Active power ch1     SIGNED 24-bit 2's complement       ║
 * ║    0x0B  Q1       Reactive power ch1   SIGNED 24-bit 2's complement       ║
 * ║    0x0C  S1       Apparent power ch1   unsigned  own scale                ║
 * ║    0x10  PFCNT1   PF count ch1         unsigned  raw counter              ║
 * ║    0x0D  EP1      Active energy ch1    unsigned  accumulator              ║
 * ║    0x0E  EQ1      Reactive energy ch1  unsigned  accumulator              ║
 * ║                                                                            ║
 * ║  FREQ NOTE: period counter — at 50Hz raw~10000, scale=200.0               ║
 * ║  ⚠ P1/Q1 SIGNED: 0xFFFFFF=-1=~0W. Not 16,777,215!                        ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CALIBRATION HISTORY                                                       ║
 * ║    Voltage  ✓ 10-point 2026-02-23  ±0.16%                                 ║
 * ║    Current  ✓ 10-point 2026-02-23  ±0.11%                                 ║
 * ║    Power    ✓ 10-point 2026-02-23  ±0.22%                                 ║
 * ║    Freq     ✓ derived from log     ±0.03Hz                                ║
 * ║    Reactive ⚠ same scale as P1 (approximate until calibrated)             ║
 * ║    Apparent ⚠ needs S1 calibration: S_actual=V*I, S_scale=raw/S_actual   ║
 * ║    Energy   ⚠ scale=1.0 (raw counts) — calibrate with known load+time    ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  ROTATION: 12 slots × 1s = 12s full cycle                                 ║
 * ║    V  I  P  Q  S  PFCNT  F  EP1  EQ1  V  I  F                            ║
 * ║    (V, I, F repeated — updated every 6s instead of 12s)                   ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CONSOLE COMMANDS                                                          ║
 * ║    VoltageSet <V>          recalibrate voltage scale (runtime)             ║
 * ║    CurrentSet <A>          recalibrate current scale (runtime)             ║
 * ║    PowerSet <W>            recalibrate power scale (runtime)               ║
 * ║    HT7017_Status           full measurement + scale dump                  ║
 * ║    HT7017_Energy           kWh, kVARh, session time (NTP)                 ║
 * ║    HT7017_Energy_Reset     zero energy counters, record NTP timestamp     ║
 * ║    HT7017_Baud <rate>      change UART baud rate                          ║
 * ║    HT7017_NoParity         switch to 8N1                                  ║
 * ║                                                                            ║
 * ║  BUILD RULES                                                               ║
 * ║    No channel.h  |  No powerMeasurementCalibration  |  No CFG_SetExtended ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── UART ─────────────────────────────────────────────────────────────────────
#define HT7017_BAUD_RATE            4800
#define HT7017_PARITY_EVEN          2

// ─── Protocol ─────────────────────────────────────────────────────────────────
#define HT7017_FRAME_HEAD           0x6A
#define HT7017_RESPONSE_LEN         4

// ─── Register addresses ───────────────────────────────────────────────────────
#define HT7017_REG_RMS_I1           0x06
#define HT7017_REG_RMS_U            0x08
#define HT7017_REG_FREQ             0x09
#define HT7017_REG_POWER_P1         0x0A    // SIGNED
#define HT7017_REG_POWER_Q1         0x0B    // SIGNED — reactive power
#define HT7017_REG_POWER_S1         0x0C    // unsigned — apparent power
#define HT7017_REG_PFCNT1           0x10    // unsigned — PF count ch1
#define HT7017_REG_EP1              0x0D    // unsigned — active energy accum
#define HT7017_REG_EQ1              0x0E    // unsigned — reactive energy accum
#define HT7017_REG_EMUSR            0x19    // status flags

// ─── Calibrated scale factors — v10 (10-point calibration 2026-02-23) ────────
#define HT7017_DEFAULT_VOLTAGE_SCALE    10961.98f   // ±0.16%
#define HT7017_DEFAULT_CURRENT_SCALE    1661.41f    // ±0.11%
#define HT7017_DEFAULT_POWER_SCALE      -3.1777f    // ±0.22% (negative: signed raw negative for real power)
#define HT7017_DEFAULT_FREQ_SCALE       200.0f      // 50Hz grid: raw~10000

// Current shunt DC offset — raw counts at zero current (no-load baseline)
// Subtracted before current scaling. Re-measure after hardware change.
#define HT7017_CURRENT_OFFSET           440.0f

// ─── New v11 scale factors ────────────────────────────────────────────────────

// Reactive power Q1 — same signed convention as P1.
// Initial value = same as P1 scale (reasonable starting approximation).
// Refine: connect inductive load, read Q1 raw, measure VAR with power meter,
// then: Q_SCALE = signed_raw_Q1 / actual_VAR
#define HT7017_DEFAULT_REACTIVE_SCALE   HT7017_DEFAULT_POWER_SCALE

// Apparent power S1 — UNSIGNED, different raw units from P1.
// Calibrate: run any load, S_actual = V_actual * I_actual
// then: S_SCALE = raw_S1 / S_actual
// Default 1.0 shows raw counts — will be large numbers until calibrated.
#define HT7017_DEFAULT_APPARENT_SCALE   1.0f

// Energy accumulator scale EP1/EQ1 — converts raw delta to Wh/VARh.
// Default 1.0 = raw counts shown as Wh (not correct until calibrated).
// Calibrate: run known load P watts for T hours, observe raw_delta:
//   EP1_SCALE = raw_delta / (P * T / 3600)  →  raw_delta / actual_Wh
// Once EP1_SCALE is set the displayed kWh will be accurate.
#define HT7017_DEFAULT_EP1_SCALE        1.0f

// ─── OpenBeken channel mapping ────────────────────────────────────────────────
// All values published as integer * 100 (e.g. 22751 = 227.51V).
// Configure matching channels in OpenBeken as "divided by 100".
// Energy channels (WH, VARH) published as integer * 1000 (e.g. 1234 = 1.234Wh).
// Configure those as "divided by 1000".
// Set to 0 to disable publishing for that channel.

#define HT7017_CHANNEL_VOLTAGE      10
#define HT7017_CHANNEL_CURRENT      11
#define HT7017_CHANNEL_POWER        12
#define HT7017_CHANNEL_FREQ         13
#define HT7017_CHANNEL_PF           14
#define HT7017_CHANNEL_REACTIVE     15   // Q1 reactive power (VAR)
#define HT7017_CHANNEL_APPARENT     16   // S1 apparent power (VA)
#define HT7017_CHANNEL_WH           17   // active energy total (Wh * 1000)
#define HT7017_CHANNEL_VARH         18   // reactive energy total (VARh * 1000)

// ─── Behaviour ────────────────────────────────────────────────────────────────
#define HT7017_MAX_MISS_COUNT       3

// ─── Public API — v10 ─────────────────────────────────────────────────────────
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

// ─── Public API — v11 (new) ───────────────────────────────────────────────────
float    HT7017_GetReactivePower(void);
float    HT7017_GetApparentS1(void);
float    HT7017_GetWh(void);
float    HT7017_GetVARh(void);

#endif // __DRV_HT7017_H__
