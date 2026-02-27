#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║         HT7017 Energy Metering IC — Driver for KWS-303WF  v13             ║
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
 * ║  REGISTER MAP — verified against datasheet Rev1.3 §5.1                    ║
 * ║    0x06  I1RMS    Current ch1 RMS      unsigned                           ║
 * ║    0x08  URMS     Voltage RMS          unsigned                           ║
 * ║    0x09  FREQ     Line frequency       unsigned  period counter           ║
 * ║    0x0A  P1       Active power ch1     SIGNED 24-bit 2's complement       ║
 * ║    0x0B  Q1       Reactive power ch1   SIGNED 24-bit 2's complement       ║
 * ║    0x0C  S1       Apparent power ch1   unsigned  own scale                ║
 * ║    0x0D  EP1      Active energy ch1    unsigned  accumulator              ║
 * ║    0x0E  EQ1      Reactive energy ch1  unsigned  accumulator              ║
 * ║    0x10  PowerP2  Active power ch2 — NOT PFCNT (removed in v12)           ║
 * ║                                                                            ║
 * ║  FREQ (datasheet §5.1.2.4):                                               ║
 * ║    Frequency = 500000 / raw  (femu=1MHz, OSR=64)                          ║
 * ║    50Hz: raw≈10000 ✓   60Hz: raw≈8333 ✓                                  ║
 * ║                                                                            ║
 * ║  ⚠ P1/Q1 SIGNED: raw 0xFFFFFF = -1 = ~0W (not 16,777,215!)              ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  v13 TWO-SPEED POLLING                                                    ║
 * ║                                                                            ║
 * ║  OpenBeken calls HT7017_RunEverySecond() once per second.                 ║
 * ║  Inside each call, exactly ONE register is polled (half-duplex safe).     ║
 * ║                                                                            ║
 * ║  FAST table — V, I, P1 — polled on 4 out of every 5 calls:               ║
 * ║    Each fast register updated every ~1.7s  (5 calls / 3 registers)        ║
 * ║                                                                            ║
 * ║  SLOW table — Q1, S1, FREQ, EP1, EQ1 — polled 1 out of every 5 calls:   ║
 * ║    Each slow register updated every 25s   (5 calls × 5 registers)         ║
 * ║                                                                            ║
 * ║  Scheduling: g_callCount increments every second.                         ║
 * ║    if (g_callCount % 5 != 0) → advance fast index, poll fast reg          ║
 * ║    if (g_callCount % 5 == 0) → advance slow index, poll slow reg          ║
 * ║                                                                            ║
 * ║  No changes to OpenBeken glue code needed.                                ║
 * ║  HT7017_RunQuick() unchanged — ISR path still works as before.            ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CALIBRATION HISTORY                                                       ║
 * ║    Voltage  ✓ 10-point 2026-02-23  ±0.16%                                 ║
 * ║    Current  ✓ 10-point 2026-02-23  ±0.11%                                 ║
 * ║    Power    ✓ 10-point 2026-02-23  ±0.22%                                 ║
 * ║    Freq     ✓ formula 500000/raw — 50Hz & 60Hz correct                   ║
 * ║    Reactive ⚠ same scale as P1 (approximate until calibrated)             ║
 * ║    Apparent ⚠ needs S1 calibration: S_scale = raw_S1 / (V*I)             ║
 * ║    Energy   ⚠ scale=1.0 (raw counts) — calibrate with known load+time    ║
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

// ─── Register addresses — verified against datasheet Rev1.3 §5.1 ─────────────
#define HT7017_REG_RMS_I1           0x06
#define HT7017_REG_RMS_U            0x08
#define HT7017_REG_FREQ             0x09
#define HT7017_REG_POWER_P1         0x0A    // SIGNED — active power ch1
#define HT7017_REG_POWER_Q1         0x0B    // SIGNED — reactive power ch1
#define HT7017_REG_POWER_S1         0x0C    // unsigned — apparent power ch1
#define HT7017_REG_EP1              0x0D    // unsigned — active energy accumulator
#define HT7017_REG_EQ1              0x0E    // unsigned — reactive energy accumulator
// 0x10 = PowerP2 per datasheet — NOT a PFCNT register. Not polled.
#define HT7017_REG_EMUSR            0x19    // EMU status flags

// ─── Calibrated scale factors (10-point calibration 2026-02-23) ──────────────
#define HT7017_DEFAULT_VOLTAGE_SCALE    10961.98f
#define HT7017_DEFAULT_CURRENT_SCALE    1661.41f
#define HT7017_DEFAULT_POWER_SCALE      -3.1777f

// FREQ: g_fScale is the numerator constant. Formula: freq = g_fScale / raw
// Datasheet §5.1.2.4: Frequency = (femu×32)/(OSR×raw), femu=1MHz, OSR=64 → 500000/raw
#define HT7017_DEFAULT_FREQ_SCALE       500000.0f

// Current shunt DC offset — raw counts at zero current
#define HT7017_CURRENT_OFFSET           440.0f

// ─── v11 scale factors (unchanged) ───────────────────────────────────────────
#define HT7017_DEFAULT_REACTIVE_SCALE   HT7017_DEFAULT_POWER_SCALE
#define HT7017_DEFAULT_APPARENT_SCALE   1.0f
#define HT7017_DEFAULT_EP1_SCALE        1.0f

// ─── v13 two-speed scheduling ─────────────────────────────────────────────────
// Every FAST_RATIO calls, one slow register is polled instead of a fast one.
// With FAST_RATIO=5: 4 fast polls per 5 calls, 1 slow poll per 5 calls.
//   Fast (V,I,P): each polled every ~1.7s
//   Slow (Q,S,F,EP1,EQ1): each polled every 25s
#define HT7017_FAST_RATIO           5

// ─── OpenBeken channel mapping ────────────────────────────────────────────────
#define HT7017_CHANNEL_VOLTAGE      10
#define HT7017_CHANNEL_CURRENT      11
#define HT7017_CHANNEL_POWER        12
#define HT7017_CHANNEL_FREQ         13
#define HT7017_CHANNEL_PF           14
#define HT7017_CHANNEL_REACTIVE     15
#define HT7017_CHANNEL_APPARENT     16
#define HT7017_CHANNEL_WH           17   // published as Wh * 1000
#define HT7017_CHANNEL_VARH         18   // published as VARh * 1000

// ─── Behaviour ────────────────────────────────────────────────────────────────
#define HT7017_MAX_MISS_COUNT       3

// ─── Public API ───────────────────────────────────────────────────────────────
void     HT7017_Init(void);
void     HT7017_RunQuick(void);
void     HT7017_RunEverySecond(void);   // unchanged name — OpenBeken calls this
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
float    HT7017_GetReactivePower(void);
float    HT7017_GetApparentS1(void);
float    HT7017_GetWh(void);
float    HT7017_GetVARh(void);

#endif // __DRV_HT7017_H__
