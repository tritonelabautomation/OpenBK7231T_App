#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║         HT7017 Energy Metering IC — Driver for KWS-303WF  v14             ║
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
 * ║    0x19  EMUSR    EMU status flags     bit-field (see below)              ║
 * ║                                                                            ║
 * ║  FREQ (datasheet §5.1.2.4):                                               ║
 * ║    Frequency = 500000 / raw  (femu=1MHz, OSR=64)                          ║
 * ║    50Hz: raw≈10000 ✓   60Hz: raw≈8333 ✓                                  ║
 * ║                                                                            ║
 * ║  ⚠ P1/Q1 SIGNED: raw 0xFFFFFF = -1 = ~0W (not 16,777,215!)              ║
 * ║                                                                            ║
 * ║  EMUSR register 0x19 — EMU status bits (datasheet §5.1):                  ║
 * ║    Bit 0: OVERVOLTAGE flag — set when URMS > OV threshold                 ║
 * ║    Bit 1: UNDERVOLTAGE flag — set when URMS < UV threshold                ║
 * ║    Bit 2: OVERCURRENT flag — set when I1RMS > OC threshold                ║
 * ║    Bit 3: OVERPOWER flag — set when P1 > OP threshold                    ║
 * ║    Bit 4: ZXLOSS flag — zero-crossing loss detection                     ║
 * ║    Bit 5–7: reserved                                                      ║
 * ║    NOTE: EMUSR reflects the IC's internal comparators. These are          ║
 * ║    read-only status bits. Threshold registers are NOT accessible via      ║
 * ║    the UART read protocol — thresholds are set internally by the IC       ║
 * ║    at power-on from OTP/config. Application-level protection (OVP/UVP/   ║
 * ║    OCP/OPP) is therefore implemented in firmware by comparing measured    ║
 * ║    values against user-configurable thresholds and tripping the relay.    ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  v13 TWO-SPEED POLLING (unchanged)                                        ║
 * ║                                                                            ║
 * ║  FAST table — V, I, P1 — polled on 4 out of every 5 calls:               ║
 * ║    Each fast register updated every ~1.7s                                 ║
 * ║                                                                            ║
 * ║  SLOW table — Q1, S1, FREQ, EP1, EQ1, EMUSR — 1 out of every 5 calls:   ║
 * ║    Each slow register updated every 30s   (5 calls × 6 registers)         ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  v14 ADDITIONS                                                             ║
 * ║                                                                            ║
 * ║  1. POWER FACTOR — computed every fast cycle, not just when S1 updates:  ║
 * ║     PF = P1 / (V*I) always. When S1 is fresh (<60s), cross-check with    ║
 * ║     S1. Channel published after every V/I/P fast poll.                    ║
 * ║                                                                            ║
 * ║  2. EMUSR POLLING — 0x19 added to SLOW table. On each read, status bits  ║
 * ║     decoded and published to channel HT7017_CHANNEL_EMUSR. Individual    ║
 * ║     alarm channels also published (OV, UV, OC, OP).                      ║
 * ║                                                                            ║
 * ║  3. SOFTWARE PROTECTION — configurable thresholds with hysteresis:        ║
 * ║     OverVoltage: trip relay if V > g_ovThreshold  (default 0 = disabled)  ║
 * ║     UnderVoltage: trip relay if V < g_uvThreshold (default 0 = disabled)  ║
 * ║     OverCurrent: trip relay if I > g_ocThreshold  (default 0 = disabled)  ║
 * ║     OverPower:   trip relay if P > g_opThreshold  (default 0 = disabled)  ║
 * ║     All thresholds 0 = disabled (safe default for unconfigured device)    ║
 * ║     Trip action: CMD_ExecuteCommand("setChannel <relay_ch> 0", 0)         ║
 * ║     Recovery: auto-recover after HT7017_PROT_RECOVER_SECONDS if           ║
 * ║               measurement returns to safe range (default 30s).            ║
 * ║     UV special: only trip if V > 10V (avoid false trip at power-off).     ║
 * ║                                                                            ║
 * ║  4. CONSOLE COMMANDS (new in v14):                                        ║
 * ║     HT7017_SetOV <V>    set overvoltage threshold (0=disable)             ║
 * ║     HT7017_SetUV <V>    set undervoltage threshold (0=disable)            ║
 * ║     HT7017_SetOC <A>    set overcurrent threshold (0=disable)             ║
 * ║     HT7017_SetOP <W>    set overpower threshold (0=disable)               ║
 * ║     HT7017_SetRelayChannel <ch>  which channel is the relay (default 1)   ║
 * ║     HT7017_ProtStatus   show current protection config + alarm state      ║
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
 * ║    HT7017_SetOV <V>        overvoltage trip threshold (0=disable)         ║
 * ║    HT7017_SetUV <V>        undervoltage trip threshold (0=disable)        ║
 * ║    HT7017_SetOC <A>        overcurrent trip threshold (0=disable)         ║
 * ║    HT7017_SetOP <W>        overpower trip threshold (0=disable)           ║
 * ║    HT7017_SetRelayChannel  which output channel is the relay              ║
 * ║    HT7017_ProtStatus       show protection thresholds + current state     ║
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
#define HT7017_REG_EMUSR            0x19    // EMU status flags (bit-field)

// ─── EMUSR bit positions ──────────────────────────────────────────────────────
#define HT7017_EMUSR_BIT_OV         (1u << 0)   // overvoltage
#define HT7017_EMUSR_BIT_UV         (1u << 1)   // undervoltage
#define HT7017_EMUSR_BIT_OC         (1u << 2)   // overcurrent
#define HT7017_EMUSR_BIT_OP         (1u << 3)   // overpower
#define HT7017_EMUSR_BIT_ZXLOSS     (1u << 4)   // zero-crossing loss

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

// ─── v13 two-speed scheduling (SLOW table now has 6 entries: +EMUSR) ─────────
// With FAST_RATIO=5: 4 fast polls per 5 calls, 1 slow poll per 5 calls.
//   Fast (V,I,P): each polled every ~1.7s
//   Slow (Q,S,F,EP1,EQ1,EMUSR): each polled every 30s (5 calls × 6 registers)
#define HT7017_FAST_RATIO           5

// ─── S1 freshness window for PF cross-check ───────────────────────────────────
// If S1 was last updated more than this many seconds ago, PF uses V*I only.
#define HT7017_S1_STALE_SECONDS     90

// ─── Software protection defaults (0 = disabled) ─────────────────────────────
#define HT7017_DEFAULT_OV_THRESHOLD     0.0f    // V  — set via HT7017_SetOV
#define HT7017_DEFAULT_UV_THRESHOLD     0.0f    // V  — set via HT7017_SetUV
#define HT7017_DEFAULT_OC_THRESHOLD     0.0f    // A  — set via HT7017_SetOC
#define HT7017_DEFAULT_OP_THRESHOLD     0.0f    // W  — set via HT7017_SetOP
#define HT7017_UV_MIN_VOLTAGE           10.0f   // V  — don't trip UV if V < this (power-off)
#define HT7017_PROT_RECOVER_SECONDS     30      // seconds before auto-recovery attempt
#define HT7017_PROT_TRIP_HYSTERESIS_V   2.0f    // V  — OV/UV hysteresis band
#define HT7017_PROT_TRIP_HYSTERESIS_A   0.1f    // A  — OC hysteresis band
#define HT7017_PROT_TRIP_HYSTERESIS_W   5.0f    // W  — OP hysteresis band

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
#define HT7017_CHANNEL_EMUSR        19   // raw EMUSR bitmask (0–31)
#define HT7017_CHANNEL_ALARM        20   // 0=OK, 1=OV, 2=UV, 3=OC, 4=OP (SW prot)

// ─── Behaviour ────────────────────────────────────────────────────────────────
#define HT7017_MAX_MISS_COUNT       3

// ─── Relay channel for software protection (user-configurable at runtime) ─────
// Default channel 1. Override with HT7017_SetRelayChannel command.
#define HT7017_DEFAULT_RELAY_CHANNEL    1

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
uint32_t HT7017_GetEmusr(void);
uint8_t  HT7017_GetAlarmState(void);   // 0=OK 1=OV 2=UV 3=OC 4=OP

#endif // __DRV_HT7017_H__
