#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║         HT7017 Energy Metering IC — Driver for KWS-303WF  v15             ║
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
 * ║    Bit 0: OVERVOLTAGE  Bit 1: UNDERVOLTAGE  Bit 2: OVERCURRENT            ║
 * ║    Bit 3: OVERPOWER    Bit 4: ZXLOSS        Bits 5-7: reserved            ║
 * ║    NOTE: read-only status. Thresholds are OTP — cannot be set via UART.   ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  v15 TWO-SPEED POLLING                                                    ║
 * ║                                                                            ║
 * ║  FAST table — V, I, P, FREQ (4 registers):                                ║
 * ║    Polled 4 out of every 5 calls → each updated every ~1.25s              ║
 * ║    FREQ moved from slow to fast — was 30s, now 1.25s (FIX #1)            ║
 * ║                                                                            ║
 * ║  SLOW table — Q, S, EP1, EQ1, EMUSR (5 registers):                       ║
 * ║    Polled 1 out of every 5 calls → each updated every 25s                 ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  v15 FIXES over v14                                                        ║
 * ║                                                                            ║
 * ║  FIX 1 — FREQ moved to fast table                                         ║
 * ║    Was: slow table slot S2 → updated every 30s                            ║
 * ║    Now: fast table slot F3 → updated every ~1.25s                         ║
 * ║    FAST_TABLE_SIZE 3→4.  SLOW_TABLE_SIZE 6→5 (FREQ removed from slow).   ║
 * ║                                                                            ║
 * ║  FIX 2 — Hysteresis now actually applied in EvaluateProtection()          ║
 * ║    v14 defined HYSTERESIS macros but never used them — relay chatters at  ║
 * ║    the threshold boundary. v15 uses separate trip/clear thresholds:       ║
 * ║      Trip  when value > threshold                                          ║
 * ║      Clear when value < threshold - hysteresis  (OV/OC/OP)               ║
 * ║      Clear when value > threshold + hysteresis  (UV)                      ║
 * ║                                                                            ║
 * ║  FIX 3 — Channel scaling made consistent                                  ║
 * ║    Channels 1-7 use per-measurement multipliers:                          ║
 * ║      FREQ:  50.12 Hz  → setChannel 4 5012  (÷100 = Hz)                  ║
 * ║      PF:    0.980     → setChannel 5 980    (÷1000 = PF)                 ║
 * ║      ALARM: state 2   → setChannel 7 2      (raw int, NOT ×multiplier)   ║
 * ║    ALARM channel publishes raw 0-4 (not ×any scale) — it is a state      ║
 * ║    code, not a measurement. Rules read it as: if ch7 == 1 (OV), etc.     ║
 * ║                                                                            ║
 * ║  FIX 4 — Recovery call-count formula corrected                            ║
 * ║    v14: recover_calls = RECOVER_SECONDS * 4 / 5  (wrong, ~20% short)     ║
 * ║    v15: recover_calls = RECOVER_SECONDS          (g_callCount = 1/second) ║
 * ║                                                                            ║
 * ║  FIX 5 — Misleading g_callCount comment removed                           ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  AUTOEXEC CHANNEL TYPES (copy into autoexec.bat)                          ║
 * ║    setChannelType 1  voltage_div100            // Voltage (V)             ║
 * ║    setChannelType 2  current_div1000           // Current (A)             ║
 * ║    setChannelType 3  power_div10               // Power (W)               ║
 * ║    setChannelType 4  ReadOnly                  // Frequency (÷100 = Hz)   ║
 * ║    setChannelType 5  PowerFactor_div1000       // Power Factor            ║
 * ║    setChannelType 6  EnergyTotal_kWh_div10000  // Energy (Wh×10)         ║
 * ║    setChannelType 7  ReadOnly                  // Alarm state 0-4         ║
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
 * ║    PowerSet <W>            recalibrate power + Q + S scales (runtime)     ║
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

// ─── Register addresses ───────────────────────────────────────────────────────
#define HT7017_REG_RMS_I1           0x06
#define HT7017_REG_RMS_U            0x08
#define HT7017_REG_FREQ             0x09
#define HT7017_REG_POWER_P1         0x0A    // SIGNED
#define HT7017_REG_POWER_Q1         0x0B    // SIGNED
#define HT7017_REG_POWER_S1         0x0C
#define HT7017_REG_EP1              0x0D
#define HT7017_REG_EQ1              0x0E
#define HT7017_REG_EMUSR            0x19

// ─── EMUSR bit positions ──────────────────────────────────────────────────────
#define HT7017_EMUSR_BIT_OV         (1u << 0)
#define HT7017_EMUSR_BIT_UV         (1u << 1)
#define HT7017_EMUSR_BIT_OC         (1u << 2)
#define HT7017_EMUSR_BIT_OP         (1u << 3)
#define HT7017_EMUSR_BIT_ZXLOSS     (1u << 4)

// ─── Calibrated scale factors (10-point calibration 2026-02-23) ──────────────
#define HT7017_DEFAULT_VOLTAGE_SCALE    10961.98f
#define HT7017_DEFAULT_CURRENT_SCALE    2342.02f
#define HT7017_DEFAULT_POWER_SCALE      -3.1777f
// FREQ: freq = HT7017_DEFAULT_FREQ_SCALE / raw  (datasheet §5.1.2.4)
#define HT7017_DEFAULT_FREQ_SCALE       500000.0f
#define HT7017_CURRENT_OFFSET           440.0f

// ─── Derived scale defaults ───────────────────────────────────────────────────
#define HT7017_DEFAULT_REACTIVE_SCALE   HT7017_DEFAULT_POWER_SCALE
#define HT7017_DEFAULT_APPARENT_SCALE   1.0f
#define HT7017_DEFAULT_EP1_SCALE        1.0f

// ─── Two-speed scheduling ─────────────────────────────────────────────────────
// FAST_RATIO=5: 4 fast polls per 5 calls, 1 slow poll per 5 calls.
//
//   FAST table (4 registers: V, I, P, FREQ):
//     Each register polled every  5 × 1 / 4 = 1.25 s
//
//   SLOW table (5 registers: Q, S, EP1, EQ1, EMUSR):
//     Each register polled every  5 × 5     = 25 s
//
#define HT7017_FAST_RATIO           5

// ─── S1 freshness tracking (for HTTP diagnostic only) ────────────────────────
#define HT7017_S1_STALE_SECONDS     90

// ─── Software protection thresholds (0 = disabled) ───────────────────────────
#define HT7017_DEFAULT_OV_THRESHOLD     0.0f    // V
#define HT7017_DEFAULT_UV_THRESHOLD     0.0f    // V
#define HT7017_DEFAULT_OC_THRESHOLD     0.0f    // A
#define HT7017_DEFAULT_OP_THRESHOLD     0.0f    // W

// UV guard — undervoltage only trips if V is above this value.
// Prevents false trips at power-off / startup before line is live.
#define HT7017_UV_MIN_VOLTAGE           10.0f   // V

// Auto-recovery: relay is re-enabled this many seconds after fault clears.
// g_callCount increments once per second so this is a direct second count.
#define HT7017_PROT_RECOVER_SECONDS     30

// Hysteresis bands — FIX 2: actually applied in EvaluateProtection()
//   OV trip at threshold,    clear at threshold - HYS_V
//   UV trip at threshold,    clear at threshold + HYS_V
//   OC trip at threshold,    clear at threshold - HYS_A
//   OP trip at threshold,    clear at threshold - HYS_W
#define HT7017_PROT_HYSTERESIS_V        2.0f    // V
#define HT7017_PROT_HYSTERESIS_A        0.1f    // A
#define HT7017_PROT_HYSTERESIS_W        5.0f    // W

// ─── OpenBeken channel mapping ─────────────────────────────────────────────
//
// *** AUDIT NOTE ***
// The channel numbers below (1-7) are the ACTUAL values used by drv_ht7017.c
// (HT_CH_VOLTAGE=1 … HT_CH_ALARM=7).  A previous header version listed
// channels 10-20 which were NEVER compiled into the .c file and did not match
// the display driver (drv_st7735.c) or application driver (drv_kws303wf.c).
// Those dead defines have been removed to prevent future integration confusion.
//
// ENCODING CONVENTION (all channels, except ALARM):
//   setChannel value = (float_measurement × multiplier) cast to int
//
#define HT7017_CHANNEL_VOLTAGE    1   // V   × 100   e.g. 22150 = 221.50 V
#define HT7017_CHANNEL_CURRENT    2   // A   × 1000  e.g.   550 =   0.550 A
#define HT7017_CHANNEL_POWER      3   // W   × 10    e.g.  1217 = 121.7 W
#define HT7017_CHANNEL_FREQ       4   // Hz  × 100   e.g.  5012 =  50.12 Hz
#define HT7017_CHANNEL_PF         5   // PF  × 1000  e.g.   980 =   0.980
#define HT7017_CHANNEL_WH         6   // Wh  × 10    e.g.  4700 = 470.0 Wh
#define HT7017_CHANNEL_ALARM      7   // 0-4 raw      0=OK 1=OV 2=UV 3=OC 4=OP
//
// AUTOEXEC setChannelType (reference):
//   setChannelType 1  voltage_div100          // Voltage   (V)
//   setChannelType 2  current_div1000         // Current   (A)
//   setChannelType 3  power_div10             // Power     (W)
//   setChannelType 4  ReadOnly                // Frequency (÷100 = Hz)
//   setChannelType 5  PowerFactor_div1000     // Power Factor
//   setChannelType 6  EnergyTotal_kWh_div10000// Active energy (Wh×10, display ÷10000 = kWh)
//   setChannelType 7  ReadOnly                // Alarm state 0-4

// ─── Behaviour ────────────────────────────────────────────────────────────────
#define HT7017_MAX_MISS_COUNT       3

// ─── Default relay channel for software protection ────────────────────────────
#define HT7017_DEFAULT_RELAY_CHANNEL    1

// ─── Public API ───────────────────────────────────────────────────────────────
void     HT7017_Init(void);
void     HT7017_RunQuick(void);
void     HT7017_RunEverySecond(void);

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
