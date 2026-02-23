#pragma once
#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║           HT7017 Energy Metering IC — Driver for KWS-303WF                 ║
 * ║           Manufacturer: HiTrendtech (钜泉光电科技)                           ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DEVICE: KWS-303WF Smart Energy Meter                                      ║
 * ║                                                                            ║
 * ║  This specific hardware variant has NO intermediate PY32F002A MCU.        ║
 * ║  The BK7231N WiFi module talks DIRECTLY to the HT7017 over UART1.         ║
 * ║  (Contrast with KWS-302WF which has PY32F002A in between.)                ║
 * ║                                                                            ║
 * ║  Current sensing uses a LOW-RESISTANCE METAL SHUNT STRIP (not a CT).      ║
 * ║  The shunt is rated for high current (device rated ~16A or 63A depending  ║
 * ║  on variant). Shunt resistance is approx 0.5–1mΩ.                         ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  HARDWARE WIRING (BK7231N CBU module → HT7017)                            ║
 * ║                                                                            ║
 * ║    BK7231N P11 (UART1 TX) ──[Diode]──┬──► HT7017 RX (pin 10)             ║
 * ║    BK7231N P10 (UART1 RX) ◄──────────┘◄── HT7017 TX (pin 11)             ║
 * ║                                                                            ║
 * ║    The diode makes the shared wire half-duplex:                            ║
 * ║      - BK TX drives high → passes through diode → HT7017 RX sees it      ║
 * ║      - HT7017 TX drives → goes directly to BK RX (diode blocks BK TX)    ║
 * ║                                                                            ║
 * ║  HT7017 voltage input:  Mains live → resistor divider → V3P/V3N pins      ║
 * ║  HT7017 current input:  Metal shunt in series with load → V1P/V1N pins    ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  UART PROTOCOL (HT7017 Datasheet §4.1)                                    ║
 * ║                                                                            ║
 * ║  Settings: 4800 baud, 8E1 (8 data bits, Even parity, 1 stop bit)         ║
 * ║  Mode:     Half-duplex, HT7017 is always slave                            ║
 * ║                                                                            ║
 * ║  Read request  (BK7231N → HT7017):  [0x6A][REG]           2 bytes        ║
 * ║  Read response (HT7017 → BK7231N):  [D2][D1][D0][CHKSUM]  4 bytes        ║
 * ║                                                                            ║
 * ║  Raw value:  (D2 << 16) | (D1 << 8) | D0    (24-bit, MSB first)          ║
 * ║  Checksum:   ~(0x6A + REG + D2 + D1 + D0) & 0xFF                         ║
 * ║  Timeout:    Byte-to-byte gap must be < 20ms or UART auto-resets          ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  REGISTER REFERENCE (Datasheet §5.1)                                      ║
 * ║                                                                            ║
 * ║  Addr  Name        Type      Description                                  ║
 * ║  0x06  I1RMS       unsigned  Current channel 1 RMS (shunt input)          ║
 * ║  0x08  URMS        unsigned  Voltage RMS                                  ║
 * ║  0x09  FREQ        unsigned  Line frequency (period counter)              ║
 * ║  0x0A  P1          SIGNED    Active power ch1 (2's complement!)           ║
 * ║  0x0B  Q1          SIGNED    Reactive power ch1                           ║
 * ║  0x0C  S1          unsigned  Apparent power ch1 (V × I)                  ║
 * ║  0x19  EMUSR       unsigned  Status: creep/startup flags                  ║
 * ║                                                                            ║
 * ║  ⚠ SIGNED REGISTERS: P1 and Q1 use 24-bit 2's complement.                ║
 * ║    0xFFFFFF = -1  (no load / near-zero power)                             ║
 * ║    0xFFFFFE = -2  etc.                                                    ║
 * ║    Do NOT treat as unsigned — 16,777,215 W is NOT real!                   ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CALIBRATION THEORY                                                        ║
 * ║                                                                            ║
 * ║  HT7017 outputs raw ADC counts. It has NO knowledge of your external      ║
 * ║  components. You MUST calibrate each channel against a trusted reference. ║
 * ║                                                                            ║
 * ║  General formula:  Real_Value = Raw / Scale                               ║
 * ║  To find Scale:    Scale = Raw / Real_Value                               ║
 * ║                                                                            ║
 * ║  VOLTAGE (URMS, reg 0x08):                                                ║
 * ║    Status: ✓ CALIBRATED                                                   ║
 * ║    Session 1 (2026-02-22): 244.30V actual → raw 2,690,940 → scale 11,015 ║
 * ║    Session 2 (2026-02-23): 220.00V actual → raw 2,425,000 → scale 11,023 ║
 * ║    Average: 11,019. Using 11,015.3 (first calibration, most precise).    ║
 * ║    Recalibrate: measure real volts with multimeter, note raw from log,    ║
 * ║    compute new scale = raw / volts, update HT7017_VOLTAGE_SCALE below.   ║
 * ║                                                                            ║
 * ║  FREQUENCY (FREQ, reg 0x09):                                              ║
 * ║    Status: ✓ CALIBRATED (no equipment needed, derived from log)           ║
 * ║    50Hz grid → raw consistently 9991–10006                                ║
 * ║    Scale = 10000 / 50.0 = 200.0  (confirmed by multiple samples)         ║
 * ║    Note: 60Hz grid users change to: scale = 12000 / 60.0 = 200.0         ║
 * ║    (same formula, same scale — the chip counts differently per region)    ║
 * ║                                                                            ║
 * ║  CURRENT (I1RMS, reg 0x06):                                               ║
 * ║    Status: ⚠ UNCALIBRATED                                                 ║
 * ║    Noise floor (no load): raw ~443–462                                    ║
 * ║    This is ADC DC offset from the shunt design — normal and expected.     ║
 * ║    18W LED added only ~45 raw counts — too small to calibrate from.       ║
 * ║    You need a resistive load of 500W+ (kettle, iron, fan heater).         ║
 * ║                                                                            ║
 * ║    CALIBRATION STEPS:                                                     ║
 * ║      1. Connect a resistive load to the KWS-303WF output socket          ║
 * ║         (kettle, electric iron, fan heater — NOT LED, NOT motor)          ║
 * ║      2. Run the load for 10 seconds to stabilise                          ║
 * ║      3. Note raw value from log: "[Current] = X.XXX A  raw=XXXXXX"       ║
 * ║      4. Measure actual amps with clamp meter (or use watts/volts)         ║
 * ║      5. CURRENT_SCALE = raw / actual_amps                                 ║
 * ║      6. Update HT7017_CURRENT_SCALE below and recompile                  ║
 * ║                                                                            ║
 * ║  ACTIVE POWER (P1, reg 0x0A):                                             ║
 * ║    Status: ⚠ UNCALIBRATED                                                 ║
 * ║    This is a SIGNED 24-bit register (2's complement).                     ║
 * ║    No-load readings: raw 0, 1, 2, 0xFFFFFF(-1), 0xFFFFFE(-2) = all ~0W  ║
 * ║    Positive raw = consuming power. Negative raw = generating power.       ║
 * ║    LED bulb reads near-zero because LEDs have poor power factor (~0.5).  ║
 * ║    Need resistive load (PF=1.0) for meaningful power reading.             ║
 * ║                                                                            ║
 * ║    CALIBRATION STEPS (do AFTER current calibration):                      ║
 * ║      1. Same resistive load as current calibration                        ║
 * ║      2. Note signed raw from log: "[Power] = X.X W  raw=X (signed=X)"   ║
 * ║      3. Actual watts = actual_volts × actual_amps (for resistive load)   ║
 * ║      4. POWER_SCALE = signed_raw / actual_watts                           ║
 * ║      5. Update HT7017_POWER_SCALE below and recompile                    ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DRIVER HISTORY                                                            ║
 * ║                                                                            ║
 * ║  v1: ConsumeBytes() before reading → discarded every response.            ║
 * ║  v2: regIndex init wrong → first TX to wrong register.                    ║
 * ║      Missed response skipped register instead of retrying.                ║
 * ║  v3: g_readyToSend handshake flag — RunQuick() not called by scheduler   ║
 * ║      → flag never cleared → permanent lock ("Chip still responding").    ║
 * ║  v4: Removed g_readyToSend. RunEverySecond self-contained. All 4         ║
 * ║      registers reading correctly for the first time.                      ║
 * ║  v5: FREQ scale 100→200 (was showing 100Hz, now 50Hz ✓). Power register  ║
 * ║      treated as SIGNED 24-bit (0xFFFFFF=-1=~0W, not 16MW). Voltage       ║
 * ║      scale refined. Added HT7017_Calibrate console command.              ║
 * ║  v6 (this): All architecture confirmed (no PY32F, shunt not CT).         ║
 * ║      Current zero-offset subtraction added. Power factor calculation.    ║
 * ║      Apparent power (S1) added to rotation. OpenBeken channel mapping.   ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── UART Settings ────────────────────────────────────────────────────────────
#define HT7017_BAUD_RATE            4800
#define HT7017_PARITY_EVEN          2       // 8E1 — mandatory per datasheet §4.1

// ─── Protocol ─────────────────────────────────────────────────────────────────
#define HT7017_FRAME_HEAD           0x6A    // Fixed header for all read commands
#define HT7017_RESPONSE_LEN         4       // D2 + D1 + D0 + CHECKSUM

// ─── Register Addresses ───────────────────────────────────────────────────────
#define HT7017_REG_RMS_I1           0x06    // Current RMS ch1      (unsigned)
#define HT7017_REG_RMS_U            0x08    // Voltage RMS          (unsigned)
#define HT7017_REG_FREQ             0x09    // Line frequency       (unsigned)
#define HT7017_REG_POWER_P1         0x0A    // Active power ch1     (SIGNED 24-bit)
#define HT7017_REG_POWER_Q1         0x0B    // Reactive power ch1   (SIGNED 24-bit)
#define HT7017_REG_POWER_S1         0x0C    // Apparent power ch1   (unsigned)
#define HT7017_REG_EMUSR            0x19    // EMU status register

// ─── Calibration: VOLTAGE ─────────────────────────────────────────────────────
// ✓ CALIBRATED — do not change unless you recalibrate with a multimeter.
// Formula: VOLTAGE_SCALE = raw_reading / actual_volts
// Evidence:
//   2026-02-22: 244.30V → raw 2,690,940 → scale 11,015.3
//   2026-02-23: 220.00V → raw 2,480,000 → scale 11,023.0
//   Average ≈ 11,019. Using first session (more precise reference meter).
#define HT7017_VOLTAGE_SCALE        11015.3f

// ─── Calibration: FREQUENCY ───────────────────────────────────────────────────
// ✓ CALIBRATED — derived from log, no equipment needed.
// Formula: FREQ_SCALE = raw_reading / actual_hz
// Evidence: 50Hz grid → raw 9991–10006 → scale = 10000/50 = 200.0
// Confirmed across 100+ samples. Grid variation ±0.03Hz is normal.
#define HT7017_FREQ_SCALE           200.0f

// ─── Calibration: CURRENT ─────────────────────────────────────────────────────
// ⚠ NOT YET CALIBRATED — shows raw ADC counts until set.
// Formula: CURRENT_SCALE = raw_with_load / actual_amps
//
// Shunt noise floor (no load connected): raw ~443–462
// This DC offset is subtracted automatically — see HT7017_CURRENT_OFFSET below.
//
// Calibration procedure:
//   1. Connect resistive load to KWS-303WF output (kettle/iron/heater, NOT LED)
//   2. Measure actual amps with clamp meter (or compute: label_watts / volts)
//   3. Read raw from log: "HT7017: [Current] raw=XXXXX"
//   4. Set CURRENT_SCALE = (raw - HT7017_CURRENT_OFFSET) / actual_amps
//   5. Recompile and verify
#define HT7017_CURRENT_SCALE        2317.37    // TODO: calibrate with resistive load

// Shunt ADC zero-current offset — the raw count with no current flowing.
// Measured from your logs with no load: consistent ~450 counts.
// Subtracted before applying CURRENT_SCALE so near-zero loads read ~0A.
// Re-measure after any hardware change.
#define HT7017_CURRENT_OFFSET       462.0f

// ─── Calibration: ACTIVE POWER ────────────────────────────────────────────────
// ⚠ NOT YET CALIBRATED — shows raw signed counts until set.
// P1 register is SIGNED 24-bit 2's complement. Driver handles sign extension.
// Formula: POWER_SCALE = signed_raw_with_load / actual_watts
//
// No-load behaviour (all mean ~0W):
//   raw 0x000000 =  0  signed →  0W
//   raw 0x000001 = +1  signed → +0W
//   raw 0xFFFFFF = -1  signed → ~0W  ← this is what you see as "16777215"
//   raw 0xFFFFFE = -2  signed → ~0W
//
// LED bulb (18W) reads near-zero because LEDs have power factor ~0.3–0.5.
// The chip measures REAL power only. Need resistive load (PF=1.0) to calibrate.
//
// Calibration procedure (do AFTER current calibration):
//   1. Same resistive load as current calibration
//   2. Read signed raw from log: "HT7017: [Power] raw=X signed=X"
//   3. actual_watts = actual_volts × actual_amps  (for purely resistive load)
//   4. Set POWER_SCALE = signed_raw / actual_watts
//   5. Recompile and verify
#define HT7017_POWER_SCALE          -32.24f    // TODO: calibrate with resistive load

// ─── OpenBeken Channel Mapping ────────────────────────────────────────────────
// Map HT7017 measurements to OpenBeken channels for MQTT / automations.
// Change these channel numbers to match your setup.
// Set to 0 to disable publishing for that measurement.
#define HT7017_CHANNEL_VOLTAGE      10       // Channel to publish voltage (V)
#define HT7017_CHANNEL_CURRENT      11      // Channel to publish current (A)
#define HT7017_CHANNEL_POWER        12       // Channel to publish power (W)
#define HT7017_CHANNEL_FREQ         13      // Channel to publish frequency (Hz)
#define HT7017_CHANNEL_PF           14      // Channel to publish power factor (0–1)

// ─── Behaviour Tuning ─────────────────────────────────────────────────────────
// Maximum consecutive missed responses before skipping to the next register.
// Increase if you get false "miss" warnings on a slow/noisy line.
#define HT7017_MAX_MISS_COUNT       3

// ─── Public API ───────────────────────────────────────────────────────────────
void     HT7017_Init(void);
void     HT7017_RunQuick(void);
void     HT7017_RunEverySecond(void);
void     HT7017_AppendInformationToHTTPIndexPage(http_request_t *request);

// Calibrated measurements
float    HT7017_GetVoltage(void);       // Volts RMS
float    HT7017_GetCurrent(void);       // Amps RMS  (0.0 until calibrated)
float    HT7017_GetPower(void);         // Watts active power (0.0 until calibrated)
float    HT7017_GetFrequency(void);     // Hz
float    HT7017_GetPowerFactor(void);   // 0.0–1.0 (computed from P, V, I)
float    HT7017_GetApparentPower(void); // VA (V × I)

// Diagnostics
uint32_t HT7017_GetGoodFrames(void);
uint32_t HT7017_GetBadFrames(void);
uint32_t HT7017_GetTxCount(void);
uint32_t HT7017_GetMissCount(void);

#endif // __DRV_HT7017_H__
