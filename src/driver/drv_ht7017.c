/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║           HT7017 Energy Metering IC — Driver for KWS-303WF  v6            ║
 * ║           Manufacturer: HiTrendtech (钜泉光电科技)                           ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CONFIRMED ARCHITECTURE (KWS-303WF, verified 2026-02-23):                 ║
 * ║                                                                            ║
 * ║    BK7231N CBU ──UART1 4800 8E1──► HT7017 (direct, no intermediate MCU)  ║
 * ║                                                                            ║
 * ║    This is NOT the KWS-302WF architecture which has a PY32F002A MCU       ║
 * ║    between the WiFi module and HT7017. On the 303WF, the BK7231N talks    ║
 * ║    to HT7017 directly via a half-duplex diode circuit on P10/P11.         ║
 * ║                                                                            ║
 * ║    Current sensing: Metal shunt strip (NOT a current transformer).        ║
 * ║    Shunt resistance: ~0.5–1mΩ. Rated for ~16A continuous.                ║
 * ║    Consequence: Noise floor ~450 raw counts. Very small loads (<100W)     ║
 * ║    may be invisible. Subtract offset before computing amps.               ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  KEY LESSONS FROM DEBUGGING (chronological)                               ║
 * ║                                                                            ║
 * ║  1. v1 BUG: ConsumeBytes() called BEFORE read → discarded every response  ║
 * ║     FIX: Always consume AFTER reading bytes, never before.                ║
 * ║                                                                            ║
 * ║  2. v2 BUG: regIndex=0 at init, but first call increments before TX,     ║
 * ║     so first request went to reg 0x06 (Current) not 0x08 (Voltage).      ║
 * ║     FIX: Init regIndex = REG_TABLE_SIZE - 1 so first increment → 0.      ║
 * ║                                                                            ║
 * ║  3. v3 BUG: g_readyToSend handshake flag — RunQuick() is NOT scheduled   ║
 * ║     by OpenBeken for this driver, so flag stuck at 0 → permanent lock.   ║
 * ║     FIX: Remove flag entirely. RunEverySecond is self-contained.          ║
 * ║                                                                            ║
 * ║  4. v5 FIX: FREQ scale 100.0→200.0 (was reading 100Hz, correct is 50Hz) ║
 * ║     Evidence: raw 10000 / 200.0 = 50.000Hz ✓                             ║
 * ║                                                                            ║
 * ║  5. v5 FIX: Power register (0x0A) is SIGNED 24-bit 2's complement.       ║
 * ║     0xFFFFFF shown as 16,777,215W is actually -1 = ~0W (no load).        ║
 * ║     FIX: Sign-extend bit 23 to 32-bit int32_t before dividing.           ║
 * ║                                                                            ║
 * ║  6. v6 NEW: Current offset subtraction (shunt DC bias ~450 counts).      ║
 * ║     Power factor computation from P, V, I readings.                       ║
 * ║     Apparent power (S1 reg 0x0C) added to register rotation.             ║
 * ║     OpenBeken channel publishing integrated.                              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../obk_config.h"
#if ENABLE_DRIVER_HT7017

#include "drv_ht7017.h"
#include "drv_uart.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ─── Register Table Entry ─────────────────────────────────────────────────────
typedef struct {
    uint8_t     reg;            // HT7017 register address
    float      *target;         // pointer to storage variable
    float       scale;          // divide raw by this to get real value
    uint8_t     is_signed;      // 1 = 24-bit 2's complement (P1, Q1)
    uint8_t     has_offset;     // 1 = subtract HT7017_CURRENT_OFFSET first
    const char *name;           // display name for logs
    const char *unit;           // display unit for logs
    uint8_t     channel;        // OpenBeken channel to publish to (0=disabled)
} RegRead_t;

// ─── Measurement Storage ──────────────────────────────────────────────────────
static float g_voltage       = 0.0f;   // V RMS
static float g_current       = 0.0f;   // A RMS  (offset-corrected)
static float g_power         = 0.0f;   // W active (real power)
static float g_freq          = 0.0f;   // Hz
static float g_apparent      = 0.0f;   // VA apparent power (S1)
static float g_power_factor  = 0.0f;   // 0.0–1.0

// ─── Register Rotation Table ──────────────────────────────────────────────────
//
//  Registers polled in order, one per second:
//    0x08 URMS  — Voltage RMS          (unsigned, no offset)
//    0x06 I1RMS — Current RMS          (unsigned, subtract noise floor offset)
//    0x0A P1    — Active power         (SIGNED 24-bit, no offset)
//    0x09 FREQ  — Frequency            (unsigned, no offset)
//    0x0C S1    — Apparent power       (unsigned, no offset)
//
//  One full cycle = 5 seconds. Each measurement updates every 5 seconds.
//  To reduce cycle time, remove registers you don't need.
//
static const RegRead_t g_regTable[] = {
    /*reg                  target         scale                      signed  offset  name        unit  ch */
    { HT7017_REG_RMS_U,    &g_voltage,    HT7017_VOLTAGE_SCALE,      0,      0,     "Voltage",  "V",  HT7017_CHANNEL_VOLTAGE  },
    { HT7017_REG_RMS_I1,   &g_current,    HT7017_CURRENT_SCALE,      0,      1,     "Current",  "A",  HT7017_CHANNEL_CURRENT  },
    { HT7017_REG_POWER_P1, &g_power,      HT7017_POWER_SCALE,        1,      0,     "Power",    "W",  HT7017_CHANNEL_POWER    },
    { HT7017_REG_FREQ,     &g_freq,       HT7017_FREQ_SCALE,         0,      0,     "Freq",     "Hz", HT7017_CHANNEL_FREQ     },
    { HT7017_REG_POWER_S1, &g_apparent,   HT7017_POWER_SCALE,        0,      0,     "Apparent", "VA", 0                       },
};
#define REG_TABLE_SIZE  ((uint8_t)(sizeof(g_regTable) / sizeof(g_regTable[0])))

// ─── Driver State ─────────────────────────────────────────────────────────────
static uint8_t  g_regIndex    = 0;
static uint8_t  g_missCount   = 0;
static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;
static uint32_t g_totalMisses = 0;

// ─── Checksum ─────────────────────────────────────────────────────────────────

static uint8_t HT7017_Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    // Datasheet §4.1.9: sum all frame bytes, discard carry, bitwise NOT
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

// ─── Raw to Real-World Value ──────────────────────────────────────────────────

static float HT7017_Convert(uint32_t raw, const RegRead_t *r)
{
    float working;

    if (r->is_signed) {
        // Sign-extend 24-bit 2's complement to int32
        int32_t s = (int32_t)raw;
        if (s & 0x800000) s |= (int32_t)0xFF000000;
        working = (float)s;
    } else {
        working = (float)raw;
    }

    // Subtract shunt DC offset before scaling current
    if (r->has_offset) {
        working -= HT7017_CURRENT_OFFSET;
        if (working < 0.0f) working = 0.0f; // clamp — can't have negative RMS
    }

    return working / r->scale;
}

// ─── Update Derived Values ────────────────────────────────────────────────────

static void HT7017_UpdateDerived(void)
{
    // Power factor = real power / apparent power
    // Only meaningful when apparent power > 0 and we have a real load
    float apparent = g_voltage * g_current; // V × A = VA
    if (apparent > 0.5f && g_power > 0.0f) {
        g_power_factor = g_power / apparent;
        // Clamp to valid range
        if (g_power_factor > 1.0f) g_power_factor = 1.0f;
        if (g_power_factor < 0.0f) g_power_factor = 0.0f;
    } else {
        g_power_factor = 0.0f;
    }

    // Publish power factor to channel if configured
    if (HT7017_CHANNEL_PF > 0) {
        CHANNEL_Set(HT7017_CHANNEL_PF, (int)(g_power_factor * 100), 0);
    }
}

// ─── Publish to OpenBeken Channel ────────────────────────────────────────────

static void HT7017_PublishChannel(const RegRead_t *r, float value)
{
    if (r->channel == 0) return;
    // Multiply by 100 and publish as integer (e.g. 22051 = 220.51V)
    // Adjust multiplier to suit your channel type configuration in OpenBeken
    CHANNEL_Set(r->channel, (int)(value * 100), 0);
}

// ─── Process a Complete 4-byte Frame ─────────────────────────────────────────

static void HT7017_ProcessFrame(uint8_t d2, uint8_t d1, uint8_t d0, uint8_t cs)
{
    const RegRead_t *r   = &g_regTable[g_regIndex];
    uint8_t          exp = HT7017_Checksum(r->reg, d2, d1, d0);

    if (cs != exp) {
        g_badFrames++;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: BAD CHECKSUM reg=0x%02X got=0x%02X exp=0x%02X "
                  "data=%02X %02X %02X (bad=%u)",
                  r->reg, cs, exp, d2, d1, d0, g_badFrames);
        return;
    }

    uint32_t raw   = ((uint32_t)d2 << 16) | ((uint32_t)d1 << 8) | d0;
    float    value = HT7017_Convert(raw, r);
    *r->target     = value;
    g_goodFrames++;
    g_missCount = 0;

    // Log — show both raw and signed for signed registers so calibration is easy
    if (r->is_signed) {
        int32_t s = (int32_t)raw;
        if (s & 0x800000) s |= (int32_t)0xFF000000;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] = %.3f %s  raw=%u (signed=%d)  CS=OK (good=%u)",
                  r->name, value, r->unit, raw, s, g_goodFrames);
    } else if (r->has_offset) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] = %.3f %s  raw=%u (offset=%u net=%d)  CS=OK (good=%u)",
                  r->name, value, r->unit, raw,
                  (uint32_t)HT7017_CURRENT_OFFSET,
                  (int32_t)(raw - (uint32_t)HT7017_CURRENT_OFFSET),
                  g_goodFrames);
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] = %.3f %s  raw=%u  CS=OK (good=%u)",
                  r->name, value, r->unit, raw, g_goodFrames);
    }

    // Publish to OpenBeken channel
    HT7017_PublishChannel(r, value);

    // Recompute power factor after any measurement update
    HT7017_UpdateDerived();
}

// ─── Send Read Request ────────────────────────────────────────────────────────

static void HT7017_SendRequest(uint8_t reg)
{
    // Flush any stale/partial data before transmitting new request
    UART_ConsumeBytes(UART_GetDataSize());
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ─── Console Command: HT7017_Status ──────────────────────────────────────────

static commandResult_t CMD_Status(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "╔══════ HT7017 Status (KWS-303WF) ══════╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Voltage       : %.2f V", g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Current       : %.3f A  %s",
              g_current,
              HT7017_CURRENT_SCALE == 1.0f ? "[UNCALIBRATED — raw counts]" : "");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Active Power  : %.1f W  %s",
              g_power,
              HT7017_POWER_SCALE == 1.0f ? "[UNCALIBRATED — raw counts]" : "");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Apparent Power: %.1f VA", g_voltage * g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Power Factor  : %.3f", g_power_factor);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Frequency     : %.3f Hz", g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Frames        : good=%u  bad=%u  misses=%u  tx=%u",
              g_goodFrames, g_badFrames, g_totalMisses, g_txCount);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Current reg   : 0x%02X [%s]  regIdx=%u",
              g_regTable[g_regIndex].reg,
              g_regTable[g_regIndex].name, g_regIndex);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Scales        : V=%.1f I=%.1f P=%.1f F=%.1f",
              HT7017_VOLTAGE_SCALE, HT7017_CURRENT_SCALE,
              HT7017_POWER_SCALE, HT7017_FREQ_SCALE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Current offset: %.0f raw counts (shunt DC bias)",
              HT7017_CURRENT_OFFSET);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "╚════════════════════════════════════════╝");
    return CMD_RES_OK;
}

// ─── Console Command: HT7017_Calibrate ───────────────────────────────────────

static commandResult_t CMD_Calibrate(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "╔══════ HT7017 Calibration Guide ══════╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              " Device: KWS-303WF (BK7231N → HT7017 direct, shunt current sensing)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, " ");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              " VOLTAGE — ✓ Calibrated");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Current reading : %.2f V  (scale=%.1f)",
              g_voltage, HT7017_VOLTAGE_SCALE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   To recalibrate  : new_scale = raw / actual_volts");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, " ");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              " FREQUENCY — ✓ Calibrated");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Current reading : %.3f Hz  (scale=%.1f)",
              g_freq, HT7017_FREQ_SCALE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, " ");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              " CURRENT — %s",
              HT7017_CURRENT_SCALE == 1.0f ? "⚠ NOT CALIBRATED" : "✓ Calibrated");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Noise floor     : raw %.0f counts (no load)",
              HT7017_CURRENT_OFFSET);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Current reading : %.3f (scale=%.1f)",
              g_current, HT7017_CURRENT_SCALE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Steps: 1) Plug resistive load (kettle/iron, NOT LED/motor)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "          2) Wait 10s to stabilise");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "          3) Note raw from log: 'HT7017: [Current] raw=XXXXX'");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "          4) Measure amps with clamp meter (or watts/volts)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "          5) CURRENT_SCALE = (raw - %.0f) / actual_amps",
              HT7017_CURRENT_OFFSET);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "          6) Update drv_ht7017.h, recompile");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, " ");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              " POWER — %s",
              HT7017_POWER_SCALE == 1.0f ? "⚠ NOT CALIBRATED" : "✓ Calibrated");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Current reading : %.1f (signed raw, scale=%.1f)",
              g_power, HT7017_POWER_SCALE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   Same load as current calibration. Note signed= value from log.");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   POWER_SCALE = signed_raw / actual_watts");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "   actual_watts = actual_volts × actual_amps  (resistive load)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "╚══════════════════════════════════════╝");
    return CMD_RES_OK;
}

// ─── Console Command: HT7017_Baud ────────────────────────────────────────────

static commandResult_t CMD_Baud(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int baud = atoi(args);
    if (baud <= 0) return CMD_RES_BAD_ARGUMENT;
    UART_InitUART(baud, HT7017_PARITY_EVEN, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud 8E1", baud);
    return CMD_RES_OK;
}

// ─── Console Command: HT7017_NoParity ────────────────────────────────────────

static commandResult_t CMD_NoParity(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    UART_InitUART(HT7017_BAUD_RATE, 0, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud 8N1 (no parity)", HT7017_BAUD_RATE);
    return CMD_RES_OK;
}

// ─── Init ─────────────────────────────────────────────────────────────────────

void HT7017_Init(void)
{
    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("HT7017_Status",    CMD_Status,    NULL);
    CMD_RegisterCommand("HT7017_Calibrate", CMD_Calibrate, NULL);
    CMD_RegisterCommand("HT7017_Baud",      CMD_Baud,      NULL);
    CMD_RegisterCommand("HT7017_NoParity",  CMD_NoParity,  NULL);

    // Init to last index so first increment in RunEverySecond wraps to 0
    // → first request goes to reg 0x08 (Voltage), not 0x06 (Current)
    g_regIndex    = REG_TABLE_SIZE - 1;
    g_missCount   = 0;
    g_totalMisses = 0;
    g_txCount     = 0;
    g_goodFrames  = 0;
    g_badFrames   = 0;

    if (CFG_HasFlag(26)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017 v6: Init OK — UART2 (Pin 6/7), 4800 8E1");
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017 v6: Init OK — UART1 (P10=RX P11=TX), 4800 8E1");
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v6: Device=KWS-303WF  Architecture=BK7231N→HT7017 direct  Sensing=shunt");

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v6: Scales — V=%.1f  I=%.1f%s  P=%.1f%s  F=%.1f",
              HT7017_VOLTAGE_SCALE,
              HT7017_CURRENT_SCALE,
              (HT7017_CURRENT_SCALE == 1.0f) ? "(UNCAL)" : "",
              HT7017_POWER_SCALE,
              (HT7017_POWER_SCALE   == 1.0f) ? "(UNCAL)" : "",
              HT7017_FREQ_SCALE);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v6: Channels — V=%u I=%u P=%u F=%u PF=%u",
              HT7017_CHANNEL_VOLTAGE, HT7017_CHANNEL_CURRENT,
              HT7017_CHANNEL_POWER,   HT7017_CHANNEL_FREQ,
              HT7017_CHANNEL_PF);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v6: %u registers in rotation (one per second, full cycle=%us)",
              REG_TABLE_SIZE, REG_TABLE_SIZE);
}

// ─── RunEverySecond ───────────────────────────────────────────────────────────
//
// Called once per second. Fully self-contained — does not depend on RunQuick.
//
// Each call:
//   1. Check RX buffer for response from last second's request
//   2. Process it (validate checksum, convert, store, publish)
//   3. Advance register index (or retry on miss)
//   4. Send next read request
//
void HT7017_RunEverySecond(void)
{
    int available = UART_GetDataSize();

    if (available >= HT7017_RESPONSE_LEN) {
        // ── Response received ─────────────────────────────────────────────────
        uint8_t d2 = UART_GetByte(0);
        uint8_t d1 = UART_GetByte(1);
        uint8_t d0 = UART_GetByte(2);
        uint8_t cs = UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);     // consume AFTER reading

        HT7017_ProcessFrame(d2, d1, d0, cs);

        // Advance to next register on any response (good or bad checksum)
        g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
        g_missCount = 0;

    } else {
        // ── No response — timeout ─────────────────────────────────────────────
        g_missCount++;
        g_totalMisses++;

        if (g_txCount > 0) {
            // Only log misses after we've sent at least one request
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: No response for reg 0x%02X [%s] "
                      "(miss=%u total_miss=%u good=%u bad=%u)",
                      g_regTable[g_regIndex].reg,
                      g_regTable[g_regIndex].name,
                      g_missCount, g_totalMisses,
                      g_goodFrames, g_badFrames);
        }

        UART_ConsumeBytes(UART_GetDataSize());   // flush any partial garbage

        if (g_missCount >= HT7017_MAX_MISS_COUNT) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: %u misses on reg 0x%02X — skipping to next",
                      HT7017_MAX_MISS_COUNT, g_regTable[g_regIndex].reg);
            g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
            g_missCount = 0;
        }
        // else: stay on same register, retry next second
    }

    // Send read request for the current register
    uint8_t reg = g_regTable[g_regIndex].reg;
    HT7017_SendRequest(reg);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: TX > 6A %02X [%s] (tx=%u)",
              reg, g_regTable[g_regIndex].name, g_txCount);
}

// ─── RunQuick ─────────────────────────────────────────────────────────────────
//
// Called rapidly by scheduler if registered. Optional — RunEverySecond handles
// everything if RunQuick is not called (as is the case in this OpenBeken build).
// If RunQuick IS called, it processes responses faster (within ms vs 1s).
//
void HT7017_RunQuick(void)
{
    if (UART_GetDataSize() < HT7017_RESPONSE_LEN) return;

    uint8_t d2 = UART_GetByte(0);
    uint8_t d1 = UART_GetByte(1);
    uint8_t d0 = UART_GetByte(2);
    uint8_t cs = UART_GetByte(3);
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);     // consume AFTER reading

    HT7017_ProcessFrame(d2, d1, d0, cs);
    // Do NOT advance g_regIndex here — RunEverySecond manages the rotation.
    // RunQuick just processes bytes early; RunEverySecond will find the buffer
    // empty next second and correctly advance to the next register.
}

// ─── HTTP Status Page ─────────────────────────────────────────────────────────

void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[1024];
    float apparent = g_voltage * g_current;

    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 Energy Monitor — KWS-303WF</h5>"
        "<table border='1' cellpadding='5' style='border-collapse:collapse;font-family:monospace'>"
        "<tr><th>Parameter</th><th>Value</th><th>Status</th></tr>"
        "<tr>"
          "<td>Voltage</td>"
          "<td><b>%.2f V</b></td>"
          "<td style='color:green'>&#10003; calibrated</td>"
        "</tr>"
        "<tr>"
          "<td>Current</td>"
          "<td><b>%.3f %s</b></td>"
          "<td style='color:%s'>%s</td>"
        "</tr>"
        "<tr>"
          "<td>Active Power</td>"
          "<td><b>%.1f %s</b></td>"
          "<td style='color:%s'>%s</td>"
        "</tr>"
        "<tr>"
          "<td>Apparent Power</td>"
          "<td><b>%.1f VA</b></td>"
          "<td>V &times; I</td>"
        "</tr>"
        "<tr>"
          "<td>Power Factor</td>"
          "<td><b>%.3f</b></td>"
          "<td>P / (V&times;I)</td>"
        "</tr>"
        "<tr>"
          "<td>Frequency</td>"
          "<td><b>%.3f Hz</b></td>"
          "<td style='color:green'>&#10003; calibrated</td>"
        "</tr>"
        "<tr>"
          "<td colspan='3' style='font-size:0.85em;color:#666'>"
          "Frames: good=%u &nbsp; bad=%u &nbsp; misses=%u &nbsp; tx=%u"
          "</td>"
        "</tr>"
        "</table>",
        g_voltage,
        g_current,
        (HT7017_CURRENT_SCALE == 1.0f) ? "raw" : "A",
        (HT7017_CURRENT_SCALE == 1.0f) ? "orange" : "green",
        (HT7017_CURRENT_SCALE == 1.0f) ? "&#9888; needs calibration" : "&#10003; calibrated",
        g_power,
        (HT7017_POWER_SCALE == 1.0f) ? "raw" : "W",
        (HT7017_POWER_SCALE == 1.0f) ? "orange" : "green",
        (HT7017_POWER_SCALE == 1.0f) ? "&#9888; needs calibration" : "&#10003; calibrated",
        apparent,
        g_power_factor,
        g_freq,
        g_goodFrames, g_badFrames, g_totalMisses, g_txCount);

    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

// ─── Getters ──────────────────────────────────────────────────────────────────
float    HT7017_GetVoltage(void)        { return g_voltage;      }
float    HT7017_GetCurrent(void)        { return g_current;      }
float    HT7017_GetPower(void)          { return g_power;        }
float    HT7017_GetFrequency(void)      { return g_freq;         }
float    HT7017_GetPowerFactor(void)    { return g_power_factor; }
float    HT7017_GetApparentPower(void)  { return g_apparent;     }
uint32_t HT7017_GetGoodFrames(void)     { return g_goodFrames;   }
uint32_t HT7017_GetBadFrames(void)      { return g_badFrames;    }
uint32_t HT7017_GetTxCount(void)        { return g_txCount;      }
uint32_t HT7017_GetMissCount(void)      { return g_totalMisses;  }

#endif // ENABLE_DRIVER_HT7017
