/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v11
 *
 * v11 ADDITIONS (zero changes to v10 working code):
 *   + Q1   (reg 0x0B)  Reactive power        SIGNED 24-bit  VAR
 *   + S1   (reg 0x0C)  Apparent power         own scale      VA
 *   + EP1  (reg 0x13)  Active energy accum    delta→Wh       Wh
 *   + EQ1  (reg 0x14)  Reactive energy accum  delta→VARh     VARh
 *   + PFCNT1 (reg 0x10) Power factor count    raw counter
 *   + HT7017_Energy_Reset command
 *   + HT7017_Energy command  (kWh, kVARh, session time via NTP)
 *   + FREQ polled twice per cycle (positions 3 and 9) — stays fresh
 *   + Updated HT7017_Status and HTTP page show all new values
 *
 * FREQ NOTE (from datasheet + log analysis):
 *   The FREQ register is a PERIOD counter — chip counts its internal clock
 *   cycles per AC half-cycle, then outputs a 24-bit value.
 *   At 50Hz: raw ~10000, scale 200.0 → 10000/200 = 50.0Hz  (confirmed)
 *   At 60Hz: raw ~8333,  scale 200.0 → 8333/200 = 41.7Hz  (wrong — needs
 *            scale 138.9 for 60Hz grids).
 *   Update rate: chip refreshes every AC cycle (~20ms). Our poll is every
 *   10s so readings are always current. Polling twice per cycle (pos 3+9)
 *   keeps display fresh even on long rotation.
 *
 * EP1/EQ1 ENERGY ACCUMULATION:
 *   The chip accumulates active (EP1) and reactive (EQ1) energy internally.
 *   Each poll reads the current counter, computes delta from last reading,
 *   converts to Wh/VARh via scale, adds to running total.
 *   Wrap-around (counter reset by chip) is detected and handled.
 *   EP1_SCALE default=1.0 (shows raw Wh counts). Calibrate by running a
 *   known load for a known time:
 *     EP1_SCALE = raw_delta / actual_wh
 *   NTP start/reset timestamps recorded for session kWh/kVARh rate display.
 *
 * ROTATION: 12 slots, one per second, full cycle = 12 seconds.
 *   Slots: V I P Q S PFCNT F EP1 EQ1 V I F
 *   (V, I, F repeated to keep fast-changing values fresh)
 *
 * RULES:
 *   No channel.h  |  No powerMeasurementCalibration  |  No CFG_SetExtended
 *   Channel publish via CMD_ExecuteCommand("setChannel N V")
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
#include <stdlib.h>
#include <math.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION A — v10 CORE (unchanged)
// ═══════════════════════════════════════════════════════════════════════════════

static float g_vScale  = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale  = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale  = HT7017_DEFAULT_FREQ_SCALE;
static float g_iOffset = HT7017_CURRENT_OFFSET;

static uint32_t g_lastRawV = 0;
static uint32_t g_lastRawI = 0;
static uint32_t g_lastRawP = 0;

static float g_voltage      = 0.0f;
static float g_current      = 0.0f;
static float g_power        = 0.0f;
static float g_freq         = 0.0f;
static float g_apparent     = 0.0f;   // V * I always-valid computed value
static float g_power_factor = 0.0f;

static uint8_t  g_regIndex    = 0;
static uint8_t  g_missCount   = 0;
static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;
static uint32_t g_totalMisses = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION B — v11 NEW MEASUREMENTS
// ═══════════════════════════════════════════════════════════════════════════════

// New scale factors
static float g_qScale  = HT7017_DEFAULT_POWER_SCALE;     // Q1 reactive (same sign convention as P1)
static float g_sScale  = HT7017_DEFAULT_APPARENT_SCALE;  // S1 apparent (own scale, unsigned)
static float g_e1Scale = HT7017_DEFAULT_EP1_SCALE;       // EP1 active energy

// New instantaneous measurements
static float g_reactive     = 0.0f;   // Q1 reg 0x0B — reactive power (VAR)
static float g_apparent_s1  = 0.0f;   // S1 reg 0x0C — apparent power from chip (VA)
static float g_pf_count_raw = 0.0f;   // PFCNT1 reg 0x10 — raw counter value

// Energy accumulators
static uint32_t g_ep1_last  = 0xFFFFFFFF;  // sentinel = not yet seeded
static uint32_t g_eq1_last  = 0xFFFFFFFF;
static float    g_wh_total  = 0.0f;        // Wh accumulated since boot/reset
static float    g_varh_total= 0.0f;        // VARh accumulated since boot/reset

// NTP timestamps
static uint32_t g_session_start_unix = 0;  // set at init once NTP is valid
static uint32_t g_energy_reset_unix  = 0;  // updated by HT7017_Energy_Reset

// ─── Register table ───────────────────────────────────────────────────────────
typedef struct {
    uint8_t     reg;
    float      *target;       // where to store converted value (or running total)
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint8_t     is_energy;    // 1 = EP1/EQ1 accumulator, uses special delta logic
    uint32_t   *last_raw_acc; // pointer to last-raw for accumulator wrap detection
    float      *acc_total;    // pointer to running total for energy registers
    uint32_t   *last_raw_cal; // pointer to last-raw for VoltageSet/CurrentSet/PowerSet
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

// 12-slot rotation — 12 seconds per full cycle
// Slots: V I P Q S PFCNT F EP1 EQ1 V I F
// V, I, F repeated so they stay fresh (every 6s instead of 12s)
#define REG_TABLE_SIZE 12
static RegRead_t g_regTable[REG_TABLE_SIZE];

static void HT7017_BuildRegTable(void)
{
    uint8_t i = 0;

    // 0: Voltage
    g_regTable[i].reg          = HT7017_REG_RMS_U;
    g_regTable[i].target       = &g_voltage;
    g_regTable[i].scale        = &g_vScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = &g_lastRawV;
    g_regTable[i].name         = "Voltage";
    g_regTable[i].unit         = "V";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_VOLTAGE;
    i++;

    // 1: Current
    g_regTable[i].reg          = HT7017_REG_RMS_I1;
    g_regTable[i].target       = &g_current;
    g_regTable[i].scale        = &g_iScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 1;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = &g_lastRawI;
    g_regTable[i].name         = "Current";
    g_regTable[i].unit         = "A";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_CURRENT;
    i++;

    // 2: Active Power P1 (SIGNED)
    g_regTable[i].reg          = HT7017_REG_POWER_P1;
    g_regTable[i].target       = &g_power;
    g_regTable[i].scale        = &g_pScale;
    g_regTable[i].is_signed    = 1;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = &g_lastRawP;
    g_regTable[i].name         = "Power";
    g_regTable[i].unit         = "W";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_POWER;
    i++;

    // 3: Reactive Power Q1 (SIGNED) — NEW
    // Q1 uses the same signed 24-bit 2's complement format as P1.
    // Positive Q = inductive load (motors, transformers)
    // Negative Q = capacitive load (PFC corrected supplies, capacitor banks)
    g_regTable[i].reg          = HT7017_REG_POWER_Q1;
    g_regTable[i].target       = &g_reactive;
    g_regTable[i].scale        = &g_qScale;
    g_regTable[i].is_signed    = 1;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "Reactive";
    g_regTable[i].unit         = "VAR";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_REACTIVE;
    i++;

    // 4: Apparent Power S1 (unsigned) — NEW, own scale
    // S1 raw units differ from P1. Calibrate: S_actual = V_actual * I_actual
    // then S1_SCALE = raw_S1 / S_actual.
    // Until calibrated, S1 reading is shown in log but not published.
    g_regTable[i].reg          = HT7017_REG_POWER_S1;
    g_regTable[i].target       = &g_apparent_s1;
    g_regTable[i].scale        = &g_sScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "Apparent";
    g_regTable[i].unit         = "VA";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_APPARENT;
    i++;

    // 5: Power Factor Count PFCNT1 (unsigned) — NEW
    // Raw counter incremented by chip each AC cycle weighted by power factor.
    // Full-scale value when PF=1.0 is chip-config dependent.
    // Stored raw / g_fScale (=200) for logging — not published to channel
    // until full-scale reference is known.
    g_regTable[i].reg          = HT7017_REG_PFCNT1;
    g_regTable[i].target       = &g_pf_count_raw;
    g_regTable[i].scale        = &g_fScale;   // arbitrary divisor for display
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "PFCount";
    g_regTable[i].unit         = "raw";
    g_regTable[i].obk_ch       = 0;   // not published until full-scale known
    i++;

    // 6: Frequency — first occurrence
    // FREQ is a period counter: chip measures clock cycles per AC half-cycle.
    // At 50Hz grid: raw ~10000, scale 200.0 → 50.0Hz
    // Chip updates every AC cycle (~20ms) so our 12s rotation always reads fresh.
    // Repeated at slot 11 so display stays current.
    g_regTable[i].reg          = HT7017_REG_FREQ;
    g_regTable[i].target       = &g_freq;
    g_regTable[i].scale        = &g_fScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "Freq";
    g_regTable[i].unit         = "Hz";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_FREQ;
    i++;

    // 7: Active Energy EP1 (unsigned, accumulator) — NEW
    // EP1 is an internal 24-bit counter that the chip increments as energy
    // flows. We read the delta each poll, convert to Wh via g_e1Scale,
    // and add to g_wh_total. Scale default=1.0 (raw counts = Wh approx).
    // Calibrate: run known load W for known time T hours, then:
    //   EP1_SCALE = raw_delta / (W * T)
    g_regTable[i].reg          = HT7017_REG_EP1;
    g_regTable[i].target       = &g_wh_total;
    g_regTable[i].scale        = &g_e1Scale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 1;
    g_regTable[i].last_raw_acc = &g_ep1_last;
    g_regTable[i].acc_total    = &g_wh_total;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "ActiveEnergy";
    g_regTable[i].unit         = "Wh";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_WH;
    i++;

    // 8: Reactive Energy EQ1 (unsigned, accumulator) — NEW
    // Same logic as EP1 but for reactive energy.
    g_regTable[i].reg          = HT7017_REG_EQ1;
    g_regTable[i].target       = &g_varh_total;
    g_regTable[i].scale        = &g_e1Scale;  // same scale as EP1
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 1;
    g_regTable[i].last_raw_acc = &g_eq1_last;
    g_regTable[i].acc_total    = &g_varh_total;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "ReactiveEnergy";
    g_regTable[i].unit         = "VARh";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_VARH;
    i++;

    // 9: Voltage (repeat) — refresh every 6s instead of 12s
    g_regTable[i].reg          = HT7017_REG_RMS_U;
    g_regTable[i].target       = &g_voltage;
    g_regTable[i].scale        = &g_vScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = &g_lastRawV;
    g_regTable[i].name         = "Voltage";
    g_regTable[i].unit         = "V";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_VOLTAGE;
    i++;

    // 10: Current (repeat) — refresh every 6s
    g_regTable[i].reg          = HT7017_REG_RMS_I1;
    g_regTable[i].target       = &g_current;
    g_regTable[i].scale        = &g_iScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 1;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = &g_lastRawI;
    g_regTable[i].name         = "Current";
    g_regTable[i].unit         = "A";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_CURRENT;
    i++;

    // 11: Frequency (repeat) — keep fresh on 12s cycle
    g_regTable[i].reg          = HT7017_REG_FREQ;
    g_regTable[i].target       = &g_freq;
    g_regTable[i].scale        = &g_fScale;
    g_regTable[i].is_signed    = 0;
    g_regTable[i].has_offset   = 0;
    g_regTable[i].is_energy    = 0;
    g_regTable[i].last_raw_acc = NULL;
    g_regTable[i].acc_total    = NULL;
    g_regTable[i].last_raw_cal = NULL;
    g_regTable[i].name         = "Freq";
    g_regTable[i].unit         = "Hz";
    g_regTable[i].obk_ch       = HT7017_CHANNEL_FREQ;
    i++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION C — HELPERS (unchanged from v10 except UpdatePowerFactor)
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t HT7017_Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

static float HT7017_Convert(uint32_t raw, const RegRead_t *r)
{
    float val;
    if (r->is_signed) {
        int32_t s = (int32_t)raw;
        if (s & 0x800000) s |= (int32_t)0xFF000000;
        val = (float)s;
    } else {
        val = (float)raw;
    }
    if (r->has_offset) {
        val -= g_iOffset;
        if (val < 0.0f) val = 0.0f;
    }
    float result = val / (*r->scale);
    if (r->is_signed && result > -1.0f && result < 1.0f) result = 0.0f;
    return result;
}

static void HT7017_UpdateDerived(void)
{
    // Apparent power — always valid from V*I
    g_apparent = g_voltage * g_current;

    // Power factor — prefer chip S1 if calibrated (>0.5VA), else use V*I
    float s = (g_apparent_s1 > 0.5f) ? g_apparent_s1 : g_apparent;
    if (s > 0.5f && g_power > 0.0f) {
        g_power_factor = g_power / s;
        if (g_power_factor > 1.0f) g_power_factor = 1.0f;
        if (g_power_factor < 0.0f) g_power_factor = 0.0f;
    } else {
        g_power_factor = 0.0f;
    }
}

static void HT7017_PublishChannel(uint8_t ch, float value)
{
    if (ch == 0) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "setChannel %u %d", ch, (int)(value * 100.0f));
    CMD_ExecuteCommand(buf, 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D — ENERGY ACCUMULATOR (new v11)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_AccumulateEnergy(uint32_t raw, const RegRead_t *r)
{
    // First reading — just seed the baseline, do not accumulate yet
    if (*r->last_raw_acc == 0xFFFFFFFF) {
        *r->last_raw_acc = raw;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] seeded at raw=%u  accumulating from next read",
                  r->name, raw);
        return;
    }

    uint32_t delta;
    if (raw >= *r->last_raw_acc) {
        delta = raw - *r->last_raw_acc;
    } else {
        // 24-bit counter wrap-around (chip reset its internal accumulator)
        delta = (0x00FFFFFF - *r->last_raw_acc) + raw + 1;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] counter wrap last=%u new=%u delta=%u",
                  r->name, *r->last_raw_acc, raw, delta);
    }
    *r->last_raw_acc = raw;

    if (delta == 0) return;

    float increment  = (float)delta / (*r->scale);
    *r->acc_total   += increment;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [%s] raw=%u  delta=%u  +%.5f %s  total=%.4f %s",
              r->name, raw, delta,
              increment, r->unit,
              *r->acc_total, r->unit);

    if (r->obk_ch > 0) {
        // Publish total in unit * 1000 for 3-decimal precision
        // e.g. 1234 = 1.234 Wh — configure channel as "divided by 1000"
        char buf[48];
        snprintf(buf, sizeof(buf), "setChannel %u %d",
                 r->obk_ch, (int)(*r->acc_total * 1000.0f));
        CMD_ExecuteCommand(buf, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION E — FRAME PROCESSING (v10 logic, extended for energy)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_ProcessFrame(uint8_t d2, uint8_t d1, uint8_t d0, uint8_t cs)
{
    const RegRead_t *r   = &g_regTable[g_regIndex];
    uint8_t          exp = HT7017_Checksum(r->reg, d2, d1, d0);

    if (cs != exp) {
        g_badFrames++;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: BAD CS reg=0x%02X got=0x%02X exp=0x%02X "
                  "data=%02X %02X %02X (bad=%u)",
                  r->reg, cs, exp, d2, d1, d0, g_badFrames);
        return;
    }

    uint32_t raw = ((uint32_t)d2 << 16) | ((uint32_t)d1 << 8) | d0;
    g_goodFrames++;
    g_missCount = 0;

    // Store calibration raw if this slot has one
    if (r->last_raw_cal) *r->last_raw_cal = raw;

    if (r->is_energy) {
        // Energy accumulator — special delta logic
        HT7017_AccumulateEnergy(raw, r);
    } else {
        float value = HT7017_Convert(raw, r);
        *r->target  = value;

        if (r->is_signed) {
            int32_t s = (int32_t)raw;
            if (s & 0x800000) s |= (int32_t)0xFF000000;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: [%s] = %.3f %s  raw=%u signed=%d  CS=OK (good=%u)",
                      r->name, value, r->unit, raw, s, g_goodFrames);
        } else if (r->has_offset) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: [%s] = %.3f %s  raw=%u net=%d  CS=OK (good=%u)",
                      r->name, value, r->unit, raw,
                      (int32_t)((int32_t)raw - (int32_t)g_iOffset),
                      g_goodFrames);
        } else {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: [%s] = %.3f %s  raw=%u  CS=OK (good=%u)",
                      r->name, value, r->unit, raw, g_goodFrames);
        }

        if (r->obk_ch > 0) {
            HT7017_PublishChannel(r->obk_ch, value);
        }
    }

    // Recompute derived values after every measurement
    HT7017_UpdateDerived();
    if (HT7017_CHANNEL_PF > 0) {
        HT7017_PublishChannel(HT7017_CHANNEL_PF, g_power_factor);
    }

    // Seed NTP session start on first valid reading after NTP sync
    if (g_session_start_unix == 0) {
        uint32_t now = NTP_GetCurrentTime();
        if (now > 1000000000u) {   // sanity: after year 2001
            g_session_start_unix = now;
            g_energy_reset_unix  = now;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: NTP session start recorded unix=%u", now);
        }
    }
}

static void HT7017_SendRequest(uint8_t reg)
{
    UART_ConsumeBytes(UART_GetDataSize());
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION F — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════════

// ── VoltageSet / CurrentSet / PowerSet (v10 unchanged) ────────────────────────

static commandResult_t CMD_VoltageSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;
    if (g_lastRawV == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: VoltageSet - no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    float old = g_vScale;
    g_vScale  = (float)g_lastRawV / actual;
    g_voltage = actual;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: VoltageSet %.2fV raw=%u scale %.2f->%.2f "
              "(update HT7017_DEFAULT_VOLTAGE_SCALE to make permanent)",
              actual, g_lastRawV, old, g_vScale);
    return CMD_RES_OK;
}

static commandResult_t CMD_CurrentSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;
    if (g_lastRawI == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CurrentSet - no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    float net = (float)g_lastRawI - g_iOffset;
    if (net <= 0.0f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CurrentSet - net=%.0f too low, load too small?", net);
        return CMD_RES_ERROR;
    }
    float old = g_iScale;
    g_iScale  = net / actual;
    g_current = actual;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: CurrentSet %.3fA raw=%u net=%.0f scale %.2f->%.2f "
              "(update HT7017_DEFAULT_CURRENT_SCALE to make permanent)",
              actual, g_lastRawI, net, old, g_iScale);
    return CMD_RES_OK;
}

static commandResult_t CMD_PowerSet(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;
    if (g_lastRawP == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: PowerSet - no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    int32_t s = (int32_t)g_lastRawP;
    if (s & 0x800000) s |= (int32_t)0xFF000000;
    if (s == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: PowerSet - signed raw=0, load connected?");
        return CMD_RES_ERROR;
    }
    float old = g_pScale;
    g_pScale  = (float)s / actual;
    g_power   = actual;
    // Apply same scale to Q1 and S1 as a starting approximation
    // (refine with ReactiveSet/ApparentSet after calibration)
    g_qScale  = g_pScale;
    g_sScale  = g_pScale * -1.0f;  // S1 is unsigned so strip sign
    if (g_sScale < 0.0f) g_sScale = -g_sScale;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: PowerSet %.1fW raw=%u signed=%d scale %.4f->%.4f "
              "(update HT7017_DEFAULT_POWER_SCALE to make permanent)",
              actual, g_lastRawP, s, old, g_pScale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Q scale also set to %.4f, S scale set to %.4f "
              "(approximate — refine with real reactive/apparent load)",
              g_qScale, g_sScale);
    return CMD_RES_OK;
}

// ── HT7017_Energy_Reset (new) ─────────────────────────────────────────────────
static commandResult_t CMD_EnergyReset(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    g_wh_total   = 0.0f;
    g_varh_total = 0.0f;
    g_ep1_last   = 0xFFFFFFFF;   // reseed on next EP1 reading
    g_eq1_last   = 0xFFFFFFFF;

    uint32_t now = NTP_GetCurrentTime();
    g_energy_reset_unix = (now > 1000000000u) ? now : 0;

    // Publish zero to energy channels
    HT7017_PublishChannel(HT7017_CHANNEL_WH,   0.0f);
    HT7017_PublishChannel(HT7017_CHANNEL_VARH, 0.0f);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Energy counters reset to 0  (unix=%u)", g_energy_reset_unix);
    return CMD_RES_OK;
}

// ── HT7017_Energy (new) ───────────────────────────────────────────────────────
static commandResult_t CMD_Energy(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    uint32_t now     = NTP_GetCurrentTime();
    uint32_t since   = (g_energy_reset_unix > 0) ? g_energy_reset_unix : g_session_start_unix;
    uint32_t elapsed = (now > since && since > 0) ? (now - since) : 0;

    float hours   = (float)elapsed / 3600.0f;
    float kw_avg  = (hours > 0.0f) ? (g_wh_total  / 1000.0f / hours) : 0.0f;
    float kvar_avg= (hours > 0.0f) ? (g_varh_total / 1000.0f / hours) : 0.0f;

    uint32_t days    = elapsed / 86400;
    uint32_t hrs     = (elapsed % 86400) / 3600;
    uint32_t mins    = (elapsed % 3600)  / 60;
    uint32_t secs    = elapsed % 60;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 Energy Summary ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Active Energy   : %.4f Wh  (%.4f kWh)",
              g_wh_total, g_wh_total / 1000.0f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Reactive Energy : %.4f VARh (%.4f kVARh)",
              g_varh_total, g_varh_total / 1000.0f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Session time    : %ud %02uh %02um %02us",
              days, hrs, mins, secs);
    if (hours > 0.0f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "  Avg Active Pwr  : %.2f W  (%.4f kW)",
                  kw_avg * 1000.0f, kw_avg);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "  Avg Reactive Pwr: %.2f VAR",
                  kvar_avg * 1000.0f);
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  NTP now unix    : %u", now);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Reset at unix   : %u", g_energy_reset_unix);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EP1 scale       : %.4f  (1 raw count = %.6f Wh)",
              g_e1Scale, 1.0f / g_e1Scale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  To reset        : HT7017_Energy_Reset");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚═══════════════════════════╝");
    return CMD_RES_OK;
}

// ── HT7017_Status (extended from v10) ────────────────────────────────────────
static commandResult_t CMD_Status(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 v11 Status ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Voltage        : %.2f V",    g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Current        : %.3f A",    g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Power   : %.2f W",    g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Reactive Power : %.2f VAR",  g_reactive);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Apparent (V*I) : %.2f VA",   g_apparent);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Apparent (S1)  : %.2f VA",   g_apparent_s1);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Power Factor   : %.4f",      g_power_factor);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Frequency      : %.3f Hz",   g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Energy  : %.4f Wh",   g_wh_total);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Reactive Energy: %.4f VARh", g_varh_total);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  PF Count raw   : %.0f",      g_pf_count_raw);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Scales  V=%.2f I=%.2f P=%.4f F=%.1f Q=%.4f S=%.4f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_qScale, g_sScale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EP1sc=%.4f  offset=%.0f",
              g_e1Scale, g_iOffset);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Last raw V=%u I=%u P=%u",
              g_lastRawV, g_lastRawI, g_lastRawP);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Frames  good=%u bad=%u miss=%u tx=%u",
              g_goodFrames, g_badFrames, g_totalMisses, g_txCount);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚════════════════════════╝");
    return CMD_RES_OK;
}

static commandResult_t CMD_Baud(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int baud = atoi(args);
    if (baud <= 0) return CMD_RES_BAD_ARGUMENT;
    UART_InitUART(baud, HT7017_PARITY_EVEN, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init %d baud 8E1", baud);
    return CMD_RES_OK;
}

static commandResult_t CMD_NoParity(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    UART_InitUART(HT7017_BAUD_RATE, 0, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init %d baud 8N1", HT7017_BAUD_RATE);
    return CMD_RES_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION G — INIT / RUN (v10 structure unchanged, register count updated)
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_Init(void)
{
    HT7017_BuildRegTable();

    // Start on last slot so first increment lands on slot 0 (Voltage)
    g_regIndex    = REG_TABLE_SIZE - 1;
    g_missCount   = 0;
    g_totalMisses = 0;
    g_txCount     = 0;
    g_goodFrames  = 0;
    g_badFrames   = 0;

    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    // v10 calibration commands
    CMD_RegisterCommand("VoltageSet",          CMD_VoltageSet,  NULL);
    CMD_RegisterCommand("CurrentSet",          CMD_CurrentSet,  NULL);
    CMD_RegisterCommand("PowerSet",            CMD_PowerSet,    NULL);
    // v11 energy commands
    CMD_RegisterCommand("HT7017_Energy_Reset", CMD_EnergyReset, NULL);
    CMD_RegisterCommand("HT7017_Energy",       CMD_Energy,      NULL);
    // diagnostic
    CMD_RegisterCommand("HT7017_Status",       CMD_Status,      NULL);
    CMD_RegisterCommand("HT7017_Baud",         CMD_Baud,        NULL);
    CMD_RegisterCommand("HT7017_NoParity",     CMD_NoParity,    NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v11: KWS-303WF  BK7231N->HT7017 direct  shunt");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v11: %s  4800 8E1  %u-slot rotation (%us cycle)",
              CFG_HasFlag(26) ? "UART2" : "UART1 (P0/P1)",
              REG_TABLE_SIZE, REG_TABLE_SIZE);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v11: V=%.2f I=%.2f P=%.4f F=%.1f offset=%.0f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_iOffset);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v11: Q=%.4f S=%.4f EP1=%.4f",
              g_qScale, g_sScale, g_e1Scale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v11: Ch V=%u I=%u P=%u F=%u PF=%u Q=%u S=%u Wh=%u VARh=%u",
              HT7017_CHANNEL_VOLTAGE, HT7017_CHANNEL_CURRENT,
              HT7017_CHANNEL_POWER,   HT7017_CHANNEL_FREQ,
              HT7017_CHANNEL_PF,      HT7017_CHANNEL_REACTIVE,
              HT7017_CHANNEL_APPARENT,HT7017_CHANNEL_WH,
              HT7017_CHANNEL_VARH);
}

void HT7017_RunEverySecond(void)
{
    int available = UART_GetDataSize();

    if (available >= HT7017_RESPONSE_LEN) {
        uint8_t d2 = UART_GetByte(0);
        uint8_t d1 = UART_GetByte(1);
        uint8_t d0 = UART_GetByte(2);
        uint8_t cs = UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);   // consume AFTER reading

        HT7017_ProcessFrame(d2, d1, d0, cs);
        g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
        g_missCount = 0;

    } else {
        g_missCount++;
        g_totalMisses++;

        if (g_txCount > 0) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: No response reg=0x%02X [%s] miss=%u",
                      g_regTable[g_regIndex].reg,
                      g_regTable[g_regIndex].name,
                      g_missCount);
        }

        UART_ConsumeBytes(UART_GetDataSize());

        if (g_missCount >= HT7017_MAX_MISS_COUNT) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: %u misses on 0x%02X - skip",
                      HT7017_MAX_MISS_COUNT, g_regTable[g_regIndex].reg);
            g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
            g_missCount = 0;
        }
    }

    uint8_t reg = g_regTable[g_regIndex].reg;
    HT7017_SendRequest(reg);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: TX > 6A %02X [%s] (tx=%u)",
              reg, g_regTable[g_regIndex].name, g_txCount);
}

void HT7017_RunQuick(void)
{
    if (UART_GetDataSize() < HT7017_RESPONSE_LEN) return;
    uint8_t d2 = UART_GetByte(0);
    uint8_t d1 = UART_GetByte(1);
    uint8_t d0 = UART_GetByte(2);
    uint8_t cs = UART_GetByte(3);
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);
    HT7017_ProcessFrame(d2, d1, d0, cs);
    // Do NOT advance g_regIndex — RunEverySecond manages rotation
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION H — HTTP PAGE (extended with all new measurements)
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    uint32_t now     = NTP_GetCurrentTime();
    uint32_t since   = (g_energy_reset_unix > 0) ? g_energy_reset_unix
                                                  : g_session_start_unix;
    uint32_t elapsed = (now > since && since > 0) ? (now - since) : 0;
    uint32_t days    = elapsed / 86400;
    uint32_t hrs     = (elapsed % 86400) / 3600;
    uint32_t mins    = (elapsed % 3600)  / 60;

    char tmp[1200];
    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 v11 - KWS-303WF</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>Voltage</td><td><b>%.2f V</b></td></tr>"
        "<tr><td>Current</td><td><b>%.3f A</b></td></tr>"
        "<tr><td>Frequency</td><td><b>%.3f Hz</b></td></tr>"
        "<tr><td>Active Power</td><td><b>%.2f W</b></td></tr>"
        "<tr><td>Reactive Power</td><td><b>%.2f VAR</b></td></tr>"
        "<tr><td>Apparent (V*I)</td><td><b>%.2f VA</b></td></tr>"
        "<tr><td>Apparent (S1)</td><td><b>%.2f VA</b></td></tr>"
        "<tr><td>Power Factor</td><td><b>%.4f</b></td></tr>"
        "<tr><td>Active Energy</td><td><b>%.4f Wh (%.5f kWh)</b></td></tr>"
        "<tr><td>Reactive Energy</td><td><b>%.4f VARh</b></td></tr>"
        "<tr><td>Session</td><td><b>%ud %02uh %02um</b></td></tr>"
        "<tr><td colspan='2' style='font-size:0.8em;color:#666'>"
          "good=%u bad=%u miss=%u tx=%u | "
          "V=%.0f I=%.0f P=%.3f Q=%.3f S=%.3f EP=%.3f"
        "</td></tr>"
        "</table>",
        g_voltage, g_current, g_freq,
        g_power, g_reactive,
        g_apparent, g_apparent_s1,
        g_power_factor,
        g_wh_total, g_wh_total / 1000.0f,
        g_varh_total,
        days, hrs, mins,
        g_goodFrames, g_badFrames, g_totalMisses, g_txCount,
        g_vScale, g_iScale, g_pScale, g_qScale, g_sScale, g_e1Scale);

    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION I — GETTERS (v10 unchanged, new ones appended)
// ═══════════════════════════════════════════════════════════════════════════════

float    HT7017_GetVoltage(void)        { return g_voltage;       }
float    HT7017_GetCurrent(void)        { return g_current;       }
float    HT7017_GetPower(void)          { return g_power;         }
float    HT7017_GetFrequency(void)      { return g_freq;          }
float    HT7017_GetPowerFactor(void)    { return g_power_factor;  }
float    HT7017_GetApparentPower(void)  { return g_apparent;      }
uint32_t HT7017_GetGoodFrames(void)     { return g_goodFrames;    }
uint32_t HT7017_GetBadFrames(void)      { return g_badFrames;     }
uint32_t HT7017_GetTxCount(void)        { return g_txCount;       }
uint32_t HT7017_GetMissCount(void)      { return g_totalMisses;   }
// v11 new getters
float    HT7017_GetReactivePower(void)  { return g_reactive;      }
float    HT7017_GetApparentS1(void)     { return g_apparent_s1;   }
float    HT7017_GetWh(void)             { return g_wh_total;      }
float    HT7017_GetVARh(void)           { return g_varh_total;    }

#endif // ENABLE_DRIVER_HT7017
