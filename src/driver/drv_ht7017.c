/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v15
 *
 * v15 FIXES over v14:
 *
 *   FIX 1 — FREQ moved to fast table
 *     Was slow table S2 → updated every 30 s (5 calls × 6 slow slots).
 *     Now fast table F3 → updated every ~1.25 s (5 calls / 4 fast slots).
 *     FAST_TABLE_SIZE 3→4.  SLOW_TABLE_SIZE 6→5 (FREQ slot removed).
 *     Grid frequency matters for EV charging and grid event detection;
 *     30 s resolution is unacceptable.
 *
 *   FIX 2 — Hysteresis actually applied
 *     v14 defined PROT_TRIP_HYSTERESIS_* macros but never used them.
 *     Result: relay chatters when measurement sits exactly at threshold.
 *     v15 uses separate trip/clear thresholds in EvaluateProtection():
 *       OV: trip  if V  > ovThreshold
 *           clear if V  < ovThreshold - HYSTERESIS_V
 *       UV: trip  if V  < uvThreshold  (and V > UV_MIN_VOLTAGE)
 *           clear if V  > uvThreshold + HYSTERESIS_V
 *       OC: trip  if I  > ocThreshold
 *           clear if I  < ocThreshold - HYSTERESIS_A
 *       OP: trip  if P  > opThreshold
 *           clear if P  < opThreshold - HYSTERESIS_W
 *
 *   FIX 3 — Channel scaling consistent
 *     All measurement channels: (float × 100) integer via HT7017_PublishChannel.
 *     EMUSR channel: was published raw (no ×100). Now ×100 like all others.
 *     ALARM channel: was published ×100 (gave 0/100/200/300/400 for states 0-4).
 *       Corrected to raw integer 0-4 — it is a state code, not a measurement.
 *       autoexec rules: "if ch20 == 1" means OV (not "if ch20 == 100").
 *
 *   FIX 4 — Recovery call-count formula corrected
 *     v14: recover_calls = RECOVER_SECONDS * 4 / 5  (~20% too short)
 *     v15: recover_calls = RECOVER_SECONDS
 *     Reason: g_callCount increments once per second regardless of which
 *     table was polled — it is a wall-clock second counter.
 *
 *   FIX 5 — Misleading comment on g_callCount removed
 *
 *   ALL EARLIER FIXES PRESERVED:
 *     v12 FIX 1 — FREQ formula: 500000/raw not raw/500000
 *     v12 FIX 2 — 0x10 is PowerP2, not PFCNT
 *     v12 FIX 3 — g_sScale = fabsf(g_pScale) in CMD_PowerSet
 *     v12 FIX 4 — Energy seeded flags: bool, not 0xFFFFFFFF sentinel
 *     v13      — Two-speed scheduling (architecture unchanged)
 *     v14 FIX  — PF = P/(V×I) always, published on every fast cycle
 *     v14 FIX  — EMUSR polled from slow table
 *     v14 FIX  — Software OV/UV/OC/OP protection with relay control
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
// SECTION A — SCALE FACTORS AND MEASUREMENTS
// ═══════════════════════════════════════════════════════════════════════════════

static float g_vScale  = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale  = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale  = HT7017_DEFAULT_FREQ_SCALE;
static float g_qScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_sScale  = HT7017_DEFAULT_APPARENT_SCALE;
static float g_e1Scale = HT7017_DEFAULT_EP1_SCALE;
static float g_iOffset = HT7017_CURRENT_OFFSET;

// Last raw values — used by calibration commands
static uint32_t g_lastRawV = 0;
static uint32_t g_lastRawI = 0;
static uint32_t g_lastRawP = 0;

// Instantaneous measurements
static float g_voltage      = 0.0f;
static float g_current      = 0.0f;
static float g_power        = 0.0f;
static float g_freq         = 0.0f;
static float g_reactive     = 0.0f;
static float g_apparent_s1  = 0.0f;
static float g_apparent     = 0.0f;   // V × I (always fresh)
static float g_power_factor = 0.0f;

// S1 freshness tracking (for HTTP diagnostic)
static uint32_t g_s1_last_update_tick = 0;

// EMU status
static uint32_t g_emusr_raw = 0;

// Energy accumulators
static uint32_t g_ep1_last   = 0;
static uint32_t g_eq1_last   = 0;
static bool     g_ep1_seeded = false;
static bool     g_eq1_seeded = false;
static float    g_wh_total   = 0.0f;
static float    g_varh_total = 0.0f;

// NTP session timestamps
static uint32_t g_session_start_unix = 0;
static uint32_t g_energy_reset_unix  = 0;

// Diagnostics
static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;
static uint32_t g_totalMisses = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION A2 — SOFTWARE PROTECTION STATE
// ═══════════════════════════════════════════════════════════════════════════════

static float    g_ovThreshold  = HT7017_DEFAULT_OV_THRESHOLD;
static float    g_uvThreshold  = HT7017_DEFAULT_UV_THRESHOLD;
static float    g_ocThreshold  = HT7017_DEFAULT_OC_THRESHOLD;
static float    g_opThreshold  = HT7017_DEFAULT_OP_THRESHOLD;
static uint8_t  g_relayChannel = HT7017_DEFAULT_RELAY_CHANNEL;
static uint8_t  g_alarmState   = 0;      // 0=OK 1=OV 2=UV 3=OC 4=OP
static bool     g_relayTripped = false;
static uint32_t g_tripCallCount = 0;

// ─── g_callCount: incremented once per RunEverySecond() call = 1/second ───────
static uint32_t g_callCount = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION B — REGISTER TABLES
// ═══════════════════════════════════════════════════════════════════════════════

typedef struct {
    uint8_t     reg;
    float      *target;
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint8_t     is_energy;
    uint8_t     is_emusr;
    uint32_t   *last_raw_acc;
    bool       *seeded_flag;
    float      *acc_total;
    uint32_t   *last_raw_cal;
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

// ── FAST table: V, I, P, FREQ (FIX 1: FREQ promoted from slow) ───────────────
#define FAST_TABLE_SIZE 4
static RegRead_t g_fastTable[FAST_TABLE_SIZE];

// ── SLOW table: Q, S, EP1, EQ1, EMUSR ────────────────────────────────────────
#define SLOW_TABLE_SIZE 5
static RegRead_t g_slowTable[SLOW_TABLE_SIZE];

static void HT7017_BuildTables(void)
{
    // ── FAST TABLE ────────────────────────────────────────────────────────────

    // F0: Voltage RMS
    g_fastTable[0].reg          = HT7017_REG_RMS_U;
    g_fastTable[0].target       = &g_voltage;
    g_fastTable[0].scale        = &g_vScale;
    g_fastTable[0].is_signed    = 0;
    g_fastTable[0].has_offset   = 0;
    g_fastTable[0].is_energy    = 0;
    g_fastTable[0].is_emusr     = 0;
    g_fastTable[0].last_raw_acc = NULL;
    g_fastTable[0].seeded_flag  = NULL;
    g_fastTable[0].acc_total    = NULL;
    g_fastTable[0].last_raw_cal = &g_lastRawV;
    g_fastTable[0].name         = "Voltage";
    g_fastTable[0].unit         = "V";
    g_fastTable[0].obk_ch       = HT7017_CHANNEL_VOLTAGE;

    // F1: Current RMS
    g_fastTable[1].reg          = HT7017_REG_RMS_I1;
    g_fastTable[1].target       = &g_current;
    g_fastTable[1].scale        = &g_iScale;
    g_fastTable[1].is_signed    = 0;
    g_fastTable[1].has_offset   = 1;
    g_fastTable[1].is_energy    = 0;
    g_fastTable[1].is_emusr     = 0;
    g_fastTable[1].last_raw_acc = NULL;
    g_fastTable[1].seeded_flag  = NULL;
    g_fastTable[1].acc_total    = NULL;
    g_fastTable[1].last_raw_cal = &g_lastRawI;
    g_fastTable[1].name         = "Current";
    g_fastTable[1].unit         = "A";
    g_fastTable[1].obk_ch       = HT7017_CHANNEL_CURRENT;

    // F2: Active Power P1 (SIGNED)
    g_fastTable[2].reg          = HT7017_REG_POWER_P1;
    g_fastTable[2].target       = &g_power;
    g_fastTable[2].scale        = &g_pScale;
    g_fastTable[2].is_signed    = 1;
    g_fastTable[2].has_offset   = 0;
    g_fastTable[2].is_energy    = 0;
    g_fastTable[2].is_emusr     = 0;
    g_fastTable[2].last_raw_acc = NULL;
    g_fastTable[2].seeded_flag  = NULL;
    g_fastTable[2].acc_total    = NULL;
    g_fastTable[2].last_raw_cal = &g_lastRawP;
    g_fastTable[2].name         = "Power";
    g_fastTable[2].unit         = "W";
    g_fastTable[2].obk_ch       = HT7017_CHANNEL_POWER;

    // F3: Frequency — FIX 1: promoted from slow table
    // Formula: freq = g_fScale / raw  (500000 / raw)
    g_fastTable[3].reg          = HT7017_REG_FREQ;
    g_fastTable[3].target       = &g_freq;
    g_fastTable[3].scale        = &g_fScale;
    g_fastTable[3].is_signed    = 0;
    g_fastTable[3].has_offset   = 0;
    g_fastTable[3].is_energy    = 0;
    g_fastTable[3].is_emusr     = 0;
    g_fastTable[3].last_raw_acc = NULL;
    g_fastTable[3].seeded_flag  = NULL;
    g_fastTable[3].acc_total    = NULL;
    g_fastTable[3].last_raw_cal = NULL;
    g_fastTable[3].name         = "Freq";
    g_fastTable[3].unit         = "Hz";
    g_fastTable[3].obk_ch       = HT7017_CHANNEL_FREQ;

    // ── SLOW TABLE ────────────────────────────────────────────────────────────

    // S0: Reactive Power Q1 (SIGNED)
    g_slowTable[0].reg          = HT7017_REG_POWER_Q1;
    g_slowTable[0].target       = &g_reactive;
    g_slowTable[0].scale        = &g_qScale;
    g_slowTable[0].is_signed    = 1;
    g_slowTable[0].has_offset   = 0;
    g_slowTable[0].is_energy    = 0;
    g_slowTable[0].is_emusr     = 0;
    g_slowTable[0].last_raw_acc = NULL;
    g_slowTable[0].seeded_flag  = NULL;
    g_slowTable[0].acc_total    = NULL;
    g_slowTable[0].last_raw_cal = NULL;
    g_slowTable[0].name         = "Reactive";
    g_slowTable[0].unit         = "VAR";
    g_slowTable[0].obk_ch       = HT7017_CHANNEL_REACTIVE;

    // S1: Apparent Power S1
    g_slowTable[1].reg          = HT7017_REG_POWER_S1;
    g_slowTable[1].target       = &g_apparent_s1;
    g_slowTable[1].scale        = &g_sScale;
    g_slowTable[1].is_signed    = 0;
    g_slowTable[1].has_offset   = 0;
    g_slowTable[1].is_energy    = 0;
    g_slowTable[1].is_emusr     = 0;
    g_slowTable[1].last_raw_acc = NULL;
    g_slowTable[1].seeded_flag  = NULL;
    g_slowTable[1].acc_total    = NULL;
    g_slowTable[1].last_raw_cal = NULL;
    g_slowTable[1].name         = "Apparent";
    g_slowTable[1].unit         = "VA";
    g_slowTable[1].obk_ch       = HT7017_CHANNEL_APPARENT;

    // S2: Active Energy EP1
    g_slowTable[2].reg          = HT7017_REG_EP1;
    g_slowTable[2].target       = &g_wh_total;
    g_slowTable[2].scale        = &g_e1Scale;
    g_slowTable[2].is_signed    = 0;
    g_slowTable[2].has_offset   = 0;
    g_slowTable[2].is_energy    = 1;
    g_slowTable[2].is_emusr     = 0;
    g_slowTable[2].last_raw_acc = &g_ep1_last;
    g_slowTable[2].seeded_flag  = &g_ep1_seeded;
    g_slowTable[2].acc_total    = &g_wh_total;
    g_slowTable[2].last_raw_cal = NULL;
    g_slowTable[2].name         = "ActiveEnergy";
    g_slowTable[2].unit         = "Wh";
    g_slowTable[2].obk_ch       = HT7017_CHANNEL_WH;

    // S3: Reactive Energy EQ1
    g_slowTable[3].reg          = HT7017_REG_EQ1;
    g_slowTable[3].target       = &g_varh_total;
    g_slowTable[3].scale        = &g_e1Scale;
    g_slowTable[3].is_signed    = 0;
    g_slowTable[3].has_offset   = 0;
    g_slowTable[3].is_energy    = 1;
    g_slowTable[3].is_emusr     = 0;
    g_slowTable[3].last_raw_acc = &g_eq1_last;
    g_slowTable[3].seeded_flag  = &g_eq1_seeded;
    g_slowTable[3].acc_total    = &g_varh_total;
    g_slowTable[3].last_raw_cal = NULL;
    g_slowTable[3].name         = "ReactiveEnergy";
    g_slowTable[3].unit         = "VARh";
    g_slowTable[3].obk_ch       = HT7017_CHANNEL_VARH;

    // S4: EMUSR — EMU status register
    g_slowTable[4].reg          = HT7017_REG_EMUSR;
    g_slowTable[4].target       = NULL;
    g_slowTable[4].scale        = NULL;
    g_slowTable[4].is_signed    = 0;
    g_slowTable[4].has_offset   = 0;
    g_slowTable[4].is_energy    = 0;
    g_slowTable[4].is_emusr     = 1;
    g_slowTable[4].last_raw_acc = NULL;
    g_slowTable[4].seeded_flag  = NULL;
    g_slowTable[4].acc_total    = NULL;
    g_slowTable[4].last_raw_cal = NULL;
    g_slowTable[4].name         = "EMUSR";
    g_slowTable[4].unit         = "bits";
    g_slowTable[4].obk_ch       = HT7017_CHANNEL_EMUSR;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION C — SCHEDULING STATE
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t g_fastIndex   = 0;
static uint8_t g_slowIndex   = 0;
static bool    g_lastWasFast = true;
static uint8_t g_fastMiss    = 0;
static uint8_t g_slowMiss    = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D — HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t HT7017_Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

static float HT7017_Convert(uint32_t raw, const RegRead_t *r)
{
    // FREQ is a period counter: freq = scale / raw  (v12 FIX 1)
    if (r->reg == HT7017_REG_FREQ) {
        if (raw == 0) return 0.0f;
        return (*r->scale) / (float)raw;
    }

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

// Publish a measurement channel: all values stored as (float × 100) integer.
// 230.15 V  → setChannel 10 23015
// 50.06 Hz  → setChannel 13 5006
// 0.95 PF   → setChannel 14 95
// NOTE: EMUSR and ALARM use separate publish helpers (see below).
static void HT7017_PublishChannel(uint8_t ch, float value)
{
    if (ch == 0) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "setChannel %u %d", ch, (int)(value * 100.0f));
    CMD_ExecuteCommand(buf, 0);
}

// Publish EMUSR bitmask — also ×100 for consistency with all other channels.
// raw bitmask 3 → setChannel 19 300  (÷100 in display = 3)
static void HT7017_PublishEmusr(uint8_t ch, uint32_t bitmask)
{
    if (ch == 0) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "setChannel %u %u", ch,
             (unsigned)(bitmask & 0x1F) * 100u);
    CMD_ExecuteCommand(buf, 0);
}

// Publish ALARM state — raw integer 0-4, NOT ×100.
// This is a state code, not a measurement. Rules read: "if ch20 == 1" for OV.
static void HT7017_PublishAlarm(uint8_t ch, uint8_t state)
{
    if (ch == 0) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "setChannel %u %u", ch, (unsigned)state);
    CMD_ExecuteCommand(buf, 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D2 — POWER FACTOR (PF = P / (V×I), published every fast frame)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_UpdatePowerFactor(void)
{
    g_apparent = g_voltage * g_current;

    if (g_apparent > 0.5f && g_power > 0.0f) {
        g_power_factor = g_power / g_apparent;
        if (g_power_factor > 1.0f) g_power_factor = 1.0f;
        if (g_power_factor < 0.0f) g_power_factor = 0.0f;
    } else {
        g_power_factor = 0.0f;
    }

    if (HT7017_CHANNEL_PF > 0) {
        HT7017_PublishChannel(HT7017_CHANNEL_PF, g_power_factor);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D3 — SOFTWARE PROTECTION WITH HYSTERESIS (FIX 2 + FIX 3 + FIX 4)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_EvaluateProtection(void)
{
    bool any_threshold = (g_ovThreshold > 0.0f) ||
                         (g_uvThreshold > 0.0f) ||
                         (g_ocThreshold > 0.0f) ||
                         (g_opThreshold > 0.0f);
    if (!any_threshold && !g_relayTripped) return;

    uint8_t new_alarm = 0;

    if (!g_relayTripped) {
        // ── Not currently tripped: check trip conditions ──────────────────────
        // Priority: OV > OC > OP > UV
        if (g_ovThreshold > 0.0f && g_voltage > g_ovThreshold) {
            new_alarm = 1;  // OV
        } else if (g_ocThreshold > 0.0f && g_current > g_ocThreshold) {
            new_alarm = 3;  // OC
        } else if (g_opThreshold > 0.0f && g_power > g_opThreshold) {
            new_alarm = 4;  // OP
        } else if (g_uvThreshold > 0.0f &&
                   g_voltage < g_uvThreshold &&
                   g_voltage > HT7017_UV_MIN_VOLTAGE) {
            new_alarm = 2;  // UV
        }

        if (new_alarm != 0) {
            g_alarmState    = new_alarm;
            g_relayTripped  = true;
            g_tripCallCount = g_callCount;

            char buf[32];
            snprintf(buf, sizeof(buf), "setChannel %u 0", g_relayChannel);
            CMD_ExecuteCommand(buf, 0);

            HT7017_PublishAlarm(HT7017_CHANNEL_ALARM, g_alarmState);

            const char *names[] = { "OK", "OV", "UV", "OC", "OP" };
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: PROTECTION TRIP [%s]  V=%.2f I=%.3f P=%.1f  relay ch=%u OFF",
                      names[new_alarm], g_voltage, g_current, g_power, g_relayChannel);
        }

    } else {
        // ── Currently tripped: check hysteresis clear conditions (FIX 2) ──────
        bool cleared = false;

        switch (g_alarmState) {
            case 1: // OV — clear when V drops below (threshold - hysteresis)
                cleared = (g_ovThreshold > 0.0f) &&
                          (g_voltage < g_ovThreshold - HT7017_PROT_HYSTERESIS_V);
                break;
            case 2: // UV — clear when V rises above (threshold + hysteresis)
                cleared = (g_uvThreshold > 0.0f) &&
                          (g_voltage > g_uvThreshold + HT7017_PROT_HYSTERESIS_V);
                break;
            case 3: // OC — clear when I drops below (threshold - hysteresis)
                cleared = (g_ocThreshold > 0.0f) &&
                          (g_current < g_ocThreshold - HT7017_PROT_HYSTERESIS_A);
                break;
            case 4: // OP — clear when P drops below (threshold - hysteresis)
                cleared = (g_opThreshold > 0.0f) &&
                          (g_power < g_opThreshold - HT7017_PROT_HYSTERESIS_W);
                break;
            default:
                cleared = true;
                break;
        }

        if (cleared) {
            // FIX 4: recovery window in seconds = g_callCount ticks (1/s)
            uint32_t elapsed = g_callCount - g_tripCallCount;
            if (elapsed >= (uint32_t)HT7017_PROT_RECOVER_SECONDS) {
                g_relayTripped = false;
                g_alarmState   = 0;

                char buf[32];
                snprintf(buf, sizeof(buf), "setChannel %u 1", g_relayChannel);
                CMD_ExecuteCommand(buf, 0);

                HT7017_PublishAlarm(HT7017_CHANNEL_ALARM, 0);

                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: PROTECTION CLEAR — relay ch=%u restored  "
                          "V=%.2f I=%.3f P=%.1f",
                          g_relayChannel, g_voltage, g_current, g_power);
            }
        } else {
            // Still in fault — fault type may have changed (e.g. OV faded, OC hit)
            // Re-evaluate which fault is active now
            uint8_t active = 0;
            if (g_ovThreshold > 0.0f && g_voltage > g_ovThreshold)         active = 1;
            else if (g_ocThreshold > 0.0f && g_current > g_ocThreshold)    active = 3;
            else if (g_opThreshold > 0.0f && g_power > g_opThreshold)      active = 4;
            else if (g_uvThreshold > 0.0f &&
                     g_voltage < g_uvThreshold &&
                     g_voltage > HT7017_UV_MIN_VOLTAGE)                     active = 2;

            if (active != 0 && active != g_alarmState) {
                g_alarmState = active;
                HT7017_PublishAlarm(HT7017_CHANNEL_ALARM, g_alarmState);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D4 — EMUSR PROCESSING
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_ProcessEmusr(uint32_t raw)
{
    g_emusr_raw = raw;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [EMUSR] raw=0x%06X  OV=%u UV=%u OC=%u OP=%u ZXLOSS=%u",
              raw,
              (raw & HT7017_EMUSR_BIT_OV)     ? 1u : 0u,
              (raw & HT7017_EMUSR_BIT_UV)     ? 1u : 0u,
              (raw & HT7017_EMUSR_BIT_OC)     ? 1u : 0u,
              (raw & HT7017_EMUSR_BIT_OP)     ? 1u : 0u,
              (raw & HT7017_EMUSR_BIT_ZXLOSS) ? 1u : 0u);

    // FIX 3: EMUSR published ×100 like all other channels
    HT7017_PublishEmusr(HT7017_CHANNEL_EMUSR, raw);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION E — ENERGY ACCUMULATOR (unchanged from v12)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_AccumulateEnergy(uint32_t raw, const RegRead_t *r)
{
    if (!(*r->seeded_flag)) {
        *r->last_raw_acc = raw;
        *r->seeded_flag  = true;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] seeded at raw=%u  accumulating from next read",
                  r->name, raw);
        return;
    }

    uint32_t delta;
    if (raw >= *r->last_raw_acc) {
        delta = raw - *r->last_raw_acc;
    } else {
        delta = (0x00FFFFFF - *r->last_raw_acc) + raw + 1;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: [%s] counter wrap last=%u new=%u delta=%u",
                  r->name, *r->last_raw_acc, raw, delta);
    }
    *r->last_raw_acc = raw;

    if (delta == 0) return;

    float increment = (float)delta / (*r->scale);
    *r->acc_total  += increment;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [%s] raw=%u  delta=%u  +%.5f %s  total=%.4f %s",
              r->name, raw, delta, increment, r->unit, *r->acc_total, r->unit);

    if (r->obk_ch > 0) {
        char buf[48];
        snprintf(buf, sizeof(buf), "setChannel %u %d",
                 r->obk_ch, (int)(*r->acc_total * 1000.0f));
        CMD_ExecuteCommand(buf, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION F — FRAME PROCESSING
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_ProcessFrame(uint8_t d2, uint8_t d1, uint8_t d0, uint8_t cs,
                                 const RegRead_t *r)
{
    uint8_t exp = HT7017_Checksum(r->reg, d2, d1, d0);
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

    if (r->last_raw_cal) *r->last_raw_cal = raw;

    // ── Special paths ─────────────────────────────────────────────────────────
    if (r->is_emusr) {
        HT7017_ProcessEmusr(raw);
        return;
    }

    if (r->is_energy) {
        HT7017_AccumulateEnergy(raw, r);
    } else {
        if (r->reg == HT7017_REG_POWER_S1) {
            g_s1_last_update_tick = g_callCount;
        }

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
                      (int32_t)((int32_t)raw - (int32_t)g_iOffset), g_goodFrames);
        } else {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: [%s] = %.3f %s  raw=%u  CS=OK (good=%u)",
                      r->name, value, r->unit, raw, g_goodFrames);
        }

        if (r->obk_ch > 0) {
            HT7017_PublishChannel(r->obk_ch, value);
        }
    }

    // PF recomputed after every frame (uses V×I — always fresh)
    HT7017_UpdatePowerFactor();

    // Protection evaluated after every frame
    HT7017_EvaluateProtection();

    // NTP session start
    if (g_session_start_unix == 0) {
        uint32_t now = NTP_GetCurrentTime();
        if (now > 1000000000u) {
            g_session_start_unix = now;
            g_energy_reset_unix  = now;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: NTP session start unix=%u", now);
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
// SECTION G — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════════

static commandResult_t CMD_VoltageSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;
    if (g_lastRawV == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: VoltageSet - no reading yet"); return CMD_RES_ERROR;
    }
    float old = g_vScale;
    g_vScale  = (float)g_lastRawV / actual;
    g_voltage = actual;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: VoltageSet %.2fV raw=%u scale %.2f->%.2f",
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
                  "HT7017: CurrentSet - no reading yet"); return CMD_RES_ERROR;
    }
    float net = (float)g_lastRawI - g_iOffset;
    if (net <= 0.0f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CurrentSet - net=%.0f too low", net);
        return CMD_RES_ERROR;
    }
    float old = g_iScale;
    g_iScale  = net / actual;
    g_current = actual;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: CurrentSet %.3fA raw=%u net=%.0f scale %.2f->%.2f",
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
                  "HT7017: PowerSet - no reading yet"); return CMD_RES_ERROR;
    }
    int32_t s = (int32_t)g_lastRawP;
    if (s & 0x800000) s |= (int32_t)0xFF000000;
    if (s == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: PowerSet - signed raw=0"); return CMD_RES_ERROR;
    }
    float old = g_pScale;
    g_pScale  = (float)s / actual;
    g_power   = actual;
    g_qScale  = g_pScale;
    g_sScale  = fabsf(g_pScale);   // v12 FIX 3
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: PowerSet %.1fW raw=%u signed=%d scale %.4f->%.4f",
              actual, g_lastRawP, s, old, g_pScale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Q=%.4f S=%.4f (approximate — refine with reactive load)",
              g_qScale, g_sScale);
    return CMD_RES_OK;
}

static commandResult_t CMD_EnergyReset(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    g_wh_total   = 0.0f;
    g_varh_total = 0.0f;
    g_ep1_seeded = false;
    g_eq1_seeded = false;

    uint32_t now = NTP_GetCurrentTime();
    g_energy_reset_unix = (now > 1000000000u) ? now : 0;

    HT7017_PublishChannel(HT7017_CHANNEL_WH,   0.0f);
    HT7017_PublishChannel(HT7017_CHANNEL_VARH, 0.0f);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Energy reset  unix=%u", g_energy_reset_unix);
    return CMD_RES_OK;
}

static commandResult_t CMD_Energy(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    uint32_t now     = NTP_GetCurrentTime();
    uint32_t since   = (g_energy_reset_unix > 0) ? g_energy_reset_unix : g_session_start_unix;
    uint32_t elapsed = (now > since && since > 0) ? (now - since) : 0;
    float hours      = (float)elapsed / 3600.0f;

    uint32_t days = elapsed / 86400;
    uint32_t hrs  = (elapsed % 86400) / 3600;
    uint32_t mins = (elapsed % 3600)  / 60;
    uint32_t secs = elapsed % 60;

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
                  "  Avg Active Pwr  : %.2f W",
                  (g_wh_total / hours));
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "  Avg Reactive Pwr: %.2f VAR",
                  (g_varh_total / hours));
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  NTP now unix    : %u   Reset unix: %u",
              now, g_energy_reset_unix);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EP1 scale       : %.4f  (1 count = %.6f Wh)",
              g_e1Scale, 1.0f / g_e1Scale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚═══════════════════════════╝");
    return CMD_RES_OK;
}

static commandResult_t CMD_Status(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    const char *alarm_names[] = { "OK", "OV", "UV", "OC", "OP" };
    uint32_t s1_age_secs = (g_callCount > g_s1_last_update_tick)
                         ? (g_callCount - g_s1_last_update_tick) : 0;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 v15 Status ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Voltage        : %.2f V",  g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Current        : %.3f A",  g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Frequency      : %.3f Hz", g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Power   : %.2f W",  g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Reactive Power : %.2f VAR",g_reactive);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Apparent (V*I) : %.2f VA", g_apparent);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Apparent (S1)  : %.2f VA  (age ~%us)",
              g_apparent_s1, s1_age_secs);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Power Factor   : %.4f  (V*I basis)",
              g_power_factor);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Energy  : %.4f Wh",  g_wh_total);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Reactive Energy: %.4f VARh",g_varh_total);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EMUSR (0x19)   : 0x%02X  OV=%u UV=%u OC=%u OP=%u ZXLOSS=%u",
              g_emusr_raw,
              (g_emusr_raw & HT7017_EMUSR_BIT_OV)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_UV)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_OC)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_OP)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_ZXLOSS) ? 1u : 0u);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  SW Alarm       : %s  relay_ch=%u %s",
              alarm_names[g_alarmState], g_relayChannel,
              g_relayTripped ? "[TRIPPED]" : "[OK]");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  SW Thresholds  : OV=%.1f UV=%.1f OC=%.3f OP=%.1f  "
              "Hys: V=%.1f A=%.2f W=%.1f",
              g_ovThreshold, g_uvThreshold, g_ocThreshold, g_opThreshold,
              HT7017_PROT_HYSTERESIS_V, HT7017_PROT_HYSTERESIS_A,
              HT7017_PROT_HYSTERESIS_W);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Scales  V=%.2f I=%.2f P=%.4f F=%.0f Q=%.4f S=%.4f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_qScale, g_sScale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EP1sc=%.4f  offset=%.0f  rawV=%u rawI=%u rawP=%u",
              g_e1Scale, g_iOffset, g_lastRawV, g_lastRawI, g_lastRawP);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Frames good=%u bad=%u miss=%u tx=%u  callCount=%u",
              g_goodFrames, g_badFrames, g_totalMisses, g_txCount, g_callCount);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Poll  FAST(V,I,P,F) ~1.25s  SLOW(Q,S,EP1,EQ1,EMUSR) ~25s  "
              "fastIdx=%u slowIdx=%u",
              g_fastIndex, g_slowIndex);
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

// ── Protection threshold commands ─────────────────────────────────────────────

static commandResult_t CMD_SetOV(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float v = (float)atof(args);
    if (v < 0.0f) return CMD_RES_BAD_ARGUMENT;
    g_ovThreshold = v;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: OV threshold=%.1f V  clear<%.1f V  %s",
              v, v - HT7017_PROT_HYSTERESIS_V,
              v == 0.0f ? "(disabled)" : "");
    return CMD_RES_OK;
}

static commandResult_t CMD_SetUV(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float v = (float)atof(args);
    if (v < 0.0f) return CMD_RES_BAD_ARGUMENT;
    g_uvThreshold = v;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UV threshold=%.1f V  clear>%.1f V  guard>%.0f V  %s",
              v, v + HT7017_PROT_HYSTERESIS_V, HT7017_UV_MIN_VOLTAGE,
              v == 0.0f ? "(disabled)" : "");
    return CMD_RES_OK;
}

static commandResult_t CMD_SetOC(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float a = (float)atof(args);
    if (a < 0.0f) return CMD_RES_BAD_ARGUMENT;
    g_ocThreshold = a;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: OC threshold=%.3f A  clear<%.3f A  %s",
              a, a - HT7017_PROT_HYSTERESIS_A,
              a == 0.0f ? "(disabled)" : "");
    return CMD_RES_OK;
}

static commandResult_t CMD_SetOP(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float w = (float)atof(args);
    if (w < 0.0f) return CMD_RES_BAD_ARGUMENT;
    g_opThreshold = w;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: OP threshold=%.1f W  clear<%.1f W  %s",
              w, w - HT7017_PROT_HYSTERESIS_W,
              w == 0.0f ? "(disabled)" : "");
    return CMD_RES_OK;
}

static commandResult_t CMD_SetRelayChannel(const void *ctx, const char *cmd,
                                            const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int ch = atoi(args);
    if (ch <= 0 || ch > 32) return CMD_RES_BAD_ARGUMENT;
    g_relayChannel = (uint8_t)ch;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: relay channel=%u", g_relayChannel);
    return CMD_RES_OK;
}

static commandResult_t CMD_ProtStatus(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    const char *alarm_names[] = { "OK", "OV", "UV", "OC", "OP" };
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 Protection ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Relay ch=%u  State=%s  %s",
              g_relayChannel, alarm_names[g_alarmState],
              g_relayTripped ? "[TRIPPED]" : "[OK]");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  OV: trip>%.1fV  clear<%.1fV  %s",
              g_ovThreshold, g_ovThreshold - HT7017_PROT_HYSTERESIS_V,
              g_ovThreshold > 0.0f ? "" : "(disabled)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  UV: trip<%.1fV  clear>%.1fV  guard>%.0fV  %s",
              g_uvThreshold, g_uvThreshold + HT7017_PROT_HYSTERESIS_V,
              HT7017_UV_MIN_VOLTAGE,
              g_uvThreshold > 0.0f ? "" : "(disabled)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  OC: trip>%.3fA  clear<%.3fA  %s",
              g_ocThreshold, g_ocThreshold - HT7017_PROT_HYSTERESIS_A,
              g_ocThreshold > 0.0f ? "" : "(disabled)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  OP: trip>%.1fW  clear<%.1fW  %s",
              g_opThreshold, g_opThreshold - HT7017_PROT_HYSTERESIS_W,
              g_opThreshold > 0.0f ? "" : "(disabled)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Recover %us after fault clears",
              HT7017_PROT_RECOVER_SECONDS);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  EMUSR: 0x%02X  OV=%u UV=%u OC=%u OP=%u ZXLOSS=%u",
              g_emusr_raw,
              (g_emusr_raw & HT7017_EMUSR_BIT_OV)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_UV)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_OC)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_OP)     ? 1u : 0u,
              (g_emusr_raw & HT7017_EMUSR_BIT_ZXLOSS) ? 1u : 0u);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Now: V=%.2f I=%.3f P=%.1f",
              g_voltage, g_current, g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚═══════════════════════╝");
    return CMD_RES_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION H — INIT
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_Init(void)
{
    HT7017_BuildTables();

    // Start indices at last slot so first increment wraps to slot 0
    g_fastIndex           = FAST_TABLE_SIZE - 1;
    g_slowIndex           = SLOW_TABLE_SIZE - 1;
    g_callCount           = 0;
    g_lastWasFast         = true;
    g_fastMiss            = 0;
    g_slowMiss            = 0;
    g_txCount             = 0;
    g_goodFrames          = 0;
    g_badFrames           = 0;
    g_totalMisses         = 0;
    g_emusr_raw           = 0;
    g_alarmState          = 0;
    g_relayTripped        = false;
    g_tripCallCount       = 0;
    g_s1_last_update_tick = 0;

    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("VoltageSet",             CMD_VoltageSet,      NULL);
    CMD_RegisterCommand("CurrentSet",             CMD_CurrentSet,      NULL);
    CMD_RegisterCommand("PowerSet",               CMD_PowerSet,        NULL);
    CMD_RegisterCommand("HT7017_Energy_Reset",    CMD_EnergyReset,     NULL);
    CMD_RegisterCommand("HT7017_Energy",          CMD_Energy,          NULL);
    CMD_RegisterCommand("HT7017_Status",          CMD_Status,          NULL);
    CMD_RegisterCommand("HT7017_Baud",            CMD_Baud,            NULL);
    CMD_RegisterCommand("HT7017_NoParity",        CMD_NoParity,        NULL);
    CMD_RegisterCommand("HT7017_SetOV",           CMD_SetOV,           NULL);
    CMD_RegisterCommand("HT7017_SetUV",           CMD_SetUV,           NULL);
    CMD_RegisterCommand("HT7017_SetOC",           CMD_SetOC,           NULL);
    CMD_RegisterCommand("HT7017_SetOP",           CMD_SetOP,           NULL);
    CMD_RegisterCommand("HT7017_SetRelayChannel", CMD_SetRelayChannel, NULL);
    CMD_RegisterCommand("HT7017_ProtStatus",      CMD_ProtStatus,      NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v15: KWS-303WF  BK7231N->HT7017 direct  4800 8E1");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v15: FAST(V,I,P,FREQ) ~1.25s  SLOW(Q,S,EP1,EQ1,EMUSR) ~25s");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v15: PF=P/(V*I) on every fast cycle  "
              "Hys OV/UV=%.1fV OC=%.2fA OP=%.1fW",
              HT7017_PROT_HYSTERESIS_V, HT7017_PROT_HYSTERESIS_A,
              HT7017_PROT_HYSTERESIS_W);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v15: V=%.2f I=%.2f P=%.4f F=%.0f Q=%.4f S=%.4f EP1=%.4f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_qScale, g_sScale,
              g_e1Scale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v15: Ch V=%u I=%u P=%u F=%u PF=%u Q=%u S=%u "
              "Wh=%u VARh=%u EMUSR=%u ALM=%u  relay_ch=%u",
              HT7017_CHANNEL_VOLTAGE,  HT7017_CHANNEL_CURRENT,
              HT7017_CHANNEL_POWER,    HT7017_CHANNEL_FREQ,
              HT7017_CHANNEL_PF,       HT7017_CHANNEL_REACTIVE,
              HT7017_CHANNEL_APPARENT, HT7017_CHANNEL_WH,
              HT7017_CHANNEL_VARH,     HT7017_CHANNEL_EMUSR,
              HT7017_CHANNEL_ALARM,    g_relayChannel);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION I — RUN EVERY SECOND
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_RunEverySecond(void)
{
    // ── Step 1: process reply from previous TX ────────────────────────────────
    int available = UART_GetDataSize();

    if (available >= HT7017_RESPONSE_LEN) {
        uint8_t d2 = UART_GetByte(0);
        uint8_t d1 = UART_GetByte(1);
        uint8_t d0 = UART_GetByte(2);
        uint8_t cs = UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);

        if (g_lastWasFast) {
            HT7017_ProcessFrame(d2, d1, d0, cs, &g_fastTable[g_fastIndex]);
            g_fastMiss = 0;
        } else {
            HT7017_ProcessFrame(d2, d1, d0, cs, &g_slowTable[g_slowIndex]);
            g_slowMiss = 0;
        }

    } else {
        g_totalMisses++;

        if (g_txCount > 0) {
            if (g_lastWasFast) {
                g_fastMiss++;
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: No reply FAST reg=0x%02X [%s] miss=%u",
                          g_fastTable[g_fastIndex].reg,
                          g_fastTable[g_fastIndex].name, g_fastMiss);
                if (g_fastMiss >= HT7017_MAX_MISS_COUNT) {
                    g_fastIndex = (g_fastIndex + 1) % FAST_TABLE_SIZE;
                    g_fastMiss  = 0;
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: FAST skip -> idx=%u", g_fastIndex);
                }
            } else {
                g_slowMiss++;
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: No reply SLOW reg=0x%02X [%s] miss=%u",
                          g_slowTable[g_slowIndex].reg,
                          g_slowTable[g_slowIndex].name, g_slowMiss);
                if (g_slowMiss >= HT7017_MAX_MISS_COUNT) {
                    g_slowIndex = (g_slowIndex + 1) % SLOW_TABLE_SIZE;
                    g_slowMiss  = 0;
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: SLOW skip -> idx=%u", g_slowIndex);
                }
            }
        }

        UART_ConsumeBytes(UART_GetDataSize());
    }

    // ── Step 2: decide next TX ────────────────────────────────────────────────
    g_callCount++;

    if (g_callCount % HT7017_FAST_RATIO == 0) {
        g_slowIndex   = (g_slowIndex + 1) % SLOW_TABLE_SIZE;
        g_lastWasFast = false;
        uint8_t reg   = g_slowTable[g_slowIndex].reg;
        HT7017_SendRequest(reg);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: TX 6A %02X [SLOW:%s] tx=%u",
                  reg, g_slowTable[g_slowIndex].name, g_txCount);
    } else {
        g_fastIndex   = (g_fastIndex + 1) % FAST_TABLE_SIZE;
        g_lastWasFast = true;
        uint8_t reg   = g_fastTable[g_fastIndex].reg;
        HT7017_SendRequest(reg);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: TX 6A %02X [FAST:%s] tx=%u",
                  reg, g_fastTable[g_fastIndex].name, g_txCount);
    }
}

void HT7017_RunQuick(void)
{
    if (UART_GetDataSize() < HT7017_RESPONSE_LEN) return;
    uint8_t d2 = UART_GetByte(0);
    uint8_t d1 = UART_GetByte(1);
    uint8_t d0 = UART_GetByte(2);
    uint8_t cs = UART_GetByte(3);
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);

    if (g_lastWasFast) {
        HT7017_ProcessFrame(d2, d1, d0, cs, &g_fastTable[g_fastIndex]);
    } else {
        HT7017_ProcessFrame(d2, d1, d0, cs, &g_slowTable[g_slowIndex]);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION J — HTTP PAGE
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

    const char *alarm_str = "OK";
    const char *alarm_col = "#000";
    switch (g_alarmState) {
        case 1: alarm_str = "OV TRIP"; alarm_col = "#c00"; break;
        case 2: alarm_str = "UV TRIP"; alarm_col = "#a50"; break;
        case 3: alarm_str = "OC TRIP"; alarm_col = "#c00"; break;
        case 4: alarm_str = "OP TRIP"; alarm_col = "#c00"; break;
    }

    uint32_t s1_age = (g_callCount > g_s1_last_update_tick)
                    ? (g_callCount - g_s1_last_update_tick) : 0;

    char tmp[1700];
    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 v15 - KWS-303WF</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>Voltage</td><td><b>%.2f V</b></td></tr>"
        "<tr><td>Current</td><td><b>%.3f A</b></td></tr>"
        "<tr><td>Frequency</td><td><b>%.3f Hz</b></td></tr>"
        "<tr><td>Active Power</td><td><b>%.2f W</b></td></tr>"
        "<tr><td>Reactive Power</td><td><b>%.2f VAR</b></td></tr>"
        "<tr><td>Apparent (V*I)</td><td><b>%.2f VA</b></td></tr>"
        "<tr><td>Apparent (S1, age %us)</td><td><b>%.2f VA</b></td></tr>"
        "<tr><td>Power Factor</td><td><b>%.4f</b></td></tr>"
        "<tr><td>Active Energy</td><td><b>%.4f Wh (%.5f kWh)</b></td></tr>"
        "<tr><td>Reactive Energy</td><td><b>%.4f VARh</b></td></tr>"
        "<tr><td>EMUSR (HW 0x%02X)</td><td><b>"
          "OV=%u UV=%u OC=%u OP=%u ZXLOSS=%u</b></td></tr>"
        "<tr><td>SW Protection</td><td><b style='color:%s'>%s</b>"
          " OV=%.0f UV=%.0f OC=%.2f OP=%.0f</td></tr>"
        "<tr><td>Session</td><td><b>%ud %02uh %02um</b></td></tr>"
        "<tr><td colspan='2' style='font-size:0.8em;color:#666'>"
          "good=%u bad=%u miss=%u tx=%u | "
          "V=%.0f I=%.0f P=%.3f F=%.0f Q=%.3f S=%.3f EP=%.3f"
        "</td></tr>"
        "</table>",
        g_voltage, g_current, g_freq,
        g_power, g_reactive,
        g_apparent, s1_age, g_apparent_s1,
        g_power_factor,
        g_wh_total, g_wh_total / 1000.0f,
        g_varh_total,
        g_emusr_raw,
        (g_emusr_raw & HT7017_EMUSR_BIT_OV)     ? 1u : 0u,
        (g_emusr_raw & HT7017_EMUSR_BIT_UV)     ? 1u : 0u,
        (g_emusr_raw & HT7017_EMUSR_BIT_OC)     ? 1u : 0u,
        (g_emusr_raw & HT7017_EMUSR_BIT_OP)     ? 1u : 0u,
        (g_emusr_raw & HT7017_EMUSR_BIT_ZXLOSS) ? 1u : 0u,
        alarm_col, alarm_str,
        g_ovThreshold, g_uvThreshold, g_ocThreshold, g_opThreshold,
        days, hrs, mins,
        g_goodFrames, g_badFrames, g_totalMisses, g_txCount,
        g_vScale, g_iScale, g_pScale, g_fScale, g_qScale, g_sScale,
        g_e1Scale);

    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION K — GETTERS
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
float    HT7017_GetReactivePower(void)  { return g_reactive;      }
float    HT7017_GetApparentS1(void)     { return g_apparent_s1;   }
float    HT7017_GetWh(void)             { return g_wh_total;      }
float    HT7017_GetVARh(void)           { return g_varh_total;    }
uint32_t HT7017_GetEmusr(void)          { return g_emusr_raw;     }
uint8_t  HT7017_GetAlarmState(void)     { return g_alarmState;    }

#endif // ENABLE_DRIVER_HT7017
