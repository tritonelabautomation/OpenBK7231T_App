/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v13
 *
 * v13 CHANGE — Two-speed polling for EV charging use case:
 *
 *   PROBLEM (v12 and earlier):
 *     Single rotation of 12 registers, one per second.
 *     V, I, P each appeared at most twice in the 12-slot table → updated
 *     every 6 seconds at best. With a load connecting/disconnecting during
 *     EV charging this lag is unacceptable — current could show 0A for up
 *     to 6 seconds after the charger starts or stops.
 *
 *   SOLUTION:
 *     Two separate register tables, each with its own index pointer.
 *     Scheduling lives entirely inside HT7017_RunEverySecond() using a
 *     simple call counter — zero changes to OpenBeken glue code.
 *
 *     FAST table: V, I, P1  (3 registers)
 *     SLOW table: Q1, S1, FREQ, EP1, EQ1  (5 registers)
 *
 *     g_callCount increments every second.
 *     if (g_callCount % HT7017_FAST_RATIO != 0) → poll next FAST register
 *     if (g_callCount % HT7017_FAST_RATIO == 0) → poll next SLOW register
 *
 *     With HT7017_FAST_RATIO=5:
 *       4 calls out of 5 → fast register  (V/I/P each updated every ~1.7s)
 *       1 call out of 5  → slow register  (Q/S/F/EP1/EQ1 each updated every 25s)
 *
 *   HALF-DUPLEX SAFETY:
 *     Exactly ONE TX per RunEverySecond() call. No change to the TX/RX
 *     state machine — whichever table was last sent, we process its reply
 *     first, then decide which table sends next. A single g_lastWasFast flag
 *     tells ProcessFrame which table's index to look up.
 *
 *   MISS HANDLING:
 *     Each table has its own miss counter. A miss on a fast register advances
 *     the fast index. A miss on a slow register advances the slow index.
 *     They cannot interfere with each other.
 *
 *   ALL v12 FIXES PRESERVED:
 *     FIX 1 — FREQ formula: g_fScale/raw (500000/raw) not raw/g_fScale
 *     FIX 2 — PFCNT removed: 0x10 is PowerP2, not PFCNT
 *     FIX 3 — g_sScale: fabsf(g_pScale) in CMD_PowerSet
 *     FIX 4 — Energy seeded flags: bool not 0xFFFFFFFF sentinel
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
static float g_apparent     = 0.0f;   // V * I
static float g_power_factor = 0.0f;

// Energy accumulators — bool seeded flags (v12 FIX 4)
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
// SECTION B — REGISTER TABLE (shared type, two instances)
// ═══════════════════════════════════════════════════════════════════════════════

typedef struct {
    uint8_t     reg;
    float      *target;
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint8_t     is_energy;
    uint32_t   *last_raw_acc;
    bool       *seeded_flag;
    float      *acc_total;
    uint32_t   *last_raw_cal;
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

// ── FAST table: V, I, P — the three registers that matter for EV safety ───────
#define FAST_TABLE_SIZE 3
static RegRead_t g_fastTable[FAST_TABLE_SIZE];

// ── SLOW table: Q, S, FREQ, EP1, EQ1 — housekeeping, polled every 25s ────────
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
    g_fastTable[2].last_raw_acc = NULL;
    g_fastTable[2].seeded_flag  = NULL;
    g_fastTable[2].acc_total    = NULL;
    g_fastTable[2].last_raw_cal = &g_lastRawP;
    g_fastTable[2].name         = "Power";
    g_fastTable[2].unit         = "W";
    g_fastTable[2].obk_ch       = HT7017_CHANNEL_POWER;

    // ── SLOW TABLE ────────────────────────────────────────────────────────────

    // S0: Reactive Power Q1 (SIGNED)
    g_slowTable[0].reg          = HT7017_REG_POWER_Q1;
    g_slowTable[0].target       = &g_reactive;
    g_slowTable[0].scale        = &g_qScale;
    g_slowTable[0].is_signed    = 1;
    g_slowTable[0].has_offset   = 0;
    g_slowTable[0].is_energy    = 0;
    g_slowTable[0].last_raw_acc = NULL;
    g_slowTable[0].seeded_flag  = NULL;
    g_slowTable[0].acc_total    = NULL;
    g_slowTable[0].last_raw_cal = NULL;
    g_slowTable[0].name         = "Reactive";
    g_slowTable[0].unit         = "VAR";
    g_slowTable[0].obk_ch       = HT7017_CHANNEL_REACTIVE;

    // S1: Apparent Power S1 (unsigned, own scale)
    g_slowTable[1].reg          = HT7017_REG_POWER_S1;
    g_slowTable[1].target       = &g_apparent_s1;
    g_slowTable[1].scale        = &g_sScale;
    g_slowTable[1].is_signed    = 0;
    g_slowTable[1].has_offset   = 0;
    g_slowTable[1].is_energy    = 0;
    g_slowTable[1].last_raw_acc = NULL;
    g_slowTable[1].seeded_flag  = NULL;
    g_slowTable[1].acc_total    = NULL;
    g_slowTable[1].last_raw_cal = NULL;
    g_slowTable[1].name         = "Apparent";
    g_slowTable[1].unit         = "VA";
    g_slowTable[1].obk_ch       = HT7017_CHANNEL_APPARENT;

    // S2: Frequency (period counter — formula: g_fScale/raw)
    g_slowTable[2].reg          = HT7017_REG_FREQ;
    g_slowTable[2].target       = &g_freq;
    g_slowTable[2].scale        = &g_fScale;
    g_slowTable[2].is_signed    = 0;
    g_slowTable[2].has_offset   = 0;
    g_slowTable[2].is_energy    = 0;
    g_slowTable[2].last_raw_acc = NULL;
    g_slowTable[2].seeded_flag  = NULL;
    g_slowTable[2].acc_total    = NULL;
    g_slowTable[2].last_raw_cal = NULL;
    g_slowTable[2].name         = "Freq";
    g_slowTable[2].unit         = "Hz";
    g_slowTable[2].obk_ch       = HT7017_CHANNEL_FREQ;

    // S3: Active Energy EP1 (unsigned accumulator)
    g_slowTable[3].reg          = HT7017_REG_EP1;
    g_slowTable[3].target       = &g_wh_total;
    g_slowTable[3].scale        = &g_e1Scale;
    g_slowTable[3].is_signed    = 0;
    g_slowTable[3].has_offset   = 0;
    g_slowTable[3].is_energy    = 1;
    g_slowTable[3].last_raw_acc = &g_ep1_last;
    g_slowTable[3].seeded_flag  = &g_ep1_seeded;
    g_slowTable[3].acc_total    = &g_wh_total;
    g_slowTable[3].last_raw_cal = NULL;
    g_slowTable[3].name         = "ActiveEnergy";
    g_slowTable[3].unit         = "Wh";
    g_slowTable[3].obk_ch       = HT7017_CHANNEL_WH;

    // S4: Reactive Energy EQ1 (unsigned accumulator)
    g_slowTable[4].reg          = HT7017_REG_EQ1;
    g_slowTable[4].target       = &g_varh_total;
    g_slowTable[4].scale        = &g_e1Scale;
    g_slowTable[4].is_signed    = 0;
    g_slowTable[4].has_offset   = 0;
    g_slowTable[4].is_energy    = 1;
    g_slowTable[4].last_raw_acc = &g_eq1_last;
    g_slowTable[4].seeded_flag  = &g_eq1_seeded;
    g_slowTable[4].acc_total    = &g_varh_total;
    g_slowTable[4].last_raw_cal = NULL;
    g_slowTable[4].name         = "ReactiveEnergy";
    g_slowTable[4].unit         = "VARh";
    g_slowTable[4].obk_ch       = HT7017_CHANNEL_VARH;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION C — SCHEDULING STATE
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t  g_fastIndex   = 0;      // current position in fast table
static uint8_t  g_slowIndex   = 0;      // current position in slow table
static uint32_t g_callCount   = 0;      // incremented every RunEverySecond() call
static bool     g_lastWasFast = true;   // which table's reply to expect next
static uint8_t  g_fastMiss    = 0;      // consecutive miss count for fast table
static uint8_t  g_slowMiss    = 0;      // consecutive miss count for slow table

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D — HELPERS (identical to v12)
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t HT7017_Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

static float HT7017_Convert(uint32_t raw, const RegRead_t *r)
{
    // FIX 1 (v12): FREQ is a period counter — formula is g_fScale/raw not raw/g_fScale
    if (r->reg == HT7017_REG_FREQ) {
        if (raw == 0) return 0.0f;
        return (*r->scale) / (float)raw;  // 500000 / raw
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

static void HT7017_UpdateDerived(void)
{
    g_apparent = g_voltage * g_current;

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
// SECTION E — ENERGY ACCUMULATOR (identical to v12)
// ═══════════════════════════════════════════════════════════════════════════════

static void HT7017_AccumulateEnergy(uint32_t raw, const RegRead_t *r)
{
    // FIX 4 (v12): bool seeded flag — 0xFFFFFFFF is a valid 24-bit counter value
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
// SECTION F — FRAME PROCESSING (works for both tables)
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

    if (r->is_energy) {
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

    HT7017_UpdateDerived();
    if (HT7017_CHANNEL_PF > 0) {
        HT7017_PublishChannel(HT7017_CHANNEL_PF, g_power_factor);
    }

    if (g_session_start_unix == 0) {
        uint32_t now = NTP_GetCurrentTime();
        if (now > 1000000000u) {
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
// SECTION G — CONSOLE COMMANDS (identical to v12)
// ═══════════════════════════════════════════════════════════════════════════════

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
    g_qScale  = g_pScale;
    g_sScale  = fabsf(g_pScale);   // FIX 3 (v12)
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

static commandResult_t CMD_EnergyReset(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    g_wh_total   = 0.0f;
    g_varh_total = 0.0f;
    g_ep1_seeded = false;   // FIX 4 (v12)
    g_eq1_seeded = false;

    uint32_t now = NTP_GetCurrentTime();
    g_energy_reset_unix = (now > 1000000000u) ? now : 0;

    HT7017_PublishChannel(HT7017_CHANNEL_WH,   0.0f);
    HT7017_PublishChannel(HT7017_CHANNEL_VARH, 0.0f);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Energy counters reset to 0  (unix=%u)", g_energy_reset_unix);
    return CMD_RES_OK;
}

static commandResult_t CMD_Energy(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    uint32_t now     = NTP_GetCurrentTime();
    uint32_t since   = (g_energy_reset_unix > 0) ? g_energy_reset_unix : g_session_start_unix;
    uint32_t elapsed = (now > since && since > 0) ? (now - since) : 0;

    float hours    = (float)elapsed / 3600.0f;
    float kw_avg   = (hours > 0.0f) ? (g_wh_total  / 1000.0f / hours) : 0.0f;
    float kvar_avg = (hours > 0.0f) ? (g_varh_total / 1000.0f / hours) : 0.0f;

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

static commandResult_t CMD_Status(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 v13 Status ══╗");
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
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Scales  V=%.2f I=%.2f P=%.4f F=%.0f Q=%.4f S=%.4f",
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
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Polling fast=%u/5 calls  fastIdx=%u  slowIdx=%u  callCount=%u",
              HT7017_FAST_RATIO - 1, g_fastIndex, g_slowIndex, g_callCount);
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
// SECTION H — INIT
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_Init(void)
{
    HT7017_BuildTables();

    // Start fast index at last slot so first increment lands on slot 0 (Voltage)
    g_fastIndex   = FAST_TABLE_SIZE - 1;
    g_slowIndex   = SLOW_TABLE_SIZE - 1;
    g_callCount   = 0;
    g_lastWasFast = true;
    g_fastMiss    = 0;
    g_slowMiss    = 0;
    g_txCount     = 0;
    g_goodFrames  = 0;
    g_badFrames   = 0;
    g_totalMisses = 0;

    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("VoltageSet",          CMD_VoltageSet,  NULL);
    CMD_RegisterCommand("CurrentSet",          CMD_CurrentSet,  NULL);
    CMD_RegisterCommand("PowerSet",            CMD_PowerSet,    NULL);
    CMD_RegisterCommand("HT7017_Energy_Reset", CMD_EnergyReset, NULL);
    CMD_RegisterCommand("HT7017_Energy",       CMD_Energy,      NULL);
    CMD_RegisterCommand("HT7017_Status",       CMD_Status,      NULL);
    CMD_RegisterCommand("HT7017_Baud",         CMD_Baud,        NULL);
    CMD_RegisterCommand("HT7017_NoParity",     CMD_NoParity,    NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: KWS-303WF  BK7231N->HT7017 direct  shunt");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: %s  4800 8E1",
              CFG_HasFlag(26) ? "UART2" : "UART1 (P0/P1)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: two-speed poll  FAST(V,I,P) every ~1.7s  "
              "SLOW(Q,S,F,EP1,EQ1) every ~25s");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: V=%.2f I=%.2f P=%.4f F=%.0f offset=%.0f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_iOffset);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: Q=%.4f S=%.4f EP1=%.4f",
              g_qScale, g_sScale, g_e1Scale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v13: Ch V=%u I=%u P=%u F=%u PF=%u Q=%u S=%u Wh=%u VARh=%u",
              HT7017_CHANNEL_VOLTAGE, HT7017_CHANNEL_CURRENT,
              HT7017_CHANNEL_POWER,   HT7017_CHANNEL_FREQ,
              HT7017_CHANNEL_PF,      HT7017_CHANNEL_REACTIVE,
              HT7017_CHANNEL_APPARENT,HT7017_CHANNEL_WH,
              HT7017_CHANNEL_VARH);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION I — RUN (v13 two-speed scheduler)
// ═══════════════════════════════════════════════════════════════════════════════

void HT7017_RunEverySecond(void)
{
    // ── Step 1: process reply from whatever we sent last call ─────────────────
    int available = UART_GetDataSize();

    if (available >= HT7017_RESPONSE_LEN) {
        uint8_t d2 = UART_GetByte(0);
        uint8_t d1 = UART_GetByte(1);
        uint8_t d0 = UART_GetByte(2);
        uint8_t cs = UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);

        // Route reply to whichever table sent last
        if (g_lastWasFast) {
            HT7017_ProcessFrame(d2, d1, d0, cs, &g_fastTable[g_fastIndex]);
            g_fastMiss = 0;
        } else {
            HT7017_ProcessFrame(d2, d1, d0, cs, &g_slowTable[g_slowIndex]);
            g_slowMiss = 0;
        }

    } else {
        // No reply received
        g_totalMisses++;

        if (g_txCount > 0) {
            // Only log after first TX so boot silence is clean
            if (g_lastWasFast) {
                g_fastMiss++;
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: No response FAST reg=0x%02X [%s] miss=%u",
                          g_fastTable[g_fastIndex].reg,
                          g_fastTable[g_fastIndex].name,
                          g_fastMiss);
                if (g_fastMiss >= HT7017_MAX_MISS_COUNT) {
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: FAST skip after %u misses", g_fastMiss);
                    g_fastIndex = (g_fastIndex + 1) % FAST_TABLE_SIZE;
                    g_fastMiss  = 0;
                }
            } else {
                g_slowMiss++;
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: No response SLOW reg=0x%02X [%s] miss=%u",
                          g_slowTable[g_slowIndex].reg,
                          g_slowTable[g_slowIndex].name,
                          g_slowMiss);
                if (g_slowMiss >= HT7017_MAX_MISS_COUNT) {
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: SLOW skip after %u misses", g_slowMiss);
                    g_slowIndex = (g_slowIndex + 1) % SLOW_TABLE_SIZE;
                    g_slowMiss  = 0;
                }
            }
        }

        UART_ConsumeBytes(UART_GetDataSize());
    }

    // ── Step 2: decide which table to poll next and advance its index ─────────
    g_callCount++;

    if (g_callCount % HT7017_FAST_RATIO == 0) {
        // This call belongs to the slow table
        g_slowIndex   = (g_slowIndex + 1) % SLOW_TABLE_SIZE;
        g_lastWasFast = false;
        uint8_t reg   = g_slowTable[g_slowIndex].reg;
        HT7017_SendRequest(reg);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: TX > 6A %02X [SLOW:%s] (tx=%u)",
                  reg, g_slowTable[g_slowIndex].name, g_txCount);
    } else {
        // This call belongs to the fast table
        g_fastIndex   = (g_fastIndex + 1) % FAST_TABLE_SIZE;
        g_lastWasFast = true;
        uint8_t reg   = g_fastTable[g_fastIndex].reg;
        HT7017_SendRequest(reg);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: TX > 6A %02X [FAST:%s] (tx=%u)",
                  reg, g_fastTable[g_fastIndex].name, g_txCount);
    }
}

void HT7017_RunQuick(void)
{
    // Unchanged from v12 — ISR path, no index advancement
    if (UART_GetDataSize() < HT7017_RESPONSE_LEN) return;
    uint8_t d2 = UART_GetByte(0);
    uint8_t d1 = UART_GetByte(1);
    uint8_t d0 = UART_GetByte(2);
    uint8_t cs = UART_GetByte(3);
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);

    // Route to whichever table is currently active
    if (g_lastWasFast) {
        HT7017_ProcessFrame(d2, d1, d0, cs, &g_fastTable[g_fastIndex]);
    } else {
        HT7017_ProcessFrame(d2, d1, d0, cs, &g_slowTable[g_slowIndex]);
    }
    // Do NOT advance any index — RunEverySecond manages all index movement
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION J — HTTP PAGE (v12 unchanged, version bump only)
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
        "<h5>HT7017 v13 - KWS-303WF</h5>"
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
// SECTION K — GETTERS (unchanged)
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

#endif // ENABLE_DRIVER_HT7017
