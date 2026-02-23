/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v10
 *
 * Clean build — no channel.h, no powerMeasurementCalibration, no flash API.
 * Scale factors are compile-time #defines in drv_ht7017.h.
 * VoltageSet/CurrentSet/PowerSet update runtime variables only (no flash save).
 * Channel publishing via CHANNEL_Set() from new_pins.h (already included).
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

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ─── Runtime scale factors (from #defines in header, updated by VoltageSet etc) ──
static float g_vScale  = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale  = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale  = HT7017_DEFAULT_FREQ_SCALE;
static float g_iOffset = HT7017_CURRENT_OFFSET;

// ─── Last raw readings (for VoltageSet/CurrentSet/PowerSet) ──────────────────
static uint32_t g_lastRawV = 0;
static uint32_t g_lastRawI = 0;
static uint32_t g_lastRawP = 0;

// ─── Measurements ─────────────────────────────────────────────────────────────
static float g_voltage      = 0.0f;
static float g_current      = 0.0f;
static float g_power        = 0.0f;
static float g_freq         = 0.0f;
static float g_apparent     = 0.0f;
static float g_power_factor = 0.0f;

// ─── Driver state ─────────────────────────────────────────────────────────────
static uint8_t  g_regIndex    = 0;
static uint8_t  g_missCount   = 0;
static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;
static uint32_t g_totalMisses = 0;

// ─── Register table ───────────────────────────────────────────────────────────
typedef struct {
    uint8_t     reg;
    float      *target;
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint32_t   *last_raw;
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

static RegRead_t g_regTable[5];

static void HT7017_BuildRegTable(void)
{
    g_regTable[0].reg        = HT7017_REG_RMS_U;
    g_regTable[0].target     = &g_voltage;
    g_regTable[0].scale      = &g_vScale;
    g_regTable[0].is_signed  = 0;
    g_regTable[0].has_offset = 0;
    g_regTable[0].last_raw   = &g_lastRawV;
    g_regTable[0].name       = "Voltage";
    g_regTable[0].unit       = "V";
    g_regTable[0].obk_ch     = HT7017_CHANNEL_VOLTAGE;

    g_regTable[1].reg        = HT7017_REG_RMS_I1;
    g_regTable[1].target     = &g_current;
    g_regTable[1].scale      = &g_iScale;
    g_regTable[1].is_signed  = 0;
    g_regTable[1].has_offset = 1;
    g_regTable[1].last_raw   = &g_lastRawI;
    g_regTable[1].name       = "Current";
    g_regTable[1].unit       = "A";
    g_regTable[1].obk_ch     = HT7017_CHANNEL_CURRENT;

    g_regTable[2].reg        = HT7017_REG_POWER_P1;
    g_regTable[2].target     = &g_power;
    g_regTable[2].scale      = &g_pScale;
    g_regTable[2].is_signed  = 1;
    g_regTable[2].has_offset = 0;
    g_regTable[2].last_raw   = &g_lastRawP;
    g_regTable[2].name       = "Power";
    g_regTable[2].unit       = "W";
    g_regTable[2].obk_ch     = HT7017_CHANNEL_POWER;

    g_regTable[3].reg        = HT7017_REG_FREQ;
    g_regTable[3].target     = &g_freq;
    g_regTable[3].scale      = &g_fScale;
    g_regTable[3].is_signed  = 0;
    g_regTable[3].has_offset = 0;
    g_regTable[3].last_raw   = NULL;
    g_regTable[3].name       = "Freq";
    g_regTable[3].unit       = "Hz";
    g_regTable[3].obk_ch     = HT7017_CHANNEL_FREQ;

    // S1 not yet calibrated — not published, scale placeholder
    g_regTable[4].reg        = HT7017_REG_POWER_S1;
    g_regTable[4].target     = &g_apparent;
    g_regTable[4].scale      = &g_fScale;
    g_regTable[4].is_signed  = 0;
    g_regTable[4].has_offset = 0;
    g_regTable[4].last_raw   = NULL;
    g_regTable[4].name       = "Apparent";
    g_regTable[4].unit       = "VA";
    g_regTable[4].obk_ch     = 0;
}
#define REG_TABLE_SIZE 5

// ─── Helpers ──────────────────────────────────────────────────────────────────
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
    return val / (*r->scale);
}

static void HT7017_UpdatePowerFactor(void)
{
    float apparent = g_voltage * g_current;
    if (apparent > 0.5f && g_power > 0.0f) {
        g_power_factor = g_power / apparent;
        if (g_power_factor > 1.0f) g_power_factor = 1.0f;
    } else {
        g_power_factor = 0.0f;
    }
}

// ─── Channel publishing ────────────────────────────────────────────────────────
// Uses CMD_ExecuteCommand("setChannel N VALUE") — no channel.h needed.
// Values published as integer * 100 (e.g. 22751 = 227.51V).
// Configure matching channels in OpenBeken as "divided by 100" type.
static void HT7017_PublishChannel(uint8_t ch, float value)
{
    if (ch == 0) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "setChannel %u %d", ch, (int)(value * 100.0f));
    CMD_ExecuteCommand(buf, 0);
}

// ─── Frame processing ─────────────────────────────────────────────────────────
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
    if (r->last_raw) *r->last_raw = raw;

    float value = HT7017_Convert(raw, r);
    *r->target  = value;
    g_goodFrames++;
    g_missCount = 0;

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

    HT7017_PublishChannel(r->obk_ch, value);
    HT7017_UpdatePowerFactor();
    if (HT7017_CHANNEL_PF > 0) {
        HT7017_PublishChannel(HT7017_CHANNEL_PF, g_power_factor);
    }
}

static void HT7017_SendRequest(uint8_t reg)
{
    UART_ConsumeBytes(UART_GetDataSize());
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ─── VoltageSet / CurrentSet / PowerSet ───────────────────────────────────────
// Updates runtime scale only — no flash save.
// To make permanent: update the #define in drv_ht7017.h and recompile.

static commandResult_t CMD_VoltageSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;
    if (g_lastRawV == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: VoltageSet — no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    float old = g_vScale;
    g_vScale  = (float)g_lastRawV / actual;
    g_voltage = (float)g_lastRawV / g_vScale;
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
                  "HT7017: CurrentSet — no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    float net = (float)g_lastRawI - g_iOffset;
    if (net <= 0.0f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CurrentSet — net=%.0f too low, load too small?", net);
        return CMD_RES_ERROR;
    }
    float old = g_iScale;
    g_iScale  = net / actual;
    g_current = net / g_iScale;
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
                  "HT7017: PowerSet — no reading yet, wait one cycle");
        return CMD_RES_ERROR;
    }
    int32_t s = (int32_t)g_lastRawP;
    if (s & 0x800000) s |= (int32_t)0xFF000000;
    if (s == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: PowerSet — signed raw=0, load connected?");
        return CMD_RES_ERROR;
    }
    float old = g_pScale;
    g_pScale  = (float)s / actual;
    g_power   = (float)s / g_pScale;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: PowerSet %.1fW raw=%u signed=%d scale %.4f->%.4f "
              "(update HT7017_DEFAULT_POWER_SCALE to make permanent)",
              actual, g_lastRawP, s, old, g_pScale);
    return CMD_RES_OK;
}

// ─── HT7017_Status ────────────────────────────────────────────────────────────
static commandResult_t CMD_Status(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 v10 Status ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Voltage      : %.2f V",  g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Current      : %.3f A",  g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Power : %.1f W",  g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Apparent Pwr : %.1f VA", g_voltage * g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Power Factor : %.3f",    g_power_factor);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Frequency    : %.3f Hz", g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Scales       : V=%.2f I=%.2f P=%.4f F=%.1f offset=%.0f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_iOffset);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Last raw     : V=%u I=%u P=%u",
              g_lastRawV, g_lastRawI, g_lastRawP);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Frames       : good=%u bad=%u miss=%u tx=%u",
              g_goodFrames, g_badFrames, g_totalMisses, g_txCount);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  To recalib   : VoltageSet <V> / CurrentSet <A> / PowerSet <W>");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  To persist   : copy scale from log to #define in drv_ht7017.h");
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

// ─── Init ─────────────────────────────────────────────────────────────────────
void HT7017_Init(void)
{
    HT7017_BuildRegTable();

    // Start on last index so first increment → index 0 (Voltage reg 0x08)
    g_regIndex    = REG_TABLE_SIZE - 1;
    g_missCount   = 0;
    g_totalMisses = 0;
    g_txCount     = 0;
    g_goodFrames  = 0;
    g_badFrames   = 0;

    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("VoltageSet",      CMD_VoltageSet, NULL);
    CMD_RegisterCommand("CurrentSet",      CMD_CurrentSet, NULL);
    CMD_RegisterCommand("PowerSet",        CMD_PowerSet,   NULL);
    CMD_RegisterCommand("HT7017_Status",   CMD_Status,     NULL);
    CMD_RegisterCommand("HT7017_Baud",     CMD_Baud,       NULL);
    CMD_RegisterCommand("HT7017_NoParity", CMD_NoParity,   NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v10: KWS-303WF  BK7231N->HT7017 direct  shunt");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v10: %s  4800 8E1",
              CFG_HasFlag(26) ? "UART2 (Pin 6/7)" : "UART1 (P10=RX P11=TX)");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v10: V=%.2f I=%.2f P=%.4f F=%.1f offset=%.0f",
              g_vScale, g_iScale, g_pScale, g_fScale, g_iOffset);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 v10: Channels V=%u I=%u P=%u F=%u PF=%u",
              HT7017_CHANNEL_VOLTAGE, HT7017_CHANNEL_CURRENT,
              HT7017_CHANNEL_POWER,   HT7017_CHANNEL_FREQ,
              HT7017_CHANNEL_PF);
}

// ─── RunEverySecond ───────────────────────────────────────────────────────────
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

// ─── RunQuick ─────────────────────────────────────────────────────────────────
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

// ─── HTTP Status Page ─────────────────────────────────────────────────────────
void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[768];
    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 - KWS-303WF</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>Voltage</td><td><b>%.2f V</b></td></tr>"
        "<tr><td>Current</td><td><b>%.3f A</b></td></tr>"
        "<tr><td>Active Power</td><td><b>%.1f W</b></td></tr>"
        "<tr><td>Apparent Power</td><td><b>%.1f VA</b></td></tr>"
        "<tr><td>Power Factor</td><td><b>%.3f</b></td></tr>"
        "<tr><td>Frequency</td><td><b>%.3f Hz</b></td></tr>"
        "<tr><td colspan='2' style='font-size:0.8em;color:#666'>"
          "good=%u bad=%u miss=%u tx=%u | "
          "V=%.0f I=%.0f P=%.3f"
        "</td></tr>"
        "</table>",
        g_voltage, g_current, g_power,
        g_voltage * g_current,
        g_power_factor, g_freq,
        g_goodFrames, g_badFrames, g_totalMisses, g_txCount,
        g_vScale, g_iScale, g_pScale);

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
