/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v9
 *
 * v9 changes:
 * - Fixed linker errors by using standard CMD_ExecuteCommand
 * - Replaced guessed CFG_ flash functions with 100% safe LittleFS (LFS_WriteFile)
 * - GUI calibration (VoltageSet/CurrentSet/PowerSet) works and saves to LFS
 */

#include "../obk_config.h"
#if ENABLE_DRIVER_HT7017

#include "drv_ht7017.h"
#include "drv_uart.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include "../littlefs/our_lfs.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ─── Runtime scale factors (loaded from flash or factory default) ─────────────
static float g_vScale   = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale   = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale   = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale   = HT7017_DEFAULT_FREQ_SCALE;
static float g_iOffset  = HT7017_CURRENT_OFFSET;

// ─── Last raw readings — needed by VoltageSet/CurrentSet/PowerSet ─────────────
static uint32_t g_lastRawV = 0;
static uint32_t g_lastRawI = 0;
static uint32_t g_lastRawP = 0;   // stored as uint, sign-extended when used

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
    float      *scale;      // pointer to runtime scale
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint32_t   *last_raw;   // pointer to last-raw storage for calibration
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

static RegRead_t g_regTable[5];

static void HT7017_BuildRegTable(void)
{
    // Voltage
    g_regTable[0].reg       = HT7017_REG_RMS_U;
    g_regTable[0].target    = &g_voltage;
    g_regTable[0].scale     = &g_vScale;
    g_regTable[0].is_signed = 0;
    g_regTable[0].has_offset= 0;
    g_regTable[0].last_raw  = &g_lastRawV;
    g_regTable[0].name      = "Voltage";
    g_regTable[0].unit      = "V";
    g_regTable[0].obk_ch    = HT7017_CHANNEL_VOLTAGE;

    // Current
    g_regTable[1].reg       = HT7017_REG_RMS_I1;
    g_regTable[1].target    = &g_current;
    g_regTable[1].scale     = &g_iScale;
    g_regTable[1].is_signed = 0;
    g_regTable[1].has_offset= 1;
    g_regTable[1].last_raw  = &g_lastRawI;
    g_regTable[1].name      = "Current";
    g_regTable[1].unit      = "A";
    g_regTable[1].obk_ch    = HT7017_CHANNEL_CURRENT;

    // Power
    g_regTable[2].reg       = HT7017_REG_POWER_P1;
    g_regTable[2].target    = &g_power;
    g_regTable[2].scale     = &g_pScale;
    g_regTable[2].is_signed = 1;
    g_regTable[2].has_offset= 0;
    g_regTable[2].last_raw  = &g_lastRawP;
    g_regTable[2].name      = "Power";
    g_regTable[2].unit      = "W";
    g_regTable[2].obk_ch    = HT7017_CHANNEL_POWER;

    // Frequency
    g_regTable[3].reg       = HT7017_REG_FREQ;
    g_regTable[3].target    = &g_freq;
    g_regTable[3].scale     = &g_fScale;
    g_regTable[3].is_signed = 0;
    g_regTable[3].has_offset= 0;
    g_regTable[3].last_raw  = NULL;
    g_regTable[3].name      = "Freq";
    g_regTable[3].unit      = "Hz";
    g_regTable[3].obk_ch    = HT7017_CHANNEL_FREQ;

    // Apparent power
    g_regTable[4].reg       = HT7017_REG_POWER_S1;
    g_regTable[4].target    = &g_apparent;
    g_regTable[4].scale     = &g_pScale;
    g_regTable[4].is_signed = 0;
    g_regTable[4].has_offset= 0;
    g_regTable[4].last_raw  = NULL;
    g_regTable[4].name      = "Apparent";
    g_regTable[4].unit      = "VA";
    g_regTable[4].obk_ch    = 0;
}
#define REG_TABLE_SIZE 5

// ─── File System Persistence ──────────────────────────────────────────────────
static void HT7017_SaveScales(void)
{
    float data[4] = { g_vScale, g_iScale, g_pScale, g_iOffset };
    LFS_WriteFile("ht7017.dat", data, sizeof(data));
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Scales saved to LFS V=%.2f I=%.2f P=%.4f offset=%.0f",
              g_vScale, g_iScale, g_pScale, g_iOffset);
}

static void HT7017_LoadScales(void)
{
    float data[4];
    if (LFS_ReadFile("ht7017.dat", data, sizeof(data)) == sizeof(data)) {
        g_vScale  = data[0];
        g_iScale  = data[1];
        g_pScale  = data[2];
        g_iOffset = data[3];
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: Loaded scales from LFS");
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: No LFS scales found, using factory defaults");
    }
}

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

static void HT7017_SetChannel(uint8_t ch, float value)
{
    if (ch == 0) return;
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "setChannel %u %d", ch, (int)(value * 100.0f));
    CMD_ExecuteCommand(cmd, 0);
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

    // Store last raw for calibration commands to use
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

    HT7017_SetChannel(r->obk_ch, value);
    HT7017_UpdatePowerFactor();
    if (HT7017_CHANNEL_PF > 0) {
        HT7017_SetChannel(HT7017_CHANNEL_PF, g_power_factor);
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
static commandResult_t CMD_VoltageSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;

    if (g_lastRawV == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: VoltageSet — no raw reading yet");
        return CMD_RES_ERROR;
    }

    float old   = g_vScale;
    g_vScale    = (float)g_lastRawV / actual;
    g_voltage   = (float)g_lastRawV / g_vScale;
    HT7017_SaveScales();

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: VoltageSet %.2fV scale %.2f->%.2f", actual, old, g_vScale);
    return CMD_RES_OK;
}

static commandResult_t CMD_CurrentSet(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;

    if (g_lastRawI == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: CurrentSet — no raw reading yet");
        return CMD_RES_ERROR;
    }

    float net   = (float)g_lastRawI - g_iOffset;
    if (net <= 0.0f) return CMD_RES_ERROR;

    float old   = g_iScale;
    g_iScale    = net / actual;
    g_current   = net / g_iScale;
    HT7017_SaveScales();

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: CurrentSet %.3fA scale %.2f->%.2f", actual, old, g_iScale);
    return CMD_RES_OK;
}

static commandResult_t CMD_PowerSet(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float actual = (float)atof(args);
    if (actual <= 0.0f) return CMD_RES_BAD_ARGUMENT;

    if (g_lastRawP == 0) return CMD_RES_ERROR;

    int32_t s = (int32_t)g_lastRawP;
    if (s & 0x800000) s |= (int32_t)0xFF000000;
    if (s == 0) return CMD_RES_ERROR;

    float old   = g_pScale;
    g_pScale    = (float)s / actual;
    g_power     = (float)s / g_pScale;
    HT7017_SaveScales();

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017: PowerSet %.1fW scale %.4f->%.4f", actual, old, g_pScale);
    return CMD_RES_OK;
}

// ─── HT7017_Status ────────────────────────────────────────────────────────────
static commandResult_t CMD_Status(const void *ctx, const char *cmd, const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══ HT7017 v9 Status ══╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Voltage      : %.2f V",  g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Current      : %.3f A",  g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Active Power : %.1f W",  g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Scales       : V=%.2f I=%.2f P=%.4f", g_vScale, g_iScale, g_pScale);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚══════════════════════╝");
    return CMD_RES_OK;
}

// ─── HT7017_Baud / HT7017_NoParity ───────────────────────────────────────────
static commandResult_t CMD_Baud(const void *ctx, const char *cmd, const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    UART_InitUART(atoi(args), HT7017_PARITY_EVEN, 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_NoParity(const void *ctx, const char *cmd, const char *args, int flags)
{
    UART_InitUART(HT7017_BAUD_RATE, 0, 0);
    return CMD_RES_OK;
}

// ─── Init ─────────────────────────────────────────────────────────────────────
void HT7017_Init(void)
{
    HT7017_LoadScales();
    HT7017_BuildRegTable();

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

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "HT7017 v9 Initialized");
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
        UART_ConsumeBytes(HT7017_RESPONSE_LEN); 

        HT7017_ProcessFrame(d2, d1, d0, cs);
        g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
        g_missCount = 0;

    } else {
        g_missCount++;
        g_totalMisses++;
        UART_ConsumeBytes(UART_GetDataSize());

        if (g_missCount >= HT7017_MAX_MISS_COUNT) {
            g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
            g_missCount = 0;
        }
    }

    uint8_t reg = g_regTable[g_regIndex].reg;
    HT7017_SendRequest(reg);
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
}

// ─── HTTP Status Page ─────────────────────────────────────────────────────────
void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[768];
    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 — KWS-303WF</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>Voltage</td><td><b>%.2f V</b></td></tr>"
        "<tr><td>Current</td><td><b>%.3f A</b></td></tr>"
        "<tr><td>Active Power</td><td><b>%.1f W</b></td></tr>"
        "<tr><td>Frequency</td><td><b>%.3f Hz</b></td></tr>"
        "<tr><td colspan='2' style='font-size:0.8em;color:#666'>"
          "Scales: V=%.0f I=%.0f P=%.3f"
        "</td></tr>"
        "</table>",
        g_voltage, g_current, g_power, g_freq,
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
