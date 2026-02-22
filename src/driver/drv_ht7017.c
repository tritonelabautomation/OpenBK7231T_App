/*
 * HT7017 Single-Phase Energy Metering IC Driver  v3
 * Manufacturer: HiTrendtech (钜泉光电科技)
 *
 * Protocol Summary (Datasheet §4.1):
 *   - UART 4800 baud, 9-bit frame (8 data + even parity + stop) = 8E1
 *   - Half-duplex, HT7017 is always the slave
 *   - Read:  Master sends → [0x6A][REG_ADDR]
 *            Slave sends  → [DATA2][DATA1][DATA0][CHECKSUM]  (4 bytes)
 *   - Checksum = ~(HEAD + REG + DATA2 + DATA1 + DATA0) & 0xFF
 *   - Raw value = (DATA2<<16) | (DATA1<<8) | DATA0  (24-bit, MSB first)
 *   - Inter-byte gap must be < 20ms or UART module auto-resets
 *
 * Bug history:
 *   v1: UART_ConsumeBytes() called BEFORE reading — discarded every response.
 *   v2: g_regIndex init to 0 but RunEverySecond increments before first send,
 *       so first request went to reg 0x06 (Current) not 0x08 (Voltage).
 *       Also: on missed response, code skipped register instead of retrying.
 *   v3 (this): g_regIndex inits to REG_TABLE_SIZE-1 so first increment
 *       wraps to 0 (Voltage register) on first call. Missed responses retry
 *       same register up to 3 times before skipping. Added g_readyToSend flag
 *       so RunEverySecond only fires after RunQuick confirms transaction done.
 *
 * Calibration (performed 2026-02-22):
 *   Voltage scale = 11015.3  (raw ~2,690,940 at 244.30V actual)
 *   Current/Power scales = TBD (need known resistive load measurement)
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

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ─── Register Rotation Table ─────────────────────────────────────────────────
typedef struct {
    uint8_t     reg;
    float      *target;
    float       scale;
    const char *name;
} RegRead_t;

static float g_voltage = 0.0f;
static float g_current = 0.0f;
static float g_power   = 0.0f;
static float g_freq    = 0.0f;

static const RegRead_t g_regTable[] = {
    { HT7017_REG_RMS_U,    &g_voltage, HT7017_VOLTAGE_SCALE, "Voltage(V)"  },
    { HT7017_REG_RMS_I1,   &g_current, HT7017_CURRENT_SCALE, "Current(A)"  },
    { HT7017_REG_POWER_P1, &g_power,   HT7017_POWER_SCALE,   "Power(W)"    },
    { HT7017_REG_FREQ,     &g_freq,    HT7017_FREQ_SCALE,     "Freq(Hz)"    },
};
#define REG_TABLE_SIZE  (sizeof(g_regTable) / sizeof(g_regTable[0]))

// ─── Driver State ─────────────────────────────────────────────────────────────
static uint8_t  g_regIndex    = 0;
static uint8_t  g_waiting     = 0;   // 1 = waiting for 4-byte response
static uint8_t  g_readyToSend = 1;   // 1 = safe to send next request
static uint8_t  g_missCount   = 0;   // consecutive misses on current register
static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;

// ─── Internal Helpers ─────────────────────────────────────────────────────────

static uint8_t HT7017_CalcChecksum(uint8_t reg,
                                    uint8_t d2, uint8_t d1, uint8_t d0)
{
    uint8_t sum = (uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0);
    return (uint8_t)(~sum);
}

static void HT7017_SendRequest(uint8_t reg)
{
    UART_ConsumeBytes(UART_GetDataSize()); // flush junk before sending
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ─── Console Commands ─────────────────────────────────────────────────────────

static commandResult_t CMD_HT7017_Baud(const void *context,
                                        const char *cmd,
                                        const char *args,
                                        int cmdFlags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int baud = atoi(args);
    if (baud <= 0) return CMD_RES_BAD_ARGUMENT;
    UART_InitUART(baud, HT7017_PARITY_EVEN, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud (8E1)", baud);
    return CMD_RES_OK;
}

static commandResult_t CMD_HT7017_NoParity(const void *context,
                                            const char *cmd,
                                            const char *args,
                                            int cmdFlags)
{
    UART_InitUART(HT7017_BAUD_RATE, 0, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud (8N1)", HT7017_BAUD_RATE);
    return CMD_RES_OK;
}

static commandResult_t CMD_HT7017_Status(const void *context,
                                          const char *cmd,
                                          const char *args,
                                          int cmdFlags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "=== HT7017 Status ===");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Voltage : %.2f V",  g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Current : %.3f A",  g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Power   : %.1f W",  g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Freq    : %.2f Hz", g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  TX=%u  Good=%u  Bad=%u  RegIdx=%u  Waiting=%u  Ready=%u",
              g_txCount, g_goodFrames, g_badFrames,
              g_regIndex, g_waiting, g_readyToSend);
    return CMD_RES_OK;
}

// ─── Public Driver Functions ──────────────────────────────────────────────────

void HT7017_Init(void)
{
    UART_InitReceiveRingBuffer(256);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("HT7017_Baud",     CMD_HT7017_Baud,     NULL);
    CMD_RegisterCommand("HT7017_NoParity", CMD_HT7017_NoParity, NULL);
    CMD_RegisterCommand("HT7017_Status",   CMD_HT7017_Status,   NULL);

    if (CFG_HasFlag(26)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Init OK — UART2 (Pin 6/7), 4800 8E1");
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Init OK — UART1 (P10=RX, P11=TX), 4800 8E1");
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: VScale=%.1f CScale=%.1f PScale=%.1f FScale=%.1f",
              HT7017_VOLTAGE_SCALE, HT7017_CURRENT_SCALE,
              HT7017_POWER_SCALE, HT7017_FREQ_SCALE);

    // KEY FIX v3: init to last index so first increment in RunEverySecond
    // wraps to 0 (Voltage = 0x08) on the very first call.
    g_regIndex    = (uint8_t)(REG_TABLE_SIZE - 1);
    g_waiting     = 0;
    g_readyToSend = 1;
    g_missCount   = 0;
    g_txCount     = 0;
    g_goodFrames  = 0;
    g_badFrames   = 0;
}

/*
 * HT7017_RunEverySecond
 *
 * Called once per second by the OpenBeken scheduler.
 * Sends a read request for the current register in the rotation.
 *
 * Flow:
 *   - If g_readyToSend == 0: RunQuick hasn't confirmed response yet, skip.
 *   - If g_waiting == 1: last request timed out — retry up to 3x, then skip.
 *   - Otherwise: advance to next register and send request.
 */
void HT7017_RunEverySecond(void)
{
    // Guard: don't send until RunQuick confirms the chip finished responding
    if (!g_readyToSend) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Chip still responding to reg 0x%02X, waiting...",
                  g_regTable[g_regIndex].reg);
        return;
    }

    if (g_waiting) {
        // Request was sent last second but no valid response arrived
        g_missCount++;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: No response for reg 0x%02X "
                  "(miss=%u good=%u bad=%u tx=%u)",
                  g_regTable[g_regIndex].reg,
                  g_missCount, g_goodFrames, g_badFrames, g_txCount);
        UART_ConsumeBytes(UART_GetDataSize());
        g_waiting = 0;

        if (g_missCount >= 3) {
            // Give up on this register, move to next
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: 3 misses on reg 0x%02X — moving to next",
                      g_regTable[g_regIndex].reg);
            g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
            g_missCount = 0;
        }
        // If missCount < 3: stay on same register for retry (no index change)
    } else {
        // Normal path: advance to next register
        g_regIndex  = (g_regIndex + 1) % REG_TABLE_SIZE;
        g_missCount = 0;
    }

    uint8_t reg = g_regTable[g_regIndex].reg;
    HT7017_SendRequest(reg);
    g_waiting     = 1;
    g_readyToSend = 0; // block further sends until RunQuick clears this

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: TX > 6A %02X [%s] (tx=%u)",
              reg, g_regTable[g_regIndex].name, g_txCount);
}

/*
 * HT7017_RunQuick
 *
 * Called frequently (every few ms) by the OpenBeken scheduler.
 * Watches the RX buffer for a complete 4-byte response.
 * Validates checksum, converts raw to real-world value, stores result.
 * Sets g_readyToSend = 1 to unblock RunEverySecond for the next cycle.
 */
void HT7017_RunQuick(void)
{
    if (!g_waiting) return;

    int available = UART_GetDataSize();
    if (available < HT7017_RESPONSE_LEN) return;

    uint8_t d2 = UART_GetByte(0);
    uint8_t d1 = UART_GetByte(1);
    uint8_t d0 = UART_GetByte(2);
    uint8_t cs = UART_GetByte(3);

    // KEY: consume AFTER reading, never before
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);
    g_waiting     = 0;
    g_readyToSend = 1; // chip finished — next request can go

    uint8_t reg      = g_regTable[g_regIndex].reg;
    uint8_t expected = HT7017_CalcChecksum(reg, d2, d1, d0);

    if (cs != expected) {
        g_badFrames++;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CHECKSUM FAIL reg=0x%02X "
                  "got=0x%02X exp=0x%02X | %02X %02X %02X (bad=%u)",
                  reg, cs, expected, d2, d1, d0, g_badFrames);
        return;
    }

    uint32_t raw = ((uint32_t)d2 << 16) |
                   ((uint32_t)d1 <<  8) |
                   (uint32_t)d0;

    float value = (float)raw / g_regTable[g_regIndex].scale;
    *g_regTable[g_regIndex].target = value;

    g_goodFrames++;
    g_missCount = 0;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [%s] = %.3f  raw=%u  CS=OK (good=%u)",
              g_regTable[g_regIndex].name, value, raw, g_goodFrames);
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp),
             "<h5>HT7017 Energy Monitor</h5>"
             "<table border='1' cellpadding='4'>"
             "<tr><td>Voltage</td><td><b>%.2f V</b></td></tr>"
             "<tr><td>Current</td><td><b>%.3f A</b></td></tr>"
             "<tr><td>Power</td><td><b>%.1f W</b></td></tr>"
             "<tr><td>Frequency</td><td><b>%.2f Hz</b></td></tr>"
             "<tr><td>Good Frames</td><td>%u</td></tr>"
             "<tr><td>Bad Frames</td><td>%u</td></tr>"
             "<tr><td>TX Count</td><td>%u</td></tr>"
             "</table>",
             g_voltage, g_current, g_power, g_freq,
             g_goodFrames, g_badFrames, g_txCount);
    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

// ─── Getters ──────────────────────────────────────────────────────────────────
float    HT7017_GetVoltage(void)    { return g_voltage;    }
float    HT7017_GetCurrent(void)    { return g_current;    }
float    HT7017_GetPower(void)      { return g_power;      }
float    HT7017_GetFrequency(void)  { return g_freq;       }
uint32_t HT7017_GetGoodFrames(void) { return g_goodFrames; }
uint32_t HT7017_GetBadFrames(void)  { return g_badFrames;  }
uint32_t HT7017_GetTxCount(void)    { return g_txCount;    }

#endif // ENABLE_DRIVER_HT7017
