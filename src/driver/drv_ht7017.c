/*
 * HT7017 Single-Phase Energy Metering IC Driver
 * Manufacturer: HiTrendtech (钜泉光电科技)
 *
 * Protocol Summary (Datasheet §4.1):
 *   - UART 4800 baud, 9-bit frame (8 data + even parity + stop) = 8E1
 *   - Half-duplex, HT7017 is always the slave
 *   - Read:  Master → [0x6A][REG_ADDR]
 *            Slave  → [DATA2][DATA1][DATA0][CHECKSUM]  (4 bytes)
 *   - Checksum = ~(HEAD + REG + DATA2 + DATA1 + DATA0) & 0xFF
 *   - Raw value = (DATA2<<16) | (DATA1<<8) | DATA0  (24-bit, MSB first)
 *   - Inter-byte gap must be < 20ms or UART module auto-resets
 *
 * Root cause of original driver failure:
 *   UART_ConsumeBytes() was called BEFORE reading, discarding the HT7017
 *   response from the previous cycle. The fix is to read first, then flush.
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
// One register is read per second, cycling through the list.
typedef struct {
    uint8_t     reg;        // HT7017 register address
    float      *target;     // Pointer to output variable
    float       scale;      // Divide raw by this to get real-world value
    const char *name;       // Human-readable name for logging
} RegRead_t;

// Forward declarations of storage so the table can reference them
static float g_voltage  = 0.0f;
static float g_current  = 0.0f;
static float g_power    = 0.0f;
static float g_freq     = 0.0f;

static const RegRead_t g_regTable[] = {
    { HT7017_REG_RMS_U,    &g_voltage, HT7017_VOLTAGE_SCALE, "Voltage(V)"  },
    { HT7017_REG_RMS_I1,   &g_current, HT7017_CURRENT_SCALE, "Current(A)"  },
    { HT7017_REG_POWER_P1, &g_power,   HT7017_POWER_SCALE,   "Power(W)"    },
    { HT7017_REG_FREQ,     &g_freq,    HT7017_FREQ_SCALE,     "Freq(Hz)"    },
};
#define REG_TABLE_SIZE  (sizeof(g_regTable) / sizeof(g_regTable[0]))

// ─── Driver State ─────────────────────────────────────────────────────────────
static uint8_t  g_regIndex   = 0;       // Current position in g_regTable
static uint8_t  g_waiting    = 0;       // 1 = waiting for HT7017 response
static uint32_t g_txCount    = 0;       // Total bytes sent (diagnostic)
static uint32_t g_goodFrames = 0;       // Valid frames received
static uint32_t g_badFrames  = 0;       // Checksum failures

// ─── Internal Helpers ─────────────────────────────────────────────────────────

/*
 * Calculate the expected checksum for a read response.
 * Algorithm (Datasheet §4.1.5):
 *   Sum all bytes in the frame (HEAD + CMD + DATA bytes),
 *   discard carry (8-bit truncation), then bitwise NOT.
 */
static uint8_t HT7017_CalcChecksum(uint8_t reg,
                                    uint8_t d2, uint8_t d1, uint8_t d0)
{
    uint8_t sum = (uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0);
    return (uint8_t)(~sum);
}

/*
 * Send a single read request to the HT7017.
 * Frame: [0x6A][register_address]
 */
static void HT7017_SendRequest(uint8_t reg)
{
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ─── Console Commands ─────────────────────────────────────────────────────────

/*
 * HT7017_Baud <rate>
 * Re-initialise UART at a different baud rate for testing.
 * Example: HT7017_Baud 9600
 */
static commandResult_t CMD_HT7017_Baud(const void *context,
                                        const char *cmd,
                                        const char *args,
                                        int cmdFlags)
{
    if (!args || !*args)
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;

    int baud = atoi(args);
    if (baud <= 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Invalid baud rate '%s'", args);
        return CMD_RES_BAD_ARGUMENT;
    }

    UART_InitUART(baud, HT7017_PARITY_EVEN, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud (8E1)", baud);
    return CMD_RES_OK;
}

/*
 * HT7017_NoParity
 * Switch to 8N1 — some clone chips use no parity.
 */
static commandResult_t CMD_HT7017_NoParity(const void *context,
                                            const char *cmd,
                                            const char *args,
                                            int cmdFlags)
{
    UART_InitUART(HT7017_BAUD_RATE, 0, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: UART re-init at %d baud (8N1 - no parity)", HT7017_BAUD_RATE);
    return CMD_RES_OK;
}

/*
 * HT7017_Status
 * Print current measurements and diagnostic counters to the log.
 */
static commandResult_t CMD_HT7017_Status(const void *context,
                                          const char *cmd,
                                          const char *args,
                                          int cmdFlags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017 Status:");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Voltage : %.2f V",  g_voltage);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Current : %.3f A",  g_current);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Power   : %.1f W",  g_power);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  Freq    : %.2f Hz", g_freq);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  TX count: %u  Good frames: %u  Bad frames: %u",
              g_txCount, g_goodFrames, g_badFrames);
    return CMD_RES_OK;
}

// ─── Public Driver Functions ──────────────────────────────────────────────────

void HT7017_Init(void)
{
    // Large ring buffer — ensures we don't lose bytes between RunQuick polls
    UART_InitReceiveRingBuffer(256);

    // 4800 baud, even parity (8E1) — required by HT7017 datasheet §4.1.2
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    // Register console commands
    CMD_RegisterCommand("HT7017_Baud",     CMD_HT7017_Baud,     NULL);
    CMD_RegisterCommand("HT7017_NoParity", CMD_HT7017_NoParity, NULL);
    CMD_RegisterCommand("HT7017_Status",   CMD_HT7017_Status,   NULL);

    // Log which UART peripheral is in use (controlled by Flag 26)
    if (CFG_HasFlag(26)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Init OK — UART2 (Pin 6/7), 4800 8E1");
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: Init OK — UART1 (P10=RX, P11=TX), 4800 8E1");
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: Voltage scale=%.1f, Current scale=%.1f, Power scale=%.1f",
              HT7017_VOLTAGE_SCALE, HT7017_CURRENT_SCALE, HT7017_POWER_SCALE);

    g_regIndex   = 0;
    g_waiting    = 0;
    g_txCount    = 0;
    g_goodFrames = 0;
    g_badFrames  = 0;
}

/*
 * HT7017_RunEverySecond — called once per second by the OpenBeken scheduler.
 *
 * Sends a read request for the next register in the rotation table.
 * If the previous request never received a response, logs a warning first.
 *
 * KEY FIX vs original driver:
 *   The original flushed the RX buffer BEFORE sending, discarding the
 *   valid 4-byte response from the HT7017. Now we only flush stale/junk
 *   data and read the response inside RunQuick() BEFORE calling ConsumeBytes.
 */
void HT7017_RunEverySecond(void)
{
    if (g_waiting) {
        // Previous request got no response within 1 second — log and reset
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: No response for reg 0x%02X! "
                  "(good=%u bad=%u tx=%u)",
                  g_regTable[g_regIndex].reg,
                  g_goodFrames, g_badFrames, g_txCount);

        // Discard whatever noise may be in the buffer
        UART_ConsumeBytes(UART_GetDataSize());
        g_waiting = 0;

        // Stay on the same register — retry next second
        return;
    }

    // Advance to next register in the rotation
    g_regIndex = (g_regIndex + 1) % REG_TABLE_SIZE;

    // Small flush of any stale bytes before sending a fresh request
    UART_ConsumeBytes(UART_GetDataSize());

    // Send the read request
    uint8_t reg = g_regTable[g_regIndex].reg;
    HT7017_SendRequest(reg);
    g_waiting = 1;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: TX > 6A %02X [%s] (tx=%u)",
              reg, g_regTable[g_regIndex].name, g_txCount);
}

/*
 * HT7017_RunQuick — called frequently (every few ms) by the OpenBeken scheduler.
 *
 * Watches the RX buffer for a complete 4-byte response frame.
 * Validates the checksum, converts raw counts to real-world values,
 * then consumes the bytes AFTER reading (not before).
 */
void HT7017_RunQuick(void)
{
    if (!g_waiting)
        return;

    int available = UART_GetDataSize();
    if (available < HT7017_RESPONSE_LEN)
        return; // Not enough bytes yet — wait longer

    // Read all 4 response bytes
    uint8_t d2 = UART_GetByte(0); // DATA2 (MSB)
    uint8_t d1 = UART_GetByte(1); // DATA1
    uint8_t d0 = UART_GetByte(2); // DATA0 (LSB)
    uint8_t cs = UART_GetByte(3); // CHECKSUM

    // Consume bytes AFTER reading — this was the original bug
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);
    g_waiting = 0;

    // Validate checksum
    uint8_t reg      = g_regTable[g_regIndex].reg;
    uint8_t expected = HT7017_CalcChecksum(reg, d2, d1, d0);

    if (cs != expected) {
        g_badFrames++;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "HT7017: CHECKSUM FAIL reg=0x%02X "
                  "got=0x%02X exp=0x%02X | %02X %02X %02X "
                  "(bad=%u)",
                  reg, cs, expected, d2, d1, d0, g_badFrames);
        return;
    }

    // Assemble 24-bit raw value (big-endian, MSB first per datasheet §4.1.7)
    uint32_t raw = ((uint32_t)d2 << 16) |
                   ((uint32_t)d1 <<  8) |
                   (uint32_t)d0;

    // Convert to real-world value using the calibration scale
    float value = (float)raw / g_regTable[g_regIndex].scale;

    // Store to the target variable pointer
    *g_regTable[g_regIndex].target = value;

    g_goodFrames++;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [%s] = %.3f  raw=%u  CS=OK  (good=%u)",
              g_regTable[g_regIndex].name, value, raw, g_goodFrames);
}

/*
 * HT7017_AppendInformationToHTTPIndexPage
 * Adds a measurement summary widget to the OpenBeken web dashboard.
 */
void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[256];

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

    // Use safe concatenation — request->reply has a fixed size limit
    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

// ─── Getters ──────────────────────────────────────────────────────────────────

float HT7017_GetVoltage(void)   { return g_voltage;    }
float HT7017_GetCurrent(void)   { return g_current;    }
float HT7017_GetPower(void)     { return g_power;      }
float HT7017_GetFrequency(void) { return g_freq;       }

uint32_t HT7017_GetGoodFrames(void) { return g_goodFrames; }
uint32_t HT7017_GetBadFrames(void)  { return g_badFrames;  }
uint32_t HT7017_GetTxCount(void)    { return g_txCount;    }

#endif // ENABLE_DRIVER_HT7017
