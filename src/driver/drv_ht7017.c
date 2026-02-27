/*
 * drv_ht7017.c  –  HT7017 energy metering IC driver  v12
 *
 * Hardware: BK7231N (CBU module), UART1 RX1/TX1 → HT7017 TX/RX
 *           KWS-303WF, 300 µΩ manganin shunt, 40 A max
 *
 * Datasheet ref: HT7017 User Manual Rev1.3 (20000:1 version)
 *   femu = 1 MHz (Emuclk_Sel=1, default), OSR = 64 (default)
 *   Freq formula: Frequency = (femu × 32) / (OSR × Freq_U raw)
 *                           = 500000 / Freq_U_raw
 *   Default reset value of Freq_U = 0x2710 → 500000/10000 = 50 Hz ✓
 *
 * Fixes applied vs v11:
 *   1. FREQ formula corrected:  freq = 500000.0f / raw  (was raw/200 — only accurate at 50Hz)
 *   2. Register addresses fully verified against datasheet register table §5.1
 *      0x0B = PowerQ1  ✓   0x0C = PowerS  ✓   0x0D = EnergyP  ✓
 *      0x0E = EnergyQ  ✓   0x10 = PowerP2 (NOT PFCNT — PFCNT removed)
 *      0x13 = EnergyP_Bak (4-byte, backup cmd only)
 *      0x14 = EnergyQ_Bak (4-byte, backup cmd only)
 *      Energy is now read from 0x0D/0x0E (EnergyP/EnergyQ) only.
 *   3. PFCNT removed — no standalone PFCNT register; PFCnt (0x6F) is a
 *      calibration register not meant for polling.
 *   4. PowerP2 (0x10) / PowerQ2 (0x11) added as optional monitoring.
 *   5. Energy seeded flag: replaced 0xFFFFFFFF sentinel with bool flag.
 *   6. g_sScale: simplified to fabsf(g_pScale) — correct and clear.
 *   7. g_qScale: kept = g_pScale with comment; add ReactiveSet if needed.
 *   8. RunQuick / RunEverySecond desync fixed: separate consumed flag.
 *   9. Write-enable (WPREG 0x32) sequence added before any calibration write.
 *  10. 10 ms power-on delay enforced in HT7017_Init() per datasheet §2.2.
 *  11. ADCCON write triggers energy reliability mechanism (§3.11) — done in Init.
 *  12. Checksum verified: ~(HEAD+CMD+D2+D1+D0) & 0xFF — confirmed correct.
 *  13. Energy 24-bit unsigned wrap handled robustly.
 *  14. Signed 24-bit decode extracted into inline helper.
 *
 * Calibration commands (via UART console):
 *   VoltageSet <V>        – set voltage scale
 *   CurrentSet <A>        – set current scale
 *   PowerSet <W>          – set active power scale (also sets Q and S scale)
 *   ReactiveSet <VAR>     – set reactive power scale independently
 *   EnergyReset           – reset energy accumulators
 *
 * UART frame (read):
 *   TX: HEAD(0x6A) | CMD(0x00|reg) | — 2 bytes total
 *   RX: D2 | D1 | D0 | CS          — 4 bytes
 *   CS (from chip) = ~(HEAD+CMD+D2+D1+D0) & 0xFF
 *
 * UART frame (write):
 *   TX: HEAD(0x6A) | CMD(0x80|reg) | DATA_H | DATA_L | CS — 5 bytes
 *   CS (master) = ~(HEAD+CMD+DATA_H+DATA_L) & 0xFF
 *   RX: ACK (0x54 = OK, 0x63 = error)
 *   Must send write-enable (WPREG = 0xBC or 0xA6) first.
 */

#include "drv_ht7017.h"
#include "tuya_uart.h"   /* or your BSP UART HAL */
#include "rtos_pub.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ─────────────────────────────────────────────────────────
 * Register addresses – verified against datasheet §5.1 table
 * ───────────────────────────────────────────────────────── */
#define HT7017_REG_SPL_I1       0x00  /* ADC waveform Ch1 current (RO,3B) */
#define HT7017_REG_SPL_I2       0x01  /* ADC waveform Ch2 current (RO,3B) */
#define HT7017_REG_SPL_U        0x02  /* ADC waveform voltage     (RO,3B) */
#define HT7017_REG_DC_I         0x03  /* DC mean current          (RO,3B) */
#define HT7017_REG_DC_U         0x04  /* DC mean voltage          (RO,3B) */
/*      0x05 reserved                                                       */
#define HT7017_REG_RMS_I1       0x06  /* RMS current ch1          (RO,3B) */
#define HT7017_REG_RMS_I2       0x07  /* RMS current ch2          (RO,3B) */
#define HT7017_REG_RMS_U        0x08  /* RMS voltage              (RO,3B) */
#define HT7017_REG_FREQ_U       0x09  /* Frequency counter        (RO,2B, reset=0x2710) */
#define HT7017_REG_POWER_P1     0x0A  /* Active power ch1    (RO,3B signed) */
#define HT7017_REG_POWER_Q1     0x0B  /* Reactive power ch1  (RO,3B signed) */
#define HT7017_REG_POWER_S      0x0C  /* Apparent power      (RO,3B signed) */
#define HT7017_REG_ENERGY_P     0x0D  /* Active energy accumulator (RO,3B unsigned) */
#define HT7017_REG_ENERGY_Q     0x0E  /* Reactive energy accum.    (RO,3B unsigned) */
#define HT7017_REG_UDET_CNT     0x0F  /* SAG/Peak duration cnt     (RO,3B) */
#define HT7017_REG_POWER_P2     0x10  /* Active power ch2    (RO,3B signed) */
#define HT7017_REG_POWER_Q2     0x11  /* Reactive power ch2  (RO,3B signed) */
#define HT7017_REG_MAXUWAVE     0x12  /* Voltage half-wave peak    (RO,3B) */
#define HT7017_REG_ENERGY_P_BAK 0x13  /* Active energy backup (RO,4B – use broadcast cmd) */
#define HT7017_REG_ENERGY_Q_BAK 0x14  /* Reactive energy backup (RO,4B) */
#define HT7017_REG_EMUSR        0x19  /* EMU status               (RO,2B) */
#define HT7017_REG_SYSSTA       0x1A  /* System status (WREN bit) (RO,1B) */
#define HT7017_REG_CHIP_ID      0x1B  /* Chip ID = 0x7053F0       (RO,3B) */
#define HT7017_REG_ENERGY_P_NEG 0x1D  /* Reverse active energy    (RO,3B) */
#define HT7017_REG_ENERGY_Q_NEG 0x1E  /* Reverse reactive energy  (RO,3B) */

/* Calibration (write-protected) registers */
#define HT7017_REG_EMUIE        0x30  /* Interrupt enable          (R/W,2B) */
#define HT7017_REG_EMUIF        0x31  /* Interrupt flags           (RO,2B)  */
#define HT7017_REG_WPREG        0x32  /* Write-protect             (R/W,1B) */
#define HT7017_REG_SRSTREG      0x33  /* Software reset (write 0x55)(R/W,1B) */
#define HT7017_REG_AVG_PRMS     0x3F  /* Averaging points          (R/W,2B, default=0x04E2) */
#define HT7017_REG_EMUCFG       0x40  /* EMU config                (R/W,2B) */
#define HT7017_REG_FREQCFG      0x41  /* Clock/freq config         (R/W,2B, default=0x0088) */
#define HT7017_REG_MODULE_EN    0x42  /* Module enable             (R/W,2B, default=0x007E) */
#define HT7017_REG_ANAEN        0x43  /* ADC enable                (R/W,1B, default=0x03) */
#define HT7017_REG_ANACFG       0x44  /* Analog config             (R/W,2B, default=0x9409) */
#define HT7017_REG_IOCFG        0x45  /* IO config                 (R/W,1B) */
#define HT7017_REG_GP1          0x50  /* Active power gain ch1     (R/W,2B) */
#define HT7017_REG_GQ1          0x51  /* Reactive power gain ch1   (R/W,2B) */
#define HT7017_REG_GS1          0x52  /* Apparent power gain ch1   (R/W,2B) */
#define HT7017_REG_ALGCFG       0x57  /* Algorithm config          (R/W,2B) */
#define HT7017_REG_ADCCON       0x59  /* ADC gain config           (R/W,2B) */
#define HT7017_REG_I1OFF        0x5C  /* Ch1 DC offset             (R/W,2B) */
#define HT7017_REG_UOFF         0x5E  /* Voltage DC offset         (R/W,2B) */
#define HT7017_REG_PSTART       0x5F  /* Active power threshold    (R/W,2B, default=0x0040) */
#define HT7017_REG_QSTART       0x60  /* Reactive power threshold  (R/W,2B, default=0x0040) */
#define HT7017_REG_HFCONST      0x61  /* Pulse freq constant       (R/W,2B, default=0x0040) */
#define HT7017_REG_DEC_SHIFT    0x64  /* Phase shift               (R/W,1B) */
#define HT7017_REG_GPHS1        0x6D  /* Phase correction ch1      (R/W,2B) */

/* Write-protect unlock codes */
#define HT7017_WPEN_LOW         0xBC  /* Unlocks 0x35-0x45, 0x4A-0x4F */
#define HT7017_WPEN_HIGH        0xA6  /* Unlocks 0x50-0x7E             */
#define HT7017_WP_LOCK          0x00  /* Re-locks all                  */

/* UART frame constants */
#define HT7017_HEAD             0x6A
#define HT7017_CMD_READ         0x00  /* bit7=0 = read  */
#define HT7017_CMD_WRITE        0x80  /* bit7=1 = write */
#define HT7017_ACK_OK           0x54
#define HT7017_ACK_ERR          0x63

/* Frequency formula (femu=1MHz, OSR=64):
 *   Frequency = (1000000 × 32) / (64 × Freq_U_raw) = 500000 / Freq_U_raw
 * Default raw = 0x2710 (10000) → 500000/10000 = 50.0 Hz
 * At 60 Hz: raw = 500000/60 ≈ 8333 → 500000/8333 = 60.0 Hz ✓
 */
#define HT7017_FREQ_CONST       500000.0f

/* ─────────────────────────────────────────────────────────
 * Default calibration constants (override after empirical cal)
 * ───────────────────────────────────────────────────────── */
#ifndef HT7017_DEFAULT_VOLTAGE_SCALE
#define HT7017_DEFAULT_VOLTAGE_SCALE    200.0f   /* counts per Volt */
#endif
#ifndef HT7017_DEFAULT_CURRENT_SCALE
#define HT7017_DEFAULT_CURRENT_SCALE    1500.0f  /* counts per Amp  */
#endif
#ifndef HT7017_DEFAULT_POWER_SCALE
#define HT7017_DEFAULT_POWER_SCALE      5000.0f  /* counts per Watt */
#endif
#ifndef HT7017_DEFAULT_ENERGY_SCALE
#define HT7017_DEFAULT_ENERGY_SCALE     100.0f   /* counts per Wh   */
#endif
#ifndef HT7017_DEFAULT_CURRENT_OFFSET
#define HT7017_DEFAULT_CURRENT_OFFSET   500      /* raw ADC noise floor */
#endif

/* ─────────────────────────────────────────────────────────
 * State variables – all static, private to this file
 * ───────────────────────────────────────────────────────── */
static float  g_voltage    = 0.0f;   /* Vrms */
static float  g_current    = 0.0f;   /* Arms */
static float  g_power      = 0.0f;   /* W    */
static float  g_reactive   = 0.0f;   /* VAR  */
static float  g_apparent   = 0.0f;   /* VA   */
static float  g_freq       = 50.0f;  /* Hz   */
static float  g_energyWh   = 0.0f;   /* Wh   */
static float  g_energyVARh = 0.0f;   /* VARh */
static float  g_powerP2    = 0.0f;   /* W  ch2 (optional) */

/* Calibration scales */
static float  g_vScale     = HT7017_DEFAULT_VOLTAGE_SCALE;
static float  g_iScale     = HT7017_DEFAULT_CURRENT_SCALE;
static float  g_pScale     = HT7017_DEFAULT_POWER_SCALE;
static float  g_qScale     = HT7017_DEFAULT_POWER_SCALE;  /* same unit as P */
static float  g_sScale     = HT7017_DEFAULT_POWER_SCALE;  /* |P scale|      */
static float  g_eScale     = HT7017_DEFAULT_ENERGY_SCALE; /* counts per Wh  */
static float  g_eqScale    = HT7017_DEFAULT_ENERGY_SCALE; /* counts per VARh*/

/* Current offset – raw counts at zero load */
static int32_t g_iOffset   = HT7017_DEFAULT_CURRENT_OFFSET;

/* Last raw values captured for use by calibration commands */
static uint32_t g_lastRawV  = 0;
static int32_t  g_lastRawI  = 0;
static int32_t  g_lastRawP  = 0;
static int32_t  g_lastRawQ  = 0;
static int32_t  g_lastRawS  = 0;

/* Energy accumulation (24-bit unsigned, wraps at 0xFFFFFF) */
static uint32_t g_epLast    = 0;
static uint32_t g_eqLast    = 0;
static bool     g_epSeeded  = false;  /* safer than sentinel value */
static bool     g_eqSeeded  = false;

/* Register rotation table – registers to poll each slot */
typedef struct {
    uint8_t  reg;
    float   *scale;     /* NULL for special decoding */
    float   *result;    /* NULL for special decoding */
    bool     isSigned;  /* decode as signed 24-bit */
} RegEntry;

/* Register poll rotation (12 slots, ~1/s each at 12s cycle) */
static const RegEntry g_regTable[] = {
    /* slot  reg                    scale          result         signed */
    { HT7017_REG_RMS_U,     NULL, NULL, false }, /* 0: special  */
    { HT7017_REG_RMS_I1,    NULL, NULL, false }, /* 1: special  */
    { HT7017_REG_POWER_P1,  NULL, NULL, true  }, /* 2: special  */
    { HT7017_REG_POWER_Q1,  NULL, NULL, true  }, /* 3: special  */
    { HT7017_REG_POWER_S,   NULL, NULL, true  }, /* 4: special  */
    { HT7017_REG_FREQ_U,    NULL, NULL, false }, /* 5: special  */
    { HT7017_REG_ENERGY_P,  NULL, NULL, false }, /* 6: special  */
    { HT7017_REG_ENERGY_Q,  NULL, NULL, false }, /* 7: special  */
    { HT7017_REG_RMS_U,     NULL, NULL, false }, /* 8: repeat V */
    { HT7017_REG_RMS_I1,    NULL, NULL, false }, /* 9: repeat I */
    { HT7017_REG_POWER_P2,  NULL, NULL, true  }, /*10: ch2 P    */
    { HT7017_REG_FREQ_U,    NULL, NULL, false }, /*11: repeat F */
};
#define REG_TABLE_LEN  (sizeof(g_regTable) / sizeof(g_regTable[0]))

static uint8_t  g_regIndex       = 0;
static bool     g_frameConsumed  = false; /* set by RunQuick, cleared by RunEverySecond */
static bool     g_waitingReply   = false;
static uint8_t  g_rxBuf[5];
static uint8_t  g_rxLen         = 0;
static uint32_t g_lastSendMs    = 0;
static bool     g_initialized   = false;

/* ─────────────────────────────────────────────────────────
 * Internal helpers
 * ───────────────────────────────────────────────────────── */

/* Decode 3-byte big-endian as signed 24-bit two's complement → int32_t */
static inline int32_t decode_signed24(uint8_t d2, uint8_t d1, uint8_t d0)
{
    uint32_t raw = ((uint32_t)d2 << 16) | ((uint32_t)d1 << 8) | d0;
    if (raw & 0x800000u) raw |= 0xFF000000u;  /* sign-extend */
    return (int32_t)raw;
}

/* Decode 3-byte big-endian as unsigned 24-bit */
static inline uint32_t decode_unsigned24(uint8_t d2, uint8_t d1, uint8_t d0)
{
    return ((uint32_t)d2 << 16) | ((uint32_t)d1 << 8) | d0;
}

/* Compute UART checksum: ~(HEAD + CMD + D2 + D1 + D0) & 0xFF */
static inline uint8_t ht7017_cs(uint8_t head, uint8_t cmd,
                                  uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~((uint32_t)head + cmd + d2 + d1 + d0) & 0xFF);
}

/* Send a 2-byte read request: HEAD | (0x00 | reg) */
static void ht7017_send_read(uint8_t reg)
{
    uint8_t tx[2] = { HT7017_HEAD, (uint8_t)(HT7017_CMD_READ | (reg & 0x7F)) };
    UART1_Send(tx, 2);
    g_waitingReply  = true;
    g_frameConsumed = false;
    g_lastSendMs    = rtos_get_time();
}

/*
 * Send a write command to HT7017.
 * Write-enable MUST be set before calling (see ht7017_write_enable()).
 * Frame: HEAD | (0x80|reg) | data_H | data_L | CS
 * HT7017 replies with ACK (0x54 = OK).
 */
static bool ht7017_write_reg(uint8_t reg, uint8_t data_h, uint8_t data_l)
{
    uint8_t cmd = (uint8_t)(HT7017_CMD_WRITE | (reg & 0x7F));
    uint8_t cs  = ht7017_cs(HT7017_HEAD, cmd, 0, data_h, data_l);
    /* Note: checksum covers HEAD+CMD+DATA_H+DATA_L (no 0-pad needed for write) */
    cs = (uint8_t)(~((uint32_t)HT7017_HEAD + cmd + data_h + data_l) & 0xFF);

    uint8_t tx[5] = { HT7017_HEAD, cmd, data_h, data_l, cs };
    UART1_Send(tx, 5);

    /* Wait for ACK byte (blocking, short timeout 5ms) */
    uint8_t ack = 0;
    uint32_t t0 = rtos_get_time();
    while ((rtos_get_time() - t0) < 5) {
        if (UART1_ByteAvailable()) {
            ack = UART1_ReadByte();
            break;
        }
    }
    return (ack == HT7017_ACK_OK);
}

/* Unlock write-protect for low range (0x35-0x45, 0x4A-0x4F) */
static bool ht7017_write_enable_low(void)
{
    /* WPREG is at 0x32 – write 0xBC */
    return ht7017_write_reg(HT7017_REG_WPREG, 0x00, HT7017_WPEN_LOW);
}

/* Unlock write-protect for high range (0x50-0x7E) */
static bool ht7017_write_enable_high(void)
{
    return ht7017_write_reg(HT7017_REG_WPREG, 0x00, HT7017_WPEN_HIGH);
}

/* Re-lock write-protect */
static void ht7017_write_lock(void)
{
    ht7017_write_reg(HT7017_REG_WPREG, 0x00, HT7017_WP_LOCK);
}

/*
 * Process one complete 4-byte RX frame.
 * Called both from RunEverySecond and RunQuick.
 * Returns true if frame was valid.
 */
static bool ht7017_process_frame(uint8_t reg)
{
    if (g_rxLen < 4) return false;

    uint8_t d2 = g_rxBuf[0];
    uint8_t d1 = g_rxBuf[1];
    uint8_t d0 = g_rxBuf[2];
    uint8_t cs = g_rxBuf[3];

    uint8_t cmd = (uint8_t)(HT7017_CMD_READ | (reg & 0x7F));
    uint8_t expected_cs = ht7017_cs(HT7017_HEAD, cmd, d2, d1, d0);

    if (cs != expected_cs) {
        bk_printf("[HT7017] BAD CS reg=%02X got=%02X exp=%02X\r\n",
                  reg, cs, expected_cs);
        return false;
    }

    int32_t  sv = decode_signed24(d2, d1, d0);
    uint32_t uv = decode_unsigned24(d2, d1, d0);

    switch (reg) {
    /* ── Voltage ──────────────────────────────────── */
    case HT7017_REG_RMS_U:
        g_lastRawV = uv;
        g_voltage  = (float)uv / g_vScale;
        break;

    /* ── Current ──────────────────────────────────── */
    case HT7017_REG_RMS_I1: {
        int32_t raw_i = (int32_t)uv;  /* RMS is unsigned per datasheet */
        g_lastRawI = raw_i;
        int32_t net = raw_i - g_iOffset;
        if (net < 0) net = 0;
        g_current = (float)net / g_iScale;
        break;
    }

    /* ── Active power ─────────────────────────────── */
    case HT7017_REG_POWER_P1:
        g_lastRawP = sv;
        g_power    = (float)sv / g_pScale;
        break;

    /* ── Reactive power ───────────────────────────── */
    case HT7017_REG_POWER_Q1:
        g_lastRawQ = sv;
        g_reactive = (float)sv / g_qScale;
        break;

    /* ── Apparent power ───────────────────────────── */
    case HT7017_REG_POWER_S:
        g_lastRawS = sv;
        g_apparent = (float)sv / g_sScale;
        break;

    /* ── Frequency ────────────────────────────────── */
    case HT7017_REG_FREQ_U: {
        /*
         * Freq_U is a 16-bit period counter (MSB always 0).
         * Formula from datasheet §5.1.2.4:
         *   Frequency = (femu × 32) / (OSR × Freq_U)
         *             = (1000000 × 32) / (64 × raw)
         *             = 500000 / raw
         * Default raw = 0x2710 (10000) → 50.0 Hz
         * At 60 Hz:  raw ≈ 8333  → 60.0 Hz ✓
         */
        uint16_t raw_f = (uint16_t)(uv & 0xFFFF);
        if (raw_f > 0) {
            g_freq = HT7017_FREQ_CONST / (float)raw_f;
        }
        break;
    }

    /* ── Active energy accumulator ────────────────── */
    case HT7017_REG_ENERGY_P: {
        /*
         * EnergyP (0x0D): 24-bit unsigned, read-after-no-clear by default.
         * Monotonically increasing, wraps at 0xFFFFFF (16777215).
         */
        if (!g_epSeeded) {
            g_epLast   = uv;
            g_epSeeded = true;
        } else {
            uint32_t delta;
            if (uv >= g_epLast) {
                delta = uv - g_epLast;
            } else {
                /* 24-bit wrap */
                delta = (0x1000000u - g_epLast) + uv;
            }
            g_epLast    = uv;
            g_energyWh += (float)delta / g_eScale;
        }
        break;
    }

    /* ── Reactive energy accumulator ──────────────── */
    case HT7017_REG_ENERGY_Q: {
        if (!g_eqSeeded) {
            g_eqLast   = uv;
            g_eqSeeded = true;
        } else {
            uint32_t delta;
            if (uv >= g_eqLast) {
                delta = uv - g_eqLast;
            } else {
                delta = (0x1000000u - g_eqLast) + uv;
            }
            g_eqLast     = uv;
            g_energyVARh += (float)delta / g_eqScale;
        }
        break;
    }

    /* ── Channel-2 active power (optional monitor) ── */
    case HT7017_REG_POWER_P2:
        g_powerP2 = (float)sv / g_pScale;
        break;

    default:
        break;
    }

    g_rxLen = 0;
    g_waitingReply  = false;
    g_frameConsumed = true;
    return true;
}

/* ─────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────── */

void HT7017_Init(void)
{
    /* Per §2.2: wait ≥10ms after power-on before operating registers */
    rtos_delay_milliseconds(15);

    /* Send one write to ADCCON to trigger energy reliability mechanism (§3.11).
     * Write the default value 0x0000 – this satisfies the requirement
     * that MCU must write ADCCON at least once after reset before energy counts. */
    ht7017_write_enable_low();
    ht7017_write_reg(HT7017_REG_ADCCON, 0x00, 0x00);
    ht7017_write_lock();

    g_regIndex       = 0;
    g_frameConsumed  = false;
    g_waitingReply   = false;
    g_rxLen          = 0;
    g_epSeeded       = false;
    g_eqSeeded       = false;
    g_initialized    = true;

    bk_printf("[HT7017] Init OK, femu=1MHz OSR=64, freq formula=500000/raw\r\n");
}

/*
 * HT7017_RunEverySecond() – call once per second from your task scheduler.
 * Sends the next read request to the HT7017 in the rotation.
 * Processes any outstanding reply first if available.
 */
void HT7017_RunEverySecond(void)
{
    if (!g_initialized) return;

    /* Drain any pending RX bytes into buffer */
    while (UART1_ByteAvailable() && g_rxLen < sizeof(g_rxBuf)) {
        g_rxBuf[g_rxLen++] = UART1_ReadByte();
    }

    if (g_waitingReply && g_rxLen >= 4) {
        uint8_t cur_reg = g_regTable[g_regIndex].reg;
        ht7017_process_frame(cur_reg);
        /* Advance index only if this slot was not already consumed by RunQuick */
        if (!g_frameConsumed) {
            /* frame processed normally */
        }
        g_regIndex = (g_regIndex + 1) % REG_TABLE_LEN;
    } else if (g_waitingReply && !g_frameConsumed) {
        /* Timeout – no reply; advance anyway to avoid stall */
        uint32_t now = rtos_get_time();
        if ((now - g_lastSendMs) > 200) {
            bk_printf("[HT7017] timeout reg=%02X\r\n", g_regTable[g_regIndex].reg);
            g_waitingReply  = false;
            g_rxLen         = 0;
            g_regIndex      = (g_regIndex + 1) % REG_TABLE_LEN;
        }
        return;  /* don't send yet */
    } else if (g_frameConsumed) {
        /* RunQuick already consumed this slot's frame – advance index */
        g_frameConsumed = false;
        g_regIndex = (g_regIndex + 1) % REG_TABLE_LEN;
    }

    /* Send request for next slot */
    uint8_t next_reg = g_regTable[g_regIndex].reg;
    ht7017_send_read(next_reg);
}

/*
 * HT7017_RunQuick() – call from UART RX interrupt or fast loop.
 * Processes any complete 4-byte frame that has arrived.
 * Does NOT advance g_regIndex (RunEverySecond owns that).
 */
void HT7017_RunQuick(void)
{
    if (!g_initialized || !g_waitingReply) return;

    while (UART1_ByteAvailable() && g_rxLen < sizeof(g_rxBuf)) {
        g_rxBuf[g_rxLen++] = UART1_ReadByte();
    }

    if (g_rxLen >= 4) {
        uint8_t cur_reg = g_regTable[g_regIndex].reg;
        if (ht7017_process_frame(cur_reg)) {
            /* g_frameConsumed is now true; RunEverySecond will see it */
        }
    }
}

/* ─────────────────────────────────────────────────────────
 * Getters
 * ───────────────────────────────────────────────────────── */
float HT7017_GetVoltage(void)   { return g_voltage;    }
float HT7017_GetCurrent(void)   { return g_current;    }
float HT7017_GetPower(void)     { return g_power;      }
float HT7017_GetReactive(void)  { return g_reactive;   }
float HT7017_GetApparent(void)  { return g_apparent;   }
float HT7017_GetFreq(void)      { return g_freq;       }
float HT7017_GetEnergyWh(void)  { return g_energyWh;   }
float HT7017_GetEnergyVARh(void){ return g_energyVARh; }

float HT7017_GetPowerFactor(void)
{
    if (g_apparent < 0.001f) return 1.0f;
    float pf = g_power / g_apparent;
    if (pf >  1.0f) pf =  1.0f;
    if (pf < -1.0f) pf = -1.0f;
    return pf;
}

/* ─────────────────────────────────────────────────────────
 * Calibration commands
 *
 * These are called from your console/command handler.
 * Each command uses the last captured raw value from normal polling.
 *
 * IMPORTANT: calibration writes require write-enable first.
 * The scale is purely a software factor; we don't write GP1/GQ1/GS1
 * (gain registers) here – those are for hardware fine-tuning.
 * ───────────────────────────────────────────────────────── */

/* VoltageSet <actual_vrms> */
void HT7017_VoltageSet(float actual_v)
{
    if (actual_v <= 0.0f || g_lastRawV == 0) {
        bk_printf("[HT7017] VoltageSet: no raw data yet\r\n");
        return;
    }
    g_vScale = (float)g_lastRawV / actual_v;
    g_voltage = actual_v;
    bk_printf("[HT7017] VoltageSet: rawV=%lu actual=%.2fV => g_vScale=%.2f\r\n",
              (unsigned long)g_lastRawV, actual_v, g_vScale);
}

/* CurrentSet <actual_arms> */
void HT7017_CurrentSet(float actual_a)
{
    if (actual_a <= 0.0f || g_lastRawI == 0) {
        bk_printf("[HT7017] CurrentSet: no raw data yet\r\n");
        return;
    }
    /* Offset is whatever we read at zero-current – keep existing g_iOffset.
     * Scale = (raw_at_load - offset) / actual_load */
    int32_t net = g_lastRawI - g_iOffset;
    if (net <= 0) {
        bk_printf("[HT7017] CurrentSet: raw <= offset, increase load\r\n");
        return;
    }
    g_iScale  = (float)net / actual_a;
    g_current = actual_a;
    bk_printf("[HT7017] CurrentSet: rawI=%ld offset=%ld net=%ld actual=%.3fA => g_iScale=%.2f\r\n",
              (long)g_lastRawI, (long)g_iOffset, (long)net, actual_a, g_iScale);
}

/* CurrentOffsetSet – call with zero current load to capture noise floor */
void HT7017_CurrentOffsetSet(void)
{
    g_iOffset = g_lastRawI;
    g_current = 0.0f;
    bk_printf("[HT7017] CurrentOffset set to %ld\r\n", (long)g_iOffset);
}

/* PowerSet <actual_watts> – also sets Q and S scale proportionally */
void HT7017_PowerSet(float actual_w)
{
    if (actual_w <= 0.0f || g_lastRawP == 0) {
        bk_printf("[HT7017] PowerSet: no raw data yet (use resistive load)\r\n");
        return;
    }
    g_pScale  = (float)g_lastRawP / actual_w;
    g_qScale  = g_pScale;             /* Q uses same counts-per-watt unit */
    g_sScale  = fabsf(g_pScale);      /* S is always unsigned; strip any sign */
    g_power   = actual_w;
    bk_printf("[HT7017] PowerSet: rawP=%ld actual=%.2fW => g_pScale=%.2f\r\n",
              (long)g_lastRawP, actual_w, g_pScale);
}

/* ReactiveSet <actual_var> – independent Q calibration (needs reactive load) */
void HT7017_ReactiveSet(float actual_var)
{
    if (actual_var == 0.0f || g_lastRawQ == 0) {
        bk_printf("[HT7017] ReactiveSet: no raw data or zero\r\n");
        return;
    }
    g_qScale   = (float)g_lastRawQ / actual_var;
    g_reactive = actual_var;
    bk_printf("[HT7017] ReactiveSet: rawQ=%ld actual=%.2fVAR => g_qScale=%.2f\r\n",
              (long)g_lastRawQ, actual_var, g_qScale);
}

/* EnergyReset – reset software accumulators (chip keeps counting) */
void HT7017_EnergyReset(void)
{
    g_energyWh   = 0.0f;
    g_energyVARh = 0.0f;
    g_epSeeded   = false;
    g_eqSeeded   = false;
    bk_printf("[HT7017] Energy accumulators reset\r\n");
}

/* EnergyScaleSet <wh_per_count> – set after running a known load for 1h */
void HT7017_EnergyScaleSet(float wh_per_count)
{
    if (wh_per_count <= 0.0f) return;
    g_eScale  = wh_per_count;
    g_eqScale = wh_per_count;
    bk_printf("[HT7017] EnergyScale set to %.4f Wh/count\r\n", wh_per_count);
}

/* ─────────────────────────────────────────────────────────
 * Debug / status dump
 * ───────────────────────────────────────────────────────── */
void HT7017_PrintStatus(void)
{
    bk_printf("[HT7017] V=%.2fV I=%.3fA P=%.2fW Q=%.2fVAR S=%.2fVA PF=%.3f F=%.2fHz\r\n",
              g_voltage, g_current, g_power, g_reactive, g_apparent,
              HT7017_GetPowerFactor(), g_freq);
    bk_printf("[HT7017] Energy: %.4fWh  %.4fVARh\r\n", g_energyWh, g_energyVARh);
    bk_printf("[HT7017] Scales: vScale=%.2f iScale=%.2f pScale=%.2f eScale=%.4f\r\n",
              g_vScale, g_iScale, g_pScale, g_eScale);
    bk_printf("[HT7017] Raw: V=%lu I=%ld P=%ld Q=%ld S=%ld offset=%ld\r\n",
              (unsigned long)g_lastRawV, (long)g_lastRawI,
              (long)g_lastRawP, (long)g_lastRawQ, (long)g_lastRawS,
              (long)g_iOffset);
}

/* ─────────────────────────────────────────────────────────
 * Software reset (writes 0x55 to SRSTREG 0x33)
 * HT7017 resets; wait 10ms before re-initializing.
 * ───────────────────────────────────────────────────────── */
void HT7017_SoftReset(void)
{
    /* SRSTREG is in low range (0x33) – needs WPEN_LOW unlock */
    ht7017_write_enable_low();
    ht7017_write_reg(HT7017_REG_SRSTREG, 0x00, 0x55);
    /* No ACK expected (chip resets); wait for reboot */
    rtos_delay_milliseconds(20);
    bk_printf("[HT7017] Software reset issued\r\n");
    HT7017_Init();
}

/* ─────────────────────────────────────────────────────────
 * Chip ID verification (useful for bring-up)
 * Expected: 0x7053F0  (ChipID register 0x1B)
 * ───────────────────────────────────────────────────────── */
uint32_t HT7017_ReadChipID(void)
{
    ht7017_send_read(HT7017_REG_CHIP_ID);
    rtos_delay_milliseconds(10);
    while (UART1_ByteAvailable() && g_rxLen < sizeof(g_rxBuf)) {
        g_rxBuf[g_rxLen++] = UART1_ReadByte();
    }
    if (g_rxLen >= 3) {
        uint32_t id = decode_unsigned24(g_rxBuf[0], g_rxBuf[1], g_rxBuf[2]);
        bk_printf("[HT7017] ChipID=0x%06lX (expect 0x7053F0)\r\n", (unsigned long)id);
        g_rxLen        = 0;
        g_waitingReply = false;
        return id;
    }
    return 0;
}

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


