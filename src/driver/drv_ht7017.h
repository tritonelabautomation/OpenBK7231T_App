/*
 * drv_ht7017.h  –  HT7017 energy metering IC driver  v12
 *
 * All register addresses, formulas, and flag behaviour verified against:
 *   HT7017 User Manual Rev1.3 (20000:1 variant)
 *   datasheet: https://datasheet.lcsc.com/lcsc/...HT7017-TR_C81511.pdf
 *
 * Hardware: BK7231N (CBU module) UART1 RX1/TX1 ↔ HT7017
 *           KWS-303WF, 300 µΩ manganin shunt, 40 A max
 *
 * ── Calibration constants: override via -D or edit defaults in .c ──────────
 *
 *   HT7017_DEFAULT_VOLTAGE_SCALE   counts / V    (empirical, typically 150-250)
 *   HT7017_DEFAULT_CURRENT_SCALE   counts / A    (empirical, typically 800-3000)
 *   HT7017_DEFAULT_POWER_SCALE     counts / W    (empirical, typically 1000-10000)
 *   HT7017_DEFAULT_ENERGY_SCALE    counts / Wh   (empirical, must measure over 1h)
 *   HT7017_DEFAULT_CURRENT_OFFSET  raw counts @ zero current (typ 100-2000)
 *
 * ── Frequency formula (verified from datasheet §5.1.2.4) ──────────────────
 *
 *   femu = 1 MHz (Emuclk_Sel=1, chip default)
 *   OSR  = 64   (chip default)
 *   Frequency = (femu × 32) / (OSR × Freq_U_raw)
 *             = (1,000,000 × 32) / (64 × raw)
 *             = 500,000 / raw
 *
 *   raw = 0x2710 (10000) → 500000/10000 = 50.0 Hz  ✓ (chip default at 50Hz)
 *   raw = 8333            → 500000/8333  = 60.0 Hz  ✓ (60Hz grid)
 *
 * ── Register map highlights (§5.1) ────────────────────────────────────────
 *   0x06 Rms_I1  – RMS current ch1 (unsigned 24-bit)
 *   0x07 Rms_I2  – RMS current ch2 (unsigned 24-bit)
 *   0x08 Rms_U   – RMS voltage     (unsigned 24-bit)
 *   0x09 Freq_U  – Frequency counter (16-bit, reset=0x2710)
 *   0x0A PowerP1 – Active power ch1  (signed 24-bit)
 *   0x0B PowerQ1 – Reactive power ch1 (signed 24-bit)
 *   0x0C PowerS  – Apparent power    (signed 24-bit, always positive in practice)
 *   0x0D EnergyP – Active energy accumulator (unsigned 24-bit, no auto-clear)
 *   0x0E EnergyQ – Reactive energy accumulator (unsigned 24-bit)
 *   0x10 PowerP2 – Active power ch2  (NOT a PFCNT register – PFCNT does not
 *                  exist as a standalone read register in this chip)
 *   0x11 PowerQ2 – Reactive power ch2 (signed 24-bit)
 *   0x13 EnergyP_Bak – Backup register (4 bytes, requires broadcast cmd)
 *   0x14 EnergyQ_Bak – Backup register (4 bytes, requires broadcast cmd)
 *   0x1B ChipID  – Expected 0x7053F0
 *   0x1D EnergyP_Neg – Reverse active energy (unsigned 24-bit)
 *   0x1E EnergyQ_Neg – Reverse reactive energy (unsigned 24-bit)
 *   0x32 WPREG   – Write-protect (0xBC unlocks low, 0xA6 unlocks high, 0x00 locks)
 *   0x33 SRSTREG – Software reset: write 0x55
 *   0x59 ADCCON  – ADC gain (must write once after reset for energy reliability §3.11)
 */

#ifndef DRV_HT7017_H
#define DRV_HT7017_H

#include <stdint.h>
#include <stdbool.h>

/* ── Calibration defaults (override via compiler -D flags) ────────────── */
#ifndef HT7017_DEFAULT_VOLTAGE_SCALE
#define HT7017_DEFAULT_VOLTAGE_SCALE    200.0f
#endif
#ifndef HT7017_DEFAULT_CURRENT_SCALE
#define HT7017_DEFAULT_CURRENT_SCALE    1500.0f
#endif
#ifndef HT7017_DEFAULT_POWER_SCALE
#define HT7017_DEFAULT_POWER_SCALE      5000.0f
#endif
#ifndef HT7017_DEFAULT_ENERGY_SCALE
#define HT7017_DEFAULT_ENERGY_SCALE     100.0f
#endif
#ifndef HT7017_DEFAULT_CURRENT_OFFSET
#define HT7017_DEFAULT_CURRENT_OFFSET   500
#endif

/* ── Lifecycle ──────────────────────────────────────────────────────────── */
void     HT7017_Init(void);          /* Call once at startup (waits 15ms)   */
void     HT7017_RunQuick(void);      /* Call from UART ISR or fast loop      */
void     HT7017_RunEverySecond(void);/* Call from 1-Hz task scheduler        */
void     HT7017_AppendInformationToHTTPIndexPage(http_request_t *request);
//void     HT7017_SoftReset(void);     /* Issues software reset, re-inits      */
uint32_t HT7017_ReadChipID(void);    /* Blocking; returns 0x7053F0 if OK    */

/* ── Measurement getters ────────────────────────────────────────────────── */
float    HT7017_GetVoltage(void);    /* V RMS                               */
float    HT7017_GetCurrent(void);    /* A RMS                               */
float    HT7017_GetPower(void);      /* W  (active, signed)                 */
float    HT7017_GetReactive(void);   /* VAR (reactive, signed)              */
float    HT7017_GetApparent(void);   /* VA  (apparent, always ≥0)           */
float    HT7017_GetPowerFactor(void);/* -1..+1                              */
float    HT7017_GetFreq(void);       /* Hz (50 or 60)                       */
float    HT7017_GetEnergyWh(void);   /* Wh  (accumulated, wraps-safe)       */
float    HT7017_GetEnergyVARh(void); /* VARh (accumulated)                  */

/* ── Calibration commands ───────────────────────────────────────────────── */
/* Call each after polling has captured at least one raw reading.           */
/* Use purely resistive load (incandescent / heater) for best accuracy.     */

void     HT7017_VoltageSet(float actual_vrms);   /* Set V scale             */
void     HT7017_CurrentSet(float actual_arms);   /* Set I scale             */
void     HT7017_CurrentOffsetSet(void);           /* Set I offset @ zero A   */
void     HT7017_PowerSet(float actual_watts);    /* Set P scale (and Q, S)  */
void     HT7017_ReactiveSet(float actual_var);   /* Set Q scale independently*/
void     HT7017_EnergyReset(void);               /* Zero Wh/VARh counters   */
void     HT7017_EnergyScaleSet(float wh_per_count); /* Set after 1h test    */

/* ── Debug ──────────────────────────────────────────────────────────────── */
void     HT7017_PrintStatus(void);   /* Dump all values + scales to UART    */

#endif /* DRV_HT7017_H */
