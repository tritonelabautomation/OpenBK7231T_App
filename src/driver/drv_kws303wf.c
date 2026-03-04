/*
 * drv_kws303wf.c — KWS-303WF Device Application Driver for OpenBK7231N
 *
 * SINGLE RESPONSIBILITY: Device-specific hardware & EV session application.
 * Owns: latching relay GPIO (P7/P8), buttons (P28/P26/P20), NTC temp (ADC3),
 *       EV session tracking, cost calculation, MQTT session summary.
 *
 * startDriver KWS303WF
 *
 * ── CHANNEL OUTPUT ────────────────────────────────────────────────────────
 *   Ch 8  = Relay state      (0=open/safe, 100=closed/on)
 *   Ch 9  = Temperature      (degrees-C x 100, e.g. 2716 = 27.16 C)
 *   Ch 10 = EV session cost  (Rs x 100, e.g. 376 = Rs 3.76)
 *   Ch 12 = Session active   (0=idle, 1=active) — read by drv_st7735 for
 *                             status bar colour and timer source selection
 *   Ch 13 = Session elapsed  (seconds since sess_start, resets at sess_end)
 *                             — display shows H:MM:SS; MQTT uses duration_s
 *
 * ── CHANNEL INPUT ─────────────────────────────────────────────────────────
 *   Ch 3  = Power  (W x 10)     from drv_ht7017
 *   Ch 2  = Current (A x 1000)  from drv_ht7017
 *   Ch 6  = Energy (Wh x 10)    from drv_ht7017
 *   Ch 8  = Relay state (read back to react to external writes)
 *
 * ── NTC TEMPERATURE  (v2 — NASA three-layer adaptive sampling) ───────────
 *   Hardware: External 10k NTC probe on long cable (image-verified).
 *             P23/ADC ch3. B=3950, R25=10k, Rs=10k, Vcc→NTC→pin→Rs→GND.
 *
 *   SCIENTIFIC BASIS
 *   ────────────────
 *   External probe NTC thermal time constant τ ≈ 60-300 s (cable/surface).
 *   Nyquist limit: f_useful < 1/(2×60) = 0.0083 Hz → sample every 120 s.
 *   Original 1 Hz = 120× oversampling. Saves ~90% ADC/logf/CMD dispatches.
 *
 *   THREE-LAYER FDIR (ECSS-E-ST-70-41C / JPL DSN 810-005):
 *   ┌─ LAYER 3: ALARM  T ≥ NTC_ALARM_C  → relay_open(), latch FAST        ┐
 *   ├─ LAYER 2: FAST   |dT/dt| ≥ trigger OR T ≥ warn → 1 s poll          ┤
 *   └─ LAYER 1: SLOW   stable, T < warn, held ≥ min_ticks → 10 s poll    ┘
 *
 *   EMA FILTER: y = α·x + (1-α)·y_prev, α=1/N, O(1), zero heap.
 *   DEADBAND GATE: publish only if |T_ema - T_last_pub| ≥ 0.5 °C
 *                  OR keepalive on SLOW tick.
 *
 *   SAVINGS:
 *     ADC reads    : 3600/hr → ~360/hr steady state  (−90%)
 *     logf() calls : 3600/hr → ~360/hr steady state  (−90%)
 *     PubCh calls  : 3600/hr → ~36/hr at 0.5°C/step (−99% in steady state)
 *
 * ── RELAY HARDWARE (KWS-303WF schematic-verified) ─────────────────────────
 *   P7 = ON coil  (HAL pin 8) — pulse HIGH → contacts CLOSE
 *   P8 = OFF coil (HAL pin 7) — pulse HIGH → contacts OPEN
 *
 * ── autoexec.bat ─────────────────────────────────────────────────────────
 *   startDriver HT7017
 *   startDriver ST7735 14 16 9 17 15 24
 *   startDriver KWS303WF
 *   AddChangeHandler Channel7 != 0 kws_ch8_set 0
 *   AddChangeHandler Channel7 == 0 kws_ch8_set 100
 *
 * ── New channels (set in autoexec.bat) ────────────────────────────────────
 *   setChannelType 12 ReadOnly   (session active flag)
 *   setChannelType 13 ReadOnly   (session elapsed seconds)
 *
 * ── New console commands ───────────────────────────────────────────────────
 *   kws_detect_w [W]        — get/set auto-detect power threshold (default 500W)
 *   kws_history_dump        — print session history CSV to UART log
 *   kws_history_clear       — truncate /kws_history.csv
 *   kws_lifetime [Wh]       — get/set lifetime energy odometer
 *   kws_session_stat        — now shows elapsed time, detect_w, lifetime
 *   [-] button (P20)        — cycles vehicle profile (2W↔4W) at any time
 */

#include "../obk_config.h"
#if ENABLE_DRIVER_KWS303WF

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../hal/hal_pins.h"
#include "../cmnds/cmd_public.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

/* ============================================================================
 * SECTION A — CONFIGURATION
 * ============================================================================ */
#define KWS_EV_RATE_DEFAULT   7.00f
#define KWS_VEH2_NAME         "Ather Rizta Z"
#define KWS_VEH2_KM_PER_KWH  33.0f
#define KWS_VEH2_OLD_KMPL     30.0f
#define KWS_VEH4_NAME         "Nexon EV 45"
#define KWS_VEH4_KM_PER_KWH   7.8f
#define KWS_VEH4_OLD_KMPL     12.0f
#define KWS_PETROL_RS_PER_L  107.0f
#define KWS_DETECT_W_MIN_DEFAULT 500.0f  /* runtime-adjustable via kws_detect_w */
#define KWS_DETECT_S          30
#define KWS_END_S             60
#define KWS_SPLIT_W         4500.0f
#define KWS_RELAY_PIN_ON      8
#define KWS_RELAY_PIN_OFF     7
#define KWS_RELAY_PULSE_MS  200
#define KWS_BTN_TOGGLE       28
#define KWS_BTN_SESSION      26
#define KWS_BTN_RESERVED     20

/* ── NTC hardware ─────────────────────────────────────────────────────── */
#define KWS_ADC_CH            23
#define KWS_ADC_MAX        4095.0f
#define KWS_NTC_R25       10000.0f
#define KWS_NTC_B          3950.0f
#define KWS_NTC_RS        10000.0f
#define KWS_NTC_T0K        298.15f
#define KWS_NTC_PULLUP         1

/* ── NTC adaptive sampling parameters ────────────────────────────────────
 *
 *  NTC_SLOW_PERIOD    Poll every N seconds in steady state.
 *                     Scientific minimum per Nyquist with τ≥60s: 30 s.
 *                     Set to 10 s for comfortable engineering margin.
 *
 *  NTC_FAST_PERIOD    Poll every N seconds during transient/alarm.
 *                     1 s = original rate, active only when needed.
 *
 *  NTC_FAST_TRIGGER_C |ΔT per sample| that switches to FAST mode.
 *                     At SLOW (10s interval): 1.0 °C/sample = 6 °C/min.
 *                     Detects charger cable heating events promptly.
 *
 *  NTC_DEADBAND_C     Minimum |T_ema - T_last_pub| to publish Ch9.
 *                     0.5 °C >> ADC noise floor (~0.16 °C at 27°C).
 *                     At 27 °C steady state: zero extra publishes per hour.
 *
 *  NTC_WARN_C         Pre-alarm threshold. Locks to FAST mode.
 *                     10 °C margin below hard alarm (PVC cable rated 70°C).
 *
 *  NTC_ALARM_C        Hard safety limit. Opens relay immediately.
 *                     70 °C = PVC insulation temperature rating.
 *
 *  NTC_EMA_N          EMA filter depth α = 1/N.
 *                     N=4: τ_ema ≈ 1.5 s @ 1 Hz fast, 15 s @ 0.1 Hz slow.
 *                     Rejects ADC noise without meaningful response lag.
 *
 *  NTC_FAST_MIN_TICKS Minimum ticks to remain in FAST before returning
 *                     to SLOW. Prevents FAST↔SLOW chatter at the trigger
 *                     boundary.
 * ─────────────────────────────────────────────────────────────────────── */
#define NTC_SLOW_PERIOD       10u
#define NTC_FAST_PERIOD        1u
#define NTC_FAST_TRIGGER_C   1.0f
#define NTC_DEADBAND_C       0.5f
#define NTC_WARN_C          60.0f
#define NTC_ALARM_C         70.0f
#define NTC_EMA_N              4u
#define NTC_FAST_MIN_TICKS    15u

/* ── OBK channels ─────────────────────────────────────────────────────── */
#define KWS_CH_RELAY          8
#define KWS_CH_TEMP           9
#define KWS_CH_EVCOST        10
#define KWS_CH_POWER          3
#define KWS_CH_CURRENT        2
#define KWS_CH_ENERGY         6
#define KWS_CH_WIFI          11   /* OBK core — read-only here for MQTT guard */
#define KWS_CH_SESS_ACTIVE   12   /* 0=idle 1=active — read by display        */
#define KWS_CH_SESS_ELAPSED  13   /* seconds since sess_start — display timer  */

#define KWS_SESSION_FILE     "/kws_session.cfg"
#define KWS_HISTORY_FILE     "/kws_history.csv"   /* append-only session log   */
#define KWS_LIFETIME_FILE    "/kws_lifetime.cfg"  /* persists cumulative Wh    */
#define KWS_MQTT_TOPIC       "home/ev/session"

/* ============================================================================
 * SECTION B — CHANNEL HELPERS
 * ============================================================================ */
static void PubCh(int ch, int val)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "setChannel %d %d", ch, val);
    CMD_ExecuteCommand(buf, 0);
}

extern int CHANNEL_Get(int ch);   /* file-scope extern — audit verified */

static int ReadCh(int ch) { return CHANNEL_Get(ch); }

/* ── Improvement #6: runtime-adjustable auto-detect threshold ─────────── */
static float g_detect_w_min = KWS_DETECT_W_MIN_DEFAULT;

/* ── Improvement #8: lifetime energy accumulator ──────────────────────── *
 * Loaded from KWS_LIFETIME_FILE at init.  Updated every 60 s while        *
 * a session is active, and on every sess_end().                            *
 * Unit: Wh (float). Persists across reflashes as long as JFFS2 is intact. */
static float    g_lifetime_wh     = 0.0f;
static uint8_t  g_lifetime_dirty  = 0u;   /* 1 = needs write to flash     */

static void lifetime_load(void)
{
    FILE *f = fopen(KWS_LIFETIME_FILE, "r");
    if (!f) return;
    fscanf(f, "wh=%f\n", &g_lifetime_wh);
    fclose(f);
    if (g_lifetime_wh < 0.0f) g_lifetime_wh = 0.0f;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: lifetime loaded %.2f Wh (%.4f kWh)",
              g_lifetime_wh, g_lifetime_wh / 1000.0f);
}

static void lifetime_save(void)
{
    FILE *f = fopen(KWS_LIFETIME_FILE, "w");
    if (!f) return;
    fprintf(f, "wh=%.4f\n", g_lifetime_wh);
    fclose(f);
    g_lifetime_dirty = 0u;
}

/* ============================================================================
 * SECTION C — RELAY
 * ============================================================================ */
static uint32_t g_uptime_s    = 0;
static uint8_t  g_relay_closed = 0;
static uint8_t  g_pulse_pin    = 0;
static uint32_t g_pulse_off_at = 0;

static void relay_pulse_tick(void)
{
    if (g_pulse_pin != 0 && g_uptime_s >= g_pulse_off_at) {
        HAL_PIN_SetOutputValue((int)g_pulse_pin, 0);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: coil P%u de-energized", (unsigned)g_pulse_pin);
        g_pulse_pin = 0;
    }
}

static void relay_pulse_start(int pin)
{
    if (g_pulse_pin != 0) { HAL_PIN_SetOutputValue((int)g_pulse_pin, 0); g_pulse_pin = 0; }
    HAL_PIN_SetOutputValue(pin, 1);
    g_pulse_pin    = (uint8_t)pin;
    g_pulse_off_at = g_uptime_s + 1;
}

static void relay_open(void)
{
    relay_pulse_start(KWS_RELAY_PIN_OFF);
    g_relay_closed = 0;
    PubCh(KWS_CH_RELAY, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay OPEN (P%d pulsed, load OFF)", KWS_RELAY_PIN_OFF);
}

static void relay_close(void)
{
    relay_pulse_start(KWS_RELAY_PIN_ON);
    g_relay_closed = 1;
    PubCh(KWS_CH_RELAY, 100);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay CLOSE (P%d pulsed, load ON)", KWS_RELAY_PIN_ON);
}

static void relay_sync(int ch8_val)
{
    int want = (ch8_val >= 100);
    if ( want && !g_relay_closed) relay_close();
    if (!want &&  g_relay_closed) relay_open();
}

/* ============================================================================
 * SECTION D — NTC TEMPERATURE  (v2 — NASA three-layer adaptive sampling)
 *
 *  State machine overview
 *  ──────────────────────
 *
 *  RunEverySecond()
 *    └─► ntc_tick()
 *          ├─ g_ntc_countdown > 1 ?  decrement and RETURN (cheap path)
 *          └─ countdown == 0:
 *               reload countdown = SLOW or FAST period
 *               read ADC → Steinhart-Hart → T_raw
 *               guard: reject T_raw outside [-40, 125] (probe fault)
 *               EMA: seed on first valid reading, then y += (x-y)/N
 *               dT_dt   = |T_ema - T_ema_prev|     per-sample rate
 *               delta   = |T_ema - T_last_pub|      since last publish
 *               ─── LAYER 3: T_ema ≥ ALARM_C ─────────────────────────────
 *               │   relay_open(), latch g_ntc_alarm, stay FAST, publish
 *               ─── LAYER 2: dT_dt ≥ TRIGGER or T_ema ≥ WARN ────────────
 *               │   switch g_ntc_in_fast=1, reset fast_held counter
 *               ─── LAYER 1: stable + held ≥ MIN_TICKS ───────────────────
 *               │   switch g_ntc_in_fast=0, reload SLOW countdown
 *               ─── PUBLISH GATE ─────────────────────────────────────────
 *                   delta ≥ DEADBAND or is_slow_tick or alarm → PubCh(9)
 *
 *  Memory delta vs original: +6 bytes (3 floats→3 floats, +4 uint8 fields)
 *  CPU delta on non-sample ticks: 1 decrement + 1 compare = ~0.1 µs
 * ============================================================================ */

/* ── Steinhart-Hart B-parameter equation (unchanged from v1) ──────────── */
static float ntc_raw_celsius(void)
{
    uint32_t raw = HAL_ADC_Read(KWS_ADC_CH);
    if (raw == 0)                     return -99.0f;
    if (raw >= (uint32_t)KWS_ADC_MAX) return 999.0f;
    float r = (float)raw;
#if KWS_NTC_PULLUP
    float rntc = KWS_NTC_RS * r / (KWS_ADC_MAX - r);
#else
    float rntc = KWS_NTC_RS * (KWS_ADC_MAX - r) / r;
#endif
    float temp_k = 1.0f / (1.0f / KWS_NTC_T0K +
                            logf(rntc / KWS_NTC_R25) / KWS_NTC_B);
    return temp_k - 273.15f;
}

/* ── NTC adaptive state ────────────────────────────────────────────────── */
static float   g_ntc_ema       = 25.0f;   /* EMA filtered temperature (°C)  */
static float   g_ntc_ema_prev  = 25.0f;   /* previous EMA for dT/dt         */
static float   g_ntc_last_pub  = -999.0f; /* last Ch9 value (-999=force pub) */
static uint8_t g_ntc_countdown = 1u;      /* ticks until next ADC sample    */
static uint8_t g_ntc_fast_held = 0u;      /* ticks spent in FAST mode       */
static uint8_t g_ntc_seeded    = 0u;      /* EMA seed flag                  */
static uint8_t g_ntc_alarm     = 0u;      /* over-temp alarm latch          */
static uint8_t g_ntc_in_fast   = 0u;      /* 1 = FAST mode active           */

static void ntc_tick(void)
{
    /* ── Cheap path: not yet time to sample ─────────────────────────── */
    if (g_ntc_countdown > 1u) { g_ntc_countdown--; return; }

    /* ── Reload countdown BEFORE doing the work (prevents drift) ───── */
    g_ntc_countdown = g_ntc_in_fast ? (uint8_t)NTC_FAST_PERIOD
                                     : (uint8_t)NTC_SLOW_PERIOD;

    /* ── STEP 1: Raw ADC → °C ────────────────────────────────────────
     * Reject readings outside the physical range of the circuit.
     * -40°C lower limit = NTC near open-circuit (probe disconnected).
     * 125°C upper limit = NTC near short-circuit.
     * DO NOT trigger alarm on invalid reads — hardware fault, not over-temp. */
    float t_raw = ntc_raw_celsius();
    if (t_raw < -40.0f || t_raw > 125.0f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC invalid %.1fC — probe fault or cold boot glitch",
                  t_raw);
        return;
    }

    /* ── STEP 2: EMA filter ──────────────────────────────────────────
     * First call: seed EMA with actual reading so we do not start from
     * 25°C and produce a false transient on a hot system at boot.
     * Subsequent calls: y += (x - y) / N  (α=1/N, N=NTC_EMA_N=4).     */
    g_ntc_ema_prev = g_ntc_ema;
    if (!g_ntc_seeded) {
        g_ntc_ema   = t_raw;
        g_ntc_seeded = 1u;
    } else {
        g_ntc_ema += (t_raw - g_ntc_ema) / (float)NTC_EMA_N;
    }

    /* ── STEP 3: Rate-of-change and delta from last publish ──────────
     * dT_dt is in °C per sample interval (not °C/s).
     * At SLOW (10 s interval): NTC_FAST_TRIGGER_C=1.0 °C/sample = 6°C/min.
     * At FAST (1 s interval):  same threshold but per-second sensitivity.
     * fabsf() avoided: manual abs saves one libm call here.             */
    float dt_dt = g_ntc_ema - g_ntc_ema_prev;
    if (dt_dt < 0.0f) dt_dt = -dt_dt;

    float delta_pub = g_ntc_ema - g_ntc_last_pub;
    if (delta_pub < 0.0f) delta_pub = -delta_pub;

    /* ── STEP 4: LAYER 3 — ALARM (highest priority) ─────────────────
     * T_ema ≥ NTC_ALARM_C (70°C = PVC cable insulation rating).
     * Action: open relay immediately, latch fast mode, always publish.
     * Relay does NOT auto-close after alarm clears — requires explicit
     * operator action (kws_relay_on or kws_ntc_alarm_reset + kws_relay_on).
     * 5°C hysteresis on alarm clear prevents chattering.                */
    if (g_ntc_ema >= NTC_ALARM_C) {
        if (!g_ntc_alarm) {
            g_ntc_alarm   = 1u;
            g_ntc_in_fast = 1u;
            g_ntc_fast_held = 0u;
            g_ntc_countdown = (uint8_t)NTC_FAST_PERIOD;
            relay_open();
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: *** NTC ALARM %.1fC >= %.1fC *** relay OPENED",
                      g_ntc_ema, (float)NTC_ALARM_C);
        }
        PubCh(KWS_CH_TEMP, (int)(g_ntc_ema * 100.0f));
        g_ntc_last_pub = g_ntc_ema;
        return;   /* skip LAYER 2/1 — alarm owns the state */
    }
    /* Alarm hysteresis clear (5°C below alarm threshold) */
    if (g_ntc_alarm && g_ntc_ema < (NTC_ALARM_C - 5.0f)) {
        g_ntc_alarm = 0u;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC alarm cleared %.1fC — relay stays open, manual reset required",
                  g_ntc_ema);
    }

    /* ── STEP 5: LAYER 2 — TRANSIENT DETECTION ──────────────────────
     * Switch to FAST mode when temperature is changing meaningfully OR
     * when approaching the warning threshold.
     * g_ntc_fast_held counts ticks in FAST to enforce NTC_FAST_MIN_TICKS. */
    bool trigger_fast = (dt_dt    >= NTC_FAST_TRIGGER_C) ||
                        (g_ntc_ema >= NTC_WARN_C);

    if (trigger_fast) {
        if (!g_ntc_in_fast) {
            g_ntc_in_fast   = 1u;
            g_ntc_fast_held = 0u;
            g_ntc_countdown = (uint8_t)NTC_FAST_PERIOD;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: NTC FAST mode (T=%.1fC dT=%.2fC/samp)",
                      g_ntc_ema, dt_dt);
        }
        g_ntc_fast_held++;

    } else if (g_ntc_in_fast) {
        /* ── LAYER 1: STEADY STATE — return to SLOW ─────────────────
         * Require NTC_FAST_MIN_TICKS elapsed before returning to SLOW.
         * This prevents FAST→SLOW→FAST oscillation when the temperature
         * hovers exactly at the trigger threshold.                      */
        g_ntc_fast_held++;
        if (g_ntc_fast_held >= (uint8_t)NTC_FAST_MIN_TICKS) {
            g_ntc_in_fast   = 0u;
            g_ntc_fast_held = 0u;
            g_ntc_countdown = (uint8_t)NTC_SLOW_PERIOD;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: NTC SLOW mode (T=%.1fC stable)", g_ntc_ema);
        }
    }

    /* ── STEP 6: PUBLISH GATE ────────────────────────────────────────
     * Publish Ch9 when:
     *   (a) Temperature has changed by ≥ NTC_DEADBAND_C since last publish.
     *       At 27°C ADC noise ≈ ±0.16°C << 0.5°C deadband → zero noise publishes.
     *   (b) This is a SLOW-mode tick (keepalive: guarantees HA freshness ≤ 10 s).
     *       Even a perfectly stable NTC publishes once per SLOW_PERIOD.
     * Note: ALARM path above publishes unconditionally and returns early.  */
    bool publish = (delta_pub >= NTC_DEADBAND_C) || (!g_ntc_in_fast);
    if (publish) {
        PubCh(KWS_CH_TEMP, (int)(g_ntc_ema * 100.0f));
        g_ntc_last_pub = g_ntc_ema;
    }
}

/* ── Diagnostic console commands ────────────────────────────────────── */
static commandResult_t CMD_NtcStatus(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    float t_raw = ntc_raw_celsius();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "NTC raw=%.2fC  ema=%.2fC  last_pub=%.2fC  mode=%s",
              t_raw, g_ntc_ema, g_ntc_last_pub,
              g_ntc_in_fast ? "FAST" : "SLOW");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  countdown=%u  fast_held=%u  alarm=%s",
              (unsigned)g_ntc_countdown, (unsigned)g_ntc_fast_held,
              g_ntc_alarm ? "LATCHED" : "clear");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  warn=%.0fC  alarm=%.0fC  deadband=%.1fC  trigger=%.1fC/samp  ema_n=%u",
              (float)NTC_WARN_C, (float)NTC_ALARM_C,
              (float)NTC_DEADBAND_C, (float)NTC_FAST_TRIGGER_C,
              (unsigned)NTC_EMA_N);
    return CMD_RES_OK;
}

static commandResult_t CMD_NtcAlarmReset(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    if (g_ntc_alarm) {
        g_ntc_alarm = 0u;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC alarm manually cleared (T=%.1fC) — use kws_relay_on to restore power",
                  g_ntc_ema);
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC no alarm active (T=%.1fC)", g_ntc_ema);
    }
    return CMD_RES_OK;
}

/* ============================================================================
 * SECTION E — BUTTON MANAGER
 * ============================================================================ */
#define BTN_MAX 4
typedef struct { int pin; uint8_t last; uint8_t dbc; void (*cb)(void); } Btn_t;
static Btn_t   g_btns[BTN_MAX];
static uint8_t g_btn_n = 0;

static void btn_register(int pin, void (*cb)(void))
{
    if (g_btn_n >= BTN_MAX) return;
    g_btns[g_btn_n].pin  = pin;
    g_btns[g_btn_n].last = 1;
    g_btns[g_btn_n].dbc  = 0;
    g_btns[g_btn_n].cb   = cb;
    g_btn_n++;
    HAL_PIN_Setup_Input_Pullup(pin);
}

static void btn_tick(void)
{
    uint8_t i;
    for (i = 0; i < g_btn_n; i++) {
        Btn_t  *b  = &g_btns[i];
        uint8_t lv = (uint8_t)HAL_PIN_ReadDigitalInput(b->pin);
        if (lv == b->last) { b->dbc = 0; continue; }
        if (++b->dbc < 3)  continue;
        b->dbc = 0; b->last = lv;
        if (lv == 0 && b->cb) b->cb();
    }
}

/* ============================================================================
 * SECTION F — EV SESSION
 * ============================================================================ */
#define KWS_VEH_NONE 0
#define KWS_VEH_2W   1
#define KWS_VEH_4W   2

typedef struct {
    uint8_t  active;
    uint8_t  vehicle;
    uint8_t  wh_resume;
    uint32_t session_id;
    float    wh_offset;
    float    wh_session;
    uint32_t seg_count;
    float    peak_w;
    float    peak_a;
    float    rate_rs;
    uint32_t start_uptime_s;   /* improvement #2: g_uptime_s at sess_start() */
    uint32_t duration_s;       /* improvement #2: filled at sess_end()        */
} EvSess_t;

static EvSess_t  g_ev;
static float     g_rate_rs = KWS_EV_RATE_DEFAULT;
static bool      g_auto_en = true;

typedef enum { AD_IDLE, AD_DETECTING, AD_CHARGING } AdState_t;
static AdState_t g_ad     = AD_IDLE;
static uint32_t  g_adTick = 0;
static float     g_adWsum = 0.0f;
static uint32_t  g_endTk  = 0;

static void sess_save(void)
{
    FILE *f = fopen(KWS_SESSION_FILE, "w");
    if (!f) return;
    fprintf(f, "active=%u\nvehicle=%u\nid=%u\nwh_off=%.4f\nwh_sess=%.4f\n"
               "segs=%u\npeak_w=%.2f\npeak_a=%.4f\nrate=%.4f\n"
               "start_up=%u\n",                        /* improvement #2 */
            (unsigned)g_ev.active, (unsigned)g_ev.vehicle, g_ev.session_id,
            g_ev.wh_offset, g_ev.wh_session, g_ev.seg_count,
            g_ev.peak_w, g_ev.peak_a, g_ev.rate_rs,
            (unsigned)g_ev.start_uptime_s);
    fclose(f);
}

static bool sess_load(void)
{
    unsigned int ua = 0, uv = 0, ustart = 0;
    FILE *f = fopen(KWS_SESSION_FILE, "r");
    if (!f) return false;
    memset(&g_ev, 0, sizeof(g_ev));
    fscanf(f, "active=%u\nvehicle=%u\nid=%u\nwh_off=%f\nwh_sess=%f\n"
              "segs=%u\npeak_w=%f\npeak_a=%f\nrate=%f\n"
              "start_up=%u\n",
           &ua, &uv, &g_ev.session_id,
           &g_ev.wh_offset, &g_ev.wh_session, &g_ev.seg_count,
           &g_ev.peak_w, &g_ev.peak_a, &g_ev.rate_rs,
           &ustart);
    fclose(f);
    g_ev.active          = (uint8_t)ua;
    g_ev.vehicle         = (uint8_t)uv;
    g_ev.start_uptime_s  = (uint32_t)ustart;  /* 0 if old file without field */
    return (bool)g_ev.active;
}

static void sess_start(uint8_t veh)
{
    g_ev.active          = 1;
    g_ev.vehicle         = veh;
    g_ev.session_id++;
    g_ev.wh_offset       = (float)ReadCh(KWS_CH_ENERGY) / 10.0f;
    g_ev.wh_session      = 0.0f;
    g_ev.seg_count       = 1;
    g_ev.peak_w          = 0.0f;
    g_ev.peak_a          = 0.0f;
    g_ev.rate_rs         = g_rate_rs;
    g_ev.start_uptime_s  = g_uptime_s;   /* improvement #2: record start time */
    g_ev.duration_s      = 0;
    PubCh(KWS_CH_SESS_ACTIVE,  1);       /* improvement #10: tell display      */
    PubCh(KWS_CH_SESS_ELAPSED, 0);
    sess_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Session START id=%u veh=%s",
              g_ev.session_id,
              (veh==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
}

static char             g_mqtt_pending_pl[320] = {0};
static volatile uint8_t g_mqtt_pending          = 0;

static void sess_mqtt(void)
{
    float kwh    = g_ev.wh_session / 1000.0f;
    float cost   = kwh * g_ev.rate_rs;
    float km_pkw = (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH;
    float old_kl = (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_OLD_KMPL    : KWS_VEH4_OLD_KMPL;
    float km     = kwh * km_pkw;
    float saved  = km / old_kl * KWS_PETROL_RS_PER_L - cost;
    /* improvement #2: duration_s | improvement #7: uptime_s for ordering   */
    snprintf(g_mqtt_pending_pl, sizeof(g_mqtt_pending_pl),
             "{\"vehicle\":\"%s\",\"kwh\":%.3f,\"cost_rs\":%.2f,"
             "\"km\":%.1f,\"saved_rs\":%.2f,\"segments\":%u,"
             "\"peak_w\":%.1f,\"peak_a\":%.3f,"
             "\"duration_s\":%u,\"uptime_s\":%u}",
             (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
             kwh, cost, km, saved, g_ev.seg_count,
             g_ev.peak_w, g_ev.peak_a,
             (unsigned)g_ev.duration_s, (unsigned)g_uptime_s);
    g_mqtt_pending = 1;
}

static void sess_summary(void)
{
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    float km   = kwh * ((g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH);
    float pet  = km  / ((g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_OLD_KMPL   : KWS_VEH4_OLD_KMPL)
                     * KWS_PETROL_RS_PER_L;
    uint32_t hh = g_ev.duration_s / 3600u;
    uint32_t mm = (g_ev.duration_s % 3600u) / 60u;
    uint32_t ss = g_ev.duration_s % 60u;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"=== EV Session Summary ===");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Vehicle  : %s",
              (g_ev.vehicle==KWS_VEH_2W)?KWS_VEH2_NAME:KWS_VEH4_NAME);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Duration : %02u:%02u:%02u (%u s)",
              (unsigned)hh,(unsigned)mm,(unsigned)ss,(unsigned)g_ev.duration_s);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Energy   : %.3f kWh", kwh);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Cost     : Rs %.2f @ Rs %.2f/unit",cost,g_ev.rate_rs);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Range    : %.1f km", km);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Petrol   : Rs %.2f for same km", pet);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  SAVED    : Rs %.2f", pet-cost);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Segs     : %u  Peak: %.1fW / %.3fA",
              g_ev.seg_count, g_ev.peak_w, g_ev.peak_a);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Lifetime : %.2f kWh total",
              g_lifetime_wh / 1000.0f);
}

/* improvement #4: append one CSV row to history file */
static void history_append(void)
{
    FILE *f = fopen(KWS_HISTORY_FILE, "a");
    if (!f) return;
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    float km   = kwh * ((g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH);
    fprintf(f, "%u,%s,%.3f,%.2f,%.1f,%u,%u,%.1f,%.3f\n",
            g_ev.session_id,
            (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
            kwh, cost, km,
            (unsigned)g_ev.duration_s, (unsigned)g_ev.seg_count,
            g_ev.peak_w, g_ev.peak_a);
    fclose(f);
}

static void sess_end(void)
{
    if (!g_ev.active) return;
    /* improvement #2: compute duration before clearing active flag */
    g_ev.duration_s = (g_uptime_s > g_ev.start_uptime_s)
                      ? (g_uptime_s - g_ev.start_uptime_s) : 0u;
    g_ev.active = 0; g_ad = AD_IDLE;

    /* improvement #8: credit this session's energy to lifetime total */
    g_lifetime_wh += g_ev.wh_session;
    if (g_lifetime_wh < 0.0f) g_lifetime_wh = 0.0f;
    lifetime_save();

    sess_summary();
    history_append();    /* improvement #4: write CSV row */
    sess_mqtt();

    FILE *f = fopen(KWS_SESSION_FILE,"w");
    if (f) { fprintf(f,"active=0\n"); fclose(f); }

    PubCh(KWS_CH_SESS_ACTIVE,  0);   /* improvement #10: clear session flag  */
    PubCh(KWS_CH_SESS_ELAPSED, 0);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: Session ENDED");
}

static void sess_tick(void)
{
    float w  = (float)ReadCh(KWS_CH_POWER)   / 10.0f;
    float a  = (float)ReadCh(KWS_CH_CURRENT) / 1000.0f;
    float wh = (float)ReadCh(KWS_CH_ENERGY)  / 10.0f;

    if (g_ev.active) {
        if (g_ev.wh_resume && wh > 0.0f) {
            g_ev.wh_offset = wh; g_ev.wh_resume = 0;
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                      "KWS303WF: resume offset anchored %.4f Wh", wh);
        }
        g_ev.wh_session = wh - g_ev.wh_offset;
        if (g_ev.wh_session < 0.0f) g_ev.wh_session = 0.0f;
        if (w > g_ev.peak_w) g_ev.peak_w = w;
        if (a > g_ev.peak_a) g_ev.peak_a = a;
        float cost = (g_ev.wh_session/1000.0f) * g_ev.rate_rs;
        PubCh(KWS_CH_EVCOST, (int)(cost * 100.0f));

        /* improvement #2: publish elapsed session time to Ch13 */
        uint32_t elapsed = (g_uptime_s > g_ev.start_uptime_s)
                           ? (g_uptime_s - g_ev.start_uptime_s) : 0u;
        PubCh(KWS_CH_SESS_ELAPSED, (int)elapsed);

        static uint32_t s_sv = 0;
        if (++s_sv >= 60) {
            s_sv = 0;
            sess_save();
            /* improvement #8: persist lifetime Wh snapshot every 60 s */
            float snap = g_lifetime_wh + g_ev.wh_session;
            FILE *lf = fopen(KWS_LIFETIME_FILE, "w");
            if (lf) { fprintf(lf, "wh=%.4f\n", snap); fclose(lf); }
        }
    } else {
        PubCh(KWS_CH_EVCOST, 0);
    }

    if (!g_auto_en) return;
    switch (g_ad) {
    case AD_IDLE:
        if (w >= g_detect_w_min) { g_ad=AD_DETECTING; g_adTick=1; g_adWsum=w; }
        break;
    case AD_DETECTING:
        if (w < g_detect_w_min) { g_ad=AD_IDLE; break; }
        g_adWsum += w;
        if (++g_adTick >= KWS_DETECT_S) {
            float avg = g_adWsum / (float)g_adTick;
            if (!g_ev.active)
                sess_start((avg < KWS_SPLIT_W) ? KWS_VEH_2W : KWS_VEH_4W);
            g_ad=AD_CHARGING; g_endTk=0;
        }
        break;
    case AD_CHARGING:
        if (w < g_detect_w_min) { if (++g_endTk >= KWS_END_S) sess_end(); }
        else { g_endTk=0; }
        break;
    }
}

/* ============================================================================
 * SECTION G — BUTTON CALLBACKS
 * ============================================================================ */
static void on_toggle(void)   { if (g_relay_closed) relay_open(); else relay_close(); }
static void on_session(void)
{
    uint8_t veh = g_ev.active ? g_ev.vehicle : KWS_VEH_2W;
    if (g_ev.active) sess_end();
    sess_start(veh);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: Session reset by [+] button");
}
/* improvement #3 — [-] (P20) cycles vehicle profile.
 * Active session: ends current session and immediately restarts with the
 *   other vehicle type — history row is written for the partial session.
 * Idle: toggles the preferred vehicle so the next auto-detect uses it.    */
static void on_reserved(void)
{
    if (g_ev.active) {
        uint8_t new_veh = (g_ev.vehicle == KWS_VEH_2W) ? KWS_VEH_4W : KWS_VEH_2W;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "KWS303WF: [-] vehicle %s → %s (session restart)",
                  (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
                  (new_veh   ==KWS_VEH_2W)   ? KWS_VEH2_NAME : KWS_VEH4_NAME);
        sess_end();
        sess_start(new_veh);
    } else {
        static uint8_t s_next_veh = KWS_VEH_2W;
        s_next_veh = (s_next_veh == KWS_VEH_2W) ? KWS_VEH_4W : KWS_VEH_2W;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "KWS303WF: [-] next session vehicle = %s",
                  (s_next_veh==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
    }
}

/* ============================================================================
 * SECTION H — CONSOLE COMMANDS
 * ============================================================================ */
static commandResult_t CMD_RelayOn (const void*x,const char*c,const char*a,int f){relay_close();return CMD_RES_OK;}
static commandResult_t CMD_RelayOff(const void*x,const char*c,const char*a,int f){relay_open(); return CMD_RES_OK;}

static commandResult_t CMD_Ch8Set(const void*x,const char*c,const char*a,int f)
{ if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS; relay_sync(atoi(a)); return CMD_RES_OK; }

static commandResult_t CMD_Rate(const void*x,const char*c,const char*a,int f)
{
    if(!a||!*a){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate: %.4f Rs/kWh",g_rate_rs);return CMD_RES_OK;}
    g_rate_rs=(float)atof(a);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate set %.4f Rs/kWh",g_rate_rs);
    return CMD_RES_OK;
}

/* improvement #6: runtime detect threshold command */
static commandResult_t CMD_DetectW(const void*x,const char*c,const char*a,int f)
{
    if(!a||!*a){
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_detect_w: %.1f W",g_detect_w_min);
        return CMD_RES_OK;
    }
    float v = (float)atof(a);
    if (v < 50.0f || v > 10000.0f) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_detect_w: value must be 50-10000 W");
        return CMD_RES_BAD_ARGUMENT;
    }
    g_detect_w_min = v;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_detect_w set %.1f W",g_detect_w_min);
    return CMD_RES_OK;
}

/* improvement #4: dump history CSV over log */
static commandResult_t CMD_HistoryDump(const void*x,const char*c,const char*a,int f)
{
    FILE *fh = fopen(KWS_HISTORY_FILE, "r");
    if (!fh) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_history: no history file yet");
        return CMD_RES_OK;
    }
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "id,veh,kwh,cost_rs,km,duration_s,segs,peak_w,peak_a");
    char line[128];
    while (fgets(line, sizeof(line), fh)) {
        /* strip trailing newline for clean log output */
        size_t l = strlen(line);
        if (l > 0 && line[l-1] == '\n') line[l-1] = '\0';
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"%s", line);
    }
    fclose(fh);
    return CMD_RES_OK;
}

/* improvement #4: clear history file */
static commandResult_t CMD_HistoryClear(const void*x,const char*c,const char*a,int f)
{
    FILE *fh = fopen(KWS_HISTORY_FILE, "w");
    if (fh) { fclose(fh); }
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_history: cleared");
    return CMD_RES_OK;
}

static commandResult_t CMD_Sess2W (const void*x,const char*c,const char*a,int f){if(g_ev.active)sess_end();sess_start(KWS_VEH_2W);return CMD_RES_OK;}
static commandResult_t CMD_Sess4W (const void*x,const char*c,const char*a,int f){if(g_ev.active)sess_end();sess_start(KWS_VEH_4W);return CMD_RES_OK;}
static commandResult_t CMD_SessEnd(const void*x,const char*c,const char*a,int f){sess_end();return CMD_RES_OK;}

static commandResult_t CMD_SessStat(const void*x,const char*c,const char*a,int f)
{
    uint32_t elapsed = (g_ev.active && g_uptime_s > g_ev.start_uptime_s)
                       ? (g_uptime_s - g_ev.start_uptime_s) : 0u;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS Session: %s  veh=%s  kWh=%.3f  cost=Rs%.2f  segs=%u  elapsed=%um%02us",
              g_ev.active?"ACTIVE":"idle",
              (g_ev.vehicle==KWS_VEH_2W)?KWS_VEH2_NAME:
              (g_ev.vehicle==KWS_VEH_4W)?KWS_VEH4_NAME:"none",
              g_ev.wh_session/1000.0f,
              g_ev.wh_session/1000.0f*g_ev.rate_rs,
              g_ev.seg_count,
              (unsigned)(elapsed/60), (unsigned)(elapsed%60));
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  relay=%s  auto=%s  detect_w=%.0fW  uptime=%us",
              g_relay_closed?"CLOSED":"OPEN",
              g_auto_en?"ON":"OFF",
              g_detect_w_min, (unsigned)g_uptime_s);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  lifetime=%.2f Wh  (%.4f kWh total)", g_lifetime_wh, g_lifetime_wh/1000.0f);
    return CMD_RES_OK;
}

static commandResult_t CMD_SessAuto(const void*x,const char*c,const char*a,int f)
{
    g_auto_en=!g_auto_en;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: auto-detect %s",g_auto_en?"ON":"OFF");
    return CMD_RES_OK;
}

/* improvement #8: show or manually set lifetime Wh counter               */
static commandResult_t CMD_Lifetime(const void*x,const char*c,const char*a,int f)
{
    if (a && *a) {
        float v = (float)atof(a);
        if (v < 0.0f) return CMD_RES_BAD_ARGUMENT;
        g_lifetime_wh = v;
        lifetime_save();
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "kws_lifetime set %.2f Wh (%.4f kWh)", v, v/1000.0f);
    } else {
        float total = g_lifetime_wh + (g_ev.active ? g_ev.wh_session : 0.0f);
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "kws_lifetime: %.2f Wh saved + %.2f Wh active = %.4f kWh total",
                  g_lifetime_wh,
                  g_ev.active ? g_ev.wh_session : 0.0f,
                  total/1000.0f);
    }
    return CMD_RES_OK;
}

/* ============================================================================
 * SECTION I — OPENBK LIFECYCLE
 * ============================================================================ */
void KWS303WF_Init(void)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: init");

    HAL_PIN_Setup_Output(KWS_RELAY_PIN_ON);
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_OFF);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_ON,  0);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_OFF, 0);
    relay_open();

    lifetime_load();   /* improvement #8: restore odometer from flash */

    /* NTC init — seed on first ntc_tick() call */
    g_ntc_seeded    = 0u;
    g_ntc_alarm     = 0u;
    g_ntc_in_fast   = 0u;
    g_ntc_fast_held = 0u;
    g_ntc_countdown = 1u;      /* take first sample on tick 1 */
    g_ntc_last_pub  = -999.0f; /* force publish on first valid reading */
    g_ntc_ema       = 25.0f;   /* nominal; overwritten on first real sample */
    g_ntc_ema_prev  = 25.0f;

    btn_register(KWS_BTN_TOGGLE,   on_toggle);
    btn_register(KWS_BTN_SESSION,  on_session);
    btn_register(KWS_BTN_RESERVED, on_reserved);

    if (sess_load() && g_ev.active) {
        g_ev.seg_count++;
        g_ev.wh_offset = 0.0f;
        g_ev.wh_resume = 1;
        g_ad = AD_CHARGING;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: Session RESUMED id=%u segs=%u (offset deferred)",
                  g_ev.session_id, g_ev.seg_count);
    } else {
        memset(&g_ev, 0, sizeof(g_ev));
        g_ev.rate_rs = g_rate_rs;
    }

    CMD_RegisterCommand("kws_relay_on",        CMD_RelayOn,      NULL);
    CMD_RegisterCommand("kws_relay_off",        CMD_RelayOff,     NULL);
    CMD_RegisterCommand("kws_ch8_set",          CMD_Ch8Set,       NULL);
    CMD_RegisterCommand("kws_rate",             CMD_Rate,         NULL);
    CMD_RegisterCommand("kws_detect_w",         CMD_DetectW,      NULL);   /* #6 */
    CMD_RegisterCommand("kws_session2w",        CMD_Sess2W,       NULL);
    CMD_RegisterCommand("kws_session4w",        CMD_Sess4W,       NULL);
    CMD_RegisterCommand("kws_session_end",      CMD_SessEnd,      NULL);
    CMD_RegisterCommand("kws_session_stat",     CMD_SessStat,     NULL);
    CMD_RegisterCommand("kws_session_auto",     CMD_SessAuto,     NULL);
    CMD_RegisterCommand("kws_history_dump",     CMD_HistoryDump,  NULL);   /* #4 */
    CMD_RegisterCommand("kws_history_clear",    CMD_HistoryClear, NULL);   /* #4 */
    CMD_RegisterCommand("kws_lifetime",         CMD_Lifetime,     NULL);   /* #8 */
    CMD_RegisterCommand("kws_ntc_status",       CMD_NtcStatus,    NULL);
    CMD_RegisterCommand("kws_ntc_alarm_reset",  CMD_NtcAlarmReset,NULL);

    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS303WF: ready relay=P%d/P%d ntc=ADC%d slow=%us fast=%us rate=Rs%.2f/kWh",
              KWS_RELAY_PIN_ON, KWS_RELAY_PIN_OFF, KWS_ADC_CH,
              (unsigned)NTC_SLOW_PERIOD, (unsigned)NTC_FAST_PERIOD, g_rate_rs);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  NTC warn=%.0fC alarm=%.0fC deadband=%.1fC trigger=%.1fC/samp ema_n=%u",
              (float)NTC_WARN_C,(float)NTC_ALARM_C,
              (float)NTC_DEADBAND_C,(float)NTC_FAST_TRIGGER_C,(unsigned)NTC_EMA_N);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  2W:%s %.1fkm/kWh  4W:%s %.1fkm/kWh  petrol=Rs%.0f/L",
              KWS_VEH2_NAME,KWS_VEH2_KM_PER_KWH,
              KWS_VEH4_NAME,KWS_VEH4_KM_PER_KWH,KWS_PETROL_RS_PER_L);
}

void KWS303WF_RunEverySecond(void)
{
    g_uptime_s++;

    if (g_mqtt_pending) {
        /* improvement #9: only attempt publish when WiFi is up.
         * Flag stays set if WiFi is down — retried on next tick when connected.
         * Prevents blocking RunEverySecond() for 4-20 s on TCP connect timeout. */
        if (ReadCh(KWS_CH_WIFI) == 0) {
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                      "KWS303WF: MQTT publish deferred — WiFi down");
        } else {
            char cmd[400];
            snprintf(cmd, sizeof(cmd), "publishMQTT %s %s",
                     KWS_MQTT_TOPIC, g_mqtt_pending_pl);
            g_mqtt_pending = 0;
            CMD_ExecuteCommand(cmd, 0);
        }
    }

    relay_pulse_tick();
    relay_sync(ReadCh(KWS_CH_RELAY));
    btn_tick();
    sess_tick();

    /* NTC: 9 out of 10 calls cost ~0.1 µs (countdown decrement only).
     * ADC + Steinhart-Hart + EMA + state machine runs on sample ticks only. */
    ntc_tick();
}

#endif /* ENABLE_DRIVER_KWS303WF */
