/*
 * drv_kws303wf.c — KWS-303WF Device Application Driver for OpenBK7231N
 * Version : see KWS_FW_VERSION_STR / KWS_FW_BUILD_DATE in obk_config.h
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
 *   Ch 14 = NTC thermal alarm (0=normal, 1=alarm latched/relay opened)
 *                             — read by drv_st7735 to colour temp red + "THM"
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
 * ── RELAY ↔ SESSION LINK (FIX-UX1 + FIX-UX3) ───────────────────────────
 *   relay CLOSE → arms AD_DETECTING with KWS_DETECT_S_FAST window (5 s).
 *                 If load ≥ g_detect_w_min sustained for 5 s → sess_start().
 *                 Normal load / standby (<500 W) → AD_IDLE, no session.
 *   relay OPEN  → sess_end() fires immediately + AD_IDLE reset.
 *   [+] SESSION button — if relay open: closes relay (arms detect window).
 *                        If relay closed + no session: starts session now.
 *                        If relay closed + session active: restarts timer.
 *   [-] VEHICLE button — cycles vehicle profile (2W↔4W)
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
 *   setChannelType 14 ReadOnly   (NTC thermal alarm latch)
 *
 * ── New console commands ───────────────────────────────────────────────────
 *   kws_detect_w [W]        — get/set auto-detect power threshold (default 500W)
 *   kws_history_dump        — print session history CSV to UART log
 *   kws_history_clear       — truncate kws_history.csv
 *   kws_lifetime [Wh]       — get/set lifetime energy odometer
 *   kws_session_stat        — shows elapsed time, detect_w, lifetime_now
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
/* IMP-O: OBK NTP driver — provides NTP_IsSynced(), NTP_GetHour/Min/Sec/Day/Month/Year().
 * Guarded by ENABLE_NTP which is set for PLATFORM_BEKEN in obk_config.h.
 * If NTP is not synced, sess_mqtt() falls back to "uptime_s" field only.   */
#ifdef ENABLE_NTP
#include "drv_ntp.h"
#endif

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

/* ── LittleFS fopen("w") quirk on BK7231N ────────────────────────────────────
 * fopen(path,"w") returns NULL when the file does not yet exist.
 * fopen(path,"a") creates it.  robust_fopen_w() tries "w" first; on failure
 * it touches the file with "a" then retries "w".  See drv_ht7017.c for the
 * full root-cause note (confirmed 2026-03-06).
 * ──────────────────────────────────────────────────────────────────────────── */
static FILE *robust_fopen_w(const char *path)
{
    FILE *f = fopen(path, "w");
    if (f) return f;
    f = fopen(path, "a");
    if (f) fclose(f);
    return fopen(path, "w");
}

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
/* FIX-UX3: two-speed confirmation window.
 * KWS_DETECT_S      — 30 s window for auto-detect from AD_IDLE (background).
 * KWS_DETECT_S_FAST — 5 s window triggered immediately after relay_close().
 *   The relay close IS the user's intent — confirm load type quickly and start
 *   the session without a 30 s dead zone.  Normal appliances (<g_detect_w_min)
 *   fail the threshold in tick 1 and the state returns to AD_IDLE, no session. */
#define KWS_DETECT_S          30
#define KWS_DETECT_S_FAST      5
#define KWS_END_S             60
#define KWS_SPLIT_W         4500.0f
#define KWS_RELAY_PIN_ON      8
#define KWS_RELAY_PIN_OFF     7
/* NOTE: KWS_RELAY_PULSE_MS is defined for documentation purposes only.
 * The actual pulse duration is 1 second (g_uptime_s + 1), enforced by
 * relay_pulse_tick() called from KWS303WF_RunQuickTick() (~10 ms tick).
 * g_pulse_off_at uses second-resolution g_uptime_s, so coil de-energises
 * within ~10 ms of the second boundary — well within the latching relay
 * minimum (≥100 ms).  De-energise overshoot improved from ≤1 s to ≤10 ms. */
#define KWS_RELAY_PULSE_MS  200   /* reference only — actual pulse = 1 s via uptime tick */
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
/* IMP-A: NTC thermal alarm channel — 0=normal, 1=alarm latched (relay opened).
 * Read by drv_st7735 to colour the temperature field red and show "THM" in the
 * alarm zone when an over-temperature trip has occurred.
 * Set in autoexec: setChannelType 14 ReadOnly                              */
#define KWS_CH_NTC_ALARM     14

/* FIX-PATH: BK7231N LittleFS VFS does NOT accept leading slash.
 * "/filename" → fopen returns NULL silently. "filename" works correctly.
 * Same fix already applied in drv_ht7017.c (FIX-CAL-1). */
#define KWS_SESSION_FILE     "kws_session.cfg"
#define KWS_HISTORY_FILE     "kws_history.csv"   /* append-only session log   */
#define KWS_LIFETIME_FILE    "kws_lifetime.cfg"  /* persists cumulative Wh    */
#define KWS_MQTT_TOPIC       "home/ev/session"
/* IMP-B: maximum rows retained in kws_history.csv.
 * Each row ≈ 80 bytes.  200 rows = ~16 kB — well within the 512 kB LittleFS
 * partition.  At 10 sessions/day this retains ~20 days of history while
 * preventing unbounded flash growth.  Adjustable here at compile time.     */
#define KWS_HISTORY_MAX_ROWS  200u

/* ── New LittleFS files (v1.1.0) ─────────────────────────────────────────
 * All paths: no leading slash (BK7231N LittleFS VFS constraint).          */
#define KWS_RATE_FILE        "kws_rate.cfg"        /* IMP-G: persisted Rs/kWh rate         */
#define KWS_PROTECT_FILE     "kws_protection.cfg"  /* IMP-H: persisted OV/UV/OC thresholds */
#define KWS_DAILY_FILE       "kws_daily.cfg"       /* IMP-J: daily kWh counter + reset ts  */
#define KWS_FAULTS_FILE      "kws_faults.log"      /* IMP-M: last-N fault log              */
/* IMP-P: per-vehicle runtime efficiency — loaded/saved via kws_vehicles.cfg.
 * Format: "veh2_km=XX.X\nveh4_km=XX.X\n"
 * Allows user to tune kWh→km conversion for their specific vehicle.        */
#define KWS_VEHICLES_FILE    "kws_vehicles.cfg"    /* IMP-P: per-vehicle efficiency factors */
/* IMP-Q: off-peak charge scheduling — loaded/saved via kws_schedule.cfg.
 * Format: "enabled=1\nstart=22\nend=6\n"  (hour integers, 0-23, 24h clock)
 * When enabled, relay is only allowed to close during [start, end) window.
 * Wraps midnight correctly (e.g. 22:00 → 06:00).                          */
#define KWS_SCHEDULE_FILE    "kws_schedule.cfg"    /* IMP-Q: off-peak window config        */

/* ── IMP-Q: off-peak scheduling defaults ─────────────────────────────────
 * KWS_SCHED_DEFAULT_START   Default window start hour (10 PM)
 * KWS_SCHED_DEFAULT_END     Default window end hour   (6 AM)
 * Disabled by default — user must run kws_schedule enable to activate.    */
#define KWS_SCHED_DEFAULT_START  22u   /* hour 0-23 — default window opens at 22:00 */
#define KWS_SCHED_DEFAULT_END     6u   /* hour 0-23 — default window closes at 06:00 */

/* ── IMP-R: multi-tier time-of-use tariff ────────────────────────────────
 * Up to KWS_RATE_TIER_MAX time-of-use slots, each with hour range + rate.
 * Stored as an extension of kws_rate.cfg:
 *   rate=7.00          (base/fallback rate, always present)
 *   tiers=2            (number of active TOU tiers, 0 = disabled)
 *   tier0=22,6,5.00    (start_h, end_h, Rs/kWh)
 *   tier1=9,17,9.50    (peak hours)
 * When tiers=0, single g_rate_rs is used as before.                       */
#define KWS_RATE_TIER_MAX    3u   /* max TOU tiers — 3 covers peak/off-peak/standard */

/* ── IMP-G: electricity rate persistence ─────────────────────────────────
 * Rate is loaded from KWS_RATE_FILE at Init().  Saved via kws_rate <val>. */

/* ── IMP-H: protection threshold defaults ────────────────────────────────
 * KWS_OV_DEFAULT   Over-voltage trip (V).  HT7017 raises alarm Ch7=1.
 * KWS_UV_DEFAULT   Under-voltage trip (V). HT7017 raises alarm Ch7=2.
 * KWS_OC_DEFAULT   Over-current trip (A).  HT7017 raises alarm Ch7=3.
 * These match the existing HT7017 defaults; persist user changes via
 * KWS_PROTECT_FILE.  Written to HT7017 registers at Init() after load.    */
#define KWS_OV_DEFAULT       250.0f  /* V — over-voltage threshold           */
#define KWS_UV_DEFAULT       180.0f  /* V — under-voltage threshold          */
#define KWS_OC_DEFAULT        16.0f  /* A — over-current threshold           */

/* ── IMP-I: CO2 saved estimate ───────────────────────────────────────────
 * India average grid emission factor 2024 (CEA) = 708 g CO2/kWh.
 * Configurable at compile time; emitted in MQTT session payload as
 * "co2_saved_g" (integer grams, EV vs equivalent fossil fuel).            */
#define KWS_CO2_GRID_G_PER_KWH  708.0f  /* g CO2 per kWh from grid India     */

/* ── IMP-J: daily kWh summary ────────────────────────────────────────────
 * KWS_DAILY_TOPIC  MQTT topic for midnight daily summary publish.
 * Midnight is detected by g_uptime_s crossing a 24 h boundary relative to
 * g_daily_start_uptime (set at Init() = 0, reset on each midnight event). */
#define KWS_DAILY_TOPIC      "home/ev/daily"

/* ── IMP-K: reboot counter ───────────────────────────────────────────────
 * Reboot count stored as a separate field in kws_lifetime.cfg.
 * Format: "wh=XXXX\nreboots=N\n"
 * On Init() the reboot counter is incremented ONCE and persisted.         */

/* ── IMP-L: MQTT heartbeat / keepalive ──────────────────────────────────
 * KWS_HEARTBEAT_S    Interval between periodic status publishes (seconds).
 *                    Default 300 s (5 min) — HA entities expire at 6× =
 *                    30 min, giving comfortable margin.
 * KWS_HEARTBEAT_TOPIC MQTT topic for periodic status payload.
 * The heartbeat payload is compact: relay, temp, session_active, lifetime,
 * uptime, wifi.  Same WiFi + attempt cap guard as session MQTT.           */
#define KWS_HEARTBEAT_S      300u               /* seconds between publishes  */
#define KWS_HEARTBEAT_TOPIC  "home/ev/status"

/* ── IMP-M: fault log ─────────────────────────────────────────────────────
 * KWS_FAULT_MAX_ROWS  Maximum fault entries retained in kws_faults.log.
 * Each row ≤ 80 bytes.  20 rows = ~1.6 kB.  On overflow oldest row is NOT
 * dropped (no LittleFS seek/shift on BK7231N); the log is capped in place.
 * The user clears with kws_faults_clear.                                  */
#define KWS_FAULT_MAX_ROWS   20u


static uint32_t g_sess_sv = 0;

static float     g_rate_rs = KWS_EV_RATE_DEFAULT;
static bool      g_auto_en = true;

typedef enum { AD_IDLE, AD_DETECTING, AD_CHARGING } AdState_t;
static AdState_t g_ad     = AD_IDLE;
static uint32_t  g_adTick = 0;
static float     g_adWsum = 0.0f;
static uint32_t  g_endTk  = 0;
/* BUG-16 FIX: g_adFast was declared at line 714 but first used at line 382
 * inside relay_open() and relay_close().  C99 requires all file-scope vars to
 * be declared before any function that references them is compiled.  Moved
 * here to sit with the rest of the auto-detect state.
 * Set to 1 by relay_close() to use KWS_DETECT_S_FAST instead of KWS_DETECT_S.
 * Cleared when AD_DETECTING exits (sess_start, AD_IDLE, or relay_open).     */
static uint8_t   g_adFast = 0;




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

/* ── IMP-G: persisted electricity rate ───────────────────────────────────
 * Loaded from KWS_RATE_FILE at Init(); saved when kws_rate <val> is used. */
/* (g_rate_rs already declared above — IMP-G adds persistence around it)   */

/* ── IMP-H: protection thresholds ────────────────────────────────────────
 * Loaded from KWS_PROTECT_FILE; written to HT7017 at Init() after load.
 * These are runtime values; defaults defined in Section A.                */
static float    g_ov_thr    = KWS_OV_DEFAULT;  /* over-voltage trip (V)    */
static float    g_uv_thr    = KWS_UV_DEFAULT;  /* under-voltage trip (V)   */
static float    g_oc_thr    = KWS_OC_DEFAULT;  /* over-current trip (A)    */

/* ── IMP-J: daily kWh accumulator ────────────────────────────────────────
 * g_daily_kwh   accumulates kWh from session ends during the current day.
 * g_daily_start_uptime  uptime_s at the start of the current day period.
 * g_daily_last_midnight uptime_s at the last midnight reset (for detection).
 * g_daily_mqtt_sent     1 = daily summary already sent this midnight event.*/
static float    g_daily_kwh           = 0.0f;
static uint32_t g_daily_start_uptime  = 0u;
static uint32_t g_daily_last_midnight = 0u;
static uint8_t  g_daily_mqtt_sent     = 0u;

/* ── IMP-K: reboot counter ────────────────────────────────────────────── */
static uint32_t g_reboot_count = 0u;   /* loaded from kws_lifetime.cfg     */

/* ── IMP-L: MQTT heartbeat ───────────────────────────────────────────────
 * g_hb_tick  counts seconds; triggers a publish when it reaches KWS_HEARTBEAT_S. */
static uint32_t g_hb_tick = 0u;

/* ── IMP-P: per-vehicle runtime efficiency factors ───────────────────────
 * Runtime equivalents of the compile-time KWS_VEH2/4_KM_PER_KWH defines.
 * Loaded from KWS_VEHICLES_FILE; set via kws_veh_config command.
 * Initialised to compile-time defaults so the firmware works out of the box
 * without the file existing.                                               */
static float g_veh2_km_pkwh = KWS_VEH2_KM_PER_KWH; /* 2-wheeler km per kWh */
static float g_veh4_km_pkwh = KWS_VEH4_KM_PER_KWH; /* 4-wheeler km per kWh */
static float g_veh2_old_kmpl = KWS_VEH2_OLD_KMPL;  /* 2-wheeler petrol km/L */
static float g_veh4_old_kmpl = KWS_VEH4_OLD_KMPL;  /* 4-wheeler petrol km/L */

/* ── IMP-Q: off-peak scheduling state ───────────────────────────────────
 * g_sched_enabled  1 = scheduling active; relay blocked outside window.
 * g_sched_start    Hour (0-23) when the charge window opens.
 * g_sched_end      Hour (0-23) when the charge window closes.
 * g_sched_blocked  1 = relay was auto-blocked this tick; log once.        */
static uint8_t  g_sched_enabled = 0u;               /* off by default        */
static uint8_t  g_sched_start   = KWS_SCHED_DEFAULT_START;
static uint8_t  g_sched_end     = KWS_SCHED_DEFAULT_END;
static uint8_t  g_sched_blocked = 0u;               /* for log dedup         */

/* ── IMP-R: time-of-use tariff tiers ────────────────────────────────────
 * Array of up to KWS_RATE_TIER_MAX active TOU tiers.
 * g_rate_tiers_n = 0 means disabled — falls back to g_rate_rs.           */
typedef struct {
    uint8_t start_h;   /* hour window opens  (0-23) */
    uint8_t end_h;     /* hour window closes (0-23) — wraps midnight if start > end */
    float   rate_rs;   /* Rs/kWh for this window */
} RateTier_t;
static RateTier_t g_rate_tiers[KWS_RATE_TIER_MAX];
static uint8_t    g_rate_tiers_n = 0u;  /* active tier count; 0 = TOU disabled */

/*
 * lifetime_load() — v1.1.0
 * IMP-K: also reads reboots= field from kws_lifetime.cfg.
 * The wh= field is mandatory; reboots= is optional (old file = 0 reboots).
 */
static void lifetime_load(void)
{
    FILE *f = fopen(KWS_LIFETIME_FILE, "r");
    if (!f) return;
    float tmp = 0.0f;
    unsigned int rb = 0u;
    /* Read mandatory wh= line; reboots= line is optional (old format). */
    int n = fscanf(f, "wh=%f\n", &tmp);
    if (n == 1) {
        /* Attempt to read optional reboots line — ignore parse failure. */
        fscanf(f, "reboots=%u\n", &rb);
    }
    fclose(f);
    /* BUG-8/9 FIX: check fscanf return and reject NaN/Inf/negative. */
    if (n != 1 || !(tmp >= 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: lifetime file invalid (n=%d val=%f) — reset to 0", n, tmp);
        g_lifetime_wh   = 0.0f;
        g_reboot_count  = 0u;
        return;
    }
    g_lifetime_wh  = tmp;
    g_reboot_count = (uint32_t)rb;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: lifetime loaded %.2f Wh (%.4f kWh) reboots=%u",
              g_lifetime_wh, g_lifetime_wh / 1000.0f, (unsigned)g_reboot_count);
}

static void lifetime_save(void)
{
    FILE *f = robust_fopen_w(KWS_LIFETIME_FILE);
    if (!f) return;
    /* IMP-K: persist reboot_count alongside Wh so a single file covers both.
     * Format: "wh=XXXX.XXXX\nreboots=N\n"
     * Backwards-compatible: old firmware ignores the reboots line.        */
    fprintf(f, "wh=%.4f\nreboots=%u\n", g_lifetime_wh, (unsigned)g_reboot_count);
    fclose(f);
    g_lifetime_dirty = 0u;
}

/* ── IMP-G: rate persistence ─────────────────────────────────────────────
 * IMP-G: rate_load() — restore saved electricity tariff from flash.
 * Called at Init() before the first RunEverySecond.                       */
static void rate_load(void)
{
    FILE *f = fopen(KWS_RATE_FILE, "r");
    if (!f) return;
    float tmp = 0.0f;
    int n = fscanf(f, "rate=%f\n", &tmp);
    fclose(f);
    if (n != 1 || !(tmp > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: rate file invalid (n=%d val=%f) — using default %.2f",
                  n, tmp, KWS_EV_RATE_DEFAULT);
        return;
    }
    g_rate_rs = tmp;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: rate loaded %.4f Rs/kWh", g_rate_rs);
}

/* IMP-G: rate_save() — persist current tariff to flash. */
static void rate_save(void)
{
    FILE *f = robust_fopen_w(KWS_RATE_FILE);
    if (!f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: rate_save failed — %s", KWS_RATE_FILE);
        return;
    }
    fprintf(f, "rate=%.4f\n", g_rate_rs);
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: rate saved %.4f Rs/kWh", g_rate_rs);
}

/* ── IMP-H: protection threshold persistence ────────────────────────────
 * IMP-H: protect_load() — restore OV/UV/OC thresholds from flash.        */
static void protect_load(void)
{
    FILE *f = fopen(KWS_PROTECT_FILE, "r");
    if (!f) return;
    float ov = 0.0f, uv = 0.0f, oc = 0.0f;
    int n = fscanf(f, "ov=%f\nuv=%f\noc=%f\n", &ov, &uv, &oc);
    fclose(f);
    /* Validate: all three must parse; all must be finite and positive.    */
    if (n != 3 || !(ov > 0.0f) || !(uv > 0.0f) || !(oc > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: protect file invalid (n=%d) — using defaults", n);
        return;
    }
    g_ov_thr = ov;
    g_uv_thr = uv;
    g_oc_thr = oc;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: protection loaded OV=%.1fV UV=%.1fV OC=%.1fA",
              g_ov_thr, g_uv_thr, g_oc_thr);
}

/* IMP-H: protect_save() — persist current thresholds to flash.
 * Called when any threshold is changed via kws_protect command.           */
static void protect_save(void)
{
    FILE *f = robust_fopen_w(KWS_PROTECT_FILE);
    if (!f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: protect_save failed — %s", KWS_PROTECT_FILE);
        return;
    }
    fprintf(f, "ov=%.2f\nuv=%.2f\noc=%.2f\n", g_ov_thr, g_uv_thr, g_oc_thr);
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: protection saved OV=%.1fV UV=%.1fV OC=%.1fA",
              g_ov_thr, g_uv_thr, g_oc_thr);
}

/* ── IMP-J: daily kWh persistence ───────────────────────────────────────
 * IMP-J: daily_load() — restore today's accumulated kWh from flash.
 * Format: "kwh=XXXX.XXX\nstart=%u\n"
 * The start field is g_uptime_s at the last midnight reset.
 * On reboot during the same calendar day this restores partial accumulation.*/
static void daily_load(void)
{
    FILE *f = fopen(KWS_DAILY_FILE, "r");
    if (!f) return;
    float kwh = 0.0f;
    uint32_t start = 0u;
    int n = fscanf(f, "kwh=%f\nstart=%u\n", &kwh, &start);
    fclose(f);
    if (n != 2 || !(kwh >= 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: daily file invalid (n=%d) — reset", n);
        return;
    }
    g_daily_kwh          = kwh;
    g_daily_start_uptime = start;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: daily loaded %.3f kWh (start_uptime=%u s)",
              g_daily_kwh, (unsigned)g_daily_start_uptime);
}

/* IMP-J: daily_save() — persist daily accumulator to flash. */
static void daily_save(void)
{
    FILE *f = robust_fopen_w(KWS_DAILY_FILE);
    if (!f) return;
    fprintf(f, "kwh=%.3f\nstart=%u\n",
            g_daily_kwh, (unsigned)g_daily_start_uptime);
    fclose(f);
}

/* ── IMP-P: per-vehicle efficiency persistence ───────────────────────────
 * IMP-P: vehicles_load() — restore per-vehicle km/kWh factors from flash.
 * Falls back silently to compile-time defaults if file is missing/corrupt. */
static void vehicles_load(void)
{
    FILE *f = fopen(KWS_VEHICLES_FILE, "r");
    if (!f) return;
    float v2 = 0.0f, v4 = 0.0f, p2 = 0.0f, p4 = 0.0f;
    int n = fscanf(f, "veh2_km=%f\nveh4_km=%f\nveh2_kmpl=%f\nveh4_kmpl=%f\n",
                   &v2, &v4, &p2, &p4);
    fclose(f);
    /* Require all four values; all must be finite and positive. */
    if (n != 4 || !(v2 > 0.0f) || !(v4 > 0.0f) || !(p2 > 0.0f) || !(p4 > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: vehicles file invalid (n=%d) — using defaults", n);
        return;
    }
    g_veh2_km_pkwh  = v2;
    g_veh4_km_pkwh  = v4;
    g_veh2_old_kmpl = p2;
    g_veh4_old_kmpl = p4;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: vehicles loaded 2W=%.1fkm/kWh(%.1fkmpl) 4W=%.1fkm/kWh(%.1fkmpl)",
              g_veh2_km_pkwh, g_veh2_old_kmpl, g_veh4_km_pkwh, g_veh4_old_kmpl);
}

/* IMP-P: vehicles_save() — persist current efficiency factors. */
static void vehicles_save(void)
{
    FILE *f = robust_fopen_w(KWS_VEHICLES_FILE);
    if (!f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: vehicles_save failed — %s", KWS_VEHICLES_FILE);
        return;
    }
    fprintf(f, "veh2_km=%.2f\nveh4_km=%.2f\nveh2_kmpl=%.2f\nveh4_kmpl=%.2f\n",
            g_veh2_km_pkwh, g_veh4_km_pkwh, g_veh2_old_kmpl, g_veh4_old_kmpl);
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: vehicles saved 2W=%.1fkm/kWh 4W=%.1fkm/kWh",
              g_veh2_km_pkwh, g_veh4_km_pkwh);
}

/* ── IMP-Q: off-peak schedule persistence ────────────────────────────────
 * IMP-Q: schedule_load() — restore scheduling config from flash.          */
static void schedule_load(void)
{
    FILE *f = fopen(KWS_SCHEDULE_FILE, "r");
    if (!f) return;
    unsigned int en = 0u, sh = 0u, eh = 0u;
    int n = fscanf(f, "enabled=%u\nstart=%u\nend=%u\n", &en, &sh, &eh);
    fclose(f);
    if (n != 3 || sh > 23u || eh > 23u) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: schedule file invalid (n=%d) — disabled", n);
        return;
    }
    g_sched_enabled = (uint8_t)(en ? 1u : 0u);
    g_sched_start   = (uint8_t)sh;
    g_sched_end     = (uint8_t)eh;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: schedule loaded enabled=%u window=%02u:00-%02u:00",
              (unsigned)g_sched_enabled, (unsigned)g_sched_start, (unsigned)g_sched_end);
}

/* IMP-Q: schedule_save() — persist scheduling config. */
static void schedule_save(void)
{
    FILE *f = robust_fopen_w(KWS_SCHEDULE_FILE);
    if (!f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: schedule_save failed — %s", KWS_SCHEDULE_FILE);
        return;
    }
    fprintf(f, "enabled=%u\nstart=%u\nend=%u\n",
            (unsigned)g_sched_enabled, (unsigned)g_sched_start, (unsigned)g_sched_end);
    fclose(f);
}

/* IMP-Q: sched_window_active() — returns 1 if current NTP hour is inside the
 * configured charge window.  Handles midnight-wrap (e.g. 22→06).
 * Returns 1 (allow) when NTP is not synced — fail-safe, don't block charging
 * just because the clock isn't set yet.                                    */
static uint8_t sched_window_active(void)
{
    if (!g_sched_enabled) return 1u;   /* scheduling off — always allow     */
#ifdef ENABLE_NTP
    if (!NTP_IsSynced()) return 1u;    /* no time yet — fail open           */
    int h = NTP_GetHour();
    uint8_t sh = g_sched_start;
    uint8_t eh = g_sched_end;
    if (sh <= eh) {
        /* Same-day window e.g. 08:00-12:00 */
        return (h >= sh && h < eh) ? 1u : 0u;
    } else {
        /* Midnight-crossing window e.g. 22:00-06:00 */
        return (h >= sh || h < eh) ? 1u : 0u;
    }
#else
    return 1u;   /* no NTP compiled in — always allow */
#endif
}

/* ── IMP-R: TOU tariff persistence ──────────────────────────────────────
 * IMP-R: tou_load() — parse kws_rate.cfg for tier lines added by kws_tou.
 * The base rate= line is already read by rate_load(); this reads extra tier
 * lines from the same file in a second pass (avoids a separate file).
 * Format appended to kws_rate.cfg:
 *   tiers=N
 *   tier0=SH,EH,RATE
 *   tier1=SH,EH,RATE
 *   ...                                                                    */
static void tou_load(void)
{
    FILE *f = fopen(KWS_RATE_FILE, "r");
    if (!f) return;
    char line[64];
    uint8_t n = 0u;
    g_rate_tiers_n = 0u;
    while (fgets(line, sizeof(line), f) && n < KWS_RATE_TIER_MAX) {
        unsigned int sh = 0u, eh = 0u;
        float rate = 0.0f;
        /* tier lines: "tierN=SH,EH,RATE" */
        if (sscanf(line, "tier%*u=%u,%u,%f", &sh, &eh, &rate) == 3
            && sh <= 23u && eh <= 23u && rate > 0.0f) {
            g_rate_tiers[n].start_h = (uint8_t)sh;
            g_rate_tiers[n].end_h   = (uint8_t)eh;
            g_rate_tiers[n].rate_rs = rate;
            n++;
        }
    }
    fclose(f);
    g_rate_tiers_n = n;
    if (n > 0u) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: TOU loaded %u tier(s)", (unsigned)n);
    }
}

/* IMP-R: tou_save() — append tier lines to kws_rate.cfg after the base rate.
 * Rewrites the whole file so previous tier lines are replaced, not appended. */
static void tou_save(void)
{
    FILE *f = robust_fopen_w(KWS_RATE_FILE);
    if (!f) return;
    fprintf(f, "rate=%.4f\n", g_rate_rs);
    fprintf(f, "tiers=%u\n", (unsigned)g_rate_tiers_n);
    for (uint8_t i = 0u; i < g_rate_tiers_n; i++) {
        fprintf(f, "tier%u=%u,%u,%.4f\n",
                (unsigned)i,
                (unsigned)g_rate_tiers[i].start_h,
                (unsigned)g_rate_tiers[i].end_h,
                g_rate_tiers[i].rate_rs);
    }
    fclose(f);
}

/* IMP-R: rate_for_hour() — returns the applicable Rs/kWh for the given hour.
 * Iterates active TOU tiers; returns g_rate_rs if none matches or tiers=0.
 * Handles midnight-crossing windows identically to sched_window_active().  */
static float rate_for_hour(int h)
{
    uint8_t i;
    for (i = 0u; i < g_rate_tiers_n; i++) {
        uint8_t sh = g_rate_tiers[i].start_h;
        uint8_t eh = g_rate_tiers[i].end_h;
        int in_window;
        if (sh <= eh) {
            in_window = (h >= (int)sh && h < (int)eh);
        } else {
            in_window = (h >= (int)sh || h < (int)eh);
        }
        if (in_window) return g_rate_tiers[i].rate_rs;
    }
    return g_rate_rs;   /* base/fallback rate */
}

/* IMP-O+R: current_rate() — returns applicable rate using NTP hour if available.
 * Used by sess_tick() to price each second correctly under TOU.
 * Falls back to g_rate_rs if NTP is not synced or TOU is disabled.        */
static float current_rate(void)
{
    if (g_rate_tiers_n == 0u) return g_rate_rs;
#ifdef ENABLE_NTP
    if (!NTP_IsSynced()) return g_rate_rs;
    return rate_for_hour(NTP_GetHour());
#else
    return g_rate_rs;
#endif
}


 * Capped at KWS_FAULT_MAX_ROWS to prevent unbounded flash growth.
 * alarm_code: 1=OV, 2=UV, 3=OC, 4=OP (power), 5=THM (NTC)
 * uptime_s: caller provides g_uptime_s for the timestamp.               */
static void fault_log(uint8_t alarm_code, uint32_t uptime_s, float value)
{
    /* Count existing rows — same approach as history_append(). */
    {
        FILE *fc = fopen(KWS_FAULTS_FILE, "r");
        if (fc) {
            uint32_t rows = 0u;
            char tmp[96];   /* PERF-1: was [4] — fault rows are "uptime,code,value\n" ~30 chars */
            while (fgets(tmp, sizeof(tmp), fc)) {
                if (tmp[0] != '\0' && strchr(tmp, '\n')) rows++;
            }
            fclose(fc);
            if (rows >= KWS_FAULT_MAX_ROWS) {
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "KWS303WF: fault log full (%u rows) — use kws_faults_clear",
                          (unsigned)rows);
                return;
            }
        }
    }
    FILE *f = fopen(KWS_FAULTS_FILE, "a");
    if (!f) return;
    /* Columns: uptime_s, alarm_code, value (V or A or C depending on code) */
    const char *code_str = "UNKN";
    switch (alarm_code) {
        case 1: code_str = "OV";  break;
        case 2: code_str = "UV";  break;
        case 3: code_str = "OC";  break;
        case 4: code_str = "OP";  break;
        case 5: code_str = "THM"; break;
        default: break;
    }
    fprintf(f, "%u,%s,%.2f\n", (unsigned)uptime_s, code_str, value);
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: fault logged [%s] %.2f @ uptime=%u s",
              code_str, value, (unsigned)uptime_s);
}

/* ============================================================================
 * EV SESSION TYPES AND STATE — hoisted before SECTION C so that relay_close()
 * and relay_open() (FIX-UX1) can reference g_ev and g_preferred_veh without
 * a forward declaration of a variable (not legal in C99).
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
/* BUG-10 FIX: preferred vehicle for next auto-detect session.
 * Set by [-] button when idle. KWS_VEH_NONE(0) = use watt-threshold logic. */
static uint8_t   g_preferred_veh = KWS_VEH_NONE;

/* ============================================================================
 * SECTION C — RELAY
 * ============================================================================ */
static uint32_t g_uptime_s    = 0;
static uint8_t  g_relay_closed = 0;
static uint8_t  g_pulse_pin    = 0;
static uint32_t g_pulse_off_at = 0;

/* Forward declarations — sess_start/sess_end are defined in SECTION F but
 * referenced in SECTION C and G.
 * relay_open() calls sess_end() (FIX-UX1).
 * on_session() calls sess_start() and sess_end() (FIX-UX3).
 * handlers in SECTION G — forward-declare to satisfy C99 single-pass rule. */
static void sess_start(uint8_t veh);
static void sess_end(void);

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
              "KWS303WF: Relay OPEN (HAL%d→GPIO-P8 OFF-coil pulsed, load OFF)",
              KWS_RELAY_PIN_OFF);
    /* FIX-UX1: auto-end session when relay opens so timer and cost stop
     * immediately.  If already idle this is a no-op (sess_end guards active). */
    if (g_ev.active) sess_end();
    /* FIX-UX3: relay is open — nothing can be detected.  Reset detector so
     * stale AD_DETECTING/AD_CHARGING state does not carry over when the relay
     * is re-closed later.  g_adFast cleared to prevent a stale fast window.   */
    g_ad     = AD_IDLE;
    g_adFast = 0;
    g_endTk  = 0;
}

/*
 * relay_close() — v1.2.0
 * IMP-Q: off-peak schedule gate added.  If scheduling is enabled and NTP is
 * synced and we are outside the configured window, the relay close is vetoed.
 * The relay stays open; a log message is printed once (g_sched_blocked dedup).
 * The user can bypass by running kws_relay_on or disabling with kws_schedule.
 */
static void relay_close(void)
{
    /* IMP-Q: schedule gate — veto relay close outside the charge window. */
    if (!sched_window_active()) {
        if (!g_sched_blocked) {
            g_sched_blocked = 1u;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: relay close BLOCKED by schedule "
                      "(window %02u:00-%02u:00, now=%02u:xx) — "
                      "use kws_schedule disable to bypass",
                      (unsigned)g_sched_start, (unsigned)g_sched_end,
#ifdef ENABLE_NTP
                      NTP_IsSynced() ? (unsigned)NTP_GetHour() : 99u
#else
                      99u
#endif
                      );
        }
        return;
    }
    g_sched_blocked = 0u;  /* reset dedup on a successful close */
    relay_pulse_start(KWS_RELAY_PIN_ON);
    g_relay_closed = 1;
    PubCh(KWS_CH_RELAY, 100);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay CLOSE (HAL%d→GPIO-P7 ON-coil pulsed, load ON)",
              KWS_RELAY_PIN_ON);
    /* FIX-UX3: arm fast detection window instead of starting session immediately.
     *
     * Previous FIX-UX1 called sess_start() here unconditionally, which started
     * a session for ANY load (phone charger, lamp, etc.) the moment the relay
     * closed.  The correct behaviour is:
     *   • Arm AD_DETECTING with a 5 s fast-confirm window.
     *   • If load ≥ g_detect_w_min for 5 consecutive seconds → sess_start().
     *   • If load < g_detect_w_min (normal appliance) → AD_IDLE, no session.
     *
     * The 5 s window (KWS_DETECT_S_FAST) is short enough that the user sees
     * "CHG" and the timer start within 5 s of plugging in the EV, yet long
     * enough to absorb HT7017 UART first-report latency (~1–2 s).
     *
     * Only arm if auto-detect is enabled.  If auto is off, the user must press
     * [+] SESSION or use kws_session2w / kws_session4w manually.
     * If a session is already active (resumed from flash), leave it — relay_sync
     * can call relay_close() when Ch8 is 100 but we must not disturb a live
     * session on every RunEverySecond tick.                                    */
    if (g_auto_en && !g_ev.active) {
        g_ad     = AD_DETECTING;
        g_adTick = 0;
        g_adWsum = 0.0f;
        g_adFast = 1;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: auto-detect armed (%ds fast window, threshold=%.0fW)",
                  KWS_DETECT_S_FAST, g_detect_w_min);
    }
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
/* NTC ADC warmup: BK7231N ADC takes several cycles to settle after power-on.
 * The first samples read artificially low counts → Steinhart-Hart maps them
 * to artificially high temperatures (observed: 52°C → real 39°C in ~2s).
 * Discard the first NTC_ADC_WARMUP_SAMPLES before seeding the EMA.         */
#define NTC_ADC_WARMUP_SAMPLES  3u
static uint8_t g_ntc_warmup    = NTC_ADC_WARMUP_SAMPLES;

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

    /* ── STEP 2: ADC warmup discard ─────────────────────────────────────
     * Silently discard the first NTC_ADC_WARMUP_SAMPLES readings after boot.
     * The BK7231N ADC settles slowly — early samples read too low, which
     * Steinhart-Hart maps to falsely high temperatures (observed ~52°C spike
     * decaying to real value in ~2 s). Log the discarded reading so the
     * behaviour is visible in the console without triggering any alarm path. */
    if (g_ntc_warmup > 0u) {
        g_ntc_warmup--;
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC warmup discard %.1fC (%u left)",
                  t_raw, (unsigned)g_ntc_warmup);
        return;
    }

    /* ── STEP 3: EMA filter ──────────────────────────────────────────
     * First call after warmup: seed EMA with actual reading so we do not
     * start from 25°C and produce a false transient on a hot system.
     * Subsequent calls: y += (x - y) / N  (α=1/N, N=NTC_EMA_N=4).     */
    g_ntc_ema_prev = g_ntc_ema;
    if (!g_ntc_seeded) {
        g_ntc_ema   = t_raw;
        g_ntc_seeded = 1u;
    } else {
        g_ntc_ema += (t_raw - g_ntc_ema) / (float)NTC_EMA_N;
    }

    /* ── STEP 4: Rate-of-change and delta from last publish ──────────
     * dT_dt is in °C per sample interval (not °C/s).
     * At SLOW (10 s interval): NTC_FAST_TRIGGER_C=1.0 °C/sample = 6°C/min.
     * At FAST (1 s interval):  same threshold but per-second sensitivity.
     * fabsf() avoided: manual abs saves one libm call here.             */
    float dt_dt = g_ntc_ema - g_ntc_ema_prev;
    if (dt_dt < 0.0f) dt_dt = -dt_dt;

    float delta_pub = g_ntc_ema - g_ntc_last_pub;
    if (delta_pub < 0.0f) delta_pub = -delta_pub;

    /* ── STEP 5: LAYER 3 — ALARM (highest priority) ─────────────────
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
            PubCh(KWS_CH_NTC_ALARM, 1);   /* IMP-A: tell display + HA */
            fault_log(5, g_uptime_s, g_ntc_ema);  /* IMP-M: log THM fault */
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
        PubCh(KWS_CH_NTC_ALARM, 0);   /* IMP-A: clear alarm channel */
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: NTC alarm cleared %.1fC — relay stays open, manual reset required",
                  g_ntc_ema);
    }

    /* ── STEP 6: LAYER 2 — TRANSIENT DETECTION ──────────────────────
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

    /* ── STEP 7: PUBLISH GATE ────────────────────────────────────────
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
        PubCh(KWS_CH_NTC_ALARM, 0);   /* IMP-A: clear display alarm */
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
/* NOTE: KWS_VEH_* defines, EvSess_t typedef, g_ev and g_preferred_veh are
 * declared before SECTION C (see "EV SESSION TYPES" block above) because
 * relay_close() and relay_open() reference them.  Do not re-declare here.   */

/* BUG-14 FIX: s_sv was a static local inside sess_tick(); it persisted across
 * sess_end()/sess_start() cycles.  If s_sv was 59 when a session ended, the
 * 60-second periodic save fired 1 second into the new session — causing an
 * unnecessary flash write and leaving the timestamp 59 s ahead of the session
 * start.  Promoted to file scope and reset in sess_start() so the 60-second
 * save period always begins fresh with each new session.                     */

static void sess_save(void)
{
    FILE *f = robust_fopen_w(KWS_SESSION_FILE);
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
    /* BUG-7 FIX: check fscanf return count.
     * A power cut mid-write leaves a truncated file. Partial reads leave
     * zeros for missing fields — e.g. active=1 with zeroed wh_offset causes
     * a phantom session resume with wrong vehicle/energy accounting.
     * Accept 9 fields for backward compat (start_up was added later). */
    int n = fscanf(f, "active=%u\nvehicle=%u\nid=%u\nwh_off=%f\nwh_sess=%f\n"
                      "segs=%u\npeak_w=%f\npeak_a=%f\nrate=%f\n"
                      "start_up=%u\n",
                   &ua, &uv, &g_ev.session_id,
                   &g_ev.wh_offset, &g_ev.wh_session, &g_ev.seg_count,
                   &g_ev.peak_w, &g_ev.peak_a, &g_ev.rate_rs,
                   &ustart);
    fclose(f);
    if (n < 9) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: sess_load corrupt (n=%d) — discarding", n);
        memset(&g_ev, 0, sizeof(g_ev));
        return false;
    }
    /* Clamp vehicle to known values to avoid vehicle=0 (KWS_VEH_NONE) from
     * a corrupt file causing wrong stats in sess_summary(). */
    if (uv < KWS_VEH_2W || uv > KWS_VEH_4W) uv = KWS_VEH_2W;
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
    g_ev.rate_rs         = current_rate();  /* IMP-R: TOU-aware rate at session start */
    g_ev.start_uptime_s  = g_uptime_s;   /* improvement #2: record start time */
    g_ev.duration_s      = 0;
    g_sess_sv            = 0;            /* BUG-14 FIX: reset 60s save timer  */
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
static uint8_t          g_mqtt_attempts         = 0;   /* WARN-1 FIX: cap retries */
#define KWS_MQTT_MAX_ATTEMPTS  3   /* abandon after 3 WiFi-up attempts */

/*
 * sess_mqtt() — v1.1.0
 * IMP-I: added co2_saved_g — grams CO2 avoided vs grid equivalent charging.
 * IMP-K: added reboot_count for uptime diagnostics in HA.
 * Worst-case payload recalculated:
 *   Before: 236 chars.  After: +22 chars (co2_saved_g + reboot_count) = 258 chars.
 *   258 < g_mqtt_pending_pl[320] ✓
 */
/*
 * sess_mqtt() — v1.2.0
 * IMP-O: adds NTP ISO-8601 "ts" field when NTP is synced; "ts":null otherwise.
 * IMP-P: uses runtime g_veh2/4_km_pkwh / g_veh2/4_old_kmpl instead of
 *         compile-time KWS_VEH2/4_KM_PER_KWH / KWS_VEH2/4_OLD_KMPL.
 * Worst-case payload size after additions:
 *   IMP-O adds ts field: "\"ts\":\"2026-03-13T22:05:47\","  = +26 chars
 *   Previous max 258 + 26 = 284 chars < g_mqtt_pending_pl[320] ✓
 */
static void sess_mqtt(void)
{
    float kwh    = g_ev.wh_session / 1000.0f;
    float cost   = kwh * g_ev.rate_rs;
    /* IMP-P: use runtime per-vehicle factors instead of compile-time constants */
    float km_pkw = (g_ev.vehicle==KWS_VEH_2W) ? g_veh2_km_pkwh  : g_veh4_km_pkwh;
    float old_kl = (g_ev.vehicle==KWS_VEH_2W) ? g_veh2_old_kmpl : g_veh4_old_kmpl;
    float km     = kwh * km_pkw;
    float saved  = km / old_kl * KWS_PETROL_RS_PER_L - cost;
    /* IMP-I: CO2 saved = (grid kWh equivalent) × grid emission factor. */
    float petrol_litres = (km > 0.0f && old_kl > 0.0f) ? (km / old_kl) : 0.0f;
    float co2_petrol_g  = petrol_litres * 2310.0f;
    float co2_ev_g      = kwh * KWS_CO2_GRID_G_PER_KWH;
    float co2_saved_g   = co2_petrol_g - co2_ev_g;
    if (co2_saved_g < 0.0f) co2_saved_g = 0.0f;

    float lifetime_now = g_lifetime_wh + kwh * 1000.0f;

    /* IMP-O: NTP ISO-8601 timestamp — "YYYY-MM-DDTHH:MM:SS" or null. */
    char ts_buf[26];   /* "2026-03-13T22:05:47" = 19 chars + quotes + null */
#ifdef ENABLE_NTP
    if (NTP_IsSynced()) {
        snprintf(ts_buf, sizeof(ts_buf), "\"%04d-%02d-%02dT%02d:%02d:%02d\"",
                 NTP_GetYear(), NTP_GetMonth(),  NTP_GetDay(),
                 NTP_GetHour(), NTP_GetMinute(), NTP_GetSecond());
    } else {
        snprintf(ts_buf, sizeof(ts_buf), "null");
    }
#else
    snprintf(ts_buf, sizeof(ts_buf), "null");
#endif

    snprintf(g_mqtt_pending_pl, sizeof(g_mqtt_pending_pl),
             "{\"vehicle\":\"%s\",\"kwh\":%.3f,\"cost_rs\":%.2f,"
             "\"km\":%.1f,\"saved_rs\":%.2f,\"segments\":%u,"
             "\"peak_w\":%.1f,\"peak_a\":%.3f,"
             "\"duration_s\":%u,\"uptime_s\":%u,"
             "\"ntc_alarm\":%u,\"lifetime_kwh\":%.3f,"
             "\"co2_saved_g\":%u,\"reboot_count\":%u,"
             "\"ts\":%s}",
             (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
             kwh, cost, km, saved, g_ev.seg_count,
             g_ev.peak_w, g_ev.peak_a,
             (unsigned)g_ev.duration_s, (unsigned)g_uptime_s,
             (unsigned)g_ntc_alarm,
             lifetime_now / 1000.0f,
             (unsigned)co2_saved_g, (unsigned)g_reboot_count,
             ts_buf);
    g_mqtt_pending  = 1;
    g_mqtt_attempts = 0;
}

static void sess_summary(void)
{
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    /* IMP-P: use runtime per-vehicle efficiency factors */
    float km   = kwh * ((g_ev.vehicle==KWS_VEH_2W) ? g_veh2_km_pkwh  : g_veh4_km_pkwh);
    float pet  = km  / ((g_ev.vehicle==KWS_VEH_2W) ? g_veh2_old_kmpl : g_veh4_old_kmpl)
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

/* improvement #4: append one CSV row to history file
 * IMP-B: bounded at KWS_HISTORY_MAX_ROWS (200).  Before appending, count
 * existing newlines in one pass.  If at limit, log a warning and skip.
 * A single fgets loop on 200×80B = 16kB takes ~1ms — well within WDT budget.
 * The user can reclaim space with kws_history_clear if needed.            */
static void history_append(void)
{
    /* ── IMP-B: count existing rows ────────────────────────────────── */
    {
        FILE *fc = fopen(KWS_HISTORY_FILE, "r");
        if (fc) {
            uint32_t rows = 0;
            char tmp[128];                 /* PERF-1: was [4], enlarged to hold a full CSV row */
            while (fgets(tmp, sizeof(tmp), fc)) {
                /* fgets returns the partial last line too; only count
                 * lines that end in '\n' = complete rows.               */
                if (tmp[0] != '\0' && strchr(tmp, '\n')) rows++;
            }
            fclose(fc);
            if (rows >= KWS_HISTORY_MAX_ROWS) {
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "KWS303WF: history full (%u rows) — "
                          "use kws_history_clear to reset",
                          (unsigned)rows);
                return;
            }
        }
    }
    FILE *f = fopen(KWS_HISTORY_FILE, "a");
    if (!f) return;
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    /* IMP-P: use runtime per-vehicle efficiency factors */
    float km   = kwh * ((g_ev.vehicle==KWS_VEH_2W) ? g_veh2_km_pkwh : g_veh4_km_pkwh);
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

    /* IMP-J: accumulate session kWh into today's daily total. */
    g_daily_kwh += g_ev.wh_session / 1000.0f;
    if (g_daily_kwh < 0.0f) g_daily_kwh = 0.0f;
    daily_save();

    sess_summary();
    history_append();    /* improvement #4: write CSV row */
    sess_mqtt();

    FILE *f = robust_fopen_w(KWS_SESSION_FILE);
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

        /* BUG-14 FIX: use file-scope g_sess_sv (reset at sess_start) instead
         * of static local — see comment at declaration above.               */
        if (++g_sess_sv >= 60) {
            g_sess_sv = 0;
            sess_save();
            /* improvement #8: persist lifetime Wh snapshot every 60 s */
            float snap = g_lifetime_wh + g_ev.wh_session;
            FILE *lf = robust_fopen_w(KWS_LIFETIME_FILE);
            if (lf) { fprintf(lf, "wh=%.4f\n", snap); fclose(lf); }
        }
    } else {
        PubCh(KWS_CH_EVCOST, 0);
    }

    if (!g_auto_en) return;
    switch (g_ad) {
    case AD_IDLE:
        /* Background detection: only fires if relay is already closed and
         * a load appears above threshold.  relay_close() arms AD_DETECTING
         * directly, so this case mainly handles loads that ramp up slowly
         * after the relay has been closed for a while.                    */
        if (g_relay_closed && w >= g_detect_w_min) {
            g_ad = AD_DETECTING; g_adTick = 1; g_adWsum = w; g_adFast = 0;
        }
        break;
    case AD_DETECTING:
        if (w < g_detect_w_min) {
            /* FIX-UX3: load below threshold — normal appliance or initial
             * standby draw.  Return to IDLE.  g_adFast cleared so next
             * relay_close() starts a fresh fast window.                   */
            g_ad = AD_IDLE; g_adFast = 0;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: load below threshold (%.0fW < %.0fW) — no session started",
                      w, g_detect_w_min);
            break;
        }
        g_adWsum += w;
        {
            /* FIX-UX3: use fast window (5 s) when armed by relay_close(),
             * standard window (30 s) for background auto-detect from AD_IDLE. */
            uint32_t confirm_s = g_adFast ? (uint32_t)KWS_DETECT_S_FAST
                                          : (uint32_t)KWS_DETECT_S;
            if (++g_adTick >= confirm_s) {
                float avg = g_adWsum / (float)g_adTick;
                if (!g_ev.active) {
                    uint8_t veh = (g_preferred_veh != KWS_VEH_NONE)
                                  ? g_preferred_veh
                                  : ((avg < KWS_SPLIT_W) ? KWS_VEH_2W : KWS_VEH_4W);
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "KWS303WF: load confirmed (avg=%.0fW, %us window) → %s",
                              avg, (unsigned)confirm_s,
                              (veh==KWS_VEH_2W)?KWS_VEH2_NAME:KWS_VEH4_NAME);
                    sess_start(veh);
                }
                g_ad = AD_CHARGING; g_adFast = 0; g_endTk = 0;
            }
        }
        break;
    case AD_CHARGING:
        if (w < g_detect_w_min) { if (++g_endTk >= KWS_END_S) sess_end(); }
        else { g_endTk = 0; }
        break;
    }
}

/* ============================================================================
 * SECTION G — BUTTON CALLBACKS
 * ============================================================================ */
/* FIX-UX3: on_toggle now implicitly starts/ends session via relay_close/open
 * arming the detect window and relay_open calling sess_end().
static void on_toggle(void)   { if (g_relay_closed) relay_open(); else relay_close(); }
static void on_session(void)
{
    /* FIX-UX3: [+] SESSION button semantics:
     *
     * Case 1 — Relay open, no session:
     *   Close the relay.  This arms the fast detect window (5 s).  The user
     *   pressing SESSION means "I want to charge now" — relay must be closed
     *   for any current to flow.  Session starts automatically after confirm.
     *
     * Case 2 — Relay closed, session active:
     *   Restart the session timer and energy offset without interrupting power.
     *   Useful if user plugged vehicle mid-session or wants to reset the counter.
     *
     * Case 3 — Relay closed, no session (auto=off or load didn't qualify):
     *   Force-start a session immediately.  The user has explicitly decided this
     *   load should be tracked — bypass the detect threshold entirely.
     *
     * Vehicle selection: active vehicle → g_preferred_veh → default 2W.     */
    if (!g_relay_closed) {
        relay_close();   /* arms fast detect window — session starts after 5 s */
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: [+] relay closed — detect window armed");
        return;
    }
    /* Relay is closed: restart or force-start */
    uint8_t veh = g_ev.active  ? g_ev.vehicle
                : (g_preferred_veh != KWS_VEH_NONE) ? g_preferred_veh
                : KWS_VEH_2W;
    if (g_ev.active) sess_end();
    sess_start(veh);
    /* Sync auto-detect state so sess_tick's AD_CHARGING path monitors end-of-charge */
    g_ad = AD_CHARGING; g_adFast = 0; g_endTk = 0;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: [+] session force-started (veh=%s)",
              (veh==KWS_VEH_2W)?KWS_VEH2_NAME:KWS_VEH4_NAME);
}
/* improvement #3 — [-] (P20) cycles vehicle profile.
 * Active session: ends current session and immediately restarts with the
 *   other vehicle type — history row is written for the partial session.
 * Idle: toggles g_preferred_veh so the next auto-detect session uses it.
 *       BUG-10 FIX: was a dead local static — now file-scope so auto-detect
 *       and on_session() can actually read it.                              */
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
        g_preferred_veh = (g_preferred_veh == KWS_VEH_2W) ? KWS_VEH_4W : KWS_VEH_2W;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "KWS303WF: [-] next session vehicle = %s",
                  (g_preferred_veh==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
    }
}

/* ============================================================================
 * SECTION H — CONSOLE COMMANDS
 * ============================================================================ */
static commandResult_t CMD_RelayOn (const void*x,const char*c,const char*a,int f){relay_close();return CMD_RES_OK;}
static commandResult_t CMD_RelayOff(const void*x,const char*c,const char*a,int f){relay_open(); return CMD_RES_OK;}

static commandResult_t CMD_Ch8Set(const void*x,const char*c,const char*a,int f)
{ if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS; relay_sync(atoi(a)); return CMD_RES_OK; }

/*
 * CMD_Rate() — v1.1.0
 * IMP-G: now calls rate_save() after setting so the tariff persists across reboots.
 */
static commandResult_t CMD_Rate(const void*x,const char*c,const char*a,int f)
{
    if(!a||!*a){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate: %.4f Rs/kWh",g_rate_rs);return CMD_RES_OK;}
    float v = (float)atof(a);
    if (!(v > 0.0f)) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate: value must be > 0");
        return CMD_RES_BAD_ARGUMENT;
    }
    g_rate_rs = v;
    rate_save();   /* IMP-G: persist to kws_rate.cfg */
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate set %.4f Rs/kWh (saved)", g_rate_rs);
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

/* improvement #4: dump history CSV over log
 * IMP-F: capped at KWS_HISTORY_DUMP_ROWS recent rows.  On a large file,
 * unlimited addLogAdv() calls can stall the UART FIFO long enough to trigger
 * the ~5 s WDT.  50 rows × ~80 chars each ≈ 4 kB output — safe.          */
#define KWS_HISTORY_DUMP_ROWS  50u
static commandResult_t CMD_HistoryDump(const void*x,const char*c,const char*a,int f)
{
    FILE *fh = fopen(KWS_HISTORY_FILE, "r");
    if (!fh) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_history: no history file yet");
        return CMD_RES_OK;
    }

    /* Count total rows first so we can skip to the last N. */
    uint32_t total = 0;
    char line[128];
    while (fgets(line, sizeof(line), fh)) {
        size_t l = strlen(line);
        if (l > 0 && line[l-1] == '\n') total++;
    }
    rewind(fh);

    uint32_t skip = (total > KWS_HISTORY_DUMP_ROWS)
                    ? (total - KWS_HISTORY_DUMP_ROWS) : 0u;

    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "kws_history: %u rows total, showing last %u",
              (unsigned)total,
              (unsigned)(total > KWS_HISTORY_DUMP_ROWS ? KWS_HISTORY_DUMP_ROWS : total));
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "id,veh,kwh,cost_rs,km,duration_s,segs,peak_w,peak_a");

    uint32_t row = 0;
    while (fgets(line, sizeof(line), fh)) {
        size_t l = strlen(line);
        if (l > 0 && line[l-1] == '\n') {
            if (row++ < skip) continue;
            line[l-1] = '\0';
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"%s", line);
        }
    }
    fclose(fh);
    return CMD_RES_OK;
}

/* improvement #4: clear history file */
static commandResult_t CMD_HistoryClear(const void*x,const char*c,const char*a,int f)
{
    FILE *fh = robust_fopen_w(KWS_HISTORY_FILE);
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
    /* IMP-D: show true running lifetime total (saved + active session) so the
     * value is current even mid-charge, not stale from the last sess_end().  */
    float lifetime_now = g_lifetime_wh + (g_ev.active ? g_ev.wh_session : 0.0f);
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
              "  relay=%s  auto=%s  detect_w=%.0fW  ad_state=%s%s  uptime=%us  ntc_alarm=%s",
              g_relay_closed?"CLOSED":"OPEN",
              g_auto_en?"ON":"OFF",
              g_detect_w_min,
              g_ad==AD_IDLE?"IDLE":g_ad==AD_DETECTING?"DETECTING":"CHARGING",
              g_adFast?" (fast)":"",
              (unsigned)g_uptime_s,
              g_ntc_alarm?"LATCHED":"clear");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  lifetime_saved=%.2f Wh  lifetime_now=%.2f Wh  (%.4f kWh total)",
              g_lifetime_wh, lifetime_now, lifetime_now/1000.0f);
    return CMD_RES_OK;
}

static commandResult_t CMD_SessAuto(const void*x,const char*c,const char*a,int f)
{
    g_auto_en = !g_auto_en;
    if (!g_auto_en && g_ad != AD_CHARGING) {
        /* FIX-UX3: turning auto off while detect window is pending — reset
         * to IDLE.  If already AD_CHARGING a session is running; leave it.  */
        g_ad = AD_IDLE; g_adFast = 0;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: auto-detect %s", g_auto_en ? "ON" : "OFF");
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

/* Device short-name set via kws_ha_devname command.
 * Must match the OBK MQTT client ID / Short Name (shown in web UI header,
 * e.g. "obkAB12CD").  Placed here — before KWS303WF_Init — so the compiler
 * sees the definition before CMD_RegisterCommand references it. */
static char g_ha_devname[40] = "";

static commandResult_t CMD_HaDevname(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: kws_ha_devname = \"%s\"", g_ha_devname);
        return CMD_RES_OK;
    }
    snprintf(g_ha_devname, sizeof(g_ha_devname), "%s", args);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: HA devname set to \"%s\"", g_ha_devname);
    return CMD_RES_OK;
}

/* ── IMP-H: kws_protect — get/set and persist OV/UV/OC thresholds ────────
 * Usage: kws_protect              → print current thresholds
 *        kws_protect ov <V>       → set over-voltage trip
 *        kws_protect uv <V>       → set under-voltage trip
 *        kws_protect oc <A>       → set over-current trip
 * All changes are saved to kws_protection.cfg immediately.
 * NOTE: Thresholds are logged here and written to file.  Applying them to
 * the HT7017 registers requires a restart (HT7017_Init re-reads them via
 * the protect_load() → g_ov/uv/oc_thr → passed to HT7017 at Init).      */
static commandResult_t CMD_Protect(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_protect: OV=%.1fV UV=%.1fV OC=%.1fA",
                  g_ov_thr, g_uv_thr, g_oc_thr);
        return CMD_RES_OK;
    }
    /* Parse "ov <val>", "uv <val>", "oc <val>" */
    char key[4] = {0};
    float val = 0.0f;
    if (sscanf(args, "%3s %f", key, &val) != 2 || !(val > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_protect usage: kws_protect ov|uv|oc <value>");
        return CMD_RES_BAD_ARGUMENT;
    }
    if      (strcmp(key,"ov")==0) { g_ov_thr = val; }
    else if (strcmp(key,"uv")==0) { g_uv_thr = val; }
    else if (strcmp(key,"oc")==0) { g_oc_thr = val; }
    else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_protect: unknown key '%s' (use ov/uv/oc)", key);
        return CMD_RES_BAD_ARGUMENT;
    }
    protect_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "kws_protect: %s set to %.2f (saved). Restart to apply to HT7017.",
              key, val);
    return CMD_RES_OK;
}

/* ── IMP-J: kws_daily_reset — manually reset the daily kWh accumulator ───
 * Useful when integrating with NTP-based midnight CALENDAR_EVENTs.        */
static commandResult_t CMD_DailyReset(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: daily reset — was %.3f kWh", g_daily_kwh);
    g_daily_kwh          = 0.0f;
    g_daily_start_uptime = g_uptime_s;
    g_daily_mqtt_sent    = 0u;
    daily_save();
    return CMD_RES_OK;
}

/* ── IMP-L: kws_heartbeat — get/set heartbeat interval ───────────────────
 * Usage: kws_heartbeat            → print current interval
 *        kws_heartbeat <seconds>  → set new interval (0 = disable)       */
static commandResult_t CMD_Heartbeat(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_heartbeat: %u s (topic: %s)",
                  (unsigned)KWS_HEARTBEAT_S, KWS_HEARTBEAT_TOPIC);
        return CMD_RES_OK;
    }
    /* Note: interval is compile-time constant KWS_HEARTBEAT_S — runtime
     * change resets the counter so next publish fires in <new_interval>. */
    g_hb_tick = 0u;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: heartbeat counter reset (compile-time interval=%u s)",
              (unsigned)KWS_HEARTBEAT_S);
    return CMD_RES_OK;
}

/* ── IMP-M: kws_faults_dump — print fault log to UART console ────────────
 * Reads kws_faults.log and prints each row, capped at KWS_FAULT_MAX_ROWS.
 * Format per row: uptime_s, code, value.                                  */
static commandResult_t CMD_FaultsDump(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    FILE *f = fopen(KWS_FAULTS_FILE, "r");
    if (!f) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_faults_dump: no fault log found");
        return CMD_RES_OK;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "=== Fault Log ===");
    char buf[96];
    uint32_t rows = 0u;
    while (fgets(buf, sizeof(buf), f) && rows < KWS_FAULT_MAX_ROWS) {
        /* Strip trailing newline for clean log output */
        size_t len = strlen(buf);
        if (len > 0 && buf[len-1] == '\n') buf[len-1] = '\0';
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  %s", buf);
        rows++;
    }
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "=== %u fault(s) total ===", (unsigned)rows);
    return CMD_RES_OK;
}

/* IMP-M: kws_faults_clear — truncate fault log (zero it out). */
static commandResult_t CMD_FaultsClear(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    FILE *f = robust_fopen_w(KWS_FAULTS_FILE);
    if (f) { fclose(f); }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: fault log cleared");
    return CMD_RES_OK;
}


/* ── IMP-P: kws_veh_config — get/set per-vehicle runtime efficiency ───────
 * Usage: kws_veh_config                           → print current values
 *        kws_veh_config 2w <km/kWh> <oldkmpl>    → set 2-wheeler factors
 *        kws_veh_config 4w <km/kWh> <oldkmpl>    → set 4-wheeler factors
 * All changes are persisted to kws_vehicles.cfg immediately.
 * Example: kws_veh_config 2w 28.5 38.0
 *          → Ather 450X, 28.5 km/kWh; old Activa at 38 km/L petrol        */
static commandResult_t CMD_VehConfig(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config: 2W=%.2fkm/kWh(old=%.1fkmpl) "
                  "4W=%.2fkm/kWh(old=%.1fkmpl)",
                  g_veh2_km_pkwh, g_veh2_old_kmpl,
                  g_veh4_km_pkwh, g_veh4_old_kmpl);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "  compile defaults: 2W=%.1f 4W=%.1f",
                  (float)KWS_VEH2_KM_PER_KWH, (float)KWS_VEH4_KM_PER_KWH);
        return CMD_RES_OK;
    }
    char key[4] = {0};
    float km_pkwh = 0.0f, old_kmpl = 0.0f;
    if (sscanf(args, "%3s %f %f", key, &km_pkwh, &old_kmpl) != 3
        || !(km_pkwh > 0.0f) || !(old_kmpl > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config usage: kws_veh_config 2w|4w <km/kWh> <oldkmpl>");
        return CMD_RES_BAD_ARGUMENT;
    }
    if (strcmp(key, "2w") == 0 || strcmp(key, "2W") == 0) {
        g_veh2_km_pkwh  = km_pkwh;
        g_veh2_old_kmpl = old_kmpl;
    } else if (strcmp(key, "4w") == 0 || strcmp(key, "4W") == 0) {
        g_veh4_km_pkwh  = km_pkwh;
        g_veh4_old_kmpl = old_kmpl;
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config: unknown type '%s' (use 2w or 4w)", key);
        return CMD_RES_BAD_ARGUMENT;
    }
    vehicles_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "kws_veh_config: %s set to %.2fkm/kWh old=%.1fkmpl (saved)",
              key, km_pkwh, old_kmpl);
    return CMD_RES_OK;
}

/* ── IMP-Q: kws_schedule — get/set/enable/disable off-peak window ─────────
 * Usage: kws_schedule                     → print current state
 *        kws_schedule enable              → activate scheduling
 *        kws_schedule disable             → deactivate (always allow)
 *        kws_schedule <start_h> <end_h>   → set window hours (enables too)
 * Hours are integers 0-23 (24h clock).  Midnight-crossing supported.
 * Example: kws_schedule 22 6   → charge window 22:00 PM to 06:00 AM
 * Note: requires NTP to be running (startDriver NTP in autoexec.bat).     */
static commandResult_t CMD_Schedule(const void *ctx, const char *cmd,
                                    const char *args, int flags)
{
    if (!args || !*args) {
#ifdef ENABLE_NTP
        int synced = NTP_IsSynced();
        int h = synced ? NTP_GetHour() : -1;
        uint8_t active = sched_window_active();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: enabled=%u window=%02u:00-%02u:00 "
                  "ntp=%s current_h=%d in_window=%u",
                  (unsigned)g_sched_enabled,
                  (unsigned)g_sched_start, (unsigned)g_sched_end,
                  synced ? "synced" : "NOT_SYNCED", h,
                  (unsigned)active);
#else
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: enabled=%u window=%02u:00-%02u:00 (no NTP)",
                  (unsigned)g_sched_enabled,
                  (unsigned)g_sched_start, (unsigned)g_sched_end);
#endif
        return CMD_RES_OK;
    }
    if (strcmp(args, "enable") == 0) {
        g_sched_enabled = 1u;
        g_sched_blocked = 0u;
        schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: schedule ENABLED window=%02u:00-%02u:00",
                  (unsigned)g_sched_start, (unsigned)g_sched_end);
        return CMD_RES_OK;
    }
    if (strcmp(args, "disable") == 0) {
        g_sched_enabled = 0u;
        g_sched_blocked = 0u;
        schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: schedule DISABLED");
        return CMD_RES_OK;
    }
    /* Parse "START END" hour pair */
    unsigned int sh = 0u, eh = 0u;
    if (sscanf(args, "%u %u", &sh, &eh) != 2 || sh > 23u || eh > 23u) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule usage: kws_schedule enable|disable|<start_h 0-23> <end_h 0-23>");
        return CMD_RES_BAD_ARGUMENT;
    }
    g_sched_start   = (uint8_t)sh;
    g_sched_end     = (uint8_t)eh;
    g_sched_enabled = 1u;   /* setting a window implicitly enables scheduling */
    g_sched_blocked = 0u;
    schedule_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: schedule window set %02u:00-%02u:00 (enabled, saved)",
              (unsigned)g_sched_start, (unsigned)g_sched_end);
    return CMD_RES_OK;
}

/* ── IMP-R: kws_tou — get/set/clear time-of-use tariff tiers ─────────────
 * Usage: kws_tou                              → print current tiers
 *        kws_tou add <start_h> <end_h> <rate> → add a tier
 *        kws_tou clear                        → remove all tiers (use base rate)
 * Hours 0-23.  Rate in Rs/kWh.  Max KWS_RATE_TIER_MAX (3) tiers.
 * Changes persist immediately to kws_rate.cfg (appended after base rate).
 * Example: kws_tou add 22 6 5.50    → off-peak tier 10PM-6AM at Rs 5.50
 *          kws_tou add 9 17 9.50    → peak tier 9AM-5PM at Rs 9.50
 * Note: first matching tier wins — order tiers from most-specific to broadest.
 * Base rate (kws_rate <val>) remains as fallback when no tier matches.    */
static commandResult_t CMD_Tou(const void *ctx, const char *cmd,
                               const char *args, int flags)
{
    if (!args || !*args) {
        if (g_rate_tiers_n == 0u) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "kws_tou: TOU disabled — single rate Rs%.4f/kWh", g_rate_rs);
        } else {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "kws_tou: %u tier(s), base=Rs%.4f/kWh",
                      (unsigned)g_rate_tiers_n, g_rate_rs);
            for (uint8_t i = 0u; i < g_rate_tiers_n; i++) {
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "  tier%u: %02u:00-%02u:00  Rs%.4f/kWh",
                          (unsigned)i,
                          (unsigned)g_rate_tiers[i].start_h,
                          (unsigned)g_rate_tiers[i].end_h,
                          g_rate_tiers[i].rate_rs);
            }
#ifdef ENABLE_NTP
            if (NTP_IsSynced()) {
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "  current hour=%02d → active rate=Rs%.4f/kWh",
                          NTP_GetHour(), current_rate());
            }
#endif
        }
        return CMD_RES_OK;
    }
    if (strcmp(args, "clear") == 0) {
        g_rate_tiers_n = 0u;
        tou_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: TOU cleared — single rate Rs%.4f/kWh active", g_rate_rs);
        return CMD_RES_OK;
    }
    /* Parse "add SH EH RATE" */
    unsigned int sh = 0u, eh = 0u;
    float rate = 0.0f;
    char sub[8] = {0};
    if (sscanf(args, "%7s %u %u %f", sub, &sh, &eh, &rate) != 4
        || strcmp(sub, "add") != 0
        || sh > 23u || eh > 23u || !(rate > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou usage: kws_tou [add <start_h> <end_h> <rate>|clear]");
        return CMD_RES_BAD_ARGUMENT;
    }
    if (g_rate_tiers_n >= KWS_RATE_TIER_MAX) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou: max %u tiers reached — use kws_tou clear first",
                  (unsigned)KWS_RATE_TIER_MAX);
        return CMD_RES_BAD_ARGUMENT;
    }
    uint8_t i = g_rate_tiers_n;
    g_rate_tiers[i].start_h = (uint8_t)sh;
    g_rate_tiers[i].end_h   = (uint8_t)eh;
    g_rate_tiers[i].rate_rs = rate;
    g_rate_tiers_n++;
    tou_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: TOU tier%u added %02u:00-%02u:00 Rs%.4f/kWh (saved, %u tier(s) total)",
              (unsigned)i, (unsigned)sh, (unsigned)eh, rate,
              (unsigned)g_rate_tiers_n);
    return CMD_RES_OK;
}

/*
 * KWS303WF_Init() — v1.2.0
 * IMP-G: rate_load() restores persisted tariff.
 * IMP-H: protect_load() restores OV/UV/OC thresholds.
 * IMP-J: daily_load() restores today's accumulated kWh.
 * IMP-K: increments g_reboot_count and persists immediately after lifetime_load().
    /* IMP-L: g_hb_tick initialised to 0 — first heartbeat fires after KWS_HEARTBEAT_S s. */
 * IMP-P: vehicles_load() restores per-vehicle efficiency factors.
 * IMP-Q: schedule_load() restores off-peak window config.
 * IMP-R: tou_load() restores TOU tariff tiers from kws_rate.cfg.
 */

/* ── IMP-P: kws_veh_config — get/set runtime vehicle efficiency factors ──
 * Usage: kws_veh_config                       → print current values
 *        kws_veh_config 2w <km/kWh> [km/L]   → set 2-wheeler factors
 *        kws_veh_config 4w <km/kWh> [km/L]   → set 4-wheeler factors
 * km/L (old petrol efficiency) is optional; omit to keep current value.   */
static commandResult_t CMD_VehConfig(const void *ctx, const char *cmd,
                                     const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config:"
                  "  2W: %.2f km/kWh  %.2f km/L(petrol)"
                  "  4W: %.2f km/kWh  %.2f km/L(petrol)",
                  g_veh2_km_pkwh, g_veh2_old_kmpl,
                  g_veh4_km_pkwh, g_veh4_old_kmpl);
        return CMD_RES_OK;
    }
    char  type[4] = {0};
    float km_pkwh = 0.0f, km_pl = 0.0f;
    int n = sscanf(args, "%3s %f %f", type, &km_pkwh, &km_pl);
    if (n < 2 || !(km_pkwh > 0.0f)) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config usage: kws_veh_config 2w|4w <km/kWh> [km/L]");
        return CMD_RES_BAD_ARGUMENT;
    }
    /* km/L is optional — only update if provided (n==3) and positive. */
    int is2w = (strcmp(type,"2w")==0 || strcmp(type,"2W")==0);
    int is4w = (strcmp(type,"4w")==0 || strcmp(type,"4W")==0);
    if (!is2w && !is4w) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_veh_config: type must be 2w or 4w");
        return CMD_RES_BAD_ARGUMENT;
    }
    if (is2w) {
        g_veh2_km_pkwh = km_pkwh;
        if (n == 3 && km_pl > 0.0f) g_veh2_old_kmpl = km_pl;
    } else {
        g_veh4_km_pkwh = km_pkwh;
        if (n == 3 && km_pl > 0.0f) g_veh4_old_kmpl = km_pl;
    }
    vehicles_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "kws_veh_config: %s set %.2f km/kWh %.2f km/L (saved)",
              is2w ? "2W" : "4W",
              is2w ? g_veh2_km_pkwh : g_veh4_km_pkwh,
              is2w ? g_veh2_old_kmpl : g_veh4_old_kmpl);
    return CMD_RES_OK;
}

/* ── IMP-Q: kws_schedule — get/set/enable/disable off-peak schedule ──────
 * Usage: kws_schedule                     → print current config
 *        kws_schedule enable              → activate scheduling
 *        kws_schedule disable             → deactivate (relay always allowed)
 *        kws_schedule start <hour>        → set window open hour (0-23)
 *        kws_schedule end   <hour>        → set window close hour (0-23)
 *        kws_schedule window <SH> <EH>   → set both hours at once
 * All changes are persisted to kws_schedule.cfg immediately.              */
static commandResult_t CMD_Schedule(const void *ctx, const char *cmd,
                                    const char *args, int flags)
{
    if (!args || !*args) {
        /* Print state + NTP-aware window status */
        char ntp_status[24] = "NTP unknown";
#ifdef ENABLE_NTP
        if (NTP_IsSynced())
            snprintf(ntp_status, sizeof(ntp_status), "now=%02d:00", NTP_GetHour());
        else
            snprintf(ntp_status, sizeof(ntp_status), "NTP not synced");
#endif
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: enabled=%u window=%02u:00-%02u:00 "
                  "active=%u %s",
                  (unsigned)g_sched_enabled,
                  (unsigned)g_sched_start, (unsigned)g_sched_end,
                  (unsigned)sched_window_active(), ntp_status);
        return CMD_RES_OK;
    }

    char sub[16] = {0};
    unsigned int v1 = 0u, v2 = 0u;
    sscanf(args, "%15s %u %u", sub, &v1, &v2);

    if (strcmp(sub,"enable")==0) {
        g_sched_enabled = 1u; g_sched_blocked = 0u; schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: enabled (window %02u:00-%02u:00)",
                  (unsigned)g_sched_start, (unsigned)g_sched_end);
    } else if (strcmp(sub,"disable")==0) {
        g_sched_enabled = 0u; g_sched_blocked = 0u; schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "kws_schedule: disabled");
    } else if (strcmp(sub,"start")==0) {
        if (v1 > 23u) return CMD_RES_BAD_ARGUMENT;
        g_sched_start = (uint8_t)v1; schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: window start set to %02u:00", v1);
    } else if (strcmp(sub,"end")==0) {
        if (v1 > 23u) return CMD_RES_BAD_ARGUMENT;
        g_sched_end = (uint8_t)v1; schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: window end set to %02u:00", v1);
    } else if (strcmp(sub,"window")==0) {
        if (v1 > 23u || v2 > 23u) return CMD_RES_BAD_ARGUMENT;
        g_sched_start = (uint8_t)v1;
        g_sched_end   = (uint8_t)v2;
        schedule_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: window set %02u:00-%02u:00", v1, v2);
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_schedule: unknown sub-command '%s'", sub);
        return CMD_RES_BAD_ARGUMENT;
    }
    return CMD_RES_OK;
}

/* ── IMP-R: kws_tou — get/set time-of-use tariff tiers ──────────────────
 * Usage: kws_tou                            → print all tiers
 *        kws_tou add <SH> <EH> <Rs/kWh>    → add a tier (max KWS_RATE_TIER_MAX)
 *        kws_tou clear                      → remove all tiers (revert to flat)
 * Example: kws_tou add 22 6 5.00    (off-peak 22:00-06:00 @ Rs5/unit)
 *          kws_tou add  9 17 9.50   (peak    09:00-17:00 @ Rs9.50/unit)
 * Hour ranges wrap midnight: if start_h > end_h the window crosses midnight.
 * If no tier matches current hour, falls back to base kws_rate.           */
static commandResult_t CMD_Tou(const void *ctx, const char *cmd,
                               const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou: base=Rs%.4f/kWh tiers=%u (max %u)",
                  g_rate_rs, (unsigned)g_rate_tiers_n, (unsigned)KWS_RATE_TIER_MAX);
        for (uint8_t i = 0u; i < g_rate_tiers_n; i++) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "  tier%u: %02u:00-%02u:00 @ Rs%.4f/kWh",
                      (unsigned)i,
                      (unsigned)g_rate_tiers[i].start_h,
                      (unsigned)g_rate_tiers[i].end_h,
                      g_rate_tiers[i].rate_rs);
        }
        /* Show currently active rate if NTP is synced. */
#ifdef ENABLE_NTP
        if (NTP_IsSynced()) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "  active now (hour=%02d): Rs%.4f/kWh",
                      NTP_GetHour(), current_rate());
        }
#endif
        return CMD_RES_OK;
    }

    char sub[8] = {0};
    unsigned int sh = 0u, eh = 0u;
    float rate = 0.0f;
    sscanf(args, "%7s", sub);

    if (strcmp(sub,"clear")==0) {
        g_rate_tiers_n = 0u;
        tou_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou: all tiers cleared — flat rate Rs%.4f active",
                  g_rate_rs);
    } else if (strcmp(sub,"add")==0) {
        if (g_rate_tiers_n >= KWS_RATE_TIER_MAX) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "kws_tou: max %u tiers reached — clear first",
                      (unsigned)KWS_RATE_TIER_MAX);
            return CMD_RES_BAD_ARGUMENT;
        }
        /* Parse: "add SH EH RATE" — skip the sub-command word */
        const char *rest = args + 4u;  /* skip "add " */
        while (*rest == ' ') rest++;
        if (sscanf(rest, "%u %u %f", &sh, &eh, &rate) != 3
            || sh > 23u || eh > 23u || !(rate > 0.0f)) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "kws_tou add usage: kws_tou add <start_h> <end_h> <Rs/kWh>");
            return CMD_RES_BAD_ARGUMENT;
        }
        uint8_t idx = g_rate_tiers_n;
        g_rate_tiers[idx].start_h = (uint8_t)sh;
        g_rate_tiers[idx].end_h   = (uint8_t)eh;
        g_rate_tiers[idx].rate_rs = rate;
        g_rate_tiers_n++;
        tou_save();
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou: tier%u added %02u:00-%02u:00 @ Rs%.4f/kWh (saved)",
                  (unsigned)idx, sh, eh, rate);
    } else {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_tou: unknown sub-command '%s'", sub);
        return CMD_RES_BAD_ARGUMENT;
    }
    return CMD_RES_OK;
}

void KWS303WF_Init(void)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: init v%s %s",
              KWS_FW_VERSION_STR, KWS_FW_BUILD_DATE);

    HAL_PIN_Setup_Output(KWS_RELAY_PIN_ON);
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_OFF);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_ON,  0);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_OFF, 0);
    relay_open();

    lifetime_load();   /* improvement #8: restore odometer + IMP-K: reboot_count */

    /* IMP-K: increment reboot counter and persist immediately. */
    g_reboot_count++;
    lifetime_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: reboot #%u (lifetime %.2f Wh)",
              (unsigned)g_reboot_count, g_lifetime_wh);

    /* IMP-G: restore persisted electricity rate. */
    rate_load();

    /* IMP-H: restore persisted protection thresholds. */
    protect_load();

    /* IMP-J: restore today's daily accumulator. */
    daily_load();
    g_daily_mqtt_sent    = 0u;   /* never sent this boot */

    /* IMP-P: restore per-vehicle efficiency factors. */
    vehicles_load();

    /* IMP-Q: restore off-peak scheduling config. */
    schedule_load();

    /* IMP-R: restore TOU tariff tiers (reads extra lines from kws_rate.cfg). */
    tou_load();

    /* IMP-L: heartbeat counter starts at 0 — first publish fires after
     * KWS_HEARTBEAT_S seconds; no publish on cold boot to avoid flood. */
    g_hb_tick = 0u;


    /* NTC init — seed on first ntc_tick() call */
    g_ntc_seeded    = 0u;
    g_ntc_alarm     = 0u;
    g_ntc_in_fast   = 0u;
    g_ntc_fast_held = 0u;
    g_ntc_countdown = 1u;      /* take first sample on tick 1 */
    g_ntc_last_pub  = -999.0f; /* force publish on first valid reading */
    g_ntc_ema       = 25.0f;   /* nominal; overwritten on first real sample */
    g_ntc_ema_prev  = 25.0f;
    g_ntc_warmup    = NTC_ADC_WARMUP_SAMPLES; /* discard first N ADC reads  */
    PubCh(KWS_CH_NTC_ALARM, 0); /* IMP-A: initialise alarm channel = normal */

    btn_register(KWS_BTN_TOGGLE,   on_toggle);
    btn_register(KWS_BTN_SESSION,  on_session);
    btn_register(KWS_BTN_RESERVED, on_reserved);

    if (sess_load() && g_ev.active) {
        g_ev.seg_count++;
        g_ev.wh_offset = 0.0f;
        g_ev.wh_resume = 1;
        g_ad   = AD_CHARGING;   /* resumed session already confirmed */
        g_adFast = 0;
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
    CMD_RegisterCommand("kws_rate",             CMD_Rate,         NULL);   /* IMP-G: now persists */
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
    CMD_RegisterCommand("kws_ha_devname",       CMD_HaDevname,    NULL);
    /* IMP-H: protection threshold persistence */
    CMD_RegisterCommand("kws_protect",          CMD_Protect,      NULL);
    /* IMP-J: daily reset (also triggerable via autoexec CALENDAR_EVENT) */
    CMD_RegisterCommand("kws_daily_reset",      CMD_DailyReset,   NULL);
    /* IMP-L: heartbeat control */
    CMD_RegisterCommand("kws_heartbeat",        CMD_Heartbeat,    NULL);
    /* IMP-M: fault log inspection */
    CMD_RegisterCommand("kws_faults_dump",      CMD_FaultsDump,   NULL);
    CMD_RegisterCommand("kws_faults_clear",     CMD_FaultsClear,  NULL);
    /* IMP-P: per-vehicle runtime efficiency factors */
    CMD_RegisterCommand("kws_veh_config",       CMD_VehConfig,    NULL);
    /* IMP-Q: off-peak scheduling */
    CMD_RegisterCommand("kws_schedule",         CMD_Schedule,     NULL);
    /* IMP-R: time-of-use tariff tiers */
    CMD_RegisterCommand("kws_tou",              CMD_Tou,          NULL);

    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS303WF: ready fw=%s %s relay=P%d/P%d ntc=ADC%d slow=%us fast=%us rate=Rs%.2f/kWh",
              KWS_FW_VERSION_STR, KWS_FW_BUILD_DATE,
              KWS_RELAY_PIN_ON, KWS_RELAY_PIN_OFF, KWS_ADC_CH,
              (unsigned)NTC_SLOW_PERIOD, (unsigned)NTC_FAST_PERIOD, g_rate_rs);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  NTC warn=%.0fC alarm=%.0fC deadband=%.1fC trigger=%.1fC/samp ema_n=%u",
              (float)NTC_WARN_C,(float)NTC_ALARM_C,
              (float)NTC_DEADBAND_C,(float)NTC_FAST_TRIGGER_C,(unsigned)NTC_EMA_N);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  2W:%s %.1fkm/kWh(%.1fkmpl)  4W:%s %.1fkm/kWh(%.1fkmpl)  petrol=Rs%.0f/L",
              KWS_VEH2_NAME, g_veh2_km_pkwh, g_veh2_old_kmpl,
              KWS_VEH4_NAME, g_veh4_km_pkwh, g_veh4_old_kmpl, KWS_PETROL_RS_PER_L);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  IMP-G rate_persist=yes  IMP-H protect=OV%.0fV/UV%.0fV/OC%.0fA",
              g_ov_thr, g_uv_thr, g_oc_thr);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  IMP-I co2_factor=%.0fg/kWh  IMP-J daily=%.3fkWh  IMP-K reboots=%u",
              (float)KWS_CO2_GRID_G_PER_KWH, g_daily_kwh, (unsigned)g_reboot_count);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  IMP-L heartbeat=%us  IMP-M fault_log_cap=%u",
              (unsigned)KWS_HEARTBEAT_S, (unsigned)KWS_FAULT_MAX_ROWS);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  IMP-O NTP_ts=%s  IMP-P 2W=%.1fkm/kWh 4W=%.1fkm/kWh",
#ifdef ENABLE_NTP
              NTP_IsSynced() ? "synced" : "not_synced",
#else
              "no_ntp",
#endif
              g_veh2_km_pkwh, g_veh4_km_pkwh);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  IMP-Q sched=%s window=%02u:00-%02u:00  IMP-R tou_tiers=%u base=Rs%.2f",
              g_sched_enabled ? "ON" : "off",
              (unsigned)g_sched_start, (unsigned)g_sched_end,
              (unsigned)g_rate_tiers_n, g_rate_rs);
}

/*
 * KWS303WF_RunEverySecond() — v1.2.0
 * IMP-J: daily_tick() — detects 24 h boundary, publishes daily MQTT summary, resets accumulator.
 * IMP-L: heartbeat_tick() — publishes compact status MQTT every KWS_HEARTBEAT_S seconds.
 * IMP-M: ht7017_alarm_tick() — logs HT7017 electrical alarms (OV/UV/OC/OP) to kws_faults.log.
 * IMP-O: NTP timestamp is evaluated per-session in sess_mqtt() — no RunEverySecond work needed.
 * IMP-Q: sched_window_active() is evaluated at relay_close() call time — no tick needed.
 * IMP-R: current_rate() is evaluated at sess_start() — no tick needed.
 */

/* ── IMP-J: daily kWh tick ────────────────────────────────────────────────
 * Called once per second from RunEverySecond().
 * Midnight detection: when uptime has advanced by ≥ 86400 s from
 * g_daily_start_uptime, we treat that as a new day.  This is not wall-clock
 * midnight but it is robust across reboots and does not require NTP.
 * If NTP is available the user can call kws_daily_reset via autoexec at
 * midnight using OBK's CALENDAR_EVENT feature (not done here — stays simple).*/
static void daily_tick(void)
{
    /* Check if 24 h have elapsed since the last day-start. */
    if ((g_uptime_s - g_daily_start_uptime) < 86400u) return;

    /* Midnight boundary crossed — publish daily summary, then reset. */
    float kwh = g_daily_kwh;

    if (!g_daily_mqtt_sent && ReadCh(KWS_CH_WIFI) != 0) {
        char pl[128];
        snprintf(pl, sizeof(pl),
                 "{\"daily_kwh\":%.3f,\"lifetime_kwh\":%.3f,\"uptime_s\":%u}",
                 kwh, g_lifetime_wh / 1000.0f, (unsigned)g_uptime_s);
        char cmd[180];
        snprintf(cmd, sizeof(cmd), "publishMQTT %s %s", KWS_DAILY_TOPIC, pl);
        CMD_ExecuteCommand(cmd, 0);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: daily MQTT sent %.3f kWh", kwh);
        g_daily_mqtt_sent = 1u;
    }

    /* Reset for the new day. */
    g_daily_kwh          = 0.0f;
    g_daily_start_uptime = g_uptime_s;
    g_daily_mqtt_sent    = 0u;
    daily_save();
}

/* ── IMP-L: MQTT heartbeat/keepalive tick ────────────────────────────────
 * Publishes a compact JSON status payload every KWS_HEARTBEAT_S seconds.
 * Keeps HA entities alive between sessions (HA default expiry = 10× publish).
 * Payload ≤ 140 bytes — a separate buffer avoids clobbering g_mqtt_pending_pl.
 * Guard: WiFi must be up; no attempt cap (if broker down, try next interval). */
static void heartbeat_tick(void)
{
    if (++g_hb_tick < KWS_HEARTBEAT_S) return;
    g_hb_tick = 0u;

    if (ReadCh(KWS_CH_WIFI) == 0) return;   /* defer — no WiFi */

    char pl[160];
    snprintf(pl, sizeof(pl),
             "{\"relay\":%u,\"temp_c\":%.1f,\"session\":%u,"
             "\"lifetime_kwh\":%.3f,\"uptime_s\":%u,\"fw\":\"%s\"}",
             (unsigned)g_relay_closed,
             g_ntc_ema,
             (unsigned)g_ev.active,
             g_lifetime_wh / 1000.0f,
             (unsigned)g_uptime_s,
             KWS_FW_VERSION_STR);
    char cmd[220];
    snprintf(cmd, sizeof(cmd), "publishMQTT %s %s", KWS_HEARTBEAT_TOPIC, pl);
    CMD_ExecuteCommand(cmd, 0);
}

/* ── IMP-M: HT7017 electrical alarm fault logger ─────────────────────────
 * Reads Ch7 every second.  On rising edge (0→non-zero) logs a fault entry.
 * Ch7 values: 1=OV, 2=UV, 3=OC, 4=OP.  The autoexec ChangeHandler opens the
 * relay; we just need to persist the event.
 * Uses a previous-alarm state to log only the transition, not every second.*/
static void ht7017_alarm_tick(void)
{
    static uint8_t s_prev_alarm = 0u;  /* C99-OK: local static, no session state */
    int alarm = ReadCh(7);   /* Ch7 = HT7017 alarm code */
    uint8_t cur = (alarm > 0 && alarm <= 4) ? (uint8_t)alarm : 0u;
    if (cur != 0u && cur != s_prev_alarm) {
        /* Rising edge or change in alarm code — log it. */
        float val = 0.0f;
        switch (cur) {
            case 1: val = (float)ReadCh(KWS_CH_RELAY); /* use voltage Ch1 */ val = (float)ReadCh(1) / 100.0f; break;
            case 2: val = (float)ReadCh(1) / 100.0f; break;
            case 3: val = (float)ReadCh(KWS_CH_CURRENT) / 1000.0f; break;
            case 4: val = (float)ReadCh(KWS_CH_POWER) / 10.0f; break;
            default: break;
        }
        fault_log(cur, g_uptime_s, val);
    }
    s_prev_alarm = cur;
}


void KWS303WF_RunEverySecond(void)
{
    g_uptime_s++;

    if (g_mqtt_pending) {
        /* WARN-1 FIX: two-part guard.
         * (a) WiFi down: defer until connected. Flag stays set.
         * (b) WiFi up but broker unreachable: publishMQTT may block up to
         *     TCP timeout (~20 s) → WDT fires. Cap at KWS_MQTT_MAX_ATTEMPTS
         *     so a permanently-unreachable broker does not cause repeated WDT
         *     reboots. Flag cleared AFTER the call (not before) so a WDT
         *     reboot (static zero-init) is safe — message is lost but no loop. */
        if (ReadCh(KWS_CH_WIFI) == 0) {
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                      "KWS303WF: MQTT publish deferred — WiFi down");
        } else if (g_mqtt_attempts >= KWS_MQTT_MAX_ATTEMPTS) {
            g_mqtt_pending  = 0;
            g_mqtt_attempts = 0;
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                      "KWS303WF: MQTT abandoned after %u attempts — broker unreachable?",
                      (unsigned)KWS_MQTT_MAX_ATTEMPTS);
        } else {
            char cmd[400];
            snprintf(cmd, sizeof(cmd), "publishMQTT %s %s",
                     KWS_MQTT_TOPIC, g_mqtt_pending_pl);
            g_mqtt_attempts++;
            CMD_ExecuteCommand(cmd, 0);  /* may block if broker unreachable  */
            g_mqtt_pending = 0;          /* clear after: safe on WDT reboot  */
        }
    }

    relay_sync(ReadCh(KWS_CH_RELAY));
    sess_tick();

    /* NTC: 9 out of 10 calls cost ~0.1 µs (countdown decrement only).
     * ADC + Steinhart-Hart + EMA + state machine runs on sample ticks only. */
    ntc_tick();

    /* IMP-J: check for 24 h boundary and publish/reset daily accumulator. */
    daily_tick();

    /* IMP-L: periodic MQTT heartbeat to keep HA entities alive. */
    heartbeat_tick();

    /* IMP-M: log HT7017 electrical alarm transitions to kws_faults.log. */
    ht7017_alarm_tick();

}

/* ============================================================================
 * QUICK TICK (~10 ms, called from DRV_RunQuickTick by OBK core)
 *
 * Improvement #Q1 — RunQuickTick for buttons + relay coil de-energise.
 *
 * WHY: btn_tick() uses a 3-consecutive-read debounce. When called from
 * RunEverySecond (1 Hz) a button press takes 3+ seconds to register.
 * At the ~10 ms quick-tick rate the same 3-tick debounce window is ~30 ms —
 * imperceptible to the user.
 *
 * relay_pulse_tick() de-energises the coil after g_pulse_off_at seconds.
 * g_uptime_s is still incremented in RunEverySecond (second resolution),
 * so the comparison g_uptime_s >= g_pulse_off_at remains correct; the coil
 * is now released within the first quick-tick AFTER the second boundary
 * (~0–10 ms overshoot) instead of 0–1 s.
 * ============================================================================ */
void KWS303WF_RunQuickTick(void)
{
    relay_pulse_tick();
    btn_tick();
}

/* ============================================================================
 * HOME ASSISTANT MQTT AUTO-DISCOVERY
 *
 * Improvement #H2 — publish HA MQTT discovery payloads for all 13 channels.
 *
 * HOW TO USE:
 *   1. In autoexec.bat (or the OBK web console), set your device short-name
 *      once — it must match your OBK "Short Name" (shown in the web UI header,
 *      e.g. "obkAB12CD"):
 *          kws_ha_devname obkAB12CD
 *   2. Trigger HA discovery from the OBK web UI (MQTT → HA Discovery button).
 *
 * WHY g_ha_devname:
 *   The HA discovery state_topic must match the OBK channel publish topic:
 *   "<devname>/<ch>/get".  The devname equals the OBK MQTT client ID (Short
 *   Name).  CFG_GetShortDeviceName() was not confirmed as an available symbol
 *   in this driver's include scope, so a driver-level config var is used
 *   instead to avoid unverified symbol dependencies.
 *
 * PUBLISH MODE:
 *   publishMQTT <topic> <payload> 1  — 3rd arg '1' = raw topic, OBK does NOT
 *   prepend the devname.  Required for "homeassistant/..." discovery topics.
 *
 * Channels published (13 total, BUG-19 FIX: was "14 total", actual count is 13):
 *   sensor       : Ch1 voltage, Ch2 current, Ch3 power, Ch4 frequency,
 *                  Ch5 power_factor, Ch6 energy, Ch9 temperature,
 *                  Ch10 ev_cost, Ch13 session_elapsed
 *   binary_sensor: Ch7 alarm, Ch12 session_active, Ch14 ntc_thermal_alarm
 *   switch       : Ch8 relay
 * ============================================================================ */

/* Publish one HA MQTT discovery config via raw topic (3rd arg '1'). */
static void ha_pub(const char *ha_pfx, const char *component,
                   const char *uid, const char *cfg_json)
{
    char cmd[640];
    snprintf(cmd, sizeof(cmd),
             "publishMQTT %s/%s/%s/config %s 1",
             ha_pfx, component, uid, cfg_json);
    CMD_ExecuteCommand(cmd, 0);
}

void KWS303WF_OnHassDiscovery(const char *topic)
{
    if (g_ha_devname[0] == '\0') {
        addLogAdv(LOG_WARN, LOG_FEATURE_ENERGY,
                  "KWS303WF: HA discovery skipped — set devname first: "
                  "kws_ha_devname <your-obk-shortname>");
        return;
    }

    /* Shared device block — groups all 13 entities under one HA device card
     * (BUG-19 FIX: was 14 entities; actual count is 13).                    */
    char dev_block[256];
    snprintf(dev_block, sizeof(dev_block),
             "\"device\":{\"identifiers\":[\"%s\"],"
             "\"name\":\"KWS-303WF\","
             "\"model\":\"KWS-303WF EV Smart Plug\","
             "\"manufacturer\":\"Custom\","
             "\"sw_version\":\"%s\"}",
             g_ha_devname, KWS_FW_VERSION_STR);

    const char *dev = g_ha_devname;
    char uid[72];
    char cfg[560];

/* Macro for simple numeric sensors — reduces repetition. */
#define HA_SENSOR(label, sfx, ch_num, unit, dev_cls, tmpl, s_cls)        \
    do {                                                                   \
        snprintf(uid, sizeof(uid), "%s_" sfx, dev);                       \
        snprintf(cfg, sizeof(cfg),                                         \
            "{\"name\":\"" label "\","                                     \
            "\"unique_id\":\"%s\","                                        \
            "\"state_topic\":\"%s/" ch_num "/get\","                      \
            "\"unit_of_measurement\":\"" unit "\","                        \
            "\"device_class\":\"" dev_cls "\","                            \
            "\"value_template\":\"" tmpl "\","                             \
            "\"state_class\":\"" s_cls "\",%s}",                           \
            uid, dev, dev_block);                                          \
        ha_pub(topic, "sensor", uid, cfg);                                 \
    } while (0)

    /* Ch1 Voltage  (integer = V × 100)   */
    HA_SENSOR("Voltage",      "voltage",      "1",  "V",
              "voltage",      "{{value|float/100}}",    "measurement");
    /* Ch2 Current  (integer = A × 1000)  */
    HA_SENSOR("Current",      "current",      "2",  "A",
              "current",      "{{value|float/1000}}",   "measurement");
    /* Ch3 Power    (integer = W × 10)    */
    HA_SENSOR("Power",        "power",        "3",  "W",
              "power",        "{{value|float/10}}",     "measurement");
    /* Ch4 Frequency (integer = Hz × 100) */
    HA_SENSOR("Frequency",    "frequency",    "4",  "Hz",
              "frequency",    "{{value|float/100}}",    "measurement");
    /* Ch5 Power Factor (integer = PF × 1000) */
    HA_SENSOR("Power Factor", "power_factor", "5",  "",
              "power_factor", "{{value|float/1000}}",   "measurement");
    /* Ch6 Energy   (integer = Wh × 10 → publish as kWh) */
    HA_SENSOR("Energy",       "energy",       "6",  "kWh",
              "energy",       "{{value|float/10000}}", "total_increasing");
    /* Ch9 Temperature (integer = °C × 100) */
    HA_SENSOR("Temperature",  "temperature",  "9",  "\xC2\xB0\x43",
              "temperature",  "{{value|float/100}}",    "measurement");

#undef HA_SENSOR

    /* Ch10 — EV Session Cost (integer = Rs × 100).
     * No HA standard device_class for currency — use icon only. */
    snprintf(uid, sizeof(uid), "%s_ev_cost", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"EV Session Cost\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/10/get\","
             "\"unit_of_measurement\":\"Rs\","
             "\"icon\":\"mdi:currency-inr\","
             "\"value_template\":\"{{value|float/100}}\","
             "\"state_class\":\"measurement\",%s}",
             uid, dev, dev_block);
    ha_pub(topic, "sensor", uid, cfg);

    /* Ch13 — Session Elapsed (integer = seconds since sess_start) */
    snprintf(uid, sizeof(uid), "%s_session_elapsed", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"EV Session Elapsed\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/13/get\","
             "\"unit_of_measurement\":\"s\","
             "\"icon\":\"mdi:timer-outline\","
             "\"state_class\":\"measurement\",%s}",
             uid, dev, dev_block);
    ha_pub(topic, "sensor", uid, cfg);

    /* Ch7 — Alarm binary_sensor (0=OK, any non-zero = problem tripped) */
    snprintf(uid, sizeof(uid), "%s_alarm", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"Alarm\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/7/get\","
             "\"payload_on\":\"1\",\"payload_off\":\"0\","
             "\"device_class\":\"problem\","
             "\"value_template\":\"{{1 if value|int > 0 else 0}}\",%s}",
             uid, dev, dev_block);
    ha_pub(topic, "binary_sensor", uid, cfg);

    /* Ch12 — EV Session Active binary_sensor */
    snprintf(uid, sizeof(uid), "%s_session_active", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"EV Session Active\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/12/get\","
             "\"payload_on\":\"1\",\"payload_off\":\"0\","
             "\"icon\":\"mdi:ev-station\",%s}",
             uid, dev, dev_block);
    ha_pub(topic, "binary_sensor", uid, cfg);

    /* Ch8 — Relay switch (0 = open/off, 100 = closed/on) */
    snprintf(uid, sizeof(uid), "%s_relay", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"Relay\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/8/get\","
             "\"command_topic\":\"%s/8/set\","
             "\"payload_on\":\"100\",\"payload_off\":\"0\","
             "\"state_on\":\"100\",\"state_off\":\"0\","
             "\"device_class\":\"outlet\",%s}",
             uid, dev, dev, dev_block);
    ha_pub(topic, "switch", uid, cfg);

    /* Ch14 — NTC Thermal Alarm binary_sensor (IMP-A)
     * 0=normal, 1=alarm latched (relay was opened by over-temperature trip).
     * HA device_class "heat" gives the flame icon and "problem" logic.      */
    snprintf(uid, sizeof(uid), "%s_ntc_alarm", dev);
    snprintf(cfg, sizeof(cfg),
             "{\"name\":\"NTC Thermal Alarm\","
             "\"unique_id\":\"%s\","
             "\"state_topic\":\"%s/14/get\","
             "\"payload_on\":\"1\",\"payload_off\":\"0\","
             "\"device_class\":\"heat\","
             "\"value_template\":\"{{1 if value|int > 0 else 0}}\",%s}",
             uid, dev, dev_block);
    ha_pub(topic, "binary_sensor", uid, cfg);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: HA discovery: 13 entities published " /* BUG-17 FIX: was 14 */
              "(devname=%s prefix=%s)", dev, topic);
}

#endif /* ENABLE_DRIVER_KWS303WF */
