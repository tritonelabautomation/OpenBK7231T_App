/*
 * drv_kws303wf.c  —  KWS-303WF Device Application Driver for OpenBK7231N
 *
 * SINGLE RESPONSIBILITY: Device-specific hardware & EV session application.
 * Owns: latching relay GPIO (P7/P8), buttons (P28/P26/P20), NTC (ADC P23),
 *       EV session tracking, cost calculation, MQTT session summary.
 *
 * ── THREE-DRIVER ARCHITECTURE ────────────────────────────────────────────
 *   drv_ht7017.c   = energy meter ONLY  (no relay, no display, no session)
 *   drv_st7735.c   = display ONLY       (reads channels, draws pixels)
 *   drv_kws303wf.c = device application (THIS FILE)
 *
 * ── CBU MODULE PIN MAP (verified: https://docs.libretiny.eu/boards/cbu/) ─
 *
 *   CBU pin | GPIO  | HAL# | Function in this firmware
 *   --------|-------|------|------------------------------------------
 *      8    | P8    |  7   | Relay OFF coil (OPENS contact = load off)
 *      9    | P7    |  8   | Relay ON  coil (CLOSES contact = load on)
 *      3    | P20   | 20   | Button [-]  reserved
 *      5    | P23   | 23   | NTC thermistor ADC input
 *     11    | P26   | 26   | Button [+]  reset EV session
 *     17    | P28   | 28   | Button [PWR] toggle relay
 *
 *   RELAY PIN NOTE — CRITICAL, DO NOT "FIX":
 *   KWS_RELAY_PIN_ON  = 8  (HAL number 8 drives physical pin 9 / P7 ON coil)
 *   KWS_RELAY_PIN_OFF = 7  (HAL number 7 drives physical pin 8 / P8 OFF coil)
 *   This is verified against working hardware. The HAL GPIO number does NOT
 *   equal the "P" suffix on this PCB for these two pins. DO NOT SWAP THEM.
 *
 * ── CHANNEL OUTPUT (this driver writes) ──────────────────────────────────
 *   Ch 8  = Relay state      (0 = open/safe,  100 = closed/on)
 *   Ch 9  = Temperature      (°C × 100,  e.g. 2716 = 27.16 °C)
 *   Ch 10 = EV session cost  (Rs × 100,  e.g.  376 = Rs 3.76)
 *
 * ── CHANNEL INPUT (this driver reads) ────────────────────────────────────
 *   Ch 2  = Current  (A × 1000)  from drv_ht7017 — peak tracking
 *   Ch 3  = Power    (W × 10)    from drv_ht7017 — auto-detect
 *   Ch 6  = Energy   (Wh × 10)   from drv_ht7017 — session kWh
 *   Ch 7  = Alarm    (0–4)       from drv_ht7017 — protection state
 *   Ch 8  = READ also to react to autoexec AddChangeHandler relay commands
 *
 * ── RELAY HARDWARE ───────────────────────────────────────────────────────
 *   Bistable/latching relay. Holds position without power.
 *   Pulse the ON coil HIGH for ~1 s  → contact CLOSES (load ON).
 *   Pulse the OFF coil HIGH for ~1 s → contact OPENS  (load OFF / safe).
 *   BOOT SAFETY: OFF coil is pulsed first on every boot so relay is always
 *   open after a power cut or OTA reboot.
 *   Non-blocking: coil pin is energised in relay_open/close(), then
 *   de-energised one second later in relay_pulse_tick() without any delay().
 *
 * ── BUTTONS (active-low, internal pull-up, polled at 1 Hz) ───────────────
 *   P28 [PWR] — toggle relay ON/OFF
 *   P26 [+]   — reset / new EV session
 *   P20 [-]   — reserved (logged only)
 *
 * ── NTC TEMPERATURE ──────────────────────────────────────────────────────
 *   Steinhart–Hart B-parameter equation.
 *   Wiring: Vcc → NTC(10kΩ) → P23 → Rs(10kΩ) → GND.
 *   Published to Ch9 every second.
 *
 * ── EV SESSION ───────────────────────────────────────────────────────────
 *   Reads Ch6 (Wh×10) from drv_ht7017 and tracks offset from session start.
 *   Cost = kWh_session × rate_Rs/kWh.   Published to Ch10.
 *   Persisted to /kws_session.cfg — survives power cuts.
 *   MQTT summary published to home/ev/session on session end.
 *
 * ── WATCHDOG-SAFE MQTT (core fix — read this section carefully) ───────────
 *
 *   ROOT CAUSE OF WATCHDOG RESETS:
 *   The BK7231N software watchdog fires if the OBK main task takes more than
 *   ~5 seconds in a single RunEverySecond() call.  When MQTT broker is
 *   unreachable (network cut, broker restarted, WiFi roaming, etc.) the call
 *     CMD_ExecuteCommand("publishMQTT home/ev/session {...}", 0)
 *   enters the LWIP TCP stack and blocks waiting for a TCP connect/send ACK
 *   that never arrives.  TCP timeout on LWIP can be 4–20 seconds, which
 *   exceeds the WDT limit → device reboots.  This used to happen whenever
 *   a charging session ended while the broker was offline.
 *
 *   SOLUTION — TWO-STEP DEFERRED PUBLISH:
 *
 *   Step 1 — at sess_end():
 *     Build the full JSON payload into the static g_mqtt_pl[] buffer.
 *     Then issue:
 *       CMD_ExecuteCommand("if MQTTOn then kws_do_mqtt_publish", 0)
 *     The OBK scripting engine evaluates "MQTTOn" synchronously in C code
 *     (it checks an in-memory integer flag, no TCP call).
 *     - If MQTT is connected: "kws_do_mqtt_publish" executes immediately
 *       and g_mqtt_pending is cleared.  Total extra time: microseconds.
 *     - If MQTT is offline:  the 'if' condition is false, the command
 *       returns immediately without executing the body.  Zero blocking.
 *       g_mqtt_pending stays 1.
 *
 *   Step 2 — in RunEverySecond(), mqtt_retry_tick():
 *     Repeats the same non-blocking "if MQTTOn then kws_do_mqtt_publish"
 *     every second until it succeeds or KWS_MQTT_RETRY_MAX is reached.
 *     Each check: microseconds.  No TCP. No blocking.  WDT safe.
 *
 *   WHY "if MQTTOn then ..." AND NOT A C FUNCTION?
 *   The OBK internal MQTT C API (new_mqtt.h) is not part of the public
 *   driver interface and its function names change between builds.
 *   "if MQTTOn then ..." is the DOCUMENTED, STABLE public interface used
 *   throughout OBK autoexec examples and the official README.
 *   References:
 *     - README.md: "if MQTTOn then <cmd> else <cmd>"
 *     - docs/autoexecExamples.md: "waitFor MQTTState 1"
 *     - docs/commands.md:         "waitFor MQTTState 1"
 *
 * ── autoexec.bat ─────────────────────────────────────────────────────────
 *   startDriver HT7017
 *   startDriver ST7735 14 16 9 17 15 24
 *   startDriver KWS303WF
 *   AddChangeHandler Channel7 != 0 kws_ch8_set 0      ; fault  → open relay
 *   AddChangeHandler Channel7 == 0 kws_ch8_set 100    ; clear  → close relay
 *
 * startDriver KWS303WF
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
 * SECTION A — CONFIGURATION  (edit only this section for device tuning)
 * ============================================================================ */

/* TGSPDCL tariff (Telangana domestic 201–300 slab) — update when slab changes */
#define KWS_EV_RATE_DEFAULT   7.00f

/* Vehicle profiles */
#define KWS_VEH2_NAME         "Ather Rizta Z"
#define KWS_VEH2_KM_PER_KWH  33.0f
#define KWS_VEH2_OLD_KMPL     30.0f    /* Honda Activa 4G replaced */
#define KWS_VEH4_NAME         "Nexon EV 45"
#define KWS_VEH4_KM_PER_KWH   7.8f
#define KWS_VEH4_OLD_KMPL     12.0f    /* Petrol Nexon city mileage replaced */
#define KWS_PETROL_RS_PER_L  107.0f    /* Hyderabad pump price */

/* EV charger auto-detect thresholds */
#define KWS_DETECT_W_MIN     500.0f    /* W above this = charger running     */
#define KWS_DETECT_S          30       /* consecutive seconds before commit  */
#define KWS_END_S             60       /* seconds below threshold = done     */
#define KWS_SPLIT_W         4500.0f    /* <this=2W(Rizta), >this=4W(Nexon)  */

/* ── Relay ───────────────────────────────────────────────────────────────
 * HAL GPIO 8 drives CBU pin 9 (P7) — ON coil  (CLOSES contact, load ON).
 * HAL GPIO 7 drives CBU pin 8 (P8) — OFF coil (OPENS  contact, load OFF).
 * These values are hardware-verified. DO NOT swap them.
 * See top-of-file pin map for full explanation.                           */
#define KWS_RELAY_PIN_ON      8        /* HAL GPIO 8 → P7 → ON  coil        */
#define KWS_RELAY_PIN_OFF     7        /* HAL GPIO 7 → P8 → OFF coil (safe) */

/* ── Buttons (active-low, internal pull-up) ─────────────────────────────
 * CBU pin 17 = GPIO P28 = HAL 28 — [PWR] toggle relay
 * CBU pin 11 = GPIO P26 = HAL 26 — [+]   new/reset session
 * CBU pin  3 = GPIO P20 = HAL 20 — [-]   reserved                       */
#define KWS_BTN_TOGGLE       28
#define KWS_BTN_SESSION      26
#define KWS_BTN_RESERVED     20

/* ── NTC (CBU pin 5 = GPIO P23 = HAL ADC channel 23) ────────────────────
 * Wiring: Vcc → NTC(10kΩ) → P23 → Rs(10kΩ) → GND
 * KWS_NTC_PULLUP 1 = NTC is between Vcc and the ADC pin                 */
#define KWS_ADC_CH            23
#define KWS_ADC_MAX        4095.0f
#define KWS_NTC_R25       10000.0f
#define KWS_NTC_B          3950.0f
#define KWS_NTC_RS        10000.0f
#define KWS_NTC_T0K        298.15f
#define KWS_NTC_PULLUP         1

/* ── OBK channels written by this driver ────────────────────────────────*/
#define KWS_CH_RELAY          8
#define KWS_CH_TEMP           9
#define KWS_CH_EVCOST        10

/* ── OBK channels read by this driver ───────────────────────────────────*/
#define KWS_CH_CURRENT        2
#define KWS_CH_POWER          3
#define KWS_CH_ENERGY         6

/* ── Persistence ─────────────────────────────────────────────────────────*/
#define KWS_SESSION_FILE     "/kws_session.cfg"
#define KWS_MQTT_TOPIC       "home/ev/session"

/* ── MQTT deferred-publish settings ─────────────────────────────────────
 * How many RunEverySecond() ticks to retry before giving up.
 * 30 ticks = 30 seconds.  Each retry attempt is non-blocking (~1 µs if
 * broker offline).  After KWS_MQTT_RETRY_MAX failures the pending flag is
 * cleared to avoid publishing stale data from a very old session.        */
#define KWS_MQTT_RETRY_MAX   30

/* ── MQTT payload buffer ─────────────────────────────────────────────────
 * Static buffer holds the JSON for one session-end publish.
 * Size breakdown:
 *   "publishMQTT home/ev/session " = 29 chars (command prefix)
 *   JSON body worst case:
 *     vehicle name 20 + floats×6 ~60 + JSON keys/braces ~50 = ~130 chars
 *   KWS_MQTT_PL_SZ = 256 gives ample margin for the payload alone.
 *   KWS_MQTT_CMD_SZ = 256 + 32 = 288 for the full publishMQTT command.  */
#define KWS_MQTT_PL_SZ      256
#define KWS_MQTT_CMD_SZ     288

/* ============================================================================
 * SECTION B — FILE-SCOPE EXTERNS & CHANNEL HELPERS
 * ============================================================================ */

/* OBK channel read — declared at file scope so it is shared by all
 * functions that need it (ReadCh, sess_tick, display_tick, etc.)          */
extern int CHANNEL_Get(int ch);

static void PubCh(int ch, int val)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "setChannel %d %d", ch, val);
    CMD_ExecuteCommand(buf, 0);
}

static int ReadCh(int ch)
{
    return CHANNEL_Get(ch);
}

/* ============================================================================
 * SECTION C — RELAY  (sole owner of HAL GPIO 7 and 8)
 *
 * NON-BLOCKING PULSE DESIGN:
 *   relay_open() / relay_close() set a coil pin HIGH immediately.
 *   relay_pulse_tick() is called at the very top of RunEverySecond().
 *   It sets the pin LOW when (g_uptime_s >= g_pulse_off_at).
 *   This removes all rtos_delay_milliseconds() blocking calls.
 * ============================================================================ */
static uint32_t g_uptime_s    = 0;  /* incremented at top of RunEverySecond */
static uint8_t  g_relay_closed = 0;
static uint8_t  g_pulse_pin    = 0;   /* 0 = no active pulse */
static uint32_t g_pulse_off_at = 0;   /* uptime tick to de-energise coil */

/* Must be called first in RunEverySecond(), BEFORE any other logic */
static void relay_pulse_tick(void)
{
    if (g_pulse_pin != 0 && g_uptime_s >= g_pulse_off_at) {
        HAL_PIN_SetOutputValue((int)g_pulse_pin, 0);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: coil GPIO%u de-energised", (unsigned)g_pulse_pin);
        g_pulse_pin = 0;
    }
}

/* Start a coil pulse.  If a prior pulse is still pending, cancel it first. */
static void relay_pulse_start(int pin)
{
    if (g_pulse_pin != 0) {
        HAL_PIN_SetOutputValue((int)g_pulse_pin, 0);
        g_pulse_pin = 0;
    }
    HAL_PIN_SetOutputValue(pin, 1);
    g_pulse_pin    = (uint8_t)pin;
    g_pulse_off_at = g_uptime_s + 1; /* de-energise after 1-second tick */
}

static void relay_open(void)
{
    relay_pulse_start(KWS_RELAY_PIN_OFF);  /* GPIO7 → P8 → OFF coil */
    g_relay_closed = 0;
    PubCh(KWS_CH_RELAY, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay OPEN  (GPIO%d/P8 OFF coil pulsed — load OFF, safe)",
              KWS_RELAY_PIN_OFF);
}

static void relay_close(void)
{
    relay_pulse_start(KWS_RELAY_PIN_ON);   /* GPIO8 → P7 → ON coil */
    g_relay_closed = 1;
    PubCh(KWS_CH_RELAY, 100);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay CLOSE (GPIO%d/P7 ON coil pulsed — load ON)",
              KWS_RELAY_PIN_ON);
}

/* Sync relay to a desired state.  Guard prevents double-pulse when Ch8
 * is re-written with the same value (e.g. every tick from relay_sync). */
static void relay_sync(int ch8_val)
{
    int want = (ch8_val >= 100);
    if (want  && !g_relay_closed) relay_close();
    if (!want &&  g_relay_closed) relay_open();
}

/* ============================================================================
 * SECTION D — NTC TEMPERATURE
 *
 * Steinhart–Hart B-parameter form:
 *   1/T = 1/T0 + (1/B) × ln(Rntc / R25)
 * Wiring (KWS_NTC_PULLUP = 1):
 *   Vcc → NTC → ADC_PIN → Rs(10kΩ) → GND
 *   Rntc = Rs × raw / (ADC_MAX − raw)
 * ============================================================================ */
static float ntc_celsius(void)
{
    uint32_t raw = HAL_ADC_Read(KWS_ADC_CH);
    if (raw == 0)                     return -99.0f; /* open-circuit guard  */
    if (raw >= (uint32_t)KWS_ADC_MAX) return 999.0f; /* short-circuit guard */
    float r = (float)raw;
#if KWS_NTC_PULLUP
    float rntc = KWS_NTC_RS * r / (KWS_ADC_MAX - r);
#else
    float rntc = KWS_NTC_RS * (KWS_ADC_MAX - r) / r;
#endif
    float temp_k = 1.0f / (1.0f / KWS_NTC_T0K
                            + logf(rntc / KWS_NTC_R25) / KWS_NTC_B);
    return temp_k - 273.15f;
}

/* ============================================================================
 * SECTION E — BUTTON MANAGER
 *
 * Polled at 1 Hz from RunEverySecond().
 * Active-low: pin reads 0 when pressed, 1 when released.
 * Debounce: require 3 consecutive matching reads before firing callback.
 *   At 1 Hz this equals a 3-second debounce, suitable for physical buttons
 *   that are not time-critical.
 * ============================================================================ */
#define BTN_MAX 4
typedef struct {
    int      pin;
    uint8_t  last;   /* last stable level (1 = released at boot)        */
    uint8_t  dbc;    /* debounce counter                                 */
    void   (*cb)(void);
} Btn_t;

static Btn_t   g_btns[BTN_MAX];
static uint8_t g_btn_n = 0;

static void btn_register(int pin, void (*cb)(void))
{
    if (g_btn_n >= BTN_MAX) return;
    g_btns[g_btn_n].pin  = pin;
    g_btns[g_btn_n].last = 1;   /* assume unpressed at boot */
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
        if (++b->dbc < 3)  continue;  /* 3 stable reads required */
        b->dbc = 0;
        b->last = lv;
        if (lv == 0 && b->cb) b->cb(); /* fire on falling edge */
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
    uint8_t  wh_resume;    /* 1 = waiting for first valid Ch6 after resume  */
    uint32_t session_id;
    float    wh_offset;
    float    wh_session;
    uint32_t seg_count;
    float    peak_w;
    float    peak_a;
    float    rate_rs;
} EvSess_t;

static EvSess_t  g_ev;
static float     g_rate_rs  = KWS_EV_RATE_DEFAULT;
static bool      g_auto_en  = true;

typedef enum { AD_IDLE, AD_DETECTING, AD_CHARGING } AdState_t;
static AdState_t g_ad     = AD_IDLE;
static uint32_t  g_adTick = 0;
static float     g_adWsum = 0.0f;
static uint32_t  g_endTk  = 0;

/* ── MQTT deferred-publish state ─────────────────────────────────────────
 * All three variables are static (no heap allocation).
 *
 * g_mqtt_pl[]        – JSON payload, built at sess_end(), reused on retry.
 * g_mqtt_pending     – 1 while a publish is outstanding.
 * g_mqtt_retry_ctr   – counts retry attempts; reset on success or give-up. */
static char    g_mqtt_pl[KWS_MQTT_PL_SZ];
static uint8_t g_mqtt_pending   = 0;
static uint8_t g_mqtt_retry_ctr = 0;

/* ── Session persistence ─────────────────────────────────────────────────*/
static void sess_save(void)
{
    FILE *f = fopen(KWS_SESSION_FILE, "w");
    if (!f) return;
    fprintf(f,
            "active=%u\nvehicle=%u\nid=%u\nwh_off=%.4f\nwh_sess=%.4f\n"
            "segs=%u\npeak_w=%.2f\npeak_a=%.4f\nrate=%.4f\n",
            (unsigned)g_ev.active, (unsigned)g_ev.vehicle, g_ev.session_id,
            g_ev.wh_offset, g_ev.wh_session, g_ev.seg_count,
            g_ev.peak_w, g_ev.peak_a, g_ev.rate_rs);
    fclose(f);
}

static bool sess_load(void)
{
    /* Use plain unsigned int for fscanf — %hhu is unreliable on newlib-nano
     * (Beken SDK) and can overwrite 4 bytes into a 1-byte field, corrupting
     * adjacent struct members.  Cast to uint8_t after reading.             */
    unsigned int ua = 0, uv = 0;
    FILE *f = fopen(KWS_SESSION_FILE, "r");
    if (!f) return false;
    memset(&g_ev, 0, sizeof(g_ev));
    fscanf(f,
           "active=%u\nvehicle=%u\nid=%u\nwh_off=%f\nwh_sess=%f\n"
           "segs=%u\npeak_w=%f\npeak_a=%f\nrate=%f\n",
           &ua, &uv, &g_ev.session_id,
           &g_ev.wh_offset, &g_ev.wh_session, &g_ev.seg_count,
           &g_ev.peak_w, &g_ev.peak_a, &g_ev.rate_rs);
    fclose(f);
    g_ev.active  = (uint8_t)ua;
    g_ev.vehicle = (uint8_t)uv;
    return (bool)g_ev.active;
}

/* ── Session lifecycle ───────────────────────────────────────────────────*/
static void sess_start(uint8_t veh)
{
    g_ev.active     = 1;
    g_ev.vehicle    = veh;
    g_ev.session_id++;
    g_ev.wh_offset  = (float)ReadCh(KWS_CH_ENERGY) / 10.0f;
    g_ev.wh_session = 0.0f;
    g_ev.seg_count  = 1;
    g_ev.peak_w     = 0.0f;
    g_ev.peak_a     = 0.0f;
    g_ev.rate_rs    = g_rate_rs;
    sess_save();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Session START id=%u veh=%s",
              g_ev.session_id,
              (veh == KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
}

static void sess_summary(void)
{
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    float km   = kwh * ((g_ev.vehicle == KWS_VEH_2W)
                        ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH);
    float pet  = km  / ((g_ev.vehicle == KWS_VEH_2W)
                        ? KWS_VEH2_OLD_KMPL : KWS_VEH4_OLD_KMPL)
                 * KWS_PETROL_RS_PER_L;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "=== EV Session Summary ===");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Vehicle : %s",
              (g_ev.vehicle == KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Energy  : %.3f kWh",  kwh);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Cost    : Rs %.2f @ Rs %.2f/unit",
              cost, g_ev.rate_rs);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Range   : %.1f km",   km);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Petrol  : Rs %.2f for same km", pet);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  SAVED   : Rs %.2f",   pet - cost);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Segs    : %u  Peak: %.1fW / %.3fA",
              g_ev.seg_count, g_ev.peak_w, g_ev.peak_a);
}

/*
 * sess_build_payload — serialise session data into g_mqtt_pl[].
 *
 * Called BEFORE g_ev.active is cleared so all fields are still valid.
 * Reused by mqtt_retry_tick() on every retry attempt — no recomputation
 * needed because the data is already in the static buffer.
 */
static void sess_build_payload(void)
{
    float kwh    = g_ev.wh_session / 1000.0f;
    float cost   = kwh * g_ev.rate_rs;
    float km_pkw = (g_ev.vehicle == KWS_VEH_2W)
                   ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH;
    float old_kl = (g_ev.vehicle == KWS_VEH_2W)
                   ? KWS_VEH2_OLD_KMPL : KWS_VEH4_OLD_KMPL;
    float km     = kwh * km_pkw;
    float saved  = km / old_kl * KWS_PETROL_RS_PER_L - cost;

    snprintf(g_mqtt_pl, KWS_MQTT_PL_SZ,
             "{\"vehicle\":\"%s\",\"kwh\":%.3f,\"cost_rs\":%.2f,"
             "\"km\":%.1f,\"saved_rs\":%.2f,\"segments\":%u,"
             "\"peak_w\":%.1f,\"peak_a\":%.3f}",
             (g_ev.vehicle == KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
             kwh, cost, km, saved,
             g_ev.seg_count, g_ev.peak_w, g_ev.peak_a);
}

/*
 * sess_do_publish — fire the actual publishMQTT command.
 *
 * ONLY called from CMD_DoMqttPublish, which is ONLY invoked by the OBK
 * scripting engine after it has confirmed that MQTTOn is true.
 * Therefore this function never blocks waiting for TCP.
 *
 * Uses a static command buffer (no stack VLA, no heap).
 * Buffer size: 29 (prefix) + KWS_MQTT_PL_SZ (256) + 1 (NUL) = 286 ≤ 288.
 */
static void sess_do_publish(void)
{
    static char cmd[KWS_MQTT_CMD_SZ];  /* static: no stack pressure */
    snprintf(cmd, KWS_MQTT_CMD_SZ,
             "publishMQTT %s %s", KWS_MQTT_TOPIC, g_mqtt_pl);
    CMD_ExecuteCommand(cmd, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: MQTT published → %s", KWS_MQTT_TOPIC);
}

/*
 * sess_end — end an active charging session.
 *
 * ── WATCHDOG FIX (this is where the bug used to be) ──────────────────────
 *
 * OLD (broken):
 *   sess_mqtt();  ← called publishMQTT directly, could block 4–20 seconds,
 *                   killing the watchdog when broker was unreachable.
 *
 * NEW (fixed):
 *   1. sess_build_payload() — builds JSON into static g_mqtt_pl[]. Fast.
 *   2. g_mqtt_pending = 1, g_mqtt_retry_ctr = 0.
 *   3. CMD_ExecuteCommand("if MQTTOn then kws_do_mqtt_publish", 0)
 *      • If MQTT online:  OBK scripting engine calls kws_do_mqtt_publish,
 *        which calls sess_do_publish() then clears g_mqtt_pending.
 *        Total time: microseconds.  WDT safe.
 *      • If MQTT offline: "if MQTTOn" is false, body is skipped entirely.
 *        CMD_ExecuteCommand returns immediately.  g_mqtt_pending stays 1.
 *        Total time: microseconds.  WDT safe.
 *      Either way this function returns in well under 1 millisecond.
 *   4. mqtt_retry_tick() in RunEverySecond() will retry every second until
 *      success or KWS_MQTT_RETRY_MAX ticks have elapsed.
 */
static void sess_end(void)
{
    if (!g_ev.active) return;

    sess_summary();
    sess_build_payload();   /* must be before g_ev.active = 0 */

    g_ev.active = 0;
    g_ad        = AD_IDLE;

    /* Attempt publish.  Non-blocking regardless of MQTT state.           */
    g_mqtt_pending   = 1;
    g_mqtt_retry_ctr = 0;
    CMD_ExecuteCommand("if MQTTOn then kws_do_mqtt_publish", 0);

    FILE *f = fopen(KWS_SESSION_FILE, "w");
    if (f) { fprintf(f, "active=0\n"); fclose(f); }

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Session ENDED id=%u  mqtt_pending=%u",
              g_ev.session_id, (unsigned)g_mqtt_pending);
}

/*
 * mqtt_retry_tick — deferred MQTT publish retry (called every second).
 *
 * If g_mqtt_pending is set, issues the same non-blocking
 * "if MQTTOn then kws_do_mqtt_publish" command.
 *   - MQTT still offline → returns instantly, counter increments.
 *   - MQTT reconnected  → kws_do_mqtt_publish fires, clears g_mqtt_pending.
 * After KWS_MQTT_RETRY_MAX attempts the pending flag is cleared so stale
 * data from a long-dead session is never accidentally published later.
 *
 * Time per call when offline: ~1–5 µs.  WDT budget impact: negligible.
 */
static void mqtt_retry_tick(void)
{
    if (!g_mqtt_pending) return;

    if (g_mqtt_retry_ctr >= KWS_MQTT_RETRY_MAX) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "KWS303WF: deferred MQTT abandoned after %u retries "
                  "(broker still unreachable)", (unsigned)KWS_MQTT_RETRY_MAX);
        g_mqtt_pending   = 0;
        g_mqtt_retry_ctr = 0;
        g_mqtt_pl[0]     = '\0';
        return;
    }

    g_mqtt_retry_ctr++;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: MQTT retry %u/%u",
              (unsigned)g_mqtt_retry_ctr, (unsigned)KWS_MQTT_RETRY_MAX);

    /* Non-blocking: OBK evaluates MQTTOn flag (in-memory int), no TCP.  */
    CMD_ExecuteCommand("if MQTTOn then kws_do_mqtt_publish", 0);
}

/* ── Session tick (called every second from RunEverySecond) ──────────────*/
static void sess_tick(void)
{
    float w  = (float)ReadCh(KWS_CH_POWER)   / 10.0f;
    float a  = (float)ReadCh(KWS_CH_CURRENT) / 1000.0f;
    float wh = (float)ReadCh(KWS_CH_ENERGY)  / 10.0f;

    if (g_ev.active) {
        /* On power-cut resume, Ch6 is 0 until HT7017 completes its first
         * UART poll.  Wait for a non-zero reading before anchoring offset. */
        if (g_ev.wh_resume && wh > 0.0f) {
            g_ev.wh_offset = wh;
            g_ev.wh_resume = 0;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "KWS303WF: resume offset anchored at %.4f Wh", wh);
        }
        g_ev.wh_session = wh - g_ev.wh_offset;
        if (g_ev.wh_session < 0.0f) g_ev.wh_session = 0.0f;
        if (w > g_ev.peak_w) g_ev.peak_w = w;
        if (a > g_ev.peak_a) g_ev.peak_a = a;
        float cost = (g_ev.wh_session / 1000.0f) * g_ev.rate_rs;
        PubCh(KWS_CH_EVCOST, (int)(cost * 100.0f));
        /* Persist session to flash every 60 seconds (not every second,
         * to limit flash write cycles on the internal LittleFS).          */
        static uint32_t s_sv = 0;
        if (++s_sv >= 60) { s_sv = 0; sess_save(); }
    } else {
        PubCh(KWS_CH_EVCOST, 0);
    }

    if (!g_auto_en) return;

    switch (g_ad) {
    case AD_IDLE:
        if (w >= KWS_DETECT_W_MIN) {
            g_ad = AD_DETECTING; g_adTick = 1; g_adWsum = w;
        }
        break;
    case AD_DETECTING:
        if (w < KWS_DETECT_W_MIN) { g_ad = AD_IDLE; break; }
        g_adWsum += w;
        if (++g_adTick >= KWS_DETECT_S) {
            float avg = g_adWsum / (float)g_adTick;
            if (!g_ev.active)
                sess_start((avg < KWS_SPLIT_W) ? KWS_VEH_2W : KWS_VEH_4W);
            g_ad = AD_CHARGING; g_endTk = 0;
        }
        break;
    case AD_CHARGING:
        if (w < KWS_DETECT_W_MIN) {
            if (++g_endTk >= KWS_END_S) sess_end();
        } else {
            g_endTk = 0;
        }
        break;
    }
}

/* ============================================================================
 * SECTION G — BUTTON CALLBACKS
 * ============================================================================ */
static void on_toggle(void)
{
    if (g_relay_closed) relay_open(); else relay_close();
}

static void on_session(void)
{
    /* [+] button: end current session (if active) then start a new one
     * with the same vehicle type.  Default to 2W if no session was active. */
    uint8_t veh = g_ev.active ? g_ev.vehicle : KWS_VEH_2W;
    if (g_ev.active) sess_end();
    sess_start(veh);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Session reset/started by [+] button");
}

static void on_reserved(void)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: P20 [-] pressed (reserved)");
}

/* ============================================================================
 * SECTION H — CONSOLE COMMANDS
 * ============================================================================ */
static commandResult_t CMD_RelayOn(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{ relay_close(); return CMD_RES_OK; }

static commandResult_t CMD_RelayOff(const void *ctx, const char *cmd,
                                    const char *args, int flags)
{ relay_open(); return CMD_RES_OK; }

/* kws_ch8_set <val> — invoked by autoexec AddChangeHandler on Ch7 change */
static commandResult_t CMD_Ch8Set(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    relay_sync(atoi(args));
    return CMD_RES_OK;
}

static commandResult_t CMD_Rate(const void *ctx, const char *cmd,
                                const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_rate: current = %.4f Rs/kWh", g_rate_rs);
        return CMD_RES_OK;
    }
    g_rate_rs = (float)atof(args);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "kws_rate: set to %.4f Rs/kWh", g_rate_rs);
    return CMD_RES_OK;
}

static commandResult_t CMD_Sess2W(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{ if (g_ev.active) sess_end(); sess_start(KWS_VEH_2W); return CMD_RES_OK; }

static commandResult_t CMD_Sess4W(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{ if (g_ev.active) sess_end(); sess_start(KWS_VEH_4W); return CMD_RES_OK; }

static commandResult_t CMD_SessEnd(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{ sess_end(); return CMD_RES_OK; }

static commandResult_t CMD_SessStat(const void *ctx, const char *cmd,
                                    const char *args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS Session: %s  veh=%s  kWh=%.3f  cost=Rs%.2f  segs=%u",
              g_ev.active ? "ACTIVE" : "idle",
              (g_ev.vehicle == KWS_VEH_2W) ? KWS_VEH2_NAME :
              (g_ev.vehicle == KWS_VEH_4W) ? KWS_VEH4_NAME : "none",
              g_ev.wh_session / 1000.0f,
              g_ev.wh_session / 1000.0f * g_ev.rate_rs,
              g_ev.seg_count);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  relay=%s  auto=%s  uptime=%us  mqtt_pending=%u  mqtt_retry=%u",
              g_relay_closed ? "CLOSED" : "OPEN",
              g_auto_en ? "ON" : "OFF",
              (unsigned)g_uptime_s,
              (unsigned)g_mqtt_pending,
              (unsigned)g_mqtt_retry_ctr);
    return CMD_RES_OK;
}

static commandResult_t CMD_SessAuto(const void *ctx, const char *cmd,
                                    const char *args, int flags)
{
    g_auto_en = !g_auto_en;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: auto-detect %s", g_auto_en ? "ON" : "OFF");
    return CMD_RES_OK;
}

/*
 * CMD_DoMqttPublish — "kws_do_mqtt_publish"
 *
 * This command is ONLY called by the OBK scripting engine when it has
 * already confirmed that MQTTOn == true, via:
 *   "if MQTTOn then kws_do_mqtt_publish"
 *
 * Therefore sess_do_publish() is guaranteed to find a live MQTT connection
 * and will not block.
 *
 * It can also be invoked manually from the console for debugging.
 * If g_mqtt_pl[] is empty (no pending session data), it logs and returns.
 */
static commandResult_t CMD_DoMqttPublish(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    if (!g_mqtt_pl[0]) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "kws_do_mqtt_publish: no payload (called with no pending session)");
        return CMD_RES_OK;
    }
    sess_do_publish();
    g_mqtt_pending   = 0;
    g_mqtt_retry_ctr = 0;
    g_mqtt_pl[0]     = '\0';  /* clear so stale payload can't be re-sent */
    return CMD_RES_OK;
}

/* ============================================================================
 * SECTION I — OPENBK LIFECYCLE
 * ============================================================================ */
void KWS303WF_Init(void)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: init");

    /* ── Relay GPIO setup ────────────────────────────────────────────────
     * Set both coil pins as outputs and drive LOW first (safe default).
     * Then pulse the OFF coil (GPIO7 / P8) so the relay is guaranteed OPEN
     * on every boot, OTA update, or crash-restart.                        */
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_ON);
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_OFF);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_ON,  0);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_OFF, 0);
    relay_open();   /* BOOT SAFETY: guaranteed OPEN */

    /* ── Buttons ─────────────────────────────────────────────────────────
     * CBU pin 17 (P28/HAL28), pin 11 (P26/HAL26), pin 3 (P20/HAL20)     */
    btn_register(KWS_BTN_TOGGLE,   on_toggle);
    btn_register(KWS_BTN_SESSION,  on_session);
    btn_register(KWS_BTN_RESERVED, on_reserved);

    /* ── Session resume after power cut ──────────────────────────────────
     * HT7017 has not polled yet at Init time, so Ch6 = 0.
     * We set wh_resume = 1 so sess_tick() anchors the offset on the first
     * tick when Ch6 becomes non-zero.                                     */
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

    /* ── Clear MQTT deferred state ───────────────────────────────────────
     * If the device rebooted mid-retry the pending flag is gone.
     * That is acceptable — a session summary in the log is always present. */
    g_mqtt_pl[0]     = '\0';
    g_mqtt_pending   = 0;
    g_mqtt_retry_ctr = 0;

    /* ── Register console commands ───────────────────────────────────────*/
    CMD_RegisterCommand("kws_relay_on",        CMD_RelayOn,      NULL);
    CMD_RegisterCommand("kws_relay_off",       CMD_RelayOff,     NULL);
    CMD_RegisterCommand("kws_ch8_set",         CMD_Ch8Set,       NULL);
    CMD_RegisterCommand("kws_rate",            CMD_Rate,         NULL);
    CMD_RegisterCommand("kws_session2w",       CMD_Sess2W,       NULL);
    CMD_RegisterCommand("kws_session4w",       CMD_Sess4W,       NULL);
    CMD_RegisterCommand("kws_session_end",     CMD_SessEnd,      NULL);
    CMD_RegisterCommand("kws_session_stat",    CMD_SessStat,     NULL);
    CMD_RegisterCommand("kws_session_auto",    CMD_SessAuto,     NULL);
    /*
     * kws_do_mqtt_publish — DO NOT call this directly.
     * It is intended to be called only via:
     *   "if MQTTOn then kws_do_mqtt_publish"
     * Calling it directly when MQTT is offline will block the main loop.
     * It is registered as a command (not a private function) because OBK's
     * scripting engine can only invoke registered commands.               */
    CMD_RegisterCommand("kws_do_mqtt_publish", CMD_DoMqttPublish, NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: ready"
              "  relay=GPIO%d(P7-ON)/GPIO%d(P8-OFF)"
              "  ntc=ADC%d  rate=Rs%.2f/kWh"
              "  wdt_fix=deferred_mqtt",
              KWS_RELAY_PIN_ON, KWS_RELAY_PIN_OFF,
              KWS_ADC_CH, g_rate_rs);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "  2W: %s %.1fkm/kWh  4W: %s %.1fkm/kWh  petrol=Rs%.0f/L",
              KWS_VEH2_NAME, KWS_VEH2_KM_PER_KWH,
              KWS_VEH4_NAME, KWS_VEH4_KM_PER_KWH,
              KWS_PETROL_RS_PER_L);
}

/*
 * KWS303WF_RunEverySecond — main 1 Hz tick.
 *
 * ORDER IS IMPORTANT:
 *   1. g_uptime_s++         — increment before any time-based logic.
 *   2. relay_pulse_tick()   — de-energise coil if pulse window expired.
 *   3. mqtt_retry_tick()    — retry deferred MQTT publish (non-blocking).
 *   4. relay_sync()         — react to externally written Ch8.
 *   5. btn_tick()           — debounce and fire button callbacks.
 *   6. sess_tick()          — EV session logic and cost publish.
 *   7. ntc_celsius + PubCh  — temperature to Ch9.
 */
void KWS303WF_RunEverySecond(void)
{
    g_uptime_s++;

    /* 1. Non-blocking relay coil de-energise */
    relay_pulse_tick();

    /* 2. Watchdog-safe MQTT deferred-publish retry.
     *    This is the core fix for the watchdog resets.
     *    Each call costs ~1–5 µs when MQTT is offline.
     *    See sect F / mqtt_retry_tick() comment for full explanation.     */
    mqtt_retry_tick();

    /* 3. React to Ch8 written by autoexec AddChangeHandler               */
    relay_sync(ReadCh(KWS_CH_RELAY));

    /* 4. Button debounce poll                                             */
    btn_tick();

    /* 5. EV session auto-detect, cost accumulation, 60s save             */
    sess_tick();

    /* 6. NTC temperature → Ch9                                           */
    float tc = ntc_celsius();
    if (tc > -90.0f && tc < 990.0f)
        PubCh(KWS_CH_TEMP, (int)(tc * 100.0f));
}

#endif /* ENABLE_DRIVER_KWS303WF */
