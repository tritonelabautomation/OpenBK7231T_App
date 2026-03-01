/*
 * drv_kws303wf.c — KWS-303WF Device Application Driver for OpenBK7231N
 *
 * SINGLE RESPONSIBILITY: Device-specific hardware & EV session application.
 * Owns: latching relay GPIO (P7/P8), buttons (P28/P26/P20), NTC temp (ADC3),
 *       EV session tracking, cost calculation, MQTT session summary.
 *
 * This driver is the "glue" for this specific device.
 * drv_ht7017.c = energy meter only (knows nothing of KWS hardware)
 * drv_st7735.c = display only    (knows nothing of KWS hardware)
 * drv_kws303wf.c = device app    (knows both, coordinates via OBK channels)
 *
 * startDriver KWS303WF
 *
 * ── CHANNEL OUTPUT (this driver writes) ──────────────────────────────────
 *   Ch 8  = Relay state   (0=open/safe, 100=closed/on)
 *   Ch 9  = Temperature   (degrees-C x 100, e.g. 2716 = 27.16 C)
 *   Ch 10 = EV session cost (Rs x 100, e.g. 376 = Rs 3.76)
 *
 * ── CHANNEL INPUT (this driver reads) ────────────────────────────────────
 *   Ch 3  = Power  (W x 10)  from drv_ht7017 — for auto-detect
 *   Ch 2  = Current (A x 1000) from drv_ht7017 — for peak tracking
 *   Ch 6  = Energy (Wh x 10) from drv_ht7017 — for session kWh
 *   Ch 8  = Also READ to react to externally-written relay commands
 *           (autoexec AddChangeHandler writes Ch8=0 on fault, Ch8=100 on clear)
 *
 * ── RELAY HARDWARE (KWS-303WF schematic-verified) ─────────────────────────
 *   P7 = ON coil  — pulse HIGH 200ms then LOW = contact CLOSES (load ON)
 *   P8 = OFF coil — pulse HIGH 200ms then LOW = contact OPENS  (load OFF)
 *   Bistable/latching: holds position without power. Pulse only to change.
 *   BOOT SAFETY: always pulse P8 first so relay is OPEN on every boot.
 *
 * ── BUTTONS (active-low, internal pull-up, 1Hz poll) ─────────────────────
 *   P28 [PWR] — toggle relay ON/OFF
 *   P26 [+]   — reset/new EV session
 *   P20 [-]   — reserved
 *
 * ── NTC TEMPERATURE ───────────────────────────────────────────────────────
 *   P23 / BK7231N ADC ch3. B=3950, R25=10k, Rs=10k, Vcc-NTC-pin-Rs-GND.
 *   Published to Ch9 every second.
 *
 * ── EV SESSION ────────────────────────────────────────────────────────────
 *   Reads Ch6 (Wh x 10) from drv_ht7017. Tracks offset from session start.
 *   Cost = (kWh_session) x rate_Rs. Published to Ch10.
 *   Session persisted to /kws_session.cfg — survives power cuts.
 *   MQTT summary published to home/ev/session on session end.
 *
 * ── autoexec.bat ─────────────────────────────────────────────────────────
 *   startDriver HT7017
 *   startDriver ST7735 14 16 9 17 15 24
 *   startDriver KWS303WF
 *   AddChangeHandler Channel7 != 0 kws_ch8_set 0    (fault: open relay)
 *   AddChangeHandler Channel7 == 0 kws_ch8_set 100  (clear: close relay)
 */

#include "../obk_config.h"
#if ENABLE_DRIVER_KWS303WF

#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
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
 * SECTION A — CONFIGURATION  (update rates / device specifics here ONLY)
 * ============================================================================ */

/* TGSPDCL tariff — update this line when your slab changes */
#define KWS_EV_RATE_DEFAULT   7.00f   /* Rs/kWh — Telangana domestic 201-300 slab */

/* Vehicle profiles */
#define KWS_VEH2_NAME         "Ather Rizta Z"
#define KWS_VEH2_KM_PER_KWH  33.0f
#define KWS_VEH2_OLD_KMPL     30.0f   /* Honda Activa 4G replaced */
#define KWS_VEH4_NAME         "Nexon EV 45"
#define KWS_VEH4_KM_PER_KWH   7.8f
#define KWS_VEH4_OLD_KMPL     12.0f   /* Petrol Nexon city mileage replaced */
#define KWS_PETROL_RS_PER_L  107.0f   /* Hyderabad pump price — update here  */

/* Auto-detect thresholds */
#define KWS_DETECT_W_MIN     500.0f   /* W: above this = charger running      */
#define KWS_DETECT_S          30      /* seconds of window before committing  */
#define KWS_END_S             60      /* seconds below threshold = charge done */
#define KWS_SPLIT_W         4500.0f   /* below=2W(Rizta), above=4W(Nexon)    */

/* Relay hardware */
#define KWS_RELAY_PIN_ON      7       /* P7 = ON coil  — closes contact       */
#define KWS_RELAY_PIN_OFF     8       /* P8 = OFF coil — opens contact (safe) */
#define KWS_RELAY_PULSE_MS  200       /* coil energise duration ms            */

/* Buttons */
#define KWS_BTN_TOGGLE       28       /* P28 — toggle relay                   */
#define KWS_BTN_SESSION      26       /* P26 — new EV session                 */
#define KWS_BTN_RESERVED     20       /* P20 — reserved                       */

/* NTC */
#define KWS_ADC_CH            3       /* BK7231N ADC ch3 = P23                */
#define KWS_ADC_MAX        4095.0f
#define KWS_NTC_R25       10000.0f
#define KWS_NTC_B          3950.0f
#define KWS_NTC_RS        10000.0f
#define KWS_NTC_T0K        298.15f

/* OBK channels written by this driver */
#define KWS_CH_RELAY          8
#define KWS_CH_TEMP           9
#define KWS_CH_EVCOST        10

/* OBK channels read by this driver */
#define KWS_CH_POWER          3
#define KWS_CH_CURRENT        2
#define KWS_CH_ENERGY         6

/* Persistence */
#define KWS_SESSION_FILE     "/kws_session.cfg"
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

static int ReadCh(int ch)
{
    extern int CHANNEL_Get(int ch);
    return CHANNEL_Get(ch);
}

/* ============================================================================
 * SECTION C — RELAY  (sole owner of P7 and P8 GPIO)
 * ============================================================================ */
static uint8_t g_relay_closed = 0;

static void relay_pulse(int pin)
{
    extern int rtos_delay_milliseconds(uint32_t ms);
    HAL_PIN_SetOutputValue(pin, 1);
    (void)rtos_delay_milliseconds(KWS_RELAY_PULSE_MS);
    HAL_PIN_SetOutputValue(pin, 0);
}

static void relay_open(void)
{
    relay_pulse(KWS_RELAY_PIN_OFF);
    g_relay_closed = 0;
    PubCh(KWS_CH_RELAY, 0);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay OPEN  (P%d %dms, load OFF, safe)",
              KWS_RELAY_PIN_OFF, KWS_RELAY_PULSE_MS);
}

static void relay_close(void)
{
    relay_pulse(KWS_RELAY_PIN_ON);
    g_relay_closed = 1;
    PubCh(KWS_CH_RELAY, 100);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "KWS303WF: Relay CLOSE (P%d %dms, load ON)",
              KWS_RELAY_PIN_ON, KWS_RELAY_PULSE_MS);
}

/* Sync physical relay to match a desired channel value */
static void relay_sync(int ch8_val)
{
    int want = (ch8_val >= 100);
    if (want && !g_relay_closed)  relay_close();
    if (!want && g_relay_closed)  relay_open();
}

/* ============================================================================
 * SECTION D — NTC TEMPERATURE
 * ============================================================================ */
extern uint32_t HAL_ADC_Read(uint8_t channel);

static float ntc_celsius(void)
{
    uint32_t raw = HAL_ADC_Read(KWS_ADC_CH);
    if (raw == 0)                    return -99.0f;
    if (raw >= (uint32_t)KWS_ADC_MAX) return 999.0f;
    float r    = (float)raw;
    float rntc = KWS_NTC_RS * r / (KWS_ADC_MAX - r);
    float tk   = 1.0f / (1.0f/KWS_NTC_T0K + logf(rntc/KWS_NTC_R25)/KWS_NTC_B);
    return tk - 273.15f;
}

/* ============================================================================
 * SECTION E — BUTTON MANAGER  (polled 1Hz from RunEverySecond)
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
    extern int HAL_PIN_ReadDigitalInput(int pin);
    uint8_t i;
    for (i = 0; i < g_btn_n; i++) {
        Btn_t  *b  = &g_btns[i];
        uint8_t lv = (uint8_t)HAL_PIN_ReadDigitalInput(b->pin);
        if (lv == b->last) { b->dbc = 0; continue; }
        if (++b->dbc < 1)  continue;
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
    uint32_t session_id;
    float    wh_offset;
    float    wh_session;
    uint32_t seg_count;
    float    peak_w;
    float    peak_a;
    float    rate_rs;
} EvSess_t;

static EvSess_t  g_ev;
static float     g_rate_rs   = KWS_EV_RATE_DEFAULT;
static uint32_t  g_uptime_s  = 0;
static bool      g_auto_en   = true;

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
               "segs=%u\npeak_w=%.2f\npeak_a=%.4f\nrate=%.4f\n",
            (unsigned)g_ev.active, (unsigned)g_ev.vehicle, g_ev.session_id,
            g_ev.wh_offset, g_ev.wh_session, g_ev.seg_count,
            g_ev.peak_w, g_ev.peak_a, g_ev.rate_rs);
    fclose(f);
}

static bool sess_load(void)
{
    FILE *f = fopen(KWS_SESSION_FILE, "r");
    if (!f) return false;
    memset(&g_ev, 0, sizeof(g_ev));
    fscanf(f, "active=%hhu\nvehicle=%hhu\nid=%u\nwh_off=%f\nwh_sess=%f\n"
              "segs=%u\npeak_w=%f\npeak_a=%f\nrate=%f\n",
           &g_ev.active, &g_ev.vehicle, &g_ev.session_id,
           &g_ev.wh_offset, &g_ev.wh_session, &g_ev.seg_count,
           &g_ev.peak_w, &g_ev.peak_a, &g_ev.rate_rs);
    fclose(f);
    return (bool)g_ev.active;
}

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
              (veh==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
}

static void sess_mqtt(void)
{
    float kwh    = g_ev.wh_session / 1000.0f;
    float cost   = kwh * g_ev.rate_rs;
    float km_pkw = (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH;
    float old_kl = (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_OLD_KMPL    : KWS_VEH4_OLD_KMPL;
    float km     = kwh * km_pkw;
    float saved  = km / old_kl * KWS_PETROL_RS_PER_L - cost;
    char pl[320], cmd[380];
    snprintf(pl, sizeof(pl),
             "{\"vehicle\":\"%s\",\"kwh\":%.3f,\"cost_rs\":%.2f,"
             "\"km\":%.1f,\"saved_rs\":%.2f,\"segments\":%u,"
             "\"peak_w\":%.1f,\"peak_a\":%.3f}",
             (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME,
             kwh, cost, km, saved, g_ev.seg_count,
             g_ev.peak_w, g_ev.peak_a);
    snprintf(cmd, sizeof(cmd), "publishMQTT %s %s", KWS_MQTT_TOPIC, pl);
    CMD_ExecuteCommand(cmd, 0);
}

static void sess_summary(void)
{
    float kwh  = g_ev.wh_session / 1000.0f;
    float cost = kwh * g_ev.rate_rs;
    float km   = kwh * ((g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_KM_PER_KWH : KWS_VEH4_KM_PER_KWH);
    float pet  = km / ((g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_OLD_KMPL : KWS_VEH4_OLD_KMPL)
                 * KWS_PETROL_RS_PER_L;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"=== EV Session Summary ===");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Vehicle : %s",
              (g_ev.vehicle==KWS_VEH_2W) ? KWS_VEH2_NAME : KWS_VEH4_NAME);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Energy  : %.3f kWh", kwh);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Cost    : Rs %.2f @ Rs %.2f/unit",cost,g_ev.rate_rs);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Range   : %.1f km", km);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Petrol  : Rs %.2f for same km", pet);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  SAVED   : Rs %.2f", pet-cost);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Segs    : %u  Peak: %.1fW / %.3fA",
              g_ev.seg_count, g_ev.peak_w, g_ev.peak_a);
}

static void sess_end(void)
{
    if (!g_ev.active) return;
    g_ev.active = 0;
    g_ad        = AD_IDLE;
    sess_summary();
    sess_mqtt();
    FILE *f = fopen(KWS_SESSION_FILE,"w");
    if (f) { fprintf(f,"active=0\n"); fclose(f); }
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: Session ENDED");
}

static void sess_tick(void)
{
    float w  = (float)ReadCh(KWS_CH_POWER)   / 10.0f;
    float a  = (float)ReadCh(KWS_CH_CURRENT) / 1000.0f;
    float wh = (float)ReadCh(KWS_CH_ENERGY)  / 10.0f;

    if (g_ev.active) {
        g_ev.wh_session = wh - g_ev.wh_offset;
        if (g_ev.wh_session < 0.0f) g_ev.wh_session = 0.0f;
        if (w > g_ev.peak_w) g_ev.peak_w = w;
        if (a > g_ev.peak_a) g_ev.peak_a = a;
        float cost = (g_ev.wh_session/1000.0f) * g_ev.rate_rs;
        PubCh(KWS_CH_EVCOST, (int)(cost * 100.0f));
        static uint32_t s_sv = 0;
        if (++s_sv >= 60) { s_sv = 0; sess_save(); }
    } else {
        PubCh(KWS_CH_EVCOST, 0);
    }

    if (!g_auto_en) return;

    switch (g_ad) {
    case AD_IDLE:
        if (w >= KWS_DETECT_W_MIN) { g_ad=AD_DETECTING; g_adTick=1; g_adWsum=w; }
        break;
    case AD_DETECTING:
        if (w < KWS_DETECT_W_MIN) { g_ad=AD_IDLE; break; }
        g_adWsum += w;
        if (++g_adTick >= KWS_DETECT_S) {
            float avg = g_adWsum / (float)g_adTick;
            if (!g_ev.active)
                sess_start((avg < KWS_SPLIT_W) ? KWS_VEH_2W : KWS_VEH_4W);
            g_ad=AD_CHARGING; g_endTk=0;
        }
        break;
    case AD_CHARGING:
        if (w < KWS_DETECT_W_MIN) { if (++g_endTk >= KWS_END_S) sess_end(); }
        else                       { g_endTk=0; }
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
    if (g_ev.active) sess_end();
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: Session reset by button");
}

static void on_reserved(void)
{
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"KWS303WF: P20[-] pressed");
}

/* ============================================================================
 * SECTION H — CONSOLE COMMANDS
 * ============================================================================ */
static commandResult_t CMD_RelayOn(const void *ctx,const char *cmd,
                                   const char *args,int flags)
{ relay_close(); return CMD_RES_OK; }

static commandResult_t CMD_RelayOff(const void *ctx,const char *cmd,
                                    const char *args,int flags)
{ relay_open(); return CMD_RES_OK; }

/* kws_ch8_set <val> — called by autoexec AddChangeHandler for protection */
static commandResult_t CMD_Ch8Set(const void *ctx,const char *cmd,
                                  const char *args,int flags)
{
    if (!args||!*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    relay_sync(atoi(args));
    return CMD_RES_OK;
}

static commandResult_t CMD_Rate(const void *ctx,const char *cmd,
                                const char *args,int flags)
{
    if (!args||!*args) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate: %.4f Rs/kWh",g_rate_rs);
        return CMD_RES_OK;
    }
    g_rate_rs = (float)atof(args);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"kws_rate set %.4f Rs/kWh",g_rate_rs);
    return CMD_RES_OK;
}

static commandResult_t CMD_Sess2W(const void *ctx,const char *cmd,
                                  const char *args,int flags)
{ if(g_ev.active) sess_end(); sess_start(KWS_VEH_2W); return CMD_RES_OK; }

static commandResult_t CMD_Sess4W(const void *ctx,const char *cmd,
                                  const char *args,int flags)
{ if(g_ev.active) sess_end(); sess_start(KWS_VEH_4W); return CMD_RES_OK; }

static commandResult_t CMD_SessEnd(const void *ctx,const char *cmd,
                                   const char *args,int flags)
{ sess_end(); return CMD_RES_OK; }

static commandResult_t CMD_SessStat(const void *ctx,const char *cmd,
                                    const char *args,int flags)
{
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS Session: %s  veh=%s  kWh=%.3f  cost=Rs%.2f  segs=%u",
              g_ev.active?"ACTIVE":"idle",
              (g_ev.vehicle==KWS_VEH_2W)?KWS_VEH2_NAME:
              (g_ev.vehicle==KWS_VEH_4W)?KWS_VEH4_NAME:"none",
              g_ev.wh_session/1000.0f,
              g_ev.wh_session/1000.0f*g_ev.rate_rs,
              g_ev.seg_count);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  relay=%s  auto=%s  uptime=%us",
              g_relay_closed?"CLOSED":"OPEN",
              g_auto_en?"ON":"OFF", (unsigned)g_uptime_s);
    return CMD_RES_OK;
}

static commandResult_t CMD_SessAuto(const void *ctx,const char *cmd,
                                    const char *args,int flags)
{
    g_auto_en = !g_auto_en;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS303WF: auto-detect %s", g_auto_en?"ON":"OFF");
    return CMD_RES_OK;
}

/* ============================================================================
 * SECTION I — OPENBK LIFECYCLE
 * ============================================================================ */
void KWS303WF_Init(void)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "KWS303WF: init");

    /* Relay: setup coil pins then BOOT SAFETY open */
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_ON);
    HAL_PIN_Setup_Output(KWS_RELAY_PIN_OFF);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_ON,  0);
    HAL_PIN_SetOutputValue(KWS_RELAY_PIN_OFF, 0);
    relay_open();   /* guaranteed OPEN on every boot / OTA reboot */

    /* Buttons */
    btn_register(KWS_BTN_TOGGLE,   on_toggle);
    btn_register(KWS_BTN_SESSION,  on_session);
    btn_register(KWS_BTN_RESERVED, on_reserved);

    /* Session: resume if power cut during active charge */
    if (sess_load() && g_ev.active) {
        g_ev.seg_count++;
        g_ev.wh_offset = (float)ReadCh(KWS_CH_ENERGY) / 10.0f;
        g_ad = AD_CHARGING;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "KWS303WF: Session RESUMED id=%u segs=%u",
                  g_ev.session_id, g_ev.seg_count);
    } else {
        memset(&g_ev, 0, sizeof(g_ev));
        g_ev.rate_rs = g_rate_rs;
    }

    CMD_RegisterCommand("kws_relay_on",    CMD_RelayOn,  NULL);
    CMD_RegisterCommand("kws_relay_off",   CMD_RelayOff, NULL);
    CMD_RegisterCommand("kws_ch8_set",     CMD_Ch8Set,   NULL);
    CMD_RegisterCommand("kws_rate",        CMD_Rate,     NULL);
    CMD_RegisterCommand("kws_session2w",   CMD_Sess2W,   NULL);
    CMD_RegisterCommand("kws_session4w",   CMD_Sess4W,   NULL);
    CMD_RegisterCommand("kws_session_end", CMD_SessEnd,  NULL);
    CMD_RegisterCommand("kws_session_stat",CMD_SessStat, NULL);
    CMD_RegisterCommand("kws_session_auto",CMD_SessAuto, NULL);

    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "KWS303WF: ready  relay=P%d/P%d  ntc=ADC%d  rate=Rs%.2f/kWh",
              KWS_RELAY_PIN_ON, KWS_RELAY_PIN_OFF, KWS_ADC_CH, g_rate_rs);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  2W: %s %.1fkm/kWh  4W: %s %.1fkm/kWh  petrol=Rs%.0f/L",
              KWS_VEH2_NAME, KWS_VEH2_KM_PER_KWH,
              KWS_VEH4_NAME, KWS_VEH4_KM_PER_KWH, KWS_PETROL_RS_PER_L);
}

void KWS303WF_RunEverySecond(void)
{
    g_uptime_s++;

    /* React to Ch8 written externally by autoexec AddChangeHandler */
    relay_sync(ReadCh(KWS_CH_RELAY));

    btn_tick();
    sess_tick();

    /* NTC temperature published to Ch9 every second */
    float tc = ntc_celsius();
    if (tc > -90.0f && tc < 990.0f)
        PubCh(KWS_CH_TEMP, (int)(tc * 100.0f));
}

#endif /* ENABLE_DRIVER_KWS303WF */
