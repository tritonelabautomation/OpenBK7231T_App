/*
 * HT7017 Energy Metering IC — Driver for KWS-303WF  v18
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * v18 CHANGES over v17
 *
 *  CHANGE 1 — Relay channel logic inverted (BUG FIX)
 *    KWS-303WF relay is ACTIVE-LOW on the display/channel signal.
 *    v17 and earlier sent setChannel=1 to turn ON, setChannel=0 to turn OFF.
 *    Reality: setChannel=0 energises relay (ON), setChannel=1 de-energises (OFF).
 *    Fix: all relay setChannel calls now send inverted value:
 *         relay ON  → setChannel <ch> 0
 *         relay OFF → setChannel <ch> 1
 *    Also fixes TFT display showing wrong relay state.
 *
 *  CHANGE 2 — Auto vehicle detection by power level
 *    Trigger: active power crosses HT7017_EV_DETECT_THRESHOLD_W (500W)
 *    Window : 30 seconds of averaged power measurement
 *    Decision: avg < HT7017_EV_SPLIT_W (4500W) → Ather Rizta Z (2W)
 *              avg >= HT7017_EV_SPLIT_W         → Tata Nexon EV 45 (4W)
 *    Session auto-starts with detected vehicle after window closes.
 *    Auto-end: power stays below threshold for HT7017_EV_END_SECONDS (60s)
 *              → session ends automatically, summary logged + MQTT published.
 *    Manual override: HT7017_Session2W / HT7017_Session4W still work.
 *    HT7017_SessionAuto  enable/disable auto-detect (default: enabled)
 *
 *  CHANGE 3 — MQTT session summary on SessionEnd
 *    Publishes JSON to topic: home/ev/session
 *    Payload example:
 *    {
 *      "vehicle":"Ather Rizta Z",
 *      "kwh":3.241,
 *      "cost_rs":22.69,
 *      "km":106.9,
 *      "saved_rs":358.9,
 *      "duration_min":200,
 *      "segments":1,
 *      "peak_w":548.0,
 *      "peak_a":2.380
 *    }
 *    Uses CMD_ExecuteCommand("publishMQTT ...") — no extra header needed.
 *    Works with existing broker config in OBK settings.
 *
 *  ALL v17 changes preserved:
 *    verbose logging flag, EV session persistence, power-cut resume,
 *    vehicle profiles (Rizta + Nexon), heap monitor, ring buffer 32 bytes
 *  ALL v15/v16 fixes preserved:
 *    FREQ fast table, hysteresis, calibration LittleFS, PF=P/(V*I)
 * ═══════════════════════════════════════════════════════════════════════════
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
#include <math.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

// ═══════════════════════════════════════════════════════════════════════════
// SECTION A — VEHICLE PROFILES  (edit when prices change)
// ═══════════════════════════════════════════════════════════════════════════

// TGSPDCL domestic — marginal rate for your 201-300 unit slab
#define EV_TARIFF_RS_PER_KWH        7.00f

// Petrol price Hyderabad
#define PETROL_RS_PER_LITRE       107.0f

// Auto-detect thresholds
#define HT7017_EV_DETECT_THRESHOLD_W  500.0f   // above this → charging started
#define HT7017_EV_SPLIT_W            4500.0f   // below=2W, above=4W
#define HT7017_EV_DETECT_WINDOW_S      30u     // seconds to average before deciding
#define HT7017_EV_END_SECONDS          60u     // seconds below threshold → session end

// MQTT
#define HT7017_MQTT_TOPIC   "home/ev/session"

typedef struct {
    const char *name;
    float km_per_kwh;           // real-world EV efficiency
    float petrol_kmpl;          // real-world mileage of replaced vehicle
    float petrol_cost_per_km;   // Rs/km petrol  = PETROL_RS_PER_LITRE / kmpl
    float ev_cost_per_km;       // Rs/km EV      = EV_TARIFF / km_per_kwh
    float saving_per_km;        // Rs/km saved
} VehicleProfile_t;

//                     name               EV eff  petrol kmpl
// Vehicle 0: Ather Rizta Z         33 km/kWh   Activa 4G  30 kmpl
// Vehicle 1: Tata Nexon EV 45       7.8 km/kWh  Nexon petrol 12 kmpl city
static const VehicleProfile_t g_vehicles[2] = {
    {
        "Ather Rizta Z", 33.0f, 30.0f,
        PETROL_RS_PER_LITRE / 30.0f,
        EV_TARIFF_RS_PER_KWH / 33.0f,
        (PETROL_RS_PER_LITRE/30.0f) - (EV_TARIFF_RS_PER_KWH/33.0f)
    },
    {
        "Tata Nexon EV 45", 7.8f, 12.0f,
        PETROL_RS_PER_LITRE / 12.0f,
        EV_TARIFF_RS_PER_KWH / 7.8f,
        (PETROL_RS_PER_LITRE/12.0f) - (EV_TARIFF_RS_PER_KWH/7.8f)
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// SECTION A2 — SCALE FACTORS AND MEASUREMENTS
// ═══════════════════════════════════════════════════════════════════════════

static float g_vScale  = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale  = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale  = HT7017_DEFAULT_FREQ_SCALE;
static float g_qScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_sScale  = HT7017_DEFAULT_APPARENT_SCALE;
static float g_e1Scale = HT7017_DEFAULT_EP1_SCALE;
static float g_iOffset = HT7017_CURRENT_OFFSET;

static uint32_t g_lastRawV = 0;
static uint32_t g_lastRawI = 0;
static uint32_t g_lastRawP = 0;

static float g_voltage      = 0.0f;
static float g_current      = 0.0f;
static float g_power        = 0.0f;
static float g_freq         = 0.0f;
static float g_reactive     = 0.0f;
static float g_apparent_s1  = 0.0f;
static float g_apparent     = 0.0f;
static float g_power_factor = 0.0f;

static uint32_t g_s1_last_update_tick = 0;
static uint32_t g_emusr_raw  = 0;

static uint32_t g_ep1_last   = 0;
static uint32_t g_eq1_last   = 0;
static bool     g_ep1_seeded = false;
static bool     g_eq1_seeded = false;
static float    g_wh_total   = 0.0f;
static float    g_varh_total = 0.0f;

static uint32_t g_session_start_unix = 0;
static uint32_t g_energy_reset_unix  = 0;

static uint32_t g_txCount     = 0;
static uint32_t g_goodFrames  = 0;
static uint32_t g_badFrames   = 0;
static uint32_t g_totalMisses = 0;

static uint8_t  g_verboseLog  = 0;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION A3 — SOFTWARE PROTECTION STATE
// ═══════════════════════════════════════════════════════════════════════════

static float    g_ovThreshold  = HT7017_DEFAULT_OV_THRESHOLD;
static float    g_uvThreshold  = HT7017_DEFAULT_UV_THRESHOLD;
static float    g_ocThreshold  = HT7017_DEFAULT_OC_THRESHOLD;
static float    g_opThreshold  = HT7017_DEFAULT_OP_THRESHOLD;
static uint8_t  g_relayChannel = HT7017_DEFAULT_RELAY_CHANNEL;
static uint8_t  g_alarmState   = 0;
static bool     g_relayTripped = false;
static uint32_t g_tripCallCount = 0;
static uint32_t g_callCount = 0;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION A4 — EV SESSION STATE
// ═══════════════════════════════════════════════════════════════════════════

#define HT7017_SESSION_FILE  "/ht7017_session.cfg"
#define HT7017_CAL_FILE      "/ht7017cal.cfg"

typedef struct {
    uint32_t session_id;
    uint32_t start_unix;
    uint32_t last_save_unix;
    uint32_t segment_count;    // 1=no cuts, 2+=power interruptions
    uint8_t  vehicle_idx;      // 0=Rizta Z, 1=Nexon EV
    float    kwh_charged;      // cumulative kWh this session
    float    wh_at_seg_start;  // g_wh_total when current segment began
    float    peak_power_w;
    float    peak_current_a;
    uint8_t  active;
} EVSession_t;

static EVSession_t g_ev;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION A5 — AUTO-DETECT STATE  (v18)
// ═══════════════════════════════════════════════════════════════════════════

typedef enum {
    AD_IDLE,        // no load, waiting for threshold crossing
    AD_DETECTING,   // power above threshold, sampling for window period
    AD_CHARGING,    // session active, monitoring for end condition
} AutoDetectState_t;

static AutoDetectState_t g_adState        = AD_IDLE;
static uint32_t          g_adWindowStart  = 0;     // callCount when window opened
static float             g_adPowerSum     = 0.0f;  // sum of power samples in window
static uint32_t          g_adSampleCount  = 0;     // number of samples taken
static uint32_t          g_adBelowCount   = 0;     // seconds below threshold (end detect)
static uint8_t           g_adEnabled      = 1;     // 1=auto-detect on, 0=manual only

// ═══════════════════════════════════════════════════════════════════════════
// SECTION B — REGISTER TABLES
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    uint8_t     reg;
    float      *target;
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint8_t     is_energy;
    uint8_t     is_emusr;
    uint32_t   *last_raw_acc;
    bool       *seeded_flag;
    float      *acc_total;
    uint32_t   *last_raw_cal;
    const char *name;
    const char *unit;
    uint8_t     obk_ch;
} RegRead_t;

#define FAST_TABLE_SIZE 4
static RegRead_t g_fastTable[FAST_TABLE_SIZE];
#define SLOW_TABLE_SIZE 5
static RegRead_t g_slowTable[SLOW_TABLE_SIZE];

static void HT7017_BuildTables(void)
{
    g_fastTable[0] = (RegRead_t){ HT7017_REG_RMS_U,    &g_voltage,    &g_vScale, 0,0,0,0, NULL,NULL,NULL, &g_lastRawV, "Voltage","V",   HT7017_CHANNEL_VOLTAGE  };
    g_fastTable[1] = (RegRead_t){ HT7017_REG_RMS_I1,   &g_current,    &g_iScale, 0,1,0,0, NULL,NULL,NULL, &g_lastRawI, "Current","A",   HT7017_CHANNEL_CURRENT  };
    g_fastTable[2] = (RegRead_t){ HT7017_REG_POWER_P1, &g_power,      &g_pScale, 1,0,0,0, NULL,NULL,NULL, &g_lastRawP, "Power",  "W",   HT7017_CHANNEL_POWER    };
    g_fastTable[3] = (RegRead_t){ HT7017_REG_FREQ,     &g_freq,       &g_fScale, 0,0,0,0, NULL,NULL,NULL, NULL,        "Freq",   "Hz",  HT7017_CHANNEL_FREQ     };

    g_slowTable[0] = (RegRead_t){ HT7017_REG_POWER_Q1, &g_reactive,   &g_qScale,  1,0,0,0, NULL,NULL,NULL,               NULL, "Reactive",     "VAR", HT7017_CHANNEL_REACTIVE };
    g_slowTable[1] = (RegRead_t){ HT7017_REG_POWER_S1, &g_apparent_s1,&g_sScale,  0,0,0,0, NULL,NULL,NULL,               NULL, "Apparent",     "VA",  HT7017_CHANNEL_APPARENT };
    g_slowTable[2] = (RegRead_t){ HT7017_REG_EP1,      &g_wh_total,   &g_e1Scale, 0,0,1,0, &g_ep1_last,&g_ep1_seeded,&g_wh_total,   NULL, "ActiveEnergy", "Wh",  HT7017_CHANNEL_WH       };
    g_slowTable[3] = (RegRead_t){ HT7017_REG_EQ1,      &g_varh_total, &g_e1Scale, 0,0,1,0, &g_eq1_last,&g_eq1_seeded,&g_varh_total, NULL, "ReactiveEnergy","VARh",HT7017_CHANNEL_VARH   };
    g_slowTable[4] = (RegRead_t){ HT7017_REG_EMUSR,    NULL,          NULL,       0,0,0,1, NULL,NULL,NULL,               NULL, "EMUSR",        "bits",HT7017_CHANNEL_EMUSR    };
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION C — SCHEDULING STATE
// ═══════════════════════════════════════════════════════════════════════════

static uint8_t g_fastIndex   = 0;
static uint8_t g_slowIndex   = 0;
static bool    g_lastWasFast = true;
static uint8_t g_fastMiss    = 0;
static uint8_t g_slowMiss    = 0;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D — HELPERS
// ═══════════════════════════════════════════════════════════════════════════

static uint8_t HT7017_Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

static float HT7017_Convert(uint32_t raw, const RegRead_t *r)
{
    if (r->reg == HT7017_REG_FREQ)
        return (raw == 0) ? 0.0f : (*r->scale / (float)raw);
    float val;
    if (r->is_signed) {
        int32_t s = (int32_t)raw;
        if (s & 0x800000) s |= (int32_t)0xFF000000;
        val = (float)s;
    } else {
        val = (float)raw;
    }
    if (r->has_offset) { val -= g_iOffset; if (val < 0.0f) val = 0.0f; }
    float result = val / (*r->scale);
    if (r->is_signed && result > -1.0f && result < 1.0f) result = 0.0f;
    return result;
}

static void HT7017_PublishChannel(uint8_t ch, float value)
{
    if (!ch) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "setChannel %u %d", ch, (int)(value * 100.0f));
    CMD_ExecuteCommand(buf, 0);
}

static void HT7017_PublishEmusr(uint8_t ch, uint32_t bitmask)
{
    if (!ch) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "setChannel %u %u", ch,
             (unsigned)(bitmask & 0x1F) * 100u);
    CMD_ExecuteCommand(buf, 0);
}

static void HT7017_PublishAlarm(uint8_t ch, uint8_t state)
{
    if (!ch) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "setChannel %u %u", ch, (unsigned)state);
    CMD_ExecuteCommand(buf, 0);
}

// ── v18 BUG FIX: relay is ACTIVE-LOW on KWS-303WF ───────────────────────
// Relay ON  → send 0  (energises coil)
// Relay OFF → send 1  (de-energises coil)
// This also corrects the TFT display which was showing inverted state.
static void HT7017_SetRelay(uint8_t on)
{
    if (!g_relayChannel) return;
    char buf[32];
    // Invert: on=1 → value=0, on=0 → value=1
    snprintf(buf, sizeof(buf), "setChannel %u %u", g_relayChannel, on ? 0u : 1u);
    CMD_ExecuteCommand(buf, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D2 — POWER FACTOR
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_UpdatePowerFactor(void)
{
    g_apparent = g_voltage * g_current;
    if (g_apparent > 0.5f && g_power > 0.0f) {
        g_power_factor = g_power / g_apparent;
        if (g_power_factor > 1.0f) g_power_factor = 1.0f;
    } else {
        g_power_factor = 0.0f;
    }
    if (HT7017_CHANNEL_PF > 0)
        HT7017_PublishChannel(HT7017_CHANNEL_PF, g_power_factor);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D3 — SOFTWARE PROTECTION WITH HYSTERESIS
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_EvaluateProtection(void)
{
    bool any = g_ovThreshold>0||g_uvThreshold>0||g_ocThreshold>0||g_opThreshold>0;
    if (!any && !g_relayTripped) return;

    if (!g_relayTripped) {
        uint8_t a = 0;
        if      (g_ovThreshold>0 && g_voltage>g_ovThreshold)                 a=1;
        else if (g_ocThreshold>0 && g_current>g_ocThreshold)                 a=3;
        else if (g_opThreshold>0 && g_power>g_opThreshold)                   a=4;
        else if (g_uvThreshold>0 && g_voltage<g_uvThreshold &&
                 g_voltage>HT7017_UV_MIN_VOLTAGE)                             a=2;
        if (a) {
            const char *n[]={"OK","OV","UV","OC","OP"};
            g_alarmState=a; g_relayTripped=true; g_tripCallCount=g_callCount;
            HT7017_SetRelay(0);   // v18: use inverted relay helper
            HT7017_PublishAlarm(HT7017_CHANNEL_ALARM, a);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: PROT TRIP [%s] V=%.2f I=%.3f P=%.1f relay OFF",
                      n[a], g_voltage, g_current, g_power);
        }
    } else {
        bool clr = false;
        switch (g_alarmState) {
            case 1: clr=g_ovThreshold>0&&g_voltage<g_ovThreshold-HT7017_PROT_HYSTERESIS_V; break;
            case 2: clr=g_uvThreshold>0&&g_voltage>g_uvThreshold+HT7017_PROT_HYSTERESIS_V; break;
            case 3: clr=g_ocThreshold>0&&g_current<g_ocThreshold-HT7017_PROT_HYSTERESIS_A; break;
            case 4: clr=g_opThreshold>0&&g_power<g_opThreshold-HT7017_PROT_HYSTERESIS_W;   break;
            default: clr=true;
        }
        if (clr && (g_callCount-g_tripCallCount)>=(uint32_t)HT7017_PROT_RECOVER_SECONDS) {
            g_relayTripped=false; g_alarmState=0;
            HT7017_SetRelay(1);   // v18: use inverted relay helper
            HT7017_PublishAlarm(HT7017_CHANNEL_ALARM, 0);
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                      "HT7017: PROT CLEAR relay ON V=%.2f I=%.3f P=%.1f",
                      g_voltage, g_current, g_power);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D4 — EMUSR PROCESSING
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_ProcessEmusr(uint32_t raw)
{
    g_emusr_raw = raw;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: [EMUSR] 0x%06X OV=%u UV=%u OC=%u OP=%u ZX=%u", raw,
              (raw&HT7017_EMUSR_BIT_OV)?1u:0u, (raw&HT7017_EMUSR_BIT_UV)?1u:0u,
              (raw&HT7017_EMUSR_BIT_OC)?1u:0u, (raw&HT7017_EMUSR_BIT_OP)?1u:0u,
              (raw&HT7017_EMUSR_BIT_ZXLOSS)?1u:0u);
    HT7017_PublishEmusr(HT7017_CHANNEL_EMUSR, raw);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D5 — CALIBRATION PERSISTENCE
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_SaveCalib(void)
{
    FILE *f = fopen(HT7017_CAL_FILE, "w");
    if (!f) { addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: SaveCalib fopen failed"); return; }
    fprintf(f, "V=%f\nI=%f\nP=%f\n", g_vScale, g_iScale, g_pScale);
    fclose(f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: SaveCalib OK V=%.4f I=%.4f P=%.6f", g_vScale, g_iScale, g_pScale);
}

static void HT7017_LoadCalib(void)
{
    FILE *f = fopen(HT7017_CAL_FILE, "r");
    if (!f) { addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: LoadCalib no file, using defaults"); return; }
    float v=0,i=0,p=0;
    int n = fscanf(f, "V=%f\nI=%f\nP=%f\n", &v, &i, &p);
    fclose(f);
    if (n!=3||v<=0||i<=0||p==0) {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: LoadCalib invalid n=%d",n); return;
    }
    g_vScale=v; g_iScale=i; g_pScale=p; g_qScale=p; g_sScale=fabsf(p);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: LoadCalib OK V=%.4f I=%.4f P=%.6f", v, i, p);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D6 — EV SESSION PERSISTENCE
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_SessionSave(void)
{
    if (!g_ev.active) return;
    g_ev.last_save_unix = NTP_GetCurrentTime();
    FILE *f = fopen(HT7017_SESSION_FILE, "w");
    if (!f) { addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: SessionSave fopen failed"); return; }
    fprintf(f,
        "id=%u\nstart=%u\nlast=%u\nsegs=%u\nveh=%u\n"
        "kwh=%.6f\nwh_seg=%.6f\npkw=%.2f\npka=%.3f\nactive=%u\n",
        g_ev.session_id, g_ev.start_unix, g_ev.last_save_unix,
        g_ev.segment_count, (unsigned)g_ev.vehicle_idx,
        g_ev.kwh_charged, g_ev.wh_at_seg_start,
        g_ev.peak_power_w, g_ev.peak_current_a, (unsigned)g_ev.active);
    fclose(f);
}

static bool HT7017_SessionLoad(void)
{
    FILE *f = fopen(HT7017_SESSION_FILE, "r");
    if (!f) return false;
    unsigned veh=0, act=0;
    int n = fscanf(f,
        "id=%u\nstart=%u\nlast=%u\nsegs=%u\nveh=%u\n"
        "kwh=%f\nwh_seg=%f\npkw=%f\npka=%f\nactive=%u\n",
        &g_ev.session_id, &g_ev.start_unix, &g_ev.last_save_unix,
        &g_ev.segment_count, &veh,
        &g_ev.kwh_charged, &g_ev.wh_at_seg_start,
        &g_ev.peak_power_w, &g_ev.peak_current_a, &act);
    fclose(f);
    if (n != 10) return false;
    g_ev.vehicle_idx = (uint8_t)(veh < 2 ? veh : 0);
    g_ev.active = (uint8_t)act;
    return true;
}

// Called on every active-energy Wh tick — accumulates kWh, saves to flash
static void HT7017_SessionUpdate(void)
{
    if (!g_ev.active) return;
    float delta_wh = g_wh_total - g_ev.wh_at_seg_start;
    if (delta_wh < 0.0f) delta_wh = 0.0f;
    g_ev.kwh_charged    += delta_wh / 1000.0f;
    g_ev.wh_at_seg_start = g_wh_total;
    if (g_power   > g_ev.peak_power_w)   g_ev.peak_power_w   = g_power;
    if (g_current > g_ev.peak_current_a) g_ev.peak_current_a = g_current;
    HT7017_SessionSave();
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D7 — MQTT PUBLISH  (v18)
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_MqttPublishSession(void)
{
    if (g_ev.vehicle_idx >= 2) return;
    const VehicleProfile_t *v = &g_vehicles[g_ev.vehicle_idx];

    uint32_t now = NTP_GetCurrentTime();
    uint32_t dur = (now > g_ev.start_unix && g_ev.start_unix > 0)
                 ? now - g_ev.start_unix : 0;

    float kwh      = g_ev.kwh_charged;
    float cost_rs  = kwh * EV_TARIFF_RS_PER_KWH;
    float km       = kwh * v->km_per_kwh;
    float saved_rs = km * v->petrol_cost_per_km - cost_rs;

    char payload[320];
    snprintf(payload, sizeof(payload),
        "{"
        "\"vehicle\":\"%s\","
        "\"kwh\":%.3f,"
        "\"cost_rs\":%.2f,"
        "\"km\":%.1f,"
        "\"saved_rs\":%.2f,"
        "\"duration_min\":%u,"
        "\"segments\":%u,"
        "\"peak_w\":%.1f,"
        "\"peak_a\":%.3f"
        "}",
        v->name, kwh, cost_rs, km, saved_rs,
        (unsigned)(dur / 60u),
        (unsigned)g_ev.segment_count,
        g_ev.peak_power_w, g_ev.peak_current_a);

    // Use OBK's built-in publishMQTT command — no extra header needed.
    // Format: publishMQTT <topic> <payload>
    // OpenBeken routes this through its existing MQTT broker connection.
    char cmd[400];
    snprintf(cmd, sizeof(cmd), "publishMQTT %s %s", HT7017_MQTT_TOPIC, payload);
    CMD_ExecuteCommand(cmd, 0);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "HT7017: MQTT published to %s", HT7017_MQTT_TOPIC);
    if (g_verboseLog)
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  payload: %s", payload);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D8 — SESSION SUMMARY PRINT
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_SessionPrint(void)
{
    if (g_ev.vehicle_idx >= 2) return;
    const VehicleProfile_t *v = &g_vehicles[g_ev.vehicle_idx];

    uint32_t now = NTP_GetCurrentTime();
    uint32_t dur = (now > g_ev.start_unix && g_ev.start_unix > 0)
                 ? now - g_ev.start_unix : 0;

    float kwh      = g_ev.kwh_charged;
    float cost_rs  = kwh * EV_TARIFF_RS_PER_KWH;
    float km       = kwh * v->km_per_kwh;
    float petrol_rs= km  * v->petrol_cost_per_km;
    float saved_rs = petrol_rs - cost_rs;

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╔══════════════════════════════════════╗");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  EV Charge Session Summary");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Vehicle  : %s", v->name);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Duration : %uh %02um %02us",
              dur/3600, (dur%3600)/60, dur%60);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Power cuts: %u  (segments: %u)",
              g_ev.segment_count>0?g_ev.segment_count-1:0, g_ev.segment_count);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "──────────────────────────────────────");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Energy charged : %.3f kWh", kwh);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Electricity cost: Rs %.2f  (@ Rs %.2f/unit)",
              cost_rs, EV_TARIFF_RS_PER_KWH);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "──────────────────────────────────────");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Range added    : %.1f km  (@ %.1f km/kWh)",
              km, v->km_per_kwh);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Same km petrol : Rs %.2f  (@ Rs %.2f/km)",
              petrol_rs, v->petrol_cost_per_km);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  EV cost/km     : Rs %.2f", v->ev_cost_per_km);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  SAVED          : Rs %.2f  (%.1f%%)",
              saved_rs, (petrol_rs>0)?(saved_rs/petrol_rs*100.0f):0.0f);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "──────────────────────────────────────");
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "  Peak power : %.1f W   Peak current: %.3f A",
              g_ev.peak_power_w, g_ev.peak_current_a);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "╚══════════════════════════════════════╝");
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION D9 — AUTO-DETECT STATE MACHINE  (v18)
// ═══════════════════════════════════════════════════════════════════════════

// Internal: start a session for a detected vehicle
static void HT7017_SessionBegin(uint8_t veh_idx);   // forward declaration
static void HT7017_SessionClose(void);               // forward declaration

static void HT7017_AutoDetectTick(void)
{
    if (!g_adEnabled) return;

    switch (g_adState) {

        case AD_IDLE:
            if (g_power >= HT7017_EV_DETECT_THRESHOLD_W) {
                // Power crossed threshold — open detection window
                g_adState       = AD_DETECTING;
                g_adWindowStart = g_callCount;
                g_adPowerSum    = g_power;
                g_adSampleCount = 1;
                g_adBelowCount  = 0;
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: AutoDetect WINDOW OPEN P=%.1fW — sampling %us",
                          g_power, HT7017_EV_DETECT_WINDOW_S);
            }
            break;

        case AD_DETECTING:
            if (g_power >= HT7017_EV_DETECT_THRESHOLD_W) {
                g_adPowerSum    += g_power;
                g_adSampleCount += 1;
            } else {
                // Power dropped during window — abort, go back to idle
                // (could be inrush spike, kettle, etc.)
                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: AutoDetect aborted — power dropped in window");
                g_adState = AD_IDLE;
                break;
            }

            // Check if window period has elapsed
            if ((g_callCount - g_adWindowStart) >= HT7017_EV_DETECT_WINDOW_S) {
                float avg = (g_adSampleCount > 0)
                          ? g_adPowerSum / (float)g_adSampleCount : 0.0f;
                uint8_t veh = (avg < HT7017_EV_SPLIT_W) ? 0u : 1u;

                addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                          "HT7017: AutoDetect RESULT avg=%.1fW → %s",
                          avg, g_vehicles[veh].name);

                HT7017_SessionBegin(veh);
                g_adState      = AD_CHARGING;
                g_adBelowCount = 0;
            }
            break;

        case AD_CHARGING:
            if (!g_ev.active) {
                // Session was manually ended — go back to idle
                g_adState = AD_IDLE;
                break;
            }

            if (g_power < HT7017_EV_DETECT_THRESHOLD_W) {
                g_adBelowCount++;
                if (g_verboseLog)
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: AutoDetect low power %us/%us",
                              g_adBelowCount, HT7017_EV_END_SECONDS);

                if (g_adBelowCount >= HT7017_EV_END_SECONDS) {
                    // Charging has stopped — auto-end session
                    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                              "HT7017: AutoDetect END — power below %.0fW for %us",
                              HT7017_EV_DETECT_THRESHOLD_W, HT7017_EV_END_SECONDS);
                    HT7017_SessionClose();
                    g_adState = AD_IDLE;
                }
            } else {
                // Still charging — reset end counter
                g_adBelowCount = 0;
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION E — ENERGY ACCUMULATOR
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_AccumulateEnergy(uint32_t raw, const RegRead_t *r)
{
    if (!(*r->seeded_flag)) {
        *r->last_raw_acc = raw;
        *r->seeded_flag  = true;
        if (g_verboseLog)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: [%s] seeded raw=%u",r->name,raw);
        return;
    }

    uint32_t delta;
    if (raw >= *r->last_raw_acc) {
        delta = raw - *r->last_raw_acc;
    } else {
        delta = (0x00FFFFFF - *r->last_raw_acc) + raw + 1;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: [%s] wrap last=%u new=%u delta=%u",
                  r->name, *r->last_raw_acc, raw, delta);
    }
    *r->last_raw_acc = raw;
    if (!delta) return;

    float inc = (float)delta / (*r->scale);
    *r->acc_total += inc;

    if (g_verboseLog)
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: [%s] +%.5f %s total=%.4f",
                  r->name, inc, r->unit, *r->acc_total);

    if (r->obk_ch) {
        char buf[48];
        snprintf(buf, sizeof(buf), "setChannel %u %d",
                 r->obk_ch, (int)(*r->acc_total * 1000.0f));
        CMD_ExecuteCommand(buf, 0);
    }

    // Update EV session on active-energy tick only
    if (r->reg == HT7017_REG_EP1)
        HT7017_SessionUpdate();
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION F — FRAME PROCESSING
// ═══════════════════════════════════════════════════════════════════════════

static void HT7017_ProcessFrame(uint8_t d2, uint8_t d1, uint8_t d0, uint8_t cs,
                                 const RegRead_t *r)
{
    uint8_t exp = HT7017_Checksum(r->reg, d2, d1, d0);
    if (cs != exp) {
        g_badFrames++;
        if (g_verboseLog)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                      "HT7017: BAD CS reg=0x%02X got=0x%02X exp=0x%02X bad=%u",
                      r->reg, cs, exp, g_badFrames);
        return;
    }

    uint32_t raw = ((uint32_t)d2<<16)|((uint32_t)d1<<8)|d0;
    g_goodFrames++;
    if (r->last_raw_cal) *r->last_raw_cal = raw;

    if (r->is_emusr) { HT7017_ProcessEmusr(raw); return; }

    if (r->is_energy) {
        HT7017_AccumulateEnergy(raw, r);
    } else {
        if (r->reg == HT7017_REG_POWER_S1) g_s1_last_update_tick = g_callCount;
        float value = HT7017_Convert(raw, r);
        *r->target = value;
        if (g_verboseLog)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: [%s] = %.3f %s raw=%u",
                      r->name, value, r->unit, raw);
        if (r->obk_ch) HT7017_PublishChannel(r->obk_ch, value);
    }

    HT7017_UpdatePowerFactor();
    HT7017_EvaluateProtection();
    HT7017_AutoDetectTick();   // v18: run auto-detect every frame

    if (!g_session_start_unix) {
        uint32_t now = NTP_GetCurrentTime();
        if (now > 1000000000u) { g_session_start_unix=now; g_energy_reset_unix=now; }
    }

    // Heap monitor every 5 minutes
    if (g_callCount % 300 == 0)
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "HT7017: heap=%u good=%u bad=%u miss=%u adState=%u",
                  (unsigned)xPortGetFreeHeapSize(),
                  g_goodFrames, g_badFrames, g_totalMisses,
                  (unsigned)g_adState);
}

static void HT7017_SendRequest(uint8_t reg)
{
    UART_ConsumeBytes(UART_GetDataSize());
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION G — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════

// ── Calibration ─────────────────────────────────────────────────────────────

static commandResult_t CMD_VoltageSet(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!args||!*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float a=(float)atof(args); if(a<=0) return CMD_RES_BAD_ARGUMENT;
    if(!g_lastRawV){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: VoltageSet no reading yet");return CMD_RES_ERROR;}
    float old=g_vScale; g_vScale=(float)g_lastRawV/a; g_voltage=a;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: VoltageSet %.2fV scale %.2f->%.2f",a,old,g_vScale);
    HT7017_SaveCalib(); return CMD_RES_OK;
}

static commandResult_t CMD_CurrentSet(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!args||!*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float a=(float)atof(args); if(a<=0) return CMD_RES_BAD_ARGUMENT;
    if(!g_lastRawI){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: CurrentSet no reading yet");return CMD_RES_ERROR;}
    float net=(float)g_lastRawI-g_iOffset; if(net<=0) return CMD_RES_ERROR;
    float old=g_iScale; g_iScale=net/a; g_current=a;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: CurrentSet %.3fA scale %.2f->%.2f",a,old,g_iScale);
    HT7017_SaveCalib(); return CMD_RES_OK;
}

static commandResult_t CMD_PowerSet(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!args||!*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float a=(float)atof(args); if(a<=0) return CMD_RES_BAD_ARGUMENT;
    if(!g_lastRawP){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: PowerSet no reading yet");return CMD_RES_ERROR;}
    int32_t s=(int32_t)g_lastRawP; if(s&0x800000) s|=(int32_t)0xFF000000;
    if(!s) return CMD_RES_ERROR;
    float old=g_pScale; g_pScale=(float)s/a; g_power=a;
    g_qScale=g_pScale; g_sScale=fabsf(g_pScale);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: PowerSet %.1fW scale %.4f->%.4f",a,old,g_pScale);
    HT7017_SaveCalib(); return CMD_RES_OK;
}

static commandResult_t CMD_SaveCalib(const void *ctx,const char *cmd,const char *args,int flags)
{ HT7017_SaveCalib(); return CMD_RES_OK; }

static commandResult_t CMD_LoadCalib(const void *ctx,const char *cmd,const char *args,int flags)
{ HT7017_LoadCalib(); return CMD_RES_OK; }

// ── Energy / Status ──────────────────────────────────────────────────────────

static commandResult_t CMD_EnergyReset(const void *ctx,const char *cmd,const char *args,int flags)
{
    g_wh_total=g_varh_total=0; g_ep1_seeded=g_eq1_seeded=false;
    uint32_t now=NTP_GetCurrentTime(); g_energy_reset_unix=(now>1000000000u)?now:0;
    HT7017_PublishChannel(HT7017_CHANNEL_WH,0); HT7017_PublishChannel(HT7017_CHANNEL_VARH,0);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: Energy reset unix=%u",g_energy_reset_unix);
    return CMD_RES_OK;
}

static commandResult_t CMD_Energy(const void *ctx,const char *cmd,const char *args,int flags)
{
    uint32_t since=(g_energy_reset_unix>0)?g_energy_reset_unix:g_session_start_unix;
    uint32_t now=NTP_GetCurrentTime(), e=(now>since&&since)?now-since:0;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"╔══ Energy ══╗");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Active   : %.4f Wh  (%.5f kWh)",g_wh_total,g_wh_total/1000.0f);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Reactive : %.4f VARh",g_varh_total);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Duration : %uh %02um",e/3600,(e%3600)/60);
    if(e>0) addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Avg Pwr  : %.1f W",(g_wh_total/(e/3600.0f)));
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"╚════════════╝");
    return CMD_RES_OK;
}

static commandResult_t CMD_Status(const void *ctx,const char *cmd,const char *args,int flags)
{
    const char *an[]={"OK","OV","UV","OC","OP"};
    const char *ads[]={"IDLE","DETECTING","CHARGING"};
    const char *vname=(g_ev.vehicle_idx<2)?g_vehicles[g_ev.vehicle_idx].name:"none";
    uint32_t s1a=(g_callCount>g_s1_last_update_tick)?(g_callCount-g_s1_last_update_tick):0;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"╔══ HT7017 v18 ══╗");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  V=%.2fV I=%.3fA F=%.3fHz",g_voltage,g_current,g_freq);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  P=%.2fW PF=%.4f S=%.2fVA(V*I)",g_power,g_power_factor,g_apparent);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  S1=%.2fVA(age %us) Q=%.2fVAR",g_apparent_s1,s1a,g_reactive);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Wh=%.4f VARh=%.4f",g_wh_total,g_varh_total);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  EMUSR=0x%02X Alarm=%s %s",g_emusr_raw,an[g_alarmState],g_relayTripped?"[TRIPPED]":"[OK]");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  AutoDetect=%s state=%s belowCnt=%u",
              g_adEnabled?"ON":"OFF",ads[g_adState],g_adBelowCount);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Session=%s veh=%s segs=%u kWh=%.3f",
              g_ev.active?"ACTIVE":"IDLE",vname,g_ev.segment_count,g_ev.kwh_charged);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  Heap=%u verbose=%s good=%u bad=%u miss=%u",
              (unsigned)xPortGetFreeHeapSize(),g_verboseLog?"ON":"OFF",
              g_goodFrames,g_badFrames,g_totalMisses);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"╚════════════════╝");
    return CMD_RES_OK;
}

// ── UART / Protection ────────────────────────────────────────────────────────

static commandResult_t CMD_Baud(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!args||!*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int b=atoi(args); if(b<=0) return CMD_RES_BAD_ARGUMENT;
    UART_InitUART(b,HT7017_PARITY_EVEN,0);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: UART %d 8E1",b); return CMD_RES_OK;
}

static commandResult_t CMD_NoParity(const void *ctx,const char *cmd,const char *args,int flags)
{ UART_InitUART(HT7017_BAUD_RATE,0,0); addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: UART %d 8N1",HT7017_BAUD_RATE); return CMD_RES_OK; }

static commandResult_t CMD_SetOV(const void *ctx,const char *cmd,const char *args,int flags)
{ if(!args||!*args)return CMD_RES_NOT_ENOUGH_ARGUMENTS; float v=(float)atof(args); if(v<0)return CMD_RES_BAD_ARGUMENT; g_ovThreshold=v; addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: OV=%.1f%s",v,v==0?" (off)":""); return CMD_RES_OK; }

static commandResult_t CMD_SetUV(const void *ctx,const char *cmd,const char *args,int flags)
{ if(!args||!*args)return CMD_RES_NOT_ENOUGH_ARGUMENTS; float v=(float)atof(args); if(v<0)return CMD_RES_BAD_ARGUMENT; g_uvThreshold=v; addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: UV=%.1f%s",v,v==0?" (off)":""); return CMD_RES_OK; }

static commandResult_t CMD_SetOC(const void *ctx,const char *cmd,const char *args,int flags)
{ if(!args||!*args)return CMD_RES_NOT_ENOUGH_ARGUMENTS; float a=(float)atof(args); if(a<0)return CMD_RES_BAD_ARGUMENT; g_ocThreshold=a; addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: OC=%.3f%s",a,a==0?" (off)":""); return CMD_RES_OK; }

static commandResult_t CMD_SetOP(const void *ctx,const char *cmd,const char *args,int flags)
{ if(!args||!*args)return CMD_RES_NOT_ENOUGH_ARGUMENTS; float w=(float)atof(args); if(w<0)return CMD_RES_BAD_ARGUMENT; g_opThreshold=w; addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: OP=%.1f%s",w,w==0?" (off)":""); return CMD_RES_OK; }

static commandResult_t CMD_SetRelayChannel(const void *ctx,const char *cmd,const char *args,int flags)
{ if(!args||!*args)return CMD_RES_NOT_ENOUGH_ARGUMENTS; int c=atoi(args); if(c<=0||c>32)return CMD_RES_BAD_ARGUMENT; g_relayChannel=(uint8_t)c; addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: relay ch=%u (active-low fixed)",g_relayChannel); return CMD_RES_OK; }

static commandResult_t CMD_ProtStatus(const void *ctx,const char *cmd,const char *args,int flags)
{
    const char *n[]={"OK","OV","UV","OC","OP"};
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017 Prot: %s relay=%u %s (active-low)",
              n[g_alarmState],g_relayChannel,g_relayTripped?"[TRIPPED]":"[OK]");
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  OV=%.1f UV=%.1f OC=%.3f OP=%.1f",
              g_ovThreshold,g_uvThreshold,g_ocThreshold,g_opThreshold);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"  V=%.2f I=%.3f P=%.1f",g_voltage,g_current,g_power);
    return CMD_RES_OK;
}

// ── Verbose ──────────────────────────────────────────────────────────────────

static commandResult_t CMD_Verbose(const void *ctx,const char *cmd,const char *args,int flags)
{
    g_verboseLog=!g_verboseLog;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: verbose %s",g_verboseLog?"ON":"OFF");
    return CMD_RES_OK;
}

// ── EV Session commands ──────────────────────────────────────────────────────

// Internal: start session (used by both manual commands and auto-detect)
static void HT7017_SessionBegin(uint8_t veh_idx)
{
    uint32_t now = NTP_GetCurrentTime();
    memset(&g_ev, 0, sizeof(g_ev));
    g_ev.session_id      = (now>1000000000u)?now:g_callCount;
    g_ev.start_unix      = g_ev.session_id;
    g_ev.segment_count   = 1;
    g_ev.vehicle_idx     = veh_idx;
    g_ev.wh_at_seg_start = g_wh_total;
    g_ev.active          = 1;
    HT7017_SessionSave();
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "HT7017: Session STARTED id=%u vehicle=%s",
              g_ev.session_id, g_vehicles[veh_idx].name);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  EV Rs%.2f/km  |  Petrol Rs%.2f/km  |  Saves Rs%.2f/km",
              g_vehicles[veh_idx].ev_cost_per_km,
              g_vehicles[veh_idx].petrol_cost_per_km,
              g_vehicles[veh_idx].saving_per_km);
}

// Internal: end session (used by both manual command and auto-detect)
static void HT7017_SessionClose(void)
{
    if (!g_ev.active) return;
    HT7017_SessionPrint();
    HT7017_MqttPublishSession();   // v18: publish to HA
    g_ev.active = 0;
    HT7017_SessionSave();
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: Session ENDED");
}

static commandResult_t CMD_Session2W(const void *ctx,const char *cmd,const char *args,int flags)
{ HT7017_SessionBegin(0); g_adState=AD_CHARGING; g_adBelowCount=0; return CMD_RES_OK; }

static commandResult_t CMD_Session4W(const void *ctx,const char *cmd,const char *args,int flags)
{ HT7017_SessionBegin(1); g_adState=AD_CHARGING; g_adBelowCount=0; return CMD_RES_OK; }

static commandResult_t CMD_SessionEnd(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!g_ev.active){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: No active session");return CMD_RES_OK;}
    HT7017_SessionClose(); g_adState=AD_IDLE; return CMD_RES_OK;
}

static commandResult_t CMD_SessionStatus(const void *ctx,const char *cmd,const char *args,int flags)
{
    if(!g_ev.active){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: No active session");return CMD_RES_OK;}
    HT7017_SessionPrint(); return CMD_RES_OK;
}

static commandResult_t CMD_SessionClear(const void *ctx,const char *cmd,const char *args,int flags)
{
    memset(&g_ev,0,sizeof(g_ev)); g_adState=AD_IDLE;
    FILE *f=fopen(HT7017_SESSION_FILE,"w"); if(f) fclose(f);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: Session cleared"); return CMD_RES_OK;
}

static commandResult_t CMD_SessionAuto(const void *ctx,const char *cmd,const char *args,int flags)
{
    g_adEnabled=!g_adEnabled;
    if(!g_adEnabled) g_adState=AD_IDLE;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "HT7017: AutoDetect %s (threshold=%.0fW window=%us split=%.0fW end=%us)",
              g_adEnabled?"ON":"OFF",
              HT7017_EV_DETECT_THRESHOLD_W, HT7017_EV_DETECT_WINDOW_S,
              HT7017_EV_SPLIT_W, HT7017_EV_END_SECONDS);
    return CMD_RES_OK;
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION H — INIT
// ═══════════════════════════════════════════════════════════════════════════

void HT7017_Init(void)
{
    HT7017_BuildTables();

    g_fastIndex=FAST_TABLE_SIZE-1; g_slowIndex=SLOW_TABLE_SIZE-1;
    g_callCount=g_txCount=g_goodFrames=g_badFrames=g_totalMisses=0;
    g_lastWasFast=true; g_fastMiss=g_slowMiss=0;
    g_emusr_raw=g_alarmState=0; g_relayTripped=false;
    g_tripCallCount=0; g_s1_last_update_tick=0; g_verboseLog=0;
    memset(&g_ev,0,sizeof(g_ev));
    g_adState=AD_IDLE; g_adEnabled=1; g_adBelowCount=0;
    g_adWindowStart=0; g_adPowerSum=0; g_adSampleCount=0;

    UART_InitReceiveRingBuffer(32);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("VoltageSet",             CMD_VoltageSet,      NULL);
    CMD_RegisterCommand("CurrentSet",             CMD_CurrentSet,      NULL);
    CMD_RegisterCommand("PowerSet",               CMD_PowerSet,        NULL);
    CMD_RegisterCommand("HT7017_Energy_Reset",    CMD_EnergyReset,     NULL);
    CMD_RegisterCommand("HT7017_Energy",          CMD_Energy,          NULL);
    CMD_RegisterCommand("HT7017_Status",          CMD_Status,          NULL);
    CMD_RegisterCommand("HT7017_Baud",            CMD_Baud,            NULL);
    CMD_RegisterCommand("HT7017_NoParity",        CMD_NoParity,        NULL);
    CMD_RegisterCommand("HT7017_SetOV",           CMD_SetOV,           NULL);
    CMD_RegisterCommand("HT7017_SetUV",           CMD_SetUV,           NULL);
    CMD_RegisterCommand("HT7017_SetOC",           CMD_SetOC,           NULL);
    CMD_RegisterCommand("HT7017_SetOP",           CMD_SetOP,           NULL);
    CMD_RegisterCommand("HT7017_SetRelayChannel", CMD_SetRelayChannel, NULL);
    CMD_RegisterCommand("HT7017_ProtStatus",      CMD_ProtStatus,      NULL);
    CMD_RegisterCommand("HT7017_SaveCalib",       CMD_SaveCalib,       NULL);
    CMD_RegisterCommand("HT7017_LoadCalib",       CMD_LoadCalib,       NULL);
    CMD_RegisterCommand("HT7017_Verbose",         CMD_Verbose,         NULL);
    CMD_RegisterCommand("HT7017_Session2W",       CMD_Session2W,       NULL);
    CMD_RegisterCommand("HT7017_Session4W",       CMD_Session4W,       NULL);
    CMD_RegisterCommand("HT7017_SessionEnd",      CMD_SessionEnd,      NULL);
    CMD_RegisterCommand("HT7017_SessionStatus",   CMD_SessionStatus,   NULL);
    CMD_RegisterCommand("HT7017_SessionClear",    CMD_SessionClear,    NULL);
    CMD_RegisterCommand("HT7017_SessionAuto",     CMD_SessionAuto,     NULL);

    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "HT7017 v18: KWS-303WF 4800 8E1 | relay=ACTIVE-LOW-FIXED | "
              "AutoDetect=ON | MQTT=%s", HT7017_MQTT_TOPIC);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  2W: %s %.1fkm/kWh  saves Rs%.2f/km",
              g_vehicles[0].name, g_vehicles[0].km_per_kwh, g_vehicles[0].saving_per_km);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  4W: %s %.1fkm/kWh  saves Rs%.2f/km",
              g_vehicles[1].name, g_vehicles[1].km_per_kwh, g_vehicles[1].saving_per_km);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
              "  AutoDetect: >%.0fW for %us → vehicle, <%.0fW for %us → end",
              HT7017_EV_DETECT_THRESHOLD_W, HT7017_EV_DETECT_WINDOW_S,
              HT7017_EV_DETECT_THRESHOLD_W, HT7017_EV_END_SECONDS);

    HT7017_LoadCalib();

    // Auto-resume interrupted session on power cut reboot
    if (HT7017_SessionLoad() && g_ev.active) {
        g_ev.segment_count++;
        g_ev.wh_at_seg_start = g_wh_total;
        g_adState = AD_CHARGING;   // re-enter charging state for auto-end detection
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                  "HT7017: Session RESUMED id=%u veh=%s segs=%u kWh=%.3f",
                  g_ev.session_id,
                  (g_ev.vehicle_idx<2)?g_vehicles[g_ev.vehicle_idx].name:"?",
                  g_ev.segment_count, g_ev.kwh_charged);
    } else {
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: No active session on boot");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION I — RUN EVERY SECOND
// ═══════════════════════════════════════════════════════════════════════════

void HT7017_RunEverySecond(void)
{
    int avail = UART_GetDataSize();
    if (avail >= HT7017_RESPONSE_LEN) {
        uint8_t d2=UART_GetByte(0),d1=UART_GetByte(1),
                d0=UART_GetByte(2),cs=UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);
        if (g_lastWasFast) {
            HT7017_ProcessFrame(d2,d1,d0,cs,&g_fastTable[g_fastIndex]); g_fastMiss=0;
        } else {
            HT7017_ProcessFrame(d2,d1,d0,cs,&g_slowTable[g_slowIndex]); g_slowMiss=0;
        }
    } else {
        g_totalMisses++;
        if (g_txCount>0) {
            if (g_lastWasFast) {
                if(++g_fastMiss>=HT7017_MAX_MISS_COUNT){
                    if(g_verboseLog) addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: FAST skip [%s]",g_fastTable[g_fastIndex].name);
                    g_fastIndex=(g_fastIndex+1)%FAST_TABLE_SIZE; g_fastMiss=0;
                }
            } else {
                if(++g_slowMiss>=HT7017_MAX_MISS_COUNT){
                    if(g_verboseLog) addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: SLOW skip [%s]",g_slowTable[g_slowIndex].name);
                    g_slowIndex=(g_slowIndex+1)%SLOW_TABLE_SIZE; g_slowMiss=0;
                }
            }
        }
        UART_ConsumeBytes(UART_GetDataSize());
    }

    g_callCount++;

    if (g_callCount%HT7017_FAST_RATIO==0) {
        g_slowIndex=(g_slowIndex+1)%SLOW_TABLE_SIZE; g_lastWasFast=false;
        uint8_t reg=g_slowTable[g_slowIndex].reg; HT7017_SendRequest(reg);
        if(g_verboseLog) addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: TX SLOW 0x%02X [%s]",reg,g_slowTable[g_slowIndex].name);
    } else {
        g_fastIndex=(g_fastIndex+1)%FAST_TABLE_SIZE; g_lastWasFast=true;
        uint8_t reg=g_fastTable[g_fastIndex].reg; HT7017_SendRequest(reg);
        if(g_verboseLog) addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017: TX FAST 0x%02X [%s]",reg,g_fastTable[g_fastIndex].name);
    }
}

void HT7017_RunQuick(void)
{
    if (UART_GetDataSize()<HT7017_RESPONSE_LEN) return;
    uint8_t d2=UART_GetByte(0),d1=UART_GetByte(1),
            d0=UART_GetByte(2),cs=UART_GetByte(3);
    UART_ConsumeBytes(HT7017_RESPONSE_LEN);
    if (g_lastWasFast) HT7017_ProcessFrame(d2,d1,d0,cs,&g_fastTable[g_fastIndex]);
    else               HT7017_ProcessFrame(d2,d1,d0,cs,&g_slowTable[g_slowIndex]);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION J — HTTP PAGE
// ═══════════════════════════════════════════════════════════════════════════

void HT7017_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    uint32_t now   = NTP_GetCurrentTime();
    uint32_t since = (g_energy_reset_unix>0)?g_energy_reset_unix:g_session_start_unix;
    uint32_t e     = (now>since&&since)?now-since:0;
    const char *ac  = g_alarmState?"#c00":"#000";
    const char *as_ = g_alarmState?"FAULT":"OK";
    const char *ads[]={"IDLE","DETECTING...","CHARGING"};

    float kwh=g_ev.kwh_charged, cost_rs=0, km=0, saved_rs=0;
    const char *vname="none";
    if (g_ev.active && g_ev.vehicle_idx<2) {
        const VehicleProfile_t *v=&g_vehicles[g_ev.vehicle_idx];
        vname=v->name; cost_rs=kwh*EV_TARIFF_RS_PER_KWH;
        km=kwh*v->km_per_kwh; saved_rs=km*v->petrol_cost_per_km-cost_rs;
    }

    char tmp[920];
    snprintf(tmp, sizeof(tmp),
        "<h5>HT7017 v18 KWS-303WF</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse;font-size:0.9em'>"
        "<tr><th>Parameter</th><th>Value</th></tr>"
        "<tr><td>V / I / Hz</td><td><b>%.2fV / %.3fA / %.2fHz</b></td></tr>"
        "<tr><td>Active Power</td><td><b>%.2f W</b></td></tr>"
        "<tr><td>Power Factor</td><td><b>%.4f</b></td></tr>"
        "<tr><td>Meter Energy</td><td><b>%.4f Wh</b></td></tr>"
        "<tr><td>Protection</td><td><b style='color:%s'>%s relay ch=%u</b></td></tr>"
        "<tr><td>Auto-Detect</td><td>%s — %s</td></tr>"
        "<tr bgcolor='#eaffea'><td><b>EV Session</b></td><td><b>%s — %s</b></td></tr>"
        "<tr bgcolor='#eaffea'><td>Energy charged</td><td><b>%.3f kWh</b></td></tr>"
        "<tr bgcolor='#eaffea'><td>Electricity cost</td><td><b>Rs %.2f</b></td></tr>"
        "<tr bgcolor='#eaffea'><td>Range added</td><td><b>%.1f km</b></td></tr>"
        "<tr bgcolor='#eaffea'><td>Petrol saving</td><td><b>Rs %.2f</b></td></tr>"
        "<tr bgcolor='#eaffea'><td>Power cuts</td><td>%u</td></tr>"
        "<tr><td>Uptime</td><td>%uh %02um</td></tr>"
        "<tr><td colspan='2' style='font-size:0.75em;color:#666'>"
          "good=%u bad=%u | heap=%u | verbose=%s | MQTT=%s</td></tr>"
        "</table>",
        g_voltage,g_current,g_freq, g_power, g_power_factor, g_wh_total,
        ac,as_,g_relayChannel,
        g_adEnabled?"ON":"OFF", ads[g_adState],
        g_ev.active?"ACTIVE":"IDLE", vname,
        kwh, cost_rs, km, saved_rs,
        g_ev.segment_count>0?g_ev.segment_count-1:0,
        e/3600,(e%3600)/60,
        g_goodFrames,g_badFrames,
        (unsigned)xPortGetFreeHeapSize(), g_verboseLog?"ON":"OFF",
        HT7017_MQTT_TOPIC);

    strncat(request->reply, tmp, sizeof(request->reply)-strlen(request->reply)-1);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION K — GETTERS
// ═══════════════════════════════════════════════════════════════════════════

float    HT7017_GetVoltage(void)        { return g_voltage;       }
float    HT7017_GetCurrent(void)        { return g_current;       }
float    HT7017_GetPower(void)          { return g_power;         }
float    HT7017_GetFrequency(void)      { return g_freq;          }
float    HT7017_GetPowerFactor(void)    { return g_power_factor;  }
float    HT7017_GetApparentPower(void)  { return g_apparent;      }
uint32_t HT7017_GetGoodFrames(void)     { return g_goodFrames;    }
uint32_t HT7017_GetBadFrames(void)      { return g_badFrames;     }
uint32_t HT7017_GetTxCount(void)        { return g_txCount;       }
uint32_t HT7017_GetMissCount(void)      { return g_totalMisses;   }
float    HT7017_GetReactivePower(void)  { return g_reactive;      }
float    HT7017_GetApparentS1(void)     { return g_apparent_s1;   }
float    HT7017_GetWh(void)             { return g_wh_total;      }
float    HT7017_GetVARh(void)           { return g_varh_total;    }
uint32_t HT7017_GetEmusr(void)          { return g_emusr_raw;     }
uint8_t  HT7017_GetAlarmState(void)     { return g_alarmState;    }

#endif // ENABLE_DRIVER_HT7017
