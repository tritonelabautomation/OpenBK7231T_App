/*
 * drv_ht7017.c — HT7017 Energy Metering IC Driver for OpenBK7231N / KWS-303WF
 *
 * SINGLE RESPONSIBILITY: UART poll HT7017 → calibrate → publish to OBK channels.
 * NO relay GPIO. NO EV session. NO buttons. NO display. NO MQTT.
 *
 * CHANNEL OUTPUT MAP:
 *   Ch 1 = Voltage      (V  × 100   integer)  e.g. 22150 = 221.50 V
 *   Ch 2 = Current      (A  × 1000  integer)  e.g.   550 =   0.550 A
 *   Ch 3 = Power        (W  × 10    integer)  e.g.  1217 = 121.7 W
 *   Ch 4 = Frequency    (Hz × 100   integer)  e.g.  4990 =  49.90 Hz
 *   Ch 5 = Power Factor (PF × 1000  integer)  e.g.   980 =   0.980
 *   Ch 6 = Energy       (Wh × 10    integer)  e.g.  4700 = 470.0 Wh
 *   Ch 7 = Alarm        (0=OK 1=OV 2=UV 3=OC 4=OP)
 *
 * RELAY: Not touched here. autoexec.bat AddChangeHandler reacts to Ch7
 *        and writes Ch8. drv_kws303wf.c owns P7/P8 GPIO exclusively.
 *
 * HARDWARE: BK7231N, HT7017 SSOP-16, UART 4800 8E1
 *   Request:  0xA5 <reg>
 *   Response: D2 D1 D0 CS  (CS = ~(0xA5+reg+D2+D1+D0))
 *
 * CALIBRATION: VoltageSet → CurrentSet → PowerSet (in order)
 *   Use 1kW resistive load. Saved to /ht7017cal.cfg.
 *
 * startDriver HT7017
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

/* ═══════════════════════════════════════════════════════════
 * A — CHANNEL ASSIGNMENTS
 * ═══════════════════════════════════════════════════════════ */
#define HT_CH_VOLTAGE   1
#define HT_CH_CURRENT   2
#define HT_CH_POWER     3
#define HT_CH_FREQ      4
#define HT_CH_PF        5
#define HT_CH_ENERGY    6
#define HT_CH_ALARM     7

/* ═══════════════════════════════════════════════════════════
 * B — CALIBRATION SCALES
 * ═══════════════════════════════════════════════════════════ */
static float g_vScale  = HT7017_DEFAULT_VOLTAGE_SCALE;
static float g_iScale  = HT7017_DEFAULT_CURRENT_SCALE;
static float g_pScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_fScale  = HT7017_DEFAULT_FREQ_SCALE;
static float g_qScale  = HT7017_DEFAULT_POWER_SCALE;
static float g_sScale  = HT7017_DEFAULT_APPARENT_SCALE;
static float g_e1Scale = HT7017_DEFAULT_EP1_SCALE;
static float g_iOffset = HT7017_CURRENT_OFFSET;
#define HT7017_CAL_FILE "/ht7017cal.cfg"

/* Last raw readings captured for calibration commands */
static uint32_t g_rawV = 0, g_rawI = 0, g_rawP = 0;

/* ═══════════════════════════════════════════════════════════
 * C — LIVE MEASUREMENTS
 * ═══════════════════════════════════════════════════════════ */
static float    g_voltage      = 0.0f;
static float    g_current      = 0.0f;
static float    g_power        = 0.0f;
static float    g_freq         = 0.0f;
static float    g_reactive     = 0.0f;
static float    g_apparent_s1  = 0.0f;
static float    g_apparent     = 0.0f;
static float    g_pf           = 0.0f;
static float    g_wh           = 0.0f;
static float    g_varh         = 0.0f;
static uint32_t g_emusr        = 0;
static uint32_t g_ep1_last     = 0;
static uint32_t g_eq1_last     = 0;
static bool     g_ep1_seeded   = false;
static bool     g_eq1_seeded   = false;
static uint32_t g_s1_tick      = 0;

/* ═══════════════════════════════════════════════════════════
 * D — PROTECTION STATE
 * ═══════════════════════════════════════════════════════════ */
static float    g_ovThr = HT7017_DEFAULT_OV_THRESHOLD;
static float    g_uvThr = HT7017_DEFAULT_UV_THRESHOLD;
static float    g_ocThr = HT7017_DEFAULT_OC_THRESHOLD;
static float    g_opThr = HT7017_DEFAULT_OP_THRESHOLD;
static uint8_t  g_alarm = 0;
static bool     g_tripped = false;
static uint32_t g_tripAt  = 0;

/* ═══════════════════════════════════════════════════════════
 * E — DRIVER STATE
 * ═══════════════════════════════════════════════════════════ */
static uint32_t g_tick      = 0;
static uint32_t g_txCount   = 0;
static uint32_t g_goodFr    = 0;
static uint32_t g_badFr     = 0;
static uint32_t g_miss      = 0;
static uint8_t  g_verbose   = 0;

/* ═══════════════════════════════════════════════════════════
 * F — REGISTER TABLE
 * ═══════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t     reg;
    float      *target;
    float      *scale;
    uint8_t     is_signed;
    uint8_t     has_offset;
    uint8_t     is_energy;
    uint8_t     is_emusr;
    uint32_t   *last_raw_acc;
    bool       *seeded;
    float      *acc;
    uint32_t   *raw_cal;   /* pointer to capture raw for calibration */
    const char *name;
    const char *unit;
    uint8_t     ch;        /* OBK channel to publish (0=skip) */
    float       ch_scale;  /* phys × ch_scale = integer */
} Reg_t;

#define FAST_N 4
#define SLOW_N 5
static Reg_t g_fast[FAST_N];
static Reg_t g_slow[SLOW_N];
static uint8_t g_fi = 0, g_si = 0;
static bool    g_wasFast = true;
static uint8_t g_fmiss = 0, g_smiss = 0;

static void BuildTables(void)
{
    g_fast[0]=(Reg_t){HT7017_REG_RMS_U,    &g_voltage,   &g_vScale, 0,0,0,0,NULL,NULL,NULL,&g_rawV,"V",  "V",  HT_CH_VOLTAGE, 100.0f};
    g_fast[1]=(Reg_t){HT7017_REG_RMS_I1,   &g_current,   &g_iScale, 0,1,0,0,NULL,NULL,NULL,&g_rawI,"I",  "A",  HT_CH_CURRENT,1000.0f};
    g_fast[2]=(Reg_t){HT7017_REG_POWER_P1, &g_power,     &g_pScale, 1,0,0,0,NULL,NULL,NULL,&g_rawP,"P",  "W",  HT_CH_POWER,    10.0f};
    g_fast[3]=(Reg_t){HT7017_REG_FREQ,     &g_freq,      &g_fScale, 0,0,0,0,NULL,NULL,NULL,NULL,   "Hz", "Hz", HT_CH_FREQ,    100.0f};
    g_slow[0]=(Reg_t){HT7017_REG_POWER_Q1, &g_reactive,  &g_qScale, 1,0,0,0,NULL,NULL,NULL,NULL,   "Q",  "VAR",0,              0.0f};
    g_slow[1]=(Reg_t){HT7017_REG_POWER_S1, &g_apparent_s1,&g_sScale,0,0,0,0,NULL,NULL,NULL,NULL,   "S1", "VA", 0,              0.0f};
    g_slow[2]=(Reg_t){HT7017_REG_EP1,      &g_wh,        &g_e1Scale,0,0,1,0,&g_ep1_last,&g_ep1_seeded,&g_wh, NULL,"EP1","Wh", HT_CH_ENERGY,  10.0f};
    g_slow[3]=(Reg_t){HT7017_REG_EQ1,      &g_varh,      &g_e1Scale,0,0,1,0,&g_eq1_last,&g_eq1_seeded,&g_varh,NULL,"EQ1","VARh",0,            0.0f};
    g_slow[4]=(Reg_t){HT7017_REG_EMUSR,    NULL,NULL,     0,0,0,1,NULL,NULL,NULL,NULL,"EMUSR","",0,0.0f};
}

/* ═══════════════════════════════════════════════════════════
 * G — CHANNEL PUBLISH (via setChannel console command)
 *     No channel header dependency — uses CMD_ExecuteCommand.
 * ═══════════════════════════════════════════════════════════ */
static void PubCh(uint8_t ch, float val, float scale)
{
    if (!ch) return;
    char buf[40];
    snprintf(buf, sizeof(buf), "setChannel %u %d", ch, (int)(val * scale));
    CMD_ExecuteCommand(buf, 0);
}

/* ═══════════════════════════════════════════════════════════
 * H — FRAME HELPERS
 * ═══════════════════════════════════════════════════════════ */
static uint8_t Checksum(uint8_t reg, uint8_t d2, uint8_t d1, uint8_t d0)
{
    return (uint8_t)(~(uint8_t)(HT7017_FRAME_HEAD + reg + d2 + d1 + d0));
}

static float Convert(uint32_t raw, const Reg_t *r)
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
    float res = val / (*r->scale);
    if (r->is_signed && res > -1.0f && res < 1.0f) res = 0.0f;
    return res;
}

static void SendReq(uint8_t reg)
{
    UART_ConsumeBytes(UART_GetDataSize());
    UART_SendByte(HT7017_FRAME_HEAD);
    UART_SendByte(reg);
    g_txCount += 2;
}

/* ═══════════════════════════════════════════════════════════
 * I — POWER FACTOR
 * ═══════════════════════════════════════════════════════════ */
static void UpdatePF(void)
{
    g_apparent = g_voltage * g_current;
    if (g_apparent > 0.5f && g_power > 0.0f) {
        g_pf = g_power / g_apparent;
        if (g_pf > 1.0f) g_pf = 1.0f;
    } else {
        g_pf = 0.0f;
    }
    PubCh(HT_CH_PF, g_pf, 1000.0f);
}

/* ═══════════════════════════════════════════════════════════
 * J — SOFTWARE PROTECTION
 *
 * Fault detected → publish alarm code to Ch7 (non-zero).
 * autoexec.bat reacts:
 *   addChangeHandler Channel7 != 0 SetChannel 8 0
 *   addChangeHandler Channel7 == 0 SetChannel 8 100
 * drv_kws303wf.c watches Ch8 and fires P7/P8 coil pulse.
 * This driver NEVER writes to P7, P8, or Ch8.
 * ═══════════════════════════════════════════════════════════ */
static void Protection(void)
{
    bool any = g_ovThr>0||g_uvThr>0||g_ocThr>0||g_opThr>0;
    if (!any && !g_tripped) return;

    if (!g_tripped) {
        uint8_t a = 0;
        if      (g_ovThr>0 && g_voltage>g_ovThr)                            a=1;
        else if (g_ocThr>0 && g_current>g_ocThr)                            a=3;
        else if (g_opThr>0 && g_power>g_opThr)                              a=4;
        else if (g_uvThr>0 && g_voltage<g_uvThr && g_voltage>HT7017_UV_MIN_VOLTAGE) a=2;
        if (a) {
            const char *n[]={"","OV","UV","OC","OP"};
            g_alarm=a; g_tripped=true; g_tripAt=g_tick;
            PubCh(HT_CH_ALARM, (float)a, 1.0f);
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                "HT7017: TRIP[%s] V=%.2f I=%.3f P=%.1f → Ch%u=%u",
                n[a],g_voltage,g_current,g_power,HT_CH_ALARM,a);
        }
    } else {
        bool clr=false;
        switch(g_alarm){
            case 1:clr=g_ovThr>0&&g_voltage<g_ovThr-HT7017_PROT_HYSTERESIS_V;break;
            case 2:clr=g_uvThr>0&&g_voltage>g_uvThr+HT7017_PROT_HYSTERESIS_V;break;
            case 3:clr=g_ocThr>0&&g_current<g_ocThr-HT7017_PROT_HYSTERESIS_A;break;
            case 4:clr=g_opThr>0&&g_power<g_opThr-HT7017_PROT_HYSTERESIS_W;  break;
            default:clr=true;
        }
        if (clr && (g_tick-g_tripAt)>=(uint32_t)HT7017_PROT_RECOVER_SECONDS) {
            g_tripped=false; g_alarm=0;
            PubCh(HT_CH_ALARM,0,1.0f);
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
                "HT7017: CLEAR → Ch%u=0",HT_CH_ALARM);
        }
    }
}

/* ═══════════════════════════════════════════════════════════
 * K — ENERGY ACCUMULATOR (24-bit wrap handled)
 * ═══════════════════════════════════════════════════════════ */
static void Accumulate(uint32_t raw, const Reg_t *r)
{
    if (!(*r->seeded)) {
        *r->last_raw_acc=raw; *r->seeded=true; return;
    }
    uint32_t delta;
    if (raw >= *r->last_raw_acc) {
        delta = raw - *r->last_raw_acc;
    } else {
        delta = (0x00FFFFFFu - *r->last_raw_acc) + raw + 1u;
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
            "HT7017: [%s] 24b wrap prev=%u new=%u",r->name,*r->last_raw_acc,raw);
    }
    *r->last_raw_acc = raw;
    if (!delta) return;
    *r->acc += (float)delta / (*r->scale);
    if (r->ch) PubCh(r->ch, *r->acc, r->ch_scale);
    if (g_verbose)
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
            "HT7017:[%s] total=%.4f %s",r->name,*r->acc,r->unit);
}

/* ═══════════════════════════════════════════════════════════
 * L — FRAME PROCESSING
 * ═══════════════════════════════════════════════════════════ */
static void ProcessFrame(uint8_t d2,uint8_t d1,uint8_t d0,uint8_t cs,const Reg_t *r)
{
    if (cs != Checksum(r->reg,d2,d1,d0)) {
        g_badFr++;
        if (g_verbose)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:BAD CS reg=0x%02X",r->reg);
        return;
    }
    uint32_t raw=((uint32_t)d2<<16)|((uint32_t)d1<<8)|d0;
    g_goodFr++;
    if (r->raw_cal) *r->raw_cal=raw;

    if (r->is_emusr) {
        g_emusr=raw;
        if (g_verbose)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:EMUSR=0x%06X",raw);
        return;
    }
    if (r->is_energy) {
        Accumulate(raw,r);
    } else {
        if (r->reg==HT7017_REG_POWER_S1) g_s1_tick=g_tick;
        float v=Convert(raw,r);
        *r->target=v;
        if (r->ch) PubCh(r->ch,v,r->ch_scale);
        if (g_verbose)
            addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:[%s]=%.4f %s",r->name,v,r->unit);
    }
    UpdatePF();
    Protection();
    /* heap monitor every 5 min */
    if (g_tick%300==0)
        addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
            "HT7017:heap=%u good=%u bad=%u miss=%u",
            (unsigned)xPortGetFreeHeapSize(),g_goodFr,g_badFr,g_miss);
}

/* ═══════════════════════════════════════════════════════════
 * M — CALIBRATION PERSISTENCE
 * ═══════════════════════════════════════════════════════════ */
static void SaveCal(void)
{
    FILE *f=fopen(HT7017_CAL_FILE,"w");
    if(!f){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:cal save fail");return;}
    fprintf(f,"V=%f\nI=%f\nP=%f\n",g_vScale,g_iScale,g_pScale);
    fclose(f);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:cal saved V=%.4f I=%.4f P=%.6f",g_vScale,g_iScale,g_pScale);
}
static void LoadCal(void)
{
    FILE *f=fopen(HT7017_CAL_FILE,"r");
    if(!f){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:no cal file,using defaults");return;}
    float v=0,i=0,p=0;
    int n=fscanf(f,"V=%f\nI=%f\nP=%f\n",&v,&i,&p);
    fclose(f);
    if(n!=3||v<=0||i<=0||p==0){addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:cal invalid");return;}
    g_vScale=v;g_iScale=i;g_pScale=p;g_qScale=p;g_sScale=fabsf(p);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:cal loaded V=%.4f I=%.4f P=%.6f",v,i,p);
}

/* ═══════════════════════════════════════════════════════════
 * N — CONSOLE COMMANDS
 * ═══════════════════════════════════════════════════════════ */
static commandResult_t c_VoltageSet(const void*x,const char*c,const char*a,int f){
    if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float v=(float)atof(a);if(v<=0||!g_rawV)return CMD_RES_BAD_ARGUMENT;
    float old=g_vScale;g_vScale=(float)g_rawV/v;g_voltage=v;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:VoltageSet %.2fV %.2f→%.2f",v,old,g_vScale);
    SaveCal();return CMD_RES_OK;}

static commandResult_t c_CurrentSet(const void*x,const char*c,const char*a,int f){
    if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float v=(float)atof(a);if(v<=0||!g_rawI)return CMD_RES_BAD_ARGUMENT;
    float net=(float)g_rawI-g_iOffset;if(net<=0)return CMD_RES_ERROR;
    float old=g_iScale;g_iScale=net/v;g_current=v;
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:CurrentSet %.3fA %.2f→%.2f",v,old,g_iScale);
    SaveCal();return CMD_RES_OK;}

static commandResult_t c_PowerSet(const void*x,const char*c,const char*a,int f){
    if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    float v=(float)atof(a);if(v<=0||!g_rawP)return CMD_RES_BAD_ARGUMENT;
    int32_t s=(int32_t)g_rawP;if(s&0x800000)s|=(int32_t)0xFF000000;
    if(!s)return CMD_RES_ERROR;
    float old=g_pScale;g_pScale=(float)s/v;g_power=v;g_qScale=g_pScale;g_sScale=fabsf(g_pScale);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:PowerSet %.1fW %.4f→%.4f",v,old,g_pScale);
    SaveCal();return CMD_RES_OK;}

static commandResult_t c_SaveCal(const void*x,const char*c,const char*a,int f){SaveCal();return CMD_RES_OK;}
static commandResult_t c_LoadCal(const void*x,const char*c,const char*a,int f){LoadCal();return CMD_RES_OK;}

static commandResult_t c_EnergyReset(const void*x,const char*c,const char*a,int f){
    g_wh=g_varh=0;g_ep1_seeded=g_eq1_seeded=false;
    PubCh(HT_CH_ENERGY,0,1.0f);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:energy reset");return CMD_RES_OK;}

static commandResult_t c_Status(const void*x,const char*c,const char*a,int f){
    const char*n[]={"OK","OV","UV","OC","OP"};
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017 V=%.2f I=%.3f P=%.2f Hz=%.2f PF=%.4f",g_voltage,g_current,g_power,g_freq,g_pf);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"       Wh=%.4f Alarm=%s heap=%u good=%u bad=%u miss=%u",
        g_wh,n[g_alarm],(unsigned)xPortGetFreeHeapSize(),g_goodFr,g_badFr,g_miss);
    return CMD_RES_OK;}

static commandResult_t c_SetOV(const void*x,const char*c,const char*a,int f){if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;g_ovThr=(float)atof(a);addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:OV=%.1f",g_ovThr);return CMD_RES_OK;}
static commandResult_t c_SetUV(const void*x,const char*c,const char*a,int f){if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;g_uvThr=(float)atof(a);addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:UV=%.1f",g_uvThr);return CMD_RES_OK;}
static commandResult_t c_SetOC(const void*x,const char*c,const char*a,int f){if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;g_ocThr=(float)atof(a);addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:OC=%.3f",g_ocThr);return CMD_RES_OK;}
static commandResult_t c_SetOP(const void*x,const char*c,const char*a,int f){if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;g_opThr=(float)atof(a);addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:OP=%.1f",g_opThr);return CMD_RES_OK;}
static commandResult_t c_Verbose(const void*x,const char*c,const char*a,int f){g_verbose=!g_verbose;addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:verbose %s",g_verbose?"ON":"OFF");return CMD_RES_OK;}
static commandResult_t c_Baud(const void*x,const char*c,const char*a,int f){
    if(!a||!*a)return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    UART_InitUART(atoi(a),HT7017_PARITY_EVEN,0);
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,"HT7017:baud=%s 8E1",a);return CMD_RES_OK;}

/* ═══════════════════════════════════════════════════════════
 * O — PUBLIC GETTERS  (drv_kws303wf reads these)
 * ═══════════════════════════════════════════════════════════ */
float    HT7017_GetVoltage(void)     { return g_voltage; }
float    HT7017_GetCurrent(void)     { return g_current; }
float    HT7017_GetPower(void)       { return g_power;   }
float    HT7017_GetFrequency(void)   { return g_freq;    }
float    HT7017_GetPowerFactor(void) { return g_pf;      }
float    HT7017_GetWh(void)          { return g_wh;      }
uint8_t  HT7017_GetAlarmState(void)  { return g_alarm;   }

/* ═══════════════════════════════════════════════════════════
 * P — INIT AND RUN
 * ═══════════════════════════════════════════════════════════ */
void HT7017_Init(void)
{
    BuildTables();
    g_fi=FAST_N-1; g_si=SLOW_N-1;
    g_tick=g_txCount=g_goodFr=g_badFr=g_miss=0;
    g_wasFast=true; g_fmiss=g_smiss=0;
    g_emusr=g_alarm=0; g_tripped=false; g_tripAt=0;
    g_s1_tick=0; g_verbose=0;

    UART_InitReceiveRingBuffer(32);
    UART_InitUART(HT7017_BAUD_RATE, HT7017_PARITY_EVEN, 0);

    CMD_RegisterCommand("VoltageSet",         c_VoltageSet,  NULL);
    CMD_RegisterCommand("CurrentSet",         c_CurrentSet,  NULL);
    CMD_RegisterCommand("PowerSet",           c_PowerSet,    NULL);
    CMD_RegisterCommand("HT7017_SaveCalib",   c_SaveCal,     NULL);
    CMD_RegisterCommand("HT7017_LoadCalib",   c_LoadCal,     NULL);
    CMD_RegisterCommand("HT7017_EnergyReset", c_EnergyReset, NULL);
    CMD_RegisterCommand("HT7017_Status",      c_Status,      NULL);
    CMD_RegisterCommand("HT7017_SetOV",       c_SetOV,       NULL);
    CMD_RegisterCommand("HT7017_SetUV",       c_SetUV,       NULL);
    CMD_RegisterCommand("HT7017_SetOC",       c_SetOC,       NULL);
    CMD_RegisterCommand("HT7017_SetOP",       c_SetOP,       NULL);
    CMD_RegisterCommand("HT7017_Verbose",     c_Verbose,     NULL);
    CMD_RegisterCommand("HT7017_Baud",        c_Baud,        NULL);

    LoadCal();
    addLogAdv(LOG_INFO,LOG_FEATURE_ENERGY,
        "HT7017: ready 4800 8E1 | Ch%u=V Ch%u=I Ch%u=P Ch%u=Hz Ch%u=PF Ch%u=Wh Ch%u=Alarm",
        HT_CH_VOLTAGE,HT_CH_CURRENT,HT_CH_POWER,HT_CH_FREQ,
        HT_CH_PF,HT_CH_ENERGY,HT_CH_ALARM);
}

void HT7017_RunEverySecond(void)
{
    int avail=UART_GetDataSize();
    if (avail>=HT7017_RESPONSE_LEN) {
        uint8_t d2=UART_GetByte(0),d1=UART_GetByte(1),
                d0=UART_GetByte(2),cs=UART_GetByte(3);
        UART_ConsumeBytes(HT7017_RESPONSE_LEN);
        ProcessFrame(d2,d1,d0,cs, g_wasFast ? &g_fast[g_fi] : &g_slow[g_si]);
    } else {
        g_miss++;
        if (g_txCount>0) {
            if (g_wasFast){if(++g_fmiss>=HT7017_MAX_MISS_COUNT){g_fi=(g_fi+1)%FAST_N;g_fmiss=0;}}
            else           {if(++g_smiss>=HT7017_MAX_MISS_COUNT){g_si=(g_si+1)%SLOW_N;g_smiss=0;}}
        }
        UART_ConsumeBytes(UART_GetDataSize());
    }
    g_tick++;
    if (g_tick%HT7017_FAST_RATIO==0) {
        g_si=(g_si+1)%SLOW_N; g_wasFast=false; SendReq(g_slow[g_si].reg);
    } else {
        g_fi=(g_fi+1)%FAST_N; g_wasFast=true;  SendReq(g_fast[g_fi].reg);
    }
}

#endif /* ENABLE_DRIVER_HT7017 */
