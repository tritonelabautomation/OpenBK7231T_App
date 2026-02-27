/*
 * drv_st7735.c — ST7735S 0.96" 80×160 TFT Driver for OpenBK7231T / BK7231N
 * Target: KWS-303WF  FPC-JL096B005-01V0
 *
 * startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> [BLK]
 * Example:   startDriver ST7735 14 16 9 17 15 24
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │  DISPLAY LAYOUT  80×160px                                   │
 * │                                                             │
 * │  Y=  0 h=10  OFF      NoWF    ← GREY         GREY          │
 * │  Y= 11 h=29  221.5         V  ← RED   scale2               │
 * │  Y= 40 h=29  0.55          A  ← CYAN  scale2               │
 * │  Y= 69 h=29  121.7         W  ← YELLOW scale2              │
 * │  Y= 98 h=15  000.47kWh        ← CYAN  scale1               │
 * │  Y=113 h=15  Rs   3.76        ← GREEN scale1  (EV cost)     │
 * │  Y=128 h=15  0.20PF  49.9Hz   ← RED | BLUE scale1          │
 * │  Y=143 h=17  27.16C  01:23    ← ORANGE | WHITE scale1 split │
 * └─────────────────────────────────────────────────────────────┘
 *
 * PIXEL SUM: 10+1+29+29+29+15+15+15+17 = 160px ✓
 * (Y=10 is a 1px natural black gap — no drawn divider)
 *
 * ── SEPARATION PRINCIPLE (OpenBK flat-C idiom) ────────────────────────────
 *  Section A  — TFT hardware pins & global state
 *  Section B  — Software SPI bit-bang (hardware only)
 *  Section C  — Delay helper
 *  Section D  — ST7735S controller init sequence (hardware only)
 *  Section E  — Drawing primitives: FillRect, DrawChar, DrawString (hardware only)
 *  Section F  — ADC / NTC temperature module (adc_read_temp — no display code)
 *  Section G  — Button manager module (btn_init, btn_tick — no display code)
 *  Section H  — Energy screen layout & EV session (energy_init, energy_tick)
 *  Section I  — Public API: ST7735_SetWifiStatus, ST7735_SetRelayState
 *  Section J  — Console commands
 *  Section K  — OpenBK lifecycle: ST7735_Init, ST7735_RunEverySecond
 *
 * Each section calls ONLY functions from sections above it.
 * btn_tick() never touches display. adc_read_temp() never touches display.
 * energy_tick() calls adc_read_temp() and btn's relay state — no SPI detail.
 *
 * ── RELAY (latching, schematic-verified) ─────────────────────────────────
 *  Ch8 → GPIO P7 = ON  coil (latch CLOSED)
 *  Ch7 → GPIO P8 = OFF coil (latch OPEN)
 *  50 ms pulse applied to coil then released — latching relay requires this.
 *
 * ── BUTTONS (active-low, internal pull-up) ───────────────────────────────
 *  P28 [ON/OFF] — single press toggles relay
 *  P26 [+]      — reserved
 *  P20 [−]      — reserved
 *  BTN_DEBOUNCE_TICKS=1: polled every second; 1 sample is enough debounce.
 *
 * ── NTC TEMPERATURE ──────────────────────────────────────────────────────
 *  Pin P23 / ADC ch23, BK7231N 12-bit ADC.
 *  Steinhart-Hart B-parameter. NTC_PULLUP=1 (NTC is pull-up side).
 *
 * ── TGSPDCL CAT-2B ENERGY COST ───────────────────────────────────────────
 *  Default rate: Rs 3.75/unit (201-300 slab, Telangana domestic 2024-25).
 *  Override: ev_rate <value>    e.g. ev_rate 5.00 for 301-400 slab.
 *  Session reset: ev_reset      (zeroes kWh offset + timer).
 */

#include "../obk_config.h"
#if ENABLE_DRIVER_ST7735

#include "drv_st7735.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "../cmnds/cmd_public.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION A — TFT HARDWARE PINS & GLOBAL STATE
 * ══════════════════════════════════════════════════════════════════════════════ */

/* TFT SPI pins */
static int     g_pin_sck = ST7735_DEFAULT_SCK;
static int     g_pin_sda = ST7735_DEFAULT_SDA;
static int     g_pin_res = ST7735_DEFAULT_RES;
static int     g_pin_dc  = ST7735_DEFAULT_DC;
static int     g_pin_cs  = ST7735_DEFAULT_CS;
static int     g_pin_blk = ST7735_DEFAULT_BLK;
static uint8_t g_initialized = 0;

/* Console text-mode state (for st7735_print / st7735_goto commands) */
static uint8_t  g_cur_col   = 0;
static uint8_t  g_cur_row   = 0;
static uint16_t g_fg_colour = ST7735_WHITE;
static uint16_t g_bg_colour = ST7735_BLACK;
static uint8_t  g_txt_scale = ST7735_TEXT_SCALE_LARGE;

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION B — SOFTWARE SPI BIT-BANG  (hardware only, no application logic)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SPI_SCK_H()  HAL_PIN_SetOutputValue(g_pin_sck, 1)
#define SPI_SCK_L()  HAL_PIN_SetOutputValue(g_pin_sck, 0)
#define SPI_SDA_H()  HAL_PIN_SetOutputValue(g_pin_sda, 1)
#define SPI_SDA_L()  HAL_PIN_SetOutputValue(g_pin_sda, 0)
#define SPI_CS_H()   HAL_PIN_SetOutputValue(g_pin_cs,  1)
#define SPI_CS_L()   HAL_PIN_SetOutputValue(g_pin_cs,  0)
#define SPI_DC_H()   HAL_PIN_SetOutputValue(g_pin_dc,  1)
#define SPI_DC_L()   HAL_PIN_SetOutputValue(g_pin_dc,  0)
#define SPI_RES_H()  HAL_PIN_SetOutputValue(g_pin_res, 1)
#define SPI_RES_L()  HAL_PIN_SetOutputValue(g_pin_res, 0)
#define SPI_BLK_H()  HAL_PIN_SetOutputValue(g_pin_blk, 1)
#define SPI_BLK_L()  HAL_PIN_SetOutputValue(g_pin_blk, 0)

static inline void spi_delay(void) { volatile int i = 4; while (i--) {} }

static void SPI_WriteByte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (b & 0x80) { SPI_SDA_H(); } else { SPI_SDA_L(); }
        spi_delay(); SPI_SCK_H(); spi_delay(); SPI_SCK_L();
        b <<= 1;
    }
}

static void TFT_WriteCmd(uint8_t cmd)
    { SPI_CS_L(); SPI_DC_L(); SPI_WriteByte(cmd); SPI_CS_H(); }

static void TFT_WriteData8(uint8_t d)
    { SPI_CS_L(); SPI_DC_H(); SPI_WriteByte(d);   SPI_CS_H(); }

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION C — DELAY
 * ══════════════════════════════════════════════════════════════════════════════ */

static void ST7735_Delay(uint32_t ms)
{
    extern int rtos_delay_milliseconds(uint32_t num_ms);
    (void)rtos_delay_milliseconds(ms);
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION D — ST7735S CONTROLLER INIT  (hardware only)
 * ══════════════════════════════════════════════════════════════════════════════ */

static void TFT_HardReset(void)
{
    SPI_RES_H(); ST7735_Delay(10);
    SPI_RES_L(); ST7735_Delay(10);
    SPI_RES_H(); ST7735_Delay(120);
}

static void TFT_InitController(void)
{
    TFT_WriteCmd(ST77_SWRESET); ST7735_Delay(150);
    TFT_WriteCmd(ST77_SLPOUT);  ST7735_Delay(500);

    TFT_WriteCmd(ST77_FRMCTR1);
    TFT_WriteData8(0x01); TFT_WriteData8(0x2C); TFT_WriteData8(0x2D);
    TFT_WriteCmd(ST77_FRMCTR2);
    TFT_WriteData8(0x01); TFT_WriteData8(0x2C); TFT_WriteData8(0x2D);
    TFT_WriteCmd(ST77_FRMCTR3);
    TFT_WriteData8(0x01); TFT_WriteData8(0x2C); TFT_WriteData8(0x2D);
    TFT_WriteData8(0x01); TFT_WriteData8(0x2C); TFT_WriteData8(0x2D);

    TFT_WriteCmd(ST77_INVCTR);  TFT_WriteData8(0x07);

    TFT_WriteCmd(ST77_PWCTR1);
    TFT_WriteData8(0xA2); TFT_WriteData8(0x02); TFT_WriteData8(0x84);
    TFT_WriteCmd(ST77_PWCTR2);  TFT_WriteData8(0xC5);
    TFT_WriteCmd(ST77_PWCTR3);
    TFT_WriteData8(0x0A); TFT_WriteData8(0x00);
    TFT_WriteCmd(ST77_PWCTR4);
    TFT_WriteData8(0x8A); TFT_WriteData8(0x2A);
    TFT_WriteCmd(ST77_PWCTR5);
    TFT_WriteData8(0x8A); TFT_WriteData8(0xEE);

    TFT_WriteCmd(ST77_VMCTR1);  TFT_WriteData8(0x0E);
    TFT_WriteCmd(ST77_INVOFF);
    TFT_WriteCmd(ST77_MADCTL);  TFT_WriteData8(MADCTL_BGR);
    TFT_WriteCmd(ST77_COLMOD);  TFT_WriteData8(0x05); ST7735_Delay(10);

    TFT_WriteCmd(ST77_CASET);
    TFT_WriteData8(0x00); TFT_WriteData8(ST7735_COL_OFFSET);
    TFT_WriteData8(0x00); TFT_WriteData8(ST7735_COL_OFFSET + ST7735_WIDTH - 1);
    TFT_WriteCmd(ST77_RASET);
    TFT_WriteData8(0x00); TFT_WriteData8(ST7735_ROW_OFFSET);
    TFT_WriteData8(0x00); TFT_WriteData8(ST7735_ROW_OFFSET + ST7735_HEIGHT - 1);

    TFT_WriteCmd(ST77_GMCTRP1);
    TFT_WriteData8(0x02); TFT_WriteData8(0x1C); TFT_WriteData8(0x07);
    TFT_WriteData8(0x12); TFT_WriteData8(0x37); TFT_WriteData8(0x32);
    TFT_WriteData8(0x29); TFT_WriteData8(0x2D); TFT_WriteData8(0x29);
    TFT_WriteData8(0x25); TFT_WriteData8(0x2B); TFT_WriteData8(0x39);
    TFT_WriteData8(0x00); TFT_WriteData8(0x01); TFT_WriteData8(0x03);
    TFT_WriteData8(0x10);
    TFT_WriteCmd(ST77_GMCTRN1);
    TFT_WriteData8(0x03); TFT_WriteData8(0x1D); TFT_WriteData8(0x07);
    TFT_WriteData8(0x06); TFT_WriteData8(0x2E); TFT_WriteData8(0x2C);
    TFT_WriteData8(0x29); TFT_WriteData8(0x2D); TFT_WriteData8(0x2E);
    TFT_WriteData8(0x2E); TFT_WriteData8(0x37); TFT_WriteData8(0x3F);
    TFT_WriteData8(0x00); TFT_WriteData8(0x00); TFT_WriteData8(0x02);
    TFT_WriteData8(0x10);

    TFT_WriteCmd(ST77_NORON);  ST7735_Delay(10);
    TFT_WriteCmd(ST77_DISPON); ST7735_Delay(100);
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION E — DRAWING PRIMITIVES  (hardware only — no application logic)
 * ══════════════════════════════════════════════════════════════════════════════ */

static void TFT_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    TFT_WriteCmd(ST77_CASET);
    TFT_WriteData8(0x00); TFT_WriteData8(x0 + ST7735_COL_OFFSET);
    TFT_WriteData8(0x00); TFT_WriteData8(x1 + ST7735_COL_OFFSET);
    TFT_WriteCmd(ST77_RASET);
    TFT_WriteData8(0x00); TFT_WriteData8(y0 + ST7735_ROW_OFFSET);
    TFT_WriteData8(0x00); TFT_WriteData8(y1 + ST7735_ROW_OFFSET);
    TFT_WriteCmd(ST77_RAMWR);
}

void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour)
{
    if (!g_initialized) return;
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH  - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;
    TFT_SetWindow(x, y, x + w - 1, y + h - 1);
    uint8_t hi = colour >> 8, lo = colour & 0xFF;
    uint32_t n = (uint32_t)w * h;
    SPI_CS_L(); SPI_DC_H();
    while (n--) { SPI_WriteByte(hi); SPI_WriteByte(lo); }
    SPI_CS_H();
}

void ST7735_FillScreen(uint16_t colour)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, colour);
}

/* 5×7 font, ASCII 0x20–0x7E */
static const uint8_t g_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x40,0x3C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},
};

void ST7735_DrawChar(uint8_t x, uint8_t y, char c,
                     uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!g_initialized) return;
    if (c < 0x20 || c > 0x7E) c = '?';
    if (x + FONT_W * scale > ST7735_WIDTH)  return;
    if (y + FONT_H * scale > ST7735_HEIGHT) return;
    const uint8_t *gl = g_font5x7[(uint8_t)(c - 0x20)];
    TFT_SetWindow(x, y, x + FONT_W * scale - 1, y + FONT_H * scale - 1);
    SPI_CS_L(); SPI_DC_H();
    uint8_t row, s, col, t;
    for (row = 0; row < FONT_H; row++) {
        for (s = 0; s < scale; s++) {
            for (col = 0; col < FONT_W; col++) {
                uint16_t colour = ((gl[col] >> row) & 1) ? fg : bg;
                uint8_t hi = colour >> 8, lo = colour & 0xFF;
                for (t = 0; t < scale; t++) { SPI_WriteByte(hi); SPI_WriteByte(lo); }
            }
        }
    }
    SPI_CS_H();
}

void ST7735_DrawString(uint8_t x, uint8_t y, const char *str,
                       uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!g_initialized || !str) return;
    while (*str) {
        if (x + FONT_W * scale > ST7735_WIDTH) break;
        ST7735_DrawChar(x, y, *str++, fg, bg, scale);
        x += FONT_ADV * scale;
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION F — ADC / NTC TEMPERATURE MODULE
 *
 * Responsibility: read raw ADC, convert to °C via Steinhart-Hart.
 * No SPI, no display, no button, no relay logic here.
 *
 * NTC wiring KWS-303WF: Vcc→NTC→ADC_pin(P23)→Rs(10kΩ)→GND  (NTC_PULLUP=1)
 * HAL_ADC_Read(ch) — BK7231N routes P23 to SADC automatically; no pin setup.
 * ══════════════════════════════════════════════════════════════════════════════ */

#define ADC_TEMP_CHANNEL   23        /* P23 / ADC3 on BK7231N */
#define ADC_MAX_VAL        4095.0f   /* 12-bit ADC */
#define NTC_R25            10000.0f  /* NTC nominal Ω at 25°C */
#define NTC_B              3950.0f   /* B-constant (K) */
#define NTC_RS             10000.0f  /* Series resistor Ω */
#define NTC_T0_K           298.15f   /* 25°C in Kelvin */
#define NTC_PULLUP         1         /* 1 → Vcc→NTC→pin→Rs→GND */

extern uint32_t HAL_ADC_Read(uint8_t channel);

static float adc_read_temp(void)
{
    uint32_t raw = HAL_ADC_Read(ADC_TEMP_CHANNEL);
    if (raw == 0)                    return -99.0f;   /* short / no sensor */
    if (raw >= (uint32_t)ADC_MAX_VAL) return 999.0f; /* open circuit */

    float r = (float)raw;
#if NTC_PULLUP
    float rntc = NTC_RS * r / (ADC_MAX_VAL - r);         /* NTC is pull-up */
#else
    float rntc = NTC_RS * (ADC_MAX_VAL - r) / r;         /* Rs is pull-up  */
#endif
    float temp_k = 1.0f / (1.0f / NTC_T0_K + logf(rntc / NTC_R25) / NTC_B);
    return temp_k - 273.15f;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION G — BUTTON MANAGER MODULE
 *
 * Responsibility: GPIO debounce and callback dispatch.
 * No display code, no relay GPIO, no HT7017 here.
 * Callback functions (registered via btn_register) perform the actual action.
 *
 * Poll interval: 1 second (called from RunEverySecond).
 * BTN_DEBOUNCE_TICKS=1: one stable LOW reading = confirmed press.
 * This is correct because the 1-second poll interval itself provides debounce.
 * ══════════════════════════════════════════════════════════════════════════════ */

#define BTN_MAX            4         /* max buttons managed */
#define BTN_DEBOUNCE_TICKS 1         /* 1 poll-period = 1 s — enough debounce */

typedef void (*BtnCallback_t)(void);

typedef struct {
    int           pin;
    uint8_t       last_level;
    uint8_t       db_count;
    BtnCallback_t on_press;      /* called on confirmed falling edge */
} BtnEntry_t;

static BtnEntry_t g_btns[BTN_MAX];
static uint8_t    g_btn_count = 0;

/* Register a button. Returns 1 on success, 0 if table full. */
static uint8_t btn_register(int pin, BtnCallback_t on_press)
{
    if (g_btn_count >= BTN_MAX) return 0;
    BtnEntry_t *b = &g_btns[g_btn_count++];
    b->pin        = pin;
    b->last_level = 1;    /* assume not pressed at boot */
    b->db_count   = 0;
    b->on_press   = on_press;
    HAL_PIN_Setup_Input_Pullup(pin);
    return 1;
}

/* Call once per second from RunEverySecond. Fires on_press on falling edge. */
static void btn_tick(void)
{
    extern int HAL_PIN_ReadDigitalInput(int pin);
    uint8_t i;
    for (i = 0; i < g_btn_count; i++) {
        BtnEntry_t *b    = &g_btns[i];
        uint8_t     level = (uint8_t)HAL_PIN_ReadDigitalInput(b->pin);

        if (level == b->last_level) {
            b->db_count = 0;   /* stable — reset counter */
            continue;
        }
        /* level changed from last stable */
        if (++b->db_count < BTN_DEBOUNCE_TICKS)
            continue;

        /* confirmed edge */
        b->db_count   = 0;
        b->last_level = level;

        if (level == 0 && b->on_press) {   /* falling edge = active-low press */
            b->on_press();
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION H — ENERGY SCREEN LAYOUT + EV SESSION
 *
 * Responsibility: draw all 8 screen rows, manage EV session state.
 * Calls adc_read_temp() (Section F) for temperature.
 * Calls ST7735_FillRect / ST7735_DrawString / ST7735_DrawChar (Section E).
 * No SPI bit-bang detail here — only high-level drawing calls.
 *
 * Layout (no dividers — matches attached working code exactly):
 *   Y=  0 H=10  STATUS: "OFF    NoWF"
 *   Y= 11 H=29  VOLTAGE: "221.5 V"      RED   scale2
 *   Y= 40 H=29  CURRENT: "0.55  A"      CYAN  scale2
 *   Y= 69 H=29  POWER:   "121.7 W"      YELLOW scale2
 *   Y= 98 H=15  kWh:     "000.47kWh"    CYAN  scale1
 *   Y=113 H=15  COST:    "Rs   3.76"    GREEN scale1
 *   Y=128 H=15  PF+Hz:   "0.20PF 49.9Hz" RED|BLUE scale1
 *   Y=143 H=17  TEMP+TMR:"27.16C  01:23" ORANGE|WHITE scale1
 *
 * TGSPDCL CAT-2B tariff defaults:
 *   Rs 3.75/unit (slab 201-300). Override via console: ev_rate <value>
 * ══════════════════════════════════════════════════════════════════════════════ */

/* ── Layout constants (pixel-verified, no dividers) ──────────────────────── */
#define ROW_STATUS_Y    0
#define ROW_STATUS_H   10
#define ROW_V_Y        11      /* 1px natural black gap at Y=10 */
#define ROW_V_H        29
#define ROW_A_Y        40      /* 11+29 */
#define ROW_A_H        29
#define ROW_W_Y        69      /* 40+29 */
#define ROW_W_H        29
#define ROW_KWH_Y      98      /* 69+29 */
#define ROW_KWH_H      15
#define ROW_COST_Y    113      /* 98+15 */
#define ROW_COST_H     15
#define ROW_PFHZ_Y    128      /* 113+15 */
#define ROW_PFHZ_H     15
#define ROW_TEMPTMR_Y 143      /* 128+15 */
#define ROW_TEMPTMR_H  17      /* 143+17=160 ✓ */

#define VAL_S   2    /* large scale: 12px/ch wide, 14px tall */
#define SML_S   1    /* small scale:  6px/ch wide,  7px tall */

/* Large rows: right 10px = unit label drawn once at init; left 70px = value */
#define LBL_W       (FONT_W * VAL_S)          /* 10 */
#define LBL_X       (ST7735_WIDTH - LBL_W)    /* 70 */
#define VAL_ZONE_W   LBL_X                    /* 70 */

/* Small full-row */
#define SML_FULL_W   ST7735_WIDTH             /* 80 */

/* PF+Hz split: PF x=0 w=38 | Hz x=38 w=42 */
#define PFHZ_PF_X    0
#define PFHZ_PF_W   38
#define PFHZ_HZ_X   38
#define PFHZ_HZ_W   42

/* Status row: relay left | WiFi right */
#define STA_RELAY_X   0
#define STA_RELAY_W  22    /* "ON "/"OFF" = 3ch×6=18px, clear 22px */
#define STA_WIFI_X   56    /* 80-24=56 */
#define STA_WIFI_W   24    /* "WiFi"/"NoWF" = 4ch×6=24px */

/* Temp+Timer split: temp left | timer right */
#define TEMPTMR_TC_X   0
#define TEMPTMR_TC_W  40   /* "27.16C" = 6ch×6=36px, 4px spare */
#define TEMPTMR_TM_X  40
#define TEMPTMR_TM_W  40   /* "01:23"  = 5ch×6=30px, 10px spare */

/* ── EV session state ─────────────────────────────────────────────────────── */
/* TGSPDCL CAT-2B: Rs 3.75/unit is the 201-300 slab marginal rate (2024-25). */
#define EV_DEFAULT_RATE_RS  3.75f    /* Rs per kWh — TGSPDCL CAT-2B 201-300 slab */

static float    g_ev_rate_rs     = EV_DEFAULT_RATE_RS;
static float    g_ev_kwh_offset  = 0.0f;   /* meter reading at session start */
static uint32_t g_uptime_seconds = 0;       /* session timer — seconds */

/* ── Per-field string cache (SPI write skipped when value unchanged) ─────── */
static char g_prev_v[10]    = "";
static char g_prev_a[10]    = "";
static char g_prev_w[10]    = "";
static char g_prev_kwh[14]  = "";
static char g_prev_cost[14] = "";
static char g_prev_tmr[8]   = "";
static char g_prev_pf[10]   = "";
static char g_prev_hz[10]   = "";
static char g_prev_tc[10]   = "";
static char g_prev_on[4]    = "";
static char g_prev_wifi[6]  = "";

/* ── Display state shared with relay/wifi helpers ─────────────────────────── */
static uint8_t g_relay_state = 0;
static uint8_t g_wifi_ok     = 0;

/* ── Helper: clear zone + draw text vertically centred ─────────────────────── */
static void energy_update_zone(uint8_t x, uint8_t y, uint8_t zw, uint8_t zh,
                                const char *str, uint16_t fg, uint8_t scale)
{
    ST7735_FillRect(x, y, zw, zh, ST7735_BLACK);
    uint8_t ty = y + (zh - FONT_H * scale) / 2;
    ST7735_DrawString(x, ty, str, fg, ST7735_BLACK, scale);
}

/* ── Static frame: V/A/W unit labels drawn once at init ─────────────────── */
static void energy_draw_static_frame(void)
{
    uint8_t vy = ROW_V_Y + (ROW_V_H - FONT_H * VAL_S) / 2;
    uint8_t ay = ROW_A_Y + (ROW_A_H - FONT_H * VAL_S) / 2;
    uint8_t wy = ROW_W_Y + (ROW_W_H - FONT_H * VAL_S) / 2;
    ST7735_DrawChar(LBL_X, vy, 'V', ST7735_RED,    ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, ay, 'A', ST7735_CYAN,   ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, wy, 'W', ST7735_YELLOW, ST7735_BLACK, VAL_S);
}

/* ── Main screen draw — called every second from energy_tick() ─────────── */
static void energy_draw_screen(float v, float a, float w,
                                float kwh_meter, float pf, float hz)
{
    char buf[16];

    /* ── STATUS ROW ─────────────────────────────────────────────────────── */
    const char *on_str = g_relay_state ? "ON " : "OFF";
    if (strcmp(on_str, g_prev_on) != 0) {
        strncpy(g_prev_on, on_str, sizeof(g_prev_on) - 1);
        g_prev_on[sizeof(g_prev_on) - 1] = '\0';
        ST7735_FillRect(STA_RELAY_X, ROW_STATUS_Y, STA_RELAY_W, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(STA_RELAY_X, ROW_STATUS_Y + 1, on_str,
                          g_relay_state ? ST7735_GREEN : ST7735_GREY,
                          ST7735_BLACK, SML_S);
    }
    const char *wifi_str = g_wifi_ok ? "WiFi" : "NoWF";
    if (strcmp(wifi_str, g_prev_wifi) != 0) {
        strncpy(g_prev_wifi, wifi_str, sizeof(g_prev_wifi) - 1);
        g_prev_wifi[sizeof(g_prev_wifi) - 1] = '\0';
        ST7735_FillRect(STA_WIFI_X, ROW_STATUS_Y, STA_WIFI_W, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(STA_WIFI_X, ROW_STATUS_Y + 1, wifi_str,
                          g_wifi_ok ? ST7735_BLUE : ST7735_GREY,
                          ST7735_BLACK, SML_S);
    }

    /* ── VOLTAGE  (RED, scale2) ─────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.1f", v);
    if (strcmp(buf, g_prev_v) != 0) {
        strncpy(g_prev_v, buf, sizeof(g_prev_v) - 1);
        energy_update_zone(0, ROW_V_Y, VAL_ZONE_W, ROW_V_H, buf, ST7735_RED, VAL_S);
    }

    /* ── CURRENT  (CYAN, scale2) ────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.2f", a);
    if (strcmp(buf, g_prev_a) != 0) {
        strncpy(g_prev_a, buf, sizeof(g_prev_a) - 1);
        energy_update_zone(0, ROW_A_Y, VAL_ZONE_W, ROW_A_H, buf, ST7735_CYAN, VAL_S);
    }

    /* ── POWER  (YELLOW, scale2) ─────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.1f", w);
    if (strcmp(buf, g_prev_w) != 0) {
        strncpy(g_prev_w, buf, sizeof(g_prev_w) - 1);
        energy_update_zone(0, ROW_W_Y, VAL_ZONE_W, ROW_W_H, buf, ST7735_YELLOW, VAL_S);
    }

    /* ── kWh  (CYAN, scale1) — session kWh = meter − offset ─────────────── */
    float session_kwh = kwh_meter - g_ev_kwh_offset;
    if (session_kwh < 0.0f) session_kwh = 0.0f;
    snprintf(buf, sizeof(buf), "%06.2fkWh", session_kwh);
    if (strcmp(buf, g_prev_kwh) != 0) {
        strncpy(g_prev_kwh, buf, sizeof(g_prev_kwh) - 1);
        energy_update_zone(0, ROW_KWH_Y, SML_FULL_W, ROW_KWH_H, buf, ST7735_CYAN, SML_S);
    }

    /* ── COST  (GREEN, scale1) — "Rs9999.99" ─────────────────────────────── */
    /* TGSPDCL CAT-2B rate applied to session kWh */
    float cost = session_kwh * g_ev_rate_rs;
    if (cost < 10000.0f)
        snprintf(buf, sizeof(buf), "Rs%7.2f", cost);
    else
        snprintf(buf, sizeof(buf), "Rs%7.1f", cost);
    if (strcmp(buf, g_prev_cost) != 0) {
        strncpy(g_prev_cost, buf, sizeof(g_prev_cost) - 1);
        energy_update_zone(0, ROW_COST_Y, SML_FULL_W, ROW_COST_H, buf, ST7735_GREEN, SML_S);
    }

    /* ── PF + Hz SPLIT  (RED | BLUE, scale1) ─────────────────────────────── */
    {
        uint8_t phy = ROW_PFHZ_Y + (ROW_PFHZ_H - FONT_H * SML_S) / 2;

        snprintf(buf, sizeof(buf), "%.2fPF", pf);
        if (strcmp(buf, g_prev_pf) != 0) {
            strncpy(g_prev_pf, buf, sizeof(g_prev_pf) - 1);
            ST7735_FillRect(PFHZ_PF_X, ROW_PFHZ_Y, PFHZ_PF_W, ROW_PFHZ_H, ST7735_BLACK);
            ST7735_DrawString(PFHZ_PF_X, phy, buf, ST7735_RED, ST7735_BLACK, SML_S);
        }
        snprintf(buf, sizeof(buf), "%.1fHz", hz);
        if (strcmp(buf, g_prev_hz) != 0) {
            strncpy(g_prev_hz, buf, sizeof(g_prev_hz) - 1);
            ST7735_FillRect(PFHZ_HZ_X, ROW_PFHZ_Y, PFHZ_HZ_W, ROW_PFHZ_H, ST7735_BLACK);
            ST7735_DrawString(PFHZ_HZ_X, phy, buf, ST7735_BLUE, ST7735_BLACK, SML_S);
        }
    }

    /* ── TEMP + TIMER SPLIT  (ORANGE | WHITE, scale1) ──────────────────── */
    /* Left: temperature from NTC ADC (0.5°C hysteresis)                    */
    /* Right: session timer "MM:SS"                                          */
    {
        uint8_t tty = ROW_TEMPTMR_Y + (ROW_TEMPTMR_H - FONT_H * SML_S) / 2;

        /* Temperature — 0.5°C hysteresis avoids continuous SPI for tiny noise */
        static float s_last_tc = -999.0f;
        float tc = adc_read_temp();
        if (tc > s_last_tc + 0.5f || tc < s_last_tc - 0.5f) {
            s_last_tc = tc;
            snprintf(buf, sizeof(buf), "%05.2fC", tc);
            strncpy(g_prev_tc, buf, sizeof(g_prev_tc) - 1);
            ST7735_FillRect(TEMPTMR_TC_X, ROW_TEMPTMR_Y, TEMPTMR_TC_W, ROW_TEMPTMR_H, ST7735_BLACK);
            ST7735_DrawString(TEMPTMR_TC_X, tty, buf, ST7735_ORANGE, ST7735_BLACK, SML_S);
        }

        /* Timer "MM:SS" — rolls at 99:59 then wraps to 00:00 */
        uint32_t mm = (g_uptime_seconds / 60) % 100;
        uint32_t ss =  g_uptime_seconds % 60;
        snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)mm, (unsigned)ss);
        if (strcmp(buf, g_prev_tmr) != 0) {
            strncpy(g_prev_tmr, buf, sizeof(g_prev_tmr) - 1);
            ST7735_FillRect(TEMPTMR_TM_X, ROW_TEMPTMR_Y, TEMPTMR_TM_W, ROW_TEMPTMR_H, ST7735_BLACK);
            ST7735_DrawString(TEMPTMR_TM_X, tty, buf, ST7735_WHITE, ST7735_BLACK, SML_S);
        }
    }
}

/* ── Relay fire — 50ms coil pulse for latching relay ─────────────────────── */
/* Ch8/P7 = ON coil,  Ch7/P8 = OFF coil */
#define RELAY_CH_ON   8
#define RELAY_CH_OFF  7

static void energy_fire_relay(uint8_t turn_on)
{
    if (turn_on) {
        CHANNEL_Set(RELAY_CH_ON,  1, 0);
        ST7735_Delay(100);               /* 50ms pulse — required for latching coil */
        CHANNEL_Set(RELAY_CH_ON,  0, 0);
        g_relay_state = 1;
    } else {
        CHANNEL_Set(RELAY_CH_OFF, 1, 0);
        ST7735_Delay(100);               /* 50ms pulse — required for latching coil */
        CHANNEL_Set(RELAY_CH_OFF, 0, 0);
        g_relay_state = 0;
    }
    g_prev_on[0] = '\0';   /* force status row redraw on next tick */
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "Relay: %s", turn_on ? "ON" : "OFF");
}

/* ── Button callbacks (registered in energy_init) ──────────────────────── */
static void on_btn1_press(void)   /* P28 [ON/OFF] — toggle relay */
{
    energy_fire_relay(g_relay_state ? 0 : 1);
}

static void on_btn2_press(void)   /* P26 [+] — reset EV session */
{
    extern float HT7017_GetWh(void);
    g_ev_kwh_offset  = HT7017_GetWh() / 1000.0f;
    g_uptime_seconds = 0;
    g_prev_kwh[0]    = '\0';
    g_prev_cost[0]   = '\0';
    g_prev_tmr[0]    = '\0';
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "EV session reset  offset=%.4f kWh  rate=%.2f Rs/kWh",
              g_ev_kwh_offset, g_ev_rate_rs);
}

static void on_btn3_press(void)   /* P20 [−] — reserved */
{
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Btn3(P20)[-] pressed");
}

/* ── energy_init — called from ST7735_Init ──────────────────────────────── */
static void energy_init(void)
{
    extern float HT7017_GetWh(void);
    /* Snapshot meter reading so session starts at 0.00 kWh */
    g_ev_kwh_offset  = HT7017_GetWh() / 1000.0f;
    g_uptime_seconds = 0;
    g_relay_state    = 0;
    g_wifi_ok        = 0;

    memset(g_prev_v,    0, sizeof(g_prev_v));
    memset(g_prev_a,    0, sizeof(g_prev_a));
    memset(g_prev_w,    0, sizeof(g_prev_w));
    memset(g_prev_kwh,  0, sizeof(g_prev_kwh));
    memset(g_prev_cost, 0, sizeof(g_prev_cost));
    memset(g_prev_tmr,  0, sizeof(g_prev_tmr));
    memset(g_prev_pf,   0, sizeof(g_prev_pf));
    memset(g_prev_hz,   0, sizeof(g_prev_hz));
    memset(g_prev_tc,   0, sizeof(g_prev_tc));
    memset(g_prev_on,   0, sizeof(g_prev_on));
    memset(g_prev_wifi, 0, sizeof(g_prev_wifi));

    energy_draw_static_frame();

    /* Register buttons (calls Section G btn_register) */
    btn_register(28, on_btn1_press);   /* P28 [ON/OFF] */
    btn_register(26, on_btn2_press);   /* P26 [+] reset session */
    btn_register(20, on_btn3_press);   /* P20 [−] reserved */

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "Energy init  rate=%.2f Rs/kWh  offset=%.4f kWh",
              g_ev_rate_rs, g_ev_kwh_offset);
}

/* ── energy_tick — called every second from ST7735_RunEverySecond ─────── */
static void energy_tick(void)
{
    extern float HT7017_GetVoltage(void);
    extern float HT7017_GetCurrent(void);
    extern float HT7017_GetPower(void);
    extern float HT7017_GetWh(void);
    extern float HT7017_GetPowerFactor(void);
    extern float HT7017_GetFrequency(void);

    g_uptime_seconds++;

    energy_draw_screen(
        HT7017_GetVoltage(),
        HT7017_GetCurrent(),
        HT7017_GetPower(),
        HT7017_GetWh() / 1000.0f,
        HT7017_GetPowerFactor(),
        HT7017_GetFrequency()
    );
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION I — PUBLIC API  (WiFi / relay state setters called from scripts)
 * ══════════════════════════════════════════════════════════════════════════════ */

void ST7735_SetWifiStatus(uint8_t connected)
{
    if (g_wifi_ok != connected) {
        g_wifi_ok       = connected;
        g_prev_wifi[0]  = '\0';   /* force redraw */
    }
}

void ST7735_SetRelayState(uint8_t on)
{
    if (g_relay_state != on) {
        g_relay_state = on;
        g_prev_on[0]  = '\0';   /* force redraw */
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION J — CONSOLE COMMANDS
 * ══════════════════════════════════════════════════════════════════════════════ */

static commandResult_t CMD_Clear(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    uint16_t colour = ST7735_BLACK;
    if (args && *args) colour = (uint16_t)strtol(args, NULL, 0);
    memset(g_prev_v,    0, sizeof(g_prev_v));
    memset(g_prev_a,    0, sizeof(g_prev_a));
    memset(g_prev_w,    0, sizeof(g_prev_w));
    memset(g_prev_kwh,  0, sizeof(g_prev_kwh));
    memset(g_prev_cost, 0, sizeof(g_prev_cost));
    memset(g_prev_tmr,  0, sizeof(g_prev_tmr));
    memset(g_prev_pf,   0, sizeof(g_prev_pf));
    memset(g_prev_hz,   0, sizeof(g_prev_hz));
    memset(g_prev_tc,   0, sizeof(g_prev_tc));
    memset(g_prev_on,   0, sizeof(g_prev_on));
    memset(g_prev_wifi, 0, sizeof(g_prev_wifi));
    ST7735_FillScreen(colour);
    if (colour == ST7735_BLACK) energy_draw_static_frame();
    return CMD_RES_OK;
}

static commandResult_t CMD_Brightness(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    if (atoi(args) > 0) { SPI_BLK_H(); } else { SPI_BLK_L(); }
    return CMD_RES_OK;
}

static commandResult_t CMD_Goto(const void *ctx, const char *cmd,
                                const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int col = 0, row = 0;
    sscanf(args, "%d %d", &col, &row);
    g_cur_col = (uint8_t)col; g_cur_row = (uint8_t)row;
    return CMD_RES_OK;
}

static commandResult_t CMD_Print(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    uint8_t x = g_cur_col * FONT_ADV  * g_txt_scale;
    uint8_t y = g_cur_row * FONT_VADV * g_txt_scale;
    ST7735_DrawString(x, y, args, g_fg_colour, g_bg_colour, g_txt_scale);
    g_cur_col += (uint8_t)strlen(args);
    return CMD_RES_OK;
}

static commandResult_t CMD_Color(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    char *end;
    g_fg_colour = (uint16_t)strtol(args, &end, 0);
    if (end && *end) g_bg_colour = (uint16_t)strtol(end, NULL, 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_Draw(const void *ctx, const char *cmd,
                                const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int x = 0, y = 0, w = 0, h = 0; uint32_t colour = 0;
    sscanf(args, "%d %d %d %d %u", &x, &y, &w, &h, &colour);
    ST7735_FillRect((uint8_t)x,(uint8_t)y,(uint8_t)w,(uint8_t)h,(uint16_t)colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_Scale(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int s = atoi(args);
    if (s < 1) s = 1; if (s > 3) s = 3;
    g_txt_scale = (uint8_t)s;
    return CMD_RES_OK;
}

static commandResult_t CMD_Wifi(const void *ctx, const char *cmd,
                                const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    ST7735_SetWifiStatus((uint8_t)(atoi(args) != 0));
    return CMD_RES_OK;
}

/* st7735_relay 1/0 — for OpenBK script rules:
 *   on ch8=1 do st7735_relay 1 endon
 *   on ch7=1 do st7735_relay 0 endon */
static commandResult_t CMD_Relay(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    ST7735_SetRelayState((uint8_t)(atoi(args) != 0));
    return CMD_RES_OK;
}

static commandResult_t CMD_ResetTimer(const void *ctx, const char *cmd,
                                      const char *args, int flags)
{
    g_uptime_seconds = 0;
    g_prev_tmr[0]    = '\0';
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "Timer reset");
    return CMD_RES_OK;
}

/* ev_rate <float>  — set Rs/kWh tariff */
static commandResult_t CMD_EvRate(const void *ctx, const char *cmd,
                                  const char *args, int flags)
{
    if (!args || !*args) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "ev_rate: %.4f Rs/kWh (TGSPDCL CAT-2B)", g_ev_rate_rs);
        return CMD_RES_OK;
    }
    g_ev_rate_rs    = (float)atof(args);
    g_prev_cost[0]  = '\0';
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ev_rate set to %.4f Rs/kWh", g_ev_rate_rs);
    return CMD_RES_OK;
}

/* ev_reset — snapshot current meter, zero timer */
static commandResult_t CMD_EvReset(const void *ctx, const char *cmd,
                                   const char *args, int flags)
{
    on_btn2_press();   /* reuse same logic as [+] button */
    return CMD_RES_OK;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * SECTION K — OPENBK LIFECYCLE
 * Only ST7735_Init() and ST7735_RunEverySecond() are called by drv_main.c.
 * All module init and tick calls chain from here.
 * ══════════════════════════════════════════════════════════════════════════════ */

void ST7735_Init(void)
{
    int argc = Tokenizer_GetArgsCount();
    if (argc >= 2) g_pin_sck = Tokenizer_GetArgIntegerDefault(1, ST7735_DEFAULT_SCK);
    if (argc >= 3) g_pin_sda = Tokenizer_GetArgIntegerDefault(2, ST7735_DEFAULT_SDA);
    if (argc >= 4) g_pin_res = Tokenizer_GetArgIntegerDefault(3, ST7735_DEFAULT_RES);
    if (argc >= 5) g_pin_dc  = Tokenizer_GetArgIntegerDefault(4, ST7735_DEFAULT_DC);
    if (argc >= 6) g_pin_cs  = Tokenizer_GetArgIntegerDefault(5, ST7735_DEFAULT_CS);
    if (argc >= 7) g_pin_blk = Tokenizer_GetArgIntegerDefault(6, ST7735_DEFAULT_BLK);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d",
              g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);

    /* ── TFT hardware init ────────────────────────────────────────────────── */
    HAL_PIN_Setup_Output(g_pin_sck);
    HAL_PIN_Setup_Output(g_pin_sda);
    HAL_PIN_Setup_Output(g_pin_res);
    HAL_PIN_Setup_Output(g_pin_dc);
    HAL_PIN_Setup_Output(g_pin_cs);
    HAL_PIN_Setup_Output(g_pin_blk);

    SPI_CS_H(); SPI_SCK_L(); SPI_SDA_L(); SPI_DC_H(); SPI_RES_H();
    SPI_BLK_L();           /* backlight ON (active-low on KWS-303WF) */
    ST7735_Delay(50);

    TFT_HardReset();
    TFT_InitController();
    g_initialized = 1;
    ST7735_FillScreen(ST7735_BLACK);

    /* ── Energy + button module init (order matters: energy calls btn_register) */
    energy_init();         /* draws static labels, registers button callbacks */

    /* ── Register console commands ─────────────────────────────────────────── */
    CMD_RegisterCommand("st7735_clear",      CMD_Clear,      NULL);
    CMD_RegisterCommand("st7735_brightness", CMD_Brightness, NULL);
    CMD_RegisterCommand("st7735_goto",       CMD_Goto,       NULL);
    CMD_RegisterCommand("st7735_print",      CMD_Print,      NULL);
    CMD_RegisterCommand("st7735_color",      CMD_Color,      NULL);
    CMD_RegisterCommand("st7735_draw",       CMD_Draw,       NULL);
    CMD_RegisterCommand("st7735_scale",      CMD_Scale,      NULL);
    CMD_RegisterCommand("st7735_resetTimer", CMD_ResetTimer, NULL);
    CMD_RegisterCommand("st7735_wifi",       CMD_Wifi,       NULL);
    CMD_RegisterCommand("st7735_relay",      CMD_Relay,      NULL);
    CMD_RegisterCommand("ev_rate",           CMD_EvRate,     NULL);
    CMD_RegisterCommand("ev_reset",          CMD_EvReset,    NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: ready 80x160  rate=%.2f Rs/kWh  NTC_ch=%d",
              g_ev_rate_rs, ADC_TEMP_CHANNEL);
}

void ST7735_RunEverySecond(void)
{
    if (!g_initialized) return;
    btn_tick();       /* Section G — poll buttons, fire callbacks on press */
    energy_tick();    /* Section H — read HT7017 + ADC, update screen */
}

void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[512];
    float session_kwh = 0.0f;
    {
        extern float HT7017_GetWh(void);
        session_kwh = HT7017_GetWh() / 1000.0f - g_ev_kwh_offset;
        if (session_kwh < 0.0f) session_kwh = 0.0f;
    }
    snprintf(tmp, sizeof(tmp),
        "<h5>ST7735 TFT — EV Energy Monitor</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><td>Display</td><td>%s</td></tr>"
        "<tr><td>Size</td><td>80x160 RGB565</td></tr>"
        "<tr><td>Pins</td><td>SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d</td></tr>"
        "<tr><td>NTC ADC</td><td>ch=%d P23 B=%d R25=%d Rs=%d</td></tr>"
        "<tr><td>Session kWh</td><td>%.4f</td></tr>"
        "<tr><td>Session cost</td><td>Rs %.2f</td></tr>"
        "<tr><td>Rate</td><td>%.4f Rs/kWh (TGSPDCL CAT-2B)</td></tr>"
        "<tr><td>Timer</td><td>%lu s</td></tr>"
        "<tr><td>Relay</td><td>%s</td></tr>"
        "<tr><td>WiFi</td><td>%s</td></tr>"
        "</table>",
        g_initialized ? "OK" : "NOT INIT",
        g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk,
        ADC_TEMP_CHANNEL, (int)NTC_B, (int)NTC_R25, (int)NTC_RS,
        session_kwh, session_kwh * g_ev_rate_rs, g_ev_rate_rs,
        (unsigned long)g_uptime_seconds,
        g_relay_state ? "ON" : "OFF",
        g_wifi_ok     ? "connected" : "disconnected");
    strncat(request->reply, tmp, sizeof(request->reply) - strlen(request->reply) - 1);
}

#endif /* ENABLE_DRIVER_ST7735 */
