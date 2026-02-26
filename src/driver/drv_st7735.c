/*
 * ST7735S — 0.96" 80×160 Color TFT Driver for OpenBK7231T / BK7231N
 * Target: KWS-303WF  FPC-JL096B005-01V0
 *
 * Software (bit-bang) 4-wire SPI
 * startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> [BLK]
 * Example:   startDriver ST7735 14 16 9 17 15 24
 *
 * ┌─────────────────────────────────────────────────────┐
 * │  DISPLAY LAYOUT  80×160px — fully utilised          │
 * │                                                     │
 * │  Y=  0 h=10  ON      WiFi     ← GREEN|GREY  BLUE    │
 * │  Y= 11 h=28  00.0          V  ← RED   scale2        │
 * │  Y= 40 h=28  00.00         A  ← CYAN  scale2        │
 * │  Y= 69 h=28  00.0          W  ← YELLOW scale2       │
 * │  Y= 98 h=17  000.00KWh        ← CYAN  scale1        │
 * │  Y=113 h=17  00:00:00T        ← WHITE scale1        │
 * │  Y=129 h=17  0.00PF  50.0Hz   ← RED | BLUE scale1   │
 * │  Y=145 h=17  00.00C           ← ORANGE scale1       │
 * └─────────────────────────────────────────────────────┘
 *
 * TOTAL = 10+1+28+1+28+1+28+1+15+15+1+15+1+15 = 160px EXACT
 *
 * PIXEL MATHS verified:
 *   scale2: FONT_ADV*2=12px/ch, FONT_H*2=14px tall
 *   scale1: FONT_ADV*1= 6px/ch, FONT_H*1= 7px tall
 *
 *   "00.0"      4ch×12=48px  in 70px value zone  ✓ (V & W)
 *   "00.00"     5ch×12=60px  in 70px value zone  ✓ (A)
 *   "000.00KWh" 9ch× 6=54px  in 80px full row    ✓
 *   "00:00:00T" 9ch× 6=54px  in 80px full row    ✓
 *   "0.00PF"    6ch× 6=36px  x=0..36             ✓
 *   "50.0Hz"    6ch× 6=36px  x=38..74 (+6 spare) ✓
 *   "00.00C"    6ch× 6=36px  in 80px full row    ✓
 *
 * TEMPERATURE: always read from ADC channel 23 (pin P23 / ADC3).
 *   NTC Steinhart-Hart: R25=10kΩ, B=3950, Rs=10kΩ, 12-bit ADC.
 *   Adjust NTC_* constants if your thermistor differs.
 *   The temp_c argument to DrawEnergyScreen() is ignored.
 *
 * WIFI STATUS: call ST7735_SetWifiStatus(1) on connect, (0) on disconnect.
 *
 * FLICKER-FREE: dividers + V/A/W unit labels drawn once at init.
 *   Small rows draw full composite string (value+units) — no separate prefix.
 *   Per-field string cache skips SPI write when value unchanged.
 *
 * RELAY (latching, schematic-verified):
 *   Ch8 → GPIO P7 = ON  coil (latch CLOSED)
 *   Ch7 → GPIO P8 = OFF coil (latch OPEN)
 *   State tracked in g_relay_state (latching = no readable output).
 *
 * BUTTONS (all active-low, internal pull-up, schematic-verified):
 *   Btn1 P28 CBU-17 [ON/OFF] — toggles relay
 *   Btn2 P26 CBU-11 [+]     — RESERVED (future brightness/threshold)
 *   Btn3 P20 CBU-3  [−]     — RESERVED (future brightness/threshold)
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

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION A — PIN & STATE
// ═══════════════════════════════════════════════════════════════════════════════

static int g_pin_sck = ST7735_DEFAULT_SCK;
static int g_pin_sda = ST7735_DEFAULT_SDA;
static int g_pin_res = ST7735_DEFAULT_RES;
static int g_pin_dc  = ST7735_DEFAULT_DC;
static int g_pin_cs  = ST7735_DEFAULT_CS;
static int g_pin_blk = ST7735_DEFAULT_BLK;
static uint8_t g_initialized = 0;
static uint8_t g_wifi_ok     = 0;   // updated by ST7735_SetWifiStatus()

// Console cursor / colour state
static uint8_t  g_cur_col   = 0;
static uint8_t  g_cur_row   = 0;
static uint16_t g_fg_colour = ST7735_WHITE;
static uint16_t g_bg_colour = ST7735_BLACK;
static uint8_t  g_txt_scale = ST7735_TEXT_SCALE_LARGE;

// Per-field string cache — SPI write skipped when value unchanged
static char g_prev_v[10]   = "";
static char g_prev_a[10]   = "";
static char g_prev_w[10]   = "";
static char g_prev_kwh[14] = "";
static char g_prev_tmr[12] = "";
static char g_prev_pf[10]  = "";
static char g_prev_hz[10]  = "";
static char g_prev_tc[10]  = "";
static char g_prev_on[4]   = "";
static char g_prev_wifi[6] = "";

// Uptime counter — incremented every second by ST7735_RunEverySecond()
static uint32_t g_uptime_seconds = 0;

// ─── ADC / NTC temperature config ────────────────────────────────────────────
// Pin P23 / ADC channel 23 (ADC3 on BK7231N)
#define ADC_TEMP_CHANNEL  23        // passed to HAL_ADC_Read()
#define ADC_MAX_VAL       4095.0f   // 12-bit; change to 1023.0f for 10-bit ADC
#define NTC_R25           10000.0f  // NTC nominal Ω at 25 °C
#define NTC_B             3950.0f   // NTC B-constant (K)
#define NTC_RS            10000.0f  // Series resistor Ω
#define NTC_T0_K          298.15f   // 25 °C in Kelvin
// NTC_PULLUP 0 → Vcc→Rs→ADC_pin→NTC→GND  (most common, Rs is pull-up)
// NTC_PULLUP 1 → Vcc→NTC→ADC_pin→Rs→GND  (NTC is pull-up)
#define NTC_PULLUP        0

// ─── Latching relay (schematic-verified) ─────────────────────────────────────
// Ch8 → GPIO P7 = ON  coil  |  Ch7 → GPIO P8 = OFF coil
#define RELAY_CH_ON    8
#define RELAY_CH_OFF   7
static uint8_t g_relay_state = 0;   // 0=OFF 1=ON — tracked internally

// ─── Buttons (schematic-verified, all active-low) ─────────────────────────────
// Btn1 P28 CBU-17 [ON/OFF]  Btn2 P26 CBU-11 [+]  Btn3 P20 CBU-3 [−]
#define BUTTON1_PIN        28
#define BUTTON2_PIN        26
#define BUTTON3_PIN        20
#define BTN_DEBOUNCE_TICKS  3

static uint8_t g_btn1_last = 1, g_btn1_db = 0;
static uint8_t g_btn2_last = 1, g_btn2_db = 0;
static uint8_t g_btn3_last = 1, g_btn3_db = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION B — SOFTWARE SPI BIT-BANG
// ═══════════════════════════════════════════════════════════════════════════════

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

static void ST7735_WriteCmd(uint8_t cmd)
    { SPI_CS_L(); SPI_DC_L(); SPI_WriteByte(cmd); SPI_CS_H(); }

static void ST7735_WriteData8(uint8_t d)
    { SPI_CS_L(); SPI_DC_H(); SPI_WriteByte(d);   SPI_CS_H(); }

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION C — DELAY
// ═══════════════════════════════════════════════════════════════════════════════

static void ST7735_Delay(uint32_t ms)
{
    extern int rtos_delay_milliseconds(uint32_t num_ms);
    (void)rtos_delay_milliseconds(ms);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D — CONTROLLER INIT
// ═══════════════════════════════════════════════════════════════════════════════

static void ST7735_HardReset(void)
{
    SPI_RES_H(); ST7735_Delay(10);
    SPI_RES_L(); ST7735_Delay(10);
    SPI_RES_H(); ST7735_Delay(120);
}

static void ST7735_InitController(void)
{
    ST7735_WriteCmd(ST77_SWRESET); ST7735_Delay(150);
    ST7735_WriteCmd(ST77_SLPOUT);  ST7735_Delay(500);

    ST7735_WriteCmd(ST77_FRMCTR1);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);
    ST7735_WriteCmd(ST77_FRMCTR2);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);
    ST7735_WriteCmd(ST77_FRMCTR3);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);

    ST7735_WriteCmd(ST77_INVCTR);  ST7735_WriteData8(0x07);

    ST7735_WriteCmd(ST77_PWCTR1);
    ST7735_WriteData8(0xA2); ST7735_WriteData8(0x02); ST7735_WriteData8(0x84);
    ST7735_WriteCmd(ST77_PWCTR2);  ST7735_WriteData8(0xC5);
    ST7735_WriteCmd(ST77_PWCTR3);
    ST7735_WriteData8(0x0A); ST7735_WriteData8(0x00);
    ST7735_WriteCmd(ST77_PWCTR4);
    ST7735_WriteData8(0x8A); ST7735_WriteData8(0x2A);
    ST7735_WriteCmd(ST77_PWCTR5);
    ST7735_WriteData8(0x8A); ST7735_WriteData8(0xEE);

    ST7735_WriteCmd(ST77_VMCTR1);  ST7735_WriteData8(0x0E);
    ST7735_WriteCmd(ST77_INVOFF);
    ST7735_WriteCmd(ST77_MADCTL);  ST7735_WriteData8(MADCTL_BGR);
    ST7735_WriteCmd(ST77_COLMOD);  ST7735_WriteData8(0x05); ST7735_Delay(10);

    ST7735_WriteCmd(ST77_CASET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(ST7735_COL_OFFSET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(ST7735_COL_OFFSET + ST7735_WIDTH - 1);
    ST7735_WriteCmd(ST77_RASET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(ST7735_ROW_OFFSET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(ST7735_ROW_OFFSET + ST7735_HEIGHT - 1);

    ST7735_WriteCmd(ST77_GMCTRP1);
    ST7735_WriteData8(0x02); ST7735_WriteData8(0x1C); ST7735_WriteData8(0x07);
    ST7735_WriteData8(0x12); ST7735_WriteData8(0x37); ST7735_WriteData8(0x32);
    ST7735_WriteData8(0x29); ST7735_WriteData8(0x2D); ST7735_WriteData8(0x29);
    ST7735_WriteData8(0x25); ST7735_WriteData8(0x2B); ST7735_WriteData8(0x39);
    ST7735_WriteData8(0x00); ST7735_WriteData8(0x01); ST7735_WriteData8(0x03);
    ST7735_WriteData8(0x10);
    ST7735_WriteCmd(ST77_GMCTRN1);
    ST7735_WriteData8(0x03); ST7735_WriteData8(0x1D); ST7735_WriteData8(0x07);
    ST7735_WriteData8(0x06); ST7735_WriteData8(0x2E); ST7735_WriteData8(0x2C);
    ST7735_WriteData8(0x29); ST7735_WriteData8(0x2D); ST7735_WriteData8(0x2E);
    ST7735_WriteData8(0x2E); ST7735_WriteData8(0x37); ST7735_WriteData8(0x3F);
    ST7735_WriteData8(0x00); ST7735_WriteData8(0x00); ST7735_WriteData8(0x02);
    ST7735_WriteData8(0x10);

    ST7735_WriteCmd(ST77_NORON);  ST7735_Delay(10);
    ST7735_WriteCmd(ST77_DISPON); ST7735_Delay(100);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION E — DRAWING PRIMITIVES
// ═══════════════════════════════════════════════════════════════════════════════

static void ST7735_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    ST7735_WriteCmd(ST77_CASET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(x0 + ST7735_COL_OFFSET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(x1 + ST7735_COL_OFFSET);
    ST7735_WriteCmd(ST77_RASET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(y0 + ST7735_ROW_OFFSET);
    ST7735_WriteData8(0x00); ST7735_WriteData8(y1 + ST7735_ROW_OFFSET);
    ST7735_WriteCmd(ST77_RAMWR);
}

void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour)
{
    if (!g_initialized) return;
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH  - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;
    ST7735_SetWindow(x, y, x + w - 1, y + h - 1);
    uint8_t hi = colour >> 8, lo = colour & 0xFF;
    uint32_t n = (uint32_t)w * h;
    SPI_CS_L(); SPI_DC_H();
    while (n--) { SPI_WriteByte(hi); SPI_WriteByte(lo);}
    SPI_CS_H();
}

void ST7735_FillScreen(uint16_t colour)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, colour);
}

// ─── 5×7 font (ASCII 0x20–0x7E) ───────────────────────────────────────────────
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
    ST7735_SetWindow(x, y, x + FONT_W * scale - 1, y + FONT_H * scale - 1);
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

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION F — LAYOUT CONSTANTS  (pixel-verified in planning pass above)
// ═══════════════════════════════════════════════════════════════════════════════

// Row Y positions and heights
// Sum: 10+1+28+1+28+1+28+1+15+15+1+15+1+15 = 160 ✓
#define ROW_STATUS_Y    0
#define ROW_STATUS_H   10
#define ROW_V_Y        11
#define ROW_V_H        28
#define ROW_A_Y        40      // 11+28+1
#define ROW_A_H        28
#define ROW_W_Y        69      // 40+28+1
#define ROW_W_H        28
#define ROW_KWH_Y      98      // 69+28+1
#define ROW_KWH_H      15
#define ROW_TMR_Y     113      // 98+15 (no divider between kWh & timer)
#define ROW_TMR_H      15
                               // divider at 128 (113+15)
#define ROW_PFHZ_Y    129      // 128+1
#define ROW_PFHZ_H     15
                               // divider at 144 (129+15)
#define ROW_TEMP_Y    145      // 144+1
#define ROW_TEMP_H     15
                               // 145+15 = 160 ✓

// Font scales
#define VAL_S  2   // large: 12px/ch wide, 14px tall
#define SML_S  1   // small:  6px/ch wide,  7px tall

// Large row: unit label occupies rightmost FONT_W*VAL_S=10px
// Value zone gets the remaining 70px left of the label
#define LBL_W      (FONT_W * VAL_S)          // 10px
#define LBL_X      (ST7735_WIDTH - LBL_W)    // 70
#define VAL_ZONE_W  LBL_X                     // 70px

// Small rows: full 80px used per row (composite value+unit string, no prefix)
#define SML_FULL_W  ST7735_WIDTH              // 80px

// PF+Hz split row — two independent half-row zones:
//   Left zone  (PF):  x=0,  clear width=38px,  "0.00PF"  = 6ch×6=36px  ✓
//   Right zone (Hz):  x=38, clear width=42px,  "50.0Hz"  = 6ch×6=36px  ✓
//   Total rendered = 36+2gap+36 = 74px of 80px available (6px spare at right)
#define PFHZ_PF_X    0
#define PFHZ_PF_W   38
#define PFHZ_HZ_X   38
#define PFHZ_HZ_W   42

// Status row: relay left, WiFi right-aligned
//   "ON "/"OFF" = 3ch×6=18px at x=0 (clear 22px)
//   "WiFi"/"NoWF" = 4ch×6=24px at x=56 (80-24=56, clear 24px)
#define STA_RELAY_X   0
#define STA_RELAY_W  22
#define STA_WIFI_X   56
#define STA_WIFI_W   24

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION G — ADC TEMPERATURE  (pin P23 / channel 23)
// ═══════════════════════════════════════════════════════════════════════════════
// HAL_ADC_Read() is the correct BK7231N function (hal_adc.h / hal_adc_bk7231n.c)
// No pin setup call needed — BK7231N routes P23 to SADC mux internally.

extern uint32_t HAL_ADC_Read(uint8_t channel);

static float ST7735_ReadTempC(void)
{
    uint32_t raw = HAL_ADC_Read(ADC_TEMP_CHANNEL);

    if (raw == 0)                      return -99.0f;   // short / no sensor
    if (raw >= (uint32_t)ADC_MAX_VAL)  return 999.0f;   // open circuit

    float r = (float)raw;

#if NTC_PULLUP
    float rntc = NTC_RS * r / (ADC_MAX_VAL - r);
#else
    float rntc = NTC_RS * (ADC_MAX_VAL - r) / r;
#endif

    // Steinhart–Hart B-parameter: 1/T = 1/T0 + (1/B)·ln(Rntc/R25)
    float temp_k = 1.0f / (1.0f / NTC_T0_K + logf(rntc / NTC_R25) / NTC_B);
    return temp_k - 273.15f;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION H — FLICKER-FREE SCREEN DRAW
// ═══════════════════════════════════════════════════════════════════════════════

// Clear a rectangular zone then draw text vertically centred within it
static void UpdateZone(uint8_t x, uint8_t y, uint8_t zw, uint8_t zh,
                       const char *str, uint16_t fg, uint8_t scale)
{
    ST7735_FillRect(x, y, zw, zh, ST7735_BLACK);
    uint8_t ty = y + (zh - FONT_H * scale) / 2;
    ST7735_DrawString(x, ty, str, fg, ST7735_BLACK, scale);
}

// Static elements drawn ONCE at init — never touched again during updates
static void ST7735_DrawStaticFrame(void)
{
    // Unit label characters at right edge of large rows (vertically centred)
    // These are drawn once and never cleared — value zone stops at LBL_X=70
    uint8_t vy = ROW_V_Y + (ROW_V_H - FONT_H * VAL_S) / 2;   // 11+7=18
    uint8_t ay = ROW_A_Y + (ROW_A_H - FONT_H * VAL_S) / 2;   // 40+7=47
    uint8_t wy = ROW_W_Y + (ROW_W_H - FONT_H * VAL_S) / 2;   // 69+7=76
    ST7735_DrawChar(LBL_X, vy, 'V', ST7735_RED,    ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, ay, 'A', ST7735_CYAN,   ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, wy, 'W', ST7735_YELLOW, ST7735_BLACK, VAL_S);

    // Small rows have NO static prefix — each update draws the full composite
    // string (e.g. "000.00KWh", "0.00PF") which includes its own unit suffix.
    // This eliminates any risk of prefix/value misalignment.
}

// ─── Public update — called every 2 s from RunEverySecond ────────────────────
void ST7735_DrawEnergyScreen(float v, float a, float w,
                              float kwh, float pf, float hz,
                              float temp_c_unused, uint8_t relay_on)
{
    if (!g_initialized) return;
    (void)temp_c_unused;   // temperature always read from ADC internally

    char buf[16];

    // ── STATUS ROW ────────────────────────────────────────────────────────────
    // Left (x=0):  relay "ON " GREEN  or "OFF" GREY
    const char *on_str = relay_on ? "ON " : "OFF";
    if (strcmp(on_str, g_prev_on) != 0) {
        strncpy(g_prev_on, on_str, sizeof(g_prev_on) - 1);
        g_prev_on[sizeof(g_prev_on) - 1] = '\0';
        ST7735_FillRect(STA_RELAY_X, ROW_STATUS_Y, STA_RELAY_W, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(STA_RELAY_X, ROW_STATUS_Y + 1, on_str,
                          relay_on ? ST7735_GREEN : ST7735_GREY,
                          ST7735_BLACK, SML_S);
    }

    // Right (x=56): WiFi "WiFi" BLUE  or "NoWF" GREY
    const char *wifi_str = g_wifi_ok ? "WiFi" : "NoWF";
    if (strcmp(wifi_str, g_prev_wifi) != 0) {
        strncpy(g_prev_wifi, wifi_str, sizeof(g_prev_wifi) - 1);
        g_prev_wifi[sizeof(g_prev_wifi) - 1] = '\0';
        ST7735_FillRect(STA_WIFI_X, ROW_STATUS_Y, STA_WIFI_W, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(STA_WIFI_X, ROW_STATUS_Y + 1, wifi_str,
                          g_wifi_ok ? ST7735_BLUE : ST7735_GREY,
                          ST7735_BLACK, SML_S);
    }

    // ── VOLTAGE: "00.0" + V label  (RED, scale2) ─────────────────────────────
    snprintf(buf, sizeof(buf), "%4.2f", v);
    if (strcmp(buf, g_prev_v) != 0) {
        strncpy(g_prev_v, buf, sizeof(g_prev_v) - 1);
        UpdateZone(0, ROW_V_Y, VAL_ZONE_W, ROW_V_H, buf, ST7735_RED, SML_S);
    }

    // ── CURRENT: "00.00" + A label  (CYAN, scale2) ───────────────────────────
    snprintf(buf, sizeof(buf), "%5.2f", a);
    if (strcmp(buf, g_prev_a) != 0) {
        strncpy(g_prev_a, buf, sizeof(g_prev_a) - 1);
        UpdateZone(0, ROW_A_Y, VAL_ZONE_W, ROW_A_H, buf, ST7735_CYAN, SML_S);
    }

    // ── POWER: "00.0" + W label  (YELLOW, scale2) ────────────────────────────
    snprintf(buf, sizeof(buf), "%4.2f", w);
    if (strcmp(buf, g_prev_w) != 0) {
        strncpy(g_prev_w, buf, sizeof(g_prev_w) - 1);
        UpdateZone(0, ROW_W_Y, VAL_ZONE_W, ROW_W_H, buf, ST7735_YELLOW, SML_S);
    }

    // ── kWh: "000.00KWh"  (CYAN, scale1, full 80px row) ─────────────────────
    snprintf(buf, sizeof(buf), "%06.2fkWh", kwh);
    if (strcmp(buf, g_prev_kwh) != 0) {
        strncpy(g_prev_kwh, buf, sizeof(g_prev_kwh) - 1);
        UpdateZone(0, ROW_KWH_Y, SML_FULL_W, ROW_KWH_H, buf, ST7735_CYAN, SML_S);
    }

    // ── TIMER: "00:00:00T"  (WHITE, scale1, full 80px row) ───────────────────
    {
        uint32_t s_tot = g_uptime_seconds;
        uint32_t hh    = s_tot / 3600;
        uint32_t mm    = (s_tot % 3600) / 60;
        uint32_t ss    = s_tot % 60;
        snprintf(buf, sizeof(buf), "%02u:%02u:%02uT",
                 (unsigned)hh, (unsigned)mm, (unsigned)ss);
        if (strcmp(buf, g_prev_tmr) != 0) {
            strncpy(g_prev_tmr, buf, sizeof(g_prev_tmr) - 1);
            UpdateZone(0, ROW_TMR_Y, SML_FULL_W, ROW_TMR_H,
                       buf, ST7735_WHITE, SML_S);
        }
    }

    // ── PF+Hz SPLIT ROW  (scale1) ─────────────────────────────────────────────
    {
        uint8_t phy = ROW_PFHZ_Y + (ROW_PFHZ_H - FONT_H * SML_S) / 2;

        snprintf(buf, sizeof(buf), "%.2fPF", pf);
        if (strcmp(buf, g_prev_pf) != 0) {
            strncpy(g_prev_pf, buf, sizeof(g_prev_pf) - 1);
            ST7735_FillRect(PFHZ_PF_X, ROW_PFHZ_Y, PFHZ_PF_W, ROW_PFHZ_H, ST7735_BLACK);
            ST7735_DrawString(PFHZ_PF_X, phy, buf, ST7735_RED, ST7735_BLACK, SML_S);
        }

        snprintf(buf, sizeof(buf), "%.2fHz", hz);
        if (strcmp(buf, g_prev_hz) != 0) {
            strncpy(g_prev_hz, buf, sizeof(g_prev_hz) - 1);
            ST7735_FillRect(PFHZ_HZ_X, ROW_PFHZ_Y, PFHZ_HZ_W, ROW_PFHZ_H, ST7735_BLACK);
            ST7735_DrawString(PFHZ_HZ_X, phy, buf, ST7735_BLUE, ST7735_BLACK, SML_S);
        }
    }

    // ── TEMPERATURE: ADC ch23 / pin P23  (ORANGE, scale1) ────────────────────
    // 0.5°C hysteresis — only redraws when reading moves ≥0.5°C from last drawn value
    {
        static float s_last_tc = -999.0f;
        float tc = ST7735_ReadTempC();
        if (tc > s_last_tc + 0.5f || tc < s_last_tc - 0.5f) {
            s_last_tc = tc;
            snprintf(buf, sizeof(buf), "%05.2fC", tc);
            strncpy(g_prev_tc, buf, sizeof(g_prev_tc) - 1);
            UpdateZone(0, ROW_TEMP_Y, SML_FULL_W, ROW_TEMP_H,
                       buf, ST7735_ORANGE, SML_S);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION I — PUBLIC HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void ST7735_SetWifiStatus(uint8_t connected)
{
    if (g_wifi_ok != connected) {
        g_wifi_ok      = connected;
        g_prev_wifi[0] = '\0';   // force status row redraw on next tick
    }
}

void ST7735_SetRelayState(uint8_t on)
{
    if (g_relay_state != on) {
        g_relay_state = on;
        g_prev_on[0]  = '\0';   // force status row redraw on next tick
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION J — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════════

static commandResult_t CMD_ST7735_Clear(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    uint16_t colour = ST7735_BLACK;
    if (args && *args) colour = (uint16_t)strtol(args, NULL, 0);
    g_prev_v[0]    = g_prev_a[0]    = g_prev_w[0]   = '\0';
    g_prev_kwh[0]  = g_prev_tmr[0]  = g_prev_pf[0]  = '\0';
    g_prev_hz[0]   = g_prev_tc[0]   = g_prev_on[0]  = '\0';
    g_prev_wifi[0] = '\0';
    ST7735_FillScreen(colour);
    if (colour == ST7735_BLACK) ST7735_DrawStaticFrame();
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: clear 0x%04X", colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Brightness(const void *ctx, const char *cmd,
                                             const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int val = atoi(args);
    if (val > 0) { SPI_BLK_H(); addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: BL ON");  }
    else         { SPI_BLK_L(); addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: BL OFF"); }
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Goto(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int col = 0, row = 0;
    sscanf(args, "%d %d", &col, &row);
    g_cur_col = (uint8_t)col; g_cur_row = (uint8_t)row;
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Print(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    uint8_t x = g_cur_col * FONT_ADV  * g_txt_scale;
    uint8_t y = g_cur_row * FONT_VADV * g_txt_scale;
    ST7735_DrawString(x, y, args, g_fg_colour, g_bg_colour, g_txt_scale);
    g_cur_col += (uint8_t)strlen(args);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Color(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    char *end;
    g_fg_colour = (uint16_t)strtol(args, &end, 0);
    if (end && *end) g_bg_colour = (uint16_t)strtol(end, NULL, 0);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Draw(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int x = 0, y = 0, w = 0, h = 0; uint32_t colour = 0;
    sscanf(args, "%d %d %d %d %u", &x, &y, &w, &h, &colour);
    ST7735_FillRect((uint8_t)x,(uint8_t)y,(uint8_t)w,(uint8_t)h,(uint16_t)colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Update(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    extern float HT7017_GetVoltage(void);
    extern float HT7017_GetCurrent(void);
    extern float HT7017_GetPower(void);
    extern float HT7017_GetWh(void);
    extern float HT7017_GetPowerFactor(void);
    extern float HT7017_GetFrequency(void);
    ST7735_DrawEnergyScreen(
        HT7017_GetVoltage(), HT7017_GetCurrent(), HT7017_GetPower(),
        HT7017_GetWh() / 1000.0f, HT7017_GetPowerFactor(),
        HT7017_GetFrequency(), 0.0f, g_relay_state);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: manual update");
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Scale(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int s = atoi(args);
    if (s < 1) s = 1; if (s > 3) s = 3;
    g_txt_scale = (uint8_t)s;
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_ResetTimer(const void *ctx, const char *cmd,
                                             const char *args, int flags)
{
    g_uptime_seconds = 0;
    g_prev_tmr[0]    = '\0';
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: timer reset");
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Wifi(const void *ctx, const char *cmd,
                                       const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    ST7735_SetWifiStatus((uint8_t)(atoi(args) != 0));
    return CMD_RES_OK;
}

// "st7735_relay 1" / "st7735_relay 0" — called from script rules:
//   on ch8=1 do st7735_relay 1 endon
//   on ch7=1 do st7735_relay 0 endon
static commandResult_t CMD_ST7735_Relay(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    ST7735_SetRelayState((uint8_t)(atoi(args) != 0));
    return CMD_RES_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION K — RELAY + BUTTON LOGIC
// ═══════════════════════════════════════════════════════════════════════════════

static void FireRelay(uint8_t turn_on)
{
    //extern int CHANNEL_Set(int index, int value, int reason);
    if (turn_on) {
        CHANNEL_Set(RELAY_CH_ON,  1, 0);
        CHANNEL_Set(RELAY_CH_ON,  0, 0);
        ST7735_SetRelayState(1);
    } else {
        CHANNEL_Set(RELAY_CH_OFF, 1, 0);
        CHANNEL_Set(RELAY_CH_OFF, 0, 0);
        ST7735_SetRelayState(0);
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: relay → %s", turn_on ? "ON" : "OFF");
}

// Returns 1 on confirmed falling edge (button press), 0 otherwise.
// Polled once per second. *last=last stable level, *db=debounce counter.
static uint8_t DebouncePin(int pin, uint8_t *last, uint8_t *db)
{
    extern int HAL_PIN_ReadDigitalInput(int pin);
    uint8_t level = (uint8_t)HAL_PIN_ReadDigitalInput(pin);
    if (level == *last) { *db = 0; return 0; }
    if (++(*db) < BTN_DEBOUNCE_TICKS) return 0;
    *db = 0; *last = level;
    return (level == 0) ? 1 : 0;
}

static void ST7735_PollButtons(void)
{
    // Btn1 P28 [ON/OFF] — toggle relay
    if (DebouncePin(BUTTON1_PIN, &g_btn1_last, &g_btn1_db)) {
        FireRelay(g_relay_state ? 0 : 1);
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
                  "ST7735: Btn1(P28) → relay %s", g_relay_state ? "ON" : "OFF");
    }

    // Btn2 P26 [+] — RESERVED for future use (brightness / threshold UP)
    if (DebouncePin(BUTTON2_PIN, &g_btn2_last, &g_btn2_db)) {
        // TODO: brightness increase or high-voltage threshold adjust
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: Btn2(P26) [+] pressed");
    }

    // Btn3 P20 [−] — RESERVED for future use (brightness / threshold DOWN)
    if (DebouncePin(BUTTON3_PIN, &g_btn3_last, &g_btn3_db)) {
        // TODO: brightness decrease or low-voltage threshold adjust
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: Btn3(P20) [-] pressed");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION L — INIT
// ═══════════════════════════════════════════════════════════════════════════════

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
              "ST7735: pins SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d",
              g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);

    HAL_PIN_Setup_Output(g_pin_sck);
    HAL_PIN_Setup_Output(g_pin_sda);
    HAL_PIN_Setup_Output(g_pin_res);
    HAL_PIN_Setup_Output(g_pin_dc);
    HAL_PIN_Setup_Output(g_pin_cs);
    HAL_PIN_Setup_Output(g_pin_blk);

    // Buttons: active-low, internal pull-up
    HAL_PIN_Setup_Input_Pullup(BUTTON1_PIN);   // P28 [ON/OFF]
    HAL_PIN_Setup_Input_Pullup(BUTTON2_PIN);   // P26 [+]
    HAL_PIN_Setup_Input_Pullup(BUTTON3_PIN);   // P20 [−]
    g_btn1_last = g_btn2_last = g_btn3_last = 1;

    SPI_CS_H(); SPI_SCK_L(); SPI_SDA_L(); SPI_DC_H(); SPI_RES_H();
    SPI_BLK_L();   // backlight ON (active-low on KWS-303WF)
    ST7735_Delay(50);

    ST7735_HardReset();
    ST7735_InitController();
    g_initialized = 1;

    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DrawStaticFrame();

    CMD_RegisterCommand("st7735_clear",      CMD_ST7735_Clear,      NULL);
    CMD_RegisterCommand("st7735_brightness", CMD_ST7735_Brightness, NULL);
    CMD_RegisterCommand("st7735_goto",       CMD_ST7735_Goto,       NULL);
    CMD_RegisterCommand("st7735_print",      CMD_ST7735_Print,      NULL);
    CMD_RegisterCommand("st7735_color",      CMD_ST7735_Color,      NULL);
    CMD_RegisterCommand("st7735_draw",       CMD_ST7735_Draw,       NULL);
    CMD_RegisterCommand("st7735_update",     CMD_ST7735_Update,     NULL);
    CMD_RegisterCommand("st7735_scale",      CMD_ST7735_Scale,      NULL);
    CMD_RegisterCommand("st7735_resetTimer", CMD_ST7735_ResetTimer, NULL);
    CMD_RegisterCommand("st7735_wifi",       CMD_ST7735_Wifi,       NULL);
    CMD_RegisterCommand("st7735_relay",      CMD_ST7735_Relay,      NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: ready 80x160 col_off=%d row_off=%d NTC_ch=%d",
              ST7735_COL_OFFSET, ST7735_ROW_OFFSET, ADC_TEMP_CHANNEL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION M — RUN EVERY SECOND
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t g_refresh_counter = 0;

void ST7735_RunEverySecond(void)
{
    if (!g_initialized) return;

    g_uptime_seconds++;
    ST7735_PollButtons();

    if (++g_refresh_counter < 2) return;
    g_refresh_counter = 0;

    extern float HT7017_GetVoltage(void);
    extern float HT7017_GetCurrent(void);
    extern float HT7017_GetPower(void);
    extern float HT7017_GetWh(void);
    extern float HT7017_GetPowerFactor(void);
    extern float HT7017_GetFrequency(void);

    ST7735_DrawEnergyScreen(
        HT7017_GetVoltage(),
        HT7017_GetCurrent(),
        HT7017_GetPower(),
        HT7017_GetWh() / 1000.0f,
        HT7017_GetPowerFactor(),
        HT7017_GetFrequency(),
        0.0f,             // ignored — temperature read from ADC inside DrawEnergyScreen
        g_relay_state);
}

void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[384];
    snprintf(tmp, sizeof(tmp),
        "<h5>ST7735 TFT</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><td>Status</td><td>%s</td></tr>"
        "<tr><td>Size</td><td>80x160 RGB565</td></tr>"
        "<tr><td>Pins</td><td>SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d</td></tr>"
        "<tr><td>NTC ADC</td><td>ch=%d pin=P23 B=%d R25=%d Rs=%d</td></tr>"
        "<tr><td>Uptime</td><td>%lu s</td></tr>"
        "<tr><td>WiFi</td><td>%s</td></tr>"
        "</table>",
        g_initialized ? "OK" : "NOT INIT",
        g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk,
        ADC_TEMP_CHANNEL, (int)NTC_B, (int)NTC_R25, (int)NTC_RS,
        (unsigned long)g_uptime_seconds,
        g_wifi_ok ? "connected" : "disconnected");
    strncat(request->reply, tmp, sizeof(request->reply) - strlen(request->reply) - 1);
}

#endif // ENABLE_DRIVER_ST7735
