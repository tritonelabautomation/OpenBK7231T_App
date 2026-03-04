/*
 * drv_st7735.c — ST7735S 0.96" 80×160 TFT Display Driver for OpenBK7231N
 * Target: KWS-303WF  FPC-JL096B005-01V0
 *
 * SINGLE RESPONSIBILITY: Read OBK channels → draw pixels on screen.
 * NO relay GPIO.  NO buttons.  NO NTC.  NO session math.  NO MQTT.
 *
 * startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> [BLK]
 * Example:   startDriver ST7735 14 16 9 17 15 24
 *
 * ── CHANNEL INPUT MAP ────────────────────────────────────────────────────
 *   Ch 1  = Voltage      (V  × 100,  e.g. 22150 = 221.50 V)     from drv_ht7017
 *   Ch 2  = Current      (A  × 1000, e.g.   550 =   0.550 A)    from drv_ht7017
 *   Ch 3  = Power        (W  × 10,   e.g.  1217 = 121.7 W)      from drv_ht7017
 *   Ch 4  = Frequency    (Hz × 100,  e.g.  4990 =  49.90 Hz)    from drv_ht7017
 *   Ch 5  = Power Factor (PF × 1000, e.g.   980 =   0.980)      from drv_ht7017
 *   Ch 6  = Energy       (Wh × 10,   e.g.  4700 = 470.0 Wh)     from drv_ht7017
 *   Ch 7  = Alarm state  (0=OK 1=OV 2=UV 3=OC 4=OP)             from drv_ht7017
 *   Ch 8  = Relay state  (0=open/safe, 100=closed/on)            from drv_kws303wf
 *   Ch 9  = Temperature  (°C × 100,  e.g. 2716 = 27.16 °C)      from drv_kws303wf
 *   Ch 10 = EV cost      (Rs × 100,  e.g.  376 =  3.76 Rs)      from drv_kws303wf
 *   Ch 11 = WiFi status  (0=disconnected, 1=connected)           from OBK core
 *
 * ── DISPLAY LAYOUT  80×160px ─────────────────────────────────────────────
 *   Y=  0 h= 10  OFF/ON  [alarm]  NoWF/WiFi    GREY status bar
 *   Y= 11 h= 24  221.50                    V   RED    (scale 2)
 *   Y= 35 h= 24    0.550                   A   CYAN   (scale 2)
 *   Y= 59 h= 24  121.7                     W   YELLOW (scale 2)
 *   Y= 83 h= 23  ₹   3.76                     GREEN  (scale 2)
 *   Y=106 h= 18  000.47kWh                    CYAN   (scale 1)
 *   Y=124 h= 18  0.98PF     49.9Hz            RED  | BLUE  (scale 1)
 *   Y=142 h= 18  27.16C     01:23             ORANGE|WHITE (scale 1)
 *   PIXEL SUM: 10+1+24+24+24+23+18+18+18 = 160px
 *
 * ── RUPEE SYMBOL ₹  (U+20B9) ─────────────────────────────────────────────
 *   UTF-8 encodes ₹ as 0xE2 0x82 0xB9 (3 bytes, all > 0x7E).
 *   DrawString() walks byte-by-byte → DrawChar() ASCII guard fires → "???".
 *
 *   FIX: custom 5×7 glyph at font table index 95, addressed via sentinel
 *   RUPEE_CHAR = '\x01' (ASCII SOH — never used by OBK console/MQTT).
 *   DrawChar() intercepts RUPEE_CHAR before the ASCII range guard.
 *   snprintf format: "\x01%5.2f" → ₹ symbol rendered as one glyph.
 *
 *   Glyph {0x7F,0x50,0x58,0x54,0x62} verified pixel map:
 *     # # # # #    row 0  data bottom → screen TOP bar  (full width)
 *     # . . . #    row 1  data         → screen left stem + right
 *     # # # # .    row 2  data         → screen second bar (4-wide)
 *     # . # . .    row 3  data         → screen diagonal pivot
 *     # . . # .    row 4  data         → screen diagonal
 *     # . . . #    row 5  data         → screen diagonal foot
 *     # . . . .    row 6  data top     → screen BOTTOM anchor
 *   NOTE: glyph is stored vertically mirrored because display
 *   MADCTL has Y-axis inverted (row0 = screen bottom on this hardware)
 *
 *   Cost delta vs "Rs" prefix:
 *     Flash  : +13 bytes (+5 glyph, +8 DrawChar branch)
 *     RAM    : net 0  (p_cost −4, p_alm +4)
 *     CPU    : −1 DrawChar() call per dirty cost-row redraw
 *     Layout : frees 12px in cost row (72px vs 84px at scale 2)
 *
 * ── FIX-ST-1  (alarm indicator) ─────────────────────────────────────────
 *   Original: alarm strings drawn unconditionally every tick without erase.
 *   Fix: dirty-flag cache (p_alm) + full zone_update() erase on change.
 *   alarm raw value clamped to [0,4] before array index use (OOB guard).
 *   Trailing-space padding on alarm strings ensures full zone is erased
 *   on transition (e.g. "OV " → "   " clears all three character columns).
 *
 * ── OPENBK IDIOM ─────────────────────────────────────────────────────────
 *   Dirty-flag pattern: each cell caches its last rendered string.
 *   Redraw only on string change. No polling state. No timers.
 *   CHANNEL_Get() is the sole data source — driver is fully passive.
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

#ifndef LOG_FEATURE_ENERGY
#define LOG_FEATURE_ENERGY LOG_FEATURE_MAIN
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION A — CHANNEL ASSIGNMENTS  (must match other drivers)
 * ══════════════════════════════════════════════════════════════════════════ */
#define DISP_CH_VOLTAGE   1   /* V  × 100   */
#define DISP_CH_CURRENT   2   /* A  × 1000  */
#define DISP_CH_POWER     3   /* W  × 10    */
#define DISP_CH_FREQ      4   /* Hz × 100   */
#define DISP_CH_PF        5   /* PF × 1000  */
#define DISP_CH_ENERGY    6   /* Wh × 10    */
#define DISP_CH_ALARM     7   /* 0-4        */
#define DISP_CH_RELAY     8   /* 0=open 100=closed */
#define DISP_CH_TEMP      9   /* °C × 100   */
#define DISP_CH_EVCOST   10   /* Rs × 100   */
#define DISP_CH_WIFI     11   /* 0/1        */

/* ── Rupee sentinel ──────────────────────────────────────────────────────
 * ASCII 0x01 (SOH) is intercepted by DrawChar() and mapped to the custom
 * ₹ glyph at font table index 95.  The byte never leaves this driver.    */
#define RUPEE_CHAR  '\x01'

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION B — TFT HARDWARE PINS & GLOBAL STATE
 * ══════════════════════════════════════════════════════════════════════════ */
static int     g_pin_sck = ST7735_DEFAULT_SCK;
static int     g_pin_sda = ST7735_DEFAULT_SDA;
static int     g_pin_res = ST7735_DEFAULT_RES;
static int     g_pin_dc  = ST7735_DEFAULT_DC;
static int     g_pin_cs  = ST7735_DEFAULT_CS;
static int     g_pin_blk = ST7735_DEFAULT_BLK;
static uint8_t g_initialized = 0;

/* console text-mode state */
static uint8_t  g_cur_col   = 0;
static uint8_t  g_cur_row   = 0;
static uint16_t g_fg_colour = ST7735_WHITE;
static uint16_t g_bg_colour = ST7735_BLACK;
static uint8_t  g_txt_scale = 1;

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION C — SOFTWARE SPI BIT-BANG
 * ══════════════════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION D — DELAY
 * ══════════════════════════════════════════════════════════════════════════ */
extern int rtos_delay_milliseconds(uint32_t num_ms);

static void ST7735_Delay(uint32_t ms)
{
    (void)rtos_delay_milliseconds(ms);
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION E — ST7735S CONTROLLER INIT
 * ══════════════════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION F — DRAWING PRIMITIVES
 * ══════════════════════════════════════════════════════════════════════════ */
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
    if (x + w > ST7735_WIDTH)  w = (uint8_t)(ST7735_WIDTH  - x);
    if (y + h > ST7735_HEIGHT) h = (uint8_t)(ST7735_HEIGHT - y);
    TFT_SetWindow(x, y, (uint8_t)(x+w-1), (uint8_t)(y+h-1));
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

/* ── 5×7 font table ─────────────────────────────────────────────────────
 *
 *   Indices 0–94  : ASCII 0x20 (' ') through 0x7E ('~')  — standard
 *   Index  95     : ₹ Indian Rupee Sign (U+20B9)          — custom glyph
 *
 *   Storage format: column-major, 5 bytes per glyph, bit 0 = top row.
 *
 *   ₹ glyph  {0x7F, 0x05, 0x0D, 0x15, 0x23}  (index 95):
 *
 *     col:   0     1     2     3     4
 *           0x7A  0x05  0x0D  0x15  0x22
 *
 *     # # # # #    row 0 (data) → renders at screen BOTTOM (Y-flipped display)
 *     # . . . #    row 1 → diag foot
 *     # . . # .    row 2 → diagonal
 *     # . # . .    row 3 → pivot
 *     # # # # .    row 4 → second bar
 *     # . . . #    row 5 → left stem + right (R-top style)
 *     # # # # #    row 6 (data) → renders at screen TOP as full bar
 *   Glyph is vertically mirrored: display Y-axis is inverted on this hardware.
 *
 *   Addressed via RUPEE_CHAR = '\x01' sentinel in DrawChar().
 *   Flash cost of this extra row: +5 bytes.
 * ──────────────────────────────────────────────────────────────────────── */
static const uint8_t g_font5x7[][5] = {
    /* 0x20 ' '  */ {0x00,0x00,0x00,0x00,0x00},
    /* 0x21 '!'  */ {0x00,0x00,0x5F,0x00,0x00},
    /* 0x22 '"'  */ {0x00,0x07,0x00,0x07,0x00},
    /* 0x23 '#'  */ {0x14,0x7F,0x14,0x7F,0x14},
    /* 0x24 '$'  */ {0x24,0x2A,0x7F,0x2A,0x12},
    /* 0x25 '%'  */ {0x23,0x13,0x08,0x64,0x62},
    /* 0x26 '&'  */ {0x36,0x49,0x55,0x22,0x50},
    /* 0x27 '\'' */ {0x00,0x05,0x03,0x00,0x00},
    /* 0x28 '('  */ {0x00,0x1C,0x22,0x41,0x00},
    /* 0x29 ')'  */ {0x00,0x41,0x22,0x1C,0x00},
    /* 0x2A '*'  */ {0x14,0x08,0x3E,0x08,0x14},
    /* 0x2B '+'  */ {0x08,0x08,0x3E,0x08,0x08},
    /* 0x2C ','  */ {0x00,0x50,0x30,0x00,0x00},
    /* 0x2D '-'  */ {0x08,0x08,0x08,0x08,0x08},
    /* 0x2E '.'  */ {0x00,0x60,0x60,0x00,0x00},
    /* 0x2F '/'  */ {0x20,0x10,0x08,0x04,0x02},
    /* 0x30 '0'  */ {0x3E,0x51,0x49,0x45,0x3E},
    /* 0x31 '1'  */ {0x00,0x42,0x7F,0x40,0x00},
    /* 0x32 '2'  */ {0x42,0x61,0x51,0x49,0x46},
    /* 0x33 '3'  */ {0x21,0x41,0x45,0x4B,0x31},
    /* 0x34 '4'  */ {0x18,0x14,0x12,0x7F,0x10},
    /* 0x35 '5'  */ {0x27,0x45,0x45,0x45,0x39},
    /* 0x36 '6'  */ {0x3C,0x4A,0x49,0x49,0x30},
    /* 0x37 '7'  */ {0x01,0x71,0x09,0x05,0x03},
    /* 0x38 '8'  */ {0x36,0x49,0x49,0x49,0x36},
    /* 0x39 '9'  */ {0x06,0x49,0x49,0x29,0x1E},
    /* 0x3A ':'  */ {0x00,0x36,0x36,0x00,0x00},
    /* 0x3B ';'  */ {0x00,0x56,0x36,0x00,0x00},
    /* 0x3C '<'  */ {0x08,0x14,0x22,0x41,0x00},
    /* 0x3D '='  */ {0x14,0x14,0x14,0x14,0x14},
    /* 0x3E '>'  */ {0x00,0x41,0x22,0x14,0x08},
    /* 0x3F '?'  */ {0x02,0x01,0x51,0x09,0x06},
    /* 0x40 '@'  */ {0x32,0x49,0x79,0x41,0x3E},
    /* 0x41 'A'  */ {0x7E,0x11,0x11,0x11,0x7E},
    /* 0x42 'B'  */ {0x7F,0x49,0x49,0x49,0x36},
    /* 0x43 'C'  */ {0x3E,0x41,0x41,0x41,0x22},
    /* 0x44 'D'  */ {0x7F,0x41,0x41,0x22,0x1C},
    /* 0x45 'E'  */ {0x7F,0x49,0x49,0x49,0x41},
    /* 0x46 'F'  */ {0x7F,0x09,0x09,0x09,0x01},
    /* 0x47 'G'  */ {0x3E,0x41,0x49,0x49,0x7A},
    /* 0x48 'H'  */ {0x7F,0x08,0x08,0x08,0x7F},
    /* 0x49 'I'  */ {0x00,0x41,0x7F,0x41,0x00},
    /* 0x4A 'J'  */ {0x20,0x40,0x41,0x3F,0x01},
    /* 0x4B 'K'  */ {0x7F,0x08,0x14,0x22,0x41},
    /* 0x4C 'L'  */ {0x7F,0x40,0x40,0x40,0x40},
    /* 0x4D 'M'  */ {0x7F,0x02,0x0C,0x02,0x7F},
    /* 0x4E 'N'  */ {0x7F,0x04,0x08,0x10,0x7F},
    /* 0x4F 'O'  */ {0x3E,0x41,0x41,0x41,0x3E},
    /* 0x50 'P'  */ {0x7F,0x09,0x09,0x09,0x06},
    /* 0x51 'Q'  */ {0x3E,0x41,0x51,0x21,0x5E},
    /* 0x52 'R'  */ {0x7F,0x09,0x19,0x29,0x46},
    /* 0x53 'S'  */ {0x46,0x49,0x49,0x49,0x31},
    /* 0x54 'T'  */ {0x01,0x01,0x7F,0x01,0x01},
    /* 0x55 'U'  */ {0x3F,0x40,0x40,0x40,0x3F},
    /* 0x56 'V'  */ {0x1F,0x20,0x40,0x20,0x1F},
    /* 0x57 'W'  */ {0x3F,0x40,0x38,0x40,0x3F},
    /* 0x58 'X'  */ {0x63,0x14,0x08,0x14,0x63},
    /* 0x59 'Y'  */ {0x07,0x08,0x70,0x08,0x07},
    /* 0x5A 'Z'  */ {0x61,0x51,0x49,0x45,0x43},
    /* 0x5B '['  */ {0x00,0x7F,0x41,0x41,0x00},
    /* 0x5C '\\' */ {0x02,0x04,0x08,0x10,0x20},
    /* 0x5D ']'  */ {0x00,0x41,0x41,0x7F,0x00},
    /* 0x5E '^'  */ {0x04,0x02,0x01,0x02,0x04},
    /* 0x5F '_'  */ {0x40,0x40,0x40,0x40,0x40},
    /* 0x60 '`'  */ {0x00,0x01,0x02,0x04,0x00},
    /* 0x61 'a'  */ {0x20,0x54,0x54,0x54,0x78},
    /* 0x62 'b'  */ {0x7F,0x48,0x44,0x44,0x38},
    /* 0x63 'c'  */ {0x38,0x44,0x44,0x44,0x20},
    /* 0x64 'd'  */ {0x38,0x44,0x44,0x48,0x7F},
    /* 0x65 'e'  */ {0x38,0x54,0x54,0x54,0x18},
    /* 0x66 'f'  */ {0x08,0x7E,0x09,0x01,0x02},
    /* 0x67 'g'  */ {0x0C,0x52,0x52,0x52,0x3E},
    /* 0x68 'h'  */ {0x7F,0x08,0x04,0x04,0x78},
    /* 0x69 'i'  */ {0x00,0x44,0x7D,0x40,0x00},
    /* 0x6A 'j'  */ {0x20,0x40,0x44,0x3D,0x00},
    /* 0x6B 'k'  */ {0x7F,0x10,0x28,0x44,0x00},
    /* 0x6C 'l'  */ {0x00,0x41,0x7F,0x40,0x00},
    /* 0x6D 'm'  */ {0x7C,0x04,0x18,0x04,0x78},
    /* 0x6E 'n'  */ {0x7C,0x08,0x04,0x04,0x78},
    /* 0x6F 'o'  */ {0x38,0x44,0x44,0x44,0x38},
    /* 0x70 'p'  */ {0x7C,0x14,0x14,0x14,0x08},
    /* 0x71 'q'  */ {0x08,0x14,0x14,0x18,0x7C},
    /* 0x72 'r'  */ {0x7C,0x08,0x04,0x04,0x08},
    /* 0x73 's'  */ {0x48,0x54,0x54,0x54,0x20},
    /* 0x74 't'  */ {0x04,0x3F,0x44,0x40,0x20},
    /* 0x75 'u'  */ {0x3C,0x40,0x40,0x40,0x3C},
    /* 0x76 'v'  */ {0x1C,0x20,0x40,0x20,0x1C},
    /* 0x77 'w'  */ {0x3C,0x40,0x30,0x40,0x3C},
    /* 0x78 'x'  */ {0x44,0x28,0x10,0x28,0x44},
    /* 0x79 'y'  */ {0x0C,0x50,0x50,0x50,0x3C},
    /* 0x7A 'z'  */ {0x44,0x64,0x54,0x4C,0x44},
    /* 0x7B '{'  */ {0x00,0x08,0x36,0x41,0x00},
    /* 0x7C '|'  */ {0x00,0x00,0x7F,0x00,0x00},
    /* 0x7D '}'  */ {0x00,0x41,0x36,0x08,0x00},
    /* 0x7E '~'  */ {0x10,0x08,0x08,0x10,0x08},
    /* idx 95  ₹ */ {0x7F,0x50,0x58,0x54,0x62},
};

/* ── DrawChar — handles standard ASCII and RUPEE_CHAR sentinel ───────────
 *
 *  RUPEE_CHAR ('\x01') is caught BEFORE the ASCII range guard and mapped
 *  to glyph index 95.  All other characters use the original path.
 *  Cost of the new branch: 1 compare + 1 conditional jump per DrawChar()
 *  call.  Only fires when a cell containing '\x01' is being redrawn.
 * ──────────────────────────────────────────────────────────────────────── */
void ST7735_DrawChar(uint8_t x, uint8_t y, char c,
                     uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint8_t glyph_idx;

    if (!g_initialized) return;

    /* Resolve glyph index — RUPEE_CHAR must precede the ASCII guard */
    if ((uint8_t)c == (uint8_t)RUPEE_CHAR) {
        glyph_idx = 95u;                     /* ₹ custom glyph              */
    } else {
        if (c < 0x20 || c > 0x7E) c = '?';  /* clamp unknown chars         */
        glyph_idx = (uint8_t)(c - 0x20);     /* standard ASCII 0x20-0x7E   */
    }

    if (x + FONT_W * scale > ST7735_WIDTH)  return;
    if (y + FONT_H * scale > ST7735_HEIGHT) return;

    const uint8_t *gl = g_font5x7[glyph_idx];
    TFT_SetWindow(x, y,
                  (uint8_t)(x + FONT_W * scale - 1),
                  (uint8_t)(y + FONT_H * scale - 1));
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
        x = (uint8_t)(x + FONT_ADV * scale);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION G — DISPLAY LAYOUT & CHANNEL-TO-SCREEN RENDERING
 *
 *  Dirty-flag pattern: each cell caches its last rendered string.
 *  zone_update() returns immediately when string is unchanged.
 *  All channel values come from CHANNEL_Get() — driver is fully passive.
 * ══════════════════════════════════════════════════════════════════════════ */

/* Row geometry (pixel-verified, 1px natural gap at Y=10) */
#define ROW_STA_Y     0
#define ROW_STA_H    10
#define ROW_V_Y      11
#define ROW_V_H      24
#define ROW_A_Y      35
#define ROW_A_H      24
#define ROW_W_Y      59
#define ROW_W_H      24
#define ROW_COST_Y   83
#define ROW_COST_H   23
#define ROW_KWH_Y   106
#define ROW_KWH_H    18
#define ROW_PFHZ_Y  124
#define ROW_PFHZ_H   18
#define ROW_TTY     142
#define ROW_TTH      18

#define S2  2    /* scale-2: char cell 12×14 px */
#define S1  1    /* scale-1: char cell  6× 7 px */

/* Label column for V/A/W rows */
#define LBL_W    (FONT_W * S2)
#define LBL_X    (ST7735_WIDTH - LBL_W)
#define VAL_W    LBL_X

/* Split zones for PF|Hz and Temp|Timer rows */
#define PF_X     0
#define PF_W    38
#define HZ_X    38
#define HZ_W    42
#define TC_X     0
#define TC_W    40
#define TM_X    40
#define TM_W    40

/* Status bar zones */
#define STA_RLY_X   0
#define STA_RLY_W  22
#define STA_ALM_X  29   /* FIX-ST-1: named constant — prevents magic-number OOB */
#define STA_ALM_W  22
#define STA_WIF_X  56
#define STA_WIF_W  24

/* ── Dirty-flag string caches ─────────────────────────────────────────────
 *  p_cost: was [14] for "Rs%5.2f" (7 chars + NUL).
 *          now [10] for "\x01%5.2f" (6 chars + NUL). Saves 4 bytes RAM.
 *  p_alm:  new [4] for alarm indicator (FIX-ST-1).  +4 bytes RAM.
 *  Net RAM delta: 0 bytes.
 * ──────────────────────────────────────────────────────────────────────── */
static char p_v[10], p_a[10], p_w[10];
static char p_cost[10];   /* ₹ format "\x01%5.2f" = 6 chars, was [14] for "Rs" */
static char p_kwh[14];
static char p_pf[10], p_hz[10];
static char p_tc[10], p_tmr[8];
static char p_rly[4], p_wif[6];
static char p_alm[4];     /* FIX-ST-1: alarm indicator dirty-flag cache */

/* Clear zone, draw string, update cache — skip if string unchanged */
static void zone_update(uint8_t x, uint8_t y, uint8_t zw, uint8_t zh,
                        const char *str, uint16_t fg, uint8_t sc,
                        char *cache, uint8_t cache_sz)
{
    if (strncmp(str, cache, cache_sz) == 0) return;
    strncpy(cache, str, cache_sz - 1);
    cache[cache_sz - 1] = '\0';
    ST7735_FillRect(x, y, zw, zh, ST7735_BLACK);
    uint8_t ty = y + (uint8_t)((zh - FONT_H * sc) / 2);
    ST7735_DrawString(x, ty, str, fg, ST7735_BLACK, (uint8_t)sc);
}

/* Static V/A/W unit labels — drawn once at init and after st7735_clear */
static void draw_static_labels(void)
{
    uint8_t vy = (uint8_t)(ROW_V_Y + (ROW_V_H - FONT_H * S2) / 2);
    uint8_t ay = (uint8_t)(ROW_A_Y + (ROW_A_H - FONT_H * S2) / 2);
    uint8_t wy = (uint8_t)(ROW_W_Y + (ROW_W_H - FONT_H * S2) / 2);
    ST7735_DrawChar((uint8_t)LBL_X, vy, 'V', ST7735_RED,    ST7735_BLACK, S2);
    ST7735_DrawChar((uint8_t)LBL_X, ay, 'A', ST7735_CYAN,   ST7735_BLACK, S2);
    ST7735_DrawChar((uint8_t)LBL_X, wy, 'W', ST7735_YELLOW, ST7735_BLACK, S2);
}

extern int CHANNEL_Get(int ch);

static void display_tick(void)
{
    char buf[16];

    /* Decode all channels to physical units */
    float v    = (float)CHANNEL_Get(DISP_CH_VOLTAGE) / 100.0f;
    float a    = (float)CHANNEL_Get(DISP_CH_CURRENT) / 1000.0f;
    float w    = (float)CHANNEL_Get(DISP_CH_POWER)   / 10.0f;
    float hz   = (float)CHANNEL_Get(DISP_CH_FREQ)    / 100.0f;
    float pf   = (float)CHANNEL_Get(DISP_CH_PF)      / 1000.0f;
    float kwh  = (float)CHANNEL_Get(DISP_CH_ENERGY)  / 1000.0f;  /* Wh×10→kWh */
    float tc   = (float)CHANNEL_Get(DISP_CH_TEMP)    / 100.0f;
    float cost = (float)CHANNEL_Get(DISP_CH_EVCOST)  / 100.0f;
    int   rly  = CHANNEL_Get(DISP_CH_RELAY);
    int   wif  = CHANNEL_Get(DISP_CH_WIFI);
    int   alm_raw = CHANNEL_Get(DISP_CH_ALARM);

    /* FIX-ST-1: clamp alarm to [0,4] before use as array index.
     * Prevents OOB read if channel holds a corrupt value.        */
    int alm = (alm_raw >= 0 && alm_raw <= 4) ? alm_raw : 0;

    static uint32_t s_sec = 0;
    s_sec++;

    /* ── STATUS BAR: relay state ──────────────────────────────────────── */
    {
        const char *rs = (rly >= 100) ? "ON " : "OFF";
        uint16_t rc = (rly >= 100) ? ST7735_GREEN : ST7735_GREY;
        if (strncmp(rs, p_rly, sizeof(p_rly)) != 0) {
            strncpy(p_rly, rs, sizeof(p_rly) - 1);
            ST7735_FillRect(STA_RLY_X, ROW_STA_Y, STA_RLY_W, ROW_STA_H, ST7735_BLACK);
            ST7735_DrawString(STA_RLY_X, ROW_STA_Y + 1, rs, rc, ST7735_BLACK, S1);
        }
    }

    /* ── STATUS BAR: alarm indicator (FIX-ST-1) ──────────────────────────
     * Trailing space pads each string to 3 chars so zone_update()'s
     * FillRect fully erases any prior wider string on transition.        */
    {
        const char *an[] = {"   ", "OV ", "UV ", "OC ", "OP "};
        zone_update(STA_ALM_X, ROW_STA_Y, STA_ALM_W, ROW_STA_H,
                    an[alm], ST7735_RED, S1, p_alm, sizeof(p_alm));
    }

    /* ── STATUS BAR: wifi state ────────────────────────────────────────── */
    {
        const char *ws = wif ? "WiFi" : "NoWF";
        uint16_t wc = wif ? ST7735_BLUE : ST7735_GREY;
        if (strncmp(ws, p_wif, sizeof(p_wif)) != 0) {
            strncpy(p_wif, ws, sizeof(p_wif) - 1);
            ST7735_FillRect(STA_WIF_X, ROW_STA_Y, STA_WIF_W, ROW_STA_H, ST7735_BLACK);
            ST7735_DrawString(STA_WIF_X, ROW_STA_Y + 1, ws, wc, ST7735_BLACK, S1);
        }
    }

    /* ── VOLTAGE ────────────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.1f", v);
    zone_update(0, ROW_V_Y, VAL_W, ROW_V_H, buf,
                ST7735_RED, S2, p_v, sizeof(p_v));

    /* ── CURRENT ────────────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.3f", a);
    zone_update(0, ROW_A_Y, VAL_W, ROW_A_H, buf,
                ST7735_CYAN, S2, p_a, sizeof(p_a));

    /* ── POWER ──────────────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%5.1f", w);
    zone_update(0, ROW_W_Y, VAL_W, ROW_W_H, buf,
                ST7735_YELLOW, S2, p_w, sizeof(p_w));

    /* ── EV COST ─────────────────────────────────────────────────────────
     * '\x01' = RUPEE_CHAR is resolved to the ₹ glyph (index 95) inside
     * DrawChar().  Produces "₹  3.76" on screen at scale 2.
     * One fewer DrawChar() call vs the old "Rs%5.2f" format.            */
    snprintf(buf, sizeof(buf), "\x01%5.2f", cost);
    zone_update(0, ROW_COST_Y, ST7735_WIDTH, ROW_COST_H, buf,
                ST7735_GREEN, S2, p_cost, sizeof(p_cost));

    /* ── ENERGY (kWh) ───────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%06.3fkWh", kwh);
    zone_update(0, ROW_KWH_Y, ST7735_WIDTH, ROW_KWH_H, buf,
                ST7735_CYAN, S1, p_kwh, sizeof(p_kwh));

    /* ── POWER FACTOR ───────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%.2fPF", pf);
    zone_update(PF_X, ROW_PFHZ_Y, PF_W, ROW_PFHZ_H, buf,
                ST7735_RED, S1, p_pf, sizeof(p_pf));

    /* ── FREQUENCY ──────────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%.1fHz", hz);
    zone_update(HZ_X, ROW_PFHZ_Y, HZ_W, ROW_PFHZ_H, buf,
                ST7735_BLUE, S1, p_hz, sizeof(p_hz));

    /* ── TEMPERATURE ────────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%05.2fC", tc);
    zone_update(TC_X, ROW_TTY, TC_W, ROW_TTH, buf,
                ST7735_ORANGE, S1, p_tc, sizeof(p_tc));

    /* ── TIMER (mm:ss rolling, resets at 99:59) ─────────────────────────── */
    uint32_t mm = (s_sec / 60) % 100;
    uint32_t ss =  s_sec % 60;
    snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)mm, (unsigned)ss);
    zone_update(TM_X, ROW_TTY, TM_W, ROW_TTH, buf,
                ST7735_WHITE, S1, p_tmr, sizeof(p_tmr));
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION H — CONSOLE COMMANDS
 * ══════════════════════════════════════════════════════════════════════════ */

/* Helper: reset all dirty caches (forces full redraw on next tick) */
static void caches_clear(void)
{
    memset(p_v,   '\0', sizeof(p_v));
    memset(p_a,   '\0', sizeof(p_a));
    memset(p_w,   '\0', sizeof(p_w));
    memset(p_cost,'\0', sizeof(p_cost));
    memset(p_kwh, '\0', sizeof(p_kwh));
    memset(p_pf,  '\0', sizeof(p_pf));
    memset(p_hz,  '\0', sizeof(p_hz));
    memset(p_tc,  '\0', sizeof(p_tc));
    memset(p_tmr, '\0', sizeof(p_tmr));
    memset(p_rly, '\0', sizeof(p_rly));
    memset(p_wif, '\0', sizeof(p_wif));
    memset(p_alm, '\0', sizeof(p_alm));
}

static commandResult_t CMD_Clear(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    uint16_t colour = ST7735_BLACK;
    if (args && *args) colour = (uint16_t)strtol(args, NULL, 0);
    caches_clear();
    ST7735_FillScreen(colour);
    if (colour == ST7735_BLACK) draw_static_labels();
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
    g_cur_col = (uint8_t)col;
    g_cur_row = (uint8_t)row;
    return CMD_RES_OK;
}

static commandResult_t CMD_Print(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    uint8_t x = (uint8_t)(g_cur_col * FONT_ADV  * g_txt_scale);
    uint8_t y = (uint8_t)(g_cur_row * FONT_VADV * g_txt_scale);
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
    ST7735_FillRect((uint8_t)x, (uint8_t)y,
                    (uint8_t)w, (uint8_t)h, (uint16_t)colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_Scale(const void *ctx, const char *cmd,
                                 const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int s = atoi(args);
    if (s < 1) s = 1;
    if (s > 3) s = 3;
    g_txt_scale = (uint8_t)s;
    return CMD_RES_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION I — OPENBK LIFECYCLE
 * ══════════════════════════════════════════════════════════════════════════ */
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

    HAL_PIN_Setup_Output(g_pin_sck); HAL_PIN_Setup_Output(g_pin_sda);
    HAL_PIN_Setup_Output(g_pin_res); HAL_PIN_Setup_Output(g_pin_dc);
    HAL_PIN_Setup_Output(g_pin_cs);  HAL_PIN_Setup_Output(g_pin_blk);

    SPI_CS_H(); SPI_SCK_L(); SPI_SDA_L(); SPI_DC_H(); SPI_RES_H();
    SPI_BLK_L(); ST7735_Delay(50);

    TFT_HardReset();
    TFT_InitController();
    g_initialized = 1;
    ST7735_FillScreen(ST7735_BLACK);
    draw_static_labels();
    caches_clear();   /* force full redraw on first RunEverySecond() tick */

    CMD_RegisterCommand("st7735_clear",      CMD_Clear,      NULL);
    CMD_RegisterCommand("st7735_brightness", CMD_Brightness, NULL);
    CMD_RegisterCommand("st7735_goto",       CMD_Goto,       NULL);
    CMD_RegisterCommand("st7735_print",      CMD_Print,      NULL);
    CMD_RegisterCommand("st7735_color",      CMD_Color,      NULL);
    CMD_RegisterCommand("st7735_draw",       CMD_Draw,       NULL);
    CMD_RegisterCommand("st7735_scale",      CMD_Scale,      NULL);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: ready 80x160 rupee=idx95 sentinel=0x01 — reads Ch1-11");
}

void ST7735_RunEverySecond(void)
{
    if (!g_initialized) return;
    display_tick();
}

#endif /* ENABLE_DRIVER_ST7735 */
