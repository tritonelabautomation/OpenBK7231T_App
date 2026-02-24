/*
 * ST7735S — 0.96" 80×160 Color TFT Driver for OpenBK7231T / BK7231N
 * Target: KWS-303WF  FPC-JL096B005-01V0
 *
 * Software (bit-bang) 4-wire SPI — no hardware SPI required.
 * Uses HAL_PIN_Setup_Output / HAL_PIN_SetOutputValue — same as BL0937 driver.
 * No channel.h | No powerMeasurementCalibration | No CFG_SetExtended
 *
 * startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> [BLK]
 * Example:   startDriver ST7735 9 10 11 14 16 17
 *
 * Screen layout mimics the original KWS-303WF colour display:
 *   Red    = Voltage
 *   Yellow = Current
 *   Blue   = Power (W)
 *   Cyan   = Energy (kWh)
 *   White  = Power Factor + pulse count
 *   Green  = Frequency
 *   Red    = Temperature
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

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION A — PIN STATE
// ═══════════════════════════════════════════════════════════════════════════════

static int g_pin_sck = ST7735_DEFAULT_SCK;
static int g_pin_sda = ST7735_DEFAULT_SDA;
static int g_pin_res = ST7735_DEFAULT_RES;
static int g_pin_dc  = ST7735_DEFAULT_DC;
static int g_pin_cs  = ST7735_DEFAULT_CS;
static int g_pin_blk = ST7735_DEFAULT_BLK;
static uint8_t g_initialized = 0;

// Current text cursor and colours
static uint8_t  g_cur_col   = 0;
static uint8_t  g_cur_row   = 0;
static uint16_t g_fg_colour = ST7735_WHITE;
static uint16_t g_bg_colour = ST7735_BLACK;
static uint8_t  g_txt_scale = ST7735_TEXT_SCALE_LARGE;

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION B — SOFTWARE SPI BIT-BANG
// ═══════════════════════════════════════════════════════════════════════════════

// Inline GPIO helpers — identical pattern to BL0937 driver
#define SPI_SCK_H()   HAL_PIN_SetOutputValue(g_pin_sck, 1)
#define SPI_SCK_L()   HAL_PIN_SetOutputValue(g_pin_sck, 0)
#define SPI_SDA_H()   HAL_PIN_SetOutputValue(g_pin_sda, 1)
#define SPI_SDA_L()   HAL_PIN_SetOutputValue(g_pin_sda, 0)
#define SPI_CS_H()    HAL_PIN_SetOutputValue(g_pin_cs,  1)
#define SPI_CS_L()    HAL_PIN_SetOutputValue(g_pin_cs,  0)
#define SPI_DC_H()    HAL_PIN_SetOutputValue(g_pin_dc,  1)   // data
#define SPI_DC_L()    HAL_PIN_SetOutputValue(g_pin_dc,  0)   // command
#define SPI_RES_H()   HAL_PIN_SetOutputValue(g_pin_res, 1)
#define SPI_RES_L()   HAL_PIN_SetOutputValue(g_pin_res, 0)
// Direct SDK call to switch a GPIO pin to its PWM function
extern void gpio_config_pwm_enable(unsigned char pin);
// Direct SDK calls for the BK7231N "new" PWM hardware (5 arguments required)
extern void bk_pwm_initialize(unsigned char channel, unsigned int period, unsigned int duty, unsigned int duty2, unsigned int duty3);
extern void bk_pwm_start(unsigned char channel);

#define SPI_BLK_H()   HAL_PIN_SetOutputValue(g_pin_blk, 1)
#define SPI_BLK_L()   HAL_PIN_SetOutputValue(g_pin_blk, 0)

// Simple busy-delay — BK7231N ~80MHz, each iteration ~12.5ns
// 80 iterations ≈ 1µs.  We need ≥100ns between clock edges for ST7735S.
static inline void spi_delay(void)
{
    volatile int i = 4;   // ~50ns — ST7735S max SPI clock 15MHz → min 33ns period
    while (i--) { }
}

// Transmit one byte MSB-first on bit-bang SPI (Mode 0: CPOL=0 CPHA=0)
static void SPI_WriteByte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (b & 0x80) {
            SPI_SDA_H();
        } else {
            SPI_SDA_L();
        }
        spi_delay();
        SPI_SCK_H();
        spi_delay();
        SPI_SCK_L();
        b <<= 1;
    }
}

// Write a command byte (DC low, CS low for transaction)
static void ST7735_WriteCmd(uint8_t cmd)
{
    SPI_CS_L();
    SPI_DC_L();
    SPI_WriteByte(cmd);
    SPI_CS_H();
}

// Write a data byte (DC high)
static void ST7735_WriteData8(uint8_t d)
{
    SPI_CS_L();
    SPI_DC_H();
    SPI_WriteByte(d);
    SPI_CS_H();
}

// Write a 16-bit data word (RGB565)
static void ST7735_WriteData16(uint16_t d)
{
    SPI_CS_L();
    SPI_DC_H();
    SPI_WriteByte((uint8_t)(d >> 8));
    SPI_WriteByte((uint8_t)(d & 0xFF));
    SPI_CS_H();
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION C — RTOS-FRIENDLY DELAY
// Uses a simple spin-loop. For longer delays (>50ms) split into yield loops.
// BK7231N: calling rtos_delay_milliseconds is preferred if available but
// we can't guarantee it here, so we use HAL_Delay equivalent.
// ═══════════════════════════════════════════════════════════════════════════════

static void ST7735_Delay(uint32_t ms)
{
    // SDK signature from beken378/os/include/rtos_pub.h line 292:
    // OSStatus rtos_delay_milliseconds(uint32_t num_ms);
    // OSStatus is typically int/long — we cast to void to discard return value.
    extern int rtos_delay_milliseconds(uint32_t num_ms);
    (void)rtos_delay_milliseconds(ms);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION D — DISPLAY INITIALISATION
// Full init sequence for ST7735S 80×160 green-tab / mini160x80 variant.
// Based on confirmed-working Adafruit + TFT_eSPI sequences for this panel.
// ═══════════════════════════════════════════════════════════════════════════════

static void ST7735_HardReset(void)
{
    SPI_RES_H();
    ST7735_Delay(10);
    SPI_RES_L();
    ST7735_Delay(10);
    SPI_RES_H();
    ST7735_Delay(120);   // datasheet: 120ms after reset before commands
}

static void ST7735_InitController(void)
{
    // ── Software reset ──────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_SWRESET);
    ST7735_Delay(150);

    // ── Sleep out ───────────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_SLPOUT);
    ST7735_Delay(500);

    // ── Frame rate control (normal mode) ───────────────────────────────────
    ST7735_WriteCmd(ST77_FRMCTR1);
    ST7735_WriteData8(0x01);
    ST7735_WriteData8(0x2C);
    ST7735_WriteData8(0x2D);

    // ── Frame rate control (idle mode) ─────────────────────────────────────
    ST7735_WriteCmd(ST77_FRMCTR2);
    ST7735_WriteData8(0x01);
    ST7735_WriteData8(0x2C);
    ST7735_WriteData8(0x2D);

    // ── Frame rate control (partial mode) ──────────────────────────────────
    ST7735_WriteCmd(ST77_FRMCTR3);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);
    ST7735_WriteData8(0x01); ST7735_WriteData8(0x2C); ST7735_WriteData8(0x2D);

    // ── Display inversion control ───────────────────────────────────────────
    ST7735_WriteCmd(ST77_INVCTR);
    ST7735_WriteData8(0x07);

    // ── Power control ───────────────────────────────────────────────────────
    // ── Power control 1 ───────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_PWCTR1);
    ST7735_WriteData8(0xA2); // Increased drive voltage
    ST7735_WriteData8(0x02);
    ST7735_WriteData8(0x84);

    // ── Power control 2 ───────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_PWCTR2);
    ST7735_WriteData8(0xC5); // Max internal boost for brightness

    // ── VCOM control ──────────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_VMCTR1);
    ST7735_WriteData8(0x3F); // Sharper contrast and true black depth

    ST7735_WriteCmd(ST77_PWCTR4);
    ST7735_WriteData8(0x8A);
    ST7735_WriteData8(0x2A);

    ST7735_WriteCmd(ST77_PWCTR5);
    ST7735_WriteData8(0x8A);
    ST7735_WriteData8(0xEE);

    ST7735_WriteCmd(ST77_VMCTR1);
    ST7735_WriteData8(0x0E);

    // ── Display inversion ON (needed for correct colours on this panel) ─────
    ST7735_WriteCmd(ST77_INVOFF);

    // ── Memory data access control ──────────────────────────────────────────
    // Portrait 80×160: MX+MV = landscape reading from chip RAM
    // For our display: MX | MV | BGR gives correct portrait orientation
    // ── Memory data access control ──────────────────────────────────────────
    // Standard Portrait mode (No MV flag)
    // ── Memory data access control ──────────────────────────────────────────
    // Standard Portrait mode (Right-side up)
    ST7735_WriteCmd(ST77_MADCTL);
    ST7735_WriteData8(MADCTL_BGR);
    // ── Colour mode: 16-bit RGB565 ──────────────────────────────────────────
    ST7735_WriteCmd(ST77_COLMOD);
    ST7735_WriteData8(0x05);   // 0x05 = 16bpp
    ST7735_Delay(10);

    // ── Column address set: 0 to 79 (with offset 26 for green tab) ─────────
    ST7735_WriteCmd(ST77_CASET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(ST7735_COL_OFFSET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(ST7735_COL_OFFSET + ST7735_WIDTH - 1);

    // ── Row address set: 0 to 159 (with offset 1) ──────────────────────────
    ST7735_WriteCmd(ST77_RASET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(ST7735_ROW_OFFSET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(ST7735_ROW_OFFSET + ST7735_HEIGHT - 1);

    // ── Positive gamma correction ───────────────────────────────────────────
    ST7735_WriteCmd(ST77_GMCTRP1);
    ST7735_WriteData8(0x02); ST7735_WriteData8(0x1C); ST7735_WriteData8(0x07);
    ST7735_WriteData8(0x12); ST7735_WriteData8(0x37); ST7735_WriteData8(0x32);
    ST7735_WriteData8(0x29); ST7735_WriteData8(0x2D); ST7735_WriteData8(0x29);
    ST7735_WriteData8(0x25); ST7735_WriteData8(0x2B); ST7735_WriteData8(0x39);
    ST7735_WriteData8(0x00); ST7735_WriteData8(0x01); ST7735_WriteData8(0x03);
    ST7735_WriteData8(0x10);

    // ── Negative gamma correction ───────────────────────────────────────────
    ST7735_WriteCmd(ST77_GMCTRN1);
    ST7735_WriteData8(0x03); ST7735_WriteData8(0x1D); ST7735_WriteData8(0x07);
    ST7735_WriteData8(0x06); ST7735_WriteData8(0x2E); ST7735_WriteData8(0x2C);
    ST7735_WriteData8(0x29); ST7735_WriteData8(0x2D); ST7735_WriteData8(0x2E);
    ST7735_WriteData8(0x2E); ST7735_WriteData8(0x37); ST7735_WriteData8(0x3F);
    ST7735_WriteData8(0x00); ST7735_WriteData8(0x00); ST7735_WriteData8(0x02);
    ST7735_WriteData8(0x10);

    // ── Normal display on ───────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_NORON);
    ST7735_Delay(10);

    // ── Display on ──────────────────────────────────────────────────────────
    ST7735_WriteCmd(ST77_DISPON);
    ST7735_Delay(100);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION E — DRAWING PRIMITIVES
// ═══════════════════════════════════════════════════════════════════════════════

// Set the chip's write window to a rectangle before pixel streaming
static void ST7735_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    ST7735_WriteCmd(ST77_CASET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(x0 + ST7735_COL_OFFSET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(x1 + ST7735_COL_OFFSET);

    ST7735_WriteCmd(ST77_RASET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(y0 + ST7735_ROW_OFFSET);
    ST7735_WriteData8(0x00);
    ST7735_WriteData8(y1 + ST7735_ROW_OFFSET);

    ST7735_WriteCmd(ST77_RAMWR);
}

void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour)
{
    if (!g_initialized) return;
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH  - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = (uint8_t)(colour >> 8);
    uint8_t lo = (uint8_t)(colour & 0xFF);
    uint32_t count = (uint32_t)w * h;

    SPI_CS_L();
    SPI_DC_H();
    while (count--) {
        SPI_WriteByte(hi);
        SPI_WriteByte(lo);
    }
    SPI_CS_H();
}

void ST7735_FillScreen(uint16_t colour)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, colour);
}

// ─── 5×7 ASCII font ───────────────────────────────────────────────────────────
// Standard font, chars 0x20 (space) to 0x7E (~)
static const uint8_t g_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20 space
    {0x00,0x00,0x5F,0x00,0x00}, // 0x21 !
    {0x00,0x07,0x00,0x07,0x00}, // 0x22 "
    {0x14,0x7F,0x14,0x7F,0x14}, // 0x23 #
    {0x24,0x2A,0x7F,0x2A,0x12}, // 0x24 $
    {0x23,0x13,0x08,0x64,0x62}, // 0x25 %
    {0x36,0x49,0x55,0x22,0x50}, // 0x26 &
    {0x00,0x05,0x03,0x00,0x00}, // 0x27 '
    {0x00,0x1C,0x22,0x41,0x00}, // 0x28 (
    {0x00,0x41,0x22,0x1C,0x00}, // 0x29 )
    {0x14,0x08,0x3E,0x08,0x14}, // 0x2A *
    {0x08,0x08,0x3E,0x08,0x08}, // 0x2B +
    {0x00,0x50,0x30,0x00,0x00}, // 0x2C ,
    {0x08,0x08,0x08,0x08,0x08}, // 0x2D -
    {0x00,0x60,0x60,0x00,0x00}, // 0x2E .
    {0x20,0x10,0x08,0x04,0x02}, // 0x2F /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0x30 0
    {0x00,0x42,0x7F,0x40,0x00}, // 0x31 1
    {0x42,0x61,0x51,0x49,0x46}, // 0x32 2
    {0x21,0x41,0x45,0x4B,0x31}, // 0x33 3
    {0x18,0x14,0x12,0x7F,0x10}, // 0x34 4
    {0x27,0x45,0x45,0x45,0x39}, // 0x35 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 0x36 6
    {0x01,0x71,0x09,0x05,0x03}, // 0x37 7
    {0x36,0x49,0x49,0x49,0x36}, // 0x38 8
    {0x06,0x49,0x49,0x29,0x1E}, // 0x39 9
    {0x00,0x36,0x36,0x00,0x00}, // 0x3A :
    {0x00,0x56,0x36,0x00,0x00}, // 0x3B ;
    {0x08,0x14,0x22,0x41,0x00}, // 0x3C <
    {0x14,0x14,0x14,0x14,0x14}, // 0x3D =
    {0x00,0x41,0x22,0x14,0x08}, // 0x3E >
    {0x02,0x01,0x51,0x09,0x06}, // 0x3F ?
    {0x32,0x49,0x79,0x41,0x3E}, // 0x40 @
    {0x7E,0x11,0x11,0x11,0x7E}, // 0x41 A
    {0x7F,0x49,0x49,0x49,0x36}, // 0x42 B
    {0x3E,0x41,0x41,0x41,0x22}, // 0x43 C
    {0x7F,0x41,0x41,0x22,0x1C}, // 0x44 D
    {0x7F,0x49,0x49,0x49,0x41}, // 0x45 E
    {0x7F,0x09,0x09,0x09,0x01}, // 0x46 F
    {0x3E,0x41,0x49,0x49,0x7A}, // 0x47 G
    {0x7F,0x08,0x08,0x08,0x7F}, // 0x48 H
    {0x00,0x41,0x7F,0x41,0x00}, // 0x49 I
    {0x20,0x40,0x41,0x3F,0x01}, // 0x4A J
    {0x7F,0x08,0x14,0x22,0x41}, // 0x4B K
    {0x7F,0x40,0x40,0x40,0x40}, // 0x4C L
    {0x7F,0x02,0x0C,0x02,0x7F}, // 0x4D M
    {0x7F,0x04,0x08,0x10,0x7F}, // 0x4E N
    {0x3E,0x41,0x41,0x41,0x3E}, // 0x4F O
    {0x7F,0x09,0x09,0x09,0x06}, // 0x50 P
    {0x3E,0x41,0x51,0x21,0x5E}, // 0x51 Q
    {0x7F,0x09,0x19,0x29,0x46}, // 0x52 R
    {0x46,0x49,0x49,0x49,0x31}, // 0x53 S
    {0x01,0x01,0x7F,0x01,0x01}, // 0x54 T
    {0x3F,0x40,0x40,0x40,0x3F}, // 0x55 U
    {0x1F,0x20,0x40,0x20,0x1F}, // 0x56 V
    {0x3F,0x40,0x38,0x40,0x3F}, // 0x57 W
    {0x63,0x14,0x08,0x14,0x63}, // 0x58 X
    {0x07,0x08,0x70,0x08,0x07}, // 0x59 Y
    {0x61,0x51,0x49,0x45,0x43}, // 0x5A Z
    {0x00,0x7F,0x41,0x41,0x00}, // 0x5B [
    {0x02,0x04,0x08,0x10,0x20}, // 0x5C backslash
    {0x00,0x41,0x41,0x7F,0x00}, // 0x5D ]
    {0x04,0x02,0x01,0x02,0x04}, // 0x5E ^
    {0x40,0x40,0x40,0x40,0x40}, // 0x5F _
    {0x00,0x01,0x02,0x04,0x00}, // 0x60 `
    {0x20,0x54,0x54,0x54,0x78}, // 0x61 a
    {0x7F,0x48,0x44,0x44,0x38}, // 0x62 b
    {0x38,0x44,0x44,0x44,0x20}, // 0x63 c
    {0x38,0x44,0x44,0x48,0x7F}, // 0x64 d
    {0x38,0x54,0x54,0x54,0x18}, // 0x65 e
    {0x08,0x7E,0x09,0x01,0x02}, // 0x66 f
    {0x0C,0x52,0x52,0x52,0x3E}, // 0x67 g
    {0x7F,0x08,0x04,0x04,0x78}, // 0x68 h
    {0x00,0x44,0x7D,0x40,0x00}, // 0x69 i
    {0x20,0x40,0x44,0x3D,0x00}, // 0x6A j
    {0x7F,0x10,0x28,0x44,0x00}, // 0x6B k
    {0x00,0x41,0x7F,0x40,0x00}, // 0x6C l
    {0x7C,0x04,0x18,0x04,0x78}, // 0x6D m
    {0x7C,0x08,0x04,0x04,0x78}, // 0x6E n
    {0x38,0x44,0x44,0x44,0x38}, // 0x6F o
    {0x7C,0x14,0x14,0x14,0x08}, // 0x70 p
    {0x08,0x14,0x14,0x18,0x7C}, // 0x71 q
    {0x7C,0x08,0x04,0x04,0x08}, // 0x72 r
    {0x48,0x54,0x54,0x54,0x20}, // 0x73 s
    {0x04,0x3F,0x44,0x40,0x20}, // 0x74 t
    {0x3C,0x40,0x40,0x40,0x3C}, // 0x75 u
    {0x1C,0x20,0x40,0x20,0x1C}, // 0x76 v
    {0x3C,0x40,0x30,0x40,0x3C}, // 0x77 w
    {0x44,0x28,0x10,0x28,0x44}, // 0x78 x
    {0x0C,0x50,0x50,0x50,0x3C}, // 0x79 y
    {0x44,0x64,0x54,0x4C,0x44}, // 0x7A z
    {0x00,0x08,0x36,0x41,0x00}, // 0x7B {
    {0x00,0x00,0x7F,0x00,0x00}, // 0x7C |
    {0x00,0x41,0x36,0x08,0x00}, // 0x7D }
    {0x10,0x08,0x08,0x10,0x08}, // 0x7E ~
};

void ST7735_DrawChar(uint8_t x, uint8_t y, char c,
                     uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!g_initialized) return;
    if (c < 0x20 || c > 0x7E) c = '?';

    const uint8_t *glyph = g_font5x7[(uint8_t)(c - 0x20)];
    uint8_t col, row, bit;
    uint16_t colour;
    uint8_t s;

    uint8_t char_w = FONT_W * scale;
    uint8_t char_h = FONT_H * scale;

    if (x + char_w > ST7735_WIDTH)  return;
    if (y + char_h > ST7735_HEIGHT) return;

    ST7735_SetWindow(x, y, x + char_w - 1, y + char_h - 1);

    SPI_CS_L();
    SPI_DC_H();

    for (row = 0; row < FONT_H; row++) {
        for (s = 0; s < scale; s++) {
            for (col = 0; col < FONT_W; col++) {
                bit   = (glyph[col] >> row) & 0x01;
                colour= bit ? fg : bg;
                uint8_t hi = (uint8_t)(colour >> 8);
                uint8_t lo = (uint8_t)(colour & 0xFF);
                uint8_t t;
                for (t = 0; t < scale; t++) {
                    SPI_WriteByte(hi);
                    SPI_WriteByte(lo);
                }
            }
        }
    }

    SPI_CS_H();
}

void ST7735_DrawString(uint8_t x, uint8_t y, const char *str,
                        uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!g_initialized || !str) return;
    uint8_t adv = FONT_ADV * scale;
    while (*str) {
        if (x + adv > ST7735_WIDTH) break;
        ST7735_DrawChar(x, y, *str++, fg, bg, scale);
        x += adv;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION F — ENERGY SCREEN
// Mimics original KWS-303WF colour layout exactly
// ═══════════════════════════════════════════════════════════════════════════════

void ST7735_DrawEnergyScreen(float v, float a, float w,
                              float kwh, float pf, float hz,
                              float temp_c, uint8_t relay_on)
{
    if (!g_initialized) return;

    char buf[24];

    // ── Background ────────────────────────────────────────────────────────
    ST7735_FillScreen(ST7735_BLACK);

    // ── Row 0: Status bar ─────────────────────────────────────────────────
    // "ON" in green top-right, or "OFF" in grey
    ST7735_DrawString(0, 0, relay_on ? "ON " : "OFF",
                      relay_on ? ST7735_GREEN : ST7735_GREY,
                      ST7735_BLACK, ST7735_TEXT_SCALE_SMALL);

    // ── Row 1: Voltage — RED, large ───────────────────────────────────────
    snprintf(buf, sizeof(buf), "%6.1fV", v);
    ST7735_DrawString(0, 12, buf, ST7735_RED, ST7735_BLACK,
                      ST7735_TEXT_SCALE_LARGE);

    // ── Row 2: Current — YELLOW, large ────────────────────────────────────
    snprintf(buf, sizeof(buf), "%6.3fA", a);
    ST7735_DrawString(0, 32, buf, ST7735_YELLOW, ST7735_BLACK,
                      ST7735_TEXT_SCALE_LARGE);

    // ── Row 3: Power — BLUE, large ────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%6.1fW", w);
    ST7735_DrawString(0, 52, buf, ST7735_BLUE, ST7735_BLACK,
                      ST7735_TEXT_SCALE_LARGE);

    // ── Row 4: Energy — CYAN ──────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%7.3fkWh", kwh);
    ST7735_DrawString(0, 72, buf, ST7735_CYAN, ST7735_BLACK,
                      ST7735_TEXT_SCALE_SMALL);

    // ── Row 5: Power Factor — WHITE ───────────────────────────────────────
    snprintf(buf, sizeof(buf), "PF %4.2f", pf);
    ST7735_DrawString(0, 86, buf, ST7735_WHITE, ST7735_BLACK,
                      ST7735_TEXT_SCALE_SMALL);

    // ── Row 6: Frequency — GREEN ──────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%5.2fHz", hz);
    ST7735_DrawString(0, 100, buf, ST7735_GREEN, ST7735_BLACK,
                      ST7735_TEXT_SCALE_SMALL);

    // ── Row 7: Temperature — RED small ────────────────────────────────────
    snprintf(buf, sizeof(buf), "%5.1fC", temp_c);
    ST7735_DrawString(0, 114, buf, ST7735_RED, ST7735_BLACK,
                      ST7735_TEXT_SCALE_SMALL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION G — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════════

static commandResult_t CMD_ST7735_Clear(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    uint16_t colour = ST7735_BLACK;
    if (args && *args) {
        colour = (uint16_t)strtol(args, NULL, 0);
    }
    ST7735_FillScreen(colour);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: screen cleared colour=0x%04X", colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Brightness(const void *ctx, const char *cmd,
                                                const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    
    int val = atoi(args); 
    if (val < 0) val = 0;
    if (val > 100) val = 100;

    int pwm_ch = PIN_GetPWMIndexForPinIndex(g_pin_blk);
    if (pwm_ch < 0) return CMD_RES_BAD_ARGUMENT;

    // Inverted logic: 100% Brightness = 0 Duty | 0% Brightness = 1000 Duty
    unsigned int period = 1000;
    unsigned int duty = (100 - val) * 10; 

    bk_pwm_initialize((unsigned char)pwm_ch, period, duty, 0, 0);
    bk_pwm_start((unsigned char)pwm_ch);

    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Goto(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int col = 0, row = 0;
    sscanf(args, "%d %d", &col, &row);
    g_cur_col = (uint8_t)col;
    g_cur_row = (uint8_t)row;
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Print(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    uint8_t x = g_cur_col * FONT_ADV * g_txt_scale;
    uint8_t y = g_cur_row * FONT_VADV * g_txt_scale;
    ST7735_DrawString(x, y, args, g_fg_colour, g_bg_colour, g_txt_scale);
    // Advance cursor
    g_cur_col += (uint8_t)(strlen(args));
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Color(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    char *end;
    g_fg_colour = (uint16_t)strtol(args, &end, 0);
    if (end && *end) {
        g_bg_colour = (uint16_t)strtol(end, NULL, 0);
    }
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Draw(const void *ctx, const char *cmd,
                                        const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int x = 0, y = 0, w = 0, h = 0;
    uint32_t colour = 0;
    sscanf(args, "%d %d %d %d %u", &x, &y, &w, &h, &colour);
    ST7735_FillRect((uint8_t)x, (uint8_t)y, (uint8_t)w, (uint8_t)h,
                    (uint16_t)colour);
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Update(const void *ctx, const char *cmd,
                                          const char *args, int flags)
{
    // Trigger a manual screen redraw — reads from HT7017 getter functions
    // if they are linked in the same firmware build
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
        HT7017_GetWh() / 1000.0f,     // Wh → kWh
        HT7017_GetPowerFactor(),
        HT7017_GetFrequency(),
        0.0f,                          // temp — NTC not yet wired to this driver
        1                              // relay assumed ON
    );
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: energy screen updated");
    return CMD_RES_OK;
}

static commandResult_t CMD_ST7735_Scale(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    if (!args || !*args) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    int s = atoi(args);
    if (s < 1) s = 1;
    if (s > 3) s = 3;
    g_txt_scale = (uint8_t)s;
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: text scale=%d", s);
    return CMD_RES_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION H — INIT
// ═══════════════════════════════════════════════════════════════════════════════

void ST7735_Init(void)
{
    // Parse pin args from startDriver command if provided
    // Syntax: startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS>
    if (Tokenizer_GetArgsCount() >= 5) {
        g_pin_sck = Tokenizer_GetArgIntegerDefault(1, ST7735_DEFAULT_SCK);
        g_pin_sda = Tokenizer_GetArgIntegerDefault(2, ST7735_DEFAULT_SDA);
        g_pin_res = Tokenizer_GetArgIntegerDefault(3, ST7735_DEFAULT_RES);
        g_pin_dc  = Tokenizer_GetArgIntegerDefault(4, ST7735_DEFAULT_DC);
        g_pin_cs  = Tokenizer_GetArgIntegerDefault(5, ST7735_DEFAULT_CS);
    }

    // Setup all GPIO as outputs
    HAL_PIN_Setup_Output(g_pin_sck);
    HAL_PIN_Setup_Output(g_pin_sda);
    HAL_PIN_Setup_Output(g_pin_res);
    HAL_PIN_Setup_Output(g_pin_dc);
    HAL_PIN_Setup_Output(g_pin_cs);
    
    // Set the physical pin to PWM mode
    gpio_config_pwm_enable((unsigned char)g_pin_blk); 
    
    // Set initial brightness to 100%
    CMD_ExecuteCommand("st7735_brightness 100", 0);

    // Safe initial states
    SPI_CS_H();
    SPI_SCK_L();
    SPI_SDA_L();
    SPI_DC_H();

    // Hardware reset then controller init
    ST7735_HardReset();
    ST7735_InitController();
    g_initialized = 1;

    // Clear screen to black
    ST7735_FillScreen(ST7735_BLACK);

    // Register console commands
    CMD_RegisterCommand("st7735_clear",      CMD_ST7735_Clear,      NULL);
    CMD_RegisterCommand("st7735_brightness", CMD_ST7735_Brightness, NULL);
    CMD_RegisterCommand("st7735_goto",       CMD_ST7735_Goto,       NULL);
    CMD_RegisterCommand("st7735_print",      CMD_ST7735_Print,      NULL);
    CMD_RegisterCommand("st7735_color",      CMD_ST7735_Color,      NULL);
    CMD_RegisterCommand("st7735_draw",       CMD_ST7735_Draw,       NULL);
    CMD_RegisterCommand("st7735_update",     CMD_ST7735_Update,     NULL);
    CMD_RegisterCommand("st7735_scale",      CMD_ST7735_Scale,      NULL);

    // Splash screen
    ST7735_DrawString(2, 20, "KWS-303WF",  ST7735_CYAN,   ST7735_BLACK, 1);
    ST7735_DrawString(2, 34, "HT7017 v13", ST7735_GREEN,  ST7735_BLACK, 1);
    ST7735_DrawString(2, 48, "Starting..", ST7735_YELLOW, ST7735_BLACK, 1);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: init  SCK=%d SDA=%d RES=%d DC=%d CS=%d",
              g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: 80x160 RGB565  offset col=%d row=%d",
              ST7735_COL_OFFSET, ST7735_ROW_OFFSET);
}

// ─── Called every second by the main loop ────────────────────────────────────
// Auto-refreshes the energy screen from HT7017 getters
static uint8_t g_refresh_counter = 0;

void ST7735_RunEverySecond(void)
{
    if (!g_initialized) return;

    g_refresh_counter++;
    if (g_refresh_counter < 10) return;
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
        0.0f,
        1
    );
}

void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[256];
    snprintf(tmp, sizeof(tmp),
        "<h5>ST7735 TFT — FPC-JL096B005</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><td>Status</td><td>%s</td></tr>"
        "<tr><td>Resolution</td><td>80×160 RGB565</td></tr>"
        "<tr><td>Pins</td><td>SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d</td></tr>"
        "<tr><td>Refresh</td><td>every 5s</td></tr>"
        "</table>",
        g_initialized ? "OK" : "NOT INIT",
        g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);
    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

#endif // ENABLE_DRIVER_ST7735
