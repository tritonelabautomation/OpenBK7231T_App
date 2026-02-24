/*
 * ST7735S — 0.96" 80×160 Color TFT Driver for OpenBK7231T / BK7231N
 * Target: KWS-303WF  FPC-JL096B005-01V0
 *
 * Software (bit-bang) 4-wire SPI — no hardware SPI required.
 * Uses HAL_PIN_Setup_Output / HAL_PIN_SetOutputValue — same as BL0937 driver.
 * No channel.h | No powerMeasurementCalibration | No CFG_SetExtended
 *
 * startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> [BLK]
 * Example:   startDriver ST7735 14 16 9 17 15 24
 *
 * FLICKER-FREE UPDATE STRATEGY:
 *   - FillScreen(BLACK) called ONCE at init only — never during updates
 *   - Static labels ("V","A","W","kWh:","PF:","Tmp:") drawn ONCE at init
 *   - Each refresh ONLY clears + redraws the numeric value zone per row
 *   - String comparison skips redraw if value has not changed
 *   - Result: zero full-screen flash, smooth live updates
 *
 * DISPLAY LAYOUT  80×160px — full utilisation:
 *
 *   Y=  0  h=10   [ON/OFF]  scale1 left  |  [xx.xxHz]  scale1 right
 *   Y= 10  h= 1   ── divider (darkgrey) ──────────────────────────
 *   Y= 11  h=28   Voltage   xxxxx  [V]   scale2  RED
 *   Y= 39  h= 1   ── divider ──────────────────────────────────────
 *   Y= 40  h=28   Current   xxxxx  [A]   scale2  YELLOW
 *   Y= 68  h= 1   ── divider ──────────────────────────────────────
 *   Y= 69  h=28   Power     xxxxx  [W]   scale2  BLUE
 *   Y= 97  h= 1   ── divider ──────────────────────────────────────
 *   Y= 98  h=16   kWh: xxxxxxx            scale1  CYAN
 *   Y=114  h=16   PF:  xxxx               scale1  RED
 *   Y=130  h=16   Tmp: xxxxx C            scale1  ORANGE
 *   Y=146  h=14   (spare black bar)
 *   Y=160  end
 *
 * Pixel maths (scale2 = FONT_ADV*2 = 12px/char, FONT_H*2 = 14px tall):
 *   "999.9" = 5 chars × 12px = 60px value + 10px unit label = 70px < 80px ✓
 *   Vertical: 14px text centred in 28px row → 7px top padding ✓
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
// SECTION A — PIN & STATE
// ═══════════════════════════════════════════════════════════════════════════════

static int g_pin_sck = ST7735_DEFAULT_SCK;
static int g_pin_sda = ST7735_DEFAULT_SDA;
static int g_pin_res = ST7735_DEFAULT_RES;
static int g_pin_dc  = ST7735_DEFAULT_DC;
static int g_pin_cs  = ST7735_DEFAULT_CS;
static int g_pin_blk = ST7735_DEFAULT_BLK;
static uint8_t g_initialized = 0;

// Console cursor / colour state
static uint8_t  g_cur_col   = 0;
static uint8_t  g_cur_row   = 0;
static uint16_t g_fg_colour = ST7735_WHITE;
static uint16_t g_bg_colour = ST7735_BLACK;
static uint8_t  g_txt_scale = ST7735_TEXT_SCALE_LARGE;

// Previous rendered values — skip SPI write if unchanged
static char g_prev_v[12]   = "";
static char g_prev_a[12]   = "";
static char g_prev_w[12]   = "";
static char g_prev_kwh[12] = "";
static char g_prev_pf[12]  = "";
static char g_prev_hz[12]  = "";
static char g_prev_tc[12]  = "";
static char g_prev_on[4]   = "";

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION B — SOFTWARE SPI BIT-BANG
// ═══════════════════════════════════════════════════════════════════════════════

#define SPI_SCK_H()   HAL_PIN_SetOutputValue(g_pin_sck, 1)
#define SPI_SCK_L()   HAL_PIN_SetOutputValue(g_pin_sck, 0)
#define SPI_SDA_H()   HAL_PIN_SetOutputValue(g_pin_sda, 1)
#define SPI_SDA_L()   HAL_PIN_SetOutputValue(g_pin_sda, 0)
#define SPI_CS_H()    HAL_PIN_SetOutputValue(g_pin_cs,  1)
#define SPI_CS_L()    HAL_PIN_SetOutputValue(g_pin_cs,  0)
#define SPI_DC_H()    HAL_PIN_SetOutputValue(g_pin_dc,  1)
#define SPI_DC_L()    HAL_PIN_SetOutputValue(g_pin_dc,  0)
#define SPI_RES_H()   HAL_PIN_SetOutputValue(g_pin_res, 1)
#define SPI_RES_L()   HAL_PIN_SetOutputValue(g_pin_res, 0)
#define SPI_BLK_H()   HAL_PIN_SetOutputValue(g_pin_blk, 1)
#define SPI_BLK_L()   HAL_PIN_SetOutputValue(g_pin_blk, 0)

static inline void spi_delay(void)
{
    volatile int i = 4;
    while (i--) { }
}

static void SPI_WriteByte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (b & 0x80) { SPI_SDA_H(); } else { SPI_SDA_L(); }
        spi_delay();
        SPI_SCK_H();
        spi_delay();
        SPI_SCK_L();
        b <<= 1;
    }
}

static void ST7735_WriteCmd(uint8_t cmd)
{
    SPI_CS_L(); SPI_DC_L();
    SPI_WriteByte(cmd);
    SPI_CS_H();
}

static void ST7735_WriteData8(uint8_t d)
{
    SPI_CS_L(); SPI_DC_H();
    SPI_WriteByte(d);
    SPI_CS_H();
}

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

    ST7735_WriteCmd(ST77_INVOFF);   // black background correct on this panel

    ST7735_WriteCmd(ST77_MADCTL);   ST7735_WriteData8(MADCTL_BGR);
    ST7735_WriteCmd(ST77_COLMOD);   ST7735_WriteData8(0x05); ST7735_Delay(10);

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
    while (n--) { SPI_WriteByte(hi); SPI_WriteByte(lo); }
    SPI_CS_H();
}

void ST7735_FillScreen(uint16_t colour)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, colour);
}

// ─── 5×7 font ─────────────────────────────────────────────────────────────────
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

    const uint8_t *glyph = g_font5x7[(uint8_t)(c - 0x20)];
    ST7735_SetWindow(x, y, x + FONT_W * scale - 1, y + FONT_H * scale - 1);

    SPI_CS_L(); SPI_DC_H();
    uint8_t row, s, col, t;
    for (row = 0; row < FONT_H; row++) {
        for (s = 0; s < scale; s++) {
            for (col = 0; col < FONT_W; col++) {
                uint16_t colour = ((glyph[col] >> row) & 1) ? fg : bg;
                uint8_t hi = colour >> 8, lo = colour & 0xFF;
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
    while (*str) {
        if (x + FONT_W * scale > ST7735_WIDTH) break;
        ST7735_DrawChar(x, y, *str++, fg, bg, scale);
        x += FONT_ADV * scale;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION F — FLICKER-FREE ENERGY SCREEN
// ═══════════════════════════════════════════════════════════════════════════════

// ── Row Y positions & heights ─────────────────────────────────────────────────
#define ROW_STATUS_Y   0
#define ROW_STATUS_H   10
#define ROW_V_Y        11
#define ROW_V_H        28
#define ROW_A_Y        40
#define ROW_A_H        28
#define ROW_W_Y        69
#define ROW_W_H        28
#define ROW_KWH_Y      98
#define ROW_KWH_H      16
#define ROW_PF_Y       114
#define ROW_PF_H       16
#define ROW_TEMP_Y     130
#define ROW_TEMP_H     16
// Y=146 → Y=160: spare (14px black)

// ── Scale for each section ────────────────────────────────────────────────────
// scale2: FONT_ADV*2=12px/char, FONT_H*2=14px tall
// scale1: FONT_ADV*1= 6px/char, FONT_H*1= 7px tall
#define VAL_S   2
#define SML_S   1

// Large value zone: label char ("V"/"A"/"W") is 1 char × FONT_W*VAL_S = 10px
// Value occupies x=0 to x=69 (70px), label at x=70
#define LBL_W        (FONT_W  * VAL_S)          // 10px
#define LBL_X        (ST7735_WIDTH - LBL_W)     // 70
#define VAL_ZONE_W   LBL_X                       // 70px for value text

// Small rows: "kWh:" / "PF: " / "Tmp:" = 4 chars × 6 = 24px label
#define SML_LBL_W    (4 * FONT_ADV * SML_S)     // 24px
#define SML_VAL_X    SML_LBL_W                   // 24
#define SML_VAL_W    (ST7735_WIDTH - SML_LBL_W) // 56px

// Clear a zone then draw new value — the core of flicker-free rendering
static void UpdateZone(uint8_t x, uint8_t y, uint8_t zw, uint8_t zh,
                       const char *str, uint16_t fg, uint8_t scale)
{
    ST7735_FillRect(x, y, zw, zh, ST7735_BLACK);
    uint8_t ty = y + (zh - FONT_H * scale) / 2;  // vertical centre
    ST7735_DrawString(x, ty, str, fg, ST7735_BLACK, scale);
}

// Draw labels, dividers — called ONCE at init, never again
static void ST7735_DrawStaticFrame(void)
{
    // Horizontal dividers
    ST7735_FillRect(0, ROW_STATUS_Y + ROW_STATUS_H, ST7735_WIDTH, 1, ST7735_DARKGREY);
    ST7735_FillRect(0, ROW_V_Y      + ROW_V_H,      ST7735_WIDTH, 1, ST7735_DARKGREY);
    ST7735_FillRect(0, ROW_A_Y      + ROW_A_H,      ST7735_WIDTH, 1, ST7735_DARKGREY);
    ST7735_FillRect(0, ROW_W_Y      + ROW_W_H,      ST7735_WIDTH, 1, ST7735_DARKGREY);

    // Unit labels — right edge of large rows
    uint8_t vy = ROW_V_Y + (ROW_V_H - FONT_H * VAL_S) / 2;
    uint8_t ay = ROW_A_Y + (ROW_A_H - FONT_H * VAL_S) / 2;
    uint8_t wy = ROW_W_Y + (ROW_W_H - FONT_H * VAL_S) / 2;
    ST7735_DrawChar(LBL_X, vy, 'V', ST7735_RED,    ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, ay, 'A', ST7735_YELLOW, ST7735_BLACK, VAL_S);
    ST7735_DrawChar(LBL_X, wy, 'W', ST7735_BLUE,   ST7735_BLACK, VAL_S);

    // Prefix labels for small rows
    uint8_t ky = ROW_KWH_Y  + (ROW_KWH_H  - FONT_H * SML_S) / 2;
    uint8_t py = ROW_PF_Y   + (ROW_PF_H   - FONT_H * SML_S) / 2;
    uint8_t ty = ROW_TEMP_Y + (ROW_TEMP_H - FONT_H * SML_S) / 2;
    ST7735_DrawString(0, ky, "kWh:", ST7735_CYAN,   ST7735_BLACK, SML_S);
    ST7735_DrawString(0, py, "PF: ", ST7735_RED,    ST7735_BLACK, SML_S);
    ST7735_DrawString(0, ty, "Tmp:", ST7735_ORANGE, ST7735_BLACK, SML_S);
}

// Public energy screen update — FLICKER-FREE
void ST7735_DrawEnergyScreen(float v, float a, float w,
                              float kwh, float pf, float hz,
                              float temp_c, uint8_t relay_on)
{
    if (!g_initialized) return;
    char buf[16];

    // ── Status row: ON/OFF left, Hz right ─────────────────────────────────────
    const char *on_str = relay_on ? "ON " : "OFF";
    if (strcmp(on_str, g_prev_on) != 0) {
        strncpy(g_prev_on, on_str, sizeof(g_prev_on) - 1);
        ST7735_FillRect(0, ROW_STATUS_Y, 20, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(0, ROW_STATUS_Y + 1, on_str,
                          relay_on ? ST7735_GREEN : ST7735_GREY,
                          ST7735_BLACK, SML_S);
    }

    snprintf(buf, sizeof(buf), "%5.2fHz", hz);
    if (strcmp(buf, g_prev_hz) != 0) {
        strncpy(g_prev_hz, buf, sizeof(g_prev_hz) - 1);
        ST7735_FillRect(22, ROW_STATUS_Y, ST7735_WIDTH - 22, ROW_STATUS_H, ST7735_BLACK);
        ST7735_DrawString(22, ROW_STATUS_Y + 1, buf,
                          ST7735_GREEN, ST7735_BLACK, SML_S);
    }

    // ── Voltage — RED, scale2 ─────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%5.1f", v);
    if (strcmp(buf, g_prev_v) != 0) {
        strncpy(g_prev_v, buf, sizeof(g_prev_v) - 1);
        UpdateZone(0, ROW_V_Y, VAL_ZONE_W, ROW_V_H, buf, ST7735_RED, VAL_S);
    }

    // ── Current — YELLOW, scale2 ──────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%5.3f", a);
    if (strcmp(buf, g_prev_a) != 0) {
        strncpy(g_prev_a, buf, sizeof(g_prev_a) - 1);
        UpdateZone(0, ROW_A_Y, VAL_ZONE_W, ROW_A_H, buf, ST7735_YELLOW, VAL_S);
    }

    // ── Power — BLUE, scale2 ──────────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%5.1f", w);
    if (strcmp(buf, g_prev_w) != 0) {
        strncpy(g_prev_w, buf, sizeof(g_prev_w) - 1);
        UpdateZone(0, ROW_W_Y, VAL_ZONE_W, ROW_W_H, buf, ST7735_BLUE, VAL_S);
    }

    // ── Energy kWh — CYAN, scale1 ─────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%7.3f", kwh);
    if (strcmp(buf, g_prev_kwh) != 0) {
        strncpy(g_prev_kwh, buf, sizeof(g_prev_kwh) - 1);
        UpdateZone(SML_VAL_X, ROW_KWH_Y, SML_VAL_W, ROW_KWH_H,
                   buf, ST7735_CYAN, SML_S);
    }

    // ── Power Factor — RED, scale1 ────────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%4.2f", pf);
    if (strcmp(buf, g_prev_pf) != 0) {
        strncpy(g_prev_pf, buf, sizeof(g_prev_pf) - 1);
        UpdateZone(SML_VAL_X, ROW_PF_Y, SML_VAL_W, ROW_PF_H,
                   buf, ST7735_RED, SML_S);
    }

    // ── Temperature — ORANGE, scale1 ──────────────────────────────────────────
    snprintf(buf, sizeof(buf), "%4.1fC", temp_c);
    if (strcmp(buf, g_prev_tc) != 0) {
        strncpy(g_prev_tc, buf, sizeof(g_prev_tc) - 1);
        UpdateZone(SML_VAL_X, ROW_TEMP_Y, SML_VAL_W, ROW_TEMP_H,
                   buf, ST7735_ORANGE, SML_S);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION G — CONSOLE COMMANDS
// ═══════════════════════════════════════════════════════════════════════════════

static commandResult_t CMD_ST7735_Clear(const void *ctx, const char *cmd,
                                         const char *args, int flags)
{
    uint16_t colour = ST7735_BLACK;
    if (args && *args) colour = (uint16_t)strtol(args, NULL, 0);
    // Invalidate cache so next DrawEnergyScreen redraws all values
    g_prev_v[0]=g_prev_a[0]=g_prev_w[0]=g_prev_kwh[0]='\0';
    g_prev_pf[0]=g_prev_hz[0]=g_prev_tc[0]=g_prev_on[0]='\0';
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
    int x=0,y=0,w=0,h=0; uint32_t colour=0;
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
        HT7017_GetFrequency(), 0.0f, 1);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY, "ST7735: updated");
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

// ═══════════════════════════════════════════════════════════════════════════════
// SECTION H — INIT
// ═══════════════════════════════════════════════════════════════════════════════

void ST7735_Init(void)
{
    int argc = Tokenizer_GetArgsCount();

    // Parse all 6 pin args if provided
    // startDriver ST7735 <SCK> <SDA> <RES> <DC> <CS> <BLK>
    if (argc >= 2) g_pin_sck = Tokenizer_GetArgIntegerDefault(1, ST7735_DEFAULT_SCK);
    if (argc >= 3) g_pin_sda = Tokenizer_GetArgIntegerDefault(2, ST7735_DEFAULT_SDA);
    if (argc >= 4) g_pin_res = Tokenizer_GetArgIntegerDefault(3, ST7735_DEFAULT_RES);
    if (argc >= 5) g_pin_dc  = Tokenizer_GetArgIntegerDefault(4, ST7735_DEFAULT_DC);
    if (argc >= 6) g_pin_cs  = Tokenizer_GetArgIntegerDefault(5, ST7735_DEFAULT_CS);
    if (argc >= 7) g_pin_blk = Tokenizer_GetArgIntegerDefault(6, ST7735_DEFAULT_BLK);

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: pins SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d",
              g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);

    // Setup all GPIO as outputs FIRST
    HAL_PIN_Setup_Output(g_pin_sck);
    HAL_PIN_Setup_Output(g_pin_sda);
    HAL_PIN_Setup_Output(g_pin_res);
    HAL_PIN_Setup_Output(g_pin_dc);
    HAL_PIN_Setup_Output(g_pin_cs);
    HAL_PIN_Setup_Output(g_pin_blk);

    // Safe idle states
    SPI_CS_H();
    SPI_SCK_L();
    SPI_SDA_L();
    SPI_DC_H();
    SPI_RES_H();

    // Backlight ON immediately — so we can see if panel powers up
    SPI_BLK_L();
    ST7735_Delay(50);

    // Hardware reset + controller init
    ST7735_HardReset();
    ST7735_InitController();
    g_initialized = 1;

    // ONE-TIME full clear + static frame — never repeated during updates
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

    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: init SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d",
              g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGY,
              "ST7735: 80x160 flicker-free col_off=%d row_off=%d",
              ST7735_COL_OFFSET, ST7735_ROW_OFFSET);
}

// ─── Auto-refresh every 2 seconds ────────────────────────────────────────────
static uint8_t g_refresh_counter = 0;

void ST7735_RunEverySecond(void)
{
    if (!g_initialized) return;
    if (++g_refresh_counter < 2) return;
    g_refresh_counter = 0;

    extern float HT7017_GetVoltage(void);
    extern float HT7017_GetCurrent(void);
    extern float HT7017_GetPower(void);
    extern float HT7017_GetWh(void);
    extern float HT7017_GetPowerFactor(void);
    extern float HT7017_GetFrequency(void);

    ST7735_DrawEnergyScreen(
        HT7017_GetVoltage(), HT7017_GetCurrent(), HT7017_GetPower(),
        HT7017_GetWh() / 1000.0f, HT7017_GetPowerFactor(),
        HT7017_GetFrequency(), 0.0f, 1);
}

void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request)
{
    char tmp[256];
    snprintf(tmp, sizeof(tmp),
        "<h5>ST7735 TFT</h5>"
        "<table border='1' cellpadding='4' style='border-collapse:collapse'>"
        "<tr><td>Status</td><td>%s</td></tr>"
        "<tr><td>Size</td><td>80x160 RGB565</td></tr>"
        "<tr><td>Pins</td><td>SCK=%d SDA=%d RES=%d DC=%d CS=%d BLK=%d</td></tr>"
        "<tr><td>Mode</td><td>Flicker-free (zone update)</td></tr>"
        "</table>",
        g_initialized ? "OK" : "NOT INIT",
        g_pin_sck, g_pin_sda, g_pin_res, g_pin_dc, g_pin_cs, g_pin_blk);
    strncat(request->reply, tmp,
            sizeof(request->reply) - strlen(request->reply) - 1);
}

#endif // ENABLE_DRIVER_ST7735
