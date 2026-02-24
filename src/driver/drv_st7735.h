#pragma once
#ifndef __DRV_ST7735_H__
#define __DRV_ST7735_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  ST7735S — 0.96" 80×160 Color TFT Driver for OpenBK7231T / BK7231N        ║
 * ║  Target device : KWS-303WF  (FPC-JL096B005-01V0 display module)           ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  INTERFACE: 4-wire Software SPI (bit-bang, no hardware SPI needed)        ║
 * ║                                                                            ║
 * ║  WIRING (KWS-303WF BK7231N CBU module):                                   ║
 * ║    TFT Pin   Label    CBU Physical   CBU GPIO   Function                   ║
 * ║       3      SDA/SDIN     2            P16      SPI MOSI                   ║
 * ║       4      SCL/SCLK     1            P14      SPI SCK                    ║
 * ║       5      RS/DC        20           P17      Data/Command               ║
 * ║       6      RES          19           P9       Hardware Reset             ║
 * ║       7      CS           21           P15      SPI Chip Select            ║
 * ║       8      GND          13           GND      Ground                     ║
 * ║      10      VDD          14           3V3      Logic Power                ║
 * ║      11      LEDK         12           P24      Backlight (HIGH=ON)        ║
 * ║      12      LEDA         14           3V3      Backlight power (51.2Ω)    ║
 * ║      13      GND          13           GND      Ground                     ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CONSOLE COMMANDS                                                          ║
 * ║                                                                            ║
 * ║  startDriver ST7735 14 16 9 17 15 24  — init (SCK SDA RES DC CS BLK)     ║
 * ║  st7735_clear                          — fill screen black                ║
 * ║  st7735_clear 0xF800                   — fill screen with colour          ║
 * ║  st7735_brightness 0|1                 — backlight off/on                 ║
 * ║  st7735_goto <col> <row>               — set text cursor                  ║
 * ║  st7735_print <text>                   — print string at cursor           ║
 * ║  st7735_color <fg> [bg]                — set text colours (RGB565 hex)    ║
 * ║  st7735_draw <x> <y> <w> <h> <colour> — filled rectangle                 ║
 * ║  st7735_update                         — redraw HT7017 energy screen      ║
 * ║  st7735_scale 1|2|3                    — set text scale                   ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DISPLAY LAYOUT (portrait 80×160, mimics original KWS-303WF screen)      ║
 * ║                                                                            ║
 * ║   Row 0  ┌──────────────────┐  "ON" (green)                               ║
 * ║   Row 1  │  238.4 V         │  (red)    large                             ║
 * ║   Row 2  │    0.000 A       │  (yellow) large                             ║
 * ║   Row 3  │    0.0 W         │  (blue)   large                             ║
 * ║   Row 4  │  0.000kWh        │  (cyan)   small                             ║
 * ║   Row 5  │  PF 0.00         │  (red)    small                             ║
 * ║   Row 6  │  50.04Hz         │  (green)  small                             ║
 * ║   Row 7  │  0.0C            │  (red)    small                             ║
 * ║          └──────────────────┘                                              ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  BUILD RULES                                                               ║
 * ║    No channel.h  |  No powerMeasurementCalibration  |  No CFG_SetExtended ║
 * ║    Use HAL_PIN_Setup_Output / HAL_PIN_SetOutputValue for GPIO              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── Display geometry ──────────────────────────────────────────────────────────
#define ST7735_WIDTH        80
#define ST7735_HEIGHT       160

// ST7735S chip RAM is 132×162; visible 80×160 panel starts at these offsets
#define ST7735_COL_OFFSET   26   // (132 - 80) / 2 = 26
#define ST7735_ROW_OFFSET   1

// ─── ST7735S command set ───────────────────────────────────────────────────────
#define ST77_NOP        0x00
#define ST77_SWRESET    0x01
#define ST77_SLPOUT     0x11
#define ST77_NORON      0x13
#define ST77_INVOFF     0x20
#define ST77_INVON      0x21
#define ST77_DISPOFF    0x28
#define ST77_DISPON     0x29
#define ST77_CASET      0x2A
#define ST77_RASET      0x2B
#define ST77_RAMWR      0x2C
#define ST77_MADCTL     0x36
#define ST77_COLMOD     0x3A
#define ST77_FRMCTR1    0xB1
#define ST77_FRMCTR2    0xB2
#define ST77_FRMCTR3    0xB3
#define ST77_INVCTR     0xB4
#define ST77_PWCTR1     0xC0
#define ST77_PWCTR2     0xC1
#define ST77_PWCTR3     0xC2
#define ST77_PWCTR4     0xC3
#define ST77_PWCTR5     0xC4
#define ST77_VMCTR1     0xC5
#define ST77_GMCTRP1    0xE0
#define ST77_GMCTRN1    0xE1

// ─── MADCTL bits ───────────────────────────────────────────────────────────────
#define MADCTL_MY   0x80   // row address order
#define MADCTL_MX   0x40   // column address order
#define MADCTL_MV   0x20   // row/column exchange
#define MADCTL_ML   0x10   // vertical refresh order
#define MADCTL_BGR  0x08   // BGR colour order
#define MADCTL_MH   0x04   // horizontal refresh order

// ─── RGB565 colour constants ───────────────────────────────────────────────────
#define ST7735_BLACK    0x0000
#define ST7735_WHITE    0xFFFF
#define ST7735_RED      0xF800
#define ST7735_GREEN    0x07E0
#define ST7735_BLUE     0x001F
#define ST7735_YELLOW   0xFFE0
#define ST7735_CYAN     0x07FF
#define ST7735_MAGENTA  0xF81F
#define ST7735_ORANGE   0xFD20
#define ST7735_GREY     0x8410
#define ST7735_DARKGREY 0x4208

// ─── Font metrics ──────────────────────────────────────────────────────────────
#define FONT_W      5    // glyph pixel width
#define FONT_H      7    // glyph pixel height
#define FONT_ADV    6    // horizontal advance per character (px)
#define FONT_VADV   10   // vertical advance per row (px)

// Text scale multipliers
#define ST7735_TEXT_SCALE_LARGE  2
#define ST7735_TEXT_SCALE_SMALL  1

// ─── Default pin assignments (CBU GPIO numbers) ────────────────────────────────
#define ST7735_DEFAULT_SCK   14   // CBU P14 — SPI SCK
#define ST7735_DEFAULT_SDA   16   // CBU P16 — SPI MOSI
#define ST7735_DEFAULT_RES   9    // CBU P9  — Reset
#define ST7735_DEFAULT_DC    17   // CBU P17 — Data/Command
#define ST7735_DEFAULT_CS    15   // CBU P15 — Chip Select
#define ST7735_DEFAULT_BLK   24   // CBU P24 — Backlight (HIGH=ON)

// ─── Public API ────────────────────────────────────────────────────────────────
void ST7735_Init(void);
void ST7735_RunEverySecond(void);
void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request);

// Drawing primitives
void ST7735_FillScreen(uint16_t colour);
void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour);
void ST7735_DrawChar(uint8_t x, uint8_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
void ST7735_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);

// Energy screen — called by HT7017 after measurements update
void ST7735_DrawEnergyScreen(float v, float a, float w,
                              float kwh, float pf, float hz,
                              float temp_c, uint8_t relay_on);

#endif // __DRV_ST7735_H__
