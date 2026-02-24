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
 * ║  WIRING (recommended free pins on KWS-303WF BK7231N):                    ║
 * ║    Display       BK7231N                                                   ║
 * ║    SCK    ──────► P9                                                       ║
 * ║    SDA    ──────► P10  (MOSI only — display is write-only)                ║
 * ║    RES    ──────► P11  (reset, active LOW)                                ║
 * ║    DC     ──────► P14  (data=HIGH / command=LOW)                          ║
 * ║    CS     ──────► P16  (chip select, active LOW)                          ║
 * ║    BLK    ──────► P17  (backlight, HIGH=on) or tie to 3.3V               ║
 * ║    VCC    ──────► 3.3V                                                     ║
 * ║    GND    ──────► GND                                                      ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  CONSOLE COMMANDS (after startDriver ST7735 SCK SDA RES DC CS BLK)       ║
 * ║                                                                            ║
 * ║  startDriver ST7735 9 10 11 14 16 17   — init with pin numbers            ║
 * ║  st7735_clear                           — fill screen black               ║
 * ║  st7735_clear 0xF800                    — fill screen with colour         ║
 * ║  st7735_brightness 0|1                  — backlight off/on                ║
 * ║  st7735_goto <col> <row>                — set text cursor (col 0-12, row 0-9)
 * ║  st7735_print <text>                    — print string at cursor          ║
 * ║  st7735_color <fg> [bg]                 — set text colours (RGB565 hex)   ║
 * ║  st7735_draw <x> <y> <w> <h> <colour>  — filled rectangle                ║
 * ║  st7735_update                          — redraw HT7017 energy screen     ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  DISPLAY LAYOUT (portrait 80×160, mimics original KWS-303WF screen)      ║
 * ║                                                                            ║
 * ║   Row 0  ┌─────────────────┐  "ON"  (green) + WiFi icon                  ║
 * ║   Row 1  │  227.5 V        │  (red)                                       ║
 * ║   Row 2  │    4.032 A      │  (yellow)                                    ║
 * ║   Row 3  │    913.8 W      │  (blue)                                      ║
 * ║   Row 4  │   0.000 kWh     │  (cyan)                                      ║
 * ║   Row 5  │   0.00 PF :000T │  (white)                                     ║
 * ║   Row 6  │   50.03 Hz      │  (green)                                     ║
 * ║   Row 7  │   32 °C         │  (red)                                       ║
 * ║          └─────────────────┘                                               ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  BUILD RULES (same as rest of HT7017 driver)                               ║
 * ║    No channel.h  |  No powerMeasurementCalibration  |  No CFG_SetExtended ║
 * ║    Use HAL_PIN_Setup_Output / HAL_PIN_SetOutputValue for GPIO              ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── Display geometry ─────────────────────────────────────────────────────────
// 0.96" FPC-JL096B005 is 80×160 in portrait (same as MINI160x80 in landscape)
#define ST7735_WIDTH        80
#define ST7735_HEIGHT       160

// Column/row offsets — green-tab 80×160 variant uses these
// (chip RAM is 132×162; our visible area starts at col+2, row+1)
#define ST7735_COL_OFFSET   2   // for 80px wide on 132px RAM: (132-80)/2 = 26
#define ST7735_ROW_OFFSET   0    // standard for this panel

// ─── ST7735S command set ──────────────────────────────────────────────────────
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

// MADCTL bits
#define MADCTL_MY   0x80   // row address order
#define MADCTL_MX   0x40   // column address order
#define MADCTL_MV   0x20   // row/column exchange
#define MADCTL_ML   0x10   // vertical refresh order
#define MADCTL_BGR  0x08   // BGR colour order (vs RGB)
#define MADCTL_MH   0x04   // horizontal refresh order

// ─── RGB565 colour constants ──────────────────────────────────────────────────
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

// ─── Font: 5×7 pixel chars, 6px advance ──────────────────────────────────────
#define FONT_W      5
#define FONT_H      7
#define FONT_ADV    6    // horizontal advance per character (px)
#define FONT_VADV   10   // vertical advance per row (px)

// Text size multiplier — 2 = large, 1 = small
#define ST7735_TEXT_SCALE_LARGE  2
#define ST7735_TEXT_SCALE_SMALL  1

// ─── Default pin assignments ──────────────────────────────────────────────────
#define ST7735_DEFAULT_SCK   9
#define ST7735_DEFAULT_SDA   10
#define ST7735_DEFAULT_RES   11
#define ST7735_DEFAULT_DC    14
#define ST7735_DEFAULT_CS    16
#define ST7735_DEFAULT_BLK   17

// ─── Public API ───────────────────────────────────────────────────────────────
void ST7735_Init(void);
void ST7735_RunEverySecond(void);
void ST7735_AppendInformationToHTTPIndexPage(http_request_t *request);

// Low-level drawing (available for external use)
void ST7735_FillScreen(uint16_t colour);
void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour);
void ST7735_DrawChar(uint8_t x, uint8_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
void ST7735_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);

// Energy screen refresh — called by HT7017 after measurements update
void ST7735_DrawEnergyScreen(float v, float a, float w,
                              float kwh, float pf, float hz,
                              float temp_c, uint8_t relay_on);

#endif // __DRV_ST7735_H__
