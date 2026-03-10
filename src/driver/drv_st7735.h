#pragma once
#ifndef __DRV_ST7735_H__
#define __DRV_ST7735_H__

/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║  ST7735S — 0.96" 80×160 Color TFT Driver for OpenBK7231T / BK7231N        ║
 * ║  Target device : KWS-303WF  (FPC-JL096B005-01V0 display module)           ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  INTERFACE: 4-wire Software SPI (bit-bang)                                 ║
 * ║                                                                            ║
 * ║  WIRING (CBU module GPIO numbers):                                         ║
 * ║    TFT       CBU GPIO   Function                                           ║
 * ║    SCL/SCLK   P14       SPI SCK                                            ║
 * ║    SDA/SDIN   P16       SPI MOSI                                           ║
 * ║    RES         P9       Hardware Reset (active LOW)                        ║
 * ║    RS/DC       P17      Data=HIGH / Command=LOW                            ║
 * ║    CS          P15      Chip Select (active LOW)                           ║
 * ║    LEDK        P24      Backlight (HIGH=ON)                                ║
 * ║    LEDA        3V3      Backlight power via 51.2Ω                          ║
 * ║    VDD         3V3      Logic power                                        ║
 * ║    GND         GND                                                         ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  startDriver ST7735 14 16 9 17 15 24                                       ║
 * ║               SCK  SDA RES DC CS BLK                                       ║
 * ║                                                                            ║
 * ║  CONSOLE COMMANDS:                                                         ║
 * ║  st7735_clear [colour]          fill screen (default black)               ║
 * ║  st7735_brightness 0|1          backlight off/on                          ║
 * ║  st7735_goto <col> <row>        set text cursor                           ║
 * ║  st7735_print <text>            print at cursor                           ║
 * ║  st7735_color <fg> [bg]         RGB565 hex colours                        ║
 * ║  st7735_draw <x> <y> <w> <h> <colour>  filled rect                       ║
 * ║  st7735_update                  force immediate energy screen redraw      ║
 * ║  st7735_scale 1|2|3             set console text scale                    ║
 * ║                                                                            ║
 * ╠══════════════════════════════════════════════════════════════════════════════╣
 * ║  FLICKER-FREE LAYOUT  80×160px                                             ║
 * ║                                                                            ║
 * ║  Y=  0  [ON/OFF]  scale1          [xx.xxHz]  scale1                       ║
 * ║  Y= 11  Voltage  xxxxx [V]        scale2  RED                              ║
 * ║  Y= 40  Current  xxxxx [A]        scale2  YELLOW                          ║
 * ║  Y= 69  Power    xxxxx [W]        scale2  BLUE                            ║
 * ║  Y= 98  kWh: xxxxxxx              scale1  CYAN                            ║
 * ║  Y=114  PF:  xxxx                 scale1  RED                             ║
 * ║  Y=130  Tmp: xxxxx C              scale1  ORANGE                          ║
 * ║  Y=146  (spare)                                                            ║
 * ║                                                                            ║
 * ║  Static labels drawn ONCE at init.                                         ║
 * ║  Each update only redraws the changed numeric zone → zero flicker.        ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 */

#include "../new_common.h"
#include "../httpserver/new_http.h"

// ─── Display geometry ──────────────────────────────────────────────────────────
#define ST7735_WIDTH        80
#define ST7735_HEIGHT       160

// ST7735S chip RAM = 132×162; visible 80×160 starts at these offsets
#define ST7735_COL_OFFSET   24
#define ST7735_ROW_OFFSET   0

// ─── ST7735S command bytes ─────────────────────────────────────────────────────
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
#define MADCTL_MY   0x80
#define MADCTL_MX   0x40
#define MADCTL_MV   0x20
#define MADCTL_ML   0x10
#define MADCTL_BGR  0x08
#define MADCTL_MH   0x04

// ─── RGB565 colours ────────────────────────────────────────────────────────────
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

// ─── Font metrics (5×7 glyph, 6px advance) ────────────────────────────────────
#define FONT_W      5
#define FONT_H      7
#define FONT_ADV    6
#define FONT_VADV   10

// ─── Text scale aliases ────────────────────────────────────────────────────────
#define ST7735_TEXT_SCALE_LARGE  2
#define ST7735_TEXT_SCALE_SMALL  1

// ─── Default GPIO pin assignments ─────────────────────────────────────────────
#define ST7735_DEFAULT_SCK   14   // CBU P14
#define ST7735_DEFAULT_SDA   16   // CBU P16
#define ST7735_DEFAULT_RES   9    // CBU P9
#define ST7735_DEFAULT_DC    17   // CBU P17
#define ST7735_DEFAULT_CS    15   // CBU P15
#define ST7735_DEFAULT_BLK   24   // CBU P24

// ─── Public API ────────────────────────────────────────────────────────────────
void ST7735_Init(void);
void ST7735_RunEverySecond(void);
/* NOTE: AppendInformationToHTTPIndexPage is NOT implemented — drv_main.c
 * passes NULL for that slot.  Declaration removed (BUG-14).              */

void ST7735_FillScreen(uint16_t colour);
void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t colour);
void ST7735_DrawChar(uint8_t x, uint8_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
void ST7735_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t scale);
/* NOTE: ST7735_DrawEnergyScreen() was removed in the channel-read refactor.
 * display_tick() reads channels directly — no external caller needed.
 * Declaration removed (BUG-14).                                           */

#endif // __DRV_ST7735_H__
