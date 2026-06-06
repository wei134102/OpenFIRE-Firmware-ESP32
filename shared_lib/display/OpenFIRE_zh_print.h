/*!
 * @file OpenFIRE_zh_print.h
 * @brief Compact 12x12 Chinese glyph renderer for Adafruit_GFX / LovyanGFX.
 */
#ifndef _OPENFIRE_ZH_PRINT_H_
#define _OPENFIRE_ZH_PRINT_H_

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

#if defined(OLED_MENU_ZH)
#include "OpenFIREdisplay_zh_glyphs.h"

namespace OpenFIREZh {

static inline const uint8_t *lookupGlyph(const char *utf8, size_t &consumed)
{
    if(utf8 == nullptr || *utf8 == '\0') {
        consumed = 0;
        return nullptr;
    }

    for(unsigned i = 0; i < ZH_GLYPH_COUNT; ++i) {
        const ZhGlyphEntry &entry = kZhGlyphs[i];
        if(entry.key_len == 0) {
            continue;
        }
        bool match = true;
        for(uint8_t j = 0; j < entry.key_len; ++j) {
            if((uint8_t)utf8[j] != entry.key[j]) {
                match = false;
                break;
            }
        }
        if(match) {
            consumed = entry.key_len;
            return entry.data;
        }
    }

    consumed = 1;
    return nullptr;
}

static inline int textWidth(const char *text)
{
    int width = 0;
    const char *p = text;
    while(p != nullptr && *p != '\0') {
        size_t consumed = 0;
        if(lookupGlyph(p, consumed) != nullptr) {
            width += ZH_GLYPH_W;
        } else {
            width += 6;
            consumed = 1;
        }
        p += consumed;
    }
    return width;
}

template<typename GfxT>
static inline void printAt(GfxT *disp, int x, int y, const char *text, uint16_t fg, uint16_t bg)
{
    if(disp == nullptr || text == nullptr) {
        return;
    }

    const int width = textWidth(text);
    if(bg != 0) {
        disp->fillRect(x, y, width, ZH_GLYPH_H, bg);
    }

    int cx = x;
    const char *p = text;
    while(p != nullptr && *p != '\0') {
        size_t consumed = 0;
        const uint8_t *glyph = lookupGlyph(p, consumed);
        if(glyph != nullptr) {
            disp->drawBitmap(cx, y, glyph, ZH_GLYPH_W, ZH_GLYPH_H, fg);
            cx += ZH_GLYPH_W;
        } else {
            disp->setTextSize(1);
            disp->setTextColor(fg, bg);
            disp->setCursor(cx, y + 2);
            disp->write((uint8_t)*p);
            cx += 6;
            consumed = 1;
        }
        p += consumed;
    }
}

template<typename GfxT>
static inline void printfAt(GfxT *disp, int x, int y, uint16_t fg, uint16_t bg, const char *fmt, ...)
{
    if(disp == nullptr || fmt == nullptr) {
        return;
    }
    char buf[72];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    printAt(disp, x, y, buf, fg, bg);
}

} // namespace OpenFIREZh

#define OF_ZH_PRINT(disp, x, y, text, fg, bg) OpenFIREZh::printAt((disp), (x), (y), (text), (fg), (bg))
#define OF_ZH_PRINTF(disp, x, y, fg, bg, fmt, ...) OpenFIREZh::printfAt((disp), (x), (y), (fg), (bg), (fmt), ##__VA_ARGS__)

#else

#define OF_ZH_PRINT(disp, x, y, text, fg, bg) \
    do { \
        if((disp) != nullptr) { \
            (disp)->setTextSize(1); \
            (disp)->setTextColor((fg), (bg)); \
            (disp)->setCursor((x), (y)); \
            (disp)->print((text)); \
        } \
    } while(0)

#define OF_ZH_PRINTF(disp, x, y, fg, bg, fmt, ...) \
    do { \
        if((disp) != nullptr) { \
            (disp)->setTextSize(1); \
            (disp)->setTextColor((fg), (bg)); \
            (disp)->setCursor((x), (y)); \
            (disp)->printf((fmt), ##__VA_ARGS__); \
        } \
    } while(0)

#endif // OLED_MENU_ZH

#endif // _OPENFIRE_ZH_PRINT_H_
