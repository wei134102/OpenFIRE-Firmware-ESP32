/*!
 * @file OpenFIREdisplay.cpp
 * @brief Macros for lightgun HUD display (primarily for SSD1306 OLED modules).
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V2.0
 * @date 2026
 *
 * I thank you for producing the first original code:
 *
 * @copyright That One Seong, 2024
 * @copyright GNU Lesser General Public License
 */

// we're using our own splash screen kthx ada
#define SSD1306_NO_SPLASH

#include <Arduino.h>
#include <cstdarg>

#ifdef USE_LOVYAN_GFX
  // nulla 
#else
  //#include <Adafruit_GFX.h>
#endif

#include <Wire.h>
#include <TinyUSB_Devices.h>

#include "OpenFIREdisplay.h"
#include "OpenFIREdisplay_i18n.h"
#include "OpenFIREprefs.h"
#include "OpenFIREFeedback.h"
#include "OpenFIREDefines.h"
#include "OpenFIREcommon.h" //wei134102 add
#include "OpenFIREPlayTimer.h"

#if defined(OLED_MENU_ZH) && defined(USE_LOVYAN_GFX)
  #include "OpenFIREdisplay_zh_glyphs.h"  /* shared_lib/display via -I */
#endif

namespace {

#if defined(OLED_MENU_ZH) && defined(USE_LOVYAN_GFX)
static const uint8_t *lookupZhGlyph(const char *utf8, size_t &consumed)
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

static int pauseMenuTextWidth(const char *text)
{
    int width = 0;
    const char *p = text;
    while(p != nullptr && *p != '\0') {
        size_t consumed = 0;
        if(lookupZhGlyph(p, consumed) != nullptr) {
            width += ZH_GLYPH_W;
        } else {
            width += 6;
            consumed = 1;
        }
        p += consumed;
    }
    return width;
}

static void pauseMenuPrintZh(LGFX_SSD1306 *disp, int x, int y, const char *text, uint16_t fg, uint16_t bg)
{
    const int textWidth = pauseMenuTextWidth(text);
    if(bg != BLACK) {
        disp->fillRect(x, y, textWidth, ZH_GLYPH_H, bg);
    }

    int cx = x;
    const char *p = text;
    while(p != nullptr && *p != '\0') {
        size_t consumed = 0;
        const uint8_t *glyph = lookupZhGlyph(p, consumed);
        if(glyph != nullptr) {
            disp->drawBitmap(cx, y, glyph, ZH_GLYPH_W, ZH_GLYPH_H, fg);
            cx += ZH_GLYPH_W;
        } else {
            disp->setFont(nullptr);
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
#endif

static void pauseMenuPrint(
    #ifdef USE_LOVYAN_GFX
    LGFX_SSD1306 *disp,
    #else
    Adafruit_SSD1306 *disp,
    #endif
    int x, int y, const char *text, uint16_t fg, uint16_t bg)
{
    if(!disp || !text) {
        return;
    }
#if defined(OLED_MENU_ZH) && defined(USE_LOVYAN_GFX)
    pauseMenuPrintZh(disp, x, y, text, fg, bg);
#else
    disp->setTextSize(1);
    disp->setTextColor(fg, bg);
    disp->setCursor(x, y);
    disp->print(text);
#endif
}

static void pauseMenuPrintf(
    #ifdef USE_LOVYAN_GFX
    LGFX_SSD1306 *disp,
    #else
    Adafruit_SSD1306 *disp,
    #endif
    int x, int y, uint16_t fg, uint16_t bg, const char *fmt, ...)
{
    if(!disp || !fmt) {
        return;
    }
    char buf[56];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    pauseMenuPrint(disp, x, y, buf, fg, bg);
}

static const char *pmOnOff(bool on)
{
    return on ? PM_ON : PM_OFF;
}

static const char *pmLayoutStr(uint8_t layout)
{
    return (layout == OF_Const::layoutDiamond) ? PM_LAYOUT_DIAMOND : PM_LAYOUT_SQUARE;
}

static const char *pmAnalogModeStr(uint8_t mode)
{
    switch(mode) {
        case OF_Const::analogModeDpad:
            return PM_MODE_DPAD;
        case OF_Const::analogModeKeys:
            return PM_MODE_KEYS;
        default:
            return PM_MODE_STICK;
    }
}

static const char *pmAnalogModeStrAlt(uint8_t mode)
{
    switch(mode) {
        case OF_Const::analogModeDpad:
            return PM_MODE_DPAD;
        case OF_Const::analogModeKeys:
            return PM_MODE_KEYS_ALT;
        default:
            return PM_MODE_STICK;
    }
}

static const char *pmAxisSignStr(bool unsignedMode)
{
    return unsignedMode ? PM_AXIS_UNSIGNED : PM_AXIS_SIGNED;
}

static const char *pmInvertStr(bool inv)
{
    return inv ? PM_INVERTED : PM_NORMAL;
}

static const char *pmInvertShortStr(bool inv)
{
    return inv ? PM_INV_SHORT : PM_NORM_SHORT;
}

} // namespace

bool ExtDisplay::Begin()
{
    if(display != nullptr) {
        delete display;
        display = nullptr;
    }

    // TODO: for some reason, doing this AFTER saving updated pins settings (even when doing it from defaults and there's no default mappings for peripheral pins)
    // causes the board to hang. Even though this is all correct (and any display objects should get deleted from the above, so don't think it can be a new object thing)...
    if(OF_Prefs::pins[OF_Const::periphSCL] >= 0 && OF_Prefs::pins[OF_Const::periphSDA] >= 0) {
      #ifdef ARDUINO_ARCH_ESP32
        Wire1.setPins(OF_Prefs::pins[OF_Const::periphSDA], OF_Prefs::pins[OF_Const::periphSCL]);  // [ESP32_PORT] per esp32
        #ifdef USE_LOVYAN_GFX
        display = new LGFX_SSD1306(1/*i2c_port wire_1*/,OF_Prefs::pins[OF_Const::periphSDA], OF_Prefs::pins[OF_Const::periphSCL], OF_Prefs::toggles[OF_Const::i2cOLEDaltAddr] ? 0x3D : 0x3C, SCREEN_WIDTH, SCREEN_HEIGHT);
        #else
        display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);
        #endif
      #else // rp2040
        if(bitRead(OF_Prefs::pins[OF_Const::periphSCL], 1) && bitRead(OF_Prefs::pins[OF_Const::periphSDA], 1)) {
            // I2C1
            if(bitRead(OF_Prefs::pins[OF_Const::periphSCL], 0) && !bitRead(OF_Prefs::pins[OF_Const::periphSDA], 0)) {
                Wire1.end();
                // SDA/SCL are indeed on verified correct pins
                Wire1.setSDA(OF_Prefs::pins[OF_Const::periphSDA]);
                Wire1.setSCL(OF_Prefs::pins[OF_Const::periphSCL]);
                #ifdef USE_LOVYAN_GFX
                display = new LGFX_SSD1306(1/*i2c_port wire_1*/,OF_Prefs::pins[OF_Const::periphSDA], OF_Prefs::pins[OF_Const::periphSCL], OF_Prefs::toggles[OF_Const::i2cOLEDaltAddr] ? 0x3D : 0x3C, SCREEN_WIDTH, SCREEN_HEIGHT);
                #else
                display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);
                #endif
            } else return false;
        } else if(!bitRead(OF_Prefs::pins[OF_Const::periphSCL], 1) && !bitRead(OF_Prefs::pins[OF_Const::periphSDA], 1)) {
            // I2C0
            if(bitRead(OF_Prefs::pins[OF_Const::periphSCL], 0) && !bitRead(OF_Prefs::pins[OF_Const::periphSDA], 0)) {
                Wire.end();
                // SDA/SCL are indeed on verified correct pins
                Wire.setSDA(OF_Prefs::pins[OF_Const::periphSDA]);
                Wire.setSCL(OF_Prefs::pins[OF_Const::periphSCL]);
                #ifdef USE_LOVYAN_GFX
                display = new LGFX_SSD1306(0/*i2c_port wire*/,OF_Prefs::pins[OF_Const::periphSDA], OF_Prefs::pins[OF_Const::periphSCL], OF_Prefs::toggles[OF_Const::i2cOLEDaltAddr] ? 0x3D : 0x3C, SCREEN_WIDTH, SCREEN_HEIGHT);
                #else
                display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
                #endif
            } else return false;
        } else return false;
      #endif
    } else return false;

    #ifdef USE_LOVYAN_GFX
    if(display->init()) {
        // 配置SSD1306驱动参数（针对0.91寸的特殊配置）
        #ifdef OLED_091_INCH
        ((LGFX_SSD1306*)display)->configureSSD1306Params();
        #endif
    #else
    if(display->begin(SSD1306_SWITCHCAPVCC, OF_Prefs::toggles[OF_Const::i2cOLEDaltAddr] ? 0x3D : 0x3C)) {
        // Adafruit库的SSD1306需要手动配置0.91寸的参数
        #ifdef OLED_091_INCH
        // 0.91寸OLED配置：驱动路数0x1F(32 duty), com pin配置0x02
        // 注意：Adafruit_SSD1306库可能需要通过命令接口设置这些参数
        // 如果库不支持，可能需要修改库或使用其他方法
        #endif
    #endif    
        display->clearDisplay();
        ScreenModeChange(Screen_None);
        return true;
    } else return false;
}

void ExtDisplay::Stop()
{
    if(display != nullptr) {
        delete display;
        display = nullptr;
    }
}

// void ExtDisplay::TopPanelUpdate(const char *textPrefix, const char *profText)
// {
//     if(display != nullptr) {
//         display->fillRect(0, 0, 128, 16, BLACK);
//         display->drawFastHLine(0, 15, 128, WHITE);
//         display->setCursor(2, 2);
//         display->setTextSize(1);
//         display->setTextColor(WHITE, BLACK);
//         display->print(textPrefix);
//         if(profText != nullptr)
//             display->println(profText);
//         display->display();
//     }
// }
void ExtDisplay::TopPanelUpdate(const char *textPrefix, const char *profText)
{
    if(display != nullptr) {
        display->fillRect(0, 0, 128, 16, BLACK);
        display->drawFastHLine(0, 15, 128, WHITE);

        const bool isProfileBar = (textPrefix != nullptr &&
            (strcmp(textPrefix, "Prof: ") == 0 || strcmp(textPrefix, TXT_PROF_PREFIX) == 0));
        const bool isUsingBar = (textPrefix != nullptr &&
            (strcmp(textPrefix, "Using ") == 0 || strcmp(textPrefix, TXT_USING_PREFIX) == 0));
        const bool isCaliBar = (textPrefix != nullptr &&
            (strcmp(textPrefix, "Cali: ") == 0 || strcmp(textPrefix, TXT_CALI_PREFIX) == 0));

        int x = 2;
        if(isProfileBar) {
            pauseMenuPrint(display, x, 2, TXT_PROF_PREFIX, WHITE, BLACK);
            x += pauseMenuTextWidth(TXT_PROF_PREFIX);
        } else if(isUsingBar) {
            pauseMenuPrint(display, x, 2, TXT_USING_PREFIX, WHITE, BLACK);
            x += pauseMenuTextWidth(TXT_USING_PREFIX);
        } else if(isCaliBar) {
            pauseMenuPrint(display, x, 2, TXT_CALI_PREFIX, WHITE, BLACK);
            x += pauseMenuTextWidth(TXT_CALI_PREFIX);
        } else if(textPrefix != nullptr && textPrefix[0] != '\0') {
            if(profText == nullptr) {
                pauseMenuPrint(display, 2, 2, textPrefix, WHITE, BLACK);
            } else {
                pauseMenuPrint(display, x, 2, textPrefix, WHITE, BLACK);
                x += pauseMenuTextWidth(textPrefix);
            }
        }

        if(profText != nullptr) {
            const char *namePtr = profText;
            if(isProfileBar && strncmp(profText, "Profile ", 8) == 0) {
                namePtr = profText + 8;
            }
            pauseMenuPrint(display, x, 2, namePtr, WHITE, BLACK);
            x += pauseMenuTextWidth(namePtr);

            if(isProfileBar) {
                char gunBuf[8];
                snprintf(gunBuf, sizeof(gunBuf), " P%d", (int)((OF_Prefs::settings[OF_Const::gunId] % 4) + 1));
                pauseMenuPrint(display, x, 2, gunBuf, WHITE, BLACK);
            }
        }

        display->display();
        UpdateTimerCountdown((uint16_t)PlayTimer::GetRemainingSecondsLive());
    }
}

void ExtDisplay::UpdateTimerCountdown(uint16_t seconds)
{
    if (display == nullptr)
        return;

    // 计时数字区域宽度预留约 3 个字符（右侧）
    const uint8_t width = 26;
    const uint8_t x = 128 - width;
    const uint8_t y = 0;

    // 清除原有数字区域，保留下边框线
    display->fillRect(x, y, width, 15, BLACK);
    display->drawFastHLine(x, 15, width, WHITE);

    char buf[5];
    // 根据 PlayTimer 状态区分 "OFF"（未配置计时）和 "0"（计时结束）
    bool timerExpired = PlayTimer::IsExpired();
    bool timerOff = (PlayTimer::minutes == 0) && !timerExpired;

    if (seconds > 0) {
        if (seconds > 999) seconds = 999;
        snprintf(buf, sizeof(buf), "%3u", (unsigned)seconds);
    } else if (timerOff) {
        snprintf(buf, sizeof(buf), "%s", TXT_STATUS_OFF);
    } else if (timerExpired) {
        snprintf(buf, sizeof(buf), "  0");
    } else {
        snprintf(buf, sizeof(buf), "%s", TXT_STATUS_OFF);
    }
    pauseMenuPrint(display, x + 1, 2, buf, WHITE, BLACK);

    display->display();
}
void ExtDisplay::ScreenModeChange(const int &screenMode, const bool &isAnalog)
{
    if(display != nullptr) {
        idleTimeStamp = millis();
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        if(screenState >= Screen_Mamehook_Single &&
           screenMode == Screen_Normal) {
            currentAmmo = 0, currentLife = 0;
        }
        screenState = screenMode;
        display->setTextColor(WHITE, BLACK);
        switch(screenMode) {
          case Screen_Normal:
            if(mister) display->drawBitmap(48, 23, misterIco, MISTERKUN_WIDTH, MISTERKUN_HEIGHT, WHITE);
            else {
                #ifdef OLED_091_INCH  // 0.91寸屏幕显示底部图标在下半部分
                // 为0.91寸屏幕显示连接图标在屏幕下半部分（Y=16-31区域）
                #ifdef ARDUINO_ARCH_ESP32
                if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 24, wifiConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                #else //rp2040
                if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 24, btConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                #endif
                else { display->drawBitmap(2, 18, usbConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                
                // 显示Rumble Force Feedback状态
                #ifdef USES_RUMBLE
                if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::toggles[OF_Const::rumbleFF]) {
                    display->setCursor(46, 20);
                    display->setTextSize(1);
                    display->print("RF");
                }
                #endif                
                //wei134102 add start 显示Low Button模式状态
                if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
                    display->setCursor(64, 20);
                    display->setTextSize(1);
                    display->print("LOW");
                }
                // 显示Autofire状态
                if(OF_Prefs::toggles[OF_Const::autofire]) {
                    display->setCursor(82, 20);
                    display->setTextSize(1);
                    display->print(" AF");
                }                
                //wei134102 add end                
                if(isAnalog) { display->drawBitmap(108, 20, gamepadIco, GAMEPAD_WIDTH, GAMEPAD_HEIGHT, WHITE); }
                else { display->drawBitmap(109, 20, mouseIco, MOUSE_WIDTH, MOUSE_HEIGHT, WHITE); }
                #else // 非0.91寸屏幕的原代码
                #ifdef ARDUINO_ARCH_ESP32
                if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 46, wifiConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                #else //rp2040
                if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 46, btConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                #endif
                else { display->drawBitmap(2, 46, usbConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
                // 显示Rumble Force Feedback状态
                #ifdef USES_RUMBLE
                if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::toggles[OF_Const::rumbleFF]) {
                    display->setCursor(46, 48);
                    display->setTextSize(1);
                    display->print("RF");
                }
                #endif                
                //wei134102 add start 显示Low Button模式状态
                if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
                    display->setCursor(64, 48);
                    display->setTextSize(1);
                    display->print("LOW");
                }
                // 显示Autofire状态
                if(OF_Prefs::toggles[OF_Const::autofire]) {
                    display->setCursor(82, 48);
                    display->setTextSize(1);
                    display->print(" AF");
                }                
                //wei134102 add end                
                if(isAnalog) { display->drawBitmap(108, 49, gamepadIco, GAMEPAD_WIDTH, GAMEPAD_HEIGHT, WHITE); }
                else { display->drawBitmap(109, 48, mouseIco, MOUSE_WIDTH, MOUSE_HEIGHT, WHITE); }
                #endif // 非0.91寸屏幕的原代码
            }
            break;
          case Screen_None:
          case Screen_Docked:
            display->fillRect(0, 0, 128, 16, BLACK);
            display->drawBitmap(24, 0, customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT, WHITE);
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：适配较小的logo位置
            // display->drawBitmap(56, 16, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, WHITE);
            display->setCursor(10, 17);
            display->setTextSize(2);
            display->print("Open FIRE");
            #else
            display->drawBitmap(40, 16, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, WHITE);
            #endif
            display->display();
            break;
          case Screen_Init:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 28, 18, TXT_WELCOME, WHITE, BLACK);
            pauseMenuPrint(display, 16, 28, TXT_PULL_TRIGGER, WHITE, BLACK);
            #else
            pauseMenuPrint(display, 36, 22, TXT_WELCOME, WHITE, BLACK);
            pauseMenuPrint(display, 8, 40, TXT_PULL_TRIGGER, WHITE, BLACK);
            pauseMenuPrint(display, 8, 52, TXT_PULL_TRIGGER_LONG, WHITE, BLACK);
            #endif
            break;
          case Screen_IRTest:
            TopPanelUpdate("", TXT_IR_TEST);
            break;
          case Screen_Saving:
            TopPanelUpdate("", TXT_SAVING_PROFILES);
            pauseMenuPrint(display, 28, 22, TXT_SAVING, WHITE, BLACK);
            break;
          case Screen_SaveSuccess:
            pauseMenuPrint(display, 40, 22, TXT_SAVE_OK_LINE1, WHITE, BLACK);
            pauseMenuPrint(display, 40, 40, TXT_SAVE_OK_LINE2, WHITE, BLACK);
            break;
          case Screen_SaveError:
            pauseMenuPrint(display, 40, 22, TXT_SAVE_FAIL_LINE1, BLACK, WHITE);
            pauseMenuPrint(display, 40, 40, TXT_SAVE_FAIL_LINE2, BLACK, WHITE);
            break;
          case Screen_Mamehook_Single:
            #ifdef OLED_091_INCH  // 0.91寸屏幕显示底部图标在下半部分
            // 为0.91寸屏幕显示连接图标在屏幕下半部分（Y=16-31区域）
            #ifdef ARDUINO_ARCH_ESP32
            if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 24, wifiConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            #else //rp2040
            if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 24, btConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            #endif
            else { display->drawBitmap(2, 24, usbConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            
            // 显示Rumble Force Feedback状态
            #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::toggles[OF_Const::rumbleFF]) {
                display->setCursor(46, 26);
                display->setTextSize(1);
                display->print("RF");
            }
            #endif                
            // 显示Low Button模式状态
            if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
                display->setCursor(70, 26);
                display->setTextSize(1);
                display->print("LOW");
            }
            // 显示Autofire状态
            if(OF_Prefs::toggles[OF_Const::autofire]) {
                display->setCursor(82, 26);
                display->setTextSize(1);
                display->print(" AF");
            }                        
            //wei134102 add end
            if(isAnalog) { display->drawBitmap(108, 27, gamepadIco, GAMEPAD_WIDTH, GAMEPAD_HEIGHT, WHITE); }
            else { display->drawBitmap(109, 26, mouseIco, MOUSE_WIDTH, MOUSE_HEIGHT, WHITE); }
            #else // 非0.91寸屏幕的原代码
            #ifdef ARDUINO_ARCH_ESP32  
            if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 46, wifiConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            #else //rp2040
            if(TinyUSBDevices.onBattery) { display->drawBitmap(2, 46, btConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            #endif
            else { display->drawBitmap(2, 46, usbConnectIco, CONNECTION_WIDTH, CONNECTION_HEIGHT, WHITE); }
            //wei134102 add start
            // 显示Rumble Force Feedback状态
            #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::toggles[OF_Const::rumbleFF]) {
                display->setCursor(46, 48);
                display->setTextSize(1);
                display->print("RF");
            }
            #endif              
            // 显示Low Button模式状态
            if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
                display->setCursor(70, 48);
                display->setTextSize(1);
                display->print("LOW");
            }
            // 显示Autofire状态
            if(OF_Prefs::toggles[OF_Const::autofire]) {
                display->setCursor(82, 48);
                display->setTextSize(1);
                display->print(" AF");
            }                        
            //wei134102 add end
            if(isAnalog) { display->drawBitmap(108, 49, gamepadIco, GAMEPAD_WIDTH, GAMEPAD_HEIGHT, WHITE); }
            else { display->drawBitmap(109, 48, mouseIco, MOUSE_WIDTH, MOUSE_HEIGHT, WHITE); }
            #endif // 非0.91寸屏幕的原代码
            if(serialDisplayType == ScreenSerial_Life && lifeBar) {
              display->drawBitmap(52, 23, lifeBarBanner, LIFEBAR_BANNER_WIDTH, LIFEBAR_BANNER_HEIGHT, WHITE);
              display->drawBitmap(11, 35, lifeBarLarge, LIFEBAR_LARGE_WIDTH, LIFEBAR_LARGE_HEIGHT, WHITE);
              PrintLife(currentLife);
            } else if(serialDisplayType == ScreenSerial_Ammo) {
              PrintAmmo(currentAmmo);
            } else {
              display->drawBitmap(0, 17, mamehookIco, MAMEHOOK_WIDTH, MAMEHOOK_HEIGHT, WHITE);
            }
            break;
          case Screen_Mamehook_Dual:
            display->drawBitmap(63, 16, dividerLine, DIVIDER_WIDTH, DIVIDER_HEIGHT, WHITE);
            if(lifeBar) {
              display->drawBitmap(20, 23, lifeBarBanner, LIFEBAR_BANNER_WIDTH, LIFEBAR_BANNER_HEIGHT, WHITE);
              display->drawBitmap(2, 37, lifeBarSmall, LIFEBAR_SMALL_WIDTH, LIFEBAR_SMALL_HEIGHT, WHITE);
            }
            PrintAmmo(currentAmmo);
            PrintLife(currentLife);
            break;
        }
        display->display();
    }
}

void ExtDisplay::IdleOps()
{
    if(display != nullptr) {
        switch(screenState) {
        case Screen_Normal:
        case Screen_Mamehook_Single:
        case Screen_Mamehook_Dual:
          // 右侧倒计时：独立于顶部文本刷新，保持按秒更新
          {
              static unsigned long lastCountdownDraw = 0;
              unsigned long now = millis();
              if (now - lastCountdownDraw >= 250) {
                  lastCountdownDraw = now;
                  UpdateTimerCountdown((uint16_t)PlayTimer::GetRemainingSecondsLive());
              }
          }

          // 正常状态栏：始终显示
          // GunID + Profile + Layout + Temp + 右侧倒计时
          if(millis() - idleTimeStamp > OLED_IDLEUPD_INTERVAL) {
              idleTimeStamp = millis();

              uint8_t gunIndex = (uint8_t)(OF_Prefs::settings[OF_Const::gunId] % 4);
              const uint8_t profIdx = (uint8_t)(OF_Prefs::currentProfile % PROFILE_COUNT);

              char line[24];

              #ifdef OLED_MENU_ZH
              const char *layoutStr = (OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond)
                  ? TXT_TOP_LAYOUT_DIAM : TXT_TOP_LAYOUT_SQUA;
              #ifdef USES_TEMP
              char tempBuf[6];
              int temp = OF_FFB::temperatureCurrent;
              if (temp == (int)OF_Const::TEMPERATURE_SENSOR_ERROR_VALUE) {
                  snprintf(tempBuf, sizeof(tempBuf), "%s", TXT_TEMP_ERR_TOP);
              } else {
                  if (temp < 0) temp = 0;
                  if (temp > 99) temp = 99;
                  snprintf(tempBuf, sizeof(tempBuf), "%2dC", temp);
              }
              snprintf(line, sizeof(line), "%u %c %s %s",
                       (unsigned)(gunIndex + 1), (char)('A' + (profIdx % 4)), layoutStr, tempBuf);
              #else
              snprintf(line, sizeof(line), "%u %c %s",
                       (unsigned)(gunIndex + 1), (char)('A' + (profIdx % 4)), layoutStr);
              #endif
              TopPanelUpdate("", line);
              #else
              char prefix[8];
              snprintf(prefix, sizeof(prefix), "P%u: ", (unsigned)(gunIndex + 1));

              char profShort[5] = { 'P', '_', 'A', '\0', '\0' };
              profShort[2] = (char)('A' + (profIdx % 4));
              const char *layoutStr = (OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond)
                  ? TXT_LAYOUT_DIAM_SHORT : TXT_LAYOUT_SQUA_SHORT;

              #ifdef USES_TEMP
              char tempBuf[6];
              int temp = OF_FFB::temperatureCurrent;
              if (temp == (int)OF_Const::TEMPERATURE_SENSOR_ERROR_VALUE) {
                  snprintf(tempBuf, sizeof(tempBuf), "%s", TXT_TEMP_ERR);
              } else {
                  if (temp < 0) temp = 0;
                  if (temp > 99) temp = 99;
                  snprintf(tempBuf, sizeof(tempBuf), "%2dC", temp);
              }
              snprintf(line, sizeof(line), "%s %s %s", profShort, layoutStr, tempBuf);
              #else
              snprintf(line, sizeof(line), "%s %s", profShort, layoutStr);
              #endif

              TopPanelUpdate(prefix, line);
              #endif
          }
          break;
        case Screen_Pause:
        case Screen_Profile:
        case Screen_Saving:
        case Screen_Calibrating:
        default:
          break;
        }
    }
}

#ifdef USES_TEMP
void ExtDisplay::ShowTemp()
{
    if (OF_FFB::temperatureCurrent == (int)OF_Const::TEMPERATURE_SENSOR_ERROR_VALUE) {
        TopPanelUpdate(TXT_TEMP_SENSOR, TXT_TEMP_FAULT);
        return;
    }
    if(OF_FFB::temperatureCurrent < 10) {
        tempString[0] = OF_FFB::temperatureCurrent + '0';
        tempString[1] = ' ';
        tempString[2] = 'C';
        tempString[3] = '\0';
    } else {
        int tempLeft = OF_FFB::temperatureCurrent / 10;
        int tempRight = OF_FFB::temperatureCurrent - (tempLeft * 10);
        tempString[0] = tempLeft + '0';
        tempString[1] = tempRight + '0';
        tempString[2] = ' ';
        tempString[3] = 'C';
        tempString[4] = '\0';
    }

    TopPanelUpdate(TXT_CURRENT_TEMP, tempString);
    currentTemp = OF_FFB::temperatureCurrent;
}
#endif // USES_TEMP

// Warning: SLOOOOW, should only be used in cali/where the mouse isn't being updated.
// Use at your own discression.
void ExtDisplay::DrawVisibleIR(int *pointX, int *pointY)
{
    if(display != nullptr) {
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        for(int i = 0; i < 4; ++i) {
          pointX[i] = map(pointX[i], 0, 1920, 0, 128);
          pointY[i] = map(pointY[i], 0, 1080, 16, SCREEN_HEIGHT);
          pointY[i] = constrain(pointY[i], 16, SCREEN_HEIGHT);
          display->fillCircle(pointX[i], pointY[i], 1, WHITE);
        }
        display->display();
    }
}

void ExtDisplay::PauseScreenShow(const int &currentProf, const char* name1, const char* name2, const char* name3, const char* name4)
{
    if(display != nullptr) {
        const char* namesList[] = { name1, name2, name3, name4 };
        TopPanelUpdate(TXT_USING_PREFIX, namesList[currentProf]);
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        pauseMenuPrint(display, 0, 17, TXT_BTN_A, WHITE, BLACK);
        pauseMenuPrint(display, 24, 17, name1, WHITE, BLACK);
        pauseMenuPrint(display, 0, 28, TXT_BTN_B, WHITE, BLACK);
        pauseMenuPrint(display, 24, 28, name2, WHITE, BLACK);
        pauseMenuPrint(display, 0, 39, TXT_BTN_START, WHITE, BLACK);
        pauseMenuPrint(display, 24, 39, name3, WHITE, BLACK);
        pauseMenuPrint(display, 0, 50, TXT_BTN_SELECT, WHITE, BLACK);
        pauseMenuPrint(display, 24, 50, name4, WHITE, BLACK);
        display->display();
    }
}

void ExtDisplay::PauseListUpdate(const int &selection)
{
    if(display != nullptr) {
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        display->drawBitmap(60, 18, upArrowGlyph, ARROW_WIDTH, ARROW_HEIGHT, WHITE);
        #ifndef OLED_091_INCH  // 0.91寸屏幕不显示底部箭头
        display->drawBitmap(60, 59, downArrowGlyph, ARROW_WIDTH, ARROW_HEIGHT, WHITE);
        #endif
        display->setTextSize(1);
        // Seong Note: Yeah, some of these are pretty out-of-bounds-esque behavior,
        // but pause mode selection in actual use would prevent some of these extremes from happening.
        // Just covering our asses.
        switch(selection) {
          case ScreenPause_AnalogRangeCal:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_RANGE_CAL, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_SAVE_SETTINGS, WHITE, BLACK);
            pauseMenuPrint(display, 0, 36, PM_RANGE_CAL, BLACK, WHITE);
            pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_Calibrate:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_CALIBRATE, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_RANGE_CAL, WHITE, BLACK);
            pauseMenuPrint(display, 0, 36, PM_CALIBRATE, BLACK, WHITE);
            pauseMenuPrint(display, 0, 47, PM_PROFILE_SELECT, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_ProfileSelect:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_PROFILE_SELECT, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_CALIBRATE, WHITE, BLACK);
            pauseMenuPrint(display, 0, 36, PM_PROFILE_SELECT, BLACK, WHITE);
            pauseMenuPrint(display, 0, 47, PM_SAVE_SETTINGS, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_Save:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_SAVE_SETTINGS, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_CENTER_CAL, WHITE, BLACK);
            pauseMenuPrint(display, 0, 36, PM_SAVE_SETTINGS, BLACK, WHITE);
            pauseMenuPrint(display, 0, 47, PM_RANGE_CAL, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_Rumble:
            #ifdef OLED_091_INCH
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 20, PM_RUMBLE_TOGGLE, BLACK, WHITE);
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 20, PM_SOLENOID_TOGGLE, BLACK, WHITE);
            } else {
              pauseMenuPrint(display, 0, 20, PM_ESCAPE_KEY, BLACK, WHITE);
            }
            #else
            pauseMenuPrint(display, 0, 25, PM_SAVE_SETTINGS, WHITE, BLACK);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 36, PM_RUMBLE_TOGGLE, BLACK, WHITE);
              if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
                pauseMenuPrint(display, 0, 47, PM_SOLENOID_TOGGLE, WHITE, BLACK);
              } else {
                pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
              }
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 36, PM_SOLENOID_TOGGLE, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 36, PM_ESCAPE_KEY, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
            }
            #endif
            break;
          case ScreenPause_Solenoid:
            #ifdef OLED_091_INCH
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 20, PM_SOLENOID_TOGGLE, BLACK, WHITE);
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 20, PM_SOLENOID_TOGGLE, BLACK, WHITE);
            } else {
              pauseMenuPrint(display, 0, 20, PM_MODE_CHANGE, BLACK, WHITE);
            }
            #else
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_RUMBLE_TOGGLE, WHITE, BLACK);
              if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
                pauseMenuPrint(display, 0, 36, PM_SOLENOID_TOGGLE, BLACK, WHITE);
                pauseMenuPrint(display, 0, 47, PM_MODE_CHANGE, WHITE, BLACK);
              } else {
                pauseMenuPrint(display, 0, 36, PM_MODE_CHANGE, BLACK, WHITE);
                pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
              }
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_SAVE_SETTINGS, WHITE, BLACK);
              pauseMenuPrint(display, 0, 36, PM_SOLENOID_TOGGLE, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_MODE_CHANGE, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 25, PM_MODE_CHANGE, WHITE, BLACK);
              pauseMenuPrint(display, 0, 36, PM_CALIBRATE, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_PROFILE_SELECT, WHITE, BLACK);
            }
            #endif
            break;
          case ScreenPause_AutofireToggle:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_AUTOFIRE_TOGGLE, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_AUTOFIRE_TOGGLE, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_AUTOFIRE_FMT,
                            pmOnOff(OF_Prefs::toggles[OF_Const::autofire]));
            pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            #endif
            break;
//wei13402 add start
          case ScreenPause_LowButtonToggle:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_LOW_BUTTON, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_LOW_BUTTON, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_LOW_BUTTON_FMT,
                            pmOnOff(OF_Prefs::toggles[OF_Const::lowButtonsMode]));
            pauseMenuPrint(display, 0, 47, PM_LAYOUT_TOGGLE, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_LayoutToggle:
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_LAYOUT_FMT,
                            pmLayoutStr(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout));
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_LOW_BUTTON_FMT,
                            pmOnOff(OF_Prefs::toggles[OF_Const::lowButtonsMode]));
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_LAYOUT_FMT,
                            pmLayoutStr(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout));
            pauseMenuPrint(display, 0, 47, PM_GUN_ID_P1P4, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_GunId:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_LAYOUT_TOGGLE, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 31, BLACK, WHITE, PM_GUN_ID_P_FMT,
                            (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            #else
            pauseMenuPrint(display, 0, 25, PM_LAYOUT_TOGGLE, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_GUN_ID_P_FMT,
                            (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            pauseMenuPrint(display, 0, 47, PM_STICK_MODE, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_AnalogMode:
          {
            const char *modeStr = pmAnalogModeStr((uint8_t)OF_Prefs::settings[OF_Const::analogMode]);
            const unsigned deadzone = (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]);
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_STICK_FMT, modeStr);
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_GUN_ID_P_FMT,
                            (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_STICK_FMT, modeStr);
            pauseMenuPrintf(display, 0, 47, WHITE, BLACK, PM_DEADZONE_FMT, deadzone);
            #endif
            break;
          }
          case ScreenPause_AnalogKeysLayout:
          {
            const char *layoutStr = (OF_Prefs::settings[OF_Const::analogKeysLayout] == OF_Const::analogKeysLayoutWASD) ? "WASD" : "Arrows";
            const char *modeStr = pmAnalogModeStrAlt((uint8_t)OF_Prefs::settings[OF_Const::analogMode]);
            const unsigned deadzone = (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]);
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_KEYS_FMT, layoutStr);
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_STICK_FMT, modeStr);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_KEYS_FMT, layoutStr);
            pauseMenuPrintf(display, 0, 47, WHITE, BLACK, PM_DEADZONE_FMT, deadzone);
            #endif
            break;
          }
          case ScreenPause_AnalogDeadzone:
          {
            const char *modeStr = pmAnalogModeStr((uint8_t)OF_Prefs::settings[OF_Const::analogMode]);
            const unsigned deadzone = (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]);
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_DEADZONE_FMT, deadzone);
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_STICK_FMT, modeStr);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_DEADZONE_FMT, deadzone);
            pauseMenuPrint(display, 0, 47, PM_AXIS_MODE, WHITE, BLACK);
            #endif
            break;
          }
          case ScreenPause_AxisUnsignedToggle:
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_AXIS_FMT,
                            pmAxisSignStr(OF_Prefs::settings[OF_Const::axisUnsigned]));
            #else
            pauseMenuPrint(display, 0, 25, PM_CENTER_CAL, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_AXIS_FMT,
                            pmAxisSignStr(OF_Prefs::settings[OF_Const::axisUnsigned]));
            pauseMenuPrint(display, 0, 47, PM_SWAP_STICKS, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_AnalogSwapSticks:
            #ifdef OLED_091_INCH
            pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_SWAP_FMT,
                            pmOnOff(OF_Prefs::settings[OF_Const::analogSwapSticks]));
            #else
            pauseMenuPrint(display, 0, 25, PM_AXIS_MODE, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_SWAP_LONG_FMT,
                            pmOnOff(OF_Prefs::settings[OF_Const::analogSwapSticks]));
            #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              pauseMenuPrint(display, 0, 47, PM_RUMBLE_FFB, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            }
            #else
            pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            #endif
            #endif
            break;
          case ScreenPause_AnalogCenterCal:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_CENTER_CAL, BLACK, WHITE);
            #else
            pauseMenuPrint(display, 0, 25, PM_ESCAPE_KEY, WHITE, BLACK);
            pauseMenuPrint(display, 0, 36, PM_CENTER_CAL, BLACK, WHITE);
            pauseMenuPrint(display, 0, 47, PM_SAVE_SETTINGS, WHITE, BLACK);
            #endif
            break;
          #ifdef USES_RUMBLE
          case ScreenPause_RumbleFFToggle:
            #ifdef OLED_091_INCH
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_RUMBLE_FFB_FMT,
                              pmOnOff(OF_Prefs::toggles[OF_Const::rumbleFF]));
            } else {
              pauseMenuPrint(display, 0, 20, PM_RUMBLE_FFB_NA, BLACK, WHITE);
            }
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_GUN_ID_P_FMT,
                            (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_RUMBLE_FFB_FMT,
                              pmOnOff(OF_Prefs::toggles[OF_Const::rumbleFF]));
            } else {
              pauseMenuPrint(display, 0, 36, PM_RUMBLE_FFB_NA, BLACK, WHITE);
            }
            pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            #endif
            break;
          #endif
          case ScreenPause_PlayTimer:
          {
            #ifdef OLED_091_INCH
            if(PlayTimer::minutes == 0) {
              pauseMenuPrint(display, 0, 20, PM_TIMER_OFF, BLACK, WHITE);
            } else {
              pauseMenuPrintf(display, 0, 20, BLACK, WHITE, PM_TIMER_MIN_FMT,
                              (unsigned)PlayTimer::minutes);
            }
            #else
            pauseMenuPrint(display, 0, 25, PM_RUMBLE_FFB, WHITE, BLACK);
            if(PlayTimer::minutes == 0) {
              pauseMenuPrint(display, 0, 36, PM_TIMER_OFF, BLACK, WHITE);
            } else {
              pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_TIMER_MIN2_FMT,
                              (unsigned)PlayTimer::minutes);
            }
            pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            #endif
            break;
          }
          case ScreenPause_AnalogInvertX:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_INVERT_X, BLACK, WHITE);
            pauseMenuPrintf(display, 0, 31, BLACK, WHITE, PM_X_SHORT_FMT,
                            pmInvertShortStr(OF_Prefs::settings[OF_Const::analogInvertX]));
            #else
            pauseMenuPrint(display, 0, 25, PM_PLAY_TIMER, WHITE, BLACK);
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_X_AXIS_FMT,
                            pmInvertStr(OF_Prefs::settings[OF_Const::analogInvertX]));
            pauseMenuPrint(display, 0, 47, PM_INVERT_Y_AXIS, WHITE, BLACK);
            #endif
            break;
          case ScreenPause_AnalogInvertY:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_INVERT_Y, BLACK, WHITE);
            pauseMenuPrintf(display, 0, 31, BLACK, WHITE, PM_Y_SHORT_FMT,
                            pmInvertShortStr(OF_Prefs::settings[OF_Const::analogInvertY]));
            #else
            pauseMenuPrintf(display, 0, 25, WHITE, BLACK, PM_X_AXIS_FMT,
                            pmInvertStr(OF_Prefs::settings[OF_Const::analogInvertX]));
            pauseMenuPrintf(display, 0, 36, BLACK, WHITE, PM_Y_AXIS_FMT,
                            pmInvertStr(OF_Prefs::settings[OF_Const::analogInvertY]));
            pauseMenuPrint(display, 0, 47, PM_ESCAPE_KEY, WHITE, BLACK);
            #endif
            break;
//wei13402 add end                
//wei134102 add start
          case ScreenPause_ModeChange:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_MODE_CHANGE, BLACK, WHITE);
            #else
            if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_SOLENOID_TOGGLE, WHITE, BLACK);
            } else if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_RUMBLE_TOGGLE, WHITE, BLACK);
            } else if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
              pauseMenuPrint(display, 0, 25, PM_LOW_BUTTON_ON, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 25, PM_LOW_BUTTON_OFF, WHITE, BLACK);
            }
            pauseMenuPrint(display, 0, 36, PM_MODE_CHANGE, BLACK, WHITE);
            if(FW_Common::OLED.mister) {
              pauseMenuPrint(display, 0, 47, PM_CURRENT_MISTER, WHITE, BLACK);
            } else if(FW_Common::buttons.analogOutput) {
              pauseMenuPrint(display, 0, 47, PM_CURRENT_GAMEPAD, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 47, PM_CURRENT_MOUSEKB, WHITE, BLACK);
            }
            #endif
            break;
//wei134102 add end                        
          case ScreenPause_EscapeKey:
            #ifdef OLED_091_INCH
            pauseMenuPrint(display, 0, 20, PM_ESCAPE_KEY, BLACK, WHITE);
            #else
            if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_SOLENOID_TOGGLE, WHITE, BLACK);
              pauseMenuPrint(display, 0, 36, PM_ESCAPE_KEY, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
            } else if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              pauseMenuPrint(display, 0, 25, PM_RUMBLE_TOGGLE, WHITE, BLACK);
              pauseMenuPrint(display, 0, 36, PM_ESCAPE_KEY, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
            } else {
              pauseMenuPrint(display, 0, 25, PM_SAVE_SETTINGS, WHITE, BLACK);
              pauseMenuPrint(display, 0, 36, PM_ESCAPE_KEY, BLACK, WHITE);
              pauseMenuPrint(display, 0, 47, PM_CALIBRATE, WHITE, BLACK);
            }
            #endif
            break;
        }
        display->display();
    }
}

void ExtDisplay::PauseProfileUpdate(const int &selection, const char* name1, const char* name2, const char* name3, const char* name4)
{
    if(display != nullptr) {
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        display->drawBitmap(60, 18, upArrowGlyph, ARROW_WIDTH, ARROW_HEIGHT, WHITE);
        #ifndef OLED_091_INCH  // 0.91寸屏幕不显示底部箭头
        display->drawBitmap(60, 59, downArrowGlyph, ARROW_WIDTH, ARROW_HEIGHT, WHITE);
        #endif
        #ifdef OLED_091_INCH
        switch(selection) {
          case 0:
            pauseMenuPrint(display, 4, 20, name1, BLACK, WHITE);
            break;
          case 1:
            pauseMenuPrint(display, 4, 20, name2, BLACK, WHITE);
            break;
          case 2:
            pauseMenuPrint(display, 4, 20, name3, BLACK, WHITE);
            break;
          case 3:
            pauseMenuPrint(display, 4, 20, name4, BLACK, WHITE);
            break;
        }
        #else
        switch(selection) {
          case 0:
            pauseMenuPrint(display, 4, 25, name4, WHITE, BLACK);
            pauseMenuPrint(display, 4, 36, name1, BLACK, WHITE);
            pauseMenuPrint(display, 4, 47, name2, WHITE, BLACK);
            break;
          case 1:
            pauseMenuPrint(display, 4, 25, name1, WHITE, BLACK);
            pauseMenuPrint(display, 4, 36, name2, BLACK, WHITE);
            pauseMenuPrint(display, 4, 47, name3, WHITE, BLACK);
            break;
          case 2:
            pauseMenuPrint(display, 4, 25, name2, WHITE, BLACK);
            pauseMenuPrint(display, 4, 36, name3, BLACK, WHITE);
            pauseMenuPrint(display, 4, 47, name4, WHITE, BLACK);
            break;
          case 3:
            pauseMenuPrint(display, 4, 25, name3, WHITE, BLACK);
            pauseMenuPrint(display, 4, 36, name4, BLACK, WHITE);
            pauseMenuPrint(display, 4, 47, name1, WHITE, BLACK);
            break;
        }
        #endif
        display->display();
    }
}

void ExtDisplay::SaveScreen(const int &status)
{
    if(display != nullptr) {
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        pauseMenuPrint(display, 24, 24, TXT_SAVING, WHITE, BLACK);
        display->display();
    }
}

void ExtDisplay::PrintAmmo(const uint &ammo)
{
    if(display != nullptr) {
        currentAmmo = ammo;

        ammoEmpty = ammo ? false : true;

        #ifdef OLED_091_INCH
        // 0.91寸屏幕：弹药在下面区域显示（与血量同区），子弹图标 + 数量
        char buf[8];
        snprintf(buf, sizeof(buf), "%u", ammo);
        const int iconY = LIFE_DISPLAY_AREA_Y + (LIFE_DISPLAY_HEIGHT - AMMO_BULLET_ICON_HEIGHT) / 2;

        if(screenState == Screen_Mamehook_Single) {
            display->fillRect(0, LIFE_DISPLAY_AREA_Y, SCREEN_WIDTH, LIFE_DISPLAY_HEIGHT, BLACK);
            display->drawBitmap(2, iconY, ammoBulletIcon, AMMO_BULLET_ICON_WIDTH, AMMO_BULLET_ICON_HEIGHT, WHITE);
            display->setTextSize(COMPACT_FONT_SIZE);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(2 + AMMO_BULLET_ICON_WIDTH + 2, LIFE_DISPLAY_AREA_Y + 3);
            display->print(buf);
        } else if(screenState == Screen_Mamehook_Dual) {
            display->fillRect(0, LIFE_DISPLAY_AREA_Y, SCREEN_WIDTH, LIFE_DISPLAY_HEIGHT, BLACK);
            display->drawBitmap(AMMO_DUAL_POS_X - AMMO_BULLET_ICON_WIDTH - 4, iconY, ammoBulletIcon, AMMO_BULLET_ICON_WIDTH, AMMO_BULLET_ICON_HEIGHT, WHITE);
            display->setTextSize(COMPACT_FONT_SIZE);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(AMMO_DUAL_POS_X, LIFE_DISPLAY_AREA_Y + 3);
            display->print(buf);
        }
        #else
        // 0.96寸屏幕：保留原来的大号数字弹药显示
        uint ammoLeft = ammo / 10;
        uint ammoRight = ammo - (ammoLeft * 10);

        if(screenState == Screen_Mamehook_Single) {
            display->fillRect(40, 22, (NUMBER_GLYPH_WIDTH*2)+6, NUMBER_GLYPH_HEIGHT, BLACK);
            display->drawBitmap(40,                      22, numbers[ammoLeft],  NUMBER_GLYPH_WIDTH, NUMBER_GLYPH_HEIGHT, WHITE);
            display->drawBitmap(40+6+NUMBER_GLYPH_WIDTH, 22, numbers[ammoRight], NUMBER_GLYPH_WIDTH, NUMBER_GLYPH_HEIGHT, WHITE);
        } else if(screenState == Screen_Mamehook_Dual) {
            display->fillRect(72, 22, (NUMBER_GLYPH_WIDTH*2)+6, NUMBER_GLYPH_HEIGHT, BLACK);
            display->drawBitmap(72,                      22, numbers[ammoLeft],  NUMBER_GLYPH_WIDTH, NUMBER_GLYPH_HEIGHT, WHITE);
            display->drawBitmap(72+6+NUMBER_GLYPH_WIDTH, 22, numbers[ammoRight], NUMBER_GLYPH_WIDTH, NUMBER_GLYPH_HEIGHT, WHITE);
        }
        #endif

        display->display();
    }
}

void ExtDisplay::PrintLife(const uint &life)
{
    if(display != nullptr) {
        currentLife = life;
        lifeEmpty = life ? false : true;
        if(screenState == Screen_Mamehook_Single) {
            if(lifeBar) {
                #ifdef OLED_091_INCH
                // 0.91寸屏幕：简化生命值显示
                display->fillRect(14, 20, 100, 6, BLACK);
                display->fillRect(14, 20, life, 6, WHITE);
                if(life) {
                  display->setTextSize(1);
                  display->setCursor(52, 20);
                  display->setTextColor(WHITE, BLACK);
                  display->print(life);
                  display->println("%");
                }
                #else
                display->fillRect(14, 37, 100, 9, BLACK);
                display->fillRect(52, 51, 30, 8, BLACK);
                display->fillRect(14, 37, life, 9, WHITE);

                if(life) {
                  display->setTextSize(1);
                  display->setCursor(52, 51);
                  display->setTextColor(WHITE, BLACK);
                  display->print(life);
                  display->println(" %");
                }
                #endif
                display->display();
            } else {
                #ifdef OLED_091_INCH
                // 0.91寸屏幕：一颗心形 + 数量，如 ♥: 3
                const int lifeAreaY = LIFE_DISPLAY_AREA_Y + (LIFE_DISPLAY_HEIGHT - HEART_SMALL_HEIGHT) / 2;
                display->fillRect(0, LIFE_DISPLAY_AREA_Y, SCREEN_WIDTH, LIFE_DISPLAY_HEIGHT, BLACK);
                display->drawBitmap(2, lifeAreaY, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                display->setTextSize(COMPACT_FONT_SIZE);
                display->setTextColor(WHITE, BLACK);
                display->setCursor(2 + HEART_SMALL_WIDTH + 2, LIFE_DISPLAY_AREA_Y + 3);
                display->print(": ");
                display->print((int)life);
                #else
                display->fillRect(22, 19, HEART_LARGE_WIDTH*5+4, HEART_LARGE_HEIGHT+22+HEART_LARGE_HEIGHT, BLACK);
                switch(life) {
                  case 9:
                    display->drawBitmap(22, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);

                    display->drawBitmap(30, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+1+HEART_LARGE_WIDTH, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+2+HEART_LARGE_WIDTH*2, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+3+HEART_LARGE_WIDTH*3, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 8:
                    display->drawBitmap(22, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);

                    display->drawBitmap(39, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(39+1+HEART_LARGE_WIDTH, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(39+2+HEART_LARGE_WIDTH*2, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 7:
                    display->drawBitmap(22, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);

                    display->drawBitmap(48, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(48+1+HEART_LARGE_WIDTH, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 6:
                    display->drawBitmap(22, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);

                    display->drawBitmap(56, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 5:
                    display->drawBitmap(22, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+HEART_LARGE_WIDTH, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+HEART_LARGE_WIDTH*2, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+HEART_LARGE_WIDTH*3, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+HEART_LARGE_WIDTH*4, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 4:
                    display->drawBitmap(30, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+1+HEART_LARGE_WIDTH, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+2+HEART_LARGE_WIDTH*2, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(30+3+HEART_LARGE_WIDTH*3, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 3:
                    display->drawBitmap(39, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(39+1+HEART_LARGE_WIDTH, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(39+2+HEART_LARGE_WIDTH*2, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 2:
                    display->drawBitmap(48, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(48+1+HEART_LARGE_WIDTH, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 1:
                    display->drawBitmap(56, 30, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                  case 0:
                    break;
                  default: // 10+
                    display->drawBitmap(22, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 19, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);

                    display->drawBitmap(22, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+1+HEART_LARGE_WIDTH, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+2+HEART_LARGE_WIDTH*2, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+3+HEART_LARGE_WIDTH*3, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    display->drawBitmap(22+4+HEART_LARGE_WIDTH*4, 41, lifeIcoLarge, HEART_LARGE_WIDTH, HEART_LARGE_HEIGHT, WHITE);
                    break;
                }
                #endif // OLED_091_INCH
                display->display();
            }
        } else if(screenState == Screen_Mamehook_Dual) {
            #ifndef OLED_091_INCH  // 0.91寸屏幕不支持双屏模式
            if(lifeBar) {
                display->fillRect(4, 39, 55, 5, BLACK);
                display->fillRect(20, 51, 30, 8, BLACK);
                display->fillRect(4, 39, map(life, 0, 100, 0, 55), 5, WHITE);

                if(life) {
                  display->setTextSize(1);
                  display->setCursor(20, 51);
                  display->setTextColor(WHITE, BLACK);
                  display->print(life);
                  display->println(" %");
                }

                display->display();
            } else {
                display->fillRect(1, 22, HEART_SMALL_WIDTH*5, HEART_SMALL_HEIGHT+20+HEART_SMALL_HEIGHT, BLACK);
                switch(life) {
                  case 9:
                    display->drawBitmap(1, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);

                    display->drawBitmap(7, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH*2, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH*3, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 8:
                    display->drawBitmap(1, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);

                    display->drawBitmap(13, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(13+HEART_SMALL_WIDTH, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(13+HEART_SMALL_WIDTH*2, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 7:
                    display->drawBitmap(1, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);

                    display->drawBitmap(19, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(19+HEART_SMALL_WIDTH, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 6:
                    display->drawBitmap(1, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);

                    display->drawBitmap(25, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 5:
                    display->drawBitmap(1, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 4:
                    display->drawBitmap(7, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH*2, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(7+HEART_SMALL_WIDTH*3, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 3:
                    display->drawBitmap(13, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(13+HEART_SMALL_WIDTH, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(13+HEART_SMALL_WIDTH*2, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 2:
                    display->drawBitmap(19, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(19+HEART_SMALL_WIDTH, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 1:
                    display->drawBitmap(25, 32, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                  case 0:
                    break;
                  default: // 10+
                    display->drawBitmap(1, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 22, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);

                    display->drawBitmap(1, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*2, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*3, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    display->drawBitmap(1+HEART_SMALL_WIDTH*4, 42, lifeIcoSmall, HEART_SMALL_WIDTH, HEART_SMALL_HEIGHT, WHITE);
                    break;
                }
                display->display();
            }
            #endif // OLED_091_INCH
        }
    }
}
