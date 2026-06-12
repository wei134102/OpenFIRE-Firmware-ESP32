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

#ifdef USE_LOVYAN_GFX
  // nulla 
#else
  //#include <Adafruit_GFX.h>
#endif

#include <Wire.h>
#include <TinyUSB_Devices.h>

#include "OpenFIREdisplay.h"
#include "OpenFIREprefs.h"
#include "OpenFIREFeedback.h"
#include "OpenFIREDefines.h"
#include "OpenFIREcommon.h" //wei134102 add
#include "OpenFIREPlayTimer.h"
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
        display->setCursor(2, 2);
        display->setTextSize(1);
        display->setTextColor(WHITE, BLACK);

        // 简化 Profile 前缀为 "Pro" 并在末尾附加当前枪ID P1-P4
        bool isProfileBar = (textPrefix != nullptr && strcmp(textPrefix, "Prof: ") == 0);

        if(isProfileBar) {
            // 固定使用简写前缀
            display->print("Pro: ");
        } else if(textPrefix != nullptr) {
            display->print(textPrefix);
        }

        if(profText != nullptr) {
            const char* namePtr = profText;
            // 对 Pro 状态栏，去掉前缀 "Profile "，避免太长
            if(isProfileBar && strncmp(profText, "Profile ", 8) == 0) {
                namePtr = profText + 8;
            }
            display->print(namePtr);

            if(isProfileBar) {
                // 从全局设置中读取当前 GunId（0=P1,1=P2,2=P3,3=P4）
                uint8_t gunIndex = (uint8_t)(OF_Prefs::settings[OF_Const::gunId] % 4);
                display->print("  P");
                display->println((int)(gunIndex + 1));
            } else {
                display->println();
            }
        }

        display->display();

        // 顶部栏其它内容可能会频繁刷新（例如红外检测/温度提示），
        // 这里强制在最后重绘右侧倒计时/占位符，避免被覆盖。
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

    display->setCursor(x + 1, 2);
    display->setTextSize(1);
    display->setTextColor(WHITE, BLACK);

    char buf[5];
    // 根据 PlayTimer 状态区分 "OFF"（未配置计时）和 "0"（计时结束）
    bool timerExpired = PlayTimer::IsExpired();
    bool timerOff = (PlayTimer::minutes == 0) && !timerExpired;

    if (seconds > 0) {
        if (seconds > 999) seconds = 999;
        snprintf(buf, sizeof(buf), "%3u", (unsigned)seconds);
    } else if (timerOff) {
        snprintf(buf, sizeof(buf), "OFF");
    } else if (timerExpired) {
        snprintf(buf, sizeof(buf), "  0");
    } else {
        // 理论上不会到这里，兜底显示 OFF
        snprintf(buf, sizeof(buf), "OFF");
    }
    display->print(buf);

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
            // 0.91寸屏幕：简化显示
            display->setTextSize(1);
            display->setCursor(20, 18);
            display->println("Welcome!");
            display->setCursor(12, 24);
            display->println("Pull trigger");
            #else
            display->setTextSize(2);
            display->setCursor(20, 18);
            display->println("Welcome!");
            display->setTextSize(1);
            display->setCursor(12, 40);
            display->println(" Pull trigger to");
            display->setCursor(12, 52);
            display->println("start calibration!");
            #endif
            break;
          case Screen_IRTest:
            TopPanelUpdate("", "IR Test");
            break;
          case Screen_Saving:
            TopPanelUpdate("", "Saving Profiles");
            display->setTextSize(2);
            display->setCursor(16, 18);
            display->println("Saving...");
            break;
          case Screen_SaveSuccess:
            display->setTextSize(2);
            display->setCursor(30, 18);
            display->println("Save");
            display->setCursor(4, 40);
            display->println("successful");
            break;
          case Screen_SaveError:
            display->setTextSize(2);
            display->setCursor(30, 18);
            display->setTextColor(BLACK, WHITE);
            display->println("Save");
            display->setCursor(22, 40);
            display->println("failed");
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
          // P?(GunID): <ProfileName> <Diam/Squa> <Temp> + 右侧倒计时
          if(millis() - idleTimeStamp > OLED_IDLEUPD_INTERVAL) {
              idleTimeStamp = millis();

              // Gun ID
              uint8_t gunIndex = (uint8_t)(OF_Prefs::settings[OF_Const::gunId] % 4);
              char prefix[8];
              snprintf(prefix, sizeof(prefix), "P%u: ", (unsigned)(gunIndex + 1));

              // Profile + layout + temp
              char line[32];

              // 简写配置文件名：P_A / P_B / P_C / P_D（按槽位）
              const uint8_t profIdx = (uint8_t)(OF_Prefs::currentProfile % PROFILE_COUNT);
              char profShort[5] = { 'P', '_', 'A', '\0', '\0' };
              profShort[2] = (char)('A' + (profIdx % 4));
              const char* layoutStr = (OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond) ? "Diam" : "Squa";

              #ifdef USES_TEMP
              char tempBuf[6];
              int temp = OF_FFB::temperatureCurrent;
              if (temp == (int)OF_Const::TEMPERATURE_SENSOR_ERROR_VALUE) {
                  snprintf(tempBuf, sizeof(tempBuf), "Err");
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
        TopPanelUpdate("Temp sensor: ", "Fault!");
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

    TopPanelUpdate("Current Temp: ", tempString);
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
        TopPanelUpdate("Using ", namesList[currentProf]); // names are placeholder
        display->fillRect(0, 16, 128, SCREEN_CONTENT_HEIGHT, BLACK);
        display->setTextSize(1);
        display->setCursor(0, 17);
        display->print(" A > ");
        display->println(name1);
        display->setCursor(0, 17+11);
        display->print(" B > ");
        display->println(name2);
        display->setCursor(0, 17+(11*2));
        display->print("Str> ");
        display->println(name3);
        display->setCursor(0, 17+(11*3));
        display->print("Sel> ");
        display->println(name4);
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
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Range Calibrate ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Save Gun Settings ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Range Calibrate ");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Calibrate ");
            #endif
            break;
          case ScreenPause_Calibrate:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Calibrate ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Range Calibrate ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Calibrate ");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Profile Select ");
            #endif
            break;
          case ScreenPause_ProfileSelect:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Profile Select ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Calibrate ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Profile Select ");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Save Gun Settings ");
            #endif
            break;
          case ScreenPause_Save:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Save Gun Settings ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Center Calibrate ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Save Gun Settings ");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Range Calibrate ");
            #endif
            break;
          case ScreenPause_Rumble:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Rumble Toggle ");
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Solenoid Toggle ");
            } else {
              display->println(" Send Escape Keypress");
            }
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Save Gun Settings ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Rumble Toggle ");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
                display->println(" Solenoid Toggle ");
              } else {
                display->println(" Send Escape Keypress");
              }
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Solenoid Toggle ");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Send Escape Keypress");
            } else {
              display->println(" Send Escape Keypress");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Calibrate ");
            }
            #endif
            break;
          case ScreenPause_Solenoid:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Solenoid Toggle ");
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Solenoid Toggle ");
            } else {
              display->println(" Mode Change");
            }
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Rumble Toggle ");
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
                display->println(" Solenoid Toggle ");
                display->setTextColor(WHITE, BLACK);
                display->setCursor(0, 47);
                display->println(" Mode Change"); //wei134102 changed from "Send Escape Keypress"
              } else {
                display->println(" Mode Change"); //wei134102 changed from "Send Escape Keypress "
                display->setTextColor(WHITE, BLACK);
                display->setCursor(0, 47);
                display->println("Calibrate");
              }
            } else if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Save Gun Settings");
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->println(" Solenoid Toggle ");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Mode Change");//wei134102 changed from " Send Escape Keypress "
            } else {
              display->println(" Mode Change");//wei134102 changed from " Send Escape Keypress "
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->println(" Calibrate ");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Profile Select ");
            }
            #endif
            break;
          case ScreenPause_AutofireToggle:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Autofire Toggle ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Autofire Toggle ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Autofire: %s ", OF_Prefs::toggles[OF_Const::autofire] ? "ON" : "OFF");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Send Escape Keypress ");
            #endif
            break;            
//wei13402 add start
          case ScreenPause_LowButtonToggle:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Low Button Toggle ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Low Button Toggle ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
              display->println(" Low Button: ON ");
            } else {
              display->println(" Low Button: OFF ");
            }
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Layout Toggle ");
            #endif
            break;
          case ScreenPause_LayoutToggle:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Layout: %s ", OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond ? "Diamond" : "Square");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Low Button: " + String(OF_Prefs::toggles[OF_Const::lowButtonsMode] ? "ON" : "OFF"));
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Layout: %s ", OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond ? "Diamond" : "Square");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Gun ID (P1-P4) ");
            #endif
            break;
          case ScreenPause_GunId:
            #ifdef OLED_091_INCH
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 20);
            display->println(" Layout Toggle ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 31);
            display->printf(" Gun ID: P%d ", (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Layout Toggle ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Gun ID: P%d ", (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Stick Output Mode ");
            #endif
            break;
          case ScreenPause_AnalogMode:
          {
            // 当前模拟输出模式字符串
            const char* modeStr;
            switch (OF_Prefs::settings[OF_Const::analogMode]) {
              default:
              case OF_Const::analogModeStick: modeStr = "Gamepad Stick"; break;
              case OF_Const::analogModeDpad: modeStr = "Gamepad D-Pad"; break;
              case OF_Const::analogModeKeys: modeStr = "Keyboard Arrows"; break;
            }
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Stick: %s ", modeStr);
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->printf(" Gun ID: P%d ", (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Stick: %s ", modeStr);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->printf(" Deadzone: %u%% ",
                            (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]));
            #endif
            break;
          }
          case ScreenPause_AnalogKeysLayout:
          {
            const char* layoutStr = (OF_Prefs::settings[OF_Const::analogKeysLayout] == OF_Const::analogKeysLayoutWASD) ? "WASD" : "Arrows";
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Stick Keys: %s ", layoutStr);
            #else
            {
              const char* modeStr;
              switch (OF_Prefs::settings[OF_Const::analogMode]) {
                default:
                case OF_Const::analogModeStick: modeStr = "Gamepad Stick"; break;
                case OF_Const::analogModeDpad:  modeStr = "Gamepad D-Pad"; break;
                case OF_Const::analogModeKeys:  modeStr = "Keyboard Keys"; break;
              }
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 25);
              display->printf(" Stick: %s ", modeStr);
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->printf(" Keys: %s ", layoutStr);
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->printf(" Deadzone: %u%% ",
                              (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]));
            }
            #endif
            break;
          }
          case ScreenPause_AnalogDeadzone:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Deadzone: %u%% ", (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]));
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            // 上一项为 Stick 输出模式
            {
              const char* modeStr;
              switch (OF_Prefs::settings[OF_Const::analogMode]) {
                default:
                case OF_Const::analogModeStick: modeStr = "Gamepad Stick"; break;
                case OF_Const::analogModeDpad: modeStr = "Gamepad D-Pad"; break;
                case OF_Const::analogModeKeys: modeStr = "Keyboard Arrows"; break;
              }
              display->printf(" Stick: %s ", modeStr);
            }
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Deadzone: %u%% ",
                            (unsigned)(OF_Prefs::settings[OF_Const::analogDeadzone] > 30 ? 30 : OF_Prefs::settings[OF_Const::analogDeadzone]));
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Axis Mode ");
            #endif
            break;
          case ScreenPause_AxisUnsignedToggle:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Axis: %s ", OF_Prefs::settings[OF_Const::axisUnsigned] ? "Unsigned" : "Signed");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->printf(" Center Calibrate ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Axis: %s ", OF_Prefs::settings[OF_Const::axisUnsigned] ? "Unsigned" : "Signed");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Swap Sticks ");
            #endif
            break;
          case ScreenPause_AnalogSwapSticks:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->printf(" Swap: %s ", OF_Prefs::settings[OF_Const::analogSwapSticks] ? "ON" : "OFF");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->printf(" Axis Mode ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Swap Sticks: %s ", OF_Prefs::settings[OF_Const::analogSwapSticks] ? "ON" : "OFF");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              display->println(" Rumble FFB Toggle ");
            } else {
              display->println(" Send Escape Keypress ");
            }
            #else
            display->println(" Send Escape Keypress ");
            #endif
            #endif
            break;
          case ScreenPause_AnalogCenterCal:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Center Calibrate ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Send Escape Keypress");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Center Calibrate ");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Save Gun Settings ");
            #endif
            break;
          #ifdef USES_RUMBLE
          case ScreenPause_RumbleFFToggle:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              display->printf(" Rumble FFB: %s ", OF_Prefs::toggles[OF_Const::rumbleFF] ? "ON" : "OFF");
            } else {
              display->println(" Rumble FFB: N/A ");
            }
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->printf(" Gun ID: P%d ", (int)(OF_Prefs::settings[OF_Const::gunId] % 4) + 1);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            if(OF_Prefs::pins[OF_Const::rumblePin] >= 0) {
              display->printf(" Rumble FFB: %s ", OF_Prefs::toggles[OF_Const::rumbleFF] ? "ON" : "OFF");
            } else {
              display->println(" Rumble FFB: N/A ");
            }
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);                       
            display->println(" Send Escape Keypress ");
            #endif
            break;
          #endif            
          case ScreenPause_PlayTimer:
          {
            // 显示当前计时器设置：0=OFF，其它显示分钟数
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if (PlayTimer::minutes == 0) {
              display->println(" Play Timer: OFF ");
            } else {
              char buf[20];
              snprintf(buf, sizeof(buf), " Timer: %2u min ", (unsigned)PlayTimer::minutes);
              display->println(buf);
            }
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Rumble FFB Toggle ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            if (PlayTimer::minutes == 0) {
              display->println(" Play Timer: OFF ");
            } else {
              char buf[22];
              snprintf(buf, sizeof(buf), " Play Timer: %2u min ", (unsigned)PlayTimer::minutes);
              display->println(buf);
            }
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Send Escape Keypress ");
            #endif
            break;
          }
          case ScreenPause_AnalogInvertX:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Analog Invert X ");
            display->setCursor(0, 31);
            display->printf(" X: %s ", OF_Prefs::settings[OF_Const::analogInvertX] ? "INV" : "NORM");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->println(" Play Timer ");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" X Axis: %s ", OF_Prefs::settings[OF_Const::analogInvertX] ? "Inverted" : "Normal");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Invert Y Axis ");
            #endif
            break;
          case ScreenPause_AnalogInvertY:
            #ifdef OLED_091_INCH
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Analog Invert Y ");
            display->setCursor(0, 31);
            display->printf(" Y: %s ", OF_Prefs::settings[OF_Const::analogInvertY] ? "INV" : "NORM");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            display->printf(" X Axis: %s ", OF_Prefs::settings[OF_Const::analogInvertX] ? "Inverted" : "Normal");
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->printf(" Y Axis: %s ", OF_Prefs::settings[OF_Const::analogInvertY] ? "Inverted" : "Normal");
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            display->println(" Send Escape Keypress ");
            #endif
            break;
//wei13402 add end                
//wei134102 add start
          case ScreenPause_ModeChange:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：显示当前选中的项目和当前模式
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Mode Change ");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Solenoid Toggle ");
            } else if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Rumble Toggle ");
            } else if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
              display->println(" Low Button: ON ");              
            } else {
              display->println(" Low Button: OFF ");
            }
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 36);
            display->println(" Mode Change ");
            // Display current mode
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 47);
            if(FW_Common::OLED.mister) {
              display->println(" Current: MiSTer ");
            } else if(FW_Common::buttons.analogOutput) {
              display->println(" Current: Gamepad ");
            } else {
              display->println(" Current: Mouse/KB ");
            }
            #endif  // 这个 endif 对应第743行的 ifdef OLED_091_INCH
            break;
//wei134102 add end                        
          case ScreenPause_EscapeKey:
            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Send Escape Keypress");
            #else
            display->setTextColor(WHITE, BLACK);
            display->setCursor(0, 25);
            if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0 && OF_Prefs::pins[OF_Const::solenoidSwitch] == -1) {
              display->println(" Solenoid Toggle ");
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->println(" Send Escape Keypress");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Calibrate ");
            } else if(OF_Prefs::pins[OF_Const::rumblePin] >= 0 && OF_Prefs::pins[OF_Const::rumbleSwitch] == -1) {
              display->println(" Rumble Toggle ");
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->println(" Send Escape Keypress");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Calibrate ");
            } else {
              display->println(" Save Gun Settings ");
              display->setTextColor(BLACK, WHITE);
              display->setCursor(0, 36);
              display->println(" Send Escape Keypress");
              display->setTextColor(WHITE, BLACK);
              display->setCursor(0, 47);
              display->println(" Calibrate ");
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
        display->setTextSize(1);
        #ifdef OLED_091_INCH  // 0.91寸屏幕优化
        // 为0.91寸屏幕简化显示，只显示当前选中的配置文件
        display->setTextColor(BLACK, WHITE);
        display->setCursor(4, 20);
        switch(selection) {
          case 0:
            display->println(name1);
            break;
          case 1:
            display->println(name2);
            break;
          case 2:
            display->println(name3);
            break;
          case 3:
            display->println(name4);
            break;
        }
        #else
        switch(selection) {
          case 0: // Profile #0, etc.
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 25);
            display->println(name4);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(4, 36);
            display->println(name1);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 47);
            display->println(name2);
            break;
          case 1:
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 25);
            display->println(name1);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(4, 36);
            display->println(name2);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 47);
            display->println(name3);
            break;
          case 2:
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 25);
            display->println(name2);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(4, 36);
            display->println(name3);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 47);
            display->println(name4);
            break;
          case 3:
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 25);
            display->println(name3);
            display->setTextColor(BLACK, WHITE);
            display->setCursor(4, 36);
            display->println(name4);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(4, 47);
            display->println(name1);
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
        display->setTextColor(WHITE, BLACK);
        display->setTextSize(2);
        display->setCursor(24, 24);
        display->println("Saving...");
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
