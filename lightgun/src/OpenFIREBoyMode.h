/*!
 * @file OpenFIREBoyMode.h
 * @brief Boy Mode: allow trigger->solenoid play without wireless/USB link.
 *
 * Wireless boot behavior:
 * - Offer a 3s window where pressing TRIGGER enables Boy Mode.
 * - When enabled, firmware continues boot even if no wireless link is established.
 */
 
#ifndef _OPENFIREBOYMODE_H_
#define _OPENFIREBOYMODE_H_

#include <Arduino.h>
#include "OpenFIREcommon.h"
#include "OpenFIREFeedback.h"

class BoyMode
{
public:
    static inline bool enabled = false;

    static inline void BeginDecisionWindow(uint32_t windowMs = 3000)
    {
        if (enabled)
            return;
        decisionActive = true;
        decisionWindowMs = windowMs;
        decisionStartMs = millis();
        lastShownSeconds = 0xFFFFFFFFu;
        triggerPinInited = false;
        triggerPrev = HIGH;
        DrawCountdown();
    }

    static inline void TickDecisionWindow()
    {
        if (!decisionActive || enabled)
            return;

        const unsigned long now = millis();
        const uint32_t elapsed = (uint32_t)(now - decisionStartMs);
        if (elapsed >= decisionWindowMs)
        {
            decisionActive = false;
            return;
        }

        // 检测 ESP32 开发板的 BOOT 按钮（GPIO0）
        const int bootButtonPin = 0;
        if (!triggerPinInited)
        {
            pinMode(bootButtonPin, INPUT_PULLUP);
            triggerPinInited = true;
        }

        bool bootButtonState = digitalRead(bootButtonPin);

        // 只要检测到 BOOT 按钮按下（LOW）就启用 BOY 模式
        if (bootButtonState == LOW)
        {
            Enable();
            // 启用后立刻返回，外层 while 条件会退出，不再继续倒计时
            return;
        }

        // 只在秒数变化时才更新显示，减少 OLED 操作
        DrawCountdown();
    }

    static inline void Enable()
    {
        enabled = true;
        decisionActive = false;

        // 先设置核心变量，让外层循环能立即退出
        // 其他操作在循环退出后再处理，避免阻塞
    }

    static inline bool IsEnabled()
    {
        return enabled;
    }

private:
    static inline bool decisionActive = false;
    static inline uint32_t decisionWindowMs = 3000;
    static inline unsigned long decisionStartMs = 0;
    static inline uint32_t lastShownSeconds = 0xFFFFFFFFu;
    static inline bool triggerPinInited = false;
    static inline int triggerPrev = HIGH;

    static inline void DrawCountdown()
    {
        const unsigned long now = millis();
        const uint32_t elapsed = (uint32_t)(now - decisionStartMs);
        if (elapsed >= decisionWindowMs)
            return;

        // Round up so it shows 3,2,1.
        const uint32_t remainingMs = decisionWindowMs - elapsed;
        const uint32_t secs = (remainingMs + 999UL) / 1000UL;
        if (secs == lastShownSeconds)
            return;
        lastShownSeconds = secs;

        #ifdef USES_DISPLAY
        char buf[24];
        snprintf(buf, sizeof(buf), "Boy? BOOT %lus", (unsigned long)secs);
        FW_Common::OLED.TopPanelUpdate(buf);
        #endif
        
        // 打印调试信息到串口
        Serial.printf("BoyMode countdown: %lu\n", secs);
    }
};

#endif // _OPENFIREBOYMODE_H_

