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
#ifdef USES_DISPLAY
#include "OpenFIREdisplay_i18n.h"
#endif

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

        // 兼容两种进入方式：
        // 1) 板载 BOOT 按键（GPIO0）
        // 2) 当前映射的 Trigger 按键（与注释/用户习惯一致）
        const int bootButtonPin = 0;
        const int8_t triggerPin = OF_Prefs::pins[OF_Const::btnTrigger];
        if (!triggerPinInited)
        {
            pinMode(bootButtonPin, INPUT_PULLUP);
            if (triggerPin >= 0)
                pinMode(triggerPin, INPUT_PULLUP);
            triggerPinInited = true;
        }

        const bool bootPressed = (digitalRead(bootButtonPin) == LOW);
        const bool triggerPressed = (triggerPin >= 0) && (digitalRead(triggerPin) == LOW);

        // 只要检测到 BOOT 或 Trigger 任一按下就启用 BOY 模式
        if (bootPressed || triggerPressed)
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
        snprintf(buf, sizeof(buf), TXT_BOY_MODE_FMT, (unsigned long)secs);
        FW_Common::OLED.TopPanelUpdate(buf);
        #endif
        
        // 避免在早期启动阶段（USB CDC 未就绪）阻塞在串口输出
        // 如需调试可在串口就绪后再打印
    }
};

#endif // _OPENFIREBOYMODE_H_

