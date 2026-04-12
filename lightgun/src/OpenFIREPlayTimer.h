/*!
 * @file OpenFIREPlayTimer.h
 * @brief Simple play-time limiter used from pause menu.
 */

#ifndef _OPENFIREPLAYTIMER_H_
#define _OPENFIREPLAYTIMER_H_

#include <Arduino.h>
#include "OpenFIREcommon.h"

/// @brief Play-time limiter that can be configured from the pause menu.
/// @details
/// - minutes: 0 / 5 / 10 / 15 / 20
/// - When time is over, all buttons are released and further HID reports are disabled,
///   but USB / wireless links remain connected.
class PlayTimer
{
public:
    /// @brief Current timer length in minutes (0=disabled).
    static inline uint8_t minutes = 0;

    /// @brief Whether the timer is currently running.
    static inline bool enabled = false;

    /// @brief Whether the current session has already expired.
    static inline bool expired = false;

    /// @brief Start timestamp (millis) of current session.
    static inline unsigned long startMillis = 0;

    /// @brief Last computed remaining seconds for HUD display.
    static inline uint32_t lastRemainingSeconds = 0;

    /// @brief Set new timer length in minutes (0/5/10/15/20).
    static inline void SetMinutes(uint8_t m)
    {
        minutes = m;
        // NOTE: Persistence is intentionally not used here to keep upstream diffs minimal.
    }

    /// @brief Reset state and, if minutes > 0, start counting from now.
    /// @details Typically called when leaving pause menu back to GunMode_Run.
    static inline void ResetAndStart()
    {
        expired = false;

        if (minutes == 0)
        {
            enabled = false;
            lastRemainingSeconds = 0;
            // Ensure inputs are live again if timer was previously expired.
            FW_Common::buttons.ReportEnable();
            return;
        }

        enabled = true;
        startMillis = millis();
        lastRemainingSeconds = (uint32_t)minutes * 60UL;

        // When starting a new session, always allow reports until we actually expire.
        FW_Common::buttons.ReportEnable();
    }

    /// @brief Per-frame update, to be called from GunMode_Run loop.
    static inline void Tick()
    {
        if (!enabled || expired || minutes == 0)
            return;

        const uint32_t limitMs = (uint32_t)minutes * 60000UL;
        const unsigned long now = millis();
        const uint32_t elapsed = (uint32_t)(now - startMillis);

        if (elapsed >= limitMs)
        {
            expired = true;
            enabled = false;
            lastRemainingSeconds = 0;

            // Release any currently-held buttons so host sees a clean "all up" state,
            // then prevent any further HID reports from being sent.
            FW_Common::buttons.ReleaseAll();
            FW_Common::buttons.ReportDisable();

            #ifdef USES_DISPLAY
            FW_Common::OLED.TopPanelUpdate("Play time over");
            #endif
        }
        else
        {
            uint32_t remainingMs = limitMs - elapsed;
            uint32_t secs = remainingMs / 1000UL;
            if (secs != lastRemainingSeconds)
                lastRemainingSeconds = secs;
        }
    }

    static inline bool IsExpired()
    {
        return expired;
    }

    /// @brief Whether timer is actively counting down.
    static inline bool IsActive()
    {
        return (minutes > 0) && enabled && !expired;
    }

    /// @brief Remaining seconds (0 when inactive/expired).
    static inline uint32_t GetRemainingSeconds()
    {
        return IsActive() ? lastRemainingSeconds : 0;
    }

    /// @brief Live remaining seconds computed from millis().
    /// @details Use this for HUD drawing so countdown updates every second even if Tick cadence varies.
    static inline uint32_t GetRemainingSecondsLive()
    {
        if (!IsActive())
            return 0;

        const uint32_t limitMs = (uint32_t)minutes * 60000UL;
        const unsigned long now = millis();
        const uint32_t elapsed = (uint32_t)(now - startMillis);

        if (elapsed >= limitMs)
            return 0;

        return (limitMs - elapsed) / 1000UL;
    }

    /// @brief Whether inputs should be locked (no HID outputs) due to timer.
    static inline bool AreInputsLocked()
    {
        // 锁定条件：配置了计时（minutes>0）且已经过期
        return (minutes > 0) && expired;
    }
};

#endif // _OPENFIREPLAYTIMER_H_

