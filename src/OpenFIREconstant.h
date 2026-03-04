 /*!
 * @file OpenFIREcommon.h
 * @brief Shared constants used throughout the OpenFIRE firmware.
 *
 * @copyright That One Seong, 2025
 * @copyright GNU Lesser General Public License
 */ 

#ifndef _OPENFIRECONSTANT_H_
#define _OPENFIRECONSTANT_H_

class FW_Const
{
public:
    // operating modes
    enum GunMode_e {
        GunMode_Init = -1,
        GunMode_Run = 0,
        GunMode_Calibration,
        GunMode_Verification,
        GunMode_Pause,
        GunMode_Docked
    };

    // run modes
    // note that this is a 5 bit value when stored in the profiles
    enum RunMode_e {
        RunMode_Normal = 0,         ///< Normal gun mode, no averaging
        RunMode_Average = 1,        ///< 2 frame moving average
        RunMode_Average2 = 2,       ///< weighted average with 3 frames
        RunMode_ProfileMax = 2,     ///< maximum mode allowed for profiles
        RunMode_Processing = 3,     ///< Processing test mode
        RunMode_Count
    };

    // numbered index of buttons, must match ButtonDesc[] order
    enum ButtonIndex_e {
        BtnIdx_Trigger = 0,
        BtnIdx_A,
        BtnIdx_B,
        BtnIdx_Reload,
        BtnIdx_Start,
        BtnIdx_Select,
        BtnIdx_Up,
        BtnIdx_Down,
        BtnIdx_Left,
        BtnIdx_Right,
        BtnIdx_Pedal,
        BtnIdx_Pedal2,
        BtnIdx_Pump,
        BtnIdx_Home
    };

    // bit mask for each button, must match ButtonDesc[] order to match the proper button events
    enum ButtonMask_e {
        BtnMask_Trigger = 1 << BtnIdx_Trigger,
        BtnMask_A = 1 << BtnIdx_A,
        BtnMask_B = 1 << BtnIdx_B,
        BtnMask_Reload = 1 << BtnIdx_Reload,
        BtnMask_Start = 1 << BtnIdx_Start,
        BtnMask_Select = 1 << BtnIdx_Select,
        BtnMask_Up = 1 << BtnIdx_Up,
        BtnMask_Down = 1 << BtnIdx_Down,
        BtnMask_Left = 1 << BtnIdx_Left,
        BtnMask_Right = 1 << BtnIdx_Right,
        BtnMask_Pedal = 1 << BtnIdx_Pedal,
        BtnMask_Pedal2 = 1 << BtnIdx_Pedal2,
        BtnMask_Pump = 1 << BtnIdx_Pump,
        BtnMask_Home = 1 << BtnIdx_Home
    };

    enum CaliStage_e {
        Cali_Init = 0,
        Cali_Top,
        Cali_Bottom,
        Cali_Left,
        Cali_Right,
        Cali_Center,
        Cali_Verify
    };

    enum PauseModeSelection_e {
        PauseMode_AnalogCenterCal = 0, // 第1项：模拟摇杆中心校准
        PauseMode_Save,                // 第2项：保持设置
        PauseMode_AnalogRangeCal,      // 第3项：模拟摇杆画圈校准（范围+中心）
        PauseMode_Calibrate,
        PauseMode_ProfileSelect,
        #ifdef USES_RUMBLE
        PauseMode_RumbleToggle,
        #endif // USES_RUMBLE
        #ifdef USES_SOLENOID
        PauseMode_SolenoidToggle,
        //PauseMode_BurstFireToggle,
        #endif // USES_SOLENOID
        PauseMode_AutofireToggle,//WEI134102 add
        PauseMode_ModeChange,//wei134102 add 模式切换
        PauseMode_LowButtonToggle,//wei13402 add  低按钮模式
        PauseMode_LayoutToggle,//wei13402 add 布局切换
        PauseMode_GunId,        // 枪ID设置 P1-P4
        PauseMode_AnalogMode,   // 模拟摇杆输出类型（Stick / DPad / Keys）
        PauseMode_AnalogKeysLayout,    // 键盘模式下的按键布局（方向键 / WASD）
        PauseMode_AnalogDeadzone,      // 模拟摇杆死区设置
        PauseMode_AxisUnsignedToggle,  // 轴输出无符号模式开关（Joypad-OS 兼容）
        #ifdef USES_RUMBLE
        PauseMode_RumbleFFToggle,
        #endif // USES_RUMBLE        
        PauseMode_PlayTimer,           // 游戏时间限制（0/5/10/15/20分钟）
        PauseMode_EscapeSignal
    };
    // Layout types
    static const constexpr uint8_t layoutSquare = 0;
    static const constexpr uint8_t layoutDiamond = 1;
    //// Button Masks
    // button combo to send an escape keypress
    static const constexpr uint32_t EscapeKeyBtnMask = BtnMask_Reload | BtnMask_Start;

    // button combo to enter pause mode
    static inline constexpr uint32_t EnterPauseModeBtnMask = BtnMask_Reload | BtnMask_Select;

    // button combo to enter pause mode (holding ver)
    static inline constexpr uint32_t EnterPauseModeHoldBtnMask = BtnMask_Trigger | BtnMask_A;

    // press any button to exit hotkey pause mode back to run mode (this is not a button combo)
    static inline constexpr uint32_t ExitPauseModeBtnMask = BtnMask_Reload | BtnMask_Home;
    //wei13402 add start
    // Button combination to enter mode change menu
    static inline constexpr uint32_t ModeChangeBtnMask = BtnMask_Start | BtnMask_Right;
    //wei13402 add end
    // press and hold any button to exit simple pause menu (this is not a button combo)
    static inline constexpr uint32_t ExitPauseModeHoldBtnMask = BtnMask_A | BtnMask_B;

    // button combo to skip the center calibration step
    static inline constexpr uint32_t SkipCalCenterBtnMask = BtnMask_A;

    // button combo to save preferences to non-volatile memory
    static inline constexpr uint32_t SaveBtnMask = BtnMask_Start | BtnMask_Select;

    // button combo to increase IR sensitivity
    static inline constexpr uint32_t IRSensitivityUpBtnMask = BtnMask_B | BtnMask_Up;

    // button combo to decrease IR sensitivity
    static inline constexpr uint32_t IRSensitivityDownBtnMask = BtnMask_B | BtnMask_Down;

    // button combinations to select a run mode
    static inline constexpr uint32_t RunModeNormalBtnMask = BtnMask_Start | BtnMask_A;
    static inline constexpr uint32_t RunModeAverageBtnMask = BtnMask_Start | BtnMask_B;

    // button combination to toggle rumble in software:
    static inline constexpr uint32_t RumbleToggleBtnMask = BtnMask_Left;

    // button combination to toggle solenoid in software:
    static inline constexpr uint32_t SolenoidToggleBtnMask = BtnMask_Right;

    static inline const char* RunModeLabels[RunMode_Count] = {
        "Normal",
        "Averaging",
        "Averaging2",
        "Processing"
    };

    enum StateFlag_e {
        // print selected profile once per pause state when the COM port is open
        StateFlag_PrintSelectedProfile = (1 << 0),
        
        // report preferences once per pause state when the COM port is open
        StateFlag_PrintPreferences = (1 << 1),
        
        // enable save (allow save once per pause state)
        StateFlag_SavePreferencesEn = (1 << 2),
        
        // print preferences storage
        StateFlag_PrintPreferencesStorage = (1 << 3)
    };

    // when serial connection resets, these flags are set
    static inline constexpr uint32_t StateFlagsDtrReset = StateFlag_PrintSelectedProfile | StateFlag_PrintPreferences | StateFlag_PrintPreferencesStorage;
};

#endif // OPENFIRECONSTANT_H