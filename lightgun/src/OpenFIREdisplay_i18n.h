/*!
 * @file OpenFIREdisplay_i18n.h
 * @brief Pause menu text for OLED display (English / Chinese).
 *
 * Enable Chinese with build flag: -D OLED_MENU_ZH
 * Uses compact 12x12 glyphs in OpenFIREdisplay_zh_glyphs.h (~12KB, 4MB-safe).
 */
#ifndef _OPENFIREDISPLAY_I18N_H_
#define _OPENFIREDISPLAY_I18N_H_

#ifdef OLED_MENU_ZH

#define PM_CENTER_CAL       "中心校准"
#define PM_SAVE_SETTINGS    "保存设置"
#define PM_RANGE_CAL        "画圈校准"
#define PM_CALIBRATE        "屏幕校准"
#define PM_PROFILE_SELECT   "配置选择"
#define PM_RUMBLE_TOGGLE    "震动开关"
#define PM_SOLENOID_TOGGLE  "电磁开关"
#define PM_ESCAPE_KEY       "发送Esc"
#define PM_AUTOFIRE_TOGGLE  "连发开关"
#define PM_LOW_BUTTON       "低键模式"
#define PM_LAYOUT_TOGGLE    "布局切换"
#define PM_GUN_ID           "枪号"
#define PM_STICK_MODE       "摇杆模式"
#define PM_MODE_CHANGE      "模式切换"
#define PM_PLAY_TIMER       "游戏计时"
#define PM_AXIS_MODE        "轴模式"
#define PM_SWAP_STICKS      "互换摇杆"
#define PM_RUMBLE_FFB       "震动反馈"
#define PM_INVERT_X         "反转X轴"
#define PM_INVERT_Y         "反转Y轴"
#define PM_DEADZONE         "死区"
#define PM_ON               "开"
#define PM_OFF              "关"
#define PM_MODE_MOUSEKB     "键鼠"
#define PM_MODE_GAMEPAD     "手柄"
#define PM_MODE_MISTER      "MiSTer"
#define PM_CURRENT_MOUSEKB  "当前:键鼠"
#define PM_CURRENT_GAMEPAD  "当前:手柄"
#define PM_CURRENT_MISTER   "当前:MiSTer"
#define PM_LOW_BUTTON_ON    "低键:开"
#define PM_LOW_BUTTON_OFF   "低键:关"

#else

#define PM_CENTER_CAL       " Center Calibrate "
#define PM_SAVE_SETTINGS    " Save Gun Settings "
#define PM_RANGE_CAL        " Range Calibrate "
#define PM_CALIBRATE        " Calibrate "
#define PM_PROFILE_SELECT   " Profile Select "
#define PM_RUMBLE_TOGGLE    " Rumble Toggle "
#define PM_SOLENOID_TOGGLE  " Solenoid Toggle "
#define PM_ESCAPE_KEY       " Send Escape Keypress"
#define PM_AUTOFIRE_TOGGLE  " Autofire Toggle "
#define PM_LOW_BUTTON       " Low Button Toggle "
#define PM_LAYOUT_TOGGLE    " Layout Toggle "
#define PM_GUN_ID           " Gun ID "
#define PM_STICK_MODE       " Stick Output Mode "
#define PM_MODE_CHANGE      " Mode Change "
#define PM_PLAY_TIMER       " Play Timer "
#define PM_AXIS_MODE        " Axis Mode "
#define PM_SWAP_STICKS      " Swap Sticks "
#define PM_RUMBLE_FFB       " Rumble FFB Toggle "
#define PM_INVERT_X         " Analog Invert X "
#define PM_INVERT_Y         " Analog Invert Y "
#define PM_DEADZONE         " Deadzone "
#define PM_ON               "ON"
#define PM_OFF              "OFF"
#define PM_MODE_MOUSEKB     "Mouse/KB"
#define PM_MODE_GAMEPAD     "Gamepad"
#define PM_MODE_MISTER      "MiSTer"
#define PM_CURRENT_MOUSEKB  " Current: Mouse/KB "
#define PM_CURRENT_GAMEPAD  " Current: Gamepad "
#define PM_CURRENT_MISTER   " Current: MiSTer "
#define PM_LOW_BUTTON_ON    " Low Button: ON "
#define PM_LOW_BUTTON_OFF   " Low Button: OFF "

#endif // OLED_MENU_ZH

#endif // _OPENFIREDISPLAY_I18N_H_
