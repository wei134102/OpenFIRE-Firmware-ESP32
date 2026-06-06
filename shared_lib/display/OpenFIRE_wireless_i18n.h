/*!
 * @file OpenFIRE_wireless_i18n.h
 * @brief Wireless / dongle / pedal OLED text (English / Chinese).
 *
 * Enable Chinese with build flag: -D OLED_MENU_ZH
 */
#ifndef _OPENFIRE_WIRELESS_I18N_H_
#define _OPENFIRE_WIRELESS_I18N_H_

#ifdef OLED_MENU_ZH

#define TXT_DONGLE_TITLE            "OpenFIRE接收器"
#define TXT_RF_SNIFF_FMT            "RF嗅探ch%2d %c"
#define TXT_WIFI_SCAN_FMT           "WiFi扫描 %c"
#define TXT_BEST_CH_EVAL            "优选信道"
#define TXT_PLEASE_WAIT             "请稍候"
#define TXT_CH_PAIR_FMT             "信道%02d配对%c"
#define TXT_TURN_ON_GUN             "请开光枪"
#define TXT_ESPNOW_LISTEN           "ESP-NOW监听"
#define TXT_RF_LINK_OK              "射频已连接"
#define TXT_WAIT_GUN_STREAM         "等待数据流"
#define TXT_AWAIT_PEDAL             "等待踏板"
#define TXT_PLAYER_FMT              "玩家:%d"
#define TXT_CHANNEL_FMT             "信道:%d"
#define TXT_CH_SHORT_FMT            "信道:%d"
#define TXT_GUN_MAC_FMT             "枪:%02X:%02X:%02X:%02X:%02X:%02X"
#define TXT_DNG_MAC_FMT             "器:%02X:%02X:%02X:%02X:%02X:%02X"
#define TXT_SEARCH_PEDAL_FMT        "踏台搜索%2d秒"
#define TXT_WAIT_LINK_FMT           "信道%2d等待"
#define TXT_SELECTED                "已选"
#define TXT_CHANNEL_LABEL           "信道:"
#define TXT_CONNECTING              "连接中"
#define TXT_SEARCHING               "搜索中"
#define TXT_BEST_CHANNEL            "优选信道"
#define TXT_SCANNING_WIFI           "WiFi扫描"

#else

#define TXT_DONGLE_TITLE            "OpenFIRE DONGLE"
#define TXT_RF_SNIFF_FMT            "RF sniff ch %2d %c"
#define TXT_WIFI_SCAN_FMT           "WiFi scan %c"
#define TXT_BEST_CH_EVAL            "Best channel eval"
#define TXT_PLEASE_WAIT             "Please wait"
#define TXT_CH_PAIR_FMT             "Ch:%02d Pair %c"
#define TXT_TURN_ON_GUN             "Turn on lightgun"
#define TXT_ESPNOW_LISTEN           "ESP-NOW listen"
#define TXT_RF_LINK_OK              "RF link OK"
#define TXT_WAIT_GUN_STREAM         "Wait gun stream"
#define TXT_AWAIT_PEDAL             "AWAIT PEDAL"
#define TXT_PLAYER_FMT              "Player: %d"
#define TXT_CHANNEL_FMT             "Channel: %d"
#define TXT_CH_SHORT_FMT            "Ch:%d"
#define TXT_GUN_MAC_FMT             "Gun: %02X:%02X:%02X:%02X:%02X:%02X"
#define TXT_DNG_MAC_FMT             "Dng: %02X:%02X:%02X:%02X:%02X:%02X"
#define TXT_SEARCH_PEDAL_FMT        "Search Pedal [%2ds]"
#define TXT_WAIT_LINK_FMT           "Ch:%2d Waiting link"
#define TXT_SELECTED                "Selected"
#define TXT_CHANNEL_LABEL           "channel:"
#define TXT_CONNECTING              "Connecting"
#define TXT_SEARCHING               "Searching"
#define TXT_BEST_CHANNEL            "best channel"
#define TXT_SCANNING_WIFI           "Scanning WiFi"

#endif // OLED_MENU_ZH

#endif // _OPENFIRE_WIRELESS_I18N_H_
