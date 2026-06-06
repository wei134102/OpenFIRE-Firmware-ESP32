/**
 * SSD1306 status UI for ESP32-S3 Super Mini dongle.
 * Kept in dongle/src (not shared_lib) so it always compiles with the same
 * -D USES_OLED_DISPLAY as main.cpp; lib_extra_dirs can miss project flags.
 */
#if defined(DONGLE) && defined(USES_OLED_DISPLAY)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "OpenFIRE_wireless_i18n.h"
#include "OpenFIRE_zh_print.h"

extern Adafruit_SSD1306 display;
extern bool display_init;
extern volatile uint8_t dongle_oled_sniff_channel;
extern volatile uint8_t channel_display;

static uint8_t dongle_oled_spin_scan = 0;
static uint8_t dongle_oled_spin_link = 0;

void dongle_oled_draw_scan_status(void) {
  if (!display_init) {
    return;
  }
  const char* rotazione = "-\\|/-\\|/";
  dongle_oled_spin_scan = (dongle_oled_spin_scan + 1) % 8;
  uint8_t sniff = dongle_oled_sniff_channel;
  display.clearDisplay();
  OF_ZH_PRINT(&display, 0, 0, TXT_DONGLE_TITLE, SSD1306_WHITE, SSD1306_BLACK);
  if (sniff >= 1 && sniff <= 13) {
    OF_ZH_PRINTF(&display, 0, 12, SSD1306_WHITE, SSD1306_BLACK,
                 TXT_RF_SNIFF_FMT, sniff, rotazione[dongle_oled_spin_scan]);
  } else {
    OF_ZH_PRINTF(&display, 0, 12, SSD1306_WHITE, SSD1306_BLACK,
                 TXT_WIFI_SCAN_FMT, rotazione[dongle_oled_spin_scan]);
  }
  OF_ZH_PRINT(&display, 0, 24, TXT_BEST_CH_EVAL, SSD1306_WHITE, SSD1306_BLACK);
  OF_ZH_PRINT(&display, 0, 36, TXT_PLEASE_WAIT, SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

void dongle_oled_draw_link_status(void) {
  if (!display_init) {
    return;
  }
  const char* rotazione = "-\\|/-\\|/";
  dongle_oled_spin_link = (dongle_oled_spin_link + 1) % 8;
  uint8_t ch = channel_display;
  display.clearDisplay();
  OF_ZH_PRINT(&display, 0, 0, TXT_DONGLE_TITLE, SSD1306_WHITE, SSD1306_BLACK);
  OF_ZH_PRINTF(&display, 0, 12, SSD1306_WHITE, SSD1306_BLACK,
               TXT_CH_PAIR_FMT, ch, rotazione[dongle_oled_spin_link]);
  OF_ZH_PRINT(&display, 0, 24, TXT_TURN_ON_GUN, SSD1306_WHITE, SSD1306_BLACK);
  OF_ZH_PRINT(&display, 0, 36, TXT_ESPNOW_LISTEN, SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

#endif // DONGLE && USES_OLED_DISPLAY

// Non-empty TU when USES_OLED_DISPLAY is off (TFT dongle builds compile this file too).
static void dongle_oled_status_translation_unit_anchor(void) { (void)0; }
