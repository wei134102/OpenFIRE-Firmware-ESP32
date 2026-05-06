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
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("OpenFIRE DONGLE"));
  display.setCursor(0, 10);
  if (sniff >= 1 && sniff <= 13) {
    display.printf("RF sniff ch %2d %c", sniff, rotazione[dongle_oled_spin_scan]);
  } else {
    display.printf("WiFi scan %c", rotazione[dongle_oled_spin_scan]);
  }
  display.setCursor(0, 22);
  display.println(F("Best channel eval"));
  display.setCursor(0, 34);
  display.print(F("Please wait"));
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
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("OpenFIRE DONGLE"));
  display.setCursor(0, 12);
  display.printf("Ch:%02d Pair %c", ch, rotazione[dongle_oled_spin_link]);
  display.setCursor(0, 24);
  display.println(F("Turn on lightgun"));
  display.setCursor(0, 36);
  display.println(F("ESP-NOW listen"));
  display.display();
}

#endif // DONGLE && USES_OLED_DISPLAY

// Non-empty TU when USES_OLED_DISPLAY is off (TFT dongle builds compile this file too).
static void dongle_oled_status_translation_unit_anchor(void) { (void)0; }
