/*!
 * @file OpenFIREBoard.h
 * @brief Build configuration for various RP2040 boards.
 *
 * @copyright Mike Lynch, 2021
 * @copyright GNU Lesser General Public License
 *
 * @author Mike Lynch
 * @version V1.0
 * @date 2021
 */

#ifndef _OPENFIREBOARD_H_
#define _OPENFIREBOARD_H_

//// COMPATIBILIY FLAGS
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32)
    // DFRobot IR camera IIC clock
    // even with cheap clips and the full length IR cam cable 1MHz is fine
    #define DFROBOT_IR_IIC_CLOCK 1000000

    // software button anti-glitch
    #define BTN_AG_MASK 0xFFFFFFFF
    #define BTN_AG_MASK2 0xFFFFFFFF
#else
    // unknown board
    // this will use millis() for camera update timing instead of a hardware timer
    #define SAMCO_NO_HW_TIMER 1
    #define DFROBOT_IR_IIC_CLOCK 400000

    // software button anti-glitch mask
    #define BTN_AG_MASK1 0xF
    #define BTN_AG_MASK2 0xF
#endif // determine SAMCO_xxx board

//// ONBOARD LEDS
// Defines for built-in NeoPixels for select boards
#ifdef NEOPIXEL_POWER
    // included NeoPixel on compatible Adafruit boards has a power pin to set active to enable
    #define NEOPIXEL_ENABLEPIN NEOPIXEL_POWER
#endif // NEOPIXEL_POWER
#ifdef PIN_NEOPIXEL
    // Waveshare RP2040 Zero has a Pixel, but no enable pin, for example
    #define NEOPIXEL_PIN PIN_NEOPIXEL
#endif // PIN_NEOPIXEL

#endif // _OPENFIREBOARD_H_
