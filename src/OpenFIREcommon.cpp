 /*!
 * @file OpenFIREcommon.h
 * @brief Shared methods used throughout the OpenFIRE project.
 *
 * @copyright That One Seong, 2025
 * @copyright GNU Lesser General Public License
 */ 

#include <Arduino.h>
#include <Wire.h>
#ifdef ARDUINO_ARCH_RP2040
  // for RP2040 Wii Clock Gen
  #include <pico/stdlib.h>
  #include <hardware/clocks.h>
  #include <hardware/pwm.h>
#endif // ARDUINO_ARCH_RP2040

#include "OpenFIREcommon.h"
#include "OpenFIREFeedback.h"
#include "OpenFIRElights.h"
#include "OpenFIREserial.h"

// ============ 696969 ========== redifinizione di Serial per gestire le connessione wireless seriali ========
#ifdef OPENFIRE_WIRELESS_ENABLE
    extern Stream* Serial_OpenFIRE_Stream;
    #ifdef Serial
        #define AUX_SERIAL Serial
        #undef Serial
    #endif
    #define Serial (*Serial_OpenFIRE_Stream)
#endif // OPENFIRE_WIRELESS_ENABLE
// ============ 696969 ===== fine redifinizione di Serial per gestire le connessione wireless seriali ========

#ifdef ARDUINO_ARCH_ESP32
    ESP32FIFO esp32_fifo(8);
#endif // ARDUINO_ARCH_ESP32



// button object instance (defined in OpenFIREcommon.h/OpenFIREprefs.h)
LightgunButtons FW_Common::buttons(lgbData, ButtonCount);

void FW_Common::FeedbackSet()
{
    #ifdef USES_RUMBLE
        if(OF_Prefs::pins[OF_Const::rumblePin] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::rumblePin], OUTPUT);
        else OF_Prefs::toggles[OF_Const::rumble] = false;
    #endif // USES_RUMBLE

    #ifdef USES_SOLENOID
        if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::solenoidPin], OUTPUT);
        else OF_Prefs::toggles[OF_Const::solenoid] = false;
    #endif // USES_SOLENOID

    #ifdef USES_SWITCHES
        #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0)
                pinMode(OF_Prefs::pins[OF_Const::rumbleSwitch], INPUT_PULLUP);
        #endif // USES_RUMBLE

        #ifdef USES_SOLENOID
            if(OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0)
                pinMode(OF_Prefs::pins[OF_Const::solenoidSwitch], INPUT_PULLUP);
        #endif // USES_SOLENOID

        if(OF_Prefs::pins[OF_Const::autofireSwitch] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::autofireSwitch], INPUT_PULLUP);
    #endif // USES_SWITCHES

    #ifdef USES_ANALOG
        analogReadResolution(12);
        #ifdef USES_TEMP
        if(OF_Prefs::pins[OF_Const::analogX] >= 0 && OF_Prefs::pins[OF_Const::analogY] >= 0 &&
           OF_Prefs::pins[OF_Const::analogX] != OF_Prefs::pins[OF_Const::analogY] &&
           OF_Prefs::pins[OF_Const::analogX] != OF_Prefs::pins[OF_Const::tempPin] &&
           OF_Prefs::pins[OF_Const::analogY] != OF_Prefs::pins[OF_Const::tempPin])
        #else
        if(OF_Prefs::pins[OF_Const::analogX] >= 0 && OF_Prefs::pins[OF_Const::analogY] >= 0 &&
           OF_Prefs::pins[OF_Const::analogX] != OF_Prefs::pins[OF_Const::analogY])
        #endif // USES_TEMP
            //pinMode(analogPinX, INPUT);
            //pinMode(analogPinY, INPUT);
            analogIsValid = true;
        else analogIsValid = false;
    #endif // USES_ANALOG

    #if defined(LED_ENABLE) && defined(FOURPIN_LED)
    if(OF_Prefs::pins[OF_Const::ledR] < 0 || OF_Prefs::pins[OF_Const::ledG] < 0 || OF_Prefs::pins[OF_Const::ledB] < 0)
        ledIsValid = false;
    else {
        pinMode(OF_Prefs::pins[OF_Const::ledR], OUTPUT);
        pinMode(OF_Prefs::pins[OF_Const::ledG], OUTPUT);
        pinMode(OF_Prefs::pins[OF_Const::ledB], OUTPUT);
        ledIsValid = true;
    }
    #endif // FOURPIN_LED

    #ifdef CUSTOM_NEOPIXEL
    if(OF_Prefs::pins[OF_Const::neoPixel] >= 0)
        OF_RGB::InitExternPixel(OF_Prefs::pins[OF_Const::neoPixel]);
    #endif // CUSTOM_NEOPIXEL

    #ifdef ARDUINO_ARCH_ESP32
    if (OF_Prefs::pins[OF_Const::periphSCL] >= 0 && OF_Prefs::pins[OF_Const::periphSDA] >= 0) {
    #else //rp2040
    if(OF_Prefs::pins[OF_Const::periphSCL] >= 0 && OF_Prefs::pins[OF_Const::periphSDA] >= 0 &&
       bitRead(OF_Prefs::pins[OF_Const::camSCL], 1) != bitRead(OF_Prefs::pins[OF_Const::periphSCL], 1) &&
       bitRead(OF_Prefs::pins[OF_Const::camSDA], 1) != bitRead(OF_Prefs::pins[OF_Const::periphSDA], 1)) {
    #endif
    #ifdef USES_DISPLAY
        // wrapper will manage display validity
        // check it's not using the camera's I2C line
        if(OF_Prefs::toggles[OF_Const::i2cOLED]) {
            if(!OLED.Begin()) { if(OLED.display != nullptr) delete OLED.display; }
        }
    #endif // USES_DISPLAY
    }
}

void FW_Common::PinsReset()
{
    if(dfrIRPos != nullptr) {
        delete dfrIRPos;
        dfrIRPos = nullptr;
    }

    #ifdef USES_RUMBLE
        if(OF_Prefs::pins[OF_Const::rumblePin] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::rumblePin], INPUT);
    #endif // USES_RUMBLE

    #ifdef USES_SOLENOID
        if(OF_Prefs::pins[OF_Const::solenoidPin] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::solenoidPin], INPUT);
    #endif // USES_SOLENOID

    #ifdef USES_SWITCHES
        #ifdef USES_RUMBLE
            if(OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0)
                pinMode(OF_Prefs::pins[OF_Const::rumbleSwitch], INPUT);
        #endif // USES_RUMBLE

        #ifdef USES_SOLENOID
            if(OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0)
                pinMode(OF_Prefs::pins[OF_Const::solenoidSwitch], INPUT);
        #endif // USES_SOLENOID

        if(OF_Prefs::pins[OF_Const::autofireSwitch] >= 0)
            pinMode(OF_Prefs::pins[OF_Const::autofireSwitch], INPUT);
    #endif // USES_SWITCHES

    #ifdef LED_ENABLE
        OF_RGB::LedOff();

        #ifdef FOURPIN_LED
            if(ledIsValid) {
                pinMode(OF_Prefs::pins[OF_Const::ledR], INPUT);
                pinMode(OF_Prefs::pins[OF_Const::ledG], INPUT);
                pinMode(OF_Prefs::pins[OF_Const::ledB], INPUT);
            }
        #endif // FOURPIN_LED

        #ifdef CUSTOM_NEOPIXEL
            if(OF_RGB::externPixel != nullptr) {
                OF_RGB::externPixel->clear();
                delete OF_RGB::externPixel;
                OF_RGB::externPixel = nullptr;
            }
        #endif // CUSTOM_NEOPIXEL
    #endif // LED_ENABLE

    #ifdef USES_DISPLAY
        if(OLED.display != nullptr)
            OLED.Stop();
    #endif // USES_DISPLAY
}

void FW_Common::CameraSet()
{
    #ifdef ARDUINO_ARCH_ESP32
        Wire.setPins(OF_Prefs::pins[OF_Const::camSDA], OF_Prefs::pins[OF_Const::camSCL]); // MODIFICATO 696969 per ESP32
        dfrIRPos = new DFRobotIRPositionEx(Wire);
    #else // rp2040   
    // Sanity check: which channel do these pins correlate to?
    if(bitRead(OF_Prefs::pins[OF_Const::camSCL], 1) && bitRead(OF_Prefs::pins[OF_Const::camSDA], 1)) {
        // I2C1
        if(bitRead(OF_Prefs::pins[OF_Const::camSCL], 0) && !bitRead(OF_Prefs::pins[OF_Const::camSDA], 0)) {
            // SDA/SCL are indeed on verified correct pins
            Wire1.setSDA(OF_Prefs::pins[OF_Const::camSDA]);
            Wire1.setSCL(OF_Prefs::pins[OF_Const::camSCL]);
            dfrIRPos = new DFRobotIRPositionEx(Wire1);
        }
        
    } else if(!bitRead(OF_Prefs::pins[OF_Const::camSCL], 1) && !bitRead(OF_Prefs::pins[OF_Const::camSDA], 1)) {
        // I2C0
        if(bitRead(OF_Prefs::pins[OF_Const::camSCL], 0) && !bitRead(OF_Prefs::pins[OF_Const::camSDA], 0)) {
            // SDA/SCL are indeed on verified correct pins
            Wire.setSDA(OF_Prefs::pins[OF_Const::camSDA]);
            Wire.setSCL(OF_Prefs::pins[OF_Const::camSCL]);
            dfrIRPos = new DFRobotIRPositionEx(Wire);
        }
    }
    #endif

    #ifdef ARDUINO_ARCH_RP2040
    if(OF_Prefs::pins[OF_Const::wiiClockGen] > -1) {
        set_sys_clock_khz(125000, true);
        gpio_set_function(OF_Prefs::pins[OF_Const::wiiClockGen], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(OF_Prefs::pins[OF_Const::wiiClockGen]);
        pwm_set_clkdiv(slice_num, 1.0f);  // for sys_clock = 125MHz
        //pwm_set_clkdiv(slice_num, 3.0f); // for sys_clock = 150MHz
        pwm_set_wrap(slice_num, 4);
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(OF_Prefs::pins[OF_Const::wiiClockGen]), 2);
        pwm_set_enabled(slice_num, true);
    } else {
        set_sys_clock_khz(133000, true);
        pwm_set_enabled(pwm_gpio_to_slice_num(OF_Prefs::pins[OF_Const::wiiClockGen]), false);
        gpio_set_function(OF_Prefs::pins[OF_Const::wiiClockGen], GPIO_FUNC_SIO);
        gpio_put(OF_Prefs::pins[OF_Const::wiiClockGen], 0);
        gpio_set_dir(OF_Prefs::pins[OF_Const::wiiClockGen], GPIO_IN);
    }
    #endif // ARDUINO_ARCH_RP2040

    // Start IR Camera with basic data format
    if(dfrIRPos != nullptr) {
        if(!dfrIRPos->begin(DFROBOT_IR_IIC_CLOCK, DFRobotIRPositionEx::DataFormat_Basic, (DFRobotIRPositionEx::Sensitivity_e)OF_Prefs::profiles[OF_Prefs::currentProfile].irSens)) {
            delete dfrIRPos;
            dfrIRPos = nullptr;
            PrintIrError();
        } else camNotAvailable = false;
    } else PrintIrError();
}

void FW_Common::SetMode(const FW_Const::GunMode_e &newMode)
{
    if(gunMode == newMode)
        return;
    
    // exit current mode
    switch(gunMode) {
    case FW_Const::GunMode_Run:
        stateFlags |= FW_Const::StateFlag_PrintPreferences;
        // MAKE SURE EVERYTHING IS DISENGAGED:
        OF_FFB::FFBShutdown();
        buttons.ReleaseAll();
        buttons.ReportDisable();
        break;
    case FW_Const::GunMode_Pause:
        break;
    case FW_Const::GunMode_Docked:
        if(newMode != FW_Const::GunMode_Calibration)
            Serial.println("Undocking.");
        break;
    }
    
    // enter new mode
    gunMode = newMode;
    switch(newMode) {
    case FW_Const::GunMode_Run:
        // begin run mode with all 4 points seen
        lastSeen = 0x0F;

        #ifdef USES_DISPLAY
            if(OLED.serialDisplayType == ExtDisplay::ScreenSerial_Both)
                OLED.ScreenModeChange(ExtDisplay::Screen_Mamehook_Dual);
            else if(OF_Serial::serialMode)
                OLED.ScreenModeChange(ExtDisplay::Screen_Mamehook_Single, buttons.analogOutput);
            else OLED.ScreenModeChange(ExtDisplay::Screen_Normal, buttons.analogOutput);

            OLED.TopPanelUpdate("Prof: ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);
        #endif // USES_DISPLAY

        break;
    case FW_Const::GunMode_Calibration:
        #ifdef USES_DISPLAY
            OLED.ScreenModeChange(ExtDisplay::Screen_Calibrating);
            OLED.TopPanelUpdate("Cali: ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);
        #endif // USES_DISPLAY
        break;
    case FW_Const::GunMode_Pause:
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn | FW_Const::StateFlag_PrintSelectedProfile;
        pauseModeSelection = FW_Const::PauseMode_Calibrate;

        #ifdef USES_DISPLAY
          OLED.ScreenModeChange(ExtDisplay::Screen_Pause);
          OLED.TopPanelUpdate("Using ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);

          if(OF_Prefs::toggles[OF_Const::simplePause]) 
              OLED.PauseListUpdate(pauseModeSelection);
          else OLED.PauseScreenShow(OF_Prefs::currentProfile, OF_Prefs::profiles[0].name, OF_Prefs::profiles[1].name, OF_Prefs::profiles[2].name, OF_Prefs::profiles[3].name);
        #endif // USES_DISPLAY

        break;
    case FW_Const::GunMode_Docked:
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn;

        #ifdef USES_DISPLAY
            OLED.ScreenModeChange(ExtDisplay::Screen_Docked);
        #endif // USES_DISPLAY

        break;
    }

    #ifdef LED_ENABLE
        SetLedColorFromMode();
    #endif // LED_ENABLE
}

void FW_Common::SetRunMode(const FW_Const::RunMode_e &newMode)
{
    if(newMode >= FW_Const::RunMode_Count)
        return;

    // block Processing/test modes being applied to a profile
    if(newMode <= FW_Const::RunMode_ProfileMax && OF_Prefs::profiles[OF_Prefs::currentProfile].runMode != newMode) {
        OF_Prefs::profiles[OF_Prefs::currentProfile].runMode = newMode;
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn;
    }
    
    if(runMode != newMode) {
        runMode = newMode;
        //if(!(stateFlags & FW_Const::StateFlag_PrintSelectedProfile))
            //PrintRunMode();
    }
}

// Dedicated calibration method
void FW_Common::ExecCalMode(const bool &fromDesktop)
{
    buttons.ReportDisable();

    uint8_t calStage = 0;
    char buf[6];
    buf[0] = OF_Const::sCaliInfoUpd;

    // hold values in a buffer till calibration is complete
    int topOffset;
    int bottomOffset;
    int leftOffset;
    int rightOffset;

    // backup current values in case the user cancels
    int _topOffset = OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset;
    int _bottomOffset = OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset;
    int _leftOffset = OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset;
    int _rightOffset = OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset;
    float _TLled = OF_Prefs::profiles[OF_Prefs::currentProfile].TLled;
    float _TRled = OF_Prefs::profiles[OF_Prefs::currentProfile].TRled;
    float _adjX = OF_Prefs::profiles[OF_Prefs::currentProfile].adjX;
    float _adjY = OF_Prefs::profiles[OF_Prefs::currentProfile].adjY;

    // set current values to factory defaults
    OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset = 0;
    OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset = 0;
    OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset = 0;
    OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset = 0;

    // Force center mouse to center
    AbsMouse5.move(32768/2, 32768/2);
    AbsMouse5.report();

    // Initialize current mouse positions (local variables)
    int32_t mouseCurrentX = 32768 / 2;
    int32_t mouseCurrentY = 32768 / 2;

    // Initialize variables for incremental movement
    int32_t mouseTargetX = mouseCurrentX;
    int32_t mouseTargetY = mouseCurrentY;
    bool mouseMoving = false;

    // Jack in, CaliMan, execute!!!
    SetMode(FW_Const::GunMode_Calibration);
    Serial.printf("%c%c", OF_Const::sCaliStageUpd, FW_Const::Cali_Init);

    while(gunMode == FW_Const::GunMode_Calibration) {
        buttons.Poll(1);

        if(irPosUpdateTick) {
            irPosUpdateTick = 0;
            GetPosition();
        }

        // Handle incremental mouse movement
        if (mouseMoving) {
            int32_t deltaX = mouseTargetX - mouseCurrentX;
            int32_t deltaY = mouseTargetY - mouseCurrentY;
            int32_t stepX = 30;
            int32_t stepY = 30;

            if (abs(deltaX) < stepX) stepX = abs(deltaX);
            if (abs(deltaY) < stepY) stepY = abs(deltaY);

            if (deltaX != 0)
                mouseCurrentX += (deltaX > 0) ? stepX : -stepX;

            if (deltaY != 0)
                mouseCurrentY += (deltaY > 0) ? stepY : -stepY;

            AbsMouse5.move(mouseCurrentX, mouseCurrentY);
            AbsMouse5.report();

            if (mouseCurrentX == mouseTargetX && mouseCurrentY == mouseTargetY) {
                mouseMoving = false;
                delay(5);  // Optional small delay
            }
        }

        // Handle button presses and calibration stages
        if((buttons.pressedReleased & (FW_Const::ExitPauseModeBtnMask | FW_Const::ExitPauseModeHoldBtnMask) || Serial.read() == OF_Const::serialTerminator) && !justBooted) {
            Serial.printf("%c%c", OF_Const::sCaliStageUpd, FW_Const::Cali_Verify+1);
            Serial.flush();

            // Reapplying backed up data
            OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset = _topOffset;
            OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset = _bottomOffset;
            OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset = _leftOffset;
            OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset = _rightOffset;
            OF_Prefs::profiles[OF_Prefs::currentProfile].TLled = _TLled;
            OF_Prefs::profiles[OF_Prefs::currentProfile].TRled = _TRled;
            OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = _adjX;
            OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = _adjY;

            // Re-print the profile
            stateFlags |= FW_Const::StateFlag_PrintSelectedProfile;

            // Exit back to docked mode or run mode, depending on if pinged from Desktop App
            if(fromDesktop)
                SetMode(FW_Const::GunMode_Docked);
            else SetMode(FW_Const::GunMode_Run);

            return;
        } else if(buttons.pressed == FW_Const::BtnMask_Trigger && !mouseMoving) {
            Serial.printf("%c%c", OF_Const::sCaliStageUpd, ++calStage);
            // Ensure our messages go through, or else the HID reports eat UART.
            Serial.flush();

            switch(calStage) {
                case FW_Const::Cali_Init:
                    // Initial state, nothing to do (but center cursor for desktop use)
                    if(fromDesktop) {
                        AbsMouse5.move(32768/2, 32768/2);
                        AbsMouse5.report();
                    }
                    break;
                case FW_Const::Cali_Top:
                    // Reset Offsets
                    topOffset = 0;
                    bottomOffset = 0;
                    leftOffset = 0;
                    rightOffset = 0;

                    // Set Cam center offsets
                    if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) {
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = (OpenFIREdiamond.testMedianX() - (512 << 2)) * cos(OpenFIREdiamond.Ang()) -
                                                                     (OpenFIREdiamond.testMedianY() - (384 << 2)) * sin(OpenFIREdiamond.Ang()) + (512 << 2);
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = (OpenFIREdiamond.testMedianX() - (512 << 2)) * sin(OpenFIREdiamond.Ang()) +
                                                                     (OpenFIREdiamond.testMedianY() - (384 << 2)) * cos(OpenFIREdiamond.Ang()) + (384 << 2);
                    } else {
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = (OpenFIREsquare.testMedianX() - (512 << 2)) * cos(OpenFIREsquare.Ang()) -
                                                                     (OpenFIREsquare.testMedianY() - (384 << 2)) * sin(OpenFIREsquare.Ang()) + (512 << 2);
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = (OpenFIREsquare.testMedianX() - (512 << 2)) * sin(OpenFIREsquare.Ang()) +
                                                                     (OpenFIREsquare.testMedianY() - (384 << 2)) * cos(OpenFIREsquare.Ang()) + (384 << 2);
                        // Work out LED locations by assuming height is 100%
                        OF_Prefs::profiles[OF_Prefs::currentProfile].TLled = (res_x / 2) - ((OpenFIREsquare.W() * (res_y  / OpenFIREsquare.H())) / 2);
                        OF_Prefs::profiles[OF_Prefs::currentProfile].TRled = (res_x / 2) + ((OpenFIREsquare.W() * (res_y  / OpenFIREsquare.H())) / 2);
                    }

                    // Update Cam centre in perspective library
                    OpenFIREper.source(OF_Prefs::profiles[OF_Prefs::currentProfile].adjX, OF_Prefs::profiles[OF_Prefs::currentProfile].adjY);
                    OpenFIREper.deinit(0);

                    // Set mouse movement to top position
                    if(!fromDesktop) {
                        mouseTargetX = 32768 / 2;
                        mouseTargetY = 0;
                        mouseMoving = true;
                    }
                    break;
                case FW_Const::Cali_Bottom:
                    // Set Offset buffer
                    topOffset = mouseY;

                    buf[1] = 1;
                    memcpy(&buf[2], &topOffset, sizeof(int));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    // Set mouse movement to bottom position
                    if(!fromDesktop) {
                        mouseTargetX = 32768 / 2;
                        mouseTargetY = 32767;
                        mouseMoving = true;
                    }
                    break;
                case FW_Const::Cali_Left:
                    // Set Offset buffer
                    bottomOffset = (res_y - mouseY);

                    buf[1] = 2;
                    memcpy(&buf[2], &bottomOffset, sizeof(int));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    // Set mouse movement to left position
                    if(!fromDesktop) {
                        mouseTargetX = 0;
                        mouseTargetY = 32768 / 2;
                        mouseMoving = true;
                    }
                    break;
                case FW_Const::Cali_Right:
                    // Set Offset buffer
                    leftOffset = mouseX;

                    buf[1] = 3;
                    memcpy(&buf[2], &leftOffset, sizeof(int));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    // Set mouse movement to right position
                    if(!fromDesktop) {
                        mouseTargetX = 32767;
                        mouseTargetY = 32768 / 2;
                        mouseMoving = true;
                    }
                    break;
                case FW_Const::Cali_Center:
                    // Set Offset buffer
                    rightOffset = (res_x - mouseX);

                    buf[1] = 4;
                    memcpy(&buf[2], &rightOffset, sizeof(int));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    // Save Offset buffer to profile
                    OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset = topOffset;
                    OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset = bottomOffset;
                    OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset = leftOffset;
                    OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset = rightOffset;

                    // Move back to center calibration point
                    if(!fromDesktop) {
                        mouseTargetX = 32768 / 2;
                        mouseTargetY = 32768 / 2;
                        mouseMoving = true;
                    }
                    break;
                case FW_Const::Cali_Verify:
                    // Apply new Cam center offsets with Offsets applied
                    if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) {
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = (OpenFIREdiamond.testMedianX() - (512 << 2)) * cos(OpenFIREdiamond.Ang()) -
                                                                     (OpenFIREdiamond.testMedianY() - (384 << 2)) * sin(OpenFIREdiamond.Ang()) + (512 << 2);
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = (OpenFIREdiamond.testMedianX() - (512 << 2)) * sin(OpenFIREdiamond.Ang()) +
                                                                     (OpenFIREdiamond.testMedianY() - (384 << 2)) * cos(OpenFIREdiamond.Ang()) + (384 << 2);
                    } else {
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = (OpenFIREsquare.testMedianX() - (512 << 2)) * cos(OpenFIREsquare.Ang()) -
                                                                     (OpenFIREsquare.testMedianY() - (384 << 2)) * sin(OpenFIREsquare.Ang()) + (512 << 2);
                        OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = (OpenFIREsquare.testMedianX() - (512 << 2)) * sin(OpenFIREsquare.Ang()) +
                                                                     (OpenFIREsquare.testMedianY() - (384 << 2)) * cos(OpenFIREsquare.Ang()) + (384 << 2);
                    }

                    buf[1] = 5;
                    memcpy(&buf[2], &OF_Prefs::profiles[OF_Prefs::currentProfile].TLled, sizeof(float));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    buf[1] = 6;
                    memcpy(&buf[2], &OF_Prefs::profiles[OF_Prefs::currentProfile].TRled, sizeof(float));
                    Serial.write(buf, sizeof(buf));
                    Serial.flush();

                    // Update Cam centre in perspective library
                    OpenFIREper.source(OF_Prefs::profiles[OF_Prefs::currentProfile].adjX, OF_Prefs::profiles[OF_Prefs::currentProfile].adjY);
                    OpenFIREper.deinit(0);

                    // Let the user test.
                    SetMode(FW_Const::GunMode_Verification);
                    while(gunMode == FW_Const::GunMode_Verification) {
                        buttons.Poll();

                        if(irPosUpdateTick) {
                            irPosUpdateTick = 0;
                            GetPosition();
                        }

                        // If it's good, move onto calibration finish.
                        if(buttons.pressed == FW_Const::BtnMask_Trigger) {
                            calStage++;
                            // Stay in Verification Mode; the code outside of the calibration loop will catch us.
                            break;
                        // Press A/B to restart calibration for current profile
                        } else if(buttons.pressedReleased & FW_Const::ExitPauseModeHoldBtnMask) {
                            calStage = 0;
                            Serial.printf("%c%c", OF_Const::sCaliStageUpd, FW_Const::Cali_Init);
                            Serial.flush();

                            // (Re)set current values to factory defaults
                            OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset = 0;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset = 0;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset = 0;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset = 0;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = 512 << 2;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = 384 << 2;
                            SetMode(FW_Const::GunMode_Calibration);
                            AbsMouse5.move(32768/2, 32768/2);
                            AbsMouse5.report();
                        // Press C/Home to exit without committing new calibration values
                        } else if(buttons.pressedReleased & FW_Const::ExitPauseModeBtnMask && !justBooted) {
                            Serial.printf("%c%c", OF_Const::sCaliStageUpd, FW_Const::Cali_Verify+1);
                            Serial.flush();

                            // Reapply backed-up data
                            OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset = _topOffset;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset = _bottomOffset;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset = _leftOffset;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset = _rightOffset;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].TLled = _TLled;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].TRled = _TRled;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].adjX = _adjX;
                            OF_Prefs::profiles[OF_Prefs::currentProfile].adjY = _adjY;

                            // Re-print the profile
                            stateFlags |= FW_Const::StateFlag_PrintSelectedProfile;

                            // Re-apply the calibration stored in the profile
                            if(fromDesktop)
                                SetMode(FW_Const::GunMode_Docked);
                            else SetMode(FW_Const::GunMode_Run);
                            return;
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }

    // Break calibration
    if(justBooted) {
        // If this is an initial calibration, save it immediately!
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn;
        SavePreferences();
        if(fromDesktop)
            SetMode(FW_Const::GunMode_Docked);
    } else if(fromDesktop) {
        SetMode(FW_Const::GunMode_Docked);
    } else SetMode(FW_Const::GunMode_Run);

    #ifdef USES_RUMBLE
        if(OF_Prefs::toggles[OF_Const::rumble]) {
            analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
            delay(80);
            #ifdef ARDUINO_ARCH_ESP32
                analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);  // 696969 per EPS32
            #else // rp2040
            digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
            #endif
            delay(50);
            analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
            delay(125);
            #ifdef ARDUINO_ARCH_ESP32
                analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);  // 696969 per ESP32
            #else // rp2040            
            digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
            #endif
        }
    #endif // USES_RUMBLE

    Serial.printf("%c%c", OF_Const::sCaliStageUpd, FW_Const::Cali_Verify+1);
    Serial.flush();
}

void FW_Common::GetPosition()
{
    if(dfrIRPos != nullptr) {
        int error = dfrIRPos->basicAtomic(DFRobotIRPositionEx::Retry_2);
        if(error == DFRobotIRPositionEx::Error_Success) {
            
            // ====== 696969 ========== inserire qua filtro di KALMAN sui dati grezzi raw ======================
            #ifdef CAM_SIMPLE_KALMAN_FILTER
                memcpy(positionX,dfrIRPos->xPositions(), sizeof(positionX));
                memcpy(positionY,dfrIRPos->yPositions(), sizeof(positionY));
                seenFlags = dfrIRPos->seen();
                /*
                if(seenFlags == 0x0F) { // lo esegue solo se vede tutti e 4 i punti -> valutare di farlo se anche ne vede meno
                    // esegue il filtro su tutti i punti
                    for (uint8_t i = 0; i < 4; i++) {
                        Kalman_filter(i, positionX[i], positionY[i]);
                    }
                }
                */
            #endif // CAM_SIMPLE_KALMAN_FILTER
            // ======= 696969 ==================================================================================

            // if diamond layout, or square
            if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) { // layoutDiamond = 1
                #ifdef CAM_SIMPLE_KALMAN_FILTER
                    OpenFIREdiamond.begin(positionX, positionY, seenFlags);
                #else              
                    OpenFIREdiamond.begin(dfrIRPos->xPositions(), dfrIRPos->yPositions(), dfrIRPos->seen());
                #endif // CAM_SIMPLE_KALMAN_FILTER             

                OpenFIREper.warp(OpenFIREdiamond.X(0), OpenFIREdiamond.Y(0),
                                OpenFIREdiamond.X(1), OpenFIREdiamond.Y(1),
                                OpenFIREdiamond.X(2), OpenFIREdiamond.Y(2),
                                OpenFIREdiamond.X(3), OpenFIREdiamond.Y(3),
                                res_x / 2, 0, 0,
                                res_y / 2, res_x / 2,
                                res_y, res_x, res_y / 2);
            } else { // layoutSquare = 0
                #ifdef CAM_SIMPLE_KALMAN_FILTER
                    OpenFIREsquare.begin(positionX, positionY, seenFlags);
                #else              
                    OpenFIREsquare.begin(dfrIRPos->xPositions(), dfrIRPos->yPositions(), dfrIRPos->seen());
                #endif // CAM_SIMPLE_KALMAN_FILTER
                
                #ifdef TEST_CAM
                positionX[0] = OpenFIREsquare.X(0);
                positionY[0] = OpenFIREsquare.Y(0);
                positionX[1] = OpenFIREsquare.X(1);
                positionY[1] = OpenFIREsquare.Y(1);
                positionX[2] = OpenFIREsquare.X(2);
                positionY[2] = OpenFIREsquare.Y(2);
                positionX[3] = OpenFIREsquare.X(3);
                positionY[3] = OpenFIREsquare.Y(3);
                #endif //TESTCAM

                #ifdef CAM_SIMPLE_KALMAN_FILTER
                for (uint8_t i = 0; i < 4; i++) {
                    Kalman_filter(i, positionX[i], positionY[i]);
                }
                #endif //CAM_SIMPLE_KALMAN_FILTER

                #ifdef CAM_SIMPLE_KALMAN_FILTER
                OpenFIREper.warp(positionX[0], positionY[0],
                                positionX[1], positionY[1],
                                positionX[2], positionY[2],
                                positionX[3], positionY[3],
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TLled, 0,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TRled, 0,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TLled, res_y,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TRled, res_y);                
                #else
                OpenFIREper.warp(OpenFIREsquare.X(0), OpenFIREsquare.Y(0),
                                OpenFIREsquare.X(1), OpenFIREsquare.Y(1),
                                OpenFIREsquare.X(2), OpenFIREsquare.Y(2),
                                OpenFIREsquare.X(3), OpenFIREsquare.Y(3),
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TLled, 0,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TRled, 0,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TLled, res_y,
                                OF_Prefs::profiles[OF_Prefs::currentProfile].TRled, res_y);
                #endif // CAM_SIMPLE_KALMAN_FILTER
            }

            #ifdef CAM_SIMPLE_KALMAN_FILTER_______________________X
            int aux_getX = OpenFIREper.getX();
            int aux_getY = OpenFIREper.getY();
            Kalman_filter(aux_getX, aux_getY);
            mouseX = map(aux_getX, 0, res_x, (0 - OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset), (res_x + OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset));                 
            mouseY = map(aux_getY, 0, res_y, (0 - OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset), (res_y + OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset));          
            #else
            // Output mapped to screen resolution because offsets are measured in pixels
            mouseX = map(OpenFIREper.getX(), 0, res_x, (0 - OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset), (res_x + OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset));                 
            mouseY = map(OpenFIREper.getY(), 0, res_y, (0 - OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset), (res_y + OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset));
            #endif // CAM_SIMPLE_KALMAN_FILTER


            // =============== 696969 ============== filter ===============================
            /*
            #ifdef CAM_SIMPLE_KALMAN_FILTER
            Kalman_filter(mouseX, mouseY);
            #endif // CAM_SIMPLE_KALMAN_FILTER
            */
            // =============== 696969 ============== filter ===============================

            switch(runMode) {
                case FW_Const::RunMode_Average:
                    // 2 position moving average
                    moveIndex ^= 1;
                    moveXAxisArr[moveIndex] = mouseX;
                    moveYAxisArr[moveIndex] = mouseY;
                    mouseX = (moveXAxisArr[0] + moveXAxisArr[1]) / 2;
                    mouseY = (moveYAxisArr[0] + moveYAxisArr[1]) / 2;
                    break;
                case FW_Const::RunMode_Average2:
                    // weighted average of current position and previous 2
                    if(moveIndex < 2)
                        ++moveIndex;
                    else moveIndex = 0;

                    moveXAxisArr[moveIndex] = mouseX;
                    moveYAxisArr[moveIndex] = mouseY;
                    mouseX = (mouseX + moveXAxisArr[0] + moveXAxisArr[1] + moveXAxisArr[2]) / 4;
                    mouseY = (mouseY + moveYAxisArr[0] + moveYAxisArr[1] + moveYAxisArr[2]) / 4;
                    break;
                default:
                    break;
                }

            // Constrain that bisch so negatives don't cause underflow
            int32_t conMoveX = constrain(mouseX, 0, res_x);
            int32_t conMoveY = constrain(mouseY, 0, res_y);

            if(gunMode == FW_Const::GunMode_Run) {
                UpdateLastSeen();

                if(OF_Serial::serialARcorrection) switch(OF_Prefs::profiles[OF_Prefs::currentProfile].aspectRatio) {
                    case OF_Const::ar16_9:
                        conMoveX = map(conMoveX, 966, 6720, 0, 32767);
                        conMoveX = constrain(conMoveX, 0, 32767);
                        conMoveY = map(conMoveY, 0, res_y, 0, 32767);
                        break;
                    case OF_Const::ar16_10:
                        conMoveX = map(conMoveX, 655, 7048, 0, 32767);
                        conMoveX = constrain(conMoveX, 0, 32767);
                        conMoveY = map(conMoveY, 0, res_y, 0, 32767);
                        break;
                    case OF_Const::ar3_2:
                        conMoveX = map(conMoveX, 438, 7264, 0, 32767);
                        conMoveX = constrain(conMoveX, 0, 32767);
                        conMoveY = map(conMoveY, 0, res_y, 0, 32767);
                        break;
                    case OF_Const::ar5_4:
                        conMoveX = map(conMoveX, 0, res_x, 0, 32767);
                        conMoveY = map(conMoveY, 148, 4182, 0, 32767);
                        conMoveY = constrain(conMoveY, 0, 32767);
                        break;
                    case OF_Const::ar4_3:
                    default:
                        // Output mapped to Mouse resolution
                        conMoveX = map(conMoveX, 0, res_x, 0, 32767);
                        conMoveY = map(conMoveY, 0, res_y, 0, 32767);
                        break;
                } else {
                    // Output mapped to Mouse resolution
                    conMoveX = map(conMoveX, 0, res_x, 0, 32767);
                    conMoveY = map(conMoveY, 0, res_y, 0, 32767);
                }

                bool offXAxis = false;
                bool offYAxis = false;

                if(conMoveX == 0 || conMoveX == 32767)
                    offXAxis = true;
                
                if(conMoveY == 0 || conMoveY == 32767)
                    offYAxis = true;

                if(offXAxis || offYAxis)
                     buttons.offScreen = true;
                else buttons.offScreen = false;

                if(buttons.analogOutput)
                     Gamepad16.moveCam(conMoveX, conMoveY);
                else AbsMouse5.move(conMoveX, conMoveY);

                #ifdef TEST_CAM
                    if(millis() - testLastStamp > 30) {
                    testLastStamp = millis();
                    // RAW Camera Output mapped to screen res (1920x1080)
                    int rawX[4];
                    int rawY[4];
                    // RAW Output for viewing in processing sketch mapped to 1920x1080 screen resolution
                    for (int i = 0; i < 4; ++i) {
                        rawX[i] = map(positionX[i], 0, 1023 << 2, 1920, 0);
                        rawY[i] = map(positionY[i], 0, 768 << 2, 0, 1080);
                        //Serial.printf("P(%1d):%5d,%5d ", i, positionX[i],positionY[i]);
                    }    
                    //Serial.println();
                    #ifdef USES_DISPLAY
                        OLED.DrawVisibleIR(rawX, rawY);
                    #endif // USES_DISPLAY
                }
                    
                #endif // TEST CAM

            } else {
                if(gunMode == FW_Const::GunMode_Verification) {
                    // Output mapped to Mouse resolution
                    conMoveX = map(conMoveX, 0, res_x, 0, 32767);
                    conMoveY = map(conMoveY, 0, res_y, 0, 32767);

                    AbsMouse5.move(conMoveX, conMoveY);
                    AbsMouse5.report();
                }
                if(millis() - testLastStamp > 50) {
                    testLastStamp = millis();
                    // RAW Camera Output mapped to screen res (1920x1080)
                    int rawX[4];
                    int rawY[4];
                    // RAW Output for viewing in processing sketch mapped to 1920x1080 screen resolution
                    for (int i = 0; i < 4; ++i) {
                        if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) {
                            rawX[i] = map(OpenFIREdiamond.X(i), 0, 1023 << 2, 1920, 0);
                            rawY[i] = map(OpenFIREdiamond.Y(i), 0, 768 << 2, 0, 1080);
                        } else {
                            rawX[i] = map(OpenFIREsquare.X(i), 0, 1023 << 2, 0, 1920);
                            rawY[i] = map(OpenFIREsquare.Y(i), 0, 768 << 2, 0, 1080);
                        }
                    }

                    if(runMode == FW_Const::RunMode_Processing) {
                        int mouseXscaled = mouseX / 4;
                        int mouseYscaled = mouseY / 4;

                        if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) {
                            int testMedianX = map(OpenFIREdiamond.testMedianX(), 0, 1023 << 2, 1920, 0);
                            int testMedianY = map(OpenFIREdiamond.testMedianY(), 0, 768 << 2, 0, 1080);
                            char buf[49];
                            buf[0] = OF_Const::sTestCoords;
                            memcpy(&buf[1],  &rawX[0],      sizeof(int));
                            memcpy(&buf[5],  &rawY[0],      sizeof(int));
                            memcpy(&buf[9],  &rawX[1],      sizeof(int));
                            memcpy(&buf[13], &rawY[1],      sizeof(int));
                            memcpy(&buf[17], &rawX[2],      sizeof(int));
                            memcpy(&buf[21], &rawY[2],      sizeof(int));
                            memcpy(&buf[25], &rawX[3],      sizeof(int));
                            memcpy(&buf[29], &rawY[3],      sizeof(int));
                            memcpy(&buf[33], &mouseXscaled, sizeof(int));
                            memcpy(&buf[37], &mouseYscaled, sizeof(int));
                            memcpy(&buf[41], &testMedianX,  sizeof(int));
                            memcpy(&buf[45], &testMedianY,  sizeof(int));
                            Serial.write(buf, sizeof(buf));
                        } else {
                            int testMedianX = map(OpenFIREsquare.testMedianX(), 0, 1023 << 2, 0, 1920);
                            int testMedianY = map(OpenFIREsquare.testMedianY(), 0, 768 << 2, 0, 1080);
                            char buf[49];
                            buf[0] = OF_Const::sTestCoords;
                            memcpy(&buf[1],  &rawX[0],      sizeof(int));
                            memcpy(&buf[5],  &rawY[0],      sizeof(int));
                            memcpy(&buf[9],  &rawX[1],      sizeof(int));
                            memcpy(&buf[13], &rawY[1],      sizeof(int));
                            memcpy(&buf[17], &rawX[2],      sizeof(int));
                            memcpy(&buf[21], &rawY[2],      sizeof(int));
                            memcpy(&buf[25], &rawX[3],      sizeof(int));
                            memcpy(&buf[29], &rawY[3],      sizeof(int));
                            memcpy(&buf[33], &mouseXscaled, sizeof(int));
                            memcpy(&buf[37], &mouseYscaled, sizeof(int));
                            memcpy(&buf[41], &testMedianX,  sizeof(int));
                            memcpy(&buf[45], &testMedianY,  sizeof(int));
                            Serial.write(buf, sizeof(buf));
                        }
                    }

                    #ifdef USES_DISPLAY
                        OLED.DrawVisibleIR(rawX, rawY);
                    #endif // USES_DISPLAY
                }
            }
        } else if(error != DFRobotIRPositionEx::Error_DataMismatch)
            PrintIrError();
    } else  PrintIrError();
}

// ============== 696969 =====================================================================
#ifdef CAM_SIMPLE_KALMAN_FILTER   


//#ifdef COMMENTO
///////////////////////////////////////////////////////////////////////////////////////////////////


// ------------- INIZIO FUNZIONI HELPER -------------

template<typename T>
static inline T constrain_custom(T val, T min_val, T max_val) {
    // Se usi Arduino, puoi sostituire questa con la funzione constrain() standard.
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
    // Alternativamente: return std::max(min_val, std::min(val, max_val)); (richiede #include <algorithm>)
}

static inline float map_float_custom(float x, float in_min, float in_max, float out_min, float out_max) {
    // Mappatura lineare per float.
    if (fabsf(in_max - in_min) < 1e-6f) { // Evita divisione per zero o quasi
        return out_min; // o (out_min + out_max) / 2.0f
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float lerp_custom(float a, float b, float t) {
    // Interpolazione lineare: a quando t=0, b quando t=1.
    return a + t * (b - a);
}

static inline float smoothstep_custom(float edge0, float edge1, float x) {
    // Transizione cubica Hermite fluida da 0 a 1 quando x va da edge0 a edge1.
    if (edge0 == edge1) return (x >= edge0) ? 1.0f : 0.0f; // Evita divisione per zero
    float t = constrain_custom((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// ------------- FINE FUNZIONI HELPER -------------

// All'interno della tua classe FW_Common:
// void FW_Common::Kalman_filter(uint8_t i, int& mouse_input_x, int& mouse_input_y) { ... }

void FW_Common::Kalman_filter(uint8_t i, int& mouse_input_x, int& mouse_input_y) {

// --- 0. Conversione Input, Validazione e Gestione Dati Errati/Persi ---
    float potential_x = static_cast<float>(mouse_input_x);
    float potential_y = static_cast<float>(mouse_input_y);

    float current_raw_x;
    float current_raw_y;

    // X_MIN, X_MAX, Y_MIN, Y_MAX devono essere le costanti float definite nella tua classe
    if (true || (potential_x >= X_MIN && potential_x <= X_MAX &&
        potential_y >= Y_MIN && potential_y <= Y_MAX)) {
        // Il dato è valido e all'interno del range atteso
        current_raw_x = potential_x;
        current_raw_y = potential_y;
        // Aggiorna l'ultima posizione grezza valida conosciuta
        last_known_good_raw_x[i] = current_raw_x;
        last_known_good_raw_y[i] = current_raw_y;
    } else {
        // Il dato è fuori range (probabilmente perso o un errore di trasmissione)
        // Usa l'ultima posizione grezza valida conosciuta.
        current_raw_x = last_known_good_raw_x[i];
        current_raw_y = last_known_good_raw_y[i];
        
        // OPZIONALE AVANZATO: potresti voler segnalare al filtro che questo dato è "vecchio"
        // o meno affidabile, ad esempio aumentando temporaneamente r_dynamic[i]
        // o forzando Q a q_min_base[i] per questo ciclo, ma iniziamo senza.
        // Esempio: r_dynamic[i] = R_BASE * 10.0f; // Rendi la misura attuale meno fidata
    }


    // --- Calcoli Cinematici Iniziali ---
    float vx = current_raw_x - last_x[i];
    float vy = current_raw_y - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // --- Calcolo Distanze per Fattori Ambientali ---
    // (X_MIN, X_MAX, Y_MIN, Y_MAX, X_CENTER, Y_CENTER, HALF_WIDTH, HALF_HEIGHT devono essere definite)
    float dist_from_center_x_norm = fabsf(current_raw_x - X_CENTER) / HALF_WIDTH;
    float dist_from_center_y_norm = fabsf(current_raw_y - Y_CENTER) / HALF_HEIGHT;
    float dist_to_edge_x_norm = std::min(fabsf(current_raw_x - X_MIN), fabsf(current_raw_x - X_MAX)) / HALF_WIDTH;
    float dist_to_edge_y_norm = std::min(fabsf(current_raw_y - Y_MIN), fabsf(current_raw_y - Y_MAX)) / HALF_HEIGHT;

    // --- Calcolo del Fattore di Prossimità al Bordo Laterale X (Esponenziale) ---
    // (X_EDGE_PROXIMITY_DECAY_RATE deve essere definita)
    float x_edge_proximity_factor = expf(-X_EDGE_PROXIMITY_DECAY_RATE * dist_to_edge_x_norm);

    // --- 1. Smorzamento Adattivo della Velocità (con modulazione esponenziale per Y) ---
    // (DAMPING_AT_LOW_SPEED, DAMPING_AT_HIGH_SPEED, SPEED_LOW_THRESHOLD, SPEED_HIGH_THRESHOLD,
    //  DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE devono essere definite)
    float speed_abs_x = fabsf(vx);
    float speed_abs_y = fabsf(vy);

    float effective_damping_low_y = lerp_custom(DAMPING_AT_LOW_SPEED, DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE, x_edge_proximity_factor);

    float damping_factor_vx = lerp_custom(DAMPING_AT_LOW_SPEED, DAMPING_AT_HIGH_SPEED, smoothstep_custom(SPEED_LOW_THRESHOLD, SPEED_HIGH_THRESHOLD, speed_abs_x));
    float damping_factor_vy = lerp_custom(effective_damping_low_y, DAMPING_AT_HIGH_SPEED, smoothstep_custom(SPEED_LOW_THRESHOLD, SPEED_HIGH_THRESHOLD, speed_abs_y));
    
    vx *= damping_factor_vx;
    vy *= damping_factor_vy;

    // --- 3. (Resto) Calcolo Fattori di Stabilizzazione Ambientali ---
    // (EDGE_SMOOTHING, CORNER_SMOOTHING_EXPONENT_X, CORNER_SMOOTHING_EXPONENT_Y devono essere definite)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);
    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- 4. Combinazione Dinamica dei Fattori di Stabilizzazione (con modulazione esponenziale per Y) ---
    // (EDGE_TRANSITION_THRESHOLD, EXP_SMOOTHNESS_FACTOR, MIN_PRECISION_FACTOR,
    //  MIN_PRECISION_FACTOR_Y_AT_X_EDGE devono essere definite)
    float normalized_dist_in_transition_eff_x = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD;
    float factor_for_pow_x = constrain_custom(1.0f - normalized_dist_in_transition_eff_x, 0.0f, 1.0f);
    float weight_for_corner_x = powf(factor_for_pow_x, EXP_SMOOTHNESS_FACTOR);
    float combined_stabilization_factor_x = lerp_custom(stabilization_center_x, stabilization_corner_x, weight_for_corner_x);
    combined_stabilization_factor_x = constrain_custom(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f);

    float effective_min_precision_y = lerp_custom(MIN_PRECISION_FACTOR, MIN_PRECISION_FACTOR_Y_AT_X_EDGE, x_edge_proximity_factor);
    float normalized_dist_in_transition_eff_y = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
    float factor_for_pow_y = constrain_custom(1.0f - normalized_dist_in_transition_eff_y, 0.0f, 1.0f);
    float weight_for_corner_y = powf(factor_for_pow_y, EXP_SMOOTHNESS_FACTOR);
    float combined_stabilization_factor_y = lerp_custom(stabilization_center_y, stabilization_corner_y, weight_for_corner_y);
    combined_stabilization_factor_y = constrain_custom(combined_stabilization_factor_y, effective_min_precision_y, 1.0f);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- 5. Gestione della Dead Zone e Soglia di Stazionarietà (con modulazione esponenziale per Y) ---
    // (DEAD_ZONE_X, DEAD_ZONE_Y, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE,
    //  STATIONARY_THRESHOLD_CENTER_COUNT, STATIONARY_THRESHOLD_EDGE_COUNT,
    //  DEAD_ZONE_Y_SCALE_AT_X_EDGE devono essere definite)
    float max_dist_to_edge_norm = std::max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    float t_deadzone = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_deadzone = t_deadzone * t_deadzone;
    float current_dead_zone_multiplier = map_float_custom(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);

    float t_stationary = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_stationary = t_stationary * t_stationary;
    float current_stationary_threshold_count = map_float_custom(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER_COUNT, STATIONARY_THRESHOLD_EDGE_COUNT);

    float effective_dead_zone_y_scale = lerp_custom(1.0f, DEAD_ZONE_Y_SCALE_AT_X_EDGE, x_edge_proximity_factor);
    float actual_dead_zone_y = (DEAD_ZONE_Y * current_dead_zone_multiplier) * effective_dead_zone_y_scale;
    float actual_dead_zone_x = DEAD_ZONE_X * current_dead_zone_multiplier;

    if (fabsf(vx) < actual_dead_zone_x && fabsf(vy) < actual_dead_zone_y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > static_cast<int>(current_stationary_threshold_count)) {
            mouse_input_x = static_cast<int>(last_x[i]);
            mouse_input_y = static_cast<int>(last_y[i]);
            return;
        }
    } else {
        stationary_counter[i] = 0;
        // q_min_base è un array static inline, q_max è una costante
        // Assumiamo che Q_MAX sia definita e sia significativamente più grande di 0.005f
        // o del valore che vuoi per q_min_base.
        // Il valore di reset 0.005f era nell'originale; puoi scalarlo rispetto a Q_MAX se preferisci.
        q_min_base[i] = Q_MAX * 0.01f; // Esempio di reset, o il tuo 0.005f
    }

    // --- 6. Pre-filtraggio della Misura (Input per Kalman) ---
    float measured_x = (current_raw_x + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f;
    float measured_y = (current_raw_y + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f;

    // --- 7. Core del Filtro di Kalman ---
    // (ACCEL_NORMALIZATION_FACTOR, Q_MAX, ACCEL_Q_THRESH_..._LOWER/UPPER, JERK_Q_THRESH_..._LOWER/UPPER
    //  devono essere definite. p_x, p_y, q_min_base, r_dynamic devono essere accessibili e inizializzate.)
    float prediction_weight_v = constrain_custom(fabsf(vx) / X_MAX, 0.1f, 0.8f); // X_MAX qui è la costante float
    float prediction_weight_a = constrain_custom(fabsf(ax) / ACCEL_NORMALIZATION_FACTOR, 0.2f, 0.9f);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float accel_factor_q_x = smoothstep_custom(ACCEL_Q_THRESH_X_LOWER, ACCEL_Q_THRESH_X_UPPER, fabsf(ax));
    float jerk_factor_q_x  = smoothstep_custom(JERK_Q_THRESH_X_LOWER,  JERK_Q_THRESH_X_UPPER,  fabsf(jerk_x));
    float combined_factor_q_x = std::max(accel_factor_q_x, jerk_factor_q_x);
    float q_x = lerp_custom(q_min_base[i], Q_MAX, combined_factor_q_x); // Q_MAX è la costante

    float accel_factor_q_y = smoothstep_custom(ACCEL_Q_THRESH_Y_LOWER, ACCEL_Q_THRESH_Y_UPPER, fabsf(ay));
    float jerk_factor_q_y  = smoothstep_custom(JERK_Q_THRESH_Y_LOWER,  JERK_Q_THRESH_Y_UPPER,  fabsf(jerk_y));
    float combined_factor_q_y = std::max(accel_factor_q_y, jerk_factor_q_y);
    float q_y = lerp_custom(q_min_base[i], Q_MAX, combined_factor_q_y); // Q_MAX è la costante

    p_x[i] += q_x;
    p_y[i] += q_y;

    constexpr float MIN_COVARIANCE_AND_R = 1e-9f; // Per evitare divisioni per zero o K instabile
    if (p_x[i] < MIN_COVARIANCE_AND_R) p_x[i] = MIN_COVARIANCE_AND_R;
    if (p_y[i] < MIN_COVARIANCE_AND_R) p_y[i] = MIN_COVARIANCE_AND_R;
    // Assumiamo che r_dynamic[i] sia sempre > 0, altrimenti clampare anche quello
    // if (r_dynamic[i] < MIN_COVARIANCE_AND_R) r_dynamic[i] = MIN_COVARIANCE_AND_R; // Se r_dynamic può diventare troppo piccolo

    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (measured_x - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (measured_y - predicted_y);

    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // --- 8. Output e Aggiornamento Stato per Ciclo Successivo ---
    mouse_input_x = static_cast<int>(x_filt[i]);
    mouse_input_y = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO  // gemini pro
// static constexpr int X_MIN = 0; // etc. come prima

// Funzione helper map_float (se non si usa Arduino.h, o per controllo divisione per zero)
// Se Arduino.h è incluso, questa funzione è già presente.
float map_float_custom(float x, float in_min, float in_max, float out_min, float out_max) {
    // Controllo per evitare divisione per zero o valori molto piccoli che potrebbero causare instabilità
    if (fabsf(in_max - in_min) < 1e-6f) { // 1e-6f è un epsilon piccolo
        return out_min; // Comportamento di default, o si potrebbe restituire (out_min + out_max) / 2.0f
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funzione helper constrain (se non si usa Arduino.h, o per generalizzare)
// Se Arduino.h è incluso, questa funzione è già presente.
template<typename T>
T constrain_custom(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

// Interpolazione lineare
static inline float lerp_custom(float a, float b, float t) {
    return a + t * (b - a);
}

// Smoothstep (Hermite cubica per transizione fluida da 0 a 1)
// x: valore corrente
// edge0: inizio della transizione
// edge1: fine della transizione
static inline float smoothstep_custom(float edge0, float edge1, float x) {
    if (edge0 == edge1) return (x >= edge0) ? 1.0f : 0.0f; // Evita divisione per zero
    float t = constrain_custom((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t); // Formula standard
}

// Funzione per mappare un valore da un range a un altro con una curva di potenza (per non linearità)
static inline float map_pow_curve(float x, float in_min, float in_max, float out_min, float out_max, float exponent) {
    if (fabsf(in_max - in_min) < 1e-6f) return out_min;
    float t = constrain_custom((x - in_min) / (in_max - in_min), 0.0f, 1.0f);
    t = powf(t, exponent);
    return lerp_custom(out_min, out_max, t);
}

// All'interno della classe FW_Common
void FW_Common::Kalman_filter(uint8_t i, int& mouse_input_x, int& mouse_input_y) {

    // --- 0. Conversione Input e Calcoli Cinematici Iniziali ---
    float current_raw_x = static_cast<float>(mouse_input_x);
    float current_raw_y = static_cast<float>(mouse_input_y);

    float vx = current_raw_x - last_x[i];
    float vy = current_raw_y - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // --- 1. Smorzamento Adattivo della Velocità (Nuova Logica) ---
    // Questa sezione sostituisce "Smorzamento Proporzionale alla Velocità" e "Modulazione Continua Velocità Lenta".
    // Obiettivo: smorzare i tremori a basse velocità, mantenere reattività ad alte velocità.
    //           Il fattore di smorzamento (damping_factor) è ciò per cui la velocità viene moltiplicata.
    //           Un fattore < 1 smorza, un fattore = 1 non ha effetto.

    constexpr float SPEED_LOW_THRESHOLD = 1.0f;      // Sotto questa velocità, lo smorzamento è massimo (per tremori)
    constexpr float SPEED_HIGH_THRESHOLD = 50.0f;    // Sopra questa velocità, lo smorzamento è minimo (per reattività)
                                                    // (Questi valori dipendono dalla scala delle coordinate e dalla frequenza di campionamento)
    constexpr float DAMPING_AT_LOW_SPEED = 0.70f;   // Esempio: riduce la velocità del 30% a velocità molto basse
    constexpr float DAMPING_AT_HIGH_SPEED = 0.98f;  // Esempio: riduce la velocità del 2% a velocità molto alte (quasi nullo)

    float speed_abs_x = fabsf(vx);
    float speed_abs_y = fabsf(vy);

    // Applica una transizione smooth per il fattore di smorzamento
    float damping_factor_vx = lerp_custom(DAMPING_AT_LOW_SPEED, DAMPING_AT_HIGH_SPEED, smoothstep_custom(SPEED_LOW_THRESHOLD, SPEED_HIGH_THRESHOLD, speed_abs_x));
    float damping_factor_vy = lerp_custom(DAMPING_AT_LOW_SPEED, DAMPING_AT_HIGH_SPEED, smoothstep_custom(SPEED_LOW_THRESHOLD, SPEED_HIGH_THRESHOLD, speed_abs_y));
    
    vx *= damping_factor_vx;
    vy *= damping_factor_vy;

    // --- 3. Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali (Come Prima, ma Rivediamo gli Esponenti) ---
    float dist_from_center_x_norm = fabsf(current_raw_x - X_CENTER) / HALF_WIDTH;
    float dist_from_center_y_norm = fabsf(current_raw_y - Y_CENTER) / HALF_HEIGHT;
    float dist_to_edge_x_norm = std::min(fabsf(current_raw_x - X_MIN), fabsf(current_raw_x - X_MAX)) / HALF_WIDTH;
    float dist_to_edge_y_norm = std::min(fabsf(current_raw_y - Y_MIN), fabsf(current_raw_y - Y_MAX)) / HALF_HEIGHT;

    // Fattori di stabilizzazione (1.0 = nessuna stabilizzazione aggiuntiva, <1.0 = più stabilizzazione/smorzamento)
    // EDGE_SMOOTHING è una costante globale (es. 1.5f)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Riduciamo leggermente l'aggressività degli esponenti per i bordi, specialmente per Y, per testare l'effetto sul tremolio.
    // Valori originali: X=3.5f, Y=4.5f. Prova con valori più bassi.
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 2.5f; // Valore da affinare
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 3.0f; // Valore da affinare (reso meno aggressivo)

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);


    // --- 4. Combinazione Dinamica dei Fattori di Stabilizzazione (Senza if esplicito) ---
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f; // Come prima
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;    // Come prima

    // Calcola il peso per la stabilizzazione d'angolo in modo continuo
    // Se dist_to_edge > THRESHOLD, normalized_dist_in_transition_eff sarà >= 1, factor_for_pow sarà 0.
    float normalized_dist_in_transition_eff_x = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD;
    float factor_for_pow_x = constrain_custom(1.0f - normalized_dist_in_transition_eff_x, 0.0f, 1.0f);
    float weight_for_corner_x = powf(factor_for_pow_x, EXP_SMOOTHNESS_FACTOR);
    float combined_stabilization_factor_x = lerp_custom(stabilization_center_x, stabilization_corner_x, weight_for_corner_x);

    float normalized_dist_in_transition_eff_y = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
    float factor_for_pow_y = constrain_custom(1.0f - normalized_dist_in_transition_eff_y, 0.0f, 1.0f);
    float weight_for_corner_y = powf(factor_for_pow_y, EXP_SMOOTHNESS_FACTOR);
    float combined_stabilization_factor_y = lerp_custom(stabilization_center_y, stabilization_corner_y, weight_for_corner_y);
    
    // MIN_PRECISION_FACTOR è una costante globale (es. 0.9f)
    combined_stabilization_factor_x = constrain_custom(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f);
    combined_stabilization_factor_y = constrain_custom(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f);

    // Applica i fattori di stabilizzazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- 5. Gestione della Dead Zone e Soglia di Stazionarietà (Logica simile, ma i fattori sono già più fluidi) ---
    // (La logica interna è già abbastanza fluida con map_float e powf, la manteniamo per ora)
    float max_dist_to_edge_norm = std::max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    float t_deadzone = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_deadzone = t_deadzone * t_deadzone;
    float current_dead_zone_multiplier = map_float_custom(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);

    constexpr float STATIONARY_THRESHOLD_EDGE_COUNT = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER_COUNT = 50.0f;
    float t_stationary = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_stationary = t_stationary * t_stationary;
    float current_stationary_threshold_count = map_float_custom(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER_COUNT, STATIONARY_THRESHOLD_EDGE_COUNT);

    if (fabsf(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) &&
        fabsf(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) {
        stationary_counter[i]++;
        if (stationary_counter[i] > static_cast<int>(current_stationary_threshold_count)) {
            mouse_input_x = static_cast<int>(last_x[i]);
            mouse_input_y = static_cast<int>(last_y[i]);
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; // Questo reset è ancora utile
    }

    // --- 6. Pre-filtraggio della Misura (Input per Kalman) (Come prima) ---
    float measured_x = (current_raw_x + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f;
    float measured_y = (current_raw_y + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f;

    // --- 7. Core del Filtro di Kalman ---
    // Pesi per la predizione (Come prima, ma vx/ax sono già stati modificati in modo più fluido)
    float prediction_weight_v = constrain_custom(fabsf(vx) / X_MAX, 0.1f, 0.8f);
    constexpr float ACCEL_NORMALIZATION_FACTOR = X_MAX * 0.01f;
    float prediction_weight_a = constrain_custom(fabsf(ax) / ACCEL_NORMALIZATION_FACTOR, 0.2f, 0.9f);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q) in modo fluido
    // q_max e q_min_base[i] sono globali/statiche.
    // Le soglie ora definiscono il range di transizione per smoothstep.
    // Esempio: transizione tra 80% e 120% della soglia originale.
    constexpr float ACCEL_Q_THRESH_X_LOWER = (X_MAX * 0.002f) * 0.8f;
    constexpr float ACCEL_Q_THRESH_X_UPPER = (X_MAX * 0.002f) * 1.2f;
    constexpr float JERK_Q_THRESH_X_LOWER  = (X_MAX * 0.001f) * 0.8f;
    constexpr float JERK_Q_THRESH_X_UPPER  = (X_MAX * 0.001f) * 1.2f;
    // Simili per Y
    constexpr float ACCEL_Q_THRESH_Y_LOWER = (Y_MAX * 0.002f) * 0.8f;
    constexpr float ACCEL_Q_THRESH_Y_UPPER = (Y_MAX * 0.002f) * 1.2f;
    constexpr float JERK_Q_THRESH_Y_LOWER  = (Y_MAX * 0.001f) * 0.8f;
    constexpr float JERK_Q_THRESH_Y_UPPER  = (Y_MAX * 0.001f) * 1.2f;

    float accel_factor_q_x = smoothstep_custom(ACCEL_Q_THRESH_X_LOWER, ACCEL_Q_THRESH_X_UPPER, fabsf(ax));
    float jerk_factor_q_x  = smoothstep_custom(JERK_Q_THRESH_X_LOWER,  JERK_Q_THRESH_X_UPPER,  fabsf(jerk_x));
    float combined_factor_q_x = std::max(accel_factor_q_x, jerk_factor_q_x); // Il "più forte" dei due segnali guida Q
    float q_x = lerp_custom(q_min_base[i], q_max, combined_factor_q_x);

    float accel_factor_q_y = smoothstep_custom(ACCEL_Q_THRESH_Y_LOWER, ACCEL_Q_THRESH_Y_UPPER, fabsf(ay));
    float jerk_factor_q_y  = smoothstep_custom(JERK_Q_THRESH_Y_LOWER,  JERK_Q_THRESH_Y_UPPER,  fabsf(jerk_y));
    float combined_factor_q_y = std::max(accel_factor_q_y, jerk_factor_q_y);
    float q_y = lerp_custom(q_min_base[i], q_max, combined_factor_q_y);

    p_x[i] += q_x;
    p_y[i] += q_y;

    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (measured_x - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (measured_y - predicted_y);

    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // --- 8. Output e Aggiornamento Stato per Ciclo Successivo (Come prima) ---
    mouse_input_x = static_cast<int>(x_filt[i]);
    mouse_input_y = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx; // vx, vy, ax, ay sono quelli modificati/smorzati
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x; // jerk è quello originale
    last_jerk_y[i] = jerk_y;
}

#endif // commento


//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO // da gemini test3 riorganizzato
// Funzione helper map_float (se non si usa Arduino.h, o per controllo divisione per zero)
// Se Arduino.h è incluso, questa funzione è già presente.
float map_float_custom(float x, float in_min, float in_max, float out_min, float out_max) {
    // Controllo per evitare divisione per zero o valori molto piccoli che potrebbero causare instabilità
    if (fabsf(in_max - in_min) < 1e-6f) { // 1e-6f è un epsilon piccolo
        return out_min; // Comportamento di default, o si potrebbe restituire (out_min + out_max) / 2.0f
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funzione helper constrain (se non si usa Arduino.h, o per generalizzare)
// Se Arduino.h è incluso, questa funzione è già presente.
template<typename T>
T constrain_custom(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}


// Assumendo che FW_Common sia una classe
// class FW_Common {
// public:
//    void Kalman_filter(uint8_t i, int& mouse_input_x, int& mouse_input_y);
// };

void FW_Common::Kalman_filter(uint8_t i, int& mouse_input_x, int& mouse_input_y) {

    // --- 0. Conversione Input e Calcoli Cinematici Iniziali ---
    // Converte le coordinate di input (intere) in float per i calcoli.
    float current_raw_x = static_cast<float>(mouse_input_x);
    float current_raw_y = static_cast<float>(mouse_input_y);

    // Calcola velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y).
    // Nota: last_x[i], last_y[i] sono le posizioni filtrate del ciclo precedente.
    // vx, vy qui rappresentano il cambiamento dalla *posizione filtrata precedente* alla *posizione grezza attuale*.
    float vx = current_raw_x - last_x[i];
    float vy = current_raw_y - last_y[i];
    float ax = vx - last_vx[i];       // Accelerazione basata sulla variazione della velocità calcolata sopra
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];   // Jerk basato sulla variazione dell'accelerazione
    float jerk_y = ay - last_ay[i];

    // --- 1. Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica una maggiore stabilizzazione ai movimenti lenti e minore a quelli rapidi.
    constexpr float TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float TREMOR_MIN_FACTOR_Y = 0.25f; // Fattore Y può essere diverso come nell'originale

    // Normalizza la velocità rispetto alla dimensione massima dell'asse (X_MAX, Y_MAX).
    // Se X_MIN/Y_MIN non fossero 0, si userebbe (X_MAX - X_MIN).
    float tremor_reduction_factor_x = constrain_custom(1.0f - fabsf(vx) / X_MAX, TREMOR_MIN_FACTOR_X, 1.0f);
    float tremor_reduction_factor_y = constrain_custom(1.0f - fabsf(vy) / Y_MAX, TREMOR_MIN_FACTOR_Y, 1.0f);
    vx *= tremor_reduction_factor_x; // Applica lo smorzamento alla velocità
    vy *= tremor_reduction_factor_y;

    // --- 2. Modulazione Continua Velocità Lenta ---
    // Aumenta ulteriormente lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f;       // Soglia di velocità per attivare questo smorzamento
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; // Fattore di smorzamento massimo a velocità zero

    if (fabsf(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabsf(vx) / LOW_SPEED_THRESHOLD); // Normalizza velocità da 0 (alla soglia) a 1 (a zero velocità)
        // Mappa il fattore di smorzamento: 1.0 (nessuno smorzamento) a LOW_SPEED_DAMPING_FACTOR (massimo smorzamento)
        float current_low_speed_damping_x = map_float_custom(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }
    if (fabsf(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabsf(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float_custom(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- 3. Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Le distanze sono calcolate sull'input GREZZO corrente (`current_raw_x`, `current_raw_y`).
    // HALF_WIDTH e HALF_HEIGHT sono (MAX - MIN) / 2.0f. X_CENTER e Y_CENTER sono i centri.
    float dist_from_center_x_norm = fabsf(current_raw_x - X_CENTER) / HALF_WIDTH;
    float dist_from_center_y_norm = fabsf(current_raw_y - Y_CENTER) / HALF_HEIGHT;

    // Distanza normalizzata dal bordo più vicino.
    float dist_to_edge_x_norm = std::min(fabsf(current_raw_x - X_MIN), fabsf(current_raw_x - X_MAX)) / HALF_WIDTH;
    float dist_to_edge_y_norm = std::min(fabsf(current_raw_y - Y_MIN), fabsf(current_raw_y - Y_MAX)) / HALF_HEIGHT;

    // Fattore di stabilizzazione basato sulla distanza dal centro (meno stabilizzazione lontano dal centro).
    // EDGE_SMOOTHING è una costante globale.
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattore di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi).
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f;
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; // Esponenti diversi per X e Y come nell'originale

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- 4. Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella ai bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f; // Soglia normalizzata per la transizione
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;    // Fattore per la dolcezza della transizione

    float combined_stabilization_factor_x = stabilization_center_x;
    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD;
        // Il peso per 'corner' aumenta man mano che ci si avvicina al bordo (normalized_dist_in_transition -> 0)
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    float combined_stabilization_factor_y = stabilization_center_y;
    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    // Assicura che il fattore di stabilizzazione combinato rimanga entro limiti ragionevoli.
    // MIN_PRECISION_FACTOR è una costante globale.
    combined_stabilization_factor_x = constrain_custom(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f);
    combined_stabilization_factor_y = constrain_custom(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f);

    // Applica i fattori di stabilizzazione combinati alla velocità (già smorzata) e all'accelerazione (originale).
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x; // ax e ay sono ora modificati da questo fattore
    ay *= combined_stabilization_factor_y;

    // --- 5. Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo.
    float max_dist_to_edge_norm = std::max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    // Calcola il moltiplicatore corrente per la dead zone con una transizione dolce.
    float t_deadzone = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_deadzone = t_deadzone * t_deadzone; // Sostituito powf(t_deadzone, 2.0f)
    float current_dead_zone_multiplier = map_float_custom(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);

    // Calcola la soglia di stazionarietà corrente con una transizione dolce.
    constexpr float STATIONARY_THRESHOLD_EDGE_COUNT = 75.0f;   // Conteggio, quindi float per calcoli intermedi va bene
    constexpr float STATIONARY_THRESHOLD_CENTER_COUNT = 50.0f;
    float t_stationary = constrain_custom(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_stationary = t_stationary * t_stationary; // Sostituito powf(t_stationary, 2.0f)
    float current_stationary_threshold_count = map_float_custom(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER_COUNT, STATIONARY_THRESHOLD_EDGE_COUNT);

    // Applica la Dead Zone: se la velocità (già smorzata e stabilizzata) è molto bassa.
    // DEAD_ZONE_X e DEAD_ZONE_Y sono costanti globali.
    if (fabsf(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) &&
        fabsf(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) {
        stationary_counter[i]++;
        if (stationary_counter[i] > static_cast<int>(current_stationary_threshold_count)) {
            // Movimento considerato stazionario: blocca l'output sulla posizione filtrata precedente.
            mouse_input_x = static_cast<int>(last_x[i]);
            mouse_input_y = static_cast<int>(last_y[i]);
            // Non è necessario aggiornare last_vx, etc., qui, perché si esce.
            // last_x/y già contengono la posizione di "blocco".
            return; // Esce dalla funzione anticipatamente.
        }
    } else {
        stationary_counter[i] = 0;    // Resetta il contatore se c'è movimento.
        q_min_base[i] = 0.005f;       // Resetta la stima del rumore di processo base (q_min_base) se c'è movimento.
    }

    // --- 6. Pre-filtraggio della Misura (Input per Kalman) ---
    // L'input grezzo `current_raw_x/y` viene combinato con lo stato filtrato precedente (`x_filt`)
    // e la posizione filtrata del ciclo ancora prima (`last_x`) per creare una "misura" più stabile.
    float measured_x = (current_raw_x + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f;
    float measured_y = (current_raw_y + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f;

    // --- 7. Core del Filtro di Kalman ---
    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione (modificate).
    float prediction_weight_v = constrain_custom(fabsf(vx) / X_MAX, 0.1f, 0.8f); // Normalizzazione con X_MAX (o larghezza totale)
    
    constexpr float ACCEL_NORMALIZATION_FACTOR = X_MAX * 0.01f; // Fattore di normalizzazione per l'accelerazione
    float prediction_weight_a = constrain_custom(fabsf(ax) / ACCEL_NORMALIZATION_FACTOR, 0.2f, 0.9f);
    // La variabile `prediction_weight_j` era calcolata ma non usata, quindi è stata rimossa.

    // Predizione del prossimo stato: x_filt[i] è lo stato filtrato del ciclo precedente.
    // vx, ax sono le velocità/accelerazioni calcolate e modificate in questo ciclo.
    // Questa è una forma semplificata di predizione, non usa esplicitamente dt.
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    // ax, ay qui sono quelli modificati dai fattori di stabilizzazione.
    // jerk_x, jerk_y sono quelli "originali" calcolati all'inizio.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;
    // q_max e q_min_base[i] sono globali/statiche.
    float q_x = (fabsf(ax) > ACCEL_Q_THRESHOLD_X || fabsf(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabsf(ay) > ACCEL_Q_THRESHOLD_Y || fabsf(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore di predizione (P_k_minus = P_k-1_plus + Q)
    // p_x[i] e p_y[i] sono le covarianze dell'errore di stima del ciclo precedente (P_k-1_plus).
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    // r_dynamic[i] è il rumore di misura (R). Se non è realmente dinamico, dovrebbe essere r_base.
    // Assumiamo che r_dynamic[i] sia gestito correttamente o sia equivalente a r_base.
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato usando la misura pre-filtrata (`measured_x`, `measured_y`)
    // x_k_plus = x_k_minus + K * (z_k - x_k_minus)
    x_filt[i] = predicted_x + k_x[i] * (measured_x - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (measured_y - predicted_y);

    // Aggiornamento della covarianza dell'errore di stima (P_k_plus = (1 - K) * P_k_minus)
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // --- 8. Output e Aggiornamento Stato per Ciclo Successivo ---
    // Assegna i valori filtrati all'output (parametri passati per riferimento).
    mouse_input_x = static_cast<int>(x_filt[i]);
    mouse_input_y = static_cast<int>(y_filt[i]);

    // Aggiorna i valori "last" per il prossimo ciclo di filtraggio.
    last_x[i] = x_filt[i];             // Posizione filtrata corrente diventa "last_x" per il prossimo ciclo.
    last_y[i] = y_filt[i];
    last_vx[i] = vx;                   // Velocità modificata/smorzata usata nella predizione.
    last_vy[i] = vy;
    last_ax[i] = ax;                   // Accelerazione modificata/smorzata usata nella predizione.
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;           // Jerk "originale" (calcolato all'inizio del ciclo).
    last_jerk_y[i] = jerk_y;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
// test 8 ter
/*
funziona, ma il problema del tremolio vicino ai bordi c'è ancora, ma è perlopiù presente nei movimenti verticali e lenti .. proviamo a gestire dinamicamente anche l'altro valore e rimuovere qualche if, ne vedo ancora tantissimi nel codice ... proviamo con tentivo8 quater con le modifiche che ti ho proposto

*/



// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Calcolo della velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y)
    float vx = static_cast<float>(mouseX) - last_x[i];
    float vy = static_cast<float>(mouseY) - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabs(static_cast<float>(mouseX) - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabs(static_cast<float>(mouseY) - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = min(fabs(static_cast<float>(mouseX) - X_MIN), fabs(static_cast<float>(mouseX) - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = min(fabs(static_cast<float>(mouseY) - Y_MIN), fabs(static_cast<float>(mouseY) - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione basati sulla posizione dal centro (meno stabilizzazione lontano dal centro)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    // --- INIZIO MODIFICA: Tentativo 8 ter - Affinamento modulazione logaritmica (velocità + bordo) ---
    constexpr float BASE_TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float BASE_TREMOR_MIN_FACTOR_Y = 0.25f;

    // Parametri per la modulazione in base alla distanza dal bordo (Affinamento per Tentativo 8 bis)
    // RIDUZIONE TREMOR_MIN_BOOST_EDGE e AUMENTO EDGE_TREMOR_CURVE_EXPONENT
    constexpr float TREMOR_MIN_BOOST_EDGE_X = 0.15f; // Leggermente ridotto
    constexpr float TREMOR_MIN_BOOST_EDGE_Y = 0.20f; // Leggermente ridotto
    constexpr float EDGE_TREMOR_EFFECT_THRESHOLD = 0.25f; 
    constexpr float EDGE_TREMOR_CURVE_EXPONENT = 2.5f; // Aumentato per rendere la curva più ripida vicino al bordo

    // Calcola il fattore di influenza del bordo (0 al centro, 1 al bordo)
    float t_edge_tremor_x = 0.0f;
    if (dist_to_edge_x_norm < EDGE_TREMOR_EFFECT_THRESHOLD) {
        t_edge_tremor_x = constrain(1.0f - (dist_to_edge_x_norm / EDGE_TREMOR_EFFECT_THRESHOLD), 0.0f, 1.0f);
        t_edge_tremor_x = powf(t_edge_tremor_x, EDGE_TREMOR_CURVE_EXPONENT); 
    }

    float t_edge_tremor_y = 0.0f;
    if (dist_to_edge_y_norm < EDGE_TREMOR_EFFECT_THRESHOLD) {
        t_edge_tremor_y = constrain(1.0f - (dist_to_edge_y_norm / EDGE_TREMOR_EFFECT_THRESHOLD), 0.0f, 1.0f);
        t_edge_tremor_y = powf(t_edge_tremor_y, EDGE_TREMOR_CURVE_EXPONENT); 
    }
    
    // Calcola il TREMOR_MIN_FACTOR potenziato dal bordo
    float current_tremor_min_factor_x_edge = map_float(t_edge_tremor_x, 0.0f, 1.0f, BASE_TREMOR_MIN_FACTOR_X, BASE_TREMOR_MIN_FACTOR_X + TREMOR_MIN_BOOST_EDGE_X);
    float current_tremor_min_factor_y_edge = map_float(t_edge_tremor_y, 0.0f, 1.0f, BASE_TREMOR_MIN_FACTOR_Y, BASE_TREMOR_MIN_FACTOR_Y + TREMOR_MIN_BOOST_EDGE_Y);


    // Parametri per la modulazione in base alla velocità (dal Tentativo 8 bis)
    constexpr float TREMOR_VEL_SMOOTH_FACTOR = 0.7f; 
    constexpr float TREMOR_VEL_MIN_EFFECT = 0.0f; 
    constexpr float TREMOR_VEL_MAX_EFFECT = 0.25f; 

    // Normalizza la velocità assoluta (da 0 a 1)
    float normalized_abs_vx = fabs(vx) / X_MAX;
    float normalized_abs_vy = fabs(vy) / Y_MAX;

    // Calcola il fattore di smorzamento extra dalla velocità (più alto per velocità basse)
    float vel_boost_factor_x = powf(max(0.0f, 1.0f - normalized_abs_vx), TREMOR_VEL_SMOOTH_FACTOR);
    float vel_boost_factor_y = powf(max(0.0f, 1.0f - normalized_abs_vy), TREMOR_VEL_SMOOTH_FACTOR);

    // Mappa questo fattore in un intervallo di "boost" da applicare al TREMOR_MIN_FACTOR
    float current_tremor_vel_boost_x = map_float(vel_boost_factor_x, 0.0f, 1.0f, TREMOR_VEL_MIN_EFFECT, TREMOR_VEL_MAX_EFFECT);
    float current_tremor_vel_boost_y = map_float(vel_boost_factor_y, 0.0f, 1.0f, TREMOR_VEL_MIN_EFFECT, TREMOR_VEL_MAX_EFFECT);

    // Combina i due effetti: il base + il boost del bordo + il boost della velocità
    float final_tremor_min_factor_x = current_tremor_min_factor_x_edge + current_tremor_vel_boost_x;
    float final_tremor_min_factor_y = current_tremor_min_factor_y_edge + current_tremor_vel_boost_y;

    // Assicurati che non superi mai 1.0f (non invertire il movimento)
    final_tremor_min_factor_x = constrain(final_tremor_min_factor_x, BASE_TREMOR_MIN_FACTOR_X, 1.0f);
    final_tremor_min_factor_y = constrain(final_tremor_min_factor_y, BASE_TREMOR_MIN_FACTOR_Y, 1.0f);


    float tremor_reduction_factor_x = constrain(1.0f - fabs(vx) / X_MAX, final_tremor_min_factor_x, 1.0f); 
    float tremor_reduction_factor_y = constrain(1.0f - fabs(vy) / Y_MAX, final_tremor_min_factor_y, 1.0f); 
    // --- FINE MODIFICA ---
    
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    // Aumenta lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // Moltiplicatore della dead zone: più grande al bordo, più piccolo al centro (Tentativo 3)
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    float t_deadzone = constrain(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_deadzone = powf(t_deadzone, 2.0f); 
    float current_dead_zone_multiplier = map_float(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);

    // Soglia di stazionarietà: più alta al bordo (più tempo per bloccarsi), più bassa al centro (Tentativo 1)
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f;
    float t_stationary = constrain(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_stationary = powf(t_stationary, 2.0f); 
    float current_stationary_threshold = map_float(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE);

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = static_cast<int>(last_x[i]);
            mouseY = static_cast<int>(last_y[i]);
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; 
    }

    // --- Filtraggio e Predizione (Core del Filtro di Kalman) ---
    // Applica una media ponderata per un primo smussamento dell'input grezzo.
    mouseX = static_cast<int>((static_cast<float>(mouseX) + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((static_cast<float>(mouseY) + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione attuali.
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1f, 0.8f);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01f), 0.2f, 0.9f);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005f), 0.01f, 0.3f); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;

    float q_x = (fabs(ax) > ACCEL_Q_THRESHOLD_X || fabs(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > ACCEL_Q_THRESHOLD_Y || fabs(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (static_cast<float>(mouseX) - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (static_cast<float>(mouseY) - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO // test 8 quater .. errori
// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Calcolo della velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y)
    float vx = static_cast<float>(mouseX) - last_x[i];
    float vy = static_cast<float>(mouseY) - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabs(static_cast<float>(mouseX) - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabs(static_cast<float>(mouseY) - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = fmin(fabs(static_cast<float>(mouseX) - X_MIN), fabs(static_cast<float>(mouseX) - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = fmin(fabs(static_cast<float>(mouseY) - Y_MIN), fabs(static_cast<float>(mouseY) - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione basati sulla posizione dal centro (meno stabilizzazione lontano dal centro)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    constexpr float BASE_TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float BASE_TREMOR_MIN_FACTOR_Y = 0.25f;

    // Parametri per la modulazione in base alla distanza dal bordo (Affinamento per Tentativo 8 bis)
    constexpr float TREMOR_MIN_BOOST_EDGE_X = 0.15f; 
    constexpr float TREMOR_MIN_BOOST_EDGE_Y = 0.20f; 
    constexpr float EDGE_TREMOR_EFFECT_THRESHOLD = 0.25f; 
    constexpr float EDGE_TREMOR_CURVE_EXPONENT = 2.5f; 

    float t_edge_tremor_x = 0.0f;
    if (dist_to_edge_x_norm < EDGE_TREMOR_EFFECT_THRESHOLD) {
        t_edge_tremor_x = fmax(0.0f, fmin(1.0f, 1.0f - (dist_to_edge_x_norm / EDGE_TREMOR_EFFECT_THRESHOLD)));
        t_edge_tremor_x = powf(t_edge_tremor_x, EDGE_TREMOR_CURVE_EXPONENT); 
    }

    float t_edge_tremor_y = 0.0f;
    if (dist_to_edge_y_norm < EDGE_TREMOR_EFFECT_THRESHOLD) {
        t_edge_tremor_y = fmax(0.0f, fmin(1.0f, 1.0f - (dist_to_edge_y_norm / EDGE_TREMOR_EFFECT_THRESHOLD)));
        t_edge_tremor_y = powf(t_edge_tremor_y, EDGE_TREMOR_CURVE_EXPONENT); 
    }
    
    float current_tremor_min_factor_x_edge = map_float(t_edge_tremor_x, 0.0f, 1.0f, BASE_TREMOR_MIN_FACTOR_X, BASE_TREMOR_MIN_FACTOR_X + TREMOR_MIN_BOOST_EDGE_X);
    float current_tremor_min_factor_y_edge = map_float(t_edge_tremor_y, 0.0f, 1.0f, BASE_TREMOR_MIN_FACTOR_Y, BASE_TREMOR_MIN_FACTOR_Y + TREMOR_MIN_BOOST_EDGE_Y);


    // Parametri per la modulazione in base alla velocità (dal Tentativo 8 bis)
    constexpr float TREMOR_VEL_SMOOTH_FACTOR = 0.7f; 
    constexpr float TREMOR_VEL_MIN_EFFECT = 0.0f; 
    constexpr float TREMOR_VEL_MAX_EFFECT = 0.25f; 

    float normalized_abs_vx = fabs(vx) / X_MAX;
    float normalized_abs_vy = fabs(vy) / Y_MAX;

    float vel_boost_factor_x = powf(fmax(0.0f, 1.0f - normalized_abs_vx), TREMOR_VEL_SMOOTH_FACTOR);
    float vel_boost_factor_y = powf(fmax(0.0f, 1.0f - normalized_abs_vy), TREMOR_VEL_SMOOTH_FACTOR);

    float current_tremor_vel_boost_x = map_float(vel_boost_factor_x, 0.0f, 1.0f, TREMOR_VEL_MIN_EFFECT, TREMOR_VEL_MAX_EFFECT);
    float current_tremor_vel_boost_y = map_float(vel_boost_factor_y, 0.0f, 1.0f, TREMOR_VEL_MIN_EFFECT, TREMOR_VEL_MAX_EFFECT);

    float final_tremor_min_factor_x = current_tremor_min_factor_x_edge + current_tremor_vel_boost_x;
    float final_tremor_min_factor_y = current_tremor_min_factor_y_edge + current_tremor_vel_boost_y;

    final_tremor_min_factor_x = fmax(BASE_TREMOR_MIN_FACTOR_X, fmin(1.0f, final_tremor_min_factor_x));
    final_tremor_min_factor_y = fmax(BASE_TREMOR_MIN_FACTOR_Y, fmin(1.0f, final_tremor_min_factor_y));


    float tremor_reduction_factor_x = fmax(final_tremor_min_factor_x, fmin(1.0f, 1.0f - fabs(vx) / X_MAX)); 
    float tremor_reduction_factor_y = fmax(final_tremor_min_factor_y, fmin(1.0f, 1.0f - fabs(vy) / Y_MAX)); 
    
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- INIZIO MODIFICA: Tentativo 8 quater - Dinamizzazione di LOW_SPEED_DAMPING_FACTOR e rimozione if ---
    // Parametri per la modulazione dinamica di LOW_SPEED_DAMPING_FACTOR
    constexpr float BASE_LOW_SPEED_DAMPING_FACTOR = 0.85f; // Valore base al centro
    constexpr float LOW_SPEED_DAMPING_FACTOR_EDGE_BOOST = 0.15f; // Quanto può aumentare lo smorzamento ai bordi (es. 0.85 - 0.15 = 0.70)
    constexpr float LOW_SPEED_DAMPING_VELOCITY_SCALE = 0.1f; // Quanto la velocità influenza lo smorzamento extra (più piccolo = più sensibile a basse velocità)

    // Normalizza la velocità assoluta (usiamo un range più piccolo per questa modulazione)
    float normalized_vel_for_damp_x = fmax(0.0f, fmin(1.0f, fabs(vx) / LOW_SPEED_THRESHOLD));
    float normalized_vel_for_damp_y = fmax(0.0f, fmin(1.0f, fabs(vy) / LOW_SPEED_THRESHOLD));

    // Fattore di smorzamento extra basato sulla velocità (più alto per velocità molto basse)
    float vel_damp_effect_x = powf(1.0f - normalized_vel_for_for_damp_x, LOW_SPEED_DAMPING_VELOCITY_SCALE);
    float vel_damp_effect_y = powf(1.0f - normalized_vel_for_for_damp_y, LOW_SPEED_DAMPING_VELOCITY_SCALE);
    
    // Fattore di smorzamento extra basato sulla distanza dal bordo (più alto ai bordi)
    // Usiamo lo stesso t_edge_tremor_y che è già calcolato in modo logaritmico/curvato per il bordo Y
    float edge_damp_effect_x = t_edge_tremor_x; 
    float edge_damp_effect_y = t_edge_tremor_y; 

    // Combina i due effetti: più smorzamento se lento E al bordo
    float combined_damp_effect_x = fmax(vel_damp_effect_x, edge_damp_effect_x); // Prendiamo il massimo per dare priorità all'effetto più forte
    float combined_damp_effect_y = fmax(vel_damp_effect_y, edge_damp_effect_y); // per asse y

    // Applica il boost allo smorzamento. Ricorda che un DAMPING_FACTOR più BASSO significa più smorzamento.
    float current_low_speed_damping_x = map_float(combined_damp_effect_x, 0.0f, 1.0f, BASE_LOW_SPEED_DAMPING_FACTOR, BASE_LOW_SPEED_DAMPING_FACTOR - LOW_SPEED_DAMPING_FACTOR_EDGE_BOOST);
    float current_low_speed_damping_y = map_float(combined_damp_effect_y, 0.0f, 1.0f, BASE_LOW_SPEED_DAMPING_FACTOR, BASE_LOW_SPEED_DAMPING_FACTOR - LOW_SPEED_DAMPING_FACTOR_EDGE_BOOST);

    // Vincola i valori per sicurezza
    current_low_speed_damping_x = fmax(0.0f, fmin(1.0f, current_low_speed_damping_x));
    current_low_speed_damping_y = fmax(0.0f, fmin(1.0f, current_low_speed_damping_y));

    vx *= current_low_speed_damping_x;
    vy *= current_low_speed_damping_y;
    // --- FINE MODIFICA ---

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = fmax(MIN_PRECISION_FACTOR, fmin(1.0f, combined_stabilization_factor_x)); 
    combined_stabilization_factor_y = fmax(MIN_PRECISION_FACTOR, fmin(1.0f, combined_stabilization_factor_y)); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = fmax(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // Moltiplicatore della dead zone: più grande al bordo, più piccolo al centro (Tentativo 3)
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    float t_deadzone = fmax(0.0f, fmin(1.0f, 1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD)));
    float curved_t_deadzone = powf(t_deadzone, 2.0f); 
    float current_dead_zone_multiplier = map_float(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);

    // Soglia di stazionarietà: più alta al bordo (più tempo per bloccarsi), più bassa al centro (Tentativo 1)
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f;
    float t_stationary = fmax(0.0f, fmin(1.0f, 1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD)));
    float curved_t_stationary = powf(t_stationary, 2.0f); 
    float current_stationary_threshold = map_float(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE);

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = static_cast<int>(last_x[i]);
            mouseY = static_cast<int>(last_y[i]);
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; 
    }

    // --- Filtraggio e Predizione (Core del Filtro di Kalman) ---
    // Applica una media ponderata per un primo smussamento dell'input grezzo.
    mouseX = static_cast<int>((static_cast<float>(mouseX) + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((static_cast<float>(mouseY) + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione attuali.
    float prediction_weight_v = fmax(0.1f, fmin(0.8f, fabs(vx) / X_MAX));
    float prediction_weight_a = fmax(0.2f, fmin(0.9f, fabs(ax) / (X_MAX * 0.01f)));
    float prediction_weight_j = fmax(0.01f, fmin(0.3f, fabs(jerk_x) / (X_MAX * 0.005f))); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;

    float q_x = (fabs(ax) > ACCEL_Q_THRESHOLD_X || fabs(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > ACCEL_Q_THRESHOLD_Y || fabs(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (static_cast<float>(mouseX) - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (static_cast<float>(mouseY) - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENDO // la migliore // tentativo 3
// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Calcolo della velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y)
    float vx = static_cast<float>(mouseX) - last_x[i];
    float vy = static_cast<float>(mouseY) - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    constexpr float TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float TREMOR_MIN_FACTOR_Y = 0.25f;

    float tremor_reduction_factor_x = constrain(1.0f - fabs(vx) / X_MAX, TREMOR_MIN_FACTOR_X, 1.0f); 
    float tremor_reduction_factor_y = constrain(1.0f - fabs(vy) / Y_MAX, TREMOR_MIN_FACTOR_Y, 1.0f); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    // Aumenta lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabs(static_cast<float>(mouseX) - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabs(static_cast<float>(mouseY) - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = min(fabs(static_cast<float>(mouseX) - X_MIN), fabs(static_cast<float>(mouseX) - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = min(fabs(static_cast<float>(mouseY) - Y_MIN), fabs(static_cast<float>(mouseY) - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione basati sulla posizione dal centro (meno stabilizzazione lontano dal centro)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // --- INIZIO MODIFICA: Tentativo 3 di Dinamicità Controllata - DEAD_ZONE_MULTIPLIER ---
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    // Usiamo la stessa logica di smoothing del Tentativo 1
    float t_deadzone = constrain(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_deadzone = powf(t_deadzone, 2.0f); // Curva quadratica per una transizione dolce
    float current_dead_zone_multiplier = map_float(curved_t_deadzone, 0.0f, 1.0f, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE);
    // FINE MODIFICA ---

    // Soglia di stazionarietà: più alta al bordo (più tempo per bloccarsi), più bassa al centro
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f;
    float t_stationary = constrain(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    float curved_t_stationary = powf(t_stationary, 2.0f); 
    float current_stationary_threshold = map_float(curved_t_stationary, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE);

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = static_cast<int>(last_x[i]);
            mouseY = static_cast<int>(last_y[i]);
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; // Il q_min_base viene resettato se c'è movimento significativo
    }

    // --- Filtraggio e Predizione (Core del Filtro di Kalman) ---
    // Applica una media ponderata per un primo smussamento dell'input grezzo.
    mouseX = static_cast<int>((static_cast<float>(mouseX) + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((static_cast<float>(mouseY) + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione attuali.
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1f, 0.8f);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01f), 0.2f, 0.9f);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005f), 0.01f, 0.3f); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;

    float q_x = (fabs(ax) > ACCEL_Q_THRESHOLD_X || fabs(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > ACCEL_Q_THRESHOLD_Y || fabs(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (static_cast<float>(mouseX) - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (static_cast<float>(mouseY) - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Calcolo della velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y)
    float vx = static_cast<float>(mouseX) - last_x[i];
    float vy = static_cast<float>(mouseY) - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    constexpr float TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float TREMOR_MIN_FACTOR_Y = 0.25f;

    float tremor_reduction_factor_x = constrain(1.0f - fabs(vx) / X_MAX, TREMOR_MIN_FACTOR_X, 1.0f); 
    float tremor_reduction_factor_y = constrain(1.0f - fabs(vy) / Y_MAX, TREMOR_MIN_FACTOR_Y, 1.0f); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    // Aumenta lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabs(static_cast<float>(mouseX) - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabs(static_cast<float>(mouseY) - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = min(fabs(static_cast<float>(mouseX) - X_MIN), fabs(static_cast<float>(mouseX) - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = min(fabs(static_cast<float>(mouseY) - Y_MIN), fabs(static_cast<float>(mouseY) - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione basati sulla posizione dal centro (meno stabilizzazione lontano dal centro)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // Moltiplicatore della dead zone: più grande al bordo, più piccolo al centro
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    float current_dead_zone_multiplier = map_float(max_dist_to_edge_norm, 0.0f, EDGE_TRANSITION_THRESHOLD, DEAD_ZONE_MULTIPLIER_EDGE, DEAD_ZONE_MULTIPLIER_CENTER); 
    current_dead_zone_multiplier = constrain(current_dead_zone_multiplier, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE); 

    // --- INIZIO MODIFICA: Tentativo 1 di Dinamicità Controllata - current_stationary_threshold (Senza map_float e constrain) ---
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f;
    // useremo una curva esponenziale o logaritmica per una transizione più fluida.
    // Il fattore 't' va da 0 (lontano dal bordo) a 1 (vicino/al bordo).
    float t = constrain(1.0f - (max_dist_to_edge_norm / EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
    // Usiamo una funzione logaritmica (o simil-logaritmica) per modellare il comportamento:
    // log(x+1) è 0 per x=0 e cresce, quindi 1-log(x+1) per invertire la curva.
    // Oppure, semplicemente una power curve per una transizione più controllata.
    // Ad esempio: powf(t, exponent) -> quando t è 0, è 0; quando t è 1, è 1.
    // Per invertire, 1 - powf(1-t, exponent) -> quando t è 0, è 0; quando t è 1, è 1 (ma curva inversa).
    // O più semplicemente, una lerp (interpolazione lineare) basata su una curva smooth di t.
    // Per avere una curva logaritmica, possiamo elevare `t` a una potenza minore di 1 per un "ramp-up" veloce
    // o maggiore di 1 per un "ramp-up" lento.
    // L'esponente 2.0f produce una curva quadratica, che è già più liscia di una lineare.
    float curved_t = powf(t, 2.0f); // Usiamo una curva quadratica per una transizione più dolce
    float current_stationary_threshold = map_float(curved_t, 0.0f, 1.0f, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE);
    // --- FINE MODIFICA ---

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = static_cast<int>(last_x[i]);
            mouseY = static_cast<int>(last_y[i]);
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; // Il q_min_base viene resettato se c'è movimento significativo
    }

    // --- Filtraggio e Predizione (Core del Filtro di Kalman) ---
    // Applica una media ponderata per un primo smussamento dell'input grezzo.
    mouseX = static_cast<int>((static_cast<float>(mouseX) + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((static_cast<float>(mouseY) + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione attuali.
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1f, 0.8f);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01f), 0.2f, 0.9f);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005f), 0.01f, 0.3f); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;

    float q_x = (fabs(ax) > ACCEL_Q_THRESHOLD_X || fabs(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > ACCEL_Q_THRESHOLD_Y || fabs(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (static_cast<float>(mouseX) - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (static_cast<float>(mouseY) - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO // punto 0 pulito
// Se la funzione map_float non è usata altrove nel tuo codice e vogliamo mantenerla locale
// per il contesto del filtro, la possiamo definire all'interno.
// Se invece è una utility che usi in più punti, la manterremmo globale, ma per ora la inseriamo.

void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Lambda function per mappare i float, specifica per questo contesto.
    // Usiamo 'const' per indicare che non modifica il suo stato e 'noexcept' per sicurezza.
    const auto map_float_local = [](float x, float in_min, float in_max, float out_min, float out_max) noexcept -> float {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    // Calcolo della velocità (vx, vy), accelerazione (ax, ay) e jerk (jerk_x, jerk_y)
    float vx = static_cast<float>(mouseX) - last_x[i];
    float vy = static_cast<float>(mouseY) - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Smorzamento Proporzionale alla Velocità (Riduzione Tremori) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    constexpr float TREMOR_MIN_FACTOR_X = 0.2f;
    constexpr float TREMOR_MIN_FACTOR_Y = 0.25f;

    float tremor_reduction_factor_x = constrain(1.0f - fabs(vx) / X_MAX, TREMOR_MIN_FACTOR_X, 1.0f); 
    float tremor_reduction_factor_y = constrain(1.0f - fabs(vy) / Y_MAX, TREMOR_MIN_FACTOR_Y, 1.0f); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    // Aumenta lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float_local(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float_local(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- Calcolo Delle Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabs(static_cast<float>(mouseX) - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabs(static_cast<float>(mouseY) - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = min(fabs(static_cast<float>(mouseX) - X_MIN), fabs(static_cast<float>(mouseX) - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = min(fabs(static_cast<float>(mouseY) - Y_MIN), fabs(static_cast<float>(mouseY) - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione basati sulla posizione dal centro (meno stabilizzazione lontano dal centro)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per i bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione (Centro vs Bordi) ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    // Assicura che la stabilizzazione non scenda sotto un valore minimo (MIN_PRECISION_FACTOR)
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // Moltiplicatore della dead zone: più grande al bordo, più piccolo al centro
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    float current_dead_zone_multiplier = map_float_local(max_dist_to_edge_norm, 0.0f, EDGE_TRANSITION_THRESHOLD, DEAD_ZONE_MULTIPLIER_EDGE, DEAD_ZONE_MULTIPLIER_CENTER); 
    current_dead_zone_multiplier = constrain(current_dead_zone_multiplier, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE); 

    // Soglia di stazionarietà: più alta al bordo (più tempo per bloccarsi), più bassa al centro
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f;
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f;
    float current_stationary_threshold = map_float_local(max_dist_to_edge_norm, 0.0f, EDGE_TRANSITION_THRESHOLD, STATIONARY_THRESHOLD_EDGE, STATIONARY_THRESHOLD_CENTER); 
    current_stationary_threshold = constrain(current_stationary_threshold, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE); 

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = static_cast<int>(last_x[i]);
            mouseY = static_cast<int>(last_y[i]);
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; // Il q_min_base viene resettato se c'è movimento significativo
    }

    // --- Filtraggio e Predizione (Core del Filtro di Kalman) ---
    // Applica una media ponderata per un primo smussamento dell'input grezzo.
    mouseX = static_cast<int>((static_cast<float>(mouseX) + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((static_cast<float>(mouseY) + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione del prossimo stato, basati sulla velocità e accelerazione attuali.
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1f, 0.8f);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01f), 0.2f, 0.9f);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005f), 0.01f, 0.3f); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q): aumenta in presenza di movimenti bruschi.
    constexpr float ACCEL_Q_THRESHOLD_X = X_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_X = X_MAX * 0.001f;
    constexpr float ACCEL_Q_THRESHOLD_Y = Y_MAX * 0.002f;
    constexpr float JERK_Q_THRESHOLD_Y = Y_MAX * 0.001f;

    float q_x = (fabs(ax) > ACCEL_Q_THRESHOLD_X || fabs(jerk_x) > JERK_Q_THRESHOLD_X) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > ACCEL_Q_THRESHOLD_Y || fabs(jerk_y) > JERK_Q_THRESHOLD_Y) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (static_cast<float>(mouseX) - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (static_cast<float>(mouseY) - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
// Se la funzione map_float non è usata altrove nel tuo codice e vogliamo mantenerla locale
// per il contesto del filtro, la possiamo definire all'interno.
// Se invece è una utility che usi in più punti, la manterremmo globale, ma per ora la inseriamo.

void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {

    // Lambda function per mappare i float, specifica per questo contesto
    // È stata mantenuta esplicita qui per chiarezza, data la storia delle problematiche.
    // In un codice finale si potrebbe valutare di rimuoverla se la funzione map standard è sufficiente.
    auto map_float_local = [](float x, float in_min, float in_max, float out_min, float out_max) -> float {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    // Variabili di stato del movimento
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- Smorzamento Proporzionale alla Velocità (Tremor Reduction) ---
    // Applica più stabilizzazione a movimenti lenti, meno a rapidi.
    constexpr float TREMOR_MIN_FACTOR_X = 0.2f;  // Valore minimo per X
    constexpr float TREMOR_MIN_FACTOR_Y = 0.25f; // Valore minimo per Y

    float tremor_reduction_factor_x = constrain(1.0f - fabsf(vx) / X_MAX, TREMOR_MIN_FACTOR_X, 1.0f); 
    float tremor_reduction_factor_y = constrain(1.0f - fabsf(vy) / Y_MAX, TREMOR_MIN_FACTOR_Y, 1.0f); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    // Aumenta lo smorzamento per movimenti estremamente lenti.
    constexpr float LOW_SPEED_THRESHOLD = 2.0f; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85f; 

    if (fabsf(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0f - (fabsf(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float_local(normalized_low_vx, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabsf(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0f - (fabsf(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float_local(normalized_low_vy, 0.0f, 1.0f, 1.0f, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }

    // --- Calcolo Distanze e Fattori di Stabilizzazione Ambientali ---
    // Normalizza la distanza dal centro e dal bordo (0.0 a 1.0)
    float dist_from_center_x_norm = fabsf(mouseX - X_CENTER) / (X_MAX / 2.0f);
    float dist_from_center_y_norm = fabsf(mouseY - Y_CENTER) / (Y_MAX / 2.0f);

    float dist_to_edge_x_norm = min(fabsf(mouseX - X_MIN), fabsf(mouseX - X_MAX)) / (X_MAX / 2.0f);
    float dist_to_edge_y_norm = min(fabsf(mouseY - Y_MIN), fabsf(mouseY - Y_MAX)) / (Y_MAX / 2.0f);

    // Fattori di stabilizzazione: più ci si allontana dal centro, meno stabilizzazione (curva di smussamento)
    float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattori di stabilizzazione per gli angoli/bordi (aumenta la stabilizzazione vicino ai bordi)
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5f; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5f; 

    float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // --- Combinazione Dinamica dei Fattori di Stabilizzazione ---
    // Determina il peso tra la stabilizzazione centrale e quella sui bordi.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f; // Soglia dove inizia la transizione ai bordi
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;     // Fattore per la curva di transizione

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_x = (1.0f - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = powf(1.0f - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        combined_stabilization_factor_y = (1.0f - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    // Assicura che la stabilizzazione non scenda sotto un valore minimo (MIN_PRECISION_FACTOR)
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0f); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0f); 

    // Applica i fattori di stabilizzazione a velocità e accelerazione
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- Gestione della Dead Zone e della Soglia di Stazionarietà Dinamica ---
    // Questi valori si adattano alla distanza dal bordo per una migliore gestione.
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    // Moltiplicatore della dead zone: più grande al bordo, più piccolo al centro
    constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f; // Moltiplicatore al bordo
    constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f; // Moltiplicatore al centro
    float current_dead_zone_multiplier = map_float_local(max_dist_to_edge_norm, 0.0f, EDGE_TRANSITION_THRESHOLD, DEAD_ZONE_MULTIPLIER_EDGE, DEAD_ZONE_MULTIPLIER_CENTER); 
    current_dead_zone_multiplier = constrain(current_dead_zone_multiplier, DEAD_ZONE_MULTIPLIER_CENTER, DEAD_ZONE_MULTIPLIER_EDGE); 

    // Soglia di stazionarietà: più alta al bordo (più tempo per bloccarsi), più bassa al centro
    constexpr float STATIONARY_THRESHOLD_EDGE = 75.0f; // Soglia al bordo
    constexpr float STATIONARY_THRESHOLD_CENTER = 50.0f; // Soglia al centro
    float current_stationary_threshold = map_float_local(max_dist_to_edge_norm, 0.0f, EDGE_TRANSITION_THRESHOLD, STATIONARY_THRESHOLD_EDGE, STATIONARY_THRESHOLD_CENTER); 
    current_stationary_threshold = constrain(current_stationary_threshold, STATIONARY_THRESHOLD_CENTER, STATIONARY_THRESHOLD_EDGE); 

    if (fabsf(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabsf(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = last_x[i];
            mouseY = last_y[i];
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005f; // Il q_min_base viene resettato se c'è movimento significativo
    }

    // --- Filtraggio e Predizione (Kalman Filter Core) ---
    // Media ponderata dell'input corrente con i valori filtrati precedenti
    mouseX = static_cast<int>((mouseX + last_x[i] * 2.0f + x_filt[i] * 3.0f) / 6.0f);
    mouseY = static_cast<int>((mouseY + last_y[i] * 2.0f + y_filt[i] * 3.0f) / 6.0f);

    // Pesi per la predizione basati sulla velocità e accelerazione attuali
    float prediction_weight_v = constrain(fabsf(vx) / X_MAX, 0.1f, 0.8f);
    float prediction_weight_a = constrain(fabsf(ax) / (X_MAX * 0.01f), 0.2f, 0.9f);
    // Il peso per il jerk è stato lasciato per completezza, ma potrebbe essere rimosso se non significativo
    float prediction_weight_j = constrain(fabsf(jerk_x) / (X_MAX * 0.005f), 0.01f, 0.3f); 

    // Predizione del prossimo stato
    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Determinazione del rumore di processo (Q)
    // q_max è applicato in presenza di accelerazione o jerk significativi
    float q_x = (fabsf(ax) > X_MAX * 0.002f || fabsf(jerk_x) > X_MAX * 0.001f) ? q_max : q_min_base[i];
    float q_y = (fabsf(ay) > Y_MAX * 0.002f || fabsf(jerk_y) > Y_MAX * 0.001f) ? q_max : q_min_base[i];

    // Aggiornamento della covarianza dell'errore (P)
    p_x[i] += q_x;
    p_y[i] += q_y;

    // Calcolo del guadagno di Kalman (K)
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    // Aggiornamento dello stato filtrato
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    // Aggiornamento della covarianza dell'errore per il prossimo ciclo
    p_x[i] *= (1.0f - k_x[i]);
    p_y[i] *= (1.0f - k_y[i]);

    // Assegna i valori filtrati all'output
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Aggiorna i valori precedenti per il prossimo ciclo
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- Modulazione Continua Velocità Lenta ---
    constexpr float LOW_SPEED_THRESHOLD = 2.0; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0 - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float(normalized_low_vx, 0.0, 1.0, 1.0, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0 - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float(normalized_low_vy, 0.0, 1.0, 1.0, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }


    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22; 
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5; 

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0); 
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0); 

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- DEAD_ZONE e Stationary Threshold Dinamici e Continui (come nella versione base funzionante) ---
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    float current_dead_zone_multiplier = map_float(max_dist_to_edge_norm, 0.0, EDGE_TRANSITION_THRESHOLD, 1.5, 1.2); 
    current_dead_zone_multiplier = constrain(current_dead_zone_multiplier, 1.2, 1.5); 

    float current_stationary_threshold = map_float(max_dist_to_edge_norm, 0.0, EDGE_TRANSITION_THRESHOLD, 75.0, 50.0); 
    current_stationary_threshold = constrain(current_stationary_threshold, 50.0, 75.0); 

    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = last_x[i];
            mouseY = last_y[i];
            return; 
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENDO // nuovo tipo implementazione
// INSERISCI QUESTA FUNZIONE ALL'INIZIO DEL TUO FILE .INO O .CPP, FUORI DA QUALSIASI CLASSE O FUNZIONE.
// Ad esempio, subito dopo le tue #include o le definizioni delle costanti globali.
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // --- INIZIO: Ottimizzazione 25 (Rev. C) - Modulazione Continua Velocità Lenta ---
    // Questi if sono NECESSARI per evitare divisioni per zero o comportamenti indefiniti
    // se la velocità supera LOW_SPEED_THRESHOLD.
    // L'interno degli if è comunque progressivo.
    constexpr float LOW_SPEED_THRESHOLD = 2.0; 
    constexpr float LOW_SPEED_DAMPING_FACTOR = 0.85; 

    if (fabs(vx) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vx = 1.0 - (fabs(vx) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_x = map_float(normalized_low_vx, 0.0, 1.0, 1.0, LOW_SPEED_DAMPING_FACTOR);
        vx *= current_low_speed_damping_x;
    }

    if (fabs(vy) < LOW_SPEED_THRESHOLD) {
        float normalized_low_vy = 1.0 - (fabs(vy) / LOW_SPEED_THRESHOLD);
        float current_low_speed_damping_y = map_float(normalized_low_vy, 0.0, 1.0, 1.0, LOW_SPEED_DAMPING_FACTOR);
        vy *= current_low_speed_damping_y;
    }
    // --- FINE: Ottimizzazione 25 (Rev. C) - Modulazione Continua Velocità Lenta ---


    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22; 
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5; 

    // Questi if sono NECESSARI perché stabilization_corner e la sua interpolazione
    // si applicano solo in una specifica zona vicino al bordo.
    // L'interno degli if è comunque progressivo.
    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD; // ERRORE CORRETTO QUI
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;

    // --- INIZIO: Ottimizzazione 25 (Rev. C) - DEAD_ZONE e Stationary Threshold Dinamici e Continui ---
    float max_dist_to_edge_norm = max(dist_to_edge_x_norm, dist_to_edge_y_norm);
    
    float current_dead_zone_multiplier = map_float(max_dist_to_edge_norm, 0.0, EDGE_TRANSITION_THRESHOLD, 1.5, 1.2);
    current_dead_zone_multiplier = constrain(current_dead_zone_multiplier, 1.2, 1.5); 

    float current_stationary_threshold = map_float(max_dist_to_edge_norm, 0.0, EDGE_TRANSITION_THRESHOLD, 75.0, 50.0);
    current_stationary_threshold = constrain(current_stationary_threshold, 50.0, 75.0); 

    // Questo if è NECESSARIO per implementare la logica di "blocco" quando il cursore è fermo
    // all'interno della dead zone. Non è un cambio "continuo" in sé, ma la sua attivazione
    // è gestita da parametri (dead_zone_multiplier, stationary_threshold) che sono continui.
    if (fabs(vx) < (DEAD_ZONE_X * current_dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * current_dead_zone_multiplier)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > current_stationary_threshold) { 
            mouseX = last_x[i];
            mouseY = last_y[i];
            return; // Uscita anticipata per bloccare il cursore
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }
    // --- FINE: Ottimizzazione 25 (Rev. C) ---

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i]; 
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida per movimenti lenti
    if (fabs(vx) < 1.5) vx *= 0.90; 
    if (fabs(vy) < 1.5) vy *= 0.85; 

    // --- INIZIO: Ottimizzazione 24 - Affinamento Transizione da Fermo al Movimento sui Bordi ---
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22; 
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5; 

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 24 ---

    // Auto-selezione tra modalità adattiva e blocco
    // Modifichiamo leggermente la logica del stationary_counter per essere meno reattiva allo sblocco
    // quando siamo molto vicini al bordo.
    float dead_zone_multiplier = 1.2;
    // Se siamo molto vicini al bordo, allarghiamo leggermente la dead zone effettiva per i movimenti minimi
    // per non sbloccare troppo bruscamente.
    if (dist_to_edge_x_norm < 0.1 || dist_to_edge_y_norm < 0.1) { // 0.1 è circa il 10% dal bordo
        dead_zone_multiplier = 1.5; // Aumentiamo la tolleranza al movimento minimo vicino al bordo
    }

    if (fabs(vx) < (DEAD_ZONE_X * dead_zone_multiplier) && fabs(vy) < (DEAD_ZONE_Y * dead_zone_multiplier)) { 
        stationary_counter[i]++;
        // Aumentiamo la soglia di blocco quando siamo ai bordi per prevenire sblocchi troppo sensibili
        int stationary_threshold = 50; 
        if (dist_to_edge_x_norm < 0.1 || dist_to_edge_y_norm < 0.1) {
            stationary_threshold = 75; // Richiede più frame fermi per bloccarsi al bordo
        }

        if (stationary_counter[i] > stationary_threshold) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i]; // ERRORE CORRETTO: era 'mouseY[i]'
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida per movimenti lenti
    if (fabs(vx) < 1.5) vx *= 0.90; 
    if (fabs(vy) < 1.5) vy *= 0.85; 

    // --- INIZIO: Ottimizzazione 23 (Rev. A) - Transizione del Bordo Estremo Affinata ---
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Aumentiamo ancora la soglia per estendere l'area di transizione.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.22; // Aumentato da 0.18 a 0.22

    // Manteniamo l'EXP_SMOOTHNESS_FACTOR che ha dato buon risultato nell'ultima versione.
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5; 

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 23 (Rev. A) ---

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < (DEAD_ZONE_X * 1.2) && fabs(vy) < (DEAD_ZONE_Y * 1.2)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida per movimenti lenti
    if (fabs(vx) < 1.5) vx *= 0.90; 
    if (fabs(vy) < 1.5) vy *= 0.85; 

    // --- INIZIO: Ottimizzazione 22 - Ritorno a EXP_SMOOTHNESS_FACTOR più Dolce e Controllo Soglia ---
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Riportiamo la soglia a un valore intermedio o leggermente più basso, per stringere l'area.
    // L'effetto "magnetismo" se la zona è troppo ampia e la curva troppo ripida.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.18; // Riportato da 0.20 a 0.18

    // Riduciamo EXP_SMOOTHNESS_FACTOR per rendere la curva di transizione meno "brutale".
    // L'obiettivo è una transizione più dolce e graduale.
    constexpr float EXP_SMOOTHNESS_FACTOR = 2.5; // Ritorno a un valore più dolce, era 3.0, poi 5.0

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 22 ---

    // Auto-selezione tra modalità adattiva e blocco
    // Manteniamo il moltiplicatore della DEAD_ZONE a 1.2 per la stabilità da fermo.
    if (fabs(vx) < (DEAD_ZONE_X * 1.2) && fabs(vy) < (DEAD_ZONE_Y * 1.2)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef COMMENTO
// Non dovresti aver bisogno di includere nulla di specifico per pow() su ESP32,
// ma se dovessi riscontrare errori, prova a includere <cmath> o <math.h> all'inizio del tuo file .ino o .cpp
// #include <cmath> 

void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Definizione di PI_HALF (o PI_OVER_2) per la funzione sin.
    constexpr float PI_HALF = 1.57079632679f; // Utile solo se usata altrove, qui non ha effetto diretto

    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    // Manteniamo questi valori "ammorbiditi" per la fluidità del movimento lento.
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); 
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); 
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida per movimenti lenti vicino allo zero
    // Manteniamo i moltiplicatori leggermente aumentati per una transizione meno aggressiva.
    if (fabs(vx) < 1.5) vx *= 0.90; 
    if (fabs(vy) < 1.5) vy *= 0.85; 

    // --- INIZIO: Ottimizzazione 20 (Rev. A) ---
    // Calcola la distanza normalizzata dal centro (0 al centro, 1 ai bordi)
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    // Calcola la distanza normalizzata dal bordo più vicino (0 al bordo, 1 al centro)
    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    // Fattore di stabilizzazione che aumenta man mano che ci si allontana dal centro (effetto zona centrale)
    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattore di stabilizzazione aggiuntivo per gli angoli/bordi più estremi (effetto bordo)
    // Manteniamo gli esponenti ridotti per un effetto complessivo più dolce.
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 3.5; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 4.5; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Aumentiamo ulteriormente la soglia per ampliare l'area di transizione.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.20; 

    // Nuovo parametro per la funzione di easing esponenziale invertita (usando pow()).
    // Valori più alti = transizione più morbida all'inizio, concentrando l'effetto sul bordo estremo.
    constexpr float EXP_SMOOTHNESS_FACTOR = 3.0; // Valore consigliato, puoi provare 5.0 o 8.0 se serve più dolcezza.

    // Logica di interpolazione tra centro e bordo usando la nuova funzione di easing
    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        // normalized_dist_in_transition va da 0 (al bordo estremo) a 1 (all'inizio della zona di transizione)
        float normalized_dist_in_transition = dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD; 
        
        // weight_for_corner_exp sarà grande (vicino a 1) al bordo, e piccolo (vicino a 0) all'inizio della transizione.
        // Questo concentra l'influenza di 'stabilization_corner' verso il bordo estremo.
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);
        
        combined_stabilization_factor_x = (1.0 - weight_for_corner_exp) * stabilization_center_x + weight_for_corner_exp * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float normalized_dist_in_transition = dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD;
        float weight_for_corner_exp = pow(1.0 - normalized_dist_in_transition, EXP_SMOOTHNESS_FACTOR);

        combined_stabilization_factor_y = (1.0 - weight_for_corner_exp) * stabilization_center_y + weight_for_corner_exp * stabilization_corner_y;
    }
    
    // Assicurati che il fattore combinato non scenda sotto MIN_PRECISION_FACTOR
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    // Applica i fattori di stabilizzazione calcolati
    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 20 (Rev. A) ---

    // Auto-selezione tra modalità adattiva e blocco
    // Manteniamo il moltiplicatore della DEAD_ZONE a 1.2 per la stabilità da fermo.
    if (fabs(vx) < (DEAD_ZONE_X * 1.2) && fabs(vy) < (DEAD_ZONE_Y * 1.2)) { 
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    constexpr float PI_HALF = 1.57079632679f;

    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_ay[i];

    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i]; 

    // --- INIZIO: Ottimizzazione 17 - Ammorbidire Uscita dalla Dead Zone e Transizione Lenta ---
    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    // Aumento leggero di 0.1 e 0.15 a 0.2 e 0.25 (o 0.2 e 0.2) per ridurre l'aggressività dello smorzamento a bassa velocità.
    // Questo permette più "libertà" al cursore quando si muove lentamente.
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0); // Modificato da 0.1 a 0.2
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.25, 1.0); // Modificato da 0.15 a 0.25
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    // Aumentiamo leggermente i moltiplicatori per rendere la transizione meno aggressiva
    if (fabs(vx) < 1.5) vx *= 0.90; // Modificato da 0.85 a 0.90
    if (fabs(vy) < 1.5) vy *= 0.85; // Modificato da 0.80 a 0.85

    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    constexpr float CORNER_SMOOTHING_EXPONENT_X = 4.0; 
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 5.0; 

    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    constexpr float EDGE_TRANSITION_THRESHOLD = 0.12; 

    // Rimuoviamo il SLOW_EDGE_STABILIZATION_BOOST che potrebbe aver causato il traballamento iniziale.
    // L'idea è che la gestione del bordo deve essere intrinsecamente fluida, senza boost condizionali.
    // Se il cursore trema all'inizio del movimento, significa che lo smorzamento è troppo variabile o aggressivo.

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD);
        combined_stabilization_factor_x = (1.0 - weight_for_corner) * stabilization_center_x + weight_for_corner * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD);
        combined_stabilization_factor_y = (1.0 - weight_for_corner) * stabilization_center_y + weight_for_corner * stabilization_corner_y;
    }
    
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 17 ---

    // Auto-selezione tra modalità adattiva e blocco
    // Aumentiamo leggermente la DEAD_ZONE per prevenire un "risveglio" troppo brusco.
    // Il cursore deve rimanere fermo, ma con meno "tensione" all'inizio del movimento.
    if (fabs(vx) < (DEAD_ZONE_X * 1.5) && fabs(vy) < (DEAD_ZONE_Y * 1.5)) { // Aumentato di 1.5x
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Definizione di PI_HALF (o PI_OVER_2) per la funzione sin.
    constexpr float PI_HALF = 1.57079632679f;

    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_vy[i]; // Correzione: era last_vy[i], dovrebbe essere last_ax[i] per coerenza se volevi il jerk dell'asse x

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.15, 1.0);
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    if (fabs(vx) < 1.5) vx *= 0.85;
    if (fabs(vy) < 1.5) vy *= 0.80;

    // --- INIZIO: Ottimizzazione 13 - Allentare Delicatamente il Magnetismo ai Bordi Laterali ---
    // Calcola la distanza normalizzata dal centro (0 al centro, 1 ai bordi)
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    // Calcola la distanza normalizzata dal bordo più vicino (0 al bordo, 1 al centro)
    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    // Fattore di stabilizzazione che aumenta man mano che ci si allontana dal centro
    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattore di stabilizzazione aggiuntivo per gli angoli/bordi più estremi
    // Riduzione dell'esponente per l'asse X per ridurre il "magnetismo" laterale.
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 4.0; // Ridotto da 5.0 a 4.0
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 5.0; // Mantenuto a 5.0 (perché il problema è sui lati)
    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Soglia per l'attivazione della media ponderata: Ripristino a 0.12.
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.12;

    // Micro-tweak precedente: Rimuoviamo o riduciamo ulteriormente il boost finale,
    // o lo rendiamo specifico per l'asse Y se lì non causa problemi.
    constexpr float MICRO_BOOST_AMOUNT_X = 0.0; // Nessun boost per l'asse X
    constexpr float MICRO_BOOST_START_WEIGHT_X = 0.9; 

    constexpr float MICRO_BOOST_AMOUNT_Y = 0.05; // Manteniamo il boost per Y (se lì non ha problemi)
    constexpr float MICRO_BOOST_START_WEIGHT_Y = 0.9;

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD);
        if (weight_for_corner > MICRO_BOOST_START_WEIGHT_X) {
            float boost_factor = (weight_for_corner - MICRO_BOOST_START_WEIGHT_X) / (1.0 - MICRO_BOOST_START_WEIGHT_X);
            weight_for_corner += MICRO_BOOST_AMOUNT_X * boost_factor; // Ora 0 per X
            weight_for_corner = constrain(weight_for_corner, 0.0, 1.0);
        }
        combined_stabilization_factor_x = (1.0 - weight_for_corner) * stabilization_center_x + weight_for_corner * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD);
        if (weight_for_corner > MICRO_BOOST_START_WEIGHT_Y) {
            float boost_factor = (weight_for_corner - MICRO_BOOST_START_WEIGHT_Y) / (1.0 - MICRO_BOOST_START_WEIGHT_Y);
            weight_for_corner += MICRO_BOOST_AMOUNT_Y * boost_factor;
            weight_for_corner = constrain(weight_for_corner, 0.0, 1.0);
        }
        combined_stabilization_factor_y = (1.0 - weight_for_corner) * stabilization_center_y + weight_for_corner * stabilization_corner_y;
    }
    
    // Assicurati che il fattore combinato non scenda sotto MIN_PRECISION_FACTOR
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 13 ---

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.15, 1.0);
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    if (fabs(vx) < 1.5) vx *= 0.85;
    if (fabs(vy) < 1.5) vy *= 0.80;

    // --- INIZIO: Ottimizzazione 5 - Ammorbidire l'Attrazione ai Bordi ---
    // Calcola la distanza normalizzata dal centro (0 al centro, 1 ai bordi)
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    // Calcola la distanza normalizzata dal bordo più vicino (0 al bordo, 1 al centro)
    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    // Fattore di stabilizzazione che aumenta man mano che ci si allontana dal centro
    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattore di stabilizzazione aggiuntivo per gli angoli/bordi più estremi
    // Riportiamo gli esponenti a valori più bassi o simili a quelli dell'Ottimizzazione 3 (6.0)
    // o anche leggermente più bassi se il problema era l'aggressività
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 5.0; // Prova 5.0, era 7.0
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 5.0; // Prova 5.0, era 7.0
    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Soglia per l'attivazione della media ponderata - la riportiamo a 0.1 o anche leggermente più alta
    // per far iniziare la transizione un po' prima e in modo più graduale
    constexpr float EDGE_TRANSITION_THRESHOLD = 0.12; // Da 0.08 a 0.12

    if (dist_to_edge_x_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_x_norm / EDGE_TRANSITION_THRESHOLD);
        // Ritorno a interpolazione lineare o smoothstep più leggero se il smoothstep era troppo aggressivo
        // Proviamo con una semplice interpolazione lineare inizialmente per la massima morbidezza
        combined_stabilization_factor_x = (1.0 - weight_for_corner) * stabilization_center_x + weight_for_corner * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < EDGE_TRANSITION_THRESHOLD) {
        float weight_for_corner = 1.0 - (dist_to_edge_y_norm / EDGE_TRANSITION_THRESHOLD);
        combined_stabilization_factor_y = (1.0 - weight_for_corner) * stabilization_center_y + weight_for_corner * stabilization_corner_y;
    }
    
    // Assicurati che il fattore combinato non scenda sotto MIN_PRECISION_FACTOR
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 5 ---

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    // Riportiamo a 0.15 per tremor_reduction_factor_y (come in Punto 0) dato che il problema è ai bordi e non qui
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.15, 1.0); // Riportato al valore originale del Punto 0
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    if (fabs(vx) < 1.5) vx *= 0.85;
    if (fabs(vy) < 1.5) vy *= 0.80;

    // --- INIZIO: Ottimizzazione 3 - Ammorbidire Stabilizzazione Bordi/Angoli ---
    // Calcola la distanza normalizzata dal centro (0 al centro, 1 ai bordi)
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    // Calcola la distanza normalizzata dal bordo più vicino (0 al bordo, 1 al centro)
    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    // Fattore di stabilizzazione che aumenta man mano che ci si allontana dal centro
    // Ritorno all'uso di EDGE_SMOOTHING senza moltiplicatori specifici per Y
    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING); // Riportato a EDGE_SMOOTHING

    // Fattore di stabilizzazione aggiuntivo per gli angoli/bordi più estremi
    // Riduciamo la potenza e la rendiamo un po' meno aggressiva negli angoli
    constexpr float CORNER_SMOOTHING_EXPONENT_X = 6.0; // Riduciamo l'aggressività
    constexpr float CORNER_SMOOTHING_EXPONENT_Y = 6.0; // Riduciamo l'aggressività
    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT_X);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT_Y);

    // Nuovo approccio per combinare i fattori: Media ponderata invece di moltiplicazione pura
    // Diamo più peso al fattore centrale quando si è lontani dai bordi più estremi,
    // e un peso crescente al fattore angolare solo quando ci si avvicina moltissimo.
    float combined_stabilization_factor_x = stabilization_center_x;
    float combined_stabilization_factor_y = stabilization_center_y;

    // Applichiamo il fattore angolare solo se siamo molto vicini al bordo
    // Il threshold 0.1 indica l'ultimo 10% della distanza al bordo
    if (dist_to_edge_x_norm < 0.1) {
        // Interpola tra il fattore centrale e quello angolare man mano che ci si avvicina al bordo
        // L'interpolazione è guidata da 'dist_to_edge_x_norm' normalizzato su 0.1
        float weight_for_corner = 1.0 - (dist_to_edge_x_norm / 0.1); // Va da 0 a 1 man mano che ci si avvicina al bordo
        combined_stabilization_factor_x = (1.0 - weight_for_corner) * stabilization_center_x + weight_for_corner * stabilization_corner_x;
    }

    if (dist_to_edge_y_norm < 0.1) {
        float weight_for_corner = 1.0 - (dist_to_edge_y_norm / 0.1);
        combined_stabilization_factor_y = (1.0 - weight_for_corner) * stabilization_center_y + weight_for_corner * stabilization_corner_y;
    }

    // Assicurati che il fattore combinato non scenda sotto MIN_PRECISION_FACTOR
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 3 ---

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.15, 1.0);  // Maggiore adattamento per ridurre oscillazione verticale
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    if (fabs(vx) < 1.5) vx *= 0.85;
    if (fabs(vy) < 1.5) vy *= 0.80;

    // --- INIZIO: Ottimizzazione 1 - Stabilizzazione Unificata per Bordi e Angoli ---
    // Calcola la distanza normalizzata dal centro (0 al centro, 1 ai bordi)
    float dist_from_center_x_norm = fabs(mouseX - X_CENTER) / (X_MAX / 2.0);
    float dist_from_center_y_norm = fabs(mouseY - Y_CENTER) / (Y_MAX / 2.0);

    // Calcola la distanza normalizzata dal bordo più vicino (0 al bordo, 1 al centro)
    float dist_to_edge_x_norm = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / (X_MAX / 2.0);
    float dist_to_edge_y_norm = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / (Y_MAX / 2.0);

    // Fattore di stabilizzazione che aumenta man mano che ci si allontana dal centro
    // La potenza EDGE_SMOOTHING (che è circa 1.5 nel tuo caso) controlla la "curva"
    float stabilization_center_x = 1.0 - pow(dist_from_center_x_norm, EDGE_SMOOTHING);
    float stabilization_center_y = 1.0 - pow(dist_from_center_y_norm, EDGE_SMOOTHING);

    // Fattore di stabilizzazione aggiuntivo per gli angoli/bordi più estremi
    // Questo fattore diventa significativo solo quando dist_to_edge_norm è molto piccolo (vicino al bordo)
    // L'esponente alto (es. 8.0) assicura che l'effetto sia localizzato
    constexpr float CORNER_SMOOTHING_EXPONENT = 8.0; // Questo è il valore da calibrare per primo
    float stabilization_corner_x = 1.0 - pow(1.0 - dist_to_edge_x_norm, CORNER_SMOOTHING_EXPONENT);
    float stabilization_corner_y = 1.0 - pow(1.0 - dist_to_edge_y_norm, CORNER_SMOOTHING_EXPONENT);

    // Combiniamo i due fattori. Usiamo la moltiplicazione per un effetto cumulativo ma più controllato.
    float combined_stabilization_factor_x = stabilization_center_x * stabilization_corner_x;
    float combined_stabilization_factor_y = stabilization_center_y * stabilization_corner_y;

    // Assicurati che il fattore combinato non scenda sotto MIN_PRECISION_FACTOR
    combined_stabilization_factor_x = constrain(combined_stabilization_factor_x, MIN_PRECISION_FACTOR, 1.0);
    combined_stabilization_factor_y = constrain(combined_stabilization_factor_y, MIN_PRECISION_FACTOR, 1.0);

    vx *= combined_stabilization_factor_x;
    vy *= combined_stabilization_factor_y;
    ax *= combined_stabilization_factor_x;
    ay *= combined_stabilization_factor_y;
    // --- FINE: Ottimizzazione 1 ---

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8);
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);

    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);

    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO

///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor_x = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    float tremor_reduction_factor_y = constrain(1.0 - fabs(vy) / Y_MAX, 0.15, 1.0);  // Maggiore adattamento per ridurre oscillazione verticale
    vx *= tremor_reduction_factor_x;
    vy *= tremor_reduction_factor_y;

    // Transizione morbida: migliorata per evitare scatti nei movimenti lenti vicino ai bordi
    if (fabs(vx) < 1.5) vx *= 0.85;
    if (fabs(vy) < 1.5) vy *= 0.80;

    // Stabilizzazione progressiva ai bordi e angoli
    float edge_precision_factor_x = constrain(1.0 - pow(fabs(mouseX - X_CENTER) / X_MAX, EDGE_SMOOTHING * 0.8), MIN_PRECISION_FACTOR, 1.0);
    float edge_precision_factor_y = constrain(1.0 - pow(fabs(mouseY - Y_CENTER) / Y_MAX, EDGE_SMOOTHING * 0.9), MIN_PRECISION_FACTOR, 1.0);

    vx *= edge_precision_factor_x;
    vy *= edge_precision_factor_y;
    ax *= edge_precision_factor_x;
    ay *= edge_precision_factor_y;

    // Compensazione angolare affinata per movimenti lenti
    float corner_distance_x = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / X_MAX;
    float corner_distance_y = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / Y_MAX;

    float corner_stabilization_x = constrain(1.0 - pow(corner_distance_x, 3.0), MIN_PRECISION_FACTOR, 1.0);
    float corner_stabilization_y = constrain(1.0 - pow(corner_distance_y, 4.0), MIN_PRECISION_FACTOR, 1.0);  // Affinata la transizione verticale per movimenti più naturali

    vx *= corner_stabilization_x;
    vy *= corner_stabilization_y;
    ax *= corner_stabilization_x;
    ay *= corner_stabilization_y;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Transizione morbida: evitiamo il blocco totale e manteniamo un leggero adattamento
    if (fabs(vx) < 1.0 && fabs(vy) < 1.0) {
        vx *= 0.8;
        vy *= 0.8;
    }

    // Stabilizzazione progressiva ai bordi e agli angoli utilizzando le nuove variabili
    float edge_precision_factor_x = constrain(1.0 - pow(fabs(mouseX - X_CENTER) / X_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);
    float edge_precision_factor_y = constrain(1.0 - pow(fabs(mouseY - Y_CENTER) / Y_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);

    vx *= edge_precision_factor_x;
    vy *= edge_precision_factor_y;
    ax *= edge_precision_factor_x;
    ay *= edge_precision_factor_y;

    // Compensazione angolare migliorata
    float corner_distance_x = min(fabs(mouseX - X_MIN), fabs(mouseX - X_MAX)) / X_MAX;
    float corner_distance_y = min(fabs(mouseY - Y_MIN), fabs(mouseY - Y_MAX)) / Y_MAX;
    
    float corner_stabilization_x = constrain(1.0 - pow(corner_distance_x, 3.0), MIN_PRECISION_FACTOR, 1.0);
    float corner_stabilization_y = constrain(1.0 - pow(corner_distance_y, 3.0), MIN_PRECISION_FACTOR, 1.0);

    vx *= corner_stabilization_x;
    vy *= corner_stabilization_y;
    ax *= corner_stabilization_x;
    ay *= corner_stabilization_y;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Transizione morbida: evitiamo il blocco totale e manteniamo un leggero adattamento
    if (fabs(vx) < 1.0 && fabs(vy) < 1.0) {
        vx *= 0.8;
        vy *= 0.8;
    }

    // Stabilizzazione progressiva ai bordi e agli angoli utilizzando le nuove variabili
    float edge_precision_factor_x = constrain(1.0 - pow(fabs(mouseX - X_CENTER) / X_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);
    float edge_precision_factor_y = constrain(1.0 - pow(fabs(mouseY - Y_CENTER) / Y_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);

    vx *= edge_precision_factor_x;
    vy *= edge_precision_factor_y;
    ax *= edge_precision_factor_x;
    ay *= edge_precision_factor_y;

    // Compensazione specifica per gli angoli del monitor
    float corner_stabilization_x = constrain(1.0 - pow(fabs(mouseX - X_MIN) / X_MAX, 2.0) * pow(fabs(mouseX - X_MAX) / X_MAX, 2.0), MIN_PRECISION_FACTOR, 1.0);
    float corner_stabilization_y = constrain(1.0 - pow(fabs(mouseY - Y_MIN) / Y_MAX, 2.0) * pow(fabs(mouseY - Y_MAX) / Y_MAX, 2.0), MIN_PRECISION_FACTOR, 1.0);

    vx *= corner_stabilization_x;
    vy *= corner_stabilization_y;
    ax *= corner_stabilization_x;
    ay *= corner_stabilization_y;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMENNTO

///////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Transizione morbida: evitiamo il blocco totale e manteniamo un leggero adattamento
    if (fabs(vx) < 1.0 && fabs(vy) < 1.0) {
        vx *= 0.8;
        vy *= 0.8;
    }

    // Stabilizzazione progressiva ai bordi e agli angoli utilizzando le nuove variabili
    float edge_precision_factor_x = constrain(1.0 - pow(fabs(mouseX - X_CENTER) / X_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);
    float edge_precision_factor_y = constrain(1.0 - pow(fabs(mouseY - Y_CENTER) / Y_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);

    vx *= edge_precision_factor_x;
    vy *= edge_precision_factor_y;
    ax *= edge_precision_factor_x;
    ay *= edge_precision_factor_y;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif

#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Transizione morbida: evitiamo il blocco totale e manteniamo un leggero adattamento
    if (fabs(vx) < 1.0 && fabs(vy) < 1.0) {
        vx *= 0.8;
        vy *= 0.8;
    }

    // Stabilizzazione progressiva ai bordi e agli angoli
    float edge_precision_factor_x = constrain(1.0 - pow(fabs(mouseX - X_CENTER) / X_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);
    float edge_precision_factor_y = constrain(1.0 - pow(fabs(mouseY - Y_CENTER) / Y_MAX, EDGE_SMOOTHING), MIN_PRECISION_FACTOR, 1.0);

    vx *= edge_precision_factor_x;
    vy *= edge_precision_factor_y;
    ax *= edge_precision_factor_x;
    ay *= edge_precision_factor_y;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}

#endif

#ifdef COMMENTO  //nuovo punto 0
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: più stabilizzazione se il movimento è lento, meno se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.1, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Transizione morbida: evitiamo il blocco totale e manteniamo un leggero adattamento
    if (fabs(vx) < 1.0 && fabs(vy) < 1.0) {
        vx *= 0.8;
        vy *= 0.8;
    }

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Smorzamento proporzionale alla velocità: maggiore stabilizzazione se il movimento è lento, nessuna se rapido
    float tremor_reduction_factor = constrain(1.0 - fabs(vx) / X_MAX, 0.2, 1.0);
    vx *= tremor_reduction_factor;
    vy *= tremor_reduction_factor;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMANTO
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Riduzione del tremolio solo nei micro-movimenti
    if (fabs(vx) < 2.0 && fabs(vy) < 2.0) {
        vx *= 0.6;
        vy *= 0.6;
    }

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif // COMMENTO
#ifdef COMMENTO
void FW_Common::Kalman_filter(uint8_t i, int& mouseX, int& mouseY) {
    // Calcolo della velocità e accelerazione
    float vx = mouseX - last_x[i];
    float vy = mouseY - last_y[i];
    float ax = vx - last_vx[i];
    float ay = vy - last_vy[i];

    // Calcolo del jerk (variazione dell’accelerazione)
    float jerk_x = ax - last_ax[i];
    float jerk_y = ay - last_ay[i];

    // Filtro di media mobile per stabilizzare la posizione
    mouseX = (mouseX + last_x[i] * 2 + x_filt[i] * 3) / 6;
    mouseY = (mouseY + last_y[i] * 2 + y_filt[i] * 3) / 6;

    // Auto-selezione tra modalità adattiva e blocco
    if (fabs(vx) < DEAD_ZONE_X && fabs(vy) < DEAD_ZONE_Y) {
        stationary_counter[i]++;

        // Se la posizione è ferma da molto tempo, blocchiamo completamente l'aggiornamento
        if (stationary_counter[i] > 50) {
            mouseX = last_x[i];
            mouseY = last_y[i];
            return;
        }
    } else {
        // Reset del contatore di fermo se inizia a muoversi
        stationary_counter[i] = 0;
        q_min_base[i] = 0.005;
    }

    // Predizione migliorata con pesi variabili (ridotta aggressività del jerk)
    float prediction_weight_v = constrain(fabs(vx) / X_MAX, 0.1, 0.8); 
    float prediction_weight_a = constrain(fabs(ax) / (X_MAX * 0.01), 0.2, 0.9);
    float prediction_weight_j = constrain(fabs(jerk_x) / (X_MAX * 0.005), 0.01, 0.3);

    float predicted_x = x_filt[i] + vx * prediction_weight_v + ax * prediction_weight_a;
    float predicted_y = y_filt[i] + vy * prediction_weight_v + ay * prediction_weight_a;

    // Adattamento dinamico del rumore di processo
    float q_x = (fabs(ax) > X_MAX * 0.002 || fabs(jerk_x) > X_MAX * 0.001) ? q_max : q_min_base[i];
    float q_y = (fabs(ay) > Y_MAX * 0.002 || fabs(jerk_y) > Y_MAX * 0.001) ? q_max : q_min_base[i];

    // Aggiornamento del filtro di Kalman
    p_x[i] += q_x;
    p_y[i] += q_y;
    k_x[i] = p_x[i] / (p_x[i] + r_dynamic[i]);
    k_y[i] = p_y[i] / (p_y[i] + r_dynamic[i]);
    
    x_filt[i] = predicted_x + k_x[i] * (mouseX - predicted_x);
    y_filt[i] = predicted_y + k_y[i] * (mouseY - predicted_y);
    
    p_x[i] *= (1 - k_x[i]);
    p_y[i] *= (1 - k_y[i]);

    // Aggiorna i valori passati per riferimento
    mouseX = static_cast<int>(x_filt[i]);
    mouseY = static_cast<int>(y_filt[i]);

    // Memorizzazione per la prossima iterazione
    last_x[i] = x_filt[i];
    last_y[i] = y_filt[i];
    last_vx[i] = vx;
    last_vy[i] = vy;
    last_ax[i] = ax;
    last_ay[i] = ay;
    last_jerk_x[i] = jerk_x;
    last_jerk_y[i] = jerk_y;
}
#endif //COMMENTO
//#endif // COMMENTO


#ifdef COMMENTO

void FW_Common::Kalman_filter(uint8_t i, int& mouseX,int& mouseY)
{
// --- Blocco di codice per il filtro di Kalman ---
// Inserisci questo blocco direttamente nel tuo loop principale, dopo aver letto
// le variabili raw `mouseX` e `mouseY`.
// Al termine di questo blocco, `mouseX` e `mouseY` conterranno le coordinate filtrate.
//
// Le variabili `mouseX` e `mouseY` devono essere dichiarate (es. `int mouseX; int mouseY;`)
// e contenere le coordinate raw *prima* di questo blocco.
/*
Esempio di utilizzo (nel tuo loop principale):

int mouseX = raw_x_from_sensor; // Leggi la coordinata X grezza dal sensore
int mouseY = raw_y_from_sensor; // Leggi la coordinata Y grezza dal sensore

// --- INIZIO BLOCCO FILTRO DI KALMAN ---
*/
unsigned long currentMillis = millis(); // Tempo attuale in millisecondi.
float dt; // Delta tempo (tempo trascorso) in secondi.

// Calcolo del delta tempo (dt). Gestisce il primo avvio per evitare un dt nullo o troppo grande.
if (lastFilterExecutionTime[i] == 0) {
    dt = 0.01f; // Valore di fallback sicuro per il primo ciclo.
} else {
    dt = (float)(currentMillis - lastFilterExecutionTime[i]) / 1000.0f;
}
lastFilterExecutionTime[i] = currentMillis; // Aggiorna il timestamp per la prossima esecuzione.

// Protezione robusta per dt: evita divisioni per zero o valori irrealistici.
if (dt <= 0.0f || dt > 1.0f) {
    dt = 0.01f;
}

float dt_sq = dt * dt; // Pre-calcolo di dt*dt per efficienza nei calcoli.

// --- Logica di rilevamento del "movimento brusco volontario" ---
// Calcola la "magnitudine al quadrato del salto" tra la misura raw e la posizione stimata attuale.
float deltaX_raw_est = (float)mouseX - estimatedX[i];
float deltaY_raw_est = (float)mouseY - estimatedY[i];
float currentResidualMagnitudeSq = deltaX_raw_est * deltaX_raw_est + deltaY_raw_est * deltaY_raw_est;

// Soluzione precedente (con sqrt()):
// float currentResidualMagnitude = sqrt(deltaX_raw_est * deltaX_raw_est + deltaY_raw_est * deltaY_raw_est);

// Inizializza i valori dinamici di R (rumore di misurazione) e Q (incertezza del modello) con i valori base.
float current_R_val = BASE_R[i];
float current_Q_val = BASE_Q[i];

// Se il quadrato del "salto" è significativo (supera il quadrato della soglia), aumentiamo Q e R.
// Questa ottimizzazione evita una costosa operazione sqrt() per ogni ciclo.
if (currentResidualMagnitudeSq > BRUSQUE_MOVE_THRESHOLD_SQ[i]) { // Confronto con il quadrato della soglia
// Soluzione precedente (con sqrt()):
// if (currentResidualMagnitude > BRUSQUE_MOVE_THRESHOLD) {
    current_R_val *= BRUSQUE_MOVE_R_MULTIPLIER[i];
    current_Q_val *= BRUSQUE_MOVE_Q_MULTIPLIER[i];
}

// --- Adattamento di Q e R in base alla velocità complessiva del mouse ---
// Calcola la magnitudine al quadrato della velocità stimata complessiva per evitare la sqrt().
float totalVelocityMagnitudeSq = velocityX[i] * velocityX[i] + velocityY[i] * velocityY[i];
// La radice quadrata è ancora necessaria qui se il fattore di adattamento dipende dalla magnitudine lineare.
float totalVelocityMagnitude = sqrt(totalVelocityMagnitudeSq);

// Soluzione precedente (calcolava sqrt() due volte se usata per la soglia):
// float totalVelocityMagnitude = sqrt(velocityX * velocityX + velocityY * velocityY);


// Aumenta Q e R proporzionalmente alla velocità. Ciò permette al filtro di essere più reattivo
// durante i movimenti rapidi e più stabile quando il mouse è quasi fermo.
current_Q_val *= (1.0f + totalVelocityMagnitude * VELOCITY_Q_FACTOR[i]);
current_R_val *= (1.0f + totalVelocityMagnitude * VELOCITY_R_FACTOR[i]);

// Applica limiti massimi per Q e R per prevenire instabilità o valori eccessivi.
current_Q_val = fmin(current_Q_val, Q_MAX_ADAPTIVE[i]);
current_R_val = fmin(current_R_val, R_MAX_ADAPTIVE[i]);

// --- Applicazione del filtro di Kalman per l'asse X ---
// 1. Predizione dello stato per X: stima la prossima posizione e velocità di X.
float predictedPosX = estimatedX[i] + (velocityX[i] * dt) + (0.5f * accelerationX[i] * dt_sq);
float predictedVelX = velocityX[i] + (accelerationX[i] * dt);

// 2. Predizione dell'incertezza (P_x): aumenta l'incertezza per il rumore del modello.
P_x[i] = P_x[i] + current_Q_val;
P_x[i] = constrain(P_x[i], P_MIN[i], P_MAX[i]); // Limita P_x per stabilità.

// 3. Calcolo del Guadagno di Kalman (K_x): quanto ci fidiamo della misura corrente.
float K_x = P_x[i] / (P_x[i] + current_R_val);

// 4. Correzione con la misura per X: aggiorna le stime usando la misura raw e K_x.
float residualX = (float)mouseX - predictedPosX;
estimatedX[i] = predictedPosX + K_x * residualX;
velocityX[i] = predictedVelX + K_x * (residualX / dt);

// 5. Aggiornamento dell'accelerazione per X: adattiva per rispondere meglio ai cambi di direzione.
float adaptiveMaxAccelerationX = MAX_ACCELERATION_BASE[i] * (0.5f + 0.5f * (1.0f / (1.0f + constrain(fabs(velocityX[i]), 0.0f, MAX_VELOCITY[i]) * 0.01f)));
accelerationX[i] = constrain(accelerationX[i] + (K_x * 2.0f * residualX) / dt_sq, -adaptiveMaxAccelerationX, adaptiveMaxAccelerationX);

// 6. Limitazione della velocità per X: evita che la velocità stimata cresca indefinitamente.
velocityX[i] = constrain(velocityX[i], -MAX_VELOCITY[i], MAX_VELOCITY[i]);

// 7. Aggiornamento della covarianza (P_x): riduce l'incertezza dopo la correzione.
P_x[i] = P_x[i] * (1.0f - K_x);
P_x[i] = constrain(P_x[i], P_MIN[i], P_MAX[i]); // Mantiene P_x entro limiti.


// --- Applicazione del filtro di Kalman per l'asse Y ---
// La logica è identica a quella dell'asse X, ma applicata alle variabili Y.
// 1. Predizione dello stato per Y
float predictedPosY = estimatedY[i] + (velocityY[i] * dt) + (0.5f * accelerationY[i] * dt_sq);
float predictedVelY = velocityY[i] + (accelerationY[i] * dt);

// 2. Predizione dell'incertezza (P_y)
P_y[i] = P_y[i] + current_Q_val;
P_y[i] = constrain(P_y[i], P_MIN[i], P_MAX[i]);

// 3. Calcolo del Guadagno di Kalman (K_y)
float K_y = P_y[i] / (P_y[i] + current_R_val);

// 4. Correzione con la misura per Y
float residualY = (float)mouseY - predictedPosY;
estimatedY[i] = predictedPosY + K_y * residualY;
velocityY[i] = predictedVelY + K_y * (residualY / dt);

// 5. Aggiornamento dell'accelerazione per Y
float adaptiveMaxAccelerationY = MAX_ACCELERATION_BASE[i] * (0.5f + 0.5f * (1.0f / (1.0f + constrain(fabs(velocityY[i]), 0.0f, MAX_VELOCITY[i]) * 0.01f)));
accelerationY[i] = constrain(accelerationY[i] + (K_y * 2.0f * residualY) / dt_sq, -adaptiveMaxAccelerationY, adaptiveMaxAccelerationY);

// 6. Limitazione della velocità per Y
velocityY[i] = constrain(velocityY[i], -MAX_VELOCITY[i], MAX_VELOCITY[i]);

// 7. Aggiornamento della covarianza (P_y)
P_y[i] = P_y[i] * (1.0f - K_y);
P_y[i] = constrain(P_y[i], P_MIN[i], P_MAX[i]);

// --- AGGIORNAMENTO DELLE VARIABILI mouseX E mouseY CON I VALORI FILTRATI ---
mouseX = (int)estimatedX[i];
mouseY = (int)estimatedY[i];
/*
// --- FINE BLOCCO FILTRO DI KALMAN ---
*/

#ifdef COMMENTO
Serial.printf("RawX:%d RawY:%d FiltX:%.2f FiltY:%.2f VelX:%.1f VelY:%.1f AccX:%.1f AccY:%.1f Px:%.3f Py:%.3f R_eff:%.3f Q_eff:%.3f ResMagSq:%.1f TotalVelMag:%.1f\n",
              mouseX, mouseY, estimatedX, estimatedY, velocityX, velocityY, accelerationX, accelerationY, P_x, P_y, current_R_val, current_Q_val, currentResidualMagnitudeSq, totalVelocityMagnitude);
#endif
}

#endif // COMMENTO

#endif //CAM_SIMPLE_KALMAN_FILTER   
// ============== 696969 =====================================================================

void FW_Common::PrintIrError()
{
    // set flag to warn desktop app when docking
    if(!camNotAvailable)
        camNotAvailable = true;

    if(dockedSaving) {
        char buf[2] = { OF_Const::sError, OF_Const::sErrCam };
        Serial.write(buf, 2);
    } else if(millis() - camWarningTimestamp > CAM_WARNING_INTERVAL) {
        Serial.println("CAMERROR: Not available");
        camWarningTimestamp = millis();
    }
}

void FW_Common::UpdateLastSeen()
{
    if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout) {
        if(lastSeen != OpenFIREdiamond.seen()) {
            #ifdef MAMEHOOKER
            if(!OF_Serial::serialMode)
            #endif // MAMEHOOKER
                #ifdef LED_ENABLE
                if(!lastSeen && OpenFIREdiamond.seen())
                    OF_RGB::LedOff();
                else if(lastSeen && !OpenFIREdiamond.seen())
                    OF_RGB::SetLedPackedColor(OF_RGB::IRSeen0Color);
                #endif // LED_ENABLE

            lastSeen = OpenFIREdiamond.seen();
        }
    } else {
        if(lastSeen != OpenFIREsquare.seen()) {
            #ifdef MAMEHOOKER
            if(!OF_Serial::serialMode) {
            #endif // MAMEHOOKER
                #ifdef LED_ENABLE
                if(!lastSeen && OpenFIREsquare.seen())
                    OF_RGB::LedOff();
                else if(lastSeen && !OpenFIREsquare.seen())
                    OF_RGB::SetLedPackedColor(OF_RGB::IRSeen0Color);
                #endif // LED_ENABLE
            #ifdef MAMEHOOKER
            }
            #endif // MAMEHOOKER
            lastSeen = OpenFIREsquare.seen();
        }
    }
}

bool FW_Common::SelectCalProfile(const int &profile)
{
    if(profile >= PROFILE_COUNT)
        return false;

    if(OF_Prefs::currentProfile != profile) {
        stateFlags |= FW_Const::StateFlag_PrintSelectedProfile;
        OF_Prefs::currentProfile = profile;
    }

    OpenFIREper.source(OF_Prefs::profiles[profile].adjX, OF_Prefs::profiles[profile].adjY);                                                          
    OpenFIREper.deinit(0);

    // set IR sensitivity
    if(OF_Prefs::profiles[profile].irSens <= DFRobotIRPositionEx::Sensitivity_Max)
        SetIrSensitivity((DFRobotIRPositionEx::Sensitivity_e)OF_Prefs::profiles[profile].irSens);

    // set run mode
    if(OF_Prefs::profiles[profile].runMode < FW_Const::RunMode_Count)
        SetRunMode((FW_Const::RunMode_e)OF_Prefs::profiles[profile].runMode);

    #ifdef USES_DISPLAY
        if(gunMode != FW_Const::GunMode_Docked)
            OLED.TopPanelUpdate("Using ", OF_Prefs::profiles[profile].name);
    #endif // USES_DISPLAY
 
    #ifdef LED_ENABLE
        SetLedColorFromMode();
    #endif // LED_ENABLE

    // enable save to allow setting new default profile
    stateFlags |= FW_Const::StateFlag_SavePreferencesEn;
    return true;
}

#ifdef LED_ENABLE
void FW_Common::SetLedColorFromMode()
{
    switch(gunMode) {
    case FW_Const::GunMode_Calibration:
        OF_RGB::SetLedPackedColor(OF_RGB::CalModeColor);
        break;
    case FW_Const::GunMode_Pause:
        OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);
        break;
    case FW_Const::GunMode_Run:
        if(lastSeen)
             OF_RGB::LedOff();
        else OF_RGB::SetLedPackedColor(OF_RGB::IRSeen0Color);
        break;
    default:
        break;
    }
}
#endif // LED_ENABLE

#ifdef USES_DISPLAY
void FW_Common::RedrawDisplay()
{
    if(gunMode == FW_Const::GunMode_Docked)
        OLED.ScreenModeChange(ExtDisplay::Screen_Docked);
    else if(gunMode == FW_Const::GunMode_Pause) {
        OLED.ScreenModeChange(ExtDisplay::Screen_Pause);
        if(OF_Prefs::toggles[OF_Const::simplePause])
            OLED.PauseListUpdate(ExtDisplay::ScreenPause_Save);
        else OLED.PauseScreenShow(OF_Prefs::currentProfile,
                                  OF_Prefs::profiles[0].name,
                                  OF_Prefs::profiles[1].name,
                                  OF_Prefs::profiles[2].name,
                                  OF_Prefs::profiles[3].name);
    }
}
#endif // USES_DISPLAY

void FW_Common::SetIrSensitivity(const int &sensitivity)
{
    if(sensitivity > DFRobotIRPositionEx::Sensitivity_Max)
        return;

    if(OF_Prefs::profiles[OF_Prefs::currentProfile].irSens != sensitivity) {
        OF_Prefs::profiles[OF_Prefs::currentProfile].irSens = sensitivity;
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn;
    }

    if(dfrIRPos != nullptr) dfrIRPos->sensitivityLevel((DFRobotIRPositionEx::Sensitivity_e)sensitivity);
    //if(!(stateFlags & FW_Const::StateFlag_PrintSelectedProfile))
        //PrintIrSensitivity();
}

void FW_Common::SetIrLayout(const int &layout)
{
    // TODO: we need an enum for layout types available
    if(layout >= OF_Const::layoutTypes)
        return;

    if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout != layout) {
        OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout = layout;
        stateFlags |= FW_Const::StateFlag_SavePreferencesEn;
    }
}

int FW_Common::SavePreferences()
{
    // Unless the user's Docked,
    // Only allow one write per pause state until something changes.
    // Extra protection to ensure the same data can't write a bunch of times.
    if(gunMode != FW_Const::GunMode_Docked) {
        if(!(stateFlags & FW_Const::StateFlag_SavePreferencesEn))
            return OF_Prefs::Error_Success;

        stateFlags &= ~FW_Const::StateFlag_SavePreferencesEn;

        #ifdef USES_DISPLAY
            if(OLED.display != nullptr)
                OLED.ScreenModeChange(ExtDisplay::Screen_Saving);
        #endif // USES_DISPLAY
    }

    if(OF_Prefs::SaveProfiles() == OF_Prefs::Error_Success) {
        #ifdef USES_DISPLAY
            OLED.ScreenModeChange(ExtDisplay::Screen_SaveSuccess);
        #endif // USES_DISPLAY

        if(gunMode != FW_Const::GunMode_Docked) Serial.println("Settings saved to Flash"), Serial.flush();
        else Serial.printf("%c%c (Successfully saved to LittleFS Storage)", OF_Const::sSave, true), Serial.flush();
        OF_Prefs::SaveToggles();

        if(OF_Prefs::toggles[OF_Const::customPins])
            OF_Prefs::SavePins();

        OF_Prefs::SaveSettings();
        OF_Prefs::SaveButtons();
        OF_Prefs::SaveUSBID();

        #ifdef LED_ENABLE
            for(uint i = 0; i < 3; ++i) {
                OF_RGB::LedUpdate(25,25,255);
                delay(55);
                OF_RGB::LedOff();
                delay(40);
            }
        #endif // LED_ENABLE

        #ifdef USES_DISPLAY
            RedrawDisplay();
        #endif // USES_DISPLAY

        return OF_Prefs::Error_Success;
    } else {
        #ifdef USES_DISPLAY
            OLED.ScreenModeChange(ExtDisplay::Screen_SaveError);
        #endif // USES_DISPLAY

        // TODO: reimpl a detailed error string
        if(gunMode != FW_Const::GunMode_Docked) Serial.println("Error saving Preferences to Flash.");
        else Serial.printf("%c%c (Failed to save to LittleFS Storage)", OF_Const::sSave, false), Serial.flush();

        /*
        if(nvPrefsError != OF_Prefs::Error_Success) {
            Serial.print(NVRAMlabel);
            Serial.print(" error: ");
        #ifdef SAMCO_FLASH_ENABLE
            Serial.println(OF_Prefs::ErrorCodeToString(nvPrefsError));
        #else
            Serial.println(nvPrefsError);
        #endif // SAMCO_FLASH_ENABLE
        }*/

        #ifdef LED_ENABLE
            for(uint i = 0; i < 2; ++i) {
                OF_RGB::LedUpdate(255,10,5);
                delay(145);
                OF_RGB::LedOff();
                delay(60);
            }
        #endif // LED_ENABLE

        #ifdef USES_DISPLAY
            RedrawDisplay();
        #endif // USES_DISPLAY

        return OF_Prefs::Error_Write;
    }
}

void FW_Common::UpdateBindings(const bool &rebindStrSel)
{
    switch(gunMode) {
    case FW_Const::GunMode_Run:
        buttons.ReleaseAll();
        break;
    case FW_Const::GunMode_Docked:
    case FW_Const::GunMode_Init:
        // Updates pins
        for(int i = 0; i < ButtonCount; ++i)
            LightgunButtons::ButtonDesc[i].pin = OF_Prefs::pins[i];
        break;
    default:
        break;
    }

    if(rebindStrSel) {
        #if defined(PLAYER_START) && defined(PLAYER_SELECT)
        playerStartBtn = PLAYER_START;
        playerSelectBtn = PLAYER_SELECT;
        #else
        if(OF_Prefs::usb.devicePID > 0 && OF_Prefs::usb.devicePID < 5) {
            playerStartBtn = OF_Prefs::usb.devicePID + '0';
            playerSelectBtn = OF_Prefs::usb.devicePID + '4';
        } else {
            playerStartBtn = '1';
            playerSelectBtn = '5';
        }
        #endif // PLAYER_NUMBER
    }

    for(int i = 0; i < ButtonCount; ++i)
        memcpy(&LightgunButtons::ButtonDesc[i].reportType,
               OF_Prefs::backupButtonDesc[i],
               sizeof(OF_Prefs::backupButtonDesc[0]));

    // Updates button functions for low-button mode
    if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
        memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_A].reportType2,
               OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_Start],
               2);
        memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_B].reportType2,
               OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_Select],
               2);
    }

    #ifdef MAMEHOOKER
    if(OF_Serial::serialMappingsOffscreenShot) {
        memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportType2,
               OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_A],
               sizeof(LightgunButtons::Desc_s::reportType)*2);

        // remap bindings for low button users to make e.g. VCop 3 playable with 1 btn + pedal
        if(OF_Prefs::toggles[OF_Const::lowButtonsMode])
            memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_A].reportType,
                   OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_Reload],
                   sizeof(LightgunButtons::Desc_s::reportType)*2);
    }

    if(OF_Serial::serialMappingsPedalMode) switch(OF_Serial::serialMappingsPedalMode) {
    case 1:
        memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Pedal].reportType,
               OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_A],
               sizeof(OF_Prefs::backupButtonDesc[0]));
        break;
    case 2:
        memcpy(&LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Pedal].reportType,
               OF_Prefs::backupButtonDesc[FW_Const::BtnIdx_B],
               sizeof(OF_Prefs::backupButtonDesc[0]));
        break;
    default:
        break;
    }
    #endif // MAMEHOOKER

    UpdateStartSelect();
}

void FW_Common::UpdateStartSelect()
{
    uint8_t *btnMatchedPtr;
    for(int i = 0; i < ButtonCount; ++i) {
        do {
            btnMatchedPtr = (uint8_t*)memchr(&LightgunButtons::ButtonDesc[i].reportCode, 0xFF, sizeof(OF_Prefs::backupButtonDesc[i])-1);
            if(btnMatchedPtr != nullptr)
                *btnMatchedPtr = playerStartBtn;
        } while(btnMatchedPtr != nullptr);

        do {
            btnMatchedPtr = (uint8_t*)memchr(&LightgunButtons::ButtonDesc[i].reportCode, 0xFE, sizeof(OF_Prefs::backupButtonDesc[i])-1);
            if(btnMatchedPtr != nullptr)
                *btnMatchedPtr = playerSelectBtn;
        } while(btnMatchedPtr != nullptr);
    }
}



// ============ 696969 ========== ripristino di Serial dopo definizione per connessione seriali ==============
#ifdef OPENFIRE_WIRELESS_ENABLE
    #undef Serial
    #ifdef AUX_SERIAL
        #define Serial AUX_SERIAL
        #undef AuxSerial
    #endif
#endif // OPENFIRE_WIRELESS_ENABLE
// ============ 696969 ===== fine ripristino di Serial dopo definizione per connessione seriali ==============