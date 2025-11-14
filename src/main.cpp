/*!
 * @file OpenFIREmain.ino
 * @brief OpenFIRE - 4IR LED Lightgun sketch w/ support for force feedback and other features.
 * Forked from IR-GUN4ALL v4.2, which is based on Prow's Enhanced Fork from https://github.com/Prow7/ir-light-gun,
 * which in itself is based on the 4IR Beta "Big Code Update" SAMCO project from https://github.com/samuelballantyne/IR-Light-Gun
 *
 * @copyright Samco, https://github.com/samuelballantyne, June 2020
 * @copyright Mike Lynch, July 2021
 * @copyright That One Seong, https://github.com/SeongGino, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @author Mike Lynch
 * @author [That One Seong](SeongsSeongs@gmail.com)
 * @date 2025
 */

#include "OpenFIREmain.h"
#include "boards/OpenFIREshared.h"
#include "OpenFIREcolors.h"
#include "OpenFIRElights.h"
#include "OpenFIREserial.h"
#include "OpenFIREFeedback.h"
#include "OpenFIREprefs.h"
#include "OpenFIREconstant.h"
// 全局变量定义 - 用于频道显示
uint8_t currentChannel;
// ================= parte poi da rimuovere ==================
#ifdef CLOCK_CAM_WII
uint32_t tone_freq_main = 0;
uint32_t tone_duty_main = 0;
#endif //CLOCK_CAM_WII
// ================= fine parte poi da rimuovere =============

// =======696969===== GESTIONE DUAL CORE PER ESP32 CHE USA FREERTOS ===  INIZIALIZZAZIONE ========
#if defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
    void setup1();
    void loop1();
    TaskHandle_t task_loop1;
    void esploop1(void* pvParameters) {
        setup1();
        for (;;) loop1();
    }
#endif
// ======696969============= FINE GESTIONE DUAL CORE ESP32 ==== FINE INIZIALIZZAZIONE ============


// Sets up the environment
void setup() {

// ======== 696969 =========== X AVVIO DUAL CORE ESP32 =================================== 
#if defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
    #define STACK_SIZE_SECOND_CORE 10000  // basta 4096 ???
    #define PRIORITY_SECOND_CORE 0  // dovrebbe essere 1 ???
    xTaskCreatePinnedToCore(
    esploop1,               /* Task function. */
    "loop1",                /* name of task. */
    STACK_SIZE_SECOND_CORE, /* Stack size of task */
    NULL,                   /* parameter of the task */
    PRIORITY_SECOND_CORE,   /* priority of the task */ 
    &task_loop1,            /* Task handle to keep track of created task */
    !ARDUINO_RUNNING_CORE); /* pin task to core 0 */
#endif
// ======= 696969 ========= FINE X AVVIO DUAL CORE ESP32 =================================

    // In case some I2C devices deadlock the program
    // (can happen due to bad pin mappings)
    Wire.setTimeout(100);
    Wire1.setTimeout(100);
  
    #ifdef ARDUINO_ADAFRUIT_ITSYBITSY_RP2040
        // SAMCO 1.1 needs Pin 5 normally HIGH for the camera
        pinMode(14, OUTPUT);
        digitalWrite(14, HIGH);
    #endif // ARDUINO_ADAFRUIT_ITSYBITSY_RP2040
    #ifdef ARDUINO_RASPBERRY_PI_PICO
        // Pull switching regulator pin high to stabilize reads from ADC (analog pins)
        pinMode(23, OUTPUT);
        digitalWrite(23, HIGH);
    #endif

    // Init pins array, and OFPresets for later load ops
    OF_Prefs::LoadPresets();
    
    if(OF_Prefs::InitFS() == OF_Prefs::Error_Success) {
        //OF_Prefs::ResetPreferences(); // ============ FORMATTA IL FILE SYSTEM =================================
        OF_Prefs::LoadProfiles();
    
        // Profile sanity checks
        // resets offsets that are wayyyyy too unreasonably high
        for(int i = 0; i < PROFILE_COUNT; ++i) {
            if(OF_Prefs::profiles[i].topOffset >= 32768     || OF_Prefs::profiles[i].topOffset <= -32768 ||
               OF_Prefs::profiles[i].bottomOffset >= 32768  || OF_Prefs::profiles[i].bottomOffset <= -32768 ||
               OF_Prefs::profiles[i].rightOffset >= 32768   || OF_Prefs::profiles[i].rightOffset <= -32768 ||
               OF_Prefs::profiles[i].leftOffset >= 32768    || OF_Prefs::profiles[i].leftOffset <= -32768) {
                OF_Prefs::profiles[i].topOffset = 0;
                OF_Prefs::profiles[i].bottomOffset = 0;
                OF_Prefs::profiles[i].leftOffset = 0;
                OF_Prefs::profiles[i].rightOffset = 0;
            }
        
            if(OF_Prefs::profiles[i].irSens > DFRobotIRPositionEx::Sensitivity_Max)
                OF_Prefs::profiles[i].irSens = DFRobotIRPositionEx::Sensitivity_Default;

            if(OF_Prefs::profiles[i].runMode >= FW_Const::RunMode_Count)
                OF_Prefs::profiles[i].runMode = FW_Const::RunMode_Normal;
        }

        // if selected profile is out of range, fallback to a default instead.
        if(OF_Prefs::currentProfile >= PROFILE_COUNT)
            OF_Prefs::currentProfile = 0;

        // set the run mode
        if(OF_Prefs::profiles[OF_Prefs::currentProfile].runMode < FW_Const::RunMode_Count)
            FW_Common::runMode = (FW_Const::RunMode_e)OF_Prefs::profiles[OF_Prefs::currentProfile].runMode;

        OF_Prefs::Load();
        
        #if defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)
            uint8_t aux_espnow_wifi_channel, aux_espnow_wifi_power;
            if (OF_Prefs::LoadWireless(&aux_espnow_wifi_channel, &aux_espnow_wifi_power) == OF_Prefs::Error_Success) {
                espnow_wifi_channel = aux_espnow_wifi_channel;
                espnow_wifi_power = aux_espnow_wifi_power;
            }
            /*
            else {
                OF_Prefs::SaveWireless(&espnow_wifi_channel, &espnow_wifi_power)
            }
            */
            if (OF_Prefs::LoadLastDongleWireless(lastDongleAddress) == OF_Prefs::Error_Success) lastDongleSave = true;
                else lastDongleSave = false;

        #endif // defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)
        
    } else Serial.printf("%c%c (No Storage Available)", OF_Const::sError, (char)OF_Prefs::Error_NoStorage);
    
    

    // ===== 696969 per trasmettere i dati wireless al dongle ========
    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
        strncpy(usb_data_wireless.deviceManufacturer,MANUFACTURER_NAME,sizeof(usb_data_wireless.deviceManufacturer));
        strncpy(usb_data_wireless.deviceName,DEVICE_NAME, sizeof(usb_data_wireless.deviceName)); // cambia
        usb_data_wireless.deviceVID = DEVICE_VID;
        usb_data_wireless.devicePID = PLAYER_NUMBER; // cambia
        usb_data_wireless.devicePlayer = PLAYER_NUMBER;
        usb_data_wireless.channel = espnow_wifi_channel;
    #endif // defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
    // ===============================================================

 
    // We're setting our custom USB identifiers, as defined in the configuration area!
    #ifdef USE_TINYUSB
        // Initializes TinyUSB identifier
        if (!TinyUSBDevice.isInitialized()) { // 696969 aggiunto ..funzionava lo stesso, ma così è più sicuro .. sicuramente serve per Esp32 con libreria non integrfata nel core
            TinyUSBDevice.begin(0);
        }
        // Values are pulled from EEPROM values that were loaded earlier in setup()
        TinyUSBDevice.setManufacturerDescriptor(MANUFACTURER_NAME);

        if(OF_Prefs::usb.devicePID) {
            TinyUSBDevice.setID(DEVICE_VID, OF_Prefs::usb.devicePID);
            #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
                usb_data_wireless.devicePID = OF_Prefs::usb.devicePID;
                usb_data_wireless.devicePlayer = OF_Prefs::usb.devicePID;
            #endif // defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)    
            if(OF_Prefs::usb.deviceName[0] == '\0')
                 TinyUSBDevice.setProductDescriptor(DEVICE_NAME);
            else {
                TinyUSBDevice.setProductDescriptor(OF_Prefs::usb.deviceName);
                #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
                    strncpy(usb_data_wireless.deviceName, OF_Prefs::usb.deviceName, sizeof(usb_data_wireless.deviceName));
                #endif // defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)    
            }
        } else {
            TinyUSBDevice.setProductDescriptor(DEVICE_NAME);
            TinyUSBDevice.setID(DEVICE_VID,
            #ifdef PLAYER_NUMBER
            PLAYER_NUMBER
            #else
            1
            #endif // PLAYER_NUMBER
            );
        }

#endif //USE_TINYUSB


// ===================================================================================================================
// ===================================================================================================================
// ====== 696969 ============ spostato sopra prima della connessione =================================================

    // this is needed for both customs and builtins, as defaults are all uninitialized
    FW_Common::UpdateBindings(true);

    // Initialize DFRobot Camera Wires & Object
    FW_Common::CameraSet();

    // initialize buttons & feedback devices
    FW_Common::buttons.Begin();
    FW_Common::FeedbackSet();

    // Load saved input mode settings
    FW_Common::buttons.analogOutput = OF_Prefs::toggles[OF_Const::analogOutputMode];
    FW_Common::OLED.mister = OF_Prefs::toggles[OF_Const::misterMode];
    
    #ifdef LED_ENABLE
        OF_RGB::LedInit();
    #endif // LED_ENABLE

    ////////////////////////////////////////////////////////////
    // scrive sulla parte superiore del diplay "connessione"
    #ifdef ARDUINO_ARCH_ESP32
    #ifdef USES_DISPLAY
        //FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_Init);
        FW_Common::OLED.TopPanelUpdate(" ... CONNECTION ...");
        
        // =================== parte poi da rimuovere ====================
        #ifdef CLOCK_CAM_WII
        char buffer[50];
        sprintf(buffer, "Hz: %d - DC: %d ", tone_freq_main, tone_duty_main);
        FW_Common::OLED.TopPanelUpdate(buffer);
        #endif // CLOCK_CAM_WII
        // ================== fine parte da rimovere =====================

    #endif // USES_DISPLAY
    #endif //ARDUINO_ARCH_ESP32



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 696969 == CODICE PER CALIBRARE LEVETTA STICK IN POSIZIONE CENTRALE =============
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #if defined(ARDUINO_ARCH_ESP32) && defined(USES_ANALOG)   // la facciamo solo per ESP32 e lasciamo RP2040 come gestione originale
    uint16_t analogValueX;
    uint16_t analogValueY;
    //unsigned long startTime = 0;
    unsigned long startTime = millis();
    while ((millis()-startTime) < 2000)
    {
        analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
        analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);
        if (analogValueX > ANALOG_STICK_DEADZONE_X_MAX) ANALOG_STICK_DEADZONE_X_MAX = analogValueX;
        if (analogValueX < ANALOG_STICK_DEADZONE_X_MIN) ANALOG_STICK_DEADZONE_X_MIN = analogValueX;
        if (analogValueY > ANALOG_STICK_DEADZONE_Y_MAX) ANALOG_STICK_DEADZONE_Y_MAX = analogValueY;
        if (analogValueY < ANALOG_STICK_DEADZONE_Y_MIN) ANALOG_STICK_DEADZONE_Y_MIN = analogValueY;
    }
    
    ANALOG_STICK_DEADZONE_X_CENTER = (ANALOG_STICK_DEADZONE_X_MIN + ANALOG_STICK_DEADZONE_X_MAX) / 2; 
    ANALOG_STICK_DEADZONE_Y_CENTER = (ANALOG_STICK_DEADZONE_Y_MIN + ANALOG_STICK_DEADZONE_Y_MAX) / 2; 
    
    ANALOG_STICK_DEADZONE_X_MIN -= 400;
    ANALOG_STICK_DEADZONE_X_MAX += 400;
    ANALOG_STICK_DEADZONE_Y_MIN -= 400;
    ANALOG_STICK_DEADZONE_Y_MAX += 400;
    #endif
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 696969 == FINE CODICE CALIBRAZIONE STICK ========================================
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ===================================================================================================================

#ifdef USE_TINYUSB
        #if defined(ARDUINO_RASPBERRY_PI_PICO_W) && defined(ENABLE_CLASSIC)
        // is VBUS (USB voltage) detected?
        if(digitalRead(34) || true) { // digitalRead(34) non funziona e non è il metodo corretto
            // If so, we're connected via USB, so initializing the USB devices chunk.
            TinyUSBDevices.begin(POLL_RATE); // 696969 inserito questo al posto di quello sotto
            // TUSBDeviceSetup.begin(1); // 696969 tolto ancora non fatta completa transizione
            // wait until device mounted
            while(!USBDevice.mounted()) { yield(); }
            Serial.begin(9600);
            Serial.setTimeout(0);
            Serial_OpenFIRE_Stream = &Serial; // 696969 inserito da me
        } else {
            // Else, we're on batt, so init the Bluetooth chunks.
            if(OF_Prefs::usb.deviceName[0] == '\0')
                TinyUSBDevices.beginBT(DEVICE_NAME, DEVICE_NAME);
            else TinyUSBDevices.beginBT(OF_Prefs::usb.deviceName, OF_Prefs::usb.deviceName);
        }   
    #elif defined(ARDUINO_ARCH_RP2040) // 696969 messo per mantenere vecchio codice
        // Initializing the USB devices chunk.
        TinyUSBDevices.begin(POLL_RATE);
        // wait until device mounted
        while(!USBDevice.mounted()) { yield(); }
        Serial.begin(9600);   // 9600 = 1ms data transfer rates, default for MAMEHOOKER COM devices.
        Serial.setTimeout(0);
        Serial_OpenFIRE_Stream = &Serial; // 696969 inserito da me
    #else // 696969 inserito da me // ARDUINO_ARCH_ESP32
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // === 696969 === NUOVA GESTIONE INIZIALIZZAIZONE USB O CONNESSIONE WIRELESS =============================
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    TinyUSBDevices.begin(POLL_RATE);
    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)// SE WIRELESS
        #define MILLIS_TIMEOUT  1000 //1 secondi
        unsigned long lastMillis = millis ();
        while ((millis () - lastMillis <= MILLIS_TIMEOUT) && (!TinyUSBDevice.mounted())) { yield(); }
        if (!TinyUSBDevice.mounted()) {
            // Show wireless initialization start
            #ifdef USES_DISPLAY
                FW_Common::OLED.TopPanelUpdate("Init...");
            #endif
            
            // 频段选择功能
            #ifdef USES_DISPLAY
                // 显示频段选择提示
                FW_Common::OLED.TopPanelUpdate("Press TRIGGER to change CH");
                
                unsigned long lastTriggerTime = millis();
                int countdown = 5;
                currentChannel = espnow_wifi_channel;
                bool channelChanged = false;
                char lastStatusText[20] = "";
                
                // 等待5秒内没有检测到Trigger按键按下
                // 直接读取Trigger按键的引脚状态，绕过LightgunButtons的状态管理
                pinMode(OF_Prefs::pins[OF_Const::btnTrigger], INPUT_PULLUP);
                bool lastTriggerState = HIGH;
                
                while (millis() - lastTriggerTime < 5000) {
                    // 直接读取引脚状态
                    bool currentTriggerState = digitalRead(OF_Prefs::pins[OF_Const::btnTrigger]);
                    
                    // 检测按键按下（从高到低变化）
                    if (lastTriggerState == HIGH && currentTriggerState == LOW) {
                        // 切换频段
                        currentChannel++;
                        if (currentChannel > 12) currentChannel = 1;
                        channelChanged = true;
                        
                        // 重置倒计时
                        lastTriggerTime = millis();
                        
                        // 提供视觉反馈
                        #ifdef LED_ENABLE
                            OF_RGB::LedUpdate(0,255,0);
                            delay(100);
                            OF_RGB::LedOff();
                        #endif
                    }
                    
                    // 保存当前状态用于下一次检测
                    lastTriggerState = currentTriggerState;
                    
                    // 短暂延迟避免过于频繁的检测
                    delay(20);
                    
                    // 更新倒计时
                    int newCountdown = 5 - ((millis() - lastTriggerTime) / 1000);
                    
                    // 仅在倒计时改变或状态改变时更新显示
                    if (newCountdown != countdown || channelChanged) {
                        countdown = newCountdown;
                        char statusText[20];
                        sprintf(statusText, "CH:%d (T:%ds)", currentChannel, countdown);
                        
                        // 只有当文本内容变化时才更新显示，避免闪屏
                        if (strcmp(statusText, lastStatusText) != 0) {
                            FW_Common::OLED.TopPanelUpdate(statusText);
                            strcpy(lastStatusText, statusText);
                            channelChanged = false;
                        }
                    }
                    
                    // 短暂延迟避免过于频繁的按键检测
                    delay(50);
                }
                
                // 更新频段设置
                espnow_wifi_channel = currentChannel;
                
                // 保存新的频段设置
                OF_Prefs::SaveWireless(&espnow_wifi_channel, &espnow_wifi_power);

                
                // 显示最终选择的频段
                char finalChannelText[20];
                sprintf(finalChannelText, "CH:%d", currentChannel);
                FW_Common::OLED.TopPanelUpdate(finalChannelText);
                delay(500);
            #endif

            // 显示初始化完成            
            // Begin wireless initialization after channel selection
            #ifdef USES_DISPLAY
                FW_Common::OLED.TopPanelUpdate("Starting...");
            #endif
            SerialWireless.begin();            
            if (lastDongleSave) {
                // Show connecting to last dongle
                #ifdef USES_DISPLAY
                    FW_Common::OLED.TopPanelUpdate("Last conn...");
                #endif
                
                // PROVA A CONNETTERTI AL PRECEDENTE DONGLE INVIANDO IL PACCHETTO CHECK_CONNECTION
                if (SerialWireless.connection_gun_at_last_dongle()) {
                    // Show last dongle connection success
                    #ifdef USES_DISPLAY
                        FW_Common::OLED.TopPanelUpdate("Success!");
                    #endif
                } else {
                    // Show last dongle connection failed
                    #ifdef USES_DISPLAY
                        FW_Common::OLED.TopPanelUpdate("last connection Failed");
                        unsigned long failMillis = millis();
                        while (millis() - failMillis < 500) { yield(); }
                    #endif
                    
                    // Show searching for new dongle
                    #ifdef USES_DISPLAY
                        FW_Common::OLED.TopPanelUpdate("New Dongle");
                    #endif
                    
                    // Show scanning for dongle signals with channel info
                        #ifdef USES_DISPLAY
                            char scanningText[30];
                            sprintf(scanningText, "Scanning... CH:%d", espnow_wifi_channel);
                            FW_Common::OLED.TopPanelUpdate(scanningText);
                        #endif
                    
                    // Connect to new dongle and show result
                    bool connectResult = SerialWireless.connection_gun();
                    
                    #ifdef USES_DISPLAY
                        if (connectResult) {
                            FW_Common::OLED.TopPanelUpdate("new dongle Found!");
                        } else {
                            FW_Common::OLED.TopPanelUpdate("Not found");
                        }
                    #endif
                }

            }
            else {
                // Show searching for first dongle
                #ifdef USES_DISPLAY
                    FW_Common::OLED.TopPanelUpdate("First conn");
                #endif
                
                // Show scanning for dongle signals with channel info
                        #ifdef USES_DISPLAY
                            char scanningText[30];
                            sprintf(scanningText, "Scanning... CH:%d", espnow_wifi_channel);
                            FW_Common::OLED.TopPanelUpdate(scanningText);
                        #endif
                
                // Connect to first dongle and show result
                bool connectResult = SerialWireless.connection_gun();
                
                #ifdef USES_DISPLAY
                        if (connectResult) {
                            FW_Common::OLED.TopPanelUpdate("Found!  ");
                        } else {
                            FW_Common::OLED.TopPanelUpdate("Not found");
                        }
                    #endif
                
                //TinyUSBDevices.onBattery = false; // lo imposta a true solo dopo che è stata stabilita e riconosciuta connessione tra dongle e gun
                //uint8_t stato_wireless = 0;
            }
            
            // Show waiting for connection to establish with channel info
                #ifdef USES_DISPLAY
                    char waitingText[30];
                    sprintf(waitingText, "Waiting... CH:%d", espnow_wifi_channel);
                    FW_Common::OLED.TopPanelUpdate(waitingText);
                #endif
        }
    #endif // OPENFIRE_WIRELESS_ENABLE

    while(!TinyUSBDevice.mounted() && !TinyUSBDevices.onBattery) { yield();}

    // arriva qui solo se e' stato connesso l'usb o e' stata negoziata e stabilita una connessione wireless

    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE) && defined(USES_DISPLAY)
        if (TinyUSBDevices.onBattery) {
            // Show wireless connection established
            FW_Common::OLED.TopPanelUpdate("Link Up!");
            
            // Brief pause to show connection established
            unsigned long linkUpMillis = millis();
            while (millis() - linkUpMillis < 500) { yield(); }
            
            // Show wireless connection status with MAC address
            char mac_str[18];
            sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                    peerAddress[0], peerAddress[1], peerAddress[2], 
                    peerAddress[3], peerAddress[4], peerAddress[5]);
            FW_Common::OLED.TopPanelUpdate("Connected ", mac_str);
        }
    #endif

    if (TinyUSBDevice.mounted()) {
        Serial.begin(9600);
        Serial.setTimeout(0);
        #if defined(ARDUINO_ARCH_ESP32)
            //// tolto perchè con nuova versione arduino-esp32 o tinyUSB non funziona più bene ///// Serial.setTxTimeoutMs(0);
            //////////////////////Serial.setTxTimeoutMs(0); // default è 250ms // serve per fare come in arduino pico rp2040
            //Serial.setRxBufferSize(64); // impostato con per arduino pico .. se non si imposta è 256 di default
            //Serial.setRxBufferSize(usbPackageSize + 128);
            //Serial.setTxBufferSize(64);
        #endif // ARDUINO_ARCH_ESP32
        #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
            if (TinyUSBDevices.onBattery) {  // nel caso incredibile che l'USB sia montato nel momnto esatto in cui è stata stabilita connessione wireless
                TinyUSBDevices.onBattery = false;
                //SerialWireless.end();
            }
            if (TinyUSBDevices.wireless_mode != WIRELESS_MODE::NONE_WIRELESS) SerialWireless.end();
        #endif 
        Serial_OpenFIRE_Stream = & Serial;
        //Serial_OpenFIRE_Stream->setTimeout(0);  
        //(*Serial_OpenFIRE_Stream).setTimeout(0); 

    }  
    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
    else {     
        if (!lastDongleSave || 
            (lastDongleSave && !(memcmp(lastDongleAddress, peerAddress,6) == 0))) OF_Prefs::SaveLastDongleWireless(peerAddress);
        // CHIUDI TUTTO CIO' CHE E' USB SE E' DA CHIUDERE
        TinyUSBDevice.clearConfiguration();
        TinyUSBDevice.detach();
        Serial_OpenFIRE_Stream = &SerialWireless;
    }
    #endif

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // === 696969 === FINE NUOVA GESTIONE INIZIALIZZAIZONE USB O CONNESSIONE WIRELESS ========================
    ////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    #endif
#endif //USE_TINYUSB

    // ripristina la parte superiore del diplay come in originale
    #ifdef ARDUINO_ARCH_ESP32
    #ifdef USES_DISPLAY
        FW_Common::OLED.ScreenModeChange(ExtDisplay::ScreenMode_e::Screen_None);
        //FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_Init);
        //FW_Common::OLED.TopPanelUpdate(" ... CONNECTION ...");
    #endif // USES_DISPLAY
    #endif //ARDUINO_ARCH_ESP32




    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// 696969 ////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    #ifdef OPENFIRE_WIRELESS_ENABLE
        //extern Stream* Serial_OpenFIRE_Stream;
        #ifdef Serial
            //#define AUX_SERIAL Serial
            #undef Serial
        #endif
        #define Serial (*Serial_OpenFIRE_Stream)
    #endif // OPENFIRE_WIRELESS_ENABLE
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// 696969 ////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




    /////////////////// AbsMouse5.init(true); // 696969 rimosso per il mio nuovo OpenFire_TinyDevice

    // IR camera maxes out motion detection at ~300Hz, and millis() isn't good enough
    
    if (TinyUSBDevices.onBattery) startIrCamTimer(209);  // 120 ---- 100->10ms 66 -> 15ms per connessione wireless
      else startIrCamTimer(209); // 5ms per connessione via seriale
    
    FW_Common::OpenFIREper.source(OF_Prefs::profiles[OF_Prefs::currentProfile].adjX,
                                  OF_Prefs::profiles[OF_Prefs::currentProfile].adjY);
    FW_Common::OpenFIREper.deinit(0);

    // First boot sanity checks; all zeroes are initial config
    if((OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset    == 0 &&
        OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset == 0 && 
        OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset   == 0 &&
        OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset  == 0)) {

        // This is a first boot! Prompt to start calibration.
        unsigned int timerIntervalShort = 600;
        unsigned int timerInterval = 1000;
        OF_RGB::LedOff();
        unsigned long lastT = millis();
        bool LEDisOn = false;

        #ifdef USES_DISPLAY
            FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_Init);
        #endif // USES_DISPLAY

        while(!(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Trigger) || FW_Common::camNotAvailable) {
            // Check and process serial commands, in case user needs to change EEPROM settings.
            if(Serial.available())
                OF_Serial::SerialProcessingDocked();
            
            if(FW_Common::gunMode == FW_Const::GunMode_Docked) {
                ExecGunModeDocked();

                // Because the app offers cali options, exit straight to normal runmode
                // if we exited from docking with a setup profile.
                if(!(OF_Prefs::profiles[OF_Prefs::currentProfile].topOffset == 0 &&
                     OF_Prefs::profiles[OF_Prefs::currentProfile].bottomOffset == 0 && 
                     OF_Prefs::profiles[OF_Prefs::currentProfile].leftOffset == 0 &&
                     OF_Prefs::profiles[OF_Prefs::currentProfile].rightOffset == 0)) {
                      FW_Common::SetMode(FW_Const::GunMode_Run);
                      break;
                #ifdef USES_DISPLAY
                } else { FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_Init);
                #endif // USES_DISPLAY
                }
            }

            FW_Common::buttons.Poll(1);
            FW_Common::buttons.Repeat();

            // LED update:
            if(LEDisOn) {
                unsigned long t = millis();
                if(t - lastT > timerInterval) {
                    OF_RGB::LedOff();
                    LEDisOn = false;
                    lastT = millis();
                }
            } else {
                unsigned long t = millis();
                if(t - lastT > timerIntervalShort) {
                    OF_RGB::LedUpdate(255,125,0);
                    LEDisOn = true;
                    lastT = millis();
                } 
            }
        }

        // skip cali if we're prematurely set to run from the above loop
        // (i.e. cal'd from Desktop)
        if(FW_Common::gunMode != FW_Const::GunMode_Run)
            FW_Common::SetMode(FW_Const::GunMode_Calibration);
    } else {
        // this will turn off the DotStar/RGB LED and ensure proper transition to Run
        FW_Common::SetMode(FW_Const::GunMode_Run);
    }
}

#ifdef ARDUINO_ARCH_ESP32
void startIrCamTimer(const int &frequencyHz)
{  
    My_timer = timerBegin(1000000);      
    timerAttachInterrupt(My_timer, &esp32s3pwmIrq);
    timerAlarm(My_timer, (uint64_t) 1000000 / frequencyHz, true, 0);
}

void ARDUINO_ISR_ATTR esp32s3pwmIrq(void)
{
    FW_Common::irPosUpdateTick = 1;
}
#endif


#ifdef ARDUINO_ARCH_RP2040
void startIrCamTimer(const int &frequencyHz)
{
    rp2040EnablePWMTimer(0, frequencyHz);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, rp2040pwmIrq);
    irq_set_enabled(PWM_IRQ_WRAP, true);
}

void rp2040EnablePWMTimer(const unsigned int &slice_num, const unsigned int &frequency)
{
    pwm_config pwmcfg = pwm_get_default_config();
    float clkdiv = (float)clock_get_hz(clk_sys) / (float)(65535 * frequency);
    if(clkdiv < 1.0f) {
        clkdiv = 1.0f;
    } else {
        // really just need to round up 1 lsb
        clkdiv += 2.0f / (float)(1u << PWM_CH1_DIV_INT_LSB);
    }
    
    // set the clock divider in the config and fetch the actual value that is used
    pwm_config_set_clkdiv(&pwmcfg, clkdiv);
    clkdiv = (float)pwmcfg.div / (float)(1u << PWM_CH1_DIV_INT_LSB);
    
    // calculate wrap value that will trigger the IRQ for the target frequency
    pwm_config_set_wrap(&pwmcfg, (float)clock_get_hz(clk_sys) / (frequency * clkdiv));
    
    // initialize and start the slice and enable IRQ
    pwm_init(slice_num, &pwmcfg, true);
    pwm_set_irq_enabled(slice_num, true);
}

void rp2040pwmIrq(void)
{
    pwm_hw->intr = 0xff;
    FW_Common::irPosUpdateTick = 1;
}
#endif // ARDUINO_ARCH_RP2040 // 696969 messo qui

#ifdef DUAL_CORE
// Second core setup
// does... nothing, since timing is kinda important.
void setup1()
{
    // i sleep
}

// Second core main loop
// currently handles all button & serial processing when Core 0 is in ExecRunMode()
void loop1()
{
    #ifdef ARDUINO_ARCH_ESP32
        esp32_fifo.pop();
    #else // rp2040
    rp2040.fifo.pop();
    #endif
    while(FW_Common::gunMode == FW_Const::GunMode_Run) {
        // All buttons' outputs except for the trigger are processed here.
        FW_Common::buttons.Poll(0);

        #ifdef USES_TEMP
            if(OF_Prefs::pins[OF_Const::tempPin] > -1)
                OF_FFB::TemperatureUpdate();
        #endif // USES_TEMP

        // For processing the trigger specifically.
        // (FW_Common::buttons.debounced is a binary variable intended to be read 1 bit at a time,
        // with the 0'th point == rightmost == decimal 1 == trigger, 3 = start, 4 = select)
        if(bitRead(FW_Common::buttons.debounced, FW_Const::BtnIdx_Trigger))   // Check if we pressed the Trigger this run.
            TriggerFire();                                  // Handle button events and feedback ourselves.
        else TriggerNotFire();                              // Releasing button inputs and sending stop signals to feedback devices.

        if(millis() - lastUSBpoll >= POLL_RATE) {
            #ifdef USES_ANALOG
                if(FW_Common::analogIsValid) AnalogStickPoll();
            #endif // USES_ANALOG
            lastUSBpoll = millis();
            FW_Common::buttons.SendReports();
        }

        if(Serial.available()) OF_Serial::SerialProcessing();

        #ifdef MAMEHOOKER
            if(OF_Serial::serialMode) OF_Serial::SerialHandling();                                   // Process the force feedback from the current queue.
        #endif // MAMEHOOKER
        
        if(FW_Common::buttons.pressedReleased == FW_Const::EscapeKeyBtnMask)
            SendEscapeKey();

        if(OF_Prefs::toggles[OF_Const::holdToPause]) {
            if(FW_Common::buttons.debounced == FW_Const::EnterPauseModeHoldBtnMask
               && !FW_Common::lastSeen && !pauseHoldStarted) {
                pauseHoldStarted = true;
                pauseHoldStartstamp = millis();
                if(!OF_Serial::serialMode)
                    Serial.println("Started holding pause mode signal buttons!");

            } else if(pauseHoldStarted && (FW_Common::buttons.debounced != FW_Const::EnterPauseModeHoldBtnMask || FW_Common::lastSeen)) {
                pauseHoldStarted = false;
                if(!OF_Serial::serialMode)
                    Serial.println("Either stopped holding pause mode buttons, aimed onscreen, or pressed other buttons");

            } else if(pauseHoldStarted) {
                unsigned long t = millis();
                if(t - pauseHoldStartstamp > OF_Prefs::settings[OF_Const::holdToPauseLength]) {
                    // in the infinitely tiny chance that this happens at the same time as a serial-pinged Dock request:
                    if(FW_Common::gunMode == FW_Const::GunMode_Run) {
                        // Signal the main core to set mode, since it's more stable there.
                        // Pop blocks until we get the okay from the main core (the value returned doesn't matter atm)
                        #ifdef ARDUINO_ARCH_ESP32
                            esp32_fifo.push(FW_Const::GunMode_Pause);
                            esp32_fifo.pop();
                        #else // rp2040 
                        rp2040.fifo.push(FW_Const::GunMode_Pause);
                        rp2040.fifo.pop();
                        #endif
                    }
                }
            }
        } else {
            if(FW_Common::buttons.pressedReleased == FW_Const::EnterPauseModeBtnMask ||
               FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Home) {
                // in the infinitely tiny chance that this happens at the same time as a serial-pinged Dock request:
                if(FW_Common::gunMode == FW_Const::GunMode_Run) {
                    // Signal the main core to set mode, since it's more stable there.
                    // Pop blocks until we get the okay from the main core (the value returned doesn't matter atm)
                    #ifdef ARDUINO_ARCH_ESP32
                        esp32_fifo.push(FW_Const::GunMode_Pause);
                        esp32_fifo.pop();
                    #else // rp2040                    
                    rp2040.fifo.push(FW_Const::GunMode_Pause);
                    rp2040.fifo.pop();
                    #endif
                }
            }
        }
    }
}
#endif // DUAL_CORE
// #endif // ARDUINO_ARCH_RP2040 // 696969 tolto messo sopra

// Main core events hub
// splits off into subsequent ExecModes depending on circumstances
void loop()
{
    // poll/update button states with 1ms interval so debounce mask is more effective
    FW_Common::buttons.Poll(1);
    FW_Common::buttons.Repeat();

    if(OF_Prefs::toggles[OF_Const::holdToPause] && pauseHoldStarted) {
        #ifdef USES_RUMBLE
            if(OF_Prefs::toggles[OF_Const::rumble]) {
                analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
                delay(300);
                #ifdef ARDUINO_ARCH_ESP32
                    analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0); // 696969 per ESP32
                #else //rp2040
                digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
                #endif
            }
        #endif // USES_RUMBLE

        // if any buttons are still held, keep polling until all buttons are debounced
        while(FW_Common::buttons.debounced)
            FW_Common::buttons.Poll(1);

        pauseHoldStarted = false;
        pauseModeSelectingProfile = false;
    }

    #ifdef MAMEHOOKER
        if(Serial.available()) OF_Serial::SerialProcessing();
    #endif // MAMEHOOKER

    switch(FW_Common::gunMode) {
        case FW_Const::GunMode_Pause:
            if(OF_Prefs::toggles[OF_Const::simplePause]) {
                if(pauseModeSelectingProfile) {
                    if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_A) {
                        SetProfileSelection(false);
                    } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_B) {
                        SetProfileSelection(true);
                    } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Trigger) {
                        FW_Common::SelectCalProfile(profileModeSelection);
                        pauseModeSelectingProfile = false;
                        FW_Common::pauseModeSelection = FW_Const::PauseMode_Calibrate;

                        if(!OF_Serial::serialMode) {
                            Serial.print("Switched to profile: ");
                            Serial.println(OF_Prefs::profiles[OF_Prefs::currentProfile].name);
                            Serial.println("Going back to the main menu...");
                            Serial.println("Selecting: Calibrate current profile");
                        }

                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_Calibrate);
                        #endif // USES_DISPLAY

                    } else if(FW_Common::buttons.pressedReleased & FW_Const::ExitPauseModeBtnMask) {
                        if(!OF_Serial::serialMode)
                            Serial.println("Exiting profile selection.");

                        pauseModeSelectingProfile = false;

                        #ifdef LED_ENABLE
                            for(uint i = 0; i < 2; ++i) {
                                OF_RGB::LedUpdate(180,180,180);
                                delay(125);
                                OF_RGB::LedOff();
                                delay(100);
                            }
                            OF_RGB::LedUpdate(255,0,0);
                        #endif // LED_ENABLE

                        FW_Common::pauseModeSelection = FW_Const::PauseMode_Calibrate;

                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_Calibrate);
                        #endif // USES_DISPLAY
                    }
                } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_A) {
                    SetPauseModeSelection(false);
                } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_B) {
                    SetPauseModeSelection(true);
                } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Trigger) {
                    switch(FW_Common::pauseModeSelection) {
                        case FW_Const::PauseMode_Calibrate:
                          FW_Common::SetMode(FW_Const::GunMode_Calibration);
                          if(!OF_Serial::serialMode) {
                              Serial.print("Calibrating for current profile: ");
                              Serial.println(OF_Prefs::profiles[OF_Prefs::currentProfile].name);
                          }
                          break;
                        case FW_Const::PauseMode_ProfileSelect:
                          if(!OF_Serial::serialMode) {
                              Serial.println("Pick a profile!");
                              Serial.print("Current profile in use: ");
                              Serial.print(OF_Prefs::profiles[OF_Prefs::currentProfile].name);
                              Serial.print(" (");
                              Serial.print(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout ? "Diamond" : "Square");
                              Serial.println(")");
                          }
                          pauseModeSelectingProfile = true;
                          profileModeSelection = OF_Prefs::currentProfile;
                          #ifdef USES_DISPLAY
                              // 为每个配置文件名称添加布局类型信息
                              char profileNames[4][20]; // 为每个配置文件创建带布局信息的名称
                              for(int i = 0; i < PROFILE_COUNT; i++) {
                                  // 根据irLayout值添加布局类型
                                  if(OF_Prefs::profiles[i].irLayout) {
                                      snprintf(profileNames[i], sizeof(profileNames[i]), "%s (Diamond)", OF_Prefs::profiles[i].name);
                                  } else {
                                      snprintf(profileNames[i], sizeof(profileNames[i]), "%s (Square)", OF_Prefs::profiles[i].name);
                                  }
                              }
                              FW_Common::OLED.PauseProfileUpdate(profileModeSelection, profileNames[0], profileNames[1], profileNames[2], profileNames[3]);
                          #endif // USES_DISPLAY
                          #ifdef LED_ENABLE
                              OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);
                          #endif // LED_ENABLE
                          break;
                        case FW_Const::PauseMode_Save:
                          if(!OF_Serial::serialMode)
                              Serial.println("Saving...");
                          FW_Common::SavePreferences();
                          break;
                        case FW_Const::PauseMode_AutofireToggle:
                          if(!OF_Serial::serialMode) {
                              Serial.println("Toggling autofire...");
                          }
                          OF_Prefs::toggles[OF_Const::autofire] = !OF_Prefs::toggles[OF_Const::autofire];
                          if(!OF_Serial::serialMode) {
                              Serial.print("Autofire is now ");
                              Serial.println(OF_Prefs::toggles[OF_Const::autofire] ? "ON" : "OFF");
                          }
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AutofireToggle);
                          #endif // USES_DISPLAY
                          #ifdef LED_ENABLE
                              if(OF_Prefs::toggles[OF_Const::autofire]) {
                                  OF_RGB::LedUpdate(0,255,0);
                              } else {
                                  OF_RGB::LedUpdate(255,0,0);
                              }
                              delay(200);
                              OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);
                          #endif // LED_ENABLE
                          OF_Prefs::SaveToggles();
                          break;                          
 //wei13402 add start
                         case FW_Const::PauseMode_ModeChange:
                          if(!OF_Serial::serialMode) {
                              Serial.println("Changing input mode...");
                          }
                          // Cycle through modes: Mouse/KB -> Gamepad -> MiSTer Optimized -> Mouse/KB
                          if(!FW_Common::buttons.analogOutput && !FW_Common::OLED.mister) {
                              // Current: Mouse/KB -> Switch to Gamepad
                              FW_Common::buttons.analogOutput = true;
                              if(!OF_Serial::serialMode)
                                  Serial.println("Mode changed to: Gamepad");
                          } else if(FW_Common::buttons.analogOutput && !FW_Common::OLED.mister) {
                              // Current: Gamepad -> Switch to MiSTer Optimized
                              FW_Common::OLED.mister = true;
                              if(!OF_Serial::serialMode)
                                  Serial.println("Mode changed to: MiSTer Optimized");
                          } else {
                              // Current: MiSTer Optimized -> Switch to Mouse/KB
                              FW_Common::buttons.analogOutput = false;
                              FW_Common::OLED.mister = false;
                              if(!OF_Serial::serialMode)
                                  Serial.println("Mode changed to: Mouse/Keyboard");
                          }
                          
                          // Save the new mode settings
                          OF_Prefs::toggles[OF_Const::analogOutputMode] = FW_Common::buttons.analogOutput;
                          OF_Prefs::toggles[OF_Const::misterMode] = FW_Common::OLED.mister;
                          OF_Prefs::SaveToggles();
                          
                          // Update display to show new mode
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_ModeChange);
                          #endif // USES_DISPLAY
                          // Provide visual feedback
                          #ifdef LED_ENABLE
                              for(uint i = 0; i < 2; ++i) {
                                  OF_RGB::LedUpdate(0,255,255);
                                  delay(150);
                                  OF_RGB::LedOff();
                                  delay(100);
                              }
                          #endif // LED_ENABLE
                          break;
 //wei13102 add end
 //wei134102 add start
                         
                        case FW_Const::PauseMode_LowButtonToggle:
                          if(!OF_Serial::serialMode) {
                              Serial.println("Toggling Low Button Mode!");
                          }
                          OF_Prefs::toggles[OF_Const::lowButtonsMode] = !OF_Prefs::toggles[OF_Const::lowButtonsMode];
                          if(!OF_Serial::serialMode) {
                              Serial.print("Low Button Mode ");
                              Serial.println(OF_Prefs::toggles[OF_Const::lowButtonsMode] ? "ON" : "OFF");
                          }
                          // Update display to show new state
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_LowButtonToggle);
                          #endif // USES_DISPLAY
                          // Provide visual feedback
                          #ifdef LED_ENABLE
                              for(uint i = 0; i < 2; ++i) {
                                  OF_RGB::LedUpdate(255,165,0);
                                  delay(150);
                                  OF_RGB::LedOff();
                                  delay(100);
                              }
                          #endif // LED_ENABLE
                          OF_Prefs::SaveToggles();
                          // 更新按钮绑定以应用Low Button模式变更
                          FW_Common::UpdateBindings(false);
                          break;
                        case FW_Const::PauseMode_LayoutToggle:
                          if(!OF_Serial::serialMode) {
                              Serial.println("Toggling Layout Type!");
                          }
                          // Toggle between Diamond and Square layout
                          if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond) {
                              OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout = OF_Const::layoutSquare;
                          } else {
                              OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout = OF_Const::layoutDiamond;
                          }
                          
                          if(!OF_Serial::serialMode) {
                              Serial.print("Layout changed to: ");
                              Serial.println(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond ? "Diamond" : "Square");
                          }
                          
                          // Update display to show new layout
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_LayoutToggle);
                          #endif // USES_DISPLAY
                          
                          // Provide visual feedback with color corresponding to layout type
                          #ifdef LED_ENABLE
                              if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond) {
                                  // Diamond layout - purple
                                  OF_RGB::LedUpdate(128,0,128);
                              } else {
                                  // Square layout - blue
                                  OF_RGB::LedUpdate(0,0,255);
                              }
                              delay(500);
                              OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);
                          #endif // LED_ENABLE
                          
                          // Save the layout change
                          OF_Prefs::SaveProfiles();
                          
                          break;                          
 //wei134102 add end                             
                        #ifdef USES_RUMBLE
                        case FW_Const::PauseMode_RumbleFFToggle:
                          if(!OF_Serial::serialMode)
                              Serial.println("Toggling Rumble FFB!");
                          // Toggle rumbleFF state
                          OF_Prefs::toggles[OF_Const::rumbleFF] = !OF_Prefs::toggles[OF_Const::rumbleFF];
                          
                          // Save the setting
                          OF_Prefs::SaveToggles();
                          
                          // Update display
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_RumbleFFToggle);
                          #endif // USES_DISPLAY
                          
                          // Provide visual feedback
                          #ifdef LED_ENABLE
                              if(OF_Prefs::toggles[OF_Const::rumbleFF]) {
                                  // Green for ON
                                  OF_RGB::LedUpdate(0,255,0);
                              } else {
                                  // Red for OFF
                                  OF_RGB::LedUpdate(255,0,0);
                              }
                              delay(500);
                              OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);
                          #endif // LED_ENABLE
                          
                          break;                        
                        case FW_Const::PauseMode_RumbleToggle:
                          if(!OF_Serial::serialMode)
                              Serial.println("Toggling rumble!");
                          RumbleToggle();
                          break;
                        #endif // USES_RUMBLE
                        #ifdef USES_SOLENOID
                        case FW_Const::PauseMode_SolenoidToggle:
                          if(!OF_Serial::serialMode)
                              Serial.println("Toggling solenoid!");
                          SolenoidToggle();
                          break;
                        #endif // USES_SOLENOID
                        /*
                        #ifdef USES_SOLENOID
                        case FW_Const::PauseMode_BurstFireToggle:
                          Serial.println("Toggling solenoid burst firing!");
                          BurstFireToggle();
                          break;
                        #endif // USES_SOLENOID
                        */
                        case FW_Const::PauseMode_EscapeSignal:
                          SendEscapeKey();

                          #ifdef USES_DISPLAY
                              FW_Common::OLED.TopPanelUpdate("Sent Escape Key!");
                          #endif // USES_DISPLAY

                          #ifdef LED_ENABLE
                              for(uint i = 0; i < 3; ++i) {
                                  OF_RGB::LedUpdate(150,0,150);
                                  delay(55);
                                  OF_RGB::LedOff();
                                  delay(40);
                              }
                          #endif // LED_ENABLE

                          #ifdef USES_DISPLAY
                              FW_Common::OLED.TopPanelUpdate("Using ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);
                          #endif // USES_DISPLAY
                          break;
                        /*case FW_Const::PauseMode_Exit:
                          Serial.println("Exiting pause mode...");
                          if(FW_Common::runMode == FW_Const::RunMode_Processing) {
                              switch(OF_Prefs::profiles[OF_Prefs::currentProfile].FW_Common::runMode) {
                                  case FW_Const::RunMode_Normal:
                                    FW_Common::SetFW_Const::RunMode(FW_Const::RunMode_Normal);
                                    break;
                                  case FW_Const::RunMode_Average:
                                    FW_Common::SetFW_Const::RunMode(FW_Const::RunMode_Average);
                                    break;
                                  case FW_Const::RunMode_Average2:
                                    FW_Common::SetFW_Const::RunMode(FW_Const::RunMode_Average2);
                                    break;
                                  default:
                                    break;
                              }
                          }
                          FW_Common::SetMode(FW_Const::GunMode_Run);
                          break;
                        */
                        default:
                          Serial.println("Oops, somethnig went wrong.");
                          break;
                    }
                } else if(FW_Common::buttons.pressedReleased & FW_Const::ExitPauseModeBtnMask) {
                    if(!OF_Serial::serialMode)
                        Serial.println("Exiting pause mode...");
                    FW_Common::SetMode(FW_Const::GunMode_Run);
                }

                if(pauseExitHoldStarted && FW_Common::buttons.debounced & FW_Const::ExitPauseModeHoldBtnMask) {
                    unsigned long t = millis();
                    if(t - pauseHoldStartstamp > (OF_Prefs::settings[OF_Const::holdToPauseLength] / 2)) {
                        if(!OF_Serial::serialMode)
                            Serial.println("Exiting pause mode via hold...");

                        if(FW_Common::runMode == FW_Const::RunMode_Processing) {
                            switch(OF_Prefs::profiles[OF_Prefs::currentProfile].runMode) {
                                case FW_Const::RunMode_Normal:
                                  FW_Common::SetRunMode(FW_Const::RunMode_Normal);
                                  break;
                                case FW_Const::RunMode_Average:
                                  FW_Common::SetRunMode(FW_Const::RunMode_Average);
                                  break;
                                case FW_Const::RunMode_Average2:
                                  FW_Common::SetRunMode(FW_Const::RunMode_Average2);
                                  break;
                                default:
                                  break;
                            }
                        }

                        #ifdef USES_RUMBLE
                            if(OF_Prefs::toggles[OF_Const::rumble]) for(uint i = 0; i < 3; ++i) {
                                analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
                                delay(80);
                                #ifdef ARDUINO_ARCH_ESP32
                                    analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);  // 696969 per ESP32
                                #else // rp2040
                                digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
                                #endif
                                delay(50);
                            }
                        #endif // USES_RUMBLE

                        // if any buttons are still held, keep polling until all buttons are debounced
                        while(FW_Common::buttons.debounced)
                            FW_Common::buttons.Poll(1);

                        FW_Common::SetMode(FW_Const::GunMode_Run);
                        pauseExitHoldStarted = false;
                    }
                } else if(FW_Common::buttons.debounced & FW_Const::ExitPauseModeHoldBtnMask) {
                    pauseExitHoldStarted = true;
                    pauseHoldStartstamp = millis();
                } else if(FW_Common::buttons.pressedReleased & FW_Const::ExitPauseModeHoldBtnMask)
                    pauseExitHoldStarted = false;

            } else if(FW_Common::buttons.pressedReleased & FW_Const::ExitPauseModeBtnMask) {
                FW_Common::SetMode(FW_Const::GunMode_Run);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Trigger) {
                FW_Common::SetMode(FW_Const::GunMode_Calibration);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::RunModeNormalBtnMask) {
                FW_Common::SetRunMode(FW_Const::RunMode_Normal);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::RunModeAverageBtnMask) {
                FW_Common::SetRunMode(FW_Common::runMode == FW_Const::RunMode_Average ? FW_Const::RunMode_Average2 : FW_Const::RunMode_Average);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::IRSensitivityUpBtnMask) {
                IncreaseIrSensitivity(OF_Prefs::profiles[OF_Prefs::currentProfile].irSens);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::IRSensitivityDownBtnMask) {
                DecreaseIrSensitivity(OF_Prefs::profiles[OF_Prefs::currentProfile].irSens);
            } else if(FW_Common::buttons.pressedReleased == FW_Const::SaveBtnMask) {
                FW_Common::SavePreferences();
            #ifdef USES_RUMBLE
                } else if(FW_Common::buttons.pressedReleased == FW_Const::RumbleToggleBtnMask && OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0) {
                    RumbleToggle();
            #endif // USES_RUMBLE
            #ifdef USES_SOLENOID
                } else if(FW_Common::buttons.pressedReleased == FW_Const::SolenoidToggleBtnMask && OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0) {
                    SolenoidToggle();
            #endif // USES_SOLENOID
            } else SelectCalProfileFromBtnMask(FW_Common::buttons.pressedReleased);

            if(!OF_Serial::serialMode)
                OF_Serial::PrintResults();
            
            break;
        case FW_Const::GunMode_Docked:
            ExecGunModeDocked();
            break;
        case FW_Const::GunMode_Calibration:
            FW_Common::ExecCalMode();
            break;
        default:
            /* ---------------------- LET'S GO --------------------------- */
            switch(FW_Common::runMode) {
            case FW_Const::RunMode_Processing:
                //ExecRunModeProcessing();
                //break;
            case FW_Const::RunMode_Average:
            case FW_Const::RunMode_Average2:
            case FW_Const::RunMode_Normal:
            default:
                ExecRunMode();
                break;
            }
            break;
    }

#ifdef DEBUG_SERIAL
    OF_Serial::PrintDebugSerial();
#endif // DEBUG_SERIAL
}

/*        -----------------------------------------------        */
/* --------------------------- METHODS ------------------------- */
/*        -----------------------------------------------        */

// Main core loop
void ExecRunMode()
{
#ifdef DEBUG_SERIAL
    Serial.print("exec run mode ");
    Serial.println(FW_Const::RunModeLabels[FW_Common::runMode]);
#endif

    FW_Common::buttons.ReportEnable();
    if(FW_Common::justBooted) {
        // center the joystick so RetroArch doesn't throw a hissy fit about uncentered joysticks
        delay(100);  // Exact time needed to wait seems to vary, so make a safe assumption here.
        Gamepad16.releaseAll();
        FW_Common::justBooted = false;
    }

    #if /*defined(ARDUINO_ARCH_RP2040) &&*/ defined(DUAL_CORE)
        // wake up second core
        #ifdef ARDUINO_ARCH_ESP32
            esp32_fifo.push(0);
        #else // rp2040
        rp2040.fifo.push(0);
        #endif
    #endif // ARDUINO_ARCH_RP2040 && DUAL_CORE

    for(;;) {
        // Setting the state of our toggles, if used.
        // Only sets these values if the switches are mapped to valid pins.
        #ifdef USES_SWITCHES
            #ifdef USES_RUMBLE
                if(OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0) {
                    OF_Prefs::toggles[OF_Const::rumble] = !digitalRead(OF_Prefs::pins[OF_Const::rumbleSwitch]);
                    #ifdef MAMEHOOKER
                    if(!OF_Serial::serialMode) {
                    #endif // MAMEHOOKER
                        if(!OF_Prefs::toggles[OF_Const::rumble] && OF_FFB::rumbleHappening)
                            OF_FFB::FFBShutdown();
                    #ifdef MAMEHOOKER
                    }
                    #endif // MAMEHOOKER
                }
            #endif // USES_RUMBLE
            #ifdef USES_SOLENOID
                if(OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0) {
                    OF_Prefs::toggles[OF_Const::solenoid] = !digitalRead(OF_Prefs::pins[OF_Const::solenoidSwitch]);
                    #ifdef MAMEHOOKER
                    if(!OF_Serial::serialMode) {
                    #endif // MAMEHOOKER
                        if(!OF_Prefs::toggles[OF_Const::solenoid] && digitalRead(OF_Prefs::pins[OF_Const::solenoidPin])) {
                            OF_FFB::FFBShutdown();
                        }
                    #ifdef MAMEHOOKER
                    }
                    #endif // MAMEHOOKER
                }
            #endif // USES_SOLENOID
            if(OF_Prefs::pins[OF_Const::autofireSwitch] >= 0)
                OF_Prefs::toggles[OF_Const::autofire] = !digitalRead(OF_Prefs::pins[OF_Const::autofireSwitch]);
        #endif // USES_SWITCHES

        if(FW_Common::irPosUpdateTick) {
            FW_Common::irPosUpdateTick = 0;
            FW_Common::GetPosition();
        }
        #ifdef USES_DISPLAY
            else {
                FW_Common::OLED.IdleOps();
                #ifdef MAMEHOOKER
                    // Solenoid feedback on the second core is hella wonky when ammo updates are performed there likely due to blocking I2C transactions,
                    // so just do it here using the signal sent by it.
                    if(OF_Serial::serialDisplayChange) {
                        if(FW_Common::OLED.serialDisplayType == ExtDisplay::ScreenSerial_Ammo) {
                            FW_Common::OLED.PrintAmmo(OF_Serial::serialAmmoCount);
                        } else if(FW_Common::OLED.serialDisplayType == ExtDisplay::ScreenSerial_Life && FW_Common::OLED.lifeBar) {
                            FW_Common::OLED.PrintLife(FW_Common::dispLifePercentage);
                        } else if(FW_Common::OLED.serialDisplayType == ExtDisplay::ScreenSerial_Life) {
                            FW_Common::OLED.PrintLife(OF_Serial::serialLifeCount);
                        } else if(FW_Common::OLED.serialDisplayType == ExtDisplay::ScreenSerial_Both && FW_Common::OLED.lifeBar) {
                            FW_Common::OLED.PrintAmmo(OF_Serial::serialAmmoCount);
                            FW_Common::OLED.PrintLife(FW_Common::dispLifePercentage);
                        } else if(FW_Common::OLED.serialDisplayType == ExtDisplay::ScreenSerial_Both) {
                            FW_Common::OLED.PrintAmmo(OF_Serial::serialAmmoCount);
                            FW_Common::OLED.PrintLife(OF_Serial::serialLifeCount);
                        }

                        OF_Serial::serialDisplayChange = false;
                    }
                #endif // MAMEHOOKER
            }
        #endif // USES_DISPLAY

        // If using RP2040, we offload the button processing to the second core.
        #if /*!defined(ARDUINO_ARCH_RP2040) ||*/ !defined(DUAL_CORE)  // 696969 per ESP32     

        #ifdef USES_TEMP
            if(OF_Prefs::pins[OF_Const::tempPin] > -1)
                OF_FFB::TemperatureUpdate();
        #endif // USES_TEMP

        FW_Common::buttons.Poll(0);

        // For processing the trigger specifically.
        // (FW_Common::buttons.debounced is a binary variable intended to be read 1 bit at a time,
        // with the 0'th point == rightmost == decimal 1 == trigger, 3 = start, 4 = select)
        if(bitRead(FW_Common::buttons.debounced, FW_Const::BtnIdx_Trigger))   // Check if we pressed the Trigger this run.
            TriggerFire();                                  // Handle button events and feedback ourselves.
        else TriggerNotFire();                              // Releasing button inputs and sending stop signals to feedback devices.

        if(millis() - lastUSBpoll >= POLL_RATE) {
            #ifdef USES_ANALOG
                if(FW_Common::analogIsValid) AnalogStickPoll();
            #endif // USES_ANALOG
            lastUSBpoll = millis();
            FW_Common::buttons.SendReports();
        }

        // Run through serial receive buffer once this run, if it has contents.
        if(Serial.available()) OF_Serial::SerialProcessing();

        #ifdef MAMEHOOKER
            if(OF_Serial::serialMode) OF_Serial::SerialHandling();                                   // Process the force feedback from the current queue.
        #endif // MAMEHOOKER

        if(FW_Common::buttons.pressedReleased == FW_Const::EscapeKeyBtnMask)
            SendEscapeKey();

        if(OF_Prefs::toggles[OF_Const::holdToPause]) {
            if((FW_Common::buttons.debounced == FW_Const::EnterPauseModeHoldBtnMask)
                && !FW_Common::lastSeen && !pauseHoldStarted) {
                pauseHoldStarted = true;
                pauseHoldStartstamp = millis();
                if(!OF_Serial::serialMode)
                    Serial.println("Started holding pause mode signal buttons!");

            } else if(pauseHoldStarted && (FW_Common::buttons.debounced != FW_Const::EnterPauseModeHoldBtnMask || FW_Common::lastSeen)) {
                pauseHoldStarted = false;
                if(!OF_Serial::serialMode)
                    Serial.println("Either stopped holding pause mode buttons, aimed onscreen, or pressed other buttons");

            } else if(pauseHoldStarted) {
                unsigned long t = millis();
                if(t - pauseHoldStartstamp > OF_Prefs::settings[OF_Const::holdToPauseLength]) {
                    // MAKE SURE EVERYTHING IS DISENGAGED:
                    OF_FFB::FFBShutdown();
                    FW_Common::SetMode(FW_Const::GunMode_Pause);
                    return;
                }
            }
        } else {
            if(FW_Common::buttons.pressedReleased == FW_Const::EnterPauseModeBtnMask || FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Home) {
                // MAKE SURE EVERYTHING IS DISENGAGED:
                OF_FFB::FFBShutdown();
                FW_Common::SetMode(FW_Const::GunMode_Pause);
                return;
            }
        }
        #else  // if we're using dual cores, check the fifo.
        #ifdef ARDUINO_ARCH_ESP32
        if(esp32_fifo.pop_nb(&fifoData)) {
        #else // rp2040
        if(rp2040.fifo.pop_nb(&fifoData)) {
        #endif
            FW_Common::SetMode((FW_Const::GunMode_e)fifoData);
            fifoData = 0;
            // the value doesn't matter; all core1 is doing is waiting for any signal from the FIFO.
            #ifdef ARDUINO_ARCH_ESP32
                esp32_fifo.push(true);
            #else // rp2040
            rp2040.fifo.push(true);
            #endif
            return;
        }
        #endif // ARDUINO_ARCH_RP2040 || DUAL_CORE

#ifdef DEBUG_SERIAL
        ++frameCount;
        OF_Serial::PrintDebugSerial();
#endif // DEBUG_SERIAL
    }
}

// from Samco_4IR_Test_BETA sketch
// for use with the Samco_4IR_Processing_Sketch_BETA Processing sketch
void ExecRunModeProcessing()
{
    FW_Common::buttons.ReportDisable();

    #ifdef USES_DISPLAY
        FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_IRTest);
    #endif // USES_DISPLAY

    for(;;) {
        FW_Common::buttons.Poll(1);

        if(Serial.available()) {
            #ifdef USES_DISPLAY
                FW_Common::OLED.ScreenModeChange(ExtDisplay::Screen_Docked);
            #endif // USES_DISPLAY

            OF_Serial::SerialProcessingDocked();
        }

        if(FW_Common::runMode != FW_Const::RunMode_Processing)
            return;

        if(FW_Common::irPosUpdateTick) {
            FW_Common::irPosUpdateTick = 0;
            FW_Common::GetPosition();
        }
    }
}

// For use with the OpenFIRE app when it connects to this board.
void ExecGunModeDocked()
{
    FW_Common::buttons.ReportDisable();
    FW_Common::buttons.ReleaseAll();

    #ifdef LED_ENABLE
        OF_RGB::LedUpdate(127, 127, 255);
    #endif // LED_ENABLE

    unsigned long tempChecked = millis();
    unsigned long aStickChecked = millis();
    unsigned long currentMillis = millis();

    char buf[64];
    int pos = sprintf(&buf[0], "%.1f"
                                #ifdef GIT_HASH
                                "-%s"
                                #endif // GIT_HASH
                                , OPENFIRE_VERSION
                                #ifdef GIT_HASH
                                , GIT_HASH
                                #endif // GIT_HASH
                      );
    buf[pos++] = OF_Const::serialTerminator;
    pos += sprintf(&buf[pos], "%s", OPENFIRE_BOARD);
    buf[pos++] = OF_Const::serialTerminator;
    memcpy(&buf[pos], &OF_Prefs::usb, sizeof(OF_Prefs::USBMap_t));
    pos += sizeof(OF_Prefs::USBMap_t);
    if(FW_Common::camNotAvailable) {
        buf[pos++] = OF_Const::serialTerminator;
        buf[pos++] = OF_Const::sError;
    }
    Serial.write(buf, pos);
    Serial.flush();

    for(;;) {
        FW_Common::buttons.Poll(1);
        currentMillis = millis();

        if(!FW_Common::dockedSaving) {
            if(FW_Common::buttons.pressed) {
                for(uint i = 0; i < ButtonCount; ++i)
                    if(bitRead(FW_Common::buttons.pressed, i)) {
                        buf[0] = OF_Const::sBtnPressed, buf[1] = i;
                        Serial.write(buf, 2);
                    }
            }

            if(FW_Common::buttons.released) {
                for(uint i = 0; i < ButtonCount; ++i)
                    if(bitRead(FW_Common::buttons.released, i)) {
                        buf[0] = OF_Const::sBtnReleased, buf[1] = i;
                        Serial.write(buf, 2);
                    }
            }

            #ifdef USES_TEMP
                if(OF_Prefs::pins[OF_Const::tempPin] > -1) {
                    OF_FFB::TemperatureUpdate();

                    if(currentMillis - tempChecked >= 1000) {
                        buf[0] = OF_Const::sTemperatureUpd, buf[1] = OF_FFB::temperatureCurrent;
                        Serial.write(buf, 2);

                        tempChecked = currentMillis;
                    }
                }
            #endif // USES_TEMP
            
            #ifdef USES_ANALOG
                if(FW_Common::analogIsValid && currentMillis - aStickChecked >= 100) {
                    aStickChecked = currentMillis;

                    uint16_t analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
                    uint16_t analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);

                    buf[0] = OF_Const::sAnalogPosUpd;
                    memcpy(&buf[1], (uint8_t*)&analogValueX, sizeof(uint16_t));
                    memcpy(&buf[3], (uint8_t*)&analogValueY, sizeof(uint16_t));
                    Serial.write(buf, 5);
                }
            #endif // USES_ANALOG
        }

        if(Serial.available()) OF_Serial::SerialProcessingDocked();

        if(FW_Common::gunMode != FW_Const::GunMode_Docked)
            return;

        if(FW_Common::runMode == FW_Const::RunMode_Processing)
            ExecRunModeProcessing();
    }
}

// wait up to given amount of time for no buttons to be pressed before setting the mode
void SetModeWaitNoButtons(const FW_Const::GunMode_e &newMode, const unsigned long &maxWait)
{
    unsigned long ms = millis();
    while(FW_Common::buttons.debounced && (millis() - ms < maxWait))
        FW_Common::buttons.Poll(1);

    FW_Common::SetMode(newMode);
}

// Handles events when trigger is pulled/held
void TriggerFire()
{
    if(!FW_Common::buttons.offScreen) {
        if(!OF_FFB::triggerHeld) {
            if(FW_Common::buttons.analogOutput) // this should only ever be gamepad outputs
                Gamepad16.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode3);
            else switch(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportType) {
                case LightgunButtons::ReportType_Mouse:   AbsMouse5.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                case LightgunButtons::ReportType_Keyboard: Keyboard.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                case LightgunButtons::ReportType_Gamepad: Gamepad16.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                default: break;
            }
        }

        if(!OF_Serial::serialMode && 
           !bitRead(FW_Common::buttons.debounced, FW_Const::BtnIdx_Start) &&
           !bitRead(FW_Common::buttons.debounced, FW_Const::BtnIdx_Select))
            OF_FFB::FFBOnScreen();
    // We're shooting outside of the screen boundaries!
    } else {  
        // If we are in offscreen button mode (and aren't dragging a shot offscreen)
        if(!OF_FFB::triggerHeld) {
            if(FW_Common::buttons.analogOutput) // always uses the same button
                Gamepad16.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode3);
            else switch(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportType2) {
                case LightgunButtons::ReportType_Mouse:   AbsMouse5.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                case LightgunButtons::ReportType_Keyboard: Keyboard.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                case LightgunButtons::ReportType_Gamepad: Gamepad16.press(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                default: break;
            }

            FW_Common::triggerPressedOffscreen = true;
        }

        if(!OF_Serial::serialMode) OF_FFB::FFBOffScreen();
    }
    OF_FFB::triggerHeld = true;
}

// Handles events when trigger is released
void TriggerNotFire()
{
    if(OF_FFB::triggerHeld) {
        if(FW_Common::buttons.analogOutput)
            Gamepad16.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode3);
        else if(FW_Common::triggerPressedOffscreen) {
            switch(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportType2) {
                case LightgunButtons::ReportType_Mouse:   AbsMouse5.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                case LightgunButtons::ReportType_Keyboard: Keyboard.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                case LightgunButtons::ReportType_Gamepad: Gamepad16.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode2); break;
                default: break;
            }
            FW_Common::triggerPressedOffscreen = false;
        } else {
            switch(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportType) {
                case LightgunButtons::ReportType_Mouse:   AbsMouse5.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                case LightgunButtons::ReportType_Keyboard: Keyboard.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                case LightgunButtons::ReportType_Gamepad: Gamepad16.release(LightgunButtons::ButtonDesc[FW_Const::BtnIdx_Trigger].reportCode); break;
                default: break;
            }
        }
    }

    OF_FFB::triggerHeld = false;

    if(!OF_Serial::serialMode) OF_FFB::FFBRelease();
}

#ifdef USES_ANALOG
void AnalogStickPoll()
{
    int analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
    int analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);
    
    if(OF_Prefs::settings[OF_Const::analogMode] == OF_Const::analogModeStick) {
        // Analog stick deadzone should help mitigate overwriting USB commands for the other input channels.
        if((analogValueX < ANALOG_STICK_DEADZONE_X_MIN || analogValueX > ANALOG_STICK_DEADZONE_X_MAX) ||
        (analogValueY < ANALOG_STICK_DEADZONE_Y_MIN || analogValueY > ANALOG_STICK_DEADZONE_Y_MAX)) {
            Gamepad16.moveStick(analogValueX, analogValueY);
        } else {
            // Duplicate coords won't be reported, so no worries.
            Gamepad16.moveStick(ANALOG_STICK_CENTER_X, ANALOG_STICK_CENTER_Y);
        }
    } else {
        uint32_t newPos = 0;

        // TODO: need to consider inverted axis toggle, currently assumes axises are inverted by default
        // would this also benefit from custom Analog->Digital deadzone?
        if(analogValueY < (ANALOG_STICK_DEADZONE_Y_MIN-700))
            newPos = 2; // down
        else if(analogValueY > (ANALOG_STICK_DEADZONE_Y_MAX+700))
            newPos = 1; // up

        if(analogValueX < (ANALOG_STICK_DEADZONE_X_MIN-700))
            newPos |= 8; // right
        else if(analogValueX > (ANALOG_STICK_DEADZONE_X_MAX+700))
            newPos |= 4; // left

        switch(OF_Prefs::settings[OF_Const::analogMode]) {
        case OF_Const::analogModeDpad: Gamepad16.padUpdate(FW_Common::buttons.PadMaskConvert(newPos)); break;
        case OF_Const::analogModeKeys:
            if(FW_Common::aStickADCLastPos ^ newPos) {
                for(int i = 0; i < 4; ++i) {
                    if(FW_Common::aStickADCLastPos ^ newPos & 1 << i)
                        Keyboard.release(KEY_UP_ARROW-i);
                }
            }
            for(int i = 0; i < 4; ++i) {
                if(newPos & 1 << i)
                    Keyboard.press(KEY_UP_ARROW-i);
            }
            break;
        }
        FW_Common::aStickADCLastPos = newPos;
    }
        
}
#endif // USES_ANALOG

void SendEscapeKey()
{
    Keyboard.press(KEY_ESC);  // 696969 non corrisponde a HID_KEY_ESCAPE, ma lasciamo come impostato, boh ?
    Keyboard.report();
    delay(20);  // wait a bit so it registers on the PC.
    Keyboard.release(KEY_ESC); // 696969 non corrisponde a HID_KEY_ESCAPE, ma lasciamo come impostato, boh ?
    Keyboard.report();
    lastUSBpoll = millis();
}

// Simple Pause Menu scrolling function
// Bool determines if it's incrementing or decrementing the list
// LEDs update according to the setting being scrolled onto, if any.
void SetPauseModeSelection(const bool &isIncrement)
{
    if(isIncrement) {
        if(FW_Common::pauseModeSelection == FW_Const::PauseMode_EscapeSignal) {
            FW_Common::pauseModeSelection = FW_Const::PauseMode_Calibrate;
        } else {
            FW_Common::pauseModeSelection++;
            // If we use switches, and they ARE mapped to valid pins,
            // then skip over the manual toggle options.
            #ifdef USES_SWITCHES
                #ifdef USES_RUMBLE
                #ifdef PauseMode_RumbleFFToggle
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleFFToggle &&
                !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                    FW_Common::pauseModeSelection++;
                }
                #endif // PauseMode_RumbleFFToggle
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleToggle &&
                (OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0 || OF_Prefs::pins[OF_Const::rumblePin] == -1)) {
                    FW_Common::pauseModeSelection++;
                }
            #endif // USES_RUMBLE
                #ifdef USES_SOLENOID
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_SolenoidToggle &&
                (OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0 || OF_Prefs::pins[OF_Const::solenoidPin] == -1)) {
                    FW_Common::pauseModeSelection++;
                }
            #endif // USES_SOLENOID
            if(FW_Common::pauseModeSelection == FW_Const::PauseMode_AutofireToggle &&
            (OF_Prefs::pins[OF_Const::autofireSwitch] >= 0 || !OF_Prefs::toggles[OF_Const::solenoid])) {
                FW_Common::pauseModeSelection++;
            }
                //wei134102 add start
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_ModeChange) {
                    // ModeChange is always visible
                }
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_LowButtonToggle) {
                    // LowButtonToggle is always visible
                }
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_LayoutToggle) {
                    // LayoutToggle is always visible
                }                                
                //wei134102 add end
            #else
                #ifdef USES_RUMBLE
                    #ifdef PauseMode_RumbleFFToggle
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleFFToggle &&
                    !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                        FW_Common::pauseModeSelection++;
                    }
                    #endif // PauseMode_RumbleFFToggle                
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleToggle &&
                    !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                        FW_Common::pauseModeSelection++;
                    }
                #endif // USES_RUMBLE
                #ifdef USES_SOLENOID
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_SolenoidToggle &&
                    !(OF_Prefs::pins[OF_Const::solenoidPin] >= 0)) {
                        FW_Common::pauseModeSelection++;
                    }
                #endif // USES_SOLENOID
            #endif // USES_SWITCHES
        }
    } else {
        if(FW_Common::pauseModeSelection == FW_Const::PauseMode_Calibrate) {
            FW_Common::pauseModeSelection = FW_Const::PauseMode_EscapeSignal;
        } else {
            FW_Common::pauseModeSelection--;
            #ifdef USES_SWITCHES
                #ifdef USES_SOLENOID
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_SolenoidToggle &&
                    (OF_Prefs::pins[OF_Const::solenoidSwitch] >= 0 || OF_Prefs::pins[OF_Const::solenoidPin] == -1)) {
                        FW_Common::pauseModeSelection--;
                    }
                #endif // USES_SOLENOID
                #ifdef USES_RUMBLE
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleToggle &&
                    (OF_Prefs::pins[OF_Const::rumbleSwitch] >= 0 || OF_Prefs::pins[OF_Const::rumblePin] == -1)) {
                        FW_Common::pauseModeSelection--;
                    }
                    #ifdef PauseMode_RumbleFFToggle
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleFFToggle &&
                    !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                        FW_Common::pauseModeSelection--;
                    }
                    #endif // PauseMode_RumbleFFToggle                    
                #endif // USES_RUMBLE
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_AutofireToggle &&
                (OF_Prefs::pins[OF_Const::autofireSwitch] >= 0 || !OF_Prefs::toggles[OF_Const::solenoid])) {
                    FW_Common::pauseModeSelection--;
                }                
                //wei134102 add start
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_ModeChange) {
                    // ModeChange is always visible
                }
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_LayoutToggle) {
                    // LayoutToggle is always visible
                }                                
                //wei134102 add end
                if(FW_Common::pauseModeSelection == FW_Const::PauseMode_ModeChange) {
            #else
                #ifdef USES_SOLENOID
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_SolenoidToggle &&
                    !(OF_Prefs::pins[OF_Const::solenoidPin] >= 0)) {
                        FW_Common::pauseModeSelection--;
                    }
                #endif // USES_SOLENOID
                #ifdef USES_RUMBLE
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleToggle &&
                    !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                        FW_Common::pauseModeSelection--;
                    }

                    #ifdef PauseMode_RumbleFFToggle
                    if(FW_Common::pauseModeSelection == FW_Const::PauseMode_RumbleFFToggle &&
                    !(OF_Prefs::pins[OF_Const::rumblePin] >= 0)) {
                        FW_Common::pauseModeSelection--;
                    }
                    #endif // PauseMode_RumbleFFToggle                    
                #endif // USES_RUMBLE
            #endif // USES_SWITCHES
        }
    }

    switch(FW_Common::pauseModeSelection) {
        case FW_Const::PauseMode_Calibrate:
          Serial.println("Selecting: Calibrate current profile");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(255,0,0);
          #endif // LED_ENABLE
          break;
        case FW_Const::PauseMode_ProfileSelect:
          Serial.println("Selecting: Switch profile");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(200,50,0);
          #endif // LED_ENABLE
          break;
        case FW_Const::PauseMode_AutofireToggle:
          Serial.println("Selecting: Toggle autofire On/Off");
          #ifdef LED_ENABLE
              if(OF_Prefs::toggles[OF_Const::autofire]) {
                  OF_RGB::LedUpdate(0,255,0);
              } else {
                  OF_RGB::LedUpdate(255,0,0);
              }
          #endif // LED_ENABLE
          break;          
        //wei134102 add start
        case FW_Const::PauseMode_ModeChange:
          Serial.println("Selecting: Change input mode");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(0,255,255);
          #endif // LED_ENABLE
          break; 
        case FW_Const::PauseMode_LowButtonToggle:
          Serial.println("Selecting: Low Button Toggle");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(255,165,0);
          #endif // LED_ENABLE
          break;
        case FW_Const::PauseMode_LayoutToggle:
          Serial.println("Selecting: Layout Toggle");
          #ifdef LED_ENABLE
              // Show current layout color
              if(OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond) {
                  OF_RGB::LedUpdate(128,0,128);
              } else {
                  OF_RGB::LedUpdate(0,0,255);
              }
          #endif // LED_ENABLE
          break;
        case FW_Const::PauseMode_RumbleFFToggle:
          Serial.println("Selecting: Toggle Rumble FFB On/Off");
          #ifdef LED_ENABLE
              if(OF_Prefs::toggles[OF_Const::rumbleFF]) {
                  OF_RGB::LedUpdate(0,255,0);
              } else {
                  OF_RGB::LedUpdate(255,0,0);
              }
          #endif // LED_ENABLE
          break;               
        //wei134102 add end
        case FW_Const::PauseMode_Save:
          Serial.println("Selecting: Save Settings");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(155,100,0);
          #endif // LED_ENABLE
          break;
        #ifdef USES_RUMBLE
        case FW_Const::PauseMode_RumbleToggle:
          Serial.println("Selecting: Toggle rumble On/Off");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(100,155,0);
          #endif // LED_ENABLE
          break;
        #endif // USES_RUMBLE
        #ifdef USES_SOLENOID
        case FW_Const::PauseMode_SolenoidToggle:
          Serial.println("Selecting: Toggle solenoid On/Off");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(55,200,0);
          #endif // LED_ENABLE
          break;
        #endif // USES_SOLENOID
        /*#ifdef USES_SOLENOID
        case FW_Const::PauseMode_BurstFireToggle:
          Serial.println("Selecting: Toggle burst-firing mode");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(0,255,0);
          #endif // LED_ENABLE
          break;
        #endif // USES_SOLENOID
        */
        case FW_Const::PauseMode_EscapeSignal:
          Serial.println("Selecting: Send Escape key signal");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(150,0,150);
          #endif // LED_ENABLE
          break;
        /*case FW_Const::PauseMode_Exit:
          Serial.println("Selecting: Exit pause mode");
          break;
        */
        default:
          Serial.println("YOU'RE NOT SUPPOSED TO BE SEEING THIS");
          break;
    }
    #ifdef USES_DISPLAY
        FW_Common::OLED.PauseListUpdate(FW_Common::pauseModeSelection);
    #endif // USES_DISPLAY
}

// Simple Pause Mode - scrolls up/down profiles list
// Bool determines if it's incrementing or decrementing the list
// LEDs update according to the profile being scrolled on, if any.
void SetProfileSelection(const bool &isIncrement)
{
    if(isIncrement) {
        if(profileModeSelection >= PROFILE_COUNT - 1)
            profileModeSelection = 0;
        else profileModeSelection++;
    } else {
        if(profileModeSelection <= 0)
            profileModeSelection = PROFILE_COUNT - 1;
        else profileModeSelection--;
    }

    #ifdef LED_ENABLE
        OF_RGB::SetLedPackedColor(OF_Prefs::profiles[profileModeSelection].color);
    #endif // LED_ENABLE

    #ifdef USES_DISPLAY
        // 为每个配置文件名称添加布局类型信息
        char profileNames[4][20]; // 为每个配置文件创建带布局信息的名称
        for(int i = 0; i < PROFILE_COUNT; i++) {
            // 根据irLayout值添加布局类型
            if(OF_Prefs::profiles[i].irLayout) {
                snprintf(profileNames[i], sizeof(profileNames[i]), "%s (Diamond)", OF_Prefs::profiles[i].name);
            } else {
                snprintf(profileNames[i], sizeof(profileNames[i]), "%s (Square)", OF_Prefs::profiles[i].name);
            }
        }
        FW_Common::OLED.PauseProfileUpdate(profileModeSelection, profileNames[0], profileNames[1], profileNames[2], profileNames[3]);
    #endif // USES_DISPLAY

    Serial.print("Selecting profile: ");
    Serial.print(OF_Prefs::profiles[profileModeSelection].name);
    Serial.print(" (");
    Serial.print(OF_Prefs::profiles[profileModeSelection].irLayout ? "Diamond" : "Square");
    Serial.println(")");

    return;
}

void SelectCalProfileFromBtnMask(const uint32_t &mask)
{
    // only check if buttons are set in the mask
    if(!mask)
        return;

    for(uint i = 0; i < PROFILE_COUNT; ++i) {
        if(bitRead(mask, i)) {
            FW_Common::SelectCalProfile(i);
            return;
        }
    }
}

void IncreaseIrSensitivity(const uint32_t &sens)
{
    if(sens < DFRobotIRPositionEx::Sensitivity_Max)
        FW_Common::SetIrSensitivity(sens-1);
}

void DecreaseIrSensitivity(const uint32_t &sens)
{
    if(sens > DFRobotIRPositionEx::Sensitivity_Min)
        FW_Common::SetIrSensitivity(sens-1);
}

/*
// applies loaded screen calibration profile
bool SelectCalPrefs(unsigned int profile)
{
    if(profile >= PROFILE_COUNT) {
        return false;
    }

    // if center values are set, assume profile is populated
    if(OF_Prefs::profiles[profile].xCenter && OF_Prefs::profiles[profile].yCenter) {
        xCenter = OF_Prefs::profiles[profile].xCenter;
        yCenter = OF_Prefs::profiles[profile].yCenter;
        
        // 0 scale will be ignored
        if(OF_Prefs::profiles[profile].xScale) {
            xScale = CalScalePrefToFloat(OF_Prefs::profiles[profile].xScale);
        }
        if(OF_Prefs::profiles[profile].yScale) {
            yScale = CalScalePrefToFloat(OF_Prefs::profiles[profile].yScale);
        }
        return true;
    }
    return false;
}
*/

/*
// Unused since runtime burst fire toggle was removed from pause mode, and is accessed from the serial command M8x1
void BurstFireToggle()
{
    burstFireActive = !burstFireActive;                           // Toggle burst fire mode.
    if(burstFireActive) {  // Did we flick it on?
        Serial.println("Burst firing enabled!");
        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Orange);
        #endif
        #ifdef USES_SOLENOID
            for(byte i = 0; i < 4; ++i) {
                digitalWrite(solenoidPin, HIGH);                  // Demonstrate it by flicking the solenoid on/off three times!
                delay(OF_Prefs::settings[OF_Const::solenoidFastInterval]);                      // (at a fixed rate to distinguish it from autofire speed toggles)
                digitalWrite(solenoidPin, LOW);
                delay(OF_Prefs::settings[OF_Const::solenoidFastInterval] * 2);
            }
        #endif // USES_SOLENOID
        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);// And reset the LED back to pause mode color
        #endif // LED_ENABLE
        return;
    } else {  // Or we flicked it off.
        Serial.println("Burst firing disabled!");
        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Orange);
        #endif // LED_ENABLE
        #ifdef USES_SOLENOID
            digitalWrite(solenoidPin, HIGH);                      // Just hold it on for a second.
            delay(300);
            digitalWrite(solenoidPin, LOW);                       // Then off.
        #endif // USES_SOLENOID
        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);// And reset the LED back to pause mode color
        #endif // LED_ENABLE
        return;
    }
}
*/

#ifdef USES_RUMBLE
// Pause mode rumble enabling widget
// Does a cute rumble pattern when on, or blinks LEDs (if any)
void RumbleToggle()
{
    OF_Prefs::toggles[OF_Const::rumble] = !OF_Prefs::toggles[OF_Const::rumble];
    if(OF_Prefs::toggles[OF_Const::rumble]) {
        if(!OF_Serial::serialMode) 
            Serial.println("Rumble enabled!");

        #ifdef USES_DISPLAY
            FW_Common::OLED.TopPanelUpdate("Toggling Rumble ON");
        #endif // USES_DISPLAY

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Salmon);
        #endif // LED_ENABLE

        
        #ifdef ARDUINO_ARCH_ESP32  // 696969 per ESP32
            analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 255);       // Pulse the motor on to notify the user,
            delay(300);                                               // Hold that,
            analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);        // Then turn off,
        #else // rp2040
        digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], HIGH);       // Pulse the motor on to notify the user,
        delay(300);                                               // Hold that,
        digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);        // Then turn off,
        #endif

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);// And reset the LED back to pause mode color
        #endif // LED_ENABLE
    } else {                                                      // Or if we're turning it OFF,
        if(!OF_Serial::serialMode) 
            Serial.println("Rumble disabled!");

        #ifdef USES_DISPLAY
            FW_Common::OLED.TopPanelUpdate("Toggling Rumble OFF");
        #endif // USES_DISPLAY

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Salmon);                 // Set a color,
            delay(150);                                           // Keep it on,
            OF_RGB::LedOff();                                             // Flicker it off
            delay(100);                                           // for a bit,
            OF_RGB::SetLedPackedColor(WikiColor::Salmon);                 // Flicker it back on
            delay(150);                                           // for a bit,
            OF_RGB::LedOff();                                             // And turn it back off
            delay(200);                                           // for a bit,
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);// And reset the LED back to pause mode color
        #endif // LED_ENABLE
    }

    #ifdef USES_DISPLAY
        FW_Common::OLED.TopPanelUpdate("Using ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);
    #endif // USES_DISPLAY
}
#endif // USES_RUMBLE

#ifdef USES_SOLENOID
// Pause mode solenoid enabling widget
// Does a cute solenoid engagement, or blinks LEDs (if any)
void SolenoidToggle()
{
    OF_Prefs::toggles[OF_Const::solenoid] = !OF_Prefs::toggles[OF_Const::solenoid];                             // Toggle
    if(OF_Prefs::toggles[OF_Const::solenoid]) {                                          // If we turned ON this mode,
        if(!OF_Serial::serialMode)
            Serial.println("Solenoid enabled!");

        #ifdef USES_DISPLAY
            FW_Common::OLED.TopPanelUpdate("Toggling Solenoid ON");
        #endif // USES_DISPLAY

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Yellow);                 // Set a color,
        #endif // LED_ENABLE

        digitalWrite(OF_Prefs::pins[OF_Const::solenoidPin], HIGH);                          // Engage the solenoid on to notify the user,
        delay(300);                                               // Hold it that way for a bit,
        digitalWrite(OF_Prefs::pins[OF_Const::solenoidPin], LOW);                           // Release it,

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);    // And reset the LED back to pause mode color
        #endif // LED_ENABLE

    } else {                                                      // Or if we're turning it OFF,
        if(!OF_Serial::serialMode)
            Serial.println("Solenoid disabled!");

        #ifdef USES_DISPLAY
            FW_Common::OLED.TopPanelUpdate("Toggling Solenoid OFF");
        #endif // USES_DISPLAY

        #ifdef LED_ENABLE
            OF_RGB::SetLedPackedColor(WikiColor::Yellow);                 // Set a color,
            delay(150);                                           // Keep it on,
            OF_RGB::LedOff();                                             // Flicker it off
            delay(100);                                           // for a bit,
            OF_RGB::SetLedPackedColor(WikiColor::Yellow);                 // Flicker it back on
            delay(150);                                           // for a bit,
            OF_RGB::LedOff();                                             // And turn it back off
            delay(200);                                           // for a bit,
            OF_RGB::SetLedPackedColor(OF_Prefs::profiles[OF_Prefs::currentProfile].color);// And reset the LED back to pause mode color
        #endif // LED_ENABLE
    }

    #ifdef USES_DISPLAY
        FW_Common::OLED.TopPanelUpdate("Using ", OF_Prefs::profiles[OF_Prefs::currentProfile].name);
    #endif // USES_DISPLAY
}
#endif // USES_SOLENOID
