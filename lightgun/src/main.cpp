/*!
 * @file main.cpp
 * @brief OpenFIRE Esp32 - 4IR LED Lightgun wireless sketch w/ support for force feedback and other features.
 * Forked from OpenFIRE firmware v6.2 from https://github.com/TeamOpenFIRE/OpenFIRE-Firmware
 * which in itself is forked from IR-GUN4ALL v4.2, which is based on Prow's Enhanced Fork from https://github.com/Prow7/ir-light-gun,
 * which in itself is based on the 4IR Beta "Big Code Update" SAMCO project from https://github.com/samuelballantyne/IR-Light-Gun
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V6.0 PreRelease7
 * @date 2026
 * 
 * I thank you for producing the first original code:
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
#include "OpenFIREPlayTimer.h"
#include "OpenFIREBoyMode.h"
#include "boards/OpenFIREshared.h"
#include "OpenFIREcolors.h"
#include "OpenFIRElights.h"
#include "OpenFIREserial.h"
#include "OpenFIREFeedback.h"
#include "OpenFIREprefs.h"
#include "OpenFIREconstant.h"
#ifdef USES_DISPLAY
#include "OpenFIREdisplay_i18n.h"
#endif

// ===================================================================================
// OS ABSTRACTION: RIDEFINIZIONE DEL DELAY (FREERTOS)
// ===================================================================================
// Sostituendo la funzione nativa bloccante con vTaskDelay, garantiamo che 
// lo scheduler di FreeRTOS possa cedere il controllo ad altri task (es. radio o display) 
// durante le attese, prevenendo lo stallo dell'intero sistema operativo.
#ifdef ARDUINO_ARCH_ESP32  // 696969
    #define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))                    
#endif //ARDUINO_ARCH_ESP32

// ===================================================================================
// CONDIVISIONE RISORSE DISPLAY
// ===================================================================================
// Esponendo un puntatore reference (*&), permettiamo al modulo Wireless, isolato 
// in altri thread, di iniettare animazioni di stato (es. "Searching...") 
// direttamente sul display, aggirando l'incapsulamento standard del firmware.
// ========== serve per usare display da parte wireless =============
#ifdef USES_DISPLAY
    #ifdef USE_LOVYAN_GFX
        //LGFX_SSD1306 *display_OLED  = nullptr;   
        LGFX_SSD1306 *&display_OLED  = FW_Common::OLED.display;   //aggiunto inline per condividerla
    #else
        //Adafruit_SSD1306 *display_OLED  = nullptr; 
        Adafruit_SSD1306 *&display_OLED  = FW_Common::OLED.display; //aggiunto inline per condividerla
    #endif
#endif // USES_DISPLAY
// ========== fine serve per usare display da parte wireless =============


// ===================================================================================
// MULTITHREADING (CORE 1) E WATCHDOG
// ===================================================================================
// Il setup del dual-core scarica cicli preziosi. Tuttavia, il vTaskDelay(1) 
// è essenziale architettonicamente: costringe il task a dormire 1 ms permettendo 
// al task Idle nascosto di FreeRTOS di girare. Senza questo respiro, il Core si 
// bloccherebbe e l'Hardware Watchdog riavvierebbe la scheda in panic.

// =======696969===== GESTIONE DUAL CORE PER ESP32 CHE USA FREERTOS ===  INIZIALIZZAZIONE ========
#if defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
    void setup1();
    void loop1();
    TaskHandle_t task_loop1;
    void esploop1(void* pvParameters) {
        setup1();
        for (;;) {
            loop1();
            // Blocca questo task per 1 millisecondo (o 1 tick).
            // Questo permette all'Idle Task (priorità 0) di girare, 
            // nutrire il Watchdog e pulire il sistema.
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
#endif
// ======696969============= FINE GESTIONE DUAL CORE ESP32 ==== FINE INIZIALIZZAZIONE ============


// Sets up the environment
void setup() {

// ======== 696969 =========== X AVVIO DUAL CORE ESP32 =================================== 
#if defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
    #define STACK_SIZE_SECOND_CORE 4096 // 10000  // basta 4096 ???
    #define PRIORITY_SECOND_CORE 1   //   0  // tra 1 e 24 ?
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

// ===================================================================================
// HARDWARE FAILSAFES (I2C & ADC)
// ===================================================================================
// La stabilità fisica dell'hardware vario (cloni Arduino, cablaggi lunghi) è imprevedibile.
// I timeout I2C prevengono blocchi infiniti del bus. I pin GPIO tenuti alti 
// compensano quirk noti di board specifiche (es. stabilizzare l'LDO del Pico).

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

    // ===================================================================================
    // FORZATURA HARDWARE ADC (Previene futuri bug se cambiano i default di libreria)
    // ===================================================================================
    #if defined(ARDUINO_ARCH_ESP32) && defined(USES_ANALOG)
        
        // Fissa l'ADC a 12-bit (scala 0-4095). 
        // Vitale: tutte le costanti (es. centro a 2048) e le formule dipendono da questo.
        analogReadResolution(12); 
    
        // Fissa l'attenuazione a 11dB (misura fino a ~3.1V).
        // Vitale: impedisce che l'ADC saturi a tensioni basse, garantendo
        // l'escursione totale del joystick e la corretta lettura del TMP36.
        analogSetAttenuation(ADC_11db);
        
    #endif // defined(ARDUINO_ARCH_ESP32) && defined(USES_ANALOG)


// ===================================================================================
// DATA INTEGRITY & PROFILAZIONE
// ===================================================================================
// Una EEPROM (o File System) corrotta può caricare valori di offset impossibili, 
// distruggendo la matematica del tracking IR. Questo blocco agisce come "Sanitizer", 
// forzando il fallback a valori sicuri prima che il software possa crashare.

    // Init pins array, and OFPresets for later load ops
    OF_Prefs::LoadPresets();
    
    if(OF_Prefs::InitFS() == OF_Prefs::Error_Success) {
        // OF_Prefs::ResetPreferences(); // ============ FORMATTA IL FILE SYSTEM =================================
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

        // 根据设置初始化轴输出模式（有符号/无符号）
        // NOTE: 当前仅保留 unsignedAxis 兼容字段，实际 unsigned 映射逻辑待后续实现（见 docs/UNSIGNED_AXIS_TODO.md）
        if (OF_Const::settingsTypesCount > OF_Const::axisUnsigned) {
            Gamepad16.unsignedAxis = (OF_Prefs::settings[OF_Const::axisUnsigned] != 0);
        }

        // 根据设置初始化 GAMEPAD 模式左右摇杆互换（红外定位 <-> 物理摇杆）
        if (OF_Const::settingsTypesCount > OF_Const::analogSwapSticks) {
            Gamepad16.stickRight = (OF_Prefs::settings[OF_Const::analogSwapSticks] != 0);
        } else {
            Gamepad16.stickRight = false;
        }
        
        // 如果USB ID未设置但gunId已设置，根据gunId更新USB ID
        if(OF_Prefs::usb.devicePID == 0 && OF_Prefs::settings[OF_Const::gunId] < 4) {
            OF_Prefs::usb.devicePID = OF_Prefs::settings[OF_Const::gunId] + 1;
            OF_Prefs::usb.deviceName[9] = '1' + OF_Prefs::settings[OF_Const::gunId];
            OF_Prefs::SaveUSBID();
        }
        
        #if defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)
           ////////////////// MAI USATO //////////////////////////////
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
            //////////////////////////////// FINE MAI USATO //////////////////////////////////////
            
            if (OF_Prefs::LoadLastDongleWireless(lastDongleAddress, &lastDongleChannel) == OF_Prefs::Error_Success) lastDongleSave = true;
                else lastDongleSave = false;
            
            if (OF_Prefs::LoadLastPedalWireless(lastPedalAddress, &lastPedalChannel) == OF_Prefs::Error_Success) lastPedalSave = true;
                else lastPedalSave = false;

        #endif // defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)
        
    } else Serial.printf("%c%c (No Storage Available)", OF_Const::sError, (char)OF_Prefs::Error_NoStorage);
    
    

// ===================================================================================
// IDENTITA' DINAMICA USB (SPOOFING SHADOWING)
// ===================================================================================
// Prepariamo l'identità della periferica. Se siamo in modalità Wireless, questi 
// dati verranno inviati al Dongle affinché possa fingersi questa specifica pistola. 
// Se siamo via Cavo, configureremo il nostro TinyUSB locale con questi stessi parametri.

    // ===== 696969 per trasmettere i dati wireless al dongle ========
    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
        strncpy(usb_data_wireless.deviceManufacturer,MANUFACTURER_NAME,sizeof(usb_data_wireless.deviceManufacturer));
        strncpy(usb_data_wireless.deviceName,DEVICE_NAME, sizeof(usb_data_wireless.deviceName)); // cambia
        usb_data_wireless.deviceVID = DEVICE_VID;
        #ifdef PLAYER_NUMBER
            usb_data_wireless.devicePID = PLAYER_NUMBER; // cambia
            usb_data_wireless.devicePlayer = PLAYER_NUMBER;
        #else
            usb_data_wireless.devicePID = 1; //PLAYER 1 DEFAULT
            usb_data_wireless.devicePlayer = 1; // PLAYER 1 DEFAULT
        #endif // PLAYER_NUMBER
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


// ===================================================================================
// CALIBRAZIONE EMPIRICA HARDWARE DEGLI STICK ANALOGICI
// ===================================================================================
// Invece di affidarci a valori di deadzone hardcodati (che variano per usura 
// dei potenziometri, molla meccanica o tolleranze di fabbrica), blocchiamo il sistema 
// per 2 secondi all'avvio. Registriamo fisicamente il "rumore" di fondo dello stick 
// a riposo per stabilire un centro e limiti di sicurezza personalizzati dinamicamente.

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 696969 == CODICE PER CALIBRARE LEVETTA STICK IN POSIZIONE CENTRALE =============
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #if defined(ARDUINO_ARCH_ESP32) && defined(USES_ANALOG)   // la facciamo solo per ESP32 e lasciamo RP2040 come gestione originale
        
    #ifdef USES_DISPLAY
        unsigned long lastChange = 0;
        int8_t currentIndex = 0;
        int8_t direzione = 1;
        const char* word = "...CALIBRATION...";
        const uint8_t baseX = 10;
        const uint8_t baseY = 2;
        const uint8_t charWidth = 6;
        uint8_t len_word;
    if(display_OLED != nullptr) {    
        display_OLED->setCursor(baseX, baseY);
        display_OLED->setTextSize(1);
        display_OLED->setTextColor(WHITE, BLACK);
        display_OLED->fillRect(0, 0, SCREEN_WIDTH, 16, BLACK);  // 使用SCREEN_WIDTH宏以适应不同尺寸
        display_OLED->drawFastHLine(0, 15, SCREEN_WIDTH, WHITE);  // 使用SCREEN_WIDTH宏
        display_OLED->print(word);
        display_OLED->display();
        len_word=strlen(word);
    }
    #endif // USES_DISPLAY
    
    uint16_t analogValueX;
    uint16_t analogValueY;
       
    //unsigned long startTime = 0;
    unsigned long startTime = millis();
    while ((millis()-startTime) < 2000) // 2000
    {
        analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
        analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);
        if (analogValueX > ANALOG_STICK_DEADZONE_X_MAX) ANALOG_STICK_DEADZONE_X_MAX = analogValueX;
        if (analogValueX < ANALOG_STICK_DEADZONE_X_MIN) ANALOG_STICK_DEADZONE_X_MIN = analogValueX;
        if (analogValueY > ANALOG_STICK_DEADZONE_Y_MAX) ANALOG_STICK_DEADZONE_Y_MAX = analogValueY;
        if (analogValueY < ANALOG_STICK_DEADZONE_Y_MIN) ANALOG_STICK_DEADZONE_Y_MIN = analogValueY;

        #ifdef USES_DISPLAY
        if(display_OLED != nullptr) {
            if (millis() - lastChange > 50) {
                display_OLED->setTextColor(BLACK, BLACK);
                display_OLED->setCursor(baseX + (currentIndex * charWidth), baseY);
                display_OLED->write(word[currentIndex]);
                if ((currentIndex > 0) && (currentIndex < (len_word-1)))  {
                    display_OLED->setCursor(baseX + ((currentIndex-direzione) * charWidth), baseY);
                    display_OLED->setTextColor(WHITE, BLACK); 
                    display_OLED->write(word[currentIndex-direzione]);
                }
                display_OLED->display();
                currentIndex = currentIndex + direzione;
                if ((currentIndex == (len_word-1)) || (currentIndex == 0)) direzione *= -1;
                lastChange = millis();
            }
        }
        #endif // USES_DISPLAY
    }
    #ifdef USES_DISPLAY
        FW_Common::OLED.TopPanelUpdate(TXT_TOP_CALIB_READY);
    #endif //USES_DISPLAY

    // ===================================================================================
    // CALCOLO DEL CENTRO MECCANICO PURO
    // ===================================================================================
    // Il centro va calcolato ORA, basandosi solo sul rumore grezzo della molla.
    // ATTENZIONE: Non spostare mai questo calcolo sotto il blocco del 'buffer'. 
    // Se venisse calcolato dopo il constrain(), un taglio asimmetrico dei limiti 
    // sbilancerebbe il centro del joystick in game.
    ANALOG_STICK_DEADZONE_X_CENTER = (ANALOG_STICK_DEADZONE_X_MIN + ANALOG_STICK_DEADZONE_X_MAX) / 2; 
    ANALOG_STICK_DEADZONE_Y_CENTER = (ANALOG_STICK_DEADZONE_Y_MIN + ANALOG_STICK_DEADZONE_Y_MAX) / 2; 
    
    // Espansione di tolleranza (buffer zona morta) per assorbire piccoli rimbalzi meccanici   
    // Espansione di tolleranza con PROTEZIONE (Clamping)
    // Sottraiamo/Aggiungiamo 400 ma forziamo il risultato a rimanere nel recinto 0-4095
    int buffer = 400; 
    ANALOG_STICK_DEADZONE_X_MIN = constrain((int)ANALOG_STICK_DEADZONE_X_MIN - buffer, 0, 4095);
    ANALOG_STICK_DEADZONE_X_MAX = constrain((int)ANALOG_STICK_DEADZONE_X_MAX + buffer, 0, 4095);
    ANALOG_STICK_DEADZONE_Y_MIN = constrain((int)ANALOG_STICK_DEADZONE_Y_MIN - buffer, 0, 4095);
    ANALOG_STICK_DEADZONE_Y_MAX = constrain((int)ANALOG_STICK_DEADZONE_Y_MAX + buffer, 0, 4095);


    #endif
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 696969 == FINE CODICE CALIBRAZIONE STICK ========================================
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ===================================================================================================================

// ===================================================================================
// CONNECTION ROUTING: WIRED (USB) VS WIRELESS (ESP-NOW)
// ===================================================================================
// Al boot, il sistema attende ~1000ms per capire se è connesso via Cavo a un PC.
// Se l'Host USB monta il dispositivo, la Lightgun sceglierà la porta Seriale fisica. 
// In caso contrario, interpreterà l'assenza di host come "Modalità Batteria", 
// disattivando il driver USB locale e accendendo la radio ESP-NOW per agganciarsi al Dongle.

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
    FW_Common::OLED.TopPanelUpdate(TXT_TOP_USB_MODE);
    TinyUSBDevices.begin(POLL_RATE);
    FW_Common::OLED.TopPanelUpdate(TXT_TOP_USB_WAIT);

    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)// SE WIRELESS
        #define MILLIS_TIMEOUT  3000 //1 secondi  检测USB是否连接的时间！！！1秒
        #define BOY_MODE_WINDOW_MS 3000 // Boy 模式按键判定窗口（毫秒）
        unsigned long lastMillis = millis();
        // 先给 USB 一段时间看看是否被挂载
        while ((millis() - lastMillis <= MILLIS_TIMEOUT) && (!TinyUSBDevice.mounted())) { yield(); }

        // 如果 USB 还没挂载，先给 Boy Mode 一个 3 秒窗口，再决定要不要去扫无线
        if (!TinyUSBDevice.mounted())
        {
            BoyMode::BeginDecisionWindow(BOY_MODE_WINDOW_MS);
            while (!TinyUSBDevice.mounted() &&
                   !BoyMode::IsEnabled() &&
                   (millis() - lastMillis <= (MILLIS_TIMEOUT + BOY_MODE_WINDOW_MS))) {
                yield();  // 先让 USB 有机会处理
                BoyMode::TickDecisionWindow();
            }
            // 如果启用了 Boy Mode，这里不要做额外外设操作，避免在启动阶段阻塞
            if (BoyMode::IsEnabled())
            {
                // 进入后由后续 Boy 模式分支接管
            }
        }

        // 只有在没进 Boy Mode 的情况下，才真正去做无线连接/扫描
        if (!TinyUSBDevice.mounted() && !BoyMode::IsEnabled()) {
            SerialWireless.init_wireless();
            SerialWireless.begin(); // fare una sorta di prebegin, senza impostare peer e altro
            if (lastDongleSave) {
                // PROVA A CONNETTERTI AL PRECEDENTE DONGLE INVIANDO IL PACCHETTO CHECK_CONNECTION
                if (SerialWireless.connection_gun_at_last_dongle()) {
                } else {
                    SerialWireless.connection_gun();
                }
            }
            else {
                SerialWireless.connection_gun();
            }
        }
    #else
        while(!TinyUSBDevice.mounted() && !TinyUSBDevices.onBattery) { yield(); }
    #endif


    // arriva qui solo se e' stato connesso l'usb, e' stata negoziata una connessione wireless, oppure Boy Mode è attivo
    #ifdef ARDUINO_ARCH_ESP32
        #ifdef USES_DISPLAY
            if (TinyUSBDevice.mounted() || TinyUSBDevices.onBattery) {
                FW_Common::OLED.TopPanelUpdate(TXT_TOP_LINK_READY);
                vTaskDelay(pdMS_TO_TICKS(1000));
            } else if (BoyMode::IsEnabled()) {
                FW_Common::OLED.TopPanelUpdate(TXT_TOP_BOY_READY);
                vTaskDelay(pdMS_TO_TICKS(600));
            }
        #endif //USES_DISPLAY
    #endif // ARDUINO_ARCH_ESP32

    if (TinyUSBDevice.mounted() || BoyMode::IsEnabled()) {
        // Boy 模式下按“有线已就绪”路径处理，避免切到无线串流分支
        TinyUSBDevices.onBattery = false;
        Serial.begin(9600);
        Serial.setTimeout(0);
        FW_Common::OLED.TopPanelUpdate(TXT_TOP_USB_MOUNT);
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
            }
            if (TinyUSBDevices.wireless_mode != WIRELESS_MODE::NONE_WIRELESS) SerialWireless.end();
        #endif 
        Serial_OpenFIRE_Stream = & Serial;

    }  
    #if defined(ARDUINO_ARCH_ESP32) && defined(OPENFIRE_WIRELESS_ENABLE)
    else {     
        // Only update "last dongle" if we actually have a negotiated wireless link.
        if(!BoyMode::IsEnabled()) {
            if (!lastDongleSave || 
                (lastDongleSave && (!(memcmp(lastDongleAddress, peerAddress,6) == 0) || !(lastDongleChannel == espnow_wifi_channel))))
                OF_Prefs::SaveLastDongleWireless(peerAddress, &espnow_wifi_channel);
        }
        // CHIUDI TUTTO CIO' CHE E' USB SE E' DA CHIUDERE
        // Salvaguarda la memoria e previene errori del core USB quando non è connesso fisicamente.
        TinyUSBDevice.clearConfiguration();
        TinyUSBDevice.detach();
        
        Serial_OpenFIRE_Stream = &SerialWireless;
        // ======================== PEDAL   //696969
        if((OF_Prefs::pins[OF_Const::btnPedal] == -1) && (OF_Prefs::pins[OF_Const::btnPedal2] == -1)) {
            if (lastPedalSave && (lastPedalChannel == espnow_wifi_channel)) {
                if (!SerialWireless.connection_gun_at_last_pedal()) SerialWireless.connection_gun_at_pedal();
            } else SerialWireless.connection_gun_at_pedal();
        }
        else {
            // invia segnale al dongle che la procedura del PEDAL è completa
              SerialWireless.tx_gun_at_dongle_pedal_ready();
        }
        // ======================== FINE PEDAL

        
        if (TinyUSBDevices.is_pedal_wireless && (!lastPedalSave || 
            (lastPedalSave && (!(memcmp(lastPedalAddress, peerAddress_pedal,6) == 0) || !(lastPedalChannel == espnow_wifi_channel))))) OF_Prefs::SaveLastPedalWireless(peerAddress_pedal, &espnow_wifi_channel);
  
        // ===== PEDAL FIEN POI SPOSTARLO ================

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
    #endif // USES_DISPLAY
    #endif //ARDUINO_ARCH_ESP32

// ===================================================================================
// POLYMORPHIC I/O: IL TRUCCO DELLA SERIALE VIRTUALE
// ===================================================================================
// Architettura geniale per retrocompatibilità. Ridefinendo la macro `Serial` per 
// puntare al nostro stream dereferenziato (*Serial_OpenFIRE_Stream), l'intera codebase 
// legacy (nata per USB wired) può usare `Serial.print` e `Serial.write` come sempre. 
// I dati verranno instradati magicamente via Cavo o via Rete (ESP-NOW) a seconda 
// dell'esito della negoziazione precedente.

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
    // L'esecuzione wireless introduce overhead. Ridurre il tick rate della telecamera IR 
    // su batteria (da 5ms a ~15ms) offre più CPU all'ESP-NOW limitando lag di sistema.
    // funziona comunque bene alla stessa frequenza usata con connessione via cavo, ma volendo si può abbassare
    if (TinyUSBDevices.onBattery) startIrCamTimer(209);  // 120 ---- 100->10ms 66 -> 15ms per connessione wireless
      else startIrCamTimer(209); // 5ms per connessione via seriale
    
    FW_Common::OpenFIREper.source(OF_Prefs::profiles[OF_Prefs::currentProfile].adjX,
                                  OF_Prefs::profiles[OF_Prefs::currentProfile].adjY);
    FW_Common::OpenFIREper.deinit(0);

    // Boy 模式只需要基本硬件初始化，直接进入 Run，不需要首启校准/无线等后续流程
    if (BoyMode::IsEnabled())
    {
        FW_Common::SetMode(FW_Const::GunMode_Run);
        return;
    }

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

        // 更新游戏计时器（时间到后仅锁定按键上报，不断开 USB / 无线）
        PlayTimer::Tick();

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

        if(!BoyMode::IsEnabled() && millis() - lastUSBpoll >= POLL_RATE) {
            #ifdef USES_ANALOG
                if(FW_Common::analogIsValid) AnalogStickPoll();
            #endif // USES_ANALOG
            lastUSBpoll = millis();
            FW_Common::buttons.SendReports();
        }

        if(!BoyMode::IsEnabled() && Serial.available()) OF_Serial::SerialProcessing();

        #ifdef MAMEHOOKER
            if(!BoyMode::IsEnabled() && OF_Serial::serialMode) OF_Serial::SerialHandling();                                   // Process the force feedback from the current queue.
        #endif // MAMEHOOKER
        
        if(FW_Common::buttons.pressedReleased == FW_Const::EscapeKeyBtnMask)
            SendEscapeKey();
        
        // A+B+Trigger 快捷键检测：在运行模式下切换鼠标和手柄模式
        if(FW_Common::buttons.pressedReleased == FW_Const::ToggleMouseGamepadBtnMask) {
            // Toggle between mouse and gamepad mode
            if(!OF_Serial::serialMode) {
                Serial.println("Toggling input mode...");
            }
            
            // Only toggle between mouse and gamepad mode, ignore MISTER mode
            if(!FW_Common::buttons.analogOutput) {
                // Current: Mouse/KB -> Switch to Gamepad
                FW_Common::buttons.analogOutput = true;
                FW_Common::OLED.mister = false; // Ensure MISTER mode is disabled
                FW_Common::UpdateBindings(false);
                if(!OF_Serial::serialMode)
                    Serial.println("Mode changed to: Gamepad");
                
                // RGB灯反馈：蓝色表示手柄模式
                #ifdef LED_ENABLE
                    OF_RGB::LedUpdate(0, 0, 255); // 蓝色
                #endif // LED_ENABLE
            } else {
                // Current: Gamepad -> Switch to Mouse/KB
                FW_Common::buttons.analogOutput = false;
                FW_Common::OLED.mister = false; // Ensure MISTER mode is disabled
                FW_Common::UpdateBindings(false);
                if(!OF_Serial::serialMode)
                    Serial.println("Mode changed to: Mouse/Keyboard");
                
                // RGB灯反馈：绿色表示鼠标模式
                #ifdef LED_ENABLE
                    OF_RGB::LedUpdate(0, 255, 0); // 绿色
                #endif // LED_ENABLE
            }
            
            // 震动反馈
            #ifdef USES_RUMBLE
                if(OF_Prefs::toggles[OF_Const::rumble]) {
                    // 短震动反馈
                    analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
                    delay(200);
                    #ifdef ARDUINO_ARCH_ESP32
                        analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);
                    #else // rp2040
                        digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
                    #endif
                }
            #endif // USES_RUMBLE
            
            // Save the new mode settings
            OF_Prefs::toggles[OF_Const::analogOutputMode] = FW_Common::buttons.analogOutput;
            OF_Prefs::toggles[OF_Const::misterMode] = FW_Common::OLED.mister;
            OF_Prefs::SaveToggles();
            
            // Update display to show new mode
            #ifdef USES_DISPLAY
                if(FW_Common::buttons.analogOutput) {
                    FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_GAMEPAD);
                } else {
                    FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_MOUSEKB);
                }
                // Briefly show status then update menu
                delay(1000); // Show status for 1 second
                FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
            #endif // USES_DISPLAY
            
            // 恢复RGB灯到正常模式颜色
            #ifdef LED_ENABLE
                delay(500); // 保持反馈颜色0.5秒
                FW_Common::SetLedColorFromMode(); // 恢复根据当前模式的颜色
            #endif // LED_ENABLE
        }

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
        #ifndef COMMENTO
        #if defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
        // Blocca questo task per 1 millisecondo (o 1 tick).
        // Questo permette all'Idle Task (priorità 0) di girare, 
        // nutrire il Watchdog e pulire il sistema.
        vTaskDelay(pdMS_TO_TICKS(1));
        //yield();
        #endif // defined(ARDUINO_ARCH_ESP32) && defined(DUAL_CORE)
        #endif // COMMENTO
    }
}
#endif // DUAL_CORE
// #endif // ARDUINO_ARCH_RP2040 // 696969 tolto messo sopra

// Main core events hub
// splits off into subsequent ExecModes depending on circumstances
void loop()
{
    // Boy 模式只影响启动连接决策；运行期沿用原有触发/反馈逻辑

    // poll/update button states with 1ms interval so debounce mask is more effective
    FW_Common::buttons.Poll(1);
    FW_Common::buttons.Repeat();

    // 更新计时器（ESP32 不会跑 loop1，因此这里必须 Tick）
    PlayTimer::Tick();

    // 倒计时显示由 ExtDisplay::IdleOps() 统一驱动，避免多线程/覆盖问题

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
        pauseModeSelectingGunId = false;
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
                        FW_Common::pauseModeSelection = FW_Const::PauseMode_AnalogCenterCal;

                        if(!OF_Serial::serialMode) {
                            Serial.print("Switched to profile: ");
                            Serial.println(OF_Prefs::profiles[OF_Prefs::currentProfile].name);
                            Serial.println("Going back to the main menu...");
                            Serial.println("Selecting: Calibrate current profile");
                        }

                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
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

                        FW_Common::pauseModeSelection = FW_Const::PauseMode_AnalogCenterCal;

                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                        #endif // USES_DISPLAY
                    }
                } else if(pauseModeSelectingGunId) {
                    if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_A) {
                        gunIdModeSelection = (gunIdModeSelection == 0) ? 3 : (gunIdModeSelection - 1);
                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseProfileUpdate(gunIdModeSelection, "P1", "P2", "P3", "P4");
                        #endif
                    } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_B) {
                        gunIdModeSelection = (gunIdModeSelection >= 3) ? 0 : (gunIdModeSelection + 1);
                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseProfileUpdate(gunIdModeSelection, "P1", "P2", "P3", "P4");
                        #endif
                    } else if(FW_Common::buttons.pressedReleased == FW_Const::BtnMask_Trigger) {
                        OF_Prefs::settings[OF_Const::gunId] = gunIdModeSelection;
                        // 更新USB设备PID和名称
                        OF_Prefs::usb.devicePID = gunIdModeSelection + 1; // P1=1, P2=2, P3=3, P4=4
                        OF_Prefs::usb.deviceName[9] = '1' + gunIdModeSelection; // 更新名称中的P后面的数字
                        OF_Prefs::SaveSettings();
                        OF_Prefs::SaveUSBID();
                        pauseModeSelectingGunId = false;
                        FW_Common::pauseModeSelection = FW_Const::PauseMode_AnalogCenterCal;
                        if(!OF_Serial::serialMode) {
                            Serial.print("Gun ID set to P");
                            Serial.print((int)gunIdModeSelection + 1);
                            Serial.print(", USB PID: ");
                            Serial.println(OF_Prefs::usb.devicePID);
                            Serial.println("Please reconnect USB to apply changes.");
                        }
                        #ifdef USES_DISPLAY
                            {
                                char gunIdBuf[16];
                                snprintf(gunIdBuf, sizeof(gunIdBuf), TXT_GUN_ID_TOP_FMT, (int)gunIdModeSelection + 1);
                                FW_Common::OLED.TopPanelUpdate(gunIdBuf);
                            }
                            delay(800);
                            FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                        #endif
                    } else if(FW_Common::buttons.pressedReleased & FW_Const::ExitPauseModeBtnMask) {
                        pauseModeSelectingGunId = false;
                        FW_Common::pauseModeSelection = FW_Const::PauseMode_AnalogCenterCal;
                        #ifdef USES_DISPLAY
                            FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                        #endif
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
                                      snprintf(profileNames[i], sizeof(profileNames[i]), TXT_PROFILE_DIAM_FMT, OF_Prefs::profiles[i].name);
                                  } else {
                                      snprintf(profileNames[i], sizeof(profileNames[i]), TXT_PROFILE_SQUA_FMT, OF_Prefs::profiles[i].name);
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
                        case FW_Const::PauseMode_AnalogRangeCal:
                        {
                            int pinX = OF_Prefs::pins[OF_Const::analogX];
                            int pinY = OF_Prefs::pins[OF_Const::analogY];
                            if (pinX < 0 || pinY < 0) {
                                if (!OF_Serial::serialMode)
                                    Serial.println("RangeCal: analog pins not set (analogX/analogY)");
                                break;
                            }

                            if (!OF_Serial::serialMode)
                                Serial.println("RangeCal: rotate stick full circle for ~4s...");

                            #ifdef USES_DISPLAY
                                FW_Common::OLED.TopPanelUpdate(TXT_TOP_RANGECAL_ROTATE);
                            #endif

                            uint16_t minX = 4095, maxX = 0, minY = 4095, maxY = 0;
                            const uint32_t sampleMs = 4200;
                            uint32_t t0 = millis();
                            while (millis() - t0 < sampleMs) {
                                int x = analogRead(pinX);
                                int y = analogRead(pinY);
                                if (x < (int)minX) minX = (uint16_t)x;
                                if (x > (int)maxX) maxX = (uint16_t)x;
                                if (y < (int)minY) minY = (uint16_t)y;
                                if (y > (int)maxY) maxY = (uint16_t)y;
                                delay(2);
                            }

                            #ifdef USES_DISPLAY
                                FW_Common::OLED.TopPanelUpdate(TXT_TOP_RANGECAL_RELEASE);
                            #endif

                            delay(250); // 让摇杆回中后稍微稳定一下
                            const int centerSamples = 64;
                            uint32_t sumX = 0, sumY = 0;
                            for (int i = 0; i < centerSamples; ++i) {
                                sumX += (uint32_t)analogRead(pinX);
                                sumY += (uint32_t)analogRead(pinY);
                                delay(5);
                            }
                            uint16_t cenX = (uint16_t)(sumX / (uint32_t)centerSamples);
                            uint16_t cenY = (uint16_t)(sumY / (uint32_t)centerSamples);

                            // 计算“中心偏移”（复用原有 CenterCal 存储语义）
                            int32_t offsetX = (int32_t)ANALOG_STICK_CENTER_X - (int32_t)cenX;
                            int32_t offsetY = (int32_t)ANALOG_STICK_CENTER_Y - (int32_t)cenY;
                            const int32_t maxOffset = 512;
                            if (offsetX < -maxOffset) offsetX = -maxOffset;
                            if (offsetX >  maxOffset) offsetX =  maxOffset;
                            if (offsetY < -maxOffset) offsetY = -maxOffset;
                            if (offsetY >  maxOffset) offsetY =  maxOffset;

                            // 把采到的边缘范围平移到“应用中心偏移之后”的空间保存
                            int32_t calMinX = (int32_t)minX + offsetX;
                            int32_t calMaxX = (int32_t)maxX + offsetX;
                            int32_t calMinY = (int32_t)minY + offsetY;
                            int32_t calMaxY = (int32_t)maxY + offsetY;

                            if (calMinX < ANALOG_STICK_MIN_X) calMinX = ANALOG_STICK_MIN_X;
                            if (calMaxX > ANALOG_STICK_MAX_X) calMaxX = ANALOG_STICK_MAX_X;
                            if (calMinY < ANALOG_STICK_MIN_Y) calMinY = ANALOG_STICK_MIN_Y;
                            if (calMaxY > ANALOG_STICK_MAX_Y) calMaxY = ANALOG_STICK_MAX_Y;

                            uint32_t ok = 0;
                            // 简单有效性判断：范围太小就不启用缩放（仍然保存中心偏移）
                            if ((calMaxX - calMinX) >= 600 && (calMaxY - calMinY) >= 600) ok = 1;

                            OF_Prefs::settings[OF_Const::analogCenterOffsetX] = (uint32_t)offsetX;
                            OF_Prefs::settings[OF_Const::analogCenterOffsetY] = (uint32_t)offsetY;
                            OF_Prefs::settings[OF_Const::analogCalMinX] = (uint32_t)calMinX;
                            OF_Prefs::settings[OF_Const::analogCalMaxX] = (uint32_t)calMaxX;
                            OF_Prefs::settings[OF_Const::analogCalMinY] = (uint32_t)calMinY;
                            OF_Prefs::settings[OF_Const::analogCalMaxY] = (uint32_t)calMaxY;
                            OF_Prefs::settings[OF_Const::analogCalValid] = ok;

                            int saveErr = OF_Prefs::SaveSettings();
                            FW_Common::analogCenterJustCalibrated = true;

                            if (!OF_Serial::serialMode) {
                                Serial.print("RangeCal: rawMinX="); Serial.print(minX);
                                Serial.print(" rawMaxX="); Serial.print(maxX);
                                Serial.print(" rawMinY="); Serial.print(minY);
                                Serial.print(" rawMaxY="); Serial.println(maxY);
                                Serial.print("RangeCal: cenX="); Serial.print(cenX);
                                Serial.print(" cenY="); Serial.print(cenY);
                                Serial.print(" offX="); Serial.print(offsetX);
                                Serial.print(" offY="); Serial.println(offsetY);
                                Serial.print("RangeCal: calMinX="); Serial.print(calMinX);
                                Serial.print(" calMaxX="); Serial.print(calMaxX);
                                Serial.print(" calMinY="); Serial.print(calMinY);
                                Serial.print(" calMaxY="); Serial.print(calMaxY);
                                Serial.print(" OK="); Serial.print(ok ? "1" : "0");
                                Serial.print(" Save=");
                                Serial.println(saveErr == 0 ? "OK" : "FAIL");
                            }

                            #ifdef USES_DISPLAY
                                FW_Common::OLED.TopPanelUpdate(ok ? TXT_TOP_RANGECAL_OK : TXT_TOP_RANGECAL_WEAK);
                                delay(900);
                                FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                            #endif
                        }
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
                              // 显示状态栏提示
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_AUTOFIRE_FMT,
                                      OF_Prefs::toggles[OF_Const::autofire] ? PM_ON : PM_OFF);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(1000); // 显示1秒状态提示
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
                              FW_Common::UpdateBindings(false);  // Apply custom button mappings
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
                              FW_Common::UpdateBindings(false);  // Apply custom button mappings
                              if(!OF_Serial::serialMode)
                                  Serial.println("Mode changed to: Mouse/Keyboard");
                          }
                          
                          // Save the new mode settings
                          OF_Prefs::toggles[OF_Const::analogOutputMode] = FW_Common::buttons.analogOutput;
                          OF_Prefs::toggles[OF_Const::misterMode] = FW_Common::OLED.mister;
                          OF_Prefs::SaveToggles();
                          
                          // Update display to show new mode
                          #ifdef USES_DISPLAY
                              // 显示状态栏提示
                              if(FW_Common::OLED.mister) {
                                  FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_MISTER);
                              } else if(FW_Common::buttons.analogOutput) {
                                  FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_GAMEPAD);
                              } else {
                                  FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_MOUSEKB);
                              }
                              // 短暂显示状态后再更新菜单
                              delay(1000); // 显示1秒状态提示
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
                              // 显示状态栏提示
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_LOW_BTN_FMT,
                                      OF_Prefs::toggles[OF_Const::lowButtonsMode] ? PM_ON : PM_OFF);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(1000); // 显示1秒状态提示
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
                              // 显示状态栏提示
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_LAYOUT_FMT,
                                      OF_Prefs::profiles[OF_Prefs::currentProfile].irLayout == OF_Const::layoutDiamond
                                          ? PM_LAYOUT_DIAMOND : PM_LAYOUT_SQUARE);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(1000); // 显示1秒状态提示
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
                        case FW_Const::PauseMode_GunId:
                          pauseModeSelectingGunId = true;
                          gunIdModeSelection = (uint8_t)(OF_Prefs::settings[OF_Const::gunId] % 4);
                          if(!OF_Serial::serialMode) {
                              Serial.println("Gun ID: select P1-P4");
                          }
                          #ifdef USES_DISPLAY
                              FW_Common::OLED.PauseProfileUpdate(gunIdModeSelection, "P1", "P2", "P3", "P4");
                          #endif
                          break;
                        // 在这里插入新的 “Send Analog Stick As” 选项
                        case FW_Const::PauseMode_AnalogMode:
                        {
                            // 模式顺序：0=Stick, 1=DPad, 2=Keys
                            uint32_t mode = OF_Prefs::settings[OF_Const::analogMode];
                            mode = (mode + 1) % 3;
                            OF_Prefs::settings[OF_Const::analogMode] = mode;
                            OF_Prefs::SaveSettings();

                            const char* modeStrSerial;
                            const char* modeStrShort;
                            switch (mode) {
                                default:
                                case OF_Const::analogModeStick:
                                    modeStrSerial = "Gamepad Analog Stick";
                                    modeStrShort  = TXT_TOP_STICK_GAMEPAD;
                                    break;
                                case OF_Const::analogModeDpad:
                                    modeStrSerial = "Gamepad D-Pad";
                                    modeStrShort  = TXT_TOP_STICK_DPAD;
                                    break;
                                case OF_Const::analogModeKeys:
                                    modeStrSerial = "Keyboard Arrows";
                                    modeStrShort  = TXT_TOP_STICK_KEYS;
                                    break;
                            }

                            if (!OF_Serial::serialMode) {
                                Serial.print("Analog stick output mode set to ");
                                Serial.println(modeStrSerial);
                            }
                            #ifdef USES_DISPLAY
                                FW_Common::OLED.TopPanelUpdate(modeStrShort);
                                FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogMode);
                            #endif
                        }
                        break;
                        case FW_Const::PauseMode_AnalogKeysLayout:
                        {
                            uint32_t layout = OF_Prefs::settings[OF_Const::analogKeysLayout];
                            layout = (layout == OF_Const::analogKeysLayoutWASD) ? OF_Const::analogKeysLayoutArrows : OF_Const::analogKeysLayoutWASD;
                            OF_Prefs::settings[OF_Const::analogKeysLayout] = layout;
                            OF_Prefs::SaveSettings();
                            const char* layoutStr = (layout == OF_Const::analogKeysLayoutWASD) ? "WASD" : "Arrows";
                            if (!OF_Serial::serialMode) {
                                Serial.print("Stick keys layout: ");
                                Serial.println(layoutStr);
                            }
                            #ifdef USES_DISPLAY
                                char buf[24];
                                snprintf(buf, sizeof(buf), TXT_TOP_STICK_KEYS_FMT, layoutStr);
                                FW_Common::OLED.TopPanelUpdate(buf);
                                FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogKeysLayout);
                            #endif
                        }
                        break;
                        case FW_Const::PauseMode_AnalogDeadzone:
                        // 死区步进：0,5,10,15,20,25,30%，用 Trigger 循环切换
                        {
                            // 当前值
                            uint32_t dz = OF_Prefs::settings[OF_Const::analogDeadzone];
                            if (dz > 30) dz = 30;

                            // 进入该选项时先显示当前死区值，避免画面空白
                            #ifdef USES_DISPLAY
                                {
                                    char buf[24];
                                    snprintf(buf, sizeof(buf), TXT_TOP_DEADZONE_FMT, (unsigned)dz);
                                    FW_Common::OLED.TopPanelUpdate(buf);
                                    FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogDeadzone);
                                }
                            #endif

                            // 这里已经是 Trigger 的一次“按下并松开”事件，不需要再检测 pressed
                            static const uint8_t steps[] = {0,5,10,15,20,25,30};
                            int index = 0;
                            for (int i = 0; i < 7; ++i) {
                                if (dz <= steps[i]) { index = i; break; }
                            }
                            // 下一档，循环
                            index = (index + 1) % 7;
                            dz = steps[index];
                            OF_Prefs::settings[OF_Const::analogDeadzone] = dz;
                            OF_Prefs::SaveSettings();

                            if (!OF_Serial::serialMode) {
                                Serial.print("Analog deadzone set to ");
                                Serial.print(dz);
                                Serial.println("%");
                            }
                            #ifdef USES_DISPLAY
                                char buf[24];
                                snprintf(buf, sizeof(buf), TXT_TOP_DEADZONE_FMT, (unsigned)dz);
                                FW_Common::OLED.TopPanelUpdate(buf);
                                FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogDeadzone);
                            #endif
                        }
                        break;
                        case FW_Const::PauseMode_AnalogCenterCal:
                        {
                            int pinX = OF_Prefs::pins[OF_Const::analogX];
                            int pinY = OF_Prefs::pins[OF_Const::analogY];
                            if (pinX < 0 || pinY < 0) {
                                if (!OF_Serial::serialMode)
                                    Serial.println("CenterCal: analog pins not set (analogX/analogY)");
                                break;
                            }
                            // 读取当前模拟值，以当前物理位置作为“新中心”
                            int analogValueX = analogRead(pinX);
                            int analogValueY = analogRead(pinY);

                            int32_t offsetX = (int32_t)ANALOG_STICK_CENTER_X - (int32_t)analogValueX;
                            int32_t offsetY = (int32_t)ANALOG_STICK_CENTER_Y - (int32_t)analogValueY;

                            // 限制最大校正量，防止极端错误
                            const int32_t maxOffset = 512;
                            if (offsetX < -maxOffset) offsetX = -maxOffset;
                            if (offsetX >  maxOffset) offsetX =  maxOffset;
                            if (offsetY < -maxOffset) offsetY = -maxOffset;
                            if (offsetY >  maxOffset) offsetY =  maxOffset;

                            // settings.conf 是二进制保存/加载：这里直接以 int32 语义写入即可（两补码会被原样保存）
                            // 读取时再把 uint32_t 重新解释为 int32_t 使用
                            OF_Prefs::settings[OF_Const::analogCenterOffsetX] = (uint32_t)offsetX;
                            OF_Prefs::settings[OF_Const::analogCenterOffsetY] = (uint32_t)offsetY;
                            int saveErr = OF_Prefs::SaveSettings();
                            #ifdef USES_ANALOG
                            FW_Common::analogCenterJustCalibrated = true;  // 下次 AnalogStickPoll 会重置 IIR 滤波，中心立即生效
                            #endif

                            if (!OF_Serial::serialMode) {
                                Serial.print("CenterCal: rawX=");
                                Serial.print(analogValueX);
                                Serial.print(" rawY=");
                                Serial.print(analogValueY);
                                Serial.print(" -> offsetX=");
                                Serial.print(offsetX);
                                Serial.print(" offsetY=");
                                Serial.print(offsetY);
                                Serial.print(" storedX=");
                                Serial.print(OF_Prefs::settings[OF_Const::analogCenterOffsetX]);
                                Serial.print(" storedY=");
                                Serial.print(OF_Prefs::settings[OF_Const::analogCenterOffsetY]);
                                Serial.print(" Save=");
                                Serial.println(saveErr == 0 ? "OK" : "FAIL");
                            }
                            #ifdef USES_DISPLAY
                                FW_Common::OLED.TopPanelUpdate(TXT_TOP_CENTER_DONE);
                                FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                            #endif
                        }
                        break;
                        case FW_Const::PauseMode_AxisUnsignedToggle:
                        {
                            // 切换轴输出模式：Signed(-32767~32767) / Unsigned(0~max)
                            uint32_t mode = 0;
                            if (OF_Const::settingsTypesCount > OF_Const::axisUnsigned) {
                                mode = OF_Prefs::settings[OF_Const::axisUnsigned];
                            }
                            mode = mode ? 0 : 1; // 0 -> 1, 1 -> 0
                            OF_Prefs::settings[OF_Const::axisUnsigned] = mode;
                            OF_Prefs::SaveSettings();

                            // NOTE: 当前这里只保存兼容状态；unsignedAxis 的完整行为逻辑待后续恢复实现
                            Gamepad16.unsignedAxis = (mode != 0);

                            if (!OF_Serial::serialMode) {
                                Serial.print("Axis mode set to ");
                                Serial.println(mode ? "Unsigned (Joypad-OS)" : "Signed");
                            }
                            #ifdef USES_DISPLAY
                                {
                                    char buf[24];
                                    snprintf(buf, sizeof(buf), TXT_TOP_AXIS_FMT,
                                        mode ? PM_AXIS_UNSIGNED : PM_AXIS_SIGNED);
                                    FW_Common::OLED.TopPanelUpdate(buf);
                                }
                                FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AxisUnsignedToggle);
                            #endif
                        }
                        break;
                        case FW_Const::PauseMode_AnalogSwapSticks:
                        {
                            uint32_t v = 0;
                            if (OF_Const::settingsTypesCount > OF_Const::analogSwapSticks) {
                                v = OF_Prefs::settings[OF_Const::analogSwapSticks];
                            }
                            v = v ? 0 : 1;
                            OF_Prefs::settings[OF_Const::analogSwapSticks] = v;
                            OF_Prefs::SaveSettings();

                            // 这个开关直接控制 Gamepad16 输出时“摄像头/红外轴”和“物理摇杆轴”的左右分配
                            Gamepad16.stickRight = (v != 0);

                            if (!OF_Serial::serialMode) {
                                Serial.print("Swap sticks: ");
                                Serial.println(v ? "ON" : "OFF");
                            }
                            #ifdef USES_DISPLAY
                                {
                                    char buf[24];
                                    snprintf(buf, sizeof(buf), TXT_TOP_SWAP_FMT, v ? PM_ON : PM_OFF);
                                    FW_Common::OLED.TopPanelUpdate(buf);
                                }
                                delay(800);
                                FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogSwapSticks);
                            #endif
                        }
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
                              // 显示状态栏提示
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_RUMBLE_FFB_FMT,
                                      OF_Prefs::toggles[OF_Const::rumbleFF] ? PM_ON : PM_OFF);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(1000); // 显示1秒状态提示
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
                        case FW_Const::PauseMode_PlayTimer:
                        {
                          // 以 0/5/10/15/20 分钟循环
                          uint8_t m = PlayTimer::minutes;
                          if      (m == 0)  m = 5;
                          else if (m == 5)  m = 10;
                          else if (m == 10) m = 15;
                          else if (m == 15) m = 20;
                          else              m = 0;  // 20 -> OFF

                          PlayTimer::SetMinutes(m);

                          if(!OF_Serial::serialMode) {
                              Serial.print("Play timer set to ");
                              Serial.print((int)m);
                              Serial.println(" minutes");
                          }

                          #ifdef USES_DISPLAY
                              if (m == 0) {
                                  FW_Common::OLED.TopPanelUpdate(TXT_TOP_TIMER_OFF);
                              } else {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_TIMER_FMT, (unsigned)m);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(800);
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_PlayTimer);
                          #endif // USES_DISPLAY
                        }
                        break;
                        case FW_Const::PauseMode_AnalogInvertX:
                        {
                          // Toggle X 轴反转
                          uint32_t v = OF_Prefs::settings[OF_Const::analogInvertX];
                          v = v ? 0 : 1;
                          OF_Prefs::settings[OF_Const::analogInvertX] = v;
                          OF_Prefs::SaveSettings();

                          if (!OF_Serial::serialMode) {
                              Serial.print("Analog X axis invert: ");
                              Serial.println(v ? "ON" : "OFF");
                          }
                          #ifdef USES_DISPLAY
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_X_AXIS_FMT, v ? PM_INVERTED : PM_NORMAL);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(800);
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogInvertX);
                          #endif
                        }
                        break;
                        case FW_Const::PauseMode_AnalogInvertY:
                        {
                          // Toggle Y 轴反转
                          uint32_t v = OF_Prefs::settings[OF_Const::analogInvertY];
                          v = v ? 0 : 1;
                          OF_Prefs::settings[OF_Const::analogInvertY] = v;
                          OF_Prefs::SaveSettings();

                          if (!OF_Serial::serialMode) {
                              Serial.print("Analog Y axis invert: ");
                              Serial.println(v ? "ON" : "OFF");
                          }
                          #ifdef USES_DISPLAY
                              {
                                  char buf[24];
                                  snprintf(buf, sizeof(buf), TXT_TOP_Y_AXIS_FMT, v ? PM_INVERTED : PM_NORMAL);
                                  FW_Common::OLED.TopPanelUpdate(buf);
                              }
                              delay(800);
                              FW_Common::OLED.PauseListUpdate(ExtDisplay::ScreenPause_AnalogInvertY);
                          #endif
                        }
                        break;
                        case FW_Const::PauseMode_EscapeSignal:
                          SendEscapeKey();

                          #ifdef USES_DISPLAY
                              FW_Common::OLED.TopPanelUpdate(TXT_TOP_ESCAPE_SENT);
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
                              FW_Common::OLED.TopPanelUpdate(TXT_USING_PREFIX, OF_Prefs::profiles[OF_Prefs::currentProfile].name);
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

                    // 离开暂停菜单时重置并启动计时（若设置了非 0 分钟）
                    PlayTimer::ResetAndStart();

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

                        // 长按退出暂停时同样启动计时
                        PlayTimer::ResetAndStart();

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
            } else if(FW_Common::buttons.pressedReleased == FW_Const::ToggleMouseGamepadBtnMask) {
                // Toggle between mouse and gamepad mode
                if(!OF_Serial::serialMode) {
                    Serial.println("Toggling input mode...");
                }
                
                // Only toggle between mouse and gamepad mode, ignore MISTER mode
                if(!FW_Common::buttons.analogOutput) {
                    // Current: Mouse/KB -> Switch to Gamepad
                    FW_Common::buttons.analogOutput = true;
                    FW_Common::OLED.mister = false; // Ensure MISTER mode is disabled
                    FW_Common::UpdateBindings(false);
                    if(!OF_Serial::serialMode)
                        Serial.println("Mode changed to: Gamepad");
                    
                    // RGB灯反馈：蓝色表示手柄模式
                    #ifdef LED_ENABLE
                        OF_RGB::LedUpdate(0, 0, 255); // 蓝色
                    #endif // LED_ENABLE
                } else {
                    // Current: Gamepad -> Switch to Mouse/KB
                    FW_Common::buttons.analogOutput = false;
                    FW_Common::OLED.mister = false; // Ensure MISTER mode is disabled
                    FW_Common::UpdateBindings(false);
                    if(!OF_Serial::serialMode)
                        Serial.println("Mode changed to: Mouse/Keyboard");
                    
                    // RGB灯反馈：绿色表示鼠标模式
                    #ifdef LED_ENABLE
                        OF_RGB::LedUpdate(0, 255, 0); // 绿色
                    #endif // LED_ENABLE
                }
                
                // 震动反馈
                #ifdef USES_RUMBLE
                    if(OF_Prefs::toggles[OF_Const::rumble]) {
                        // 短震动反馈
                        analogWrite(OF_Prefs::pins[OF_Const::rumblePin], OF_Prefs::settings[OF_Const::rumbleStrength]);
                        delay(200);
                        #ifdef ARDUINO_ARCH_ESP32
                            analogWrite(OF_Prefs::pins[OF_Const::rumblePin], 0);
                        #else // rp2040
                            digitalWrite(OF_Prefs::pins[OF_Const::rumblePin], LOW);
                        #endif
                    }
                #endif // USES_RUMBLE
                
                // Save the new mode settings
                OF_Prefs::toggles[OF_Const::analogOutputMode] = FW_Common::buttons.analogOutput;
                OF_Prefs::toggles[OF_Const::misterMode] = FW_Common::OLED.mister;
                OF_Prefs::SaveToggles();
                
                // Update display to show new mode
                #ifdef USES_DISPLAY
                    if(FW_Common::buttons.analogOutput) {
                        FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_GAMEPAD);
                    } else {
                        FW_Common::OLED.TopPanelUpdate(TXT_TOP_MODE_MOUSEKB);
                    }
                    // Briefly show status then update menu
                    delay(1000); // Show status for 1 second
                    FW_Common::OLED.PauseListUpdate((int)FW_Common::pauseModeSelection);
                #endif // USES_DISPLAY
                
                // 恢复RGB灯到正常模式颜色
                #ifdef LED_ENABLE
                    delay(500); // 保持反馈颜色0.5秒
                    FW_Common::SetLedColorFromMode(); // 恢复根据当前模式的颜色
                #endif // LED_ENABLE
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

    if(BoyMode::IsEnabled())
        FW_Common::buttons.ReportDisable();
    else
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

        if(!BoyMode::IsEnabled() && millis() - lastUSBpoll >= POLL_RATE) {
            #ifdef USES_ANALOG
                if(FW_Common::analogIsValid) AnalogStickPoll();
            #endif // USES_ANALOG
            lastUSBpoll = millis();
            FW_Common::buttons.SendReports();
        }

        // Run through serial receive buffer once this run, if it has contents.
        if(!BoyMode::IsEnabled() && Serial.available()) OF_Serial::SerialProcessing();

        #ifdef MAMEHOOKER
            if(!BoyMode::IsEnabled() && OF_Serial::serialMode) OF_Serial::SerialHandling();                                   // Process the force feedback from the current queue.
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

    char buf[96];
    int pos = sprintf(&buf[0], "%.1f"
                                #ifdef GIT_HASH
                                "-%s"
                                #endif // GIT_HASH
                                , OPENFIRE_VERSION
                                #ifdef GIT_HASH
                                , GIT_HASH
                                #endif // GIT_HASH
                      );
#ifdef OPENFIRE_APP_FORK_BUILD
    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "+%s", OPENFIRE_APP_FORK_TAG_STR);
#endif
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

                    #ifdef ARDUINO_ARCH_ESP32
                    // LETTURA HARDWARE (uint16_t per coerenza ADC 12-bit)
                    uint16_t rawX = (uint16_t)analogRead(OF_Prefs::pins[OF_Const::analogX]);
                    uint16_t rawY = (uint16_t)analogRead(OF_Prefs::pins[OF_Const::analogY]);

                    // NORMALIZZAZIONE (Identica alla funzione AnalogStickPoll)
                    uint16_t analogValueX, analogValueY;

                    // Normalizzazione Asse X
                    if (rawX < ANALOG_STICK_DEADZONE_X_MIN) 
                        analogValueX = (uint16_t)map(rawX, 0, ANALOG_STICK_DEADZONE_X_MIN, 0, ANALOG_STICK_CENTER_X);
                    else if (rawX > ANALOG_STICK_DEADZONE_X_MAX) 
                        analogValueX = (uint16_t)map(rawX, ANALOG_STICK_DEADZONE_X_MAX, 4095, ANALOG_STICK_CENTER_X, 4095);
                    else 
                        analogValueX = ANALOG_STICK_CENTER_X;

                    // Normalizzazione Asse Y
                    if (rawY < ANALOG_STICK_DEADZONE_Y_MIN) 
                        analogValueY = (uint16_t)map(rawY, 0, ANALOG_STICK_DEADZONE_Y_MIN, 0, ANALOG_STICK_CENTER_Y);
                    else if (rawY > ANALOG_STICK_DEADZONE_Y_MAX) 
                        analogValueY = (uint16_t)map(rawY, ANALOG_STICK_DEADZONE_Y_MAX, 4095, ANALOG_STICK_CENTER_Y, 4095);
                    else 
                        analogValueY = ANALOG_STICK_CENTER_Y;
                    #else                  
                    // Leggiamo il valore puro dall'ADC (0-4095)
                    uint16_t analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
                    uint16_t analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);
                    #endif // COMMENTO
                    
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
    // 普通模式：计时结束后完全禁用所有按键输出（包括鼠标/键盘/手柄）
    if (PlayTimer::AreInputsLocked())
        return;
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
// =================== 696969 ========== funzione ottimizzata con calibrazione
void AnalogStickPoll()
{
    int analogValueX = analogRead(OF_Prefs::pins[OF_Const::analogX]);
    int analogValueY = analogRead(OF_Prefs::pins[OF_Const::analogY]);
    
    if(OF_Prefs::settings[OF_Const::analogMode] == OF_Const::analogModeStick) {
        auto scale_with_cal = [](int32_t v, uint32_t minU, uint32_t maxU) -> int32_t {
            const int32_t minV = (int32_t)minU;
            const int32_t maxV = (int32_t)maxU;
            if (maxV <= minV) return v;
            if (v <= minV) return ANALOG_STICK_MIN_X;
            if (v >= maxV) return ANALOG_STICK_MAX_X;
            const int64_t inSpan = (int64_t)(maxV - minV);
            const int64_t outSpan = (int64_t)(ANALOG_STICK_MAX_X - ANALOG_STICK_MIN_X);
            const int64_t num = ((int64_t)(v - minV) * outSpan) + (inSpan / 2); // 四舍五入
            return (int32_t)(ANALOG_STICK_MIN_X + (num / inSpan));
        };

        // -------- 可配置死区（0–30%）：以中心为基准 --------
        // 读取百分比（由暂停菜单写入），限制在 0–30 之间
        uint32_t dzPercent = 0;
        if (OF_Const::settingsTypesCount > OF_Const::analogDeadzone) {
            dzPercent = OF_Prefs::settings[OF_Const::analogDeadzone];
        }
        if (dzPercent > 30) dzPercent = 30;

        // 轻量滤波（IIR），抑制 ADC 抖动导致的“跳动”
        // 只影响 analogX/analogY（左摇杆），不影响摄像头/IR 映射
        static int32_t filtX = ANALOG_STICK_CENTER_X;
        static int32_t filtY = ANALOG_STICK_CENTER_Y;
        if (FW_Common::analogCenterJustCalibrated) {
            filtX = analogValueX;
            filtY = analogValueY;
            FW_Common::analogCenterJustCalibrated = false;
        } else {
            // alpha = 1/5 ≈ 0.2
            filtX = (filtX * 4 + analogValueX) / 5;
            filtY = (filtY * 4 + analogValueY) / 5;
        }

        // 直接把存储的 uint32_t 按 int32_t 语义解释为偏移
        int32_t offsetX = (int32_t)OF_Prefs::settings[OF_Const::analogCenterOffsetX];
        int32_t offsetY = (int32_t)OF_Prefs::settings[OF_Const::analogCenterOffsetY];

        int32_t adjX = filtX + offsetX;
        int32_t adjY = filtY + offsetY;

        // 限制在 ADC 合法范围内
        if (adjX < ANALOG_STICK_MIN_X) adjX = ANALOG_STICK_MIN_X;
        if (adjX > ANALOG_STICK_MAX_X) adjX = ANALOG_STICK_MAX_X;
        if (adjY < ANALOG_STICK_MIN_Y) adjY = ANALOG_STICK_MIN_Y;
        if (adjY > ANALOG_STICK_MAX_Y) adjY = ANALOG_STICK_MAX_Y;

        // 如果启用了“画圈校准（范围+中心）”，则把校准后的边缘范围缩放到完整 0..4095
        int32_t normX = adjX;
        int32_t normY = adjY;
        if (OF_Const::settingsTypesCount > OF_Const::analogCalValid &&
            OF_Prefs::settings[OF_Const::analogCalValid]) {
            normX = scale_with_cal(adjX, OF_Prefs::settings[OF_Const::analogCalMinX], OF_Prefs::settings[OF_Const::analogCalMaxX]);
            normY = scale_with_cal(adjY, OF_Prefs::settings[OF_Const::analogCalMinY], OF_Prefs::settings[OF_Const::analogCalMaxY]);
            // scale_with_cal 的输出范围是 ANALOG_STICK_MIN_X..MAX_X（与 X 相同的常量），对 Y 也适用
            if (normY < ANALOG_STICK_MIN_Y) normY = ANALOG_STICK_MIN_Y;
            if (normY > ANALOG_STICK_MAX_Y) normY = ANALOG_STICK_MAX_Y;
        }

        // 应用 X/Y 轴反转（在死区/缩放处理之后，映射到 Stick 物理范围）
        if (OF_Const::settingsTypesCount > OF_Const::analogInvertX &&
            OF_Prefs::settings[OF_Const::analogInvertX]) {
            normX = ANALOG_STICK_MIN_X + ANALOG_STICK_MAX_X - normX;
        }
        if (OF_Const::settingsTypesCount > OF_Const::analogInvertY &&
            OF_Prefs::settings[OF_Const::analogInvertY]) {
            normY = ANALOG_STICK_MIN_Y + ANALOG_STICK_MAX_Y - normY;
        }

        // 按整个量程的一半计算死区范围（4095/2 ≈ 2048）
        int32_t halfRange = (ANALOG_STICK_MAX_X - ANALOG_STICK_MIN_X) / 2; // 理论上约 2047
        int32_t dzIn      = (int32_t)halfRange * (int32_t)dzPercent / 100; // ADC 单位（进入死区阈值）
        // 加一点回滞，避免在阈值附近来回“抖动切换”
        int32_t dzHyst    = halfRange / 50; // ~2%
        if (dzHyst < 8) dzHyst = 8;
        int32_t dzOut     = dzIn + dzHyst;  // 离开死区阈值

        int32_t dx = normX - (int32_t)ANALOG_STICK_CENTER_X;
        int32_t dy = normY - (int32_t)ANALOG_STICK_CENTER_Y;

        // 按轴独立死区 + 回滞（比“X/Y 同时在死区”更稳定）
        static bool inDeadX = true;
        static bool inDeadY = true;

        int32_t adx = abs(dx);
        int32_t ady = abs(dy);

        if (inDeadX) {
            if (adx > dzOut) inDeadX = false;
        } else {
            if (adx < dzIn) inDeadX = true;
        }

        if (inDeadY) {
            if (ady > dzOut) inDeadY = false;
        } else {
            if (ady < dzIn) inDeadY = true;
        }

        uint16_t outX = (uint16_t)(inDeadX ? ANALOG_STICK_CENTER_X : (int)normX);
        uint16_t outY = (uint16_t)(inDeadY ? ANALOG_STICK_CENTER_Y : (int)normY);
        Gamepad16.moveStick(outX, outY);
    } else {
        uint32_t newPos = 0;
        // 与 analogModeStick 分支统一：settings 中保存的是“相对偏移量”（有符号），直接相加校正中心
        int32_t offX = (int32_t)OF_Prefs::settings[OF_Const::analogCenterOffsetX];
        int32_t offY = (int32_t)OF_Prefs::settings[OF_Const::analogCenterOffsetY];
        int32_t adjX = (int32_t)analogValueX + offX;
        int32_t adjY = (int32_t)analogValueY + offY;

        // 限制在 ADC 合法范围内
        if (adjX < ANALOG_STICK_MIN_X) adjX = ANALOG_STICK_MIN_X;
        if (adjX > ANALOG_STICK_MAX_X) adjX = ANALOG_STICK_MAX_X;
        if (adjY < ANALOG_STICK_MIN_Y) adjY = ANALOG_STICK_MIN_Y;
        if (adjY > ANALOG_STICK_MAX_Y) adjY = ANALOG_STICK_MAX_Y;

        // 画圈校准的范围缩放（让 DPad/Keys 的阈值判断也更稳定）
        if (OF_Const::settingsTypesCount > OF_Const::analogCalValid &&
            OF_Prefs::settings[OF_Const::analogCalValid]) {
            // 复用上面的缩放逻辑（X/Y 都映射到 0..4095）
            const int32_t minX = (int32_t)OF_Prefs::settings[OF_Const::analogCalMinX];
            const int32_t maxX = (int32_t)OF_Prefs::settings[OF_Const::analogCalMaxX];
            const int32_t minY = (int32_t)OF_Prefs::settings[OF_Const::analogCalMinY];
            const int32_t maxY = (int32_t)OF_Prefs::settings[OF_Const::analogCalMaxY];
            if (maxX > minX) {
                if (adjX <= minX) adjX = ANALOG_STICK_MIN_X;
                else if (adjX >= maxX) adjX = ANALOG_STICK_MAX_X;
                else adjX = (int32_t)(ANALOG_STICK_MIN_X + (((int64_t)(adjX - minX) * (ANALOG_STICK_MAX_X - ANALOG_STICK_MIN_X) + (int64_t)(maxX - minX) / 2) / (int64_t)(maxX - minX)));
            }
            if (maxY > minY) {
                if (adjY <= minY) adjY = ANALOG_STICK_MIN_Y;
                else if (adjY >= maxY) adjY = ANALOG_STICK_MAX_Y;
                else adjY = (int32_t)(ANALOG_STICK_MIN_Y + (((int64_t)(adjY - minY) * (ANALOG_STICK_MAX_Y - ANALOG_STICK_MIN_Y) + (int64_t)(maxY - minY) / 2) / (int64_t)(maxY - minY)));
            }
        }

        // TODO: need to consider inverted axis toggle, currently assumes axises are inverted by default
        if(adjY < (ANALOG_STICK_DEADZONE_Y_MIN-700))
            newPos = 2; // down
        else if(adjY > (ANALOG_STICK_DEADZONE_Y_MAX+700))
            newPos = 1; // up

        if(adjX < (ANALOG_STICK_DEADZONE_X_MIN-700))
            newPos |= 8; // right
        else if(adjX > (ANALOG_STICK_DEADZONE_X_MAX+700))
            newPos |= 4; // left

        switch(OF_Prefs::settings[OF_Const::analogMode]) {
        case OF_Const::analogModeDpad: Gamepad16.padUpdate(FW_Common::buttons.PadMaskConvert(newPos)); break;
        case OF_Const::analogModeKeys:
            {
                // 方向顺序: i=0 上, i=1 下, i=2 左, i=3 右
                // 箭头键: 库要求 k>=136，传 (HID码+136)；字母键: 库用 k 当 ASCII 查表，故 WASD 传 'w'/'a'/'s'/'d'
                static const uint8_t keysArrows[4] = { KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW };
                static const uint8_t keysWASD[4]   = { (uint8_t)'w', (uint8_t)'s', (uint8_t)'a', (uint8_t)'d' };
                const uint8_t* keys = (OF_Prefs::settings[OF_Const::analogKeysLayout] == OF_Const::analogKeysLayoutWASD) ? keysWASD : keysArrows;
                if (FW_Common::aStickADCLastPos ^ newPos) {
                    for (int i = 0; i < 4; ++i) {
                        if (FW_Common::aStickADCLastPos ^ newPos & 1 << i)
                            Keyboard.release(keys[i]);
                    }
                }
                for (int i = 0; i < 4; ++i) {
                    if (newPos & 1 << i)
                        Keyboard.press(keys[i]);
                }
            }
            break;
        }
        FW_Common::aStickADCLastPos = newPos;
    }
}
// =========== 696969 fine funzione ottimizzata con calibrazione =======================
#endif // USES_ANALOG

void SendEscapeKey()
{
    if (PlayTimer::AreInputsLocked()) {
        return;
    }
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
            FW_Common::pauseModeSelection = FW_Const::PauseMode_AnalogCenterCal;
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
        if(FW_Common::pauseModeSelection == FW_Const::PauseMode_AnalogCenterCal) {
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
        case FW_Const::PauseMode_AnalogCenterCal:
          Serial.println("Selecting: Center Calibrate");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(255,0,0);
          #endif // LED_ENABLE
          break;
        case FW_Const::PauseMode_AnalogRangeCal:
          Serial.println("Selecting: Range Calibrate");
          #ifdef LED_ENABLE
              OF_RGB::LedUpdate(255,0,0);
          #endif // LED_ENABLE
          break;
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

    for(uint i = 1; i <= PROFILE_COUNT; ++i) {
        if(bitRead(mask, i)) {
            FW_Common::SelectCalProfile(i - 1);
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
            FW_Common::OLED.TopPanelUpdate(TXT_TOP_RUMBLE_ON);
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
            FW_Common::OLED.TopPanelUpdate(TXT_TOP_RUMBLE_OFF);
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
        FW_Common::OLED.TopPanelUpdate(TXT_USING_PREFIX, OF_Prefs::profiles[OF_Prefs::currentProfile].name);
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
            FW_Common::OLED.TopPanelUpdate(TXT_TOP_SOLENOID_ON);
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
            FW_Common::OLED.TopPanelUpdate(TXT_TOP_SOLENOID_OFF);
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
        FW_Common::OLED.TopPanelUpdate(TXT_USING_PREFIX, OF_Prefs::profiles[OF_Prefs::currentProfile].name);
    #endif // USES_DISPLAY
}
#endif // USES_SOLENOID
