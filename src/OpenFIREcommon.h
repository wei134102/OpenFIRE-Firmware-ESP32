 /*!
 * @file OpenFIREcommon.h
 * @brief Shared runtime objects and methods used throughout the OpenFIRE firmware.
 *
 * @copyright That One Seong, 2025
 * @copyright GNU Lesser General Public License
 */ 

#ifndef _OPENFIRECOMMON_H_
#define _OPENFIRECOMMON_H_

#include <stdint.h>
#include <DFRobotIRPositionEx.h>
#ifdef USE_SQUARE_ADVANCED
    #include <OpenFIRE_Square_Advanced.h>
#else
    #include <OpenFIRE_Square.h>
#endif // USE_SQUARE_ADVANCED
#include <OpenFIRE_Diamond.h>
#include <OpenFIRE_Perspective.h>
#include <OpenFIREConst.h>
#include <LightgunButtons.h>
#include <TinyUSB_Devices.h>

#include "OpenFIREprefs.h"
#include "OpenFIREdisplay.h"
#include "OpenFIREDefines.h"
#include "OpenFIREconstant.h"

//inline AbsMouse5_ AbsMouse5(2); // 696969 rimosso da me

class FW_Common
{
public:
    //// Methods
    /// @brief    Sets feedback devices to appropriate I/O modes and initializes external devices.
    /// @details  Mainly Force Feedbacks, analog RGB, and digital devices (I2C peripherals or
    //            external NeoPixels)
    static void FeedbackSet();

    /// @brief    Unsets any currently mapped pins back to board defaults (non-pullup inputs).
    /// @note     This should be run before any sync operation, to ensure no problems
    ///           when setting pins to any new values.
    static void PinsReset();

    /// @brief    (Re-)sets IR camera position object.
    /// @note     This is run at startup, and can also be run from Docked mode
    ///           when new camera pins are defined.
    static void CameraSet();

    /// @brief    Macro for functions to run when gun enters new gunmode
    /// @param    GunMode_e
    ///           GunMode to switch to
    /// @note     Some cleanup functions are performed depending on the mode
    ///           being switched away from.
    static void SetMode(const FW_Const::GunMode_e&);

    /// @brief    Set new IR mode and apply it to the selected profile.
    /// @param    RunMode_e
    ///           IR Mode to switch to (either averaging modes, or Processing test mode)
    static void SetRunMode(const FW_Const::RunMode_e&);

    /// @brief    Gun Mode that handles calibration.
    /// @param    fromDesktop
    ///           Flag that's set when calibration is signalled from the Desktop App.
    ///           When true, any mouse position movements during calibration are skipped
    ///           to keep the process smooth for the end user.
    static void ExecCalMode(const bool &fromDesktop = false);

    /// @brief    Updates current sight position from IR cam.
    /// @details  Updates finalX and finalY values.
    static void GetPosition();

    /// @brief    Updates state of bad camera, prints when interval is met
    static void PrintIrError();

    /// @brief    Update the last seen value.
    /// @note     Only to be called during run mode, since this will modify the LED colour
    ///           of any (non-static) devices.
    static void UpdateLastSeen();

    #ifdef LED_ENABLE
    /// @brief    Macro that sets LEDs color depending on the mode it's set to
    static void SetLedColorFromMode();
    #endif // LED_ENABLE

    #ifdef USES_DISPLAY
    /// @brief    Redraws display based on current state.
    static void RedrawDisplay();
    #endif // USES_DISPLAY

    /// @brief    Applies loaded gun profile settings from profileData[PROFILE_COUNT]
    /// @param    profile
    ///           Profile slot number to load settings from.
    static bool SelectCalProfile(const int &profile);

    /// @brief    Set a new IR camera sensitivity, and apply to the currently selected calibration profile
    /// @param    sensitivity
    ///           New sensitivity to set for current cali profile.
    static void SetIrSensitivity(const int &sensitivity);

    /// @brief    Set a new IR layout type, and apply to the currently selected calibration profile
    /// @param    layout
    ///           New layout type to set for current cali profile.
    static void SetIrLayout(const int &layout);

    /// @brief    Saves profile settings to EEPROM
    /// @note     Blinks LEDs (if any) on success or failure.
    static int SavePreferences();

    /// @brief    Updates LightgunButtons::ButtonDesc[] buttons descriptor array
    ///           with new pin mappings and control bindings, if any.
    /// @param    rebindStrSel
    ///           Flag that determines whether to reset the bindings of the special macros playerStartBtn/playerSelectBtn
    static void UpdateBindings(const bool &rebindStrSel);

    /// @brief    Checks Button Descriptor and replaces instances of 0xFF/0xFE with player-relative Start/Select
    static void UpdateStartSelect();

    // =============== 696969 ==================================
    #ifdef CAM_SIMPLE_KALMAN_FILTER   
        static void Kalman_filter(uint8_t i, int& mouseX,int& mouseY);
    #endif //CAM_SIMPLE_KALMAN_FILTER   
    // =============== 696969 ==================================

    // initial gunmode
    static inline FW_Const::GunMode_e gunMode = FW_Const::GunMode_Init;

    // run mode
    static inline FW_Const::RunMode_e runMode = FW_Const::RunMode_Normal;

    // state flags, see StateFlag_e
    static inline uint32_t stateFlags = FW_Const::StateFlagsDtrReset;

    //// Camera
    // IR positioning camera
    static inline DFRobotIRPositionEx *dfrIRPos = nullptr;

    // flag to warn docked server if camera is not working currently
    static inline bool camNotAvailable = false;

    static inline uint32_t camWarningTimestamp = 0;
    #define CAM_WARNING_INTERVAL 3000

    // OpenFIRE Positioning - one for Square, one for Diamond, and a shared perspective object
    static inline OpenFIRE_Square OpenFIREsquare;
    static inline OpenFIRE_Diamond OpenFIREdiamond;
    static inline OpenFIRE_Perspective OpenFIREper;

    // IR coords stuff:
    static inline int mouseX;
    static inline int mouseY;
    static inline int moveXAxisArr[3] = {0, 0, 0};
    static inline int moveYAxisArr[3] = {0, 0, 0};
    static inline int moveIndex = 0;

    // ================================ 696969 for filter ==================================
    #ifdef TEST_CAM   
   
    static inline int positionX[4] = {0};
    static inline int positionY[4] = {0};
    #endif // TEST_CAM

    #ifdef CAM_SIMPLE_KALMAN_FILTER   
    static inline unsigned int seenFlags = 0;


    //#ifdef COMMENTO

// ------------- INIZIO SEZIONE COSTANTI GLOBALI E DI CLASSE (DA DEFINIRE/ADATTARE) -------------

//public: // o private, a seconda di dove vuoi dichiararle

    // ---- SOSTITUISCI QUESTI CON I TUOI VALORI REALI ----
    static constexpr int Actual_X_MIN = 0;
    static constexpr int Actual_X_MAX = 2046;
    static constexpr int Actual_Y_MIN = 0;
    static constexpr int Actual_Y_MAX = 1534;
    // ----------------------------------------------------

    // ---- Costanti Globali del Sistema Derivate ----
    static constexpr float X_MIN = static_cast<float>(Actual_X_MIN);
    static constexpr float X_MAX = static_cast<float>(Actual_X_MAX);
    static constexpr float Y_MIN = static_cast<float>(Actual_Y_MIN);
    static constexpr float Y_MAX = static_cast<float>(Actual_Y_MAX);

    static constexpr float WIDTH = (X_MAX - X_MIN);
    static constexpr float HEIGHT = (Y_MAX - Y_MIN);

    static constexpr float X_CENTER = (X_MAX + X_MIN) / 2.0f;
    static constexpr float Y_CENTER = (Y_MAX + Y_MIN) / 2.0f;
    static constexpr float HALF_WIDTH = WIDTH / 2.0f;
    static constexpr float HALF_HEIGHT = HEIGHT / 2.0f;

    // ---- Parametri Core del Filtro di Kalman ----
    static constexpr float Q_MAX = WIDTH / 4096.0f;    // Max rumore di processo
    static constexpr float R_BASE = HEIGHT / 8192.0f;  // Base rumore di misura (per r_dynamic)

    // ---- Parametri Dead Zone ----
    static constexpr float DEAD_ZONE_X = WIDTH * 0.002f;  // VALORE DA AFFINARE! (0.2% della larghezza)
    static constexpr float DEAD_ZONE_Y = HEIGHT * 0.002f; // VALORE DA AFFINARE! (0.2% dell'altezza)
    static constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;    // VALORE DA AFFINARE!
    static constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;  // VALORE DA AFFINARE!
    static constexpr float STATIONARY_THRESHOLD_EDGE_COUNT = 75.0f; // VALORE DA AFFINARE! (in numero di chiamate/cicli)
    static constexpr float STATIONARY_THRESHOLD_CENTER_COUNT = 50.0f; // VALORE DA AFFINARE!

    // ---- Parametri di Stabilizzazione ----
    static constexpr float EDGE_SMOOTHING = 1.5f;             // VALORE DA AFFINARE! (Esponente per stabilizzazione dal centro)
    static constexpr float MIN_PRECISION_FACTOR = 0.90f;      // VALORE DA AFFINARE! (Max smorzamento base da stabilizzazione combinata)
    static constexpr float CORNER_SMOOTHING_EXPONENT_X = 2.5f;  // VALORE DA AFFINARE!
    static constexpr float CORNER_SMOOTHING_EXPONENT_Y = 3.0f;  // VALORE DA AFFINARE! (reso meno aggressivo)
    static constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f; // VALORE DA AFFINARE! (Dist normalizzata per transizione centro/bordi)
    static constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;    // VALORE DA AFFINARE! (Esponente per peso transizione centro/bordi)

    // ---- Parametri di Predizione ----
    static constexpr float ACCEL_NORMALIZATION_FACTOR = WIDTH * 0.01f; // VALORE DA AFFINARE!

    // ---- Soglie Base per Q Dinamico ----
    static constexpr float ACCEL_Q_THRESH_X_BASE = WIDTH * 0.002f;
    static constexpr float JERK_Q_THRESH_X_BASE  = WIDTH * 0.001f;
    static constexpr float ACCEL_Q_THRESH_Y_BASE = HEIGHT * 0.002f;
    static constexpr float JERK_Q_THRESH_Y_BASE  = HEIGHT * 0.001f;

    // Range per transizione fluida di Q (basati sulle soglie base)
    static constexpr float ACCEL_Q_THRESH_X_LOWER = ACCEL_Q_THRESH_X_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_X_UPPER = ACCEL_Q_THRESH_X_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_X_LOWER  = JERK_Q_THRESH_X_BASE * 0.8f;
    static constexpr float JERK_Q_THRESH_X_UPPER  = JERK_Q_THRESH_X_BASE * 1.2f;
    static constexpr float ACCEL_Q_THRESH_Y_LOWER = ACCEL_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_Y_UPPER = ACCEL_Q_THRESH_Y_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_Y_LOWER  = JERK_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float JERK_Q_THRESH_Y_UPPER  = JERK_Q_THRESH_Y_BASE * 1.2f;

    // ---- Parametri per Smorzamento Adattivo Velocità ----
    static constexpr float SPEED_LOW_THRESHOLD = 1.5f;     // VALORE DA AFFINARE! (velocità assoluta in pixel/chiamata)
    static constexpr float SPEED_HIGH_THRESHOLD = WIDTH * 0.03f; // VALORE DA AFFINARE! (es. 3% della larghezza come velocità alta)
    static constexpr float DAMPING_AT_LOW_SPEED = 0.65f;   // VALORE DA AFFINARE! (Fattore di smorzamento a vel. basse, es. 0.65 -> riduce del 35%)
    static constexpr float DAMPING_AT_HIGH_SPEED = 0.98f;  // VALORE DA AFFINARE! (Fattore di smorzamento a vel. alte, es. 0.98 -> quasi nullo)

    // ---- Costanti per Modulazione Parametri Y Vicino ai Bordi X ----
    static constexpr float X_EDGE_PROXIMITY_DECAY_RATE = 20.0f; // **VALORE CHIAVE DA AFFINARE!** (controlla la ripidità della curva exp)

    // Valori target per Y quando X è molto vicino al bordo (basati sui valori base sopra)
    static constexpr float MIN_PRECISION_FACTOR_Y_AT_X_EDGE = 0.95f;   // VALORE DA AFFINARE! (Deve essere >= MIN_PRECISION_FACTOR)
    static constexpr float DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE = 0.80f;   // VALORE DA AFFINARE! (Deve essere >= DAMPING_AT_LOW_SPEED)
    static constexpr float DEAD_ZONE_Y_SCALE_AT_X_EDGE = 0.6f;         // VALORE DA AFFINARE! (< 1.0 per ridurre dead zone Y)

    // ---- Variabili di Stato del Filtro (Runtime) ----
    // Inizializzate per 4 emettitori/punti IR
    // q_min_base: rumore di processo minimo, usato quando il movimento è fluido.
    // Un valore piccolo indica alta fiducia nel modello di movimento.
    static inline float q_min_base[4] = { Q_MAX * 0.01f, Q_MAX * 0.01f, Q_MAX * 0.01f, Q_MAX * 0.01f}; // VALORE DA AFFINARE! (es. 1% di Q_MAX)

    // p_x, p_y: stima della covarianza dell'errore. Inizializzata a un valore che riflette l'incertezza iniziale.
    // Un valore più alto significa meno fiducia nella stima iniziale.
    static inline float p_x[4] = { WIDTH * 0.1f, WIDTH * 0.1f, WIDTH * 0.1f, WIDTH * 0.1f }; // VALORE DA AFFINARE!
    static inline float p_y[4] = { HEIGHT * 0.1f, HEIGHT * 0.1f, HEIGHT * 0.1f, HEIGHT * 0.1f }; // VALORE DA AFFINARE!

    // x_filt, y_filt: coordinate filtrate. Inizializzate al centro dello schermo.
    static inline float x_filt[4] = { X_CENTER, X_CENTER, X_CENTER, X_CENTER };
    static inline float y_filt[4] = { Y_CENTER, Y_CENTER, Y_CENTER, Y_CENTER };

    // k_x, k_y: Guadagno di Kalman. Calcolato runtime.
    static inline float k_x[4] = {0.0f};
    static inline float k_y[4] = {0.0f};

    // last_x, last_y: ultime coordinate filtrate. Inizializzate come x_filt, y_filt.
    static inline float last_x[4] = { X_CENTER, X_CENTER, X_CENTER, X_CENTER };
    static inline float last_y[4] = { Y_CENTER, Y_CENTER, Y_CENTER, Y_CENTER };

    // last_vx, last_vy, last_ax, last_ay, last_jerk_x, last_jerk_y: ultime velocità, accelerazioni, jerk. Inizializzate a 0.
    static inline float last_vx[4] = {0.0f};
    static inline float last_vy[4] = {0.0f};
    static inline float last_ax[4] = {0.0f};
    static inline float last_ay[4] = {0.0f};
    static inline float last_jerk_x[4] = {0.0f};
    static inline float last_jerk_y[4] = {0.0f};

    // r_dynamic: rumore di misura. Inizializzato con R_BASE. Può essere modificato altrove se necessario.
    static inline float r_dynamic[4] = { R_BASE, R_BASE, R_BASE, R_BASE };

    // stationary_counter: contatore per la logica di stazionarietà/blocco.
    static inline int stationary_counter[4] = {0};

    // All'interno della tua classe FW_Common, insieme alle altre variabili static inline:

    // Ultime coordinate GREZZE valide conosciute. Inizializzate al centro.
    // Assicurati che X_CENTER e Y_CENTER siano definite come static constexpr nella tua classe.
    static inline float last_known_good_raw_x[4] = { X_CENTER, X_CENTER, X_CENTER, X_CENTER };
    static inline float last_known_good_raw_y[4] = { Y_CENTER, Y_CENTER, Y_CENTER, Y_CENTER };

// ------------- FINE SEZIONE COSTANTI GLOBALI E DI CLASSE -------------


/*    
// Costanti globali (compile-time)
static constexpr int X_MIN = 0;
static constexpr int X_MAX = 2048;
static constexpr int Y_MIN = 0;
static constexpr int Y_MAX = 1536;

static constexpr float X_CENTER = (X_MAX + X_MIN) / 2;
static constexpr float Y_CENTER = (Y_MAX + Y_MIN) / 2;

static constexpr float q_max = (X_MAX - X_MIN) / 4096.0;
static constexpr float r_base = (Y_MAX - Y_MIN) / 8192.0;
static constexpr float DEAD_ZONE_X = X_MAX * 0.002;
static constexpr float DEAD_ZONE_Y = Y_MAX * 0.002;

// Costanti per stabilizzazione progressiva
static constexpr float EDGE_SMOOTHING = 1.5;  
static constexpr float MIN_PRECISION_FACTOR = 0.9;

// Pre-calcolo di valori utili derivati dalle costanti globali
static constexpr float HALF_WIDTH = (X_MAX - X_MIN) / 2.0f;
static constexpr float HALF_HEIGHT = (Y_MAX - Y_MIN) / 2.0f;

// Variabili globali statiche (runtime) per 4 posizioni
static inline float q_min_base[4] = {0.005, 0.005, 0.005, 0.005};  
static inline float p_x[4] = {q_min_base[0]}, p_y[4] = {q_min_base[0]};  
static inline float x_filt[4] = {X_MIN}, y_filt[4] = {Y_MIN};  
static inline float k_x[4], k_y[4];  
static inline float last_x[4] = {X_MIN}, last_y[4] = {Y_MIN};  
static inline float last_vx[4] = {0}, last_vy[4] = {0};  
static inline float last_ax[4] = {0}, last_ay[4] = {0};  
static inline float last_jerk_x[4] = {0}, last_jerk_y[4] = {0};  
static inline float r_dynamic[4] = {r_base};   
static inline int stationary_counter[4] = {0};  

*/

    #ifdef COMMENTO

    // Costanti globali (compile-time)
    static constexpr int X_MIN = 0;
    static constexpr int X_MAX = 2048;
    static constexpr int Y_MIN = 0;
    static constexpr int Y_MAX = 1536;

    static constexpr float q_max = (X_MAX - X_MIN) / 4096.0;
    static constexpr float r_base = (Y_MAX - Y_MIN) / 8192.0;
    static constexpr float DEAD_ZONE_X = X_MAX * 0.002;
    static constexpr float DEAD_ZONE_Y = Y_MAX * 0.002;

    // Variabili globali statiche (runtime) per 4 posizioni
    static inline float q_min_base[4] = {0.005, 0.005, 0.005, 0.005};  
    static inline float p_x[4] = {q_min_base[0]}, p_y[4] = {q_min_base[0]};  
    static inline float x_filt[4] = {X_MIN}, y_filt[4] = {Y_MIN};  
    static inline float k_x[4], k_y[4];  
    static inline float last_x[4] = {X_MIN}, last_y[4] = {Y_MIN};  
    static inline float last_vx[4] = {0}, last_vy[4] = {0};  
    static inline float last_ax[4] = {0}, last_ay[4] = {0};  
    static inline float last_jerk_x[4] = {0}, last_jerk_y[4] = {0};  
    static inline float r_dynamic[4] = {r_base};   
    static inline int stationary_counter[4] = {0};  
    static inline uint32_t last_timestamp[4] = {0, 0, 0, 0};
    #endif //COMMENTO

    //#endif //COMMENTO



    #ifdef COMMENTO

    
    // --- Variabili Globali per il filtro di Kalman ---
    // Stima dello stato attuale (posizione, velocità, accelerazione) per X e Y.
    // Queste variabili mantengono lo stato filtrato del mouse.
    static inline float estimatedX[4] = {0.0f};
    static inline float estimatedY[4] = {0.0f};
    static inline float velocityX[4] = {0.0f};
    static inline float velocityY[4] = {0.0f};
    static inline float accelerationX[4] = {0.0f};
    static inline float accelerationY[4] = {0.0f};

    // Covarianza dell'errore di stima (P) per X e Y.
    // P riflette l'incertezza nella nostra stima. Valori più alti = maggiore incertezza.
    static inline float P_x[4] = {1.0f};
    static inline float P_y[4] = {1.0f};

    // Variabile per il calcolo del tempo trascorso tra le esecuzioni del filtro.
    static inline unsigned long lastFilterExecutionTime[4] = {0};

    // --- Costanti di Kalman (valori base) ---
    // Questi valori sono fondamentali per il comportamento del filtro e richiedono calibrazione.
    static constexpr float BASE_R[4] = {1.5f}; // 0.05f;  // Rumore di misurazione base (Observation Noise). Determina quanto ci fidiamo della nuova misura raw.
    static constexpr float BASE_Q[4] = {0.05f};  //0.001f; // Incertezza del modello base (Process Noise). Riflette quanto è incerto il nostro modello di movimento.

    // --- Parametri di stabilità e reattività ---
    static constexpr float MAX_ACCELERATION_BASE[4] = {10000.0f}; // Accelerazione massima che il filtro può stimare.
    static constexpr float MAX_VELOCITY[4] = {5000.0f};           // Velocità massima che il filtro può stimare.

    // --- Parametri per la gestione dei movimenti bruschi volontari ---
    // Soglia in unità di coordinate del mouse. Se il "salto" tra la misura raw e la stima
    // supera questo valore, il filtro entra in modalità "reattiva" ai movimenti bruschi.
    // **Questo parametro è cruciale e va calibrato in base al tuo sensore!**
    static constexpr float BRUSQUE_MOVE_THRESHOLD[4] = {20.0f}; // Esempio: 20 unità del mouse.
    // Ottimizzazione: pre-calcolo del quadrato della soglia per evitare la sqrt() nel confronto principale.
    static constexpr float BRUSQUE_MOVE_THRESHOLD_SQ[4] = {BRUSQUE_MOVE_THRESHOLD[0] * BRUSQUE_MOVE_THRESHOLD[0]};

    static constexpr float BRUSQUE_MOVE_Q_MULTIPLIER[4] = {50.0f};  // Moltiplicatore per Q in caso di movimento brusco.
    static constexpr float BRUSQUE_MOVE_R_MULTIPLIER[4] = {10.0f};  // Moltiplicatore per R in caso di movimento brusco.

    // Coefficienti per l'adattamento dinamico di Q e R in base alla velocità generale del mouse.
    static constexpr float VELOCITY_Q_FACTOR[4] = {0.01f}; // Aumenta Q all'aumentare della velocità.
    static constexpr float VELOCITY_R_FACTOR[4] = {0.002f}; // Aumenta R all'aumentare della velocità.

    // Limiti per la covarianza P e per Q e R dinamici, per garantire stabilità ed evitare valori estremi.
    static constexpr float P_MAX[4] = {10.0f};
    static constexpr float P_MIN[4] = {0.001f};
    static constexpr float Q_MAX_ADAPTIVE[4] = {BASE_Q[0] * 100.0f};
    static constexpr float R_MAX_ADAPTIVE[4] = {BASE_R[0] * 50.0f}; 
    #endif // COMMENTO

    #endif // CAM_SIMPLE_KALMAN_FILTER
    // ================================ 696969 for filter ==================================

    // timer will set this to 1 when the IR position can update
    static inline volatile unsigned int irPosUpdateTick = 0;

    // Amount of IR points seen from last camera poll
    static inline unsigned int lastSeen = 0;

    // Cam Test Mode last timestamp of last print
    static inline unsigned long testLastStamp = 0;
    
    //// General Runtime Flags
    static inline bool justBooted = true;                              // For ops we need to do on initial boot (custom pins, joystick centering)
    static inline bool dockedSaving = false; //true; // false;  // 696969  se false invia dati di stick analogico, temperatura e tasti  - se true non invia nulla                     // To block sending test output in docked mode.

    static LightgunButtons buttons;

    static inline char playerStartBtn = '1';
    static inline char playerSelectBtn = '5';

    // For offscreen button stuff:
    static inline bool triggerPressedOffscreen = false;            // Set if shot offscreen; determines whether we release trigger btn code 1 or 2

    #ifdef USES_ANALOG
        static inline bool analogIsValid;                          // Flag set true if analog stick is mapped to valid nums
        static inline uint32_t aStickADCLastPos = 0;               // Analog-to-digital direction mask for digital outputs when settings[OF_Const::analogMode] is > 0
    #endif // USES_ANALOG

    #ifdef FOURPIN_LED
        static inline bool ledIsValid;                             // Flag set true if RGB pins are mapped to valid numbers
    #endif // FOURPIN_LED

    //// OLED Display interface
    #ifdef USES_DISPLAY
    static inline ExtDisplay OLED;
    // Selector for which option in the simple pause menu you're scrolled on.
    static inline uint8_t pauseModeSelection = 0;
    #ifdef MAMEHOOKER
    static inline uint16_t dispMaxLife = 0; 			                 // Max value for life in lifebar mode (100%)
    static inline uint16_t dispLifePercentage = 0; 		             // Actual value to show in lifebar mode #%
    #endif // MAMEHOOKER
    #endif // USES_DISPLAY
};

// button runtime data arrays
static inline LightgunButtonsStatic<ButtonCount> lgbData;

#ifdef ARDUINO_ARCH_ESP32

//#include <Arduino.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/queue.h>

// funzione per esp32 per emulare il comportamento di rp2040.fifo per la comunicazione multicore

class ESP32FIFO {
    private:
        QueueHandle_t core0_to_core1;
        QueueHandle_t core1_to_core0;
    
    public:
        ESP32FIFO(size_t size = 8) {
            core0_to_core1 = xQueueCreate(size, sizeof(uint32_t)); // FIFO Core 0 → Core 1
            core1_to_core0 = xQueueCreate(size, sizeof(uint32_t)); // FIFO Core 1 → Core 0
        }

        ~ESP32FIFO() {
            vQueueDelete(core0_to_core1);
            vQueueDelete(core1_to_core0);
        }
    
        inline void push(uint32_t value) {
            if (xPortGetCoreID() == 0) {
                xQueueSend(core0_to_core1, &value, portMAX_DELAY); // Core 0 → Core 1
            } else {
                xQueueSend(core1_to_core0, &value, portMAX_DELAY); // Core 1 → Core 0
            }
        }
    
        inline bool push_nb(uint32_t value) {
            if (xPortGetCoreID() == 0) {
                return xQueueSend(core0_to_core1, &value, 0) == pdTRUE;
            } else {
                return xQueueSend(core1_to_core0, &value, 0) == pdTRUE;
            }
        }
    
        inline uint32_t pop() {
            uint32_t value;
            if (xPortGetCoreID() == 0) {
                xQueueReceive(core1_to_core0, &value, portMAX_DELAY); // Core 0 legge da Core 1
            } else {
                xQueueReceive(core0_to_core1, &value, portMAX_DELAY); // Core 1 legge da Core 0
            }
            return value;
        }
    
        inline bool pop_nb(uint32_t *value) {
            if (xPortGetCoreID() == 0) {
                return xQueueReceive(core1_to_core0, value, 0) == pdTRUE;
            } else {
                return xQueueReceive(core0_to_core1, value, 0) == pdTRUE;
            }
        }
    
        inline size_t available() {
            if (xPortGetCoreID() == 0) {
                return uxQueueMessagesWaiting(core1_to_core0); // Core 0 legge da Core 1
            } else {
                return uxQueueMessagesWaiting(core0_to_core1); // Core 1 legge da Core 0
            }
        }
    };
    
extern ESP32FIFO esp32_fifo; // 8 elenti unit32_t come per rp2040.fifo  

#endif  // ARDUINO_ARCH_ESP32

#endif // _OPENFIRECOMMON_H_