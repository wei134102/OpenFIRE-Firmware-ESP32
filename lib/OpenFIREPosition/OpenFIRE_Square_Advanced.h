#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_Square_Advanced.h
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 * da un progetto di:
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 */

#ifndef _OpenFIRE_Square_Advanced_h_
#define _OpenFIRE_Square_Advanced_h_

#include <stdint.h>
#include "OpenFIREConst.h"

// Dichiarazioni esterne per costanti globali definite altrove
// Assicurati che questi nomi corrispondano esattamente a quelli nel tuo file di costanti.
extern const int MouseMaxX;
extern const int MouseMaxY;
// extern const int CamResX; // Se usate direttamente qui per calcoli
// extern const int CamResY; // Se usate direttamente qui per calcoli

class OpenFIRE_Square {
public:
    //================================================================
    // ENUM PUBBLICO E API
    //================================================================
    
    // Costruttore
    OpenFIRE_Square();

    // Metodo principale di elaborazione
    void begin(const int* px, const int* py, unsigned int seen);
    
    // Metodi 'Getter' per l'accesso allo stato finale
    int X(int index) const { return FinalX[index]; }
    int Y(int index) const { return FinalY[index]; }
    unsigned int testSee(int index) const { return see[index]; }
    int testMedianX() const { return medianX; }
    int testMedianY() const { return medianY; }
    float H() const { return height; }
    float W() const { return width; }
    float Ang() const { return angle; }
    unsigned int seen() const { return seenFlags; }

private:




//================================================================
// COSTANTI DI TUNING FILTRO KALMAN  (Pos/Vel) – version 18
//================================================================

// Limiti spazio operativo (per calcolo centro/halfwidth)
static constexpr float base_kf_X_MIN              = 0.0f;
static constexpr float base_kf_X_MAX              = static_cast<float>(MouseMaxX * 3); // 12285
static constexpr float base_kf_Y_MIN              = 0.0f;
static constexpr float base_kf_Y_MAX              = static_cast<float>(MouseMaxY * 3); // 9213

static constexpr float base_kf_X_CENTER           = (base_kf_X_MAX + base_kf_X_MIN) / 2.0f; // 6142.5
static constexpr float base_kf_Y_CENTER           = (base_kf_Y_MAX + base_kf_Y_MIN) / 2.0f; // 4606.5
static constexpr float base_kf_HALF_WIDTH         = (base_kf_X_MAX - base_kf_X_MIN) / 2.0f; // 6142.5
static constexpr float base_kf_HALF_HEIGHT        = (base_kf_Y_MAX - base_kf_Y_MIN) / 2.0f; // 4606.5

// Inizializzazione covarianza P (Posizione, Velocità)
static constexpr float base_kf_INITIAL_P_POS_VALUE = 100.0f;
static constexpr float base_kf_INITIAL_P_VEL_VALUE =  10.0f;

// Clamp min/max di P e R
static constexpr float base_kf_MIN_COVARIANCE_VALUE = 1e-6f;
static constexpr float base_kf_MAX_P_VALUE           = 1e6f;

//─────────────────────────────────────────────────────────────────────────────
// 1) Q-process: rumore di processo (velocità costante)
static constexpr float base_kf_Q_MIN_PROCESS        =  0.2f;   // smoothing se lento
static constexpr float base_kf_Q_MAX_PROCESS        =  8.0f;   // elasticità se veloce

// 2) Soglie accelerazione per modulare Q
static constexpr float base_kf_ACCEL_Q_THRESH_START =  5.0f;   // sensibilità alta
static constexpr float base_kf_ACCEL_Q_THRESH_END   = 50.0f;   // reattività rapida

//─────────────────────────────────────────────────────────────────────────────
// 3) R-measure: rumore di misura (misura vs predizione)
static constexpr float base_kf_R_MIN                =  0.05f;  // massimo peso misura
static constexpr float base_kf_R_MAX                =1000.0f;  // smoothing molto forte se fermo

// 4) Soglie accelerazione per modulare R
static constexpr float base_kf_ACCEL_R_THRESH_START = 10.0f;   // sensibile ai piccoli movimenti
static constexpr float base_kf_ACCEL_R_THRESH_END   =100.0f;   // scala su gamma ridotta

//─────────────────────────────────────────────────────────────────────────────
// 5) Soglie smoothstep per influenza ai bordi
static constexpr float base_kf_R_X_EDGE_SMOOTH_START = 0.4f;   // parte già al 40%  
static constexpr float base_kf_R_X_EDGE_SMOOTH_END   = 1.0f;   // fino al bordo
static constexpr float base_kf_R_Y_EDGE_SMOOTH_START = 0.4f;
static constexpr float base_kf_R_Y_EDGE_SMOOTH_END   = 1.0f;

// 6) Valori di R alle estremità dei bordi
static constexpr float base_kf_R_AT_X_EDGE          =2000.0f;  // smoothing aggiuntivo ai bordi
static constexpr float base_kf_R_AT_Y_EDGE          =2000.0f;
static constexpr float base_kf_R_AT_X_EDGE_FOR_Y    = 500.0f;  // cross‐axis moderato
static constexpr float base_kf_R_AT_Y_EDGE_FOR_X    = 500.0f;

// 7) Influenza incrociata asse X↔Y (ridotta)
static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_X = 0.2f;
static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_Y = 0.2f;




    //================================================================
    // STATO INTERNO DEL FILTRO DI KALMAN (BASE - Pos/Vel, dt Implicito)
    //================================================================
    static float base_kf_x_state[4][2]; 
    static float base_kf_y_state[4][2]; 

    static float base_kf_p_x_00[4], base_kf_p_x_01[4];
    static float base_kf_p_x_10[4], base_kf_p_x_11[4];

    static float base_kf_p_y_00[4], base_kf_p_y_01[4];
    static float base_kf_p_y_10[4], base_kf_p_y_11[4];

    static float base_kf_last_measured_x[4]; 
    static float base_kf_last_measured_y[4]; 
    static float base_kf_last_vx_raw[4]; 
    static float base_kf_last_vy_raw[4]; 

    static bool base_kf_is_initialized_all_points; 



    //================================================================
    // STATO DEL TRACKER (Membri non-statici, specifici dell'istanza)
    //================================================================
    
    // Stato principale e di output
    int FinalX[4];
    int FinalY[4];
    int medianX;
    int medianY;
    unsigned int see[4];
    float height;
    float width;
    float angle;
    float angleOffset[4];

    // Variabili di stato interne
    unsigned int start;
    unsigned int seenFlags;
    
    // Variabili per la logica di stima e del filtro
    float xDistTop, xDistBottom, yDistLeft, yDistRight;
    float angleTop, angleBottom, angleLeft, angleRight;
    bool initialization_complete_flag_; // Flag per l'inizializzazione generica, se usata altrove
    unsigned int seenFlags_cam;
    
    float current_input_for_kf_x_[4];
    float current_input_for_kf_y_[4];


    //================================================================
    // STATO DEL FILTRO ADATTIVO (Membri statici, condivisi per i 4 punti)
    //================================================================
    // Mantenuti static come da tua specifica architettura a istanza unica.
    // Questi sono per la Kalman_filter() per i singoli punti.
    
    static inline float p_x[4];
    static inline float p_y[4];
    static inline float x_filt[4];
    static inline float y_filt[4];
    static inline float k_x[4];
    static inline float k_y[4];
    static inline float last_x[4];
    static inline float last_y[4];
    static inline float last_vx[4];
    static inline float last_vy[4];
    static inline float last_ax[4];
    static inline float last_ay[4];
    static inline float last_jerk_x[4];
    static inline float last_jerk_y[4];
    static inline float r_dynamic[4];
    static inline int stationary_counter[4];
    static inline float last_known_good_raw_x[4];
    static inline float last_known_good_raw_y[4];
    static inline float q_min_base[4];

    //================================================================
    // VARIABILI PER FILTRO KALMAN BASATO SU CENTRO DI MASSA
    // (Utilizzate solo da Kalman_filter_All())
    //================================================================
    
    static float center_x_filt;
    static float center_y_filt;
    static float center_p_x;
    static float center_p_y;
    static float center_k_x;
    static float center_k_y;
    static float last_center_x;
    static float last_center_y;
    static float last_center_vx;
    static float last_center_vy;
    static float last_center_ax;
    static float last_center_ay;
    static float last_center_jerk_x;
    static float last_center_jerk_y;
    static float center_r_dynamic; // Nota: nel codice della funzione, r_x_dynamic e r_y_dynamic sono variabili locali.
                                  // center_r_dynamic non viene più direttamente assegnato nella funzione.
                                  // Se vuoi esporre r_x_dynamic e r_y_dynamic come membri statici,
                                  // dovresti dichiarli qui e assegnarli nel codice della funzione.
    static float center_q_min_base;
    static int center_stationary_counter;
    static float last_known_good_center_x;
    static float last_known_good_center_y;
    
    // Flag per l'inizializzazione del filtro Kalman_filter_All()
    static bool is_kalman_initialized; 

    //================================================================
    // METODI PRIVATI
    //================================================================
    
    void Kalman_filter_base();
};

#endif // _OpenFIRE_Square_Advanced_h_
#endif //USE_SQUARE_ADVANCED