#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_SquareAdvanced_.h
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
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

    enum class TrackingQuality {
        AWAITING_INITIALIZATION,
        TRACKING_OK,
        TRACKING_PARTIAL,
        TRACK_LOST_OR_POOR, // Era TRACKING_LOST_OR_POOR
        AWAITING_REACQUISITION
    };

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
    TrackingQuality getCurrentTrackingState() const { return current_tracking_state_; }

private:
/////////////////// nuovo non funzionante bene ////////////////////
#ifdef COMMENTO 
//================================================================
// COSTANTI DI TUNING DEL FILTRO DI KALMAN (BASE - Pos/Vel)
//================================================================
// Limiti dello schermo per il filtro (QUESTO È L'UNICO SPAZIO OPERATIVO)
static constexpr float base_kf_X_MIN = 0.0f;
static constexpr float base_kf_X_MAX = static_cast<float>(MouseMaxX) * 3.0f; 
static constexpr float base_kf_Y_MIN = 0.0f;
static constexpr float base_kf_Y_MAX = static_cast<float>(MouseMaxY) * 3.0f; 

// Shift per convertire le misurazioni IR originali (che sono 0-MouseMaxX/Y)
// nel sistema di coordinate [0, 3*MouseMaxX/Y] del filtro.
// NOTA: Questo shift è solo per l'input/output, il filtro opera su tutto il range [MIN, MAX].
static constexpr float base_kf_SHIFT_X = static_cast<float>(MouseMaxX);
static constexpr float base_kf_SHIFT_Y = static_cast<float>(MouseMaxY);

// Centro e metà larghezza dell'UNICO spazio operativo del filtro.
// Usati per calcolare la distanza normalizzata dai bordi di QUESTO spazio.
static constexpr float base_kf_X_CENTER = (base_kf_X_MAX + base_kf_X_MIN) / 2.0f;
static constexpr float base_kf_Y_CENTER = (base_kf_Y_MAX + base_kf_Y_MIN) / 2.0f;
static constexpr float base_kf_HALF_WIDTH = (base_kf_X_MAX - base_kf_X_MIN) / 2.0f;
static constexpr float base_kf_HALF_HEIGHT = (base_kf_Y_MAX - base_kf_Y_MIN) / 2.0f;


// Inizializzazione della covarianza dell'errore P per ogni stato (Posizione, Velocità)
static constexpr float base_kf_INITIAL_P_POS_VALUE = 100.0f;
static constexpr float base_kf_INITIAL_P_VEL_VALUE = 10.0f;

static constexpr float base_kf_MIN_COVARIANCE_VALUE = 1e-6f; // Valore minimo per P e R
static constexpr float base_kf_MAX_P_VALUE = 1e6f; // Limite superiore per P

// Range di Q (Covarianza del Processo - rumore del modello di velocità costante)
static constexpr float base_kf_Q_MIN_PROCESS = 0.5f;
static constexpr float base_kf_Q_MAX_PROCESS = 10.0f; // Manteniamo questo valore per la reattività

// Parametri per la modulazione di Q in base all'accelerazione calcolata esternamente
static constexpr float base_kf_ACCEL_Q_THRESH_START = 10.0f;
static constexpr float base_kf_ACCEL_Q_THRESH_END = 80.0f;
static constexpr float base_kf_Q_ACCEL_EXPONENT = 2.0f;

// Range di R (Covarianza della Misura)
static constexpr float base_kf_R_MIN = 0.1f;
static constexpr float base_kf_R_MAX = 2000.0f;

// Parametri per la modulazione di R in base all'accelerazione calcolata esternamente
static constexpr float base_kf_ACCEL_R_THRESH_START = 10.0f;
static constexpr float base_kf_ACCEL_R_THRESH_END = 50.0f;
static constexpr float base_kf_R_ACCEL_EXPONENT = 1.0f;

// Parametri per la modulazione asimmetrica di R ai bordi dello schermo operativo UNICO.
// Queste soglie si riferiscono al dist_norm che va da 0 (centro) a 1.0 (bordi estremi 0 o 3*MouseMaxX).
// L'effetto deve iniziare solo quando ci si avvicina ai VERI bordi dello spazio operativo del filtro (0 o 3*MouseMaxX).
// I valori 0.3333 (corrispondenti a MouseMaxX e 2*MouseMaxX nello spazio del filtro) sono le "vecchie zone di instabilità".
// Settando START a 0.60f, l'effetto inizia ben oltre queste zone, verso i bordi estremi.
static constexpr float base_kf_R_X_EDGE_SMOOTH_START = 0.60f; // <--- VALORE CORRETTO
static constexpr float base_kf_R_X_EDGE_SMOOTH_END = 0.90f;   // <--- VALORE CORRETTO
static constexpr float base_kf_R_Y_EDGE_SMOOTH_START = 0.60f; // <--- VALORE CORRETTO
static constexpr float base_kf_R_Y_EDGE_SMOOTH_END = 0.90f;   // <--- VALORE CORRETTO

// Questi valori sono la covarianza R che si raggiunge QUANDO si è ai bordi estremi dello spazio operativo.
// Devono essere alti per "frenare" il puntatore in quelle zone e impedirgli di andare troppo fuori.
static constexpr float base_kf_R_AT_X_EDGE = 1000.0f; // <--- VALORE CORRETTO
static constexpr float base_kf_R_AT_Y_EDGE = 1000.0f; // <--- VALORE CORRETTO
static constexpr float base_kf_R_AT_X_EDGE_FOR_Y = 5000.0f; // <--- VALORE CORRETTO (influenza incrociata, molto forte)
static constexpr float base_kf_R_AT_Y_EDGE_FOR_X = 5000.0f; // <--- VALORE CORRETTO (influenza incrociata, molto forte)

static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_X = 1.0f;
static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_Y = 1.0f;

static constexpr float base_kf_R_X_EDGE_EXPONENT = 4.0f;
static constexpr float base_kf_R_Y_EDGE_EXPONENT = 4.0f;

static constexpr float base_kf_MAX_EDGE_R_INFLUENCE_FACTOR = 2.0f;
#endif

///////////////////////// inizio 17 ter ////////////////////
#ifdef COMMENTO
//================================================================
// COSTANTI DI TUNING DEL FILTRO DI KALMAN (BASE - Pos/Vel) - VERSIONE 17BIS CONSOLIDATA E FINALIZZATA
//================================================================
// Limiti dello schermo per il filtro. QUESTO È L'UNICO SPAZIO OPERATIVO del filtro.
// Il filtro opera su questo range senza distinzioni di "reale" o "esteso".
static constexpr float base_kf_X_MIN = 0.0f;
static constexpr float base_kf_X_MAX = static_cast<float>(MouseMaxX * 3); // VALORE = 4095 * 3 = 12285.0f
static constexpr float base_kf_Y_MIN = 0.0f;
static constexpr float base_kf_Y_MAX = static_cast<float>(MouseMaxY * 3); // VALORE = 3071 * 3 = 9213.0f

// Centro e metà larghezza dell'UNICO spazio operativo del filtro.
// Questi sono calcolati correttamente in base a base_kf_X/Y_MAX e MIN.
static constexpr float base_kf_X_CENTER = (base_kf_X_MAX + base_kf_X_MIN) / 2.0f; // 6142.5f
static constexpr float base_kf_Y_CENTER = (base_kf_Y_MAX + base_kf_Y_MIN) / 2.0f; // 4606.5f
static constexpr float base_kf_HALF_WIDTH = (base_kf_X_MAX - base_kf_X_MIN) / 2.0f; // 6142.5f
static constexpr float base_kf_HALF_HEIGHT = (base_kf_Y_MAX - base_kf_Y_MIN) / 2.0f; // 4606.5f

// Inizializzazione della covarianza dell'errore P per ogni stato (Posizione, Velocità)
static constexpr float base_kf_INITIAL_P_POS_VALUE = 100.0f; 
static constexpr float base_kf_INITIAL_P_VEL_VALUE = 10.0f;

static constexpr float base_kf_MIN_COVARIANCE_VALUE = 1e-6f; // Valore minimo per P e R
static constexpr float base_kf_MAX_P_VALUE = 1e6f; // Limite superiore per P

// Range di Q (Covarianza del Processo - rumore del modello di velocità costante)
// Valori come forniti nella tua versione 17bis.
static constexpr float base_kf_Q_MIN_PROCESS = 0.5f; 
static constexpr float base_kf_Q_MAX_PROCESS = 5.0f;

// Parametri per la modulazione di Q in base all'accelerazione calcolata esternamente
// Valori come forniti nella tua versione 17bis.
static constexpr float base_kf_ACCEL_Q_THRESH_START = 10.0f; 
static constexpr float base_kf_ACCEL_Q_THRESH_END = 80.0f;
static constexpr float base_kf_Q_ACCEL_EXPONENT = 2.0f; 

// Range di R (Covarianza della Misura)
// Valori come forniti nella tua versione 17bis.
static constexpr float base_kf_R_MIN = 0.1f;
static constexpr float base_kf_R_MAX = 2000.0f; 

// Parametri per la modulazione di R in base all'accelerazione calcolata esternamente
// Valori come forniti nella tua versione 17bis.
static constexpr float base_kf_ACCEL_R_THRESH_START = 100.0f; 
static constexpr float base_kf_ACCEL_R_THRESH_END = 300.0f;
static constexpr float base_kf_R_ACCEL_EXPONENT = 3.0f; 

// Parametri per la modulazione asimmetrica di R ai bordi dello schermo operativo unico.
// Valori come forniti nella tua versione 17bis.
static constexpr float base_kf_R_X_EDGE_SMOOTH_START = 0.3f; 
static constexpr float base_kf_R_X_EDGE_SMOOTH_END = 0.98f; 
static constexpr float base_kf_R_Y_EDGE_SMOOTH_START = 0.5f; 
static constexpr float base_kf_R_Y_EDGE_SMOOTH_END = 0.98f; 

static constexpr float base_kf_R_AT_X_EDGE = 1.0f; 
static constexpr float base_kf_R_AT_Y_EDGE = 1.0f; 
static constexpr float base_kf_R_AT_X_EDGE_FOR_Y = 5000.0f; 
static constexpr float base_kf_R_AT_Y_EDGE_FOR_X = 1000.0f; 

static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_X = 1.0f; 
static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_Y = 1.0f; 

static constexpr float base_kf_R_X_EDGE_EXPONENT = 4.0f; 
static constexpr float base_kf_R_Y_EDGE_EXPONENT = 4.0f;

#endif
//////////////////////// fine 17 ter //////////////////////


//////////////////////// inizio 17 bis //////////////////////////////////////////
#ifndef COMMENTO // debug 17 - miglior risultato
    //================================================================
    // COSTANTI DI TUNING DEL FILTRO DI KALMAN (BASE - Pos/Vel) - DEBUG 17
    //================================================================
    // Limiti dello schermo per il filtro (usati per la modulazione R ai bordi)
    static constexpr float base_kf_X_MIN = 0.0f;
    static constexpr float base_kf_X_MAX = static_cast<float>(MouseMaxX*3);
    static constexpr float base_kf_Y_MIN = 0.0f;
    static constexpr float base_kf_Y_MAX = static_cast<float>(MouseMaxY*3);

    static constexpr float base_kf_X_CENTER = (base_kf_X_MAX + base_kf_X_MIN) / 2.0f;
    static constexpr float base_kf_Y_CENTER = (base_kf_Y_MAX + base_kf_Y_MIN) / 2.0f;
    static constexpr float base_kf_HALF_WIDTH = (base_kf_X_MAX - base_kf_X_MIN) / 2.0f;
    static constexpr float base_kf_HALF_HEIGHT = (base_kf_Y_MAX - base_kf_Y_MIN) / 2.0f;

    // Inizializzazione della covarianza dell'errore P per ogni stato (Posizione, Velocità)
    static constexpr float base_kf_INITIAL_P_POS_VALUE = 100.0f; 
    static constexpr float base_kf_INITIAL_P_VEL_VALUE = 10.0f;  

    static constexpr float base_kf_MIN_COVARIANCE_VALUE = 1e-6f; // Valore minimo per P e R
    static constexpr float base_kf_MAX_P_VALUE = 1e6f; // Limite superiore per P

    // Range di Q (Covarianza del Processo - rumore del modello di velocità costante)
    static constexpr float base_kf_Q_MIN_PROCESS = 0.5f; // Come in valori_12.txt
    static constexpr float base_kf_Q_MAX_PROCESS = 5.0f;   
    
    // Parametri per la modulazione di Q in base all'accelerazione calcolata esternamente
    static constexpr float base_kf_ACCEL_Q_THRESH_START = 10.0f; // Come in valori_12.txt
    static constexpr float base_kf_ACCEL_Q_THRESH_END = 80.0f;   // Come in valori_12.txt
    static constexpr float base_kf_Q_ACCEL_EXPONENT = 2.0f; // Come in valori_12.txt

    // Range di R (Covarianza della Misura)
    static constexpr float base_kf_R_MIN = 0.1f;   // Come in valori_12.txt
    static constexpr float base_kf_R_MAX = 2000.0f; // << RIPRISTINATO come in valori_12.txt
    
    // Parametri per la modulazione di R in base all'accelerazione calcolata esternamente
    // --- RIPRISTINATI I VALORI CHE FUNZIONAVANO PER R_base IN VALORI_12.TXT ---
    static constexpr float base_kf_ACCEL_R_THRESH_START = 100.0f; // << RIPRISTINATO come in valori_12.txt
    static constexpr float base_kf_ACCEL_R_THRESH_END = 300.0f;   // << RIPRISTINATO come in valori_12.txt
    static constexpr float base_kf_R_ACCEL_EXPONENT = 3.0f; // << RIPRISTINATO come in valori_12.txt

    // Parametri per la modulazione asimmetrica di R ai bordi dello schermo (per ridurre il "balletto")
    static constexpr float base_kf_R_X_EDGE_SMOOTH_START = 0.3f; // Mantenuto, per attivare l'effetto prima.
    static constexpr float base_kf_R_X_EDGE_SMOOTH_END = 0.98f; // Mantenuto, per concentrare l'effetto.
    static constexpr float base_kf_R_Y_EDGE_SMOOTH_START = 0.5f; // Mantenuto, per attivare l'effetto prima.
    static constexpr float base_kf_R_Y_EDGE_SMOOTH_END = 0.98f; // Mantenuto, per concentrare l'effetto.

    static constexpr float base_kf_R_AT_X_EDGE = 1.0f; 
    static constexpr float base_kf_R_AT_Y_EDGE = 1.0f; 
    static constexpr float base_kf_R_AT_X_EDGE_FOR_Y = 5000.0f; // << AUMENTATO (da 1000) drasticamente per i LATI VERTICALI (asse Y ai bordi X)
    static constexpr float base_kf_R_AT_Y_EDGE_FOR_X = 1000.0f; // Mantenuto, come in valori_12.txt

    static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_X = 1.0f; 
    static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_Y = 1.0f; 

    static constexpr float base_kf_R_X_EDGE_EXPONENT = 4.0f; 
    static constexpr float base_kf_R_Y_EDGE_EXPONENT = 4.0f; 
    #endif // debug 17
    //////////////////////////////////// fine 17 bis ///////////////////////////////////////////////////// 

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

    // Non abbiamo più bisogno di una variabile per il dt autonomo qui, dato che dt è implicito a 1.0f
    // static unsigned long base_kf_last_millis; // RIMOSSA

    //================================================================
    // COSTANTI DI TUNING DEL FILTRO DI KALMAN (POINTS - Pos/Vel/Acc)
    //================================================================
    // Limiti dello schermo per il filtro, derivati dalle tue costanti esistenti
    static constexpr float points_kf_X_MIN = 0.0f;
    static constexpr float points_kf_X_MAX = static_cast<float>(MouseMaxX);
    static constexpr float points_kf_Y_MIN = 0.0f;
    static constexpr float points_kf_Y_MAX = static_cast<float>(MouseMaxY);

    // Dati calcolati dai limiti, usati nella logica del filtro
    // Questi sono calcolati dal centro dello schermo per la modulazione R dei bordi
    static constexpr float points_kf_X_CENTER = (points_kf_X_MAX + points_kf_X_MIN) / 2.0f;
    static constexpr float points_kf_Y_CENTER = (points_kf_Y_MAX + points_kf_Y_MIN) / 2.0f;
    static constexpr float points_kf_HALF_WIDTH = (points_kf_X_MAX - points_kf_X_MIN) / 2.0f;
    static constexpr float points_kf_HALF_HEIGHT = (points_kf_Y_MAX - points_kf_Y_MIN) / 2.0f;

    // Inizializzazione della covarianza dell'errore P per ogni stato (Pos/Vel/Acc)
    static constexpr float points_kf_INITIAL_P_POS_VALUE = 100.0f; 
    static constexpr float points_kf_INITIAL_P_VEL_VALUE = 10.0f;  
    static constexpr float points_kf_INITIAL_P_ACC_VALUE = 1.0f;   

    // Valore minimo e massimo per le covarianze (per stabilità numerica e clamping)
    static constexpr float points_kf_MIN_COVARIANCE_VALUE = 1e-6f;
    static constexpr float points_kf_MAX_P_VALUE = 1e6f; // Limite superiore per P, evita divergenze estreme

    // Range di Q (Covarianza del Processo - rumore del modello, spesso sul jerk)
    static constexpr float points_kf_Q_MIN_JERK = 0.001f; // Valore minimo di Q per il jerk (movimenti lenti/fermi)
    static constexpr float points_kf_Q_MAX_JERK = 1.0f;   // Valore massimo di Q per il jerk (movimenti bruschi/veloci)
    
    // Parametri per la modulazione di Q in base all'accelerazione filtrata
    static constexpr float points_kf_ACCEL_Q_THRESH_START = 0.1f; // Accelerazione (filtrata) dove Q inizia ad aumentare
    static constexpr float points_kf_ACCEL_Q_THRESH_END = 5.0f;   // Accelerazione (filtrata) dove Q raggiunge points_kf_Q_MAX_JERK
    static constexpr float points_kf_Q_ACCEL_EXPONENT = 2.0f; // Esponente per la curva di Q (sintonizzare)

    // Range di R (Covarianza della Misura)
    static constexpr float points_kf_R_MIN = 0.1f;   // Valore minimo di R (movimenti bruschi/veloci)
    static constexpr float points_kf_R_MAX = 1000.0f; // Valore massimo di R (movimenti lenti/fermi)
    
    // Parametri per la modulazione di R in base all'accelerazione filtrata
    static constexpr float points_kf_ACCEL_R_THRESH_START = 0.05f; // Accelerazione (filtrata) dove R inizia a diminuire da points_kf_R_MAX
    static constexpr float points_kf_ACCEL_R_THRESH_END = 2.0f;    // Accelerazione (filtrata) dove R raggiunge points_kf_R_MIN
    static constexpr float points_kf_R_ACCEL_EXPONENT = 2.0f; // Esponente per la curva di R (sintonizzare)

    // Parametri per la modulazione asimmetrica di R ai bordi dello schermo (per ridurre il "balletto")
    static constexpr float points_kf_R_X_EDGE_SMOOTH_START = 0.7f;
    static constexpr float points_kf_R_X_EDGE_SMOOTH_END = 0.95f;
    static constexpr float points_kf_R_Y_EDGE_SMOOTH_START = 0.7f;
    static constexpr float points_kf_R_Y_EDGE_SMOOTH_END = 0.95f;

    static constexpr float points_kf_R_AT_X_EDGE = 1.0f; // R per X quando ai bordi X (per X stesso)
    static constexpr float points_kf_R_AT_Y_EDGE = 1.0f; // R per Y quando ai bordi Y (per Y stesso)
    static constexpr float points_kf_R_AT_X_EDGE_FOR_Y = 5.0f; // R per Y quando ai bordi X (per balletto verticale)
    static constexpr float points_kf_R_AT_Y_EDGE_FOR_X = 5.0f; // R per X quando ai bordi Y (per balletto orizzontale)

    static constexpr float points_kf_R_CROSS_AXIS_INFLUENCE_X = 1.0f; // Influenza dei bordi Y su R_x
    static constexpr float points_kf_R_CROSS_AXIS_INFLUENCE_Y = 1.0f; // Influenza dei bordi X su R_y

    static constexpr float points_kf_R_X_EDGE_EXPONENT = 2.0f; // Esponente per l'influenza dei bordi X su R_x
    static constexpr float points_kf_R_Y_EDGE_EXPONENT = 2.0f; // Esponente per l'influenza dei bordi Y su R_y

    // BASE DT per la normalizzazione:
    // Se Q e R sono stati sintonizzati assumendo che "dt=1.0f" corrisponda a 5ms.
    static constexpr float points_kf_BASE_DT_MS = 5.0f; 
    // Clamp per il dt misurato (evita dt estremi in caso di lag o spike)
    static constexpr float points_kf_MIN_MEASURED_DT_FACTOR = 0.1f; // Minimum dt factor (e.g. 0.5ms if base is 5ms)
    static constexpr float points_kf_MAX_MEASURED_DT_FACTOR = 5.0f;  // Maximum dt factor (e.g. 25ms if base is 5ms)

    //================================================================
    // STATO INTERNO DEL FILTRO DI KALMAN (POINTS)
    //================================================================
    // Stati per ciascuno dei 4 punti (Posizione, Velocità, Accelerazione per X e Y)
    static float points_kf_x_state[4][3]; // points_kf_x_state[point_idx][state_idx]
    static float points_kf_y_state[4][3]; // points_kf_y_state[point_idx][state_idx]

    // Matrici di covarianza dell'errore P (3x3 per ogni punto e per ogni asse)
    // Usiamo array per ogni componente della matrice P
    static float points_kf_p_x_00[4], points_kf_p_x_01[4], points_kf_p_x_02[4];
    static float points_kf_p_x_10[4], points_kf_p_x_11[4], points_kf_p_x_12[4];
    static float points_kf_p_x_20[4], points_kf_p_x_21[4], points_kf_p_x_22[4];

    static float points_kf_p_y_00[4], points_kf_p_y_01[4], points_kf_p_y_02[4];
    static float points_kf_p_y_10[4], points_kf_p_y_11[4], points_kf_p_y_12[4];
    static float points_kf_p_y_20[4], points_kf_p_y_21[4], points_kf_p_y_22[4];

    // Flag per l'inizializzazione *complessiva* del filtro (per tutti i 4 punti)
    static bool points_kf_is_initialized_all_points; // Sostituisce l'array di bool

    // Variabile per il calcolo autonomo del dt
    static unsigned long points_kf_last_millis; // AGGIUNTA QUESTA RIGA

    /*
    // Stato del tracking generale (non specifico del KF)
    TrackingQuality current_tracking_state_;
    unsigned int seenFlags; // per mantenere il tuo 'seenFlags' originale
    */

    //================================================================
    // COSTANTI DI TUNING DEL FILTRO DI KALMAN (CORE - Pos/Vel/Acc)
    //================================================================
    // Limiti dello schermo per il filtro, derivati dalle tue costanti esistenti
    static constexpr float core_kf_X_MIN = 0.0f;
    static constexpr float core_kf_X_MAX = static_cast<float>(MouseMaxX);
    static constexpr float core_kf_Y_MIN = 0.0f;
    static constexpr float core_kf_Y_MAX = static_cast<float>(MouseMaxY);

    // Dati calcolati dai limiti, usati nella logica del filtro
    static constexpr float core_kf_X_CENTER = (core_kf_X_MAX + core_kf_X_MIN) / 2.0f;
    static constexpr float core_kf_Y_CENTER = (core_kf_Y_MAX + core_kf_Y_MIN) / 2.0f;
    static constexpr float core_kf_HALF_WIDTH = (core_kf_X_MAX - core_kf_X_MIN) / 2.0f;
    static constexpr float core_kf_HALF_HEIGHT = (core_kf_Y_MAX - core_kf_Y_MIN) / 2.0f;

    // Inizializzazione della covarianza dell'errore P
    static constexpr float core_kf_INITIAL_P_POS_VALUE = 100.0f; // Inizializzazione per la posizione
    static constexpr float core_kf_INITIAL_P_VEL_VALUE = 10.0f;  // Inizializzazione per la velocità
    static constexpr float core_kf_INITIAL_P_ACC_VALUE = 1.0f;   // Inizializzazione per l'accelerazione

    // Valore minimo per le covarianze (per stabilità numerica)
    static constexpr float core_kf_MIN_COVARIANCE_VALUE = 1e-6f;

    // Range di Q (Covarianza del Processo - rumore del modello, spesso sul jerk)
    static constexpr float core_kf_Q_MIN_JERK = 0.001f; // Valore minimo di Q per il jerk (movimenti lenti/fermi)
    static constexpr float core_kf_Q_MAX_JERK = 1.0f;   // Valore massimo di Q per il jerk (movimenti bruschi/veloci)
    
    // Parametri per la modulazione di Q in base all'accelerazione filtrata
    static constexpr float core_kf_ACCEL_Q_THRESH_START = 0.1f; // Accelerazione (filtrata) dove Q inizia ad aumentare
    static constexpr float core_kf_ACCEL_Q_THRESH_END = 5.0f;   // Accelerazione (filtrata) dove Q raggiunge core_kf_Q_MAX_JERK
    static constexpr float core_kf_Q_ACCEL_EXPONENT = 2.0f; // Esponente per la curva di Q (sintonizzare)

    // Range di R (Covarianza della Misura)
    static constexpr float core_kf_R_MIN = 0.1f;   // Valore minimo di R (movimenti bruschi/veloci)
    static constexpr float core_kf_R_MAX = 1000.0f; // Valore massimo di R (movimenti lenti/fermi)
    
    // Parametri per la modulazione di R in base all'accelerazione filtrata
    static constexpr float core_kf_ACCEL_R_THRESH_START = 0.05f; // Accelerazione (filtrata) dove R inizia a diminuire da core_kf_R_MAX
    static constexpr float core_kf_ACCEL_R_THRESH_END = 2.0f;    // Accelerazione (filtrata) dove R raggiunge core_kf_R_MIN
    static constexpr float core_kf_R_ACCEL_EXPONENT = 2.0f; // Esponente per la curva di R (sintonizzare)

    // Parametri per la modulazione asimmetrica di R ai bordi dello schermo
    static constexpr float core_kf_R_X_EDGE_SMOOTH_START = 0.7f;
    static constexpr float core_kf_R_X_EDGE_SMOOTH_END = 0.95f;
    static constexpr float core_kf_R_Y_EDGE_SMOOTH_START = 0.7f;
    static constexpr float core_kf_R_Y_EDGE_SMOOTH_END = 0.95f;

    static constexpr float core_kf_R_AT_X_EDGE = 1.0f; // R per X quando ai bordi X (per X stesso)
    static constexpr float core_kf_R_AT_Y_EDGE = 1.0f; // R per Y quando ai bordi Y (per Y stesso)
    static constexpr float core_kf_R_AT_X_EDGE_FOR_Y = 5.0f; // R per Y quando ai bordi X (per balletto verticale)
    static constexpr float core_kf_R_AT_Y_EDGE_FOR_X = 5.0f; // R per X quando ai bordi Y (per balletto orizzontale)

    static constexpr float core_kf_R_CROSS_AXIS_INFLUENCE_X = 1.0f; // Influenza dei bordi Y su R_x
    static constexpr float core_kf_R_CROSS_AXIS_INFLUENCE_Y = 1.0f; // Influenza dei bordi X su R_y

    static constexpr float core_kf_R_X_EDGE_EXPONENT = 2.0f; // Esponente per l'influenza dei bordi X su R_x
    static constexpr float core_kf_R_Y_EDGE_EXPONENT = 2.0f; // Esponente per l'influenza dei bordi Y su R_y

    //================================================================
    // STATO INTERNO DEL FILTRO DI KALMAN (CORE)
    //================================================================
    static float core_kf_x_state[3]; // core_kf_x_state[0]=pos_x, core_kf_x_state[1]=vel_x, core_kf_x_state[2]=acc_x
    static float core_kf_y_state[3]; // core_kf_y_state[0]=pos_y, core_kf_y_state[1]=vel_y, core_kf_y_state[2]=acc_y

    // Matrici di covarianza dell'errore P (3x3 per ogni asse)
    static float core_kf_p_x_00, core_kf_p_x_01, core_kf_p_x_02;
    static float core_kf_p_x_10, core_kf_p_x_11, core_kf_p_x_12;
    static float core_kf_p_x_20, core_kf_p_x_21, core_kf_p_x_22;

    static float core_kf_p_y_00, core_kf_p_y_01, core_kf_p_y_02;
    static float core_kf_p_y_10, core_kf_p_y_11, core_kf_p_y_12;
    static float core_kf_p_y_20, core_kf_p_y_21, core_kf_p_y_22;

    // Variabili per l'inizializzazione del filtro
    static float core_kf_last_measured_x;
    static float core_kf_last_measured_y;
    static bool core_kf_is_initialized; // Flag per l'inizializzazione al primo frame

    ///// Stato del tracking generale (non specifico del KF)
    ///TrackingQuality current_tracking_state_;


    //================================================================
    // COSTANTI DI TUNING (COMUNI A ENTRAMBI I FILTRI O GENERALI)
    //================================================================
    static constexpr int Actual_X_MIN = 0;
    static constexpr int Actual_X_MAX = MouseMaxX;
    static constexpr int Actual_Y_MIN = 0;
    static constexpr int Actual_Y_MAX = MouseMaxY;

    static constexpr float KF_X_MIN = static_cast<float>(Actual_X_MIN);
    static constexpr float KF_X_MAX = static_cast<float>(Actual_X_MAX);
    static constexpr float KF_Y_MIN = static_cast<float>(Actual_Y_MIN);
    static constexpr float KF_Y_MAX = static_cast<float>(Actual_Y_MAX);

    static constexpr float KF_WIDTH = (KF_X_MAX - KF_X_MIN);
    static constexpr float KF_HEIGHT = (KF_Y_MAX - KF_Y_MIN);

    static constexpr float KF_X_CENTER = (KF_X_MAX + KF_X_MIN) / 2.0f;
    static constexpr float KF_Y_CENTER = (KF_Y_MAX + KF_Y_MIN) / 2.0f;
    static constexpr float KF_HALF_WIDTH = KF_WIDTH / 2.0f;
    static constexpr float KF_HALF_HEIGHT = KF_HEIGHT / 2.0f;

    static constexpr float MIN_COVARIANCE_AND_R = 1e-9f;

    static constexpr float SPEED_LOW_THRESHOLD = 1.5f;
    static constexpr float SPEED_HIGH_THRESHOLD = KF_WIDTH * 0.03f;
    static constexpr float DAMPING_AT_HIGH_SPEED = 1.0f; 

    // Dead-Zone (Potenzialmente usate da entrambi, verificare in Kalman_filter())
    static constexpr float DEAD_ZONE_X = KF_WIDTH * 0.0005f; 
    static constexpr float DEAD_ZONE_Y = KF_HEIGHT * 0.0005f; 
    static constexpr float STATIONARY_THRESHOLD_CENTER_COUNT = 10.0f; 

    //================================================================
    // COSTANTI SPECIFICHE PER Kalman_filter_All() (Centro di Massa)
    //================================================================
    static constexpr float INITIAL_P_VALUE = 100.0f; // Per l'inizializzazione di P

    // Parametri per la modulazione di Q in Kalman_filter_All
    static constexpr float Q_MAX = KF_WIDTH / 2048.0f; // Max Q per Kalman_filter_All
    static constexpr float ACCEL_Q_THRESH_LOWER = KF_WIDTH * 0.0016f;
    static constexpr float ACCEL_Q_THRESH_UPPER = KF_WIDTH * 0.0024f;
    static constexpr float JERK_Q_THRESH_LOWER  = KF_WIDTH * 0.0008f;
    static constexpr float JERK_Q_THRESH_UPPER  = KF_WIDTH * 0.0012f;

    // Parametri di Smorzamento per Kalman_filter_All
    static constexpr float DAMPING_AT_LOW_SPEED_X = 0.92f; // Base damping per X in Kalman_filter_All
    static constexpr float DAMPING_AT_LOW_SPEED_Y = 0.92f; // Base damping per Y in Kalman_filter_All
    static constexpr float DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE = 0.80f; // Damping specifico per Y ai bordi X

    // Parametri R per Kalman_filter_All (Logica Asimmetrica)
    static constexpr float R_BASE_CENTER_X = 0.1f;
    static constexpr float R_BASE_CENTER_Y = 0.1f;
    static constexpr float R_AT_X_EDGE = 1.0f;
    static constexpr float R_AT_Y_EDGE = 1.0f;
    static constexpr float R_AT_X_EDGE_FOR_Y = 5.0f; // CRITICO per il traballamento verticale

    static constexpr float R_CROSS_AXIS_INFLUENCE = 0.5f;
    static constexpr float R_CROSS_AXIS_INFLUENCE_Y = 1.5f;

    // smoothstep ranges for Kalman_filter_All
    static constexpr float X_EDGE_PROXIMITY_SMOOTH_START = 0.6f; 
    static constexpr float X_EDGE_PROXIMITY_SMOOTH_END = 0.95f; 
    static constexpr float R_X_EDGE_SMOOTH_START = 0.7f;
    static constexpr float R_X_EDGE_SMOOTH_END = 0.95f;
    static constexpr float R_Y_EDGE_SMOOTH_START = 0.7f;
    static constexpr float R_Y_EDGE_SMOOTH_END = 0.95f;

    // Generali per Kalman_filter_All (non più usate nel dettaglio ma le mantengo per coerenza)
    static constexpr float EDGE_SMOOTHING_EXP = 1.5f; // Usata in Kalman_filter_All

    // Nuovi parametri per la modulazione esponenziale/potenza di Q
    static constexpr float Q_EXPONENT = 2.0f; // Sintonizza questo: >1 per reattività più concentrata ad alti movimenti, <1 per più rapida ai bassi

    // Nuovi parametri per la modulazione esponenziale/potenza di R (specie per Y ai bordi X)
    static constexpr float R_Y_EDGE_EXPONENT = 3.0f; // Sintonizza questo: >1 per effetto concentrato sul bordo estremo

    // Nuovi parametri per la "spinta" a bassa velocità (se scegli di implementarla)
    // static constexpr float R_PUSH_AT_VERY_LOW_MOTION = 10.0f; // Esempio: valore da aggiungere a R a basse velocità
    // static constexpr float LOW_MOTION_THRESH_VELOCITY = 0.5f; // Esempio: soglia di velocità per bassa motion
    // static constexpr float LOW_MOTION_THRESH_ACCEL = 0.1f; // Esempio: soglia di accelerazione per bassa motion

    // Parametri per la "spinta" di R a bassa velocità/accelerazione
    static constexpr float R_PUSH_AT_VERY_LOW_MOTION = 50.0f; // Valore da aggiungere a R a basse velocità (aumentalo se serve)
    static constexpr float LOW_MOTION_THRESH_MAGNITUDE_START = 0.5f; // Soglia di motion_magnitude dove la spinta inizia
    static constexpr float LOW_MOTION_THRESH_MAGNITUDE_END = 1.5f; // Soglia di motion_magnitude dove la spinta finisce

    // Valore di R per l'asse X quando si è ai bordi Y (superiore/inferiore)
    static constexpr float R_AT_Y_EDGE_FOR_X = 5.0f; // Sintonizza questo valore (es. 5.0f, 8.0f, 10.0f)

    // Esponente per l'influenza dei bordi Y su R_x
    static constexpr float R_X_EDGE_EXPONENT = 3.0f; // Sintonizza questo (es. 2.0f, 3.0f, 4.0f)

    // Influenza dei bordi Y su R_x (simile a R_CROSS_AXIS_INFLUENCE_Y)
    static constexpr float R_CROSS_AXIS_INFLUENCE_X = 1.5f; // Sintonizza questo (es. 1.0f, 1.5f, 2.0f)

    //================================================================
    // COSTANTI SPECIFICHE PER Kalman_filter() (Singoli Punti) - REINSERITE
    //================================================================
    // Queste costanti sono state reintrodotte per permettere a Kalman_filter() di compilare.
    // I loro valori potrebbero richiedere una sintonizzazione se questa funzione è ancora in uso attivo.
    static constexpr float R_BASE = KF_HEIGHT / 16384.0f; // Questa era già nel tuo codice originale
    static constexpr float R_DYNAMIC_SLOW_MOTION = R_BASE * 120.0f; 
    static constexpr float R_DYNAMIC_VERY_FAST_MOTION = R_BASE * 0.01f; 

    static constexpr float DAMPING_AT_LOW_SPEED = 0.92f; // Vecchio damping unificato
    
    // Vecchie soglie Q per singoli punti (X e Y differenziate)
    static constexpr float ACCEL_Q_THRESH_X_BASE = KF_WIDTH * 0.002f;
    static constexpr float JERK_Q_THRESH_X_BASE  = KF_WIDTH * 0.001f;
    static constexpr float ACCEL_Q_THRESH_Y_BASE = KF_HEIGHT * 0.002f;
    static constexpr float JERK_Q_THRESH_Y_BASE  = KF_HEIGHT * 0.001f;

    static constexpr float ACCEL_Q_THRESH_X_LOWER = ACCEL_Q_THRESH_X_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_X_UPPER = ACCEL_Q_THRESH_X_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_X_LOWER  = JERK_Q_THRESH_X_BASE * 0.8f; // Corretto un typo JERK_Q_THRESH_X_X_BASE
    static constexpr float JERK_Q_THRESH_X_UPPER  = JERK_Q_THRESH_X_BASE * 1.2f;
    static constexpr float ACCEL_Q_THRESH_Y_LOWER = ACCEL_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_Y_UPPER = ACCEL_Q_THRESH_Y_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_Y_LOWER  = JERK_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float JERK_Q_THRESH_Y_UPPER  = JERK_Q_THRESH_Y_BASE * 1.2f;

    static constexpr float X_EDGE_PROXIMITY_DECAY_RATE = 20.0f; // Usata da expf
    static constexpr float EDGE_SMOOTHING = 1.5f; // Vecchio nome del parametro
    static constexpr float ACCEL_NORMALIZATION_FACTOR = 20.0f; // Per t_pred_a
    
    // Queste costanti esistevano ma potrebbero non essere usate da Kalman_filter() o Kalman_filter_All()
    static constexpr float MIN_PRECISION_FACTOR = 0.90f;
    static constexpr float CORNER_SMOOTHING_EXPONENT_X = 2.5f;
    static constexpr float CORNER_SMOOTHING_EXPONENT_Y = 3.0f;
    static constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f;
    static constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;
    static constexpr float MIN_PRECISION_FACTOR_Y_AT_X_EDGE = 0.95f;
    static constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;
    static constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f;
    static constexpr float STATIONARY_THRESHOLD_EDGE_COUNT = 15.0f;
    static constexpr float DEAD_ZONE_Y_SCALE_AT_X_EDGE = 0.6f;
    static constexpr float EDGE_SKEPTICISM_EXPONENT = 2.5f;
    static constexpr float R_SLOW_AT_CENTER = R_BASE * 60.0f;
    static constexpr float R_SLOW_AT_EDGE = R_BASE * 250.0f;
    static constexpr float R_SLOW_AT_HORIZONTAL_EDGE = R_BASE * 120.0f;
    static constexpr float R_SLOW_AT_VERTICAL_EDGE = R_BASE * 400.0f;


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
    TrackingQuality current_tracking_state_;
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