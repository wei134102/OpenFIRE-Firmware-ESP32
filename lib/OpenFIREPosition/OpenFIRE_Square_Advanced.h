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

#ifdef COMMENTO
class OpenFIRE_Square {
public:
    // ---- STATI PER LA QUALITÀ DEL TRACCIAMENTO ----
    enum class TrackingQuality {
        AWAITING_INITIALIZATION,
        TRACKING_OK,
        TRACKING_PARTIAL,
        TRACKING_LOST_OR_POOR,
        AWAITING_REACQUISITION
    };

    // ---- COSTANTI PUBBLICHE DEL SISTEMA E DEL FILTRO ----
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

    static constexpr float MAX_ASSOCIATION_DISTANCE_SQ = (KF_WIDTH * 0.25f) * (KF_WIDTH * 0.25f); // ESEMPIO! VALORE DA AFFINARE!

    static constexpr float Q_MAX = KF_WIDTH / 4096.0f; // KF_WIDTH / 65536.0f; // KF_WIDTH / 4096.0f;     // VALORE DA AFFINARE!
    static constexpr float R_BASE = KF_HEIGHT / 8192.0f;   // VALORE DA AFFINARE!

    // Nuove costanti per la sintonizzazione dinamica di R (Covarianza di Misura)
    // MIN_COVARIANCE_AND_R: Valore minimo assoluto per covarianze per evitare instabilità numerica.
    static constexpr float MIN_COVARIANCE_AND_R = 1e-9f;

    // R_DYNAMIC_SLOW_MOTION: Valore di R quando il movimento è lento/stabile (più smoothing, filtro si fida MENO della misura)
    // R_DYNAMIC_VERY_FAST_MOTION: Valore di R quando il movimento è molto veloce/brusco (più reattività, filtro si fida PIÙ della misura)
    // Questi valori sono un punto di partenza e DEVONO ESSERE SINTONIZZATI per il tuo caso d'uso!
    static constexpr float R_DYNAMIC_SLOW_MOTION      = OpenFIRE_Square::R_BASE * 5.0f;  // 5.0 Esempio: 5 volte R_BASE
    static constexpr float R_DYNAMIC_VERY_FAST_MOTION = OpenFIRE_Square::R_BASE * 0.1f;  // 0.1 Esempio: 10% di R_BASE
    // Puoi aggiungere un static_assert per un controllo di compile-time se lo desideri:
    // static_assert(R_DYNAMIC_VERY_FAST_MOTION >= MIN_COVARIANCE_AND_R, "R_DYNAMIC_VERY_FAST_MOTION must be >= MIN_COVARIANCE_AND_R");

    // Nuove costanti per moltiplicatori di R_BASE
    static constexpr float R_MULT_ESTIMATED_GEOMETRIC = 20.0f; // VALORE DA AFFINARE! (usato implicitamente prima)
    static constexpr float R_MULT_LAST_KNOWN_GOOD = 7.0f;     // VALORE DA AFFINARE! (usato come 7.0f nel cpp)

    static constexpr float DEAD_ZONE_X = KF_WIDTH * 0.002f;    // VALORE DA AFFINARE!
    static constexpr float DEAD_ZONE_Y = KF_HEIGHT * 0.002f; // VALORE DA AFFINARE!
    static constexpr float DEAD_ZONE_MULTIPLIER_EDGE = 1.5f;   // VALORE DA AFFINARE!
    static constexpr float DEAD_ZONE_MULTIPLIER_CENTER = 1.2f; // VALORE DA AFFINARE!
    static constexpr float STATIONARY_THRESHOLD_EDGE_COUNT = 75.0f; // VALORE DA AFFINARE!
    static constexpr float STATIONARY_THRESHOLD_CENTER_COUNT = 50.0f; // VALORE DA AFFINARE!

    static constexpr float EDGE_SMOOTHING = 1.5f;             // VALORE DA AFFINARE!
    static constexpr float MIN_PRECISION_FACTOR = 0.90f;      // VALORE DA AFFINARE!
    static constexpr float CORNER_SMOOTHING_EXPONENT_X = 2.5f;  // VALORE DA AFFINARE!
    static constexpr float CORNER_SMOOTHING_EXPONENT_Y = 3.0f;  // VALORE DA AFFINARE!
    static constexpr float EDGE_TRANSITION_THRESHOLD = 0.22f; // VALORE DA AFFINARE!
    static constexpr float EXP_SMOOTHNESS_FACTOR = 2.5f;      // VALORE DA AFFINARE!

    static constexpr float ACCEL_NORMALIZATION_FACTOR = KF_WIDTH * 0.01f; // VALORE DA AFFINARE!

    static constexpr float ACCEL_Q_THRESH_X_BASE = KF_WIDTH * 0.002f;
    static constexpr float JERK_Q_THRESH_X_BASE  = KF_WIDTH * 0.001f;
    static constexpr float ACCEL_Q_THRESH_Y_BASE = KF_HEIGHT * 0.002f;
    static constexpr float JERK_Q_THRESH_Y_BASE  = KF_HEIGHT * 0.001f;

    static constexpr float ACCEL_Q_THRESH_X_LOWER = ACCEL_Q_THRESH_X_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_X_UPPER = ACCEL_Q_THRESH_X_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_X_LOWER  = JERK_Q_THRESH_X_BASE * 0.8f;
    static constexpr float JERK_Q_THRESH_X_UPPER  = JERK_Q_THRESH_X_BASE * 1.2f;
    static constexpr float ACCEL_Q_THRESH_Y_LOWER = ACCEL_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float ACCEL_Q_THRESH_Y_UPPER = ACCEL_Q_THRESH_Y_BASE * 1.2f;
    static constexpr float JERK_Q_THRESH_Y_LOWER  = JERK_Q_THRESH_Y_BASE * 0.8f;
    static constexpr float JERK_Q_THRESH_Y_UPPER  = JERK_Q_THRESH_Y_BASE * 1.2f;

    static constexpr float SPEED_LOW_THRESHOLD = 1.5f;          // VALORE DA AFFINARE!
    static constexpr float SPEED_HIGH_THRESHOLD = KF_WIDTH * 0.03f; // VALORE DA AFFINARE!
    static constexpr float DAMPING_AT_LOW_SPEED = 0.65f;        // VALORE DA AFFINARE!
    static constexpr float DAMPING_AT_HIGH_SPEED = 0.98f;      // VALORE DA AFFINARE!

    static constexpr float X_EDGE_PROXIMITY_DECAY_RATE = 20.0f; // VALORE CHIAVE DA AFFINARE!
    static constexpr float MIN_PRECISION_FACTOR_Y_AT_X_EDGE = 0.95f;    // VALORE DA AFFINARE!
    static constexpr float DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE = 0.80f;    // VALORE DA AFFINARE!
    static constexpr float DEAD_ZONE_Y_SCALE_AT_X_EDGE = 0.6f;          // VALORE DA AFFINARE!

    static constexpr float fPI_KF = M_PI;

private:
    // ---- VARIABILI DI STATO DEL FILTRO DI KALMAN (static inline) ----
    static constexpr int INITIAL_FINAL_X[4] = {
        400 * CamToMouseMult, 623 * CamToMouseMult, 400 * CamToMouseMult, 623 * CamToMouseMult
    };
    static constexpr int INITIAL_FINAL_Y[4] = {
        200 * CamToMouseMult, 200 * CamToMouseMult, 568 * CamToMouseMult, 568 * CamToMouseMult
    };

    static inline float q_min_base[4] = { Q_MAX * 0.01f, Q_MAX * 0.01f, Q_MAX * 0.01f, Q_MAX * 0.01f };
    static inline float p_x[4] = { KF_WIDTH * 0.05f, KF_WIDTH * 0.05f, KF_WIDTH * 0.05f, KF_WIDTH * 0.05f };
    static inline float p_y[4] = { KF_HEIGHT * 0.05f, KF_HEIGHT * 0.05f, KF_HEIGHT * 0.05f, KF_HEIGHT * 0.05f };

    static inline float x_filt[4] = {
        static_cast<float>(INITIAL_FINAL_X[0]), static_cast<float>(INITIAL_FINAL_X[1]),
        static_cast<float>(INITIAL_FINAL_X[2]), static_cast<float>(INITIAL_FINAL_X[3])
    };
    static inline float y_filt[4] = {
        static_cast<float>(INITIAL_FINAL_Y[0]), static_cast<float>(INITIAL_FINAL_Y[1]),
        static_cast<float>(INITIAL_FINAL_Y[2]), static_cast<float>(INITIAL_FINAL_Y[3])
    };

    static inline float k_x[4] = {0.0f}; static inline float k_y[4] = {0.0f};
    static inline float last_x[4] = { x_filt[0], x_filt[1], x_filt[2], x_filt[3] };
    static inline float last_y[4] = { y_filt[0], y_filt[1], y_filt[2], y_filt[3] };
    static inline float last_vx[4] = {0.0f}; static inline float last_vy[4] = {0.0f};
    static inline float last_ax[4] = {0.0f}; static inline float last_ay[4] = {0.0f};
    static inline float last_jerk_x[4] = {0.0f}; static inline float last_jerk_y[4] = {0.0f};
    static inline float r_dynamic[4] = { R_BASE, R_BASE, R_BASE, R_BASE }; // Questo verrà aggiornato dinamicamente nel Kalman_filter()
    static inline int stationary_counter[4] = {0};
    static inline float last_known_good_raw_x[4] = { x_filt[0], x_filt[1], x_filt[2], x_filt[3] };
    static inline float last_known_good_raw_y[4] = { y_filt[0], y_filt[1], y_filt[2], y_filt[3] };

    // ---- Membri Non Statici (stato dell'istanza) ----
    int positionXX[4];       // Coordinate X (int) dei blob visti
    int positionYY[4];       // Coordinate Y (int) dei blob visti
    int positionX[4];
    int positionY[4];
    int last_FinalX[4], last_FinalY[4];  // FinalX/Y del frame precedente - int se FinalX/Y sono int
    //unsigned char see[4];               // Stato: 0=non visto, 1=visto diretto, 2=stimato
    bool initial_positions_established = false; // true se last_FinalX/Y sono validi
    float sensor_aspect_ratio = 1.0f; // Memorizza width / height
    bool aspect_ratio_calculated = false; // Flag per sapere se è stato calcolato
    
    static inline unsigned long testLastStampAux = 0;

    unsigned int start = 0;
    unsigned int seenFlags = 0;

    int FinalX[4] = {400 * CamToMouseMult, 623 * CamToMouseMult, 400 * CamToMouseMult, 623 * CamToMouseMult};
    int FinalY[4] = {200 * CamToMouseMult, 200 * CamToMouseMult, 568 * CamToMouseMult, 568 * CamToMouseMult};
    unsigned int see[4] = {0};
    int medianY = MouseMaxY / 2;
    int medianX = MouseMaxX / 2;
    float xDistTop, xDistBottom, yDistLeft, yDistRight;
    float angleTop, angleBottom, angleLeft, angleRight;
    float angle, height, width;
    float angleOffset[4];
    bool initialization_complete_flag_;
    unsigned int seenFlags_cam;
    TrackingQuality current_tracking_state_;
    float current_input_for_kf_x_[4];
    float current_input_for_kf_y_[4];

    // ---- Struttura Helper per Identificazione ----
    struct CameraPoint {
        float x, y;
        int original_cam_index;
        bool assigned_to_logical_point;
    };

    // ---- Funzioni Helper Private Static Inline ----
    template<typename T>
    static inline T constrain_custom(T val, T min_val, T max_val) {
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    }
    static inline float map_float_custom(float x, float in_min, float in_max, float out_min, float out_max) {
        if (fabsf(in_max - in_min) < 1e-6f) return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    static inline float lerp_custom(float a, float b, float t) { return a + t * (b - a); }
    static inline float smoothstep_custom(float edge0, float edge1, float x) {
        if (edge0 == edge1) return (x >= edge0) ? 1.0f : 0.0f;
        float t = constrain_custom((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    // ---- Metodi Privati per la Logica Interna ----
    void Kalman_filter();
    // Helper per stimare 2 punti mancanti quando 2 sono noti
    bool try_estimate_from_two_known_points(
        const bool known_logical_points_mask[4],   // Input: maschera di quali punti logici (0-3) sono noti
        const float current_known_x[4],             // Input: coordinate X dei punti noti (gli altri sono TBD)
        const float current_known_y[4],             // Input: coordinate Y dei punti noti
        float out_estimated_or_known_x[4],          // Output: tutte e 4 le X (2 note, 2 stimate se successo)
        float out_estimated_or_known_y[4]          // Output: tutte e 4 le Y
    );
    bool try_estimate_one_missing_point(
        const bool known_logical_points_mask[4], // Input: maschera di quali 3 punti logici sono noti
        const float current_known_x[4],         // Input: coordinate X dei 3 punti noti
        const float current_known_y[4],         // Input: coordinate Y dei 3 punti noti
        int missing_logical_idx,                // Input: indice del punto da stimare (0-3)
        float& out_estimated_x,                 // Output: X stimata
        float& out_estimated_y                 // Output: Y stimata
    );

    // === Funzione Helper Locale per Distanza al Quadrato ===
    static inline float helper_calculate_distance_sq(float x1, float y1, float x2, float y2) {
        float dx = x1 - x2;
        float dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    void identify_points_and_prepare_kf_inputs(
        const float cam_points_x_input_scaled_processed[],
        const float cam_points_y_input_scaled_processed[],
        unsigned int current_cam_seen_flags,
        int out_kf_input_x[],
        int out_kf_input_y[],
        bool out_logical_point_seen[],       // Output: true se il punto logico ha un input valido per KF (visto o stimato)
        bool out_logical_point_is_estimated[] // NUOVO Output: true se il punto è stato stimato geometricamente
    );
    void perform_kalman_reset_and_initial_identification(
        const float scaled_cam_x[],
        const float scaled_cam_y[],
        unsigned int seen_flags_from_cam,
        bool is_initial_boot
    );

public:
    // ---- COSTRUTTORE E METODI PUBBLICI ----
    OpenFIRE_Square();
    void begin(const int* px, const int* py, unsigned int seen);
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
};
#endif //COMMENTO


#endif // _OpenFIRE_Square_Advanced_h_
#endif //USE_SQUARE_ADVANCED