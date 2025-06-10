#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_Square.cpp.cpp
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2021
 */

#include <Arduino.h>
#include "OpenFIRE_Square_Advanced.h"

constexpr int buff = 50 * CamToMouseMult;

// floating point PI
constexpr float fPI = (float)PI;


// È possibile che la funzione 'map' non sia definita in questo contesto.
// Se stai usando una funzione 'map' simile a quella di Arduino,
// dovrai assicurarti che sia inclusa o definita.
// Esempio di definizione base se non è altrove:
/*
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

// === Implementazione del Costruttore ===
OpenFIRE_Square::OpenFIRE_Square() :
    medianY(MouseMaxY / 2), // Usa MouseMaxY globale
    medianX(MouseMaxX / 2), // Usa MouseMaxX globale
    xDistTop(0.0f), xDistBottom(0.0f), yDistLeft(0.0f), yDistRight(0.0f),
    angleTop(0.0f), angleBottom(0.0f), angleLeft(0.0f), angleRight(0.0f),
    angle(0.0f), height(0.0f), width(0.0f),
    initialization_complete_flag_(false),
    seenFlags_cam(0),
    current_tracking_state_(TrackingQuality::AWAITING_INITIALIZATION)
{
    for (int i = 0; i < 4; ++i) {
        angleOffset[i] = 0.0f;
        see[i] = 0; 
        // FinalX/Y ora usano x_filt/y_filt che sono inizializzati nel .h
        //FinalX[i] = static_cast<int>(roundf(OpenFIRE_Square::x_filt[i]));
        //FinalY[i] = static_cast<int>(roundf(OpenFIRE_Square::y_filt[i]));
        current_input_for_kf_x_[i] = static_cast<float>(FinalX[i]);
        current_input_for_kf_y_[i] = static_cast<float>(FinalY[i]);
    }
}

void OpenFIRE_Square::Kalman_filter() {
    for (int point_idx = 0; point_idx < 4; ++point_idx) { // Ciclo interno sui 4 punti

        float potential_x = this->current_input_for_kf_x_[point_idx];
        float potential_y = this->current_input_for_kf_y_[point_idx];

        float current_raw_x_for_kf;
        float current_raw_y_for_kf;

        current_raw_x_for_kf = potential_x;
        current_raw_y_for_kf = potential_y;

        if (potential_x >= OpenFIRE_Square::KF_X_MIN && potential_x <= OpenFIRE_Square::KF_X_MAX &&
            potential_y >= OpenFIRE_Square::KF_Y_MIN && potential_y <= OpenFIRE_Square::KF_Y_MAX) {
            OpenFIRE_Square::last_known_good_raw_x[point_idx] = potential_x;
            OpenFIRE_Square::last_known_good_raw_y[point_idx] = potential_y;
        }

        //
        // INIZIO DELLA TUA LOGICA DETTAGLIATA DEL FILTRO DI KALMAN
        // (Usa current_raw_x_for_kf e current_raw_y_for_kf come input)
        //
        float vx = current_raw_x_for_kf - OpenFIRE_Square::last_x[point_idx];
        float vy = current_raw_y_for_kf - OpenFIRE_Square::last_y[point_idx];
        float ax = vx - OpenFIRE_Square::last_vx[point_idx];
        float ay = vy - OpenFIRE_Square::last_vy[point_idx];
        float jerk_x = ax - OpenFIRE_Square::last_ax[point_idx];
        float jerk_y = ay - OpenFIRE_Square::last_ay[point_idx];

        float dist_from_center_x_norm = fabsf(current_raw_x_for_kf - OpenFIRE_Square::KF_X_CENTER) / OpenFIRE_Square::KF_HALF_WIDTH;
        float dist_from_center_y_norm = fabsf(current_raw_y_for_kf - OpenFIRE_Square::KF_Y_CENTER) / OpenFIRE_Square::KF_HALF_HEIGHT;
        float dist_to_edge_x_norm = std::min(fabsf(current_raw_x_for_kf - OpenFIRE_Square::KF_X_MIN), fabsf(current_raw_x_for_kf - OpenFIRE_Square::KF_X_MAX)) / OpenFIRE_Square::KF_HALF_WIDTH;
        float dist_to_edge_y_norm = std::min(fabsf(current_raw_y_for_kf - OpenFIRE_Square::KF_Y_MIN), fabsf(current_raw_y_for_kf - OpenFIRE_Square::KF_Y_MAX)) / OpenFIRE_Square::KF_HALF_HEIGHT;

        float x_edge_proximity_factor = expf(-OpenFIRE_Square::X_EDGE_PROXIMITY_DECAY_RATE * dist_to_edge_x_norm);

        float speed_abs_x = fabsf(vx);
        float speed_abs_y = fabsf(vy);
        float effective_damping_low_y = OpenFIRE_Square::lerp_custom(OpenFIRE_Square::DAMPING_AT_LOW_SPEED, OpenFIRE_Square::DAMPING_AT_LOW_SPEED_Y_AT_X_EDGE, x_edge_proximity_factor);
        float damping_factor_vx = OpenFIRE_Square::lerp_custom(OpenFIRE_Square::DAMPING_AT_LOW_SPEED, OpenFIRE_Square::DAMPING_AT_HIGH_SPEED, OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::SPEED_LOW_THRESHOLD, OpenFIRE_Square::SPEED_HIGH_THRESHOLD, speed_abs_x));
        float damping_factor_vy = OpenFIRE_Square::lerp_custom(effective_damping_low_y, OpenFIRE_Square::DAMPING_AT_HIGH_SPEED, OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::SPEED_LOW_THRESHOLD, OpenFIRE_Square::SPEED_HIGH_THRESHOLD, speed_abs_y));
        vx *= damping_factor_vx;
        vy *= damping_factor_vy;

        float stabilization_center_x = 1.0f - powf(dist_from_center_x_norm, OpenFIRE_Square::EDGE_SMOOTHING);
        float stabilization_center_y = 1.0f - powf(dist_from_center_y_norm, OpenFIRE_Square::EDGE_SMOOTHING);
        float stabilization_corner_x = 1.0f - powf(1.0f - dist_to_edge_x_norm, OpenFIRE_Square::CORNER_SMOOTHING_EXPONENT_X);
        float stabilization_corner_y = 1.0f - powf(1.0f - dist_to_edge_y_norm, OpenFIRE_Square::CORNER_SMOOTHING_EXPONENT_Y);

        float normalized_dist_in_transition_eff_x = dist_to_edge_x_norm / OpenFIRE_Square::EDGE_TRANSITION_THRESHOLD;
        float factor_for_pow_x = OpenFIRE_Square::constrain_custom(1.0f - normalized_dist_in_transition_eff_x, 0.0f, 1.0f);
        float weight_for_corner_x = powf(factor_for_pow_x, OpenFIRE_Square::EXP_SMOOTHNESS_FACTOR);
        float combined_stabilization_factor_x = OpenFIRE_Square::lerp_custom(stabilization_center_x, stabilization_corner_x, weight_for_corner_x);
        combined_stabilization_factor_x = OpenFIRE_Square::constrain_custom(combined_stabilization_factor_x, OpenFIRE_Square::MIN_PRECISION_FACTOR, 1.0f);

        float effective_min_precision_y = OpenFIRE_Square::lerp_custom(OpenFIRE_Square::MIN_PRECISION_FACTOR, OpenFIRE_Square::MIN_PRECISION_FACTOR_Y_AT_X_EDGE, x_edge_proximity_factor);
        float normalized_dist_in_transition_eff_y = dist_to_edge_y_norm / OpenFIRE_Square::EDGE_TRANSITION_THRESHOLD;
        float factor_for_pow_y = OpenFIRE_Square::constrain_custom(1.0f - normalized_dist_in_transition_eff_y, 0.0f, 1.0f);
        float weight_for_corner_y = powf(factor_for_pow_y, OpenFIRE_Square::EXP_SMOOTHNESS_FACTOR);
        float combined_stabilization_factor_y = OpenFIRE_Square::lerp_custom(stabilization_center_y, stabilization_corner_y, weight_for_corner_y);
        combined_stabilization_factor_y = OpenFIRE_Square::constrain_custom(combined_stabilization_factor_y, effective_min_precision_y, 1.0f);

        vx *= combined_stabilization_factor_x;
        vy *= combined_stabilization_factor_y;
        ax *= combined_stabilization_factor_x;
        ay *= combined_stabilization_factor_y;

        float max_dist_to_edge_norm = std::max(dist_to_edge_x_norm, dist_to_edge_y_norm);
        float t_deadzone = OpenFIRE_Square::constrain_custom(1.0f - (max_dist_to_edge_norm / OpenFIRE_Square::EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
        float curved_t_deadzone = t_deadzone * t_deadzone;
        float current_dead_zone_multiplier = OpenFIRE_Square::map_float_custom(curved_t_deadzone, 0.0f, 1.0f, OpenFIRE_Square::DEAD_ZONE_MULTIPLIER_CENTER, OpenFIRE_Square::DEAD_ZONE_MULTIPLIER_EDGE);

        float t_stationary = OpenFIRE_Square::constrain_custom(1.0f - (max_dist_to_edge_norm / OpenFIRE_Square::EDGE_TRANSITION_THRESHOLD), 0.0f, 1.0f);
        float curved_t_stationary = t_stationary * t_stationary;
        float current_stationary_threshold_count = OpenFIRE_Square::map_float_custom(curved_t_stationary, 0.0f, 1.0f, OpenFIRE_Square::STATIONARY_THRESHOLD_CENTER_COUNT, OpenFIRE_Square::STATIONARY_THRESHOLD_EDGE_COUNT);

        float effective_dead_zone_y_scale = OpenFIRE_Square::lerp_custom(1.0f, OpenFIRE_Square::DEAD_ZONE_Y_SCALE_AT_X_EDGE, x_edge_proximity_factor);
        float actual_dead_zone_y = (OpenFIRE_Square::DEAD_ZONE_Y * current_dead_zone_multiplier) * effective_dead_zone_y_scale;
        float actual_dead_zone_x = OpenFIRE_Square::DEAD_ZONE_X * current_dead_zone_multiplier;

        if (fabsf(vx) < actual_dead_zone_x && fabsf(vy) < actual_dead_zone_y) {
            OpenFIRE_Square::stationary_counter[point_idx]++;
            if (OpenFIRE_Square::stationary_counter[point_idx] > static_cast<int>(current_stationary_threshold_count)) {
                OpenFIRE_Square::x_filt[point_idx] = OpenFIRE_Square::last_x[point_idx];
                OpenFIRE_Square::y_filt[point_idx] = OpenFIRE_Square::last_y[point_idx];
                continue; // Salta il resto del filtro per questo point_idx
            }
        } else {
            OpenFIRE_Square::stationary_counter[point_idx] = 0;
            OpenFIRE_Square::q_min_base[point_idx] = OpenFIRE_Square::Q_MAX * 0.01f;
        }

        //float measured_x = (current_raw_x_for_kf + OpenFIRE_Square::last_x[point_idx] * 2.0f + OpenFIRE_Square::x_filt[point_idx] * 3.0f) / 6.0f;
        //float measured_y = (current_raw_y_for_kf + OpenFIRE_Square::last_y[point_idx] * 2.0f + OpenFIRE_Square::y_filt[point_idx] * 3.0f) / 6.0f;

        float measured_x = current_raw_x_for_kf; // La misura è l'input diretto (grezzo o stimato geometricamente)
        float measured_y = current_raw_y_for_kf; // La misura è l'input diretto (grezzo o stimato geometricamente)

        float prediction_weight_v = OpenFIRE_Square::constrain_custom(fabsf(vx) / OpenFIRE_Square::KF_X_MAX, 0.1f, 0.8f);
        float prediction_weight_a = OpenFIRE_Square::constrain_custom(fabsf(ax) / OpenFIRE_Square::ACCEL_NORMALIZATION_FACTOR, 0.2f, 0.9f);

        float predicted_x = OpenFIRE_Square::x_filt[point_idx] + vx * prediction_weight_v + ax * prediction_weight_a;
        float predicted_y = OpenFIRE_Square::y_filt[point_idx] + vy * prediction_weight_v + ay * prediction_weight_a;

        float accel_factor_q_x = OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::ACCEL_Q_THRESH_X_LOWER, OpenFIRE_Square::ACCEL_Q_THRESH_X_UPPER, fabsf(ax));
        float jerk_factor_q_x  = OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::JERK_Q_THRESH_X_LOWER,  OpenFIRE_Square::JERK_Q_THRESH_X_UPPER,  fabsf(jerk_x));
        float combined_factor_q_x = std::max(accel_factor_q_x, jerk_factor_q_x);
        float q_x_val = OpenFIRE_Square::lerp_custom(OpenFIRE_Square::q_min_base[point_idx], OpenFIRE_Square::Q_MAX, combined_factor_q_x);

        float accel_factor_q_y = OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::ACCEL_Q_THRESH_Y_LOWER, OpenFIRE_Square::ACCEL_Q_THRESH_Y_UPPER, fabsf(ay));
        float jerk_factor_q_y  = OpenFIRE_Square::smoothstep_custom(OpenFIRE_Square::JERK_Q_THRESH_Y_LOWER,  OpenFIRE_Square::JERK_Q_THRESH_Y_UPPER,  fabsf(jerk_y));
        float combined_factor_q_y = std::max(accel_factor_q_y, jerk_factor_q_y);
        float q_y_val = OpenFIRE_Square::lerp_custom(OpenFIRE_Square::q_min_base[point_idx], OpenFIRE_Square::Q_MAX, combined_factor_q_y);

        OpenFIRE_Square::p_x[point_idx] += q_x_val;
        OpenFIRE_Square::p_y[point_idx] += q_y_val;

        // --- INIZIO: NUOVA LOGICA PER R_DYNAMIC DINAMICO ---
        // Determina il fattore di movimento più significativo tra X e Y per questo punto.
        // Questo fattore va da 0.0 (movimento lento/stabile) a 1.0 (movimento molto veloce/brusco).
        float max_overall_motion_factor = std::max(combined_factor_q_x, combined_factor_q_y);

        // Interpola r_dynamic: quando il movimento è lento (fattore 0), usa R_DYNAMIC_SLOW_MOTION (più smoothing).
        // Quando il movimento è veloce (fattore 1), usa R_DYNAMIC_VERY_FAST_MOTION (più reattività).
        OpenFIRE_Square::r_dynamic[point_idx] = OpenFIRE_Square::lerp_custom(
            OpenFIRE_Square::R_DYNAMIC_SLOW_MOTION,
            OpenFIRE_Square::R_DYNAMIC_VERY_FAST_MOTION,
            max_overall_motion_factor
        );

        // Assicurati che r_dynamic non scenda mai sotto il valore minimo consentito per stabilità numerica.
        if (OpenFIRE_Square::r_dynamic[point_idx] < OpenFIRE_Square::MIN_COVARIANCE_AND_R) {
            OpenFIRE_Square::r_dynamic[point_idx] = OpenFIRE_Square::MIN_COVARIANCE_AND_R;
        }
        // --- FINE: NUOVA LOGICA PER R_DYNAMIC DINAMICO ---


        OpenFIRE_Square::k_x[point_idx] = OpenFIRE_Square::p_x[point_idx] / (OpenFIRE_Square::p_x[point_idx] + OpenFIRE_Square::r_dynamic[point_idx]);
        OpenFIRE_Square::k_y[point_idx] = OpenFIRE_Square::p_y[point_idx] / (OpenFIRE_Square::p_y[point_idx] + OpenFIRE_Square::r_dynamic[point_idx]);

        OpenFIRE_Square::x_filt[point_idx] = predicted_x + OpenFIRE_Square::k_x[point_idx] * (measured_x - predicted_x);
        OpenFIRE_Square::y_filt[point_idx] = predicted_y + OpenFIRE_Square::k_y[point_idx] * (measured_y - predicted_y);

        OpenFIRE_Square::p_x[point_idx] *= (1.0f - OpenFIRE_Square::k_x[point_idx]);
        OpenFIRE_Square::p_y[point_idx] *= (1.0f - OpenFIRE_Square::k_y[point_idx]);

        OpenFIRE_Square::last_x[point_idx] = OpenFIRE_Square::x_filt[point_idx];
        OpenFIRE_Square::last_y[point_idx] = OpenFIRE_Square::y_filt[point_idx];
        OpenFIRE_Square::last_vx[point_idx] = vx;
        OpenFIRE_Square::last_vy[point_idx] = vy;
        OpenFIRE_Square::last_ax[point_idx] = ax;
        OpenFIRE_Square::last_ay[point_idx] = ay;
        OpenFIRE_Square::last_jerk_x[point_idx] = jerk_x;
        OpenFIRE_Square::last_jerk_y[point_idx] = jerk_y;
        //
        // Fine della TUA logica originale del filtro di Kalman per point_idx
        //
    } // Fine del ciclo for point_idx
}


////////////////////////////////////////////////////////////////////////////////////
/////////// roba mia funzionante finoa a 3 punti ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// Definizione degli indici per chiarezza
#define A 0
#define B 1
#define C 2
#define D 3
#include <climits>

void OpenFIRE_Square::begin(const int* px, const int* py, unsigned int seen) {  

    seenFlags = seen;

    // Verifica che tutti i sensori siano visibili
    if (seenFlags == 0x0F) {
        start = 0xFF;
    } else if (!start) {
        return;  // Esci se non tutti i sensori sono stati visti
    }


    //if (seenFlags == 0x0F) {


     int num_points_seen = 0;

      // Estrai i punti visti e applica lo shift
    for (int i = 0; i < 4; ++i) {
        if ((seen >> i) & 0x01) { // Usa il parametro 'seen' qui
            positionXX[num_points_seen] = (CamMaxX - px[i]) << CamToMouseShift;
            positionYY[num_points_seen] = py[i] << CamToMouseShift;
            num_points_seen++;
        }
    }

    if (num_points_seen > 1) {
    
    if (num_points_seen == 2) { // SOLO 2 PUNTI VISIBILI, BISOGNA STIMARNE 2
// --- INIZIO BLOCCO FINALE GESTIONE 2 SENSORI (IBRIDO E OTTIMIZZATO) ---
// OBIETTIVO:
// 1. Capire se i due punti visti sono adiacenti (un lato) o opposti (una diagonale).
// 2. Se sono una diagonale, usare una ricostruzione geometrica "stateless" per resettare l'errore.
// 3. Se sono un lato, usare una logica di tracking "temporale" ottimizzata per stimare i punti mancanti.

// --- FASE 1: Discriminatore a 3 Vie (Larghezza vs Altezza vs Diagonale) ---

// 1a. Calcola la distanza al quadrato del segmento che vediamo ora.
int32_t dx = positionXX[0] - positionXX[1];
int32_t dy = positionYY[0] - positionYY[1];
int32_t current_dist_sq = dx * dx + dy * dy;

// 1b. Calcola le dimensioni di riferimento dall'ultimo frame valido (FinalX/Y).
dx = FinalX[A] - FinalX[B]; dy = FinalY[A] - FinalY[B];
int32_t last_avg_width_sq = dx * dx + dy * dy;

dx = FinalX[A] - FinalX[C]; dy = FinalY[A] - FinalY[C];
int32_t last_avg_height_sq = dx * dx + dy * dy;

// Calcolo la diagonale al quadrato con Pitagora.
int32_t last_diagonal_sq = last_avg_width_sq + last_avg_height_sq;

// 1c. Confronta la distanza attuale con i 3 riferimenti.
int32_t diff_as_width = abs(current_dist_sq - last_avg_width_sq);
int32_t diff_as_height = abs(current_dist_sq - last_avg_height_sq);
int32_t diff_as_diag = abs(current_dist_sq - last_diagonal_sq);


// --- FASE 2: Esecuzione della Logica Corretta in base al caso ---

if (diff_as_diag < diff_as_width && diff_as_diag < diff_as_height)
{
   // === CASO 1: I PUNTI VISTI SONO UNA DIAGONALE (LOGICA ADATTIVA E ROBUSTA) ===
    // Questa logica "impara" la forma del rettangolo dall'ultimo frame valido
    // e applica la trasformazione attuale (scala+rotazione) per stimare i punti mancanti.

    // --- Sotto-Fase A: Identificazione della diagonale e dei punti visti ---
    int P1, P2; // Indici dei vertici della diagonale (es. A e C, oppure B e D)
    int P3_adj; // Indice di un punto adiacente al primo punto della diagonale
    
    // Identifichiamo QUALE diagonale stiamo vedendo (AC o BD) tramite prossimità del punto medio
    int32_t mid_now_x = (positionXX[0] + positionXX[1]) / 2;
    int32_t mid_now_y = (positionYY[0] + positionYY[1]) / 2;
    int32_t mid_AC_x = (FinalX[A] + FinalX[C]) / 2;
    int32_t mid_AC_y = (FinalY[A] + FinalY[C]) / 2;
    int32_t mid_BD_x = (FinalX[B] + FinalX[D]) / 2;
    int32_t mid_BD_y = (FinalY[B] + FinalY[D]) / 2;
    int32_t dist_to_AC_sq = (mid_now_x - mid_AC_x)*(mid_now_x - mid_AC_x) + (mid_now_y - mid_AC_y)*(mid_now_y - mid_AC_y);
    int32_t dist_to_BD_sq = (mid_now_x - mid_BD_x)*(mid_now_x - mid_BD_x) + (mid_now_y - mid_BD_y)*(mid_now_y - mid_BD_y);

    if (dist_to_AC_sq < dist_to_BD_sq) {
        // Stiamo vedendo la diagonale A-C
        P1 = A; P2 = C; P3_adj = B; // Un vicino di A è B
    } else {
        // Stiamo vedendo la diagonale B-D
        P1 = B; P2 = D; P3_adj = A; // Un vicino di B è A
    }

    // Ora associamo i punti visti (0 e 1) ai loro indici corretti (es. A e C)
    int32_t new_p1x, new_p1y, new_p2x, new_p2y;
    int32_t dist_0_p1 = abs(positionXX[0] - FinalX[P1]);
    int32_t dist_0_p2 = abs(positionXX[0] - FinalX[P2]);
    if(dist_0_p1 < dist_0_p2) {
        new_p1x = positionXX[0]; new_p1y = positionYY[0]; // position[0] è P1
        new_p2x = positionXX[1]; new_p2y = positionYY[1]; // position[1] è P2
    } else {
        new_p1x = positionXX[1]; new_p1y = positionYY[1]; // position[1] è P1
        new_p2x = positionXX[0]; new_p2y = positionYY[0]; // position[0] è P2
    }

    // --- Sotto-Fase B: Stima tramite trasformazione vettoriale ---
    int32_t old_p1x = FinalX[P1];
    int32_t old_p1y = FinalY[P1];
    int32_t old_p2x = FinalX[P2];
    int32_t old_p2y = FinalY[P2];
    int32_t old_p3x = FinalX[P3_adj];
    int32_t old_p3y = FinalY[P3_adj];
    
    // Vettore della diagonale (vecchio e nuovo)
    int32_t vec_diag_orig_x = old_p2x - old_p1x;
    int32_t vec_diag_orig_y = old_p2y - old_p1y;
    int32_t vec_diag_new_x = new_p2x - new_p1x;
    int32_t vec_diag_new_y = new_p2y - new_p1y;

    // Vettore di un lato adiacente (vecchio)
    int32_t vec_adj_orig_x = old_p3x - old_p1x;
    int32_t vec_adj_orig_y = old_p3y - old_p1y;

    // Calcola la trasformazione (scala+rotazione) dalla vecchia alla nuova diagonale
    float len_sq_orig = (float)vec_diag_orig_x * vec_diag_orig_x + (float)vec_diag_orig_y * vec_diag_orig_y;
    float a = 1.0f, b = 0.0f;

    if (len_sq_orig > 1e-5f) {
        float dot_product = (float)vec_diag_new_x * vec_diag_orig_x + (float)vec_diag_new_y * vec_diag_orig_y;
        float cross_product = (float)vec_diag_new_y * vec_diag_orig_x - (float)vec_diag_new_x * vec_diag_orig_y;
        a = dot_product / len_sq_orig;
        b = cross_product / len_sq_orig;
    }

    // Applica la stessa trasformazione al vettore del lato adiacente
    float vec_adj_new_x = a * (float)vec_adj_orig_x - b * (float)vec_adj_orig_y;
    float vec_adj_new_y = b * (float)vec_adj_orig_x + a * (float)vec_adj_orig_y;
    
    // Calcola le posizioni dei due punti mancanti
    int32_t estimated_p3x = (int32_t)(new_p1x + vec_adj_new_x + 0.5f);
    int32_t estimated_p3y = (int32_t)(new_p1y + vec_adj_new_y + 0.5f);

    // L'ultimo punto si trova con la regola del parallelogramma (D = A + C - B)
    // Se P1=A, P2=C, P3_adj=B, allora il punto mancante D = A+C-B
    int P4_idx = A + C + D - (P1 + P2 + P3_adj); // Trova l'indice del 4o punto
    int32_t estimated_p4x = new_p1x + new_p2x - estimated_p3x;
    int32_t estimated_p4y = new_p1y + new_p2y - estimated_p3y;

    // Aggiorna i 4 punti nell'array positionXX/YY
    positionXX[P1] = new_p1x; positionYY[P1] = new_p1y;
    positionXX[P2] = new_p2x; positionYY[P2] = new_p2y;
    positionXX[P3_adj] = estimated_p3x; positionYY[P3_adj] = estimated_p3y;
    positionXX[P4_idx] = estimated_p4x; positionYY[P4_idx] = estimated_p4y;
}
else 
{
    // === CASO 2: I PUNTI VISTI SONO ADIACENTI (UN LATO) ===
    // Si esegue la logica di tracking temporale, ora con la stima ottimizzata.
    
    int P1, P2; // Indici target (A,B,C,D) per i due punti visti
    const int32_t P1x = positionXX[0]; 
    const int32_t P1y = positionYY[0];
    const int32_t P2x = positionXX[1]; 
    const int32_t P2y = positionYY[1];

    // --- Sotto-Fase A: Identificazione del Lato Specifico ---
    
    // Ottimizzazione: calcolo il punto medio una sola volta qui.
    const int32_t mid_now_x = (P1x + P2x) / 2;
    const int32_t mid_now_y = (P1y + P2y) / 2;

    if (diff_as_width < diff_as_height) {
        // È più probabile che sia un lato "larghezza" (orizzontale)
        bool is_AB_by_centroid = ((positionYY[0] - medianY) + (positionYY[1] - medianY)) < 0;
        int32_t mid_AB_x = (FinalX[A] + FinalX[B]) / 2;
        int32_t mid_AB_y = (FinalY[A] + FinalY[B]) / 2;
        int32_t mid_CD_x = (FinalX[C] + FinalX[D]) / 2;
        int32_t mid_CD_y = (FinalY[C] + FinalY[D]) / 2;
        int32_t dist_to_AB_sq = (mid_now_x - mid_AB_x)*(mid_now_x - mid_AB_x) + (mid_now_y - mid_AB_y)*(mid_now_y - mid_AB_y);
        int32_t dist_to_CD_sq = (mid_now_x - mid_CD_x)*(mid_now_x - mid_CD_x) + (mid_now_y - mid_CD_y)*(mid_now_y - mid_CD_y);
        bool is_AB_by_proximity = (dist_to_AB_sq < dist_to_CD_sq);

        if (is_AB_by_centroid != is_AB_by_proximity) return; 

        if (is_AB_by_centroid) {
            P1 = A; P2 = B;
            if (P1x < P2x) { positionXX[P1] = P1x; positionYY[P1] = P1y; positionXX[P2] = P2x; positionYY[P2] = P2y; } 
            else { positionXX[P1] = P2x; positionYY[P1] = P2y; positionXX[P2] = P1x; positionYY[P2] = P1y; }
        } else {
            P1 = C; P2 = D;
            if (P1x < P2x) { positionXX[P1] = P1x; positionYY[P1] = P1y; positionXX[P2] = P2x; positionYY[P2] = P2y; } 
            else { positionXX[P1] = P2x; positionYY[P1] = P2y; positionXX[P2] = P1x; positionYY[P2] = P1y; }
        }
    } else {
        // È più probabile che sia un lato "altezza" (verticale)
        bool is_AC_by_centroid = ((positionXX[0] - medianX) + (positionXX[1] - medianX)) < 0;
        int32_t mid_AC_x = (FinalX[A] + FinalX[C]) / 2;
        int32_t mid_AC_y = (FinalY[A] + FinalY[C]) / 2;
        int32_t mid_BD_x = (FinalX[B] + FinalX[D]) / 2;
        int32_t mid_BD_y = (FinalY[B] + FinalY[D]) / 2;
        int32_t dist_to_AC_sq = (mid_now_x - mid_AC_x)*(mid_now_x - mid_AC_x) + (mid_now_y - mid_AC_y)*(mid_now_y - mid_AC_y);
        int32_t dist_to_BD_sq = (mid_now_x - mid_BD_x)*(mid_now_x - mid_BD_x) + (mid_now_y - mid_BD_y)*(mid_now_y - mid_BD_y);
        bool is_AC_by_proximity = (dist_to_AC_sq < dist_to_BD_sq);

        if (is_AC_by_centroid != is_AC_by_proximity) return; 

        if (is_AC_by_centroid) {
            P1 = A; P2 = C;
            if (P1y < P2y) { positionXX[P1] = P1x; positionYY[P1] = P1y; positionXX[P2] = P2x; positionYY[P2] = P2y; } 
            else { positionXX[P1] = P2x; positionYY[P1] = P2y; positionXX[P2] = P1x; positionYY[P2] = P1y; }
        } else {
            P1 = B; P2 = D;
            if (P1y < P2y) { positionXX[P1] = P1x; positionYY[P1] = P1y; positionXX[P2] = P2x; positionYY[P2] = P2y; }
            else { positionXX[P1] = P2x; positionYY[P1] = P2y; positionXX[P2] = P1x; positionYY[P2] = P1y; }
        }
    }
    
    // --- Sotto-Fase B: Stima Temporale Ottimizzata (Senza Trigonometria) ---
    
    int32_t old_p1x = FinalX[P1];
    int32_t old_p1y = FinalY[P1];
    int32_t old_p2x = FinalX[P2];
    int32_t old_p2y = FinalY[P2];

    int32_t new_p1x = positionXX[P1];
    int32_t new_p1y = positionYY[P1];
    int32_t new_p2x = positionXX[P2];
    int32_t new_p2y = positionYY[P2];

    int P3_idx = -1;
    if (P1 == A)      { P3_idx = (P2 == B) ? C : B; }
    else if (P1 == B) { P3_idx = (P2 == A) ? D : A; }
    else if (P1 == C) { P3_idx = (P2 == A) ? D : A; }
    else if (P1 == D) { P3_idx = (P2 == B) ? C : B; }

    int32_t old_p3x = FinalX[P3_idx];
    int32_t old_p3y = FinalY[P3_idx];

    int32_t vec_side_orig_x = old_p2x - old_p1x;
    int32_t vec_side_orig_y = old_p2y - old_p1y;
    int32_t vec_adj_orig_x = old_p3x - old_p1x;
    int32_t vec_adj_orig_y = old_p3y - old_p1y;

    int32_t vec_side_new_x = new_p2x - new_p1x;
    int32_t vec_side_new_y = new_p2y - new_p1y;
    
    float len_sq_orig = (float)vec_side_orig_x * vec_side_orig_x + (float)vec_side_orig_y * vec_side_orig_y;

    float a = 1.0f; // Componente a = (scala * cos)
    float b = 0.0f; // Componente b = (scala * sin)

    if (len_sq_orig > 1e-5f) {
        float dot_product = (float)vec_side_new_x * vec_side_orig_x + (float)vec_side_new_y * vec_side_orig_y;
        float cross_product = (float)vec_side_new_y * vec_side_orig_x - (float)vec_side_new_x * vec_side_orig_y; // Versione corretta
        a = dot_product / len_sq_orig;
        b = cross_product / len_sq_orig;
    }

    float vec_adj_new_x = a * (float)vec_adj_orig_x - b * (float)vec_adj_orig_y;
    float vec_adj_new_y = b * (float)vec_adj_orig_x + a * (float)vec_adj_orig_y;

    int32_t estimated_p3x = (int32_t)(new_p1x + vec_adj_new_x + 0.5f);
    int32_t estimated_p3y = (int32_t)(new_p1y + vec_adj_new_y + 0.5f);

    int32_t estimated_p4x = (int32_t)(new_p2x + vec_adj_new_x + 0.5f);
    int32_t estimated_p4y = (int32_t)(new_p2y + vec_adj_new_y + 0.5f);
    
    for (int i = 0; i < 4; ++i) {
        if (i != P1 && i != P2) {
            if (i == P3_idx) {
                positionXX[i] = estimated_p3x;
                positionYY[i] = estimated_p3y;
            } else {
                positionXX[i] = estimated_p4x;
                positionYY[i] = estimated_p4y;
            }
        }
    }
}




    } // FINE SOLO 2 PUNTI VISIBILI

    

    
    ////////// IN CASO DI SOLI 3 PUNTI ////// CALCOLA IL QUARTO ////////////////////////
    if (num_points_seen == 3)  // 3 PUNTII VISIBILI, BISOGNA STIMARNE 1
    {
// --- Inizio Blocco di Stima per 3 Sensori Visti ---
// OBIETTIVO: Stimare la posizione del quarto punto (salvato in positionXX[3])
//            quando solo tre punti (in positionXX[0], [1], [2]) sono visibili.
// METODO: Si assume che i 3 punti visibili formino un triangolo. Il vertice
//         opposto al lato più lungo di questo triangolo è il "perno" (B).
//         Gli altri due sono gli estremi (A, C). Si applica la regola del
//         parallelogramma: PuntoMancante = A + C - B.

// --- 1. Calcolo delle distanze al quadrato tra tutte le coppie di punti ---
//    Si usa l'aritmetica intera e le distanze al quadrato per massima velocità,
//    evitando l'uso di lenti calcoli in virgola mobile o radici quadrate.

// Distanza al quadrato tra il punto 0 e il punto 1
int32_t dx01 = positionXX[0] - positionXX[1];
int32_t dy01 = positionYY[0] - positionYY[1];
int32_t d01_sq = dx01 * dx01 + dy01 * dy01;

// Distanza al quadrato tra il punto 1 e il punto 2
int32_t dx12 = positionXX[1] - positionXX[2];
int32_t dy12 = positionYY[1] - positionYY[2];
int32_t d12_sq = dx12 * dx12 + dy12 * dy12;

// Distanza al quadrato tra il punto 0 e il punto 2
int32_t dx02 = positionXX[0] - positionXX[2];
int32_t dy02 = positionYY[0] - positionYY[2];
int32_t d02_sq = dx02 * dx02 + dy02 * dy02;


// --- 2. Identificazione del perno (B) e degli estremi (A, C) ---
//    Si determina quale dei tre lati è il più lungo per identificare
//    correttamente i ruoli di ogni punto nella formula finale.
int32_t a_idx, b_idx, c_idx;

// Caso 1: Il lato più lungo è quello tra il punto 0 e il punto 1.
if (d01_sq >= d12_sq && d01_sq >= d02_sq) {
    // I punti 0 e 1 sono gli estremi A e C.
    a_idx = 0;
    c_idx = 1;
    // Il punto rimanente (2) è il perno B.
    b_idx = 2;
}
// Caso 2: Il lato più lungo è quello tra il punto 1 e il punto 2.
else if (d12_sq >= d02_sq) {
    // I punti 1 e 2 sono gli estremi A e C.
    a_idx = 1;
    c_idx = 2;
    // Il punto rimanente (0) è il perno B.
    b_idx = 0;
}
// Caso 3: Il lato più lungo è quello tra il punto 0 e il punto 2.
else {
    // I punti 0 e 2 sono gli estremi A e C.
    a_idx = 0;
    c_idx = 2;
    // Il punto rimanente (1) è il perno B.
    b_idx = 1;
}

// --- 3. Stima del punto mancante e salvataggio del risultato ---
//    Applica la regola del parallelogramma (D = A + C - B) usando gli indici
//    determinati nella fase precedente per calcolare il quarto punto.
positionXX[3] = positionXX[a_idx] + positionXX[c_idx] - positionXX[b_idx];
positionYY[3] = positionYY[a_idx] + positionYY[c_idx] - positionYY[b_idx];

// --- Fine Blocco di Stima ---

    }

    ///////////// FINE 3 SENSORI VISTI ///////////////////////
    
    ///// DA QUI IN POI SI ESEGUE SEMPRE, UTILIZZA LE COORDINATE NON ORDINATE DI 4 SENSORI VISTI

    //// tutti 4 sensori visti
// --- INIZIO BLOCCO ORDINAMENTO E GESTIONE 4 PUNTI ---
// OBIETTIVO:
// 1. Ordinare gli indici dei 4 punti in base alle loro coordinate X e Y.
// 2. Applicare una serie di euristiche per assegnare correttamente gli indici
//    ai vertici A, B, C, D del rettangolo, anche in presenza di rotazione.
// 3. Copiare le coordinate ordinate nell'array di stato finale (FinalX/Y).

int a, b, c, d; // Indici finali per A, B, C, D (tipo 'int' come da originale)
int orderX[4] = {0, 1, 2, 3};
int orderY[4] = {0, 1, 2, 3};

// --- FASE 1: Ordinamento degli Indici ---
// Utilizza un "sorting network" hardcodato, un metodo molto veloce per ordinare
// un numero fisso e piccolo di elementi (in questo caso 4). È più performante
// di un algoritmo di ordinamento generico come il bubble sort.

// Ordinamento degli indici in base alla coordinata X
{
    int tmp;
    // La sequenza di 5 confronti garantisce l'ordinamento completo
    if (positionXX[orderX[0]] > positionXX[orderX[1]]) { tmp = orderX[0]; orderX[0] = orderX[1]; orderX[1] = tmp; }
    if (positionXX[orderX[2]] > positionXX[orderX[3]]) { tmp = orderX[2]; orderX[2] = orderX[3]; orderX[3] = tmp; }
    if (positionXX[orderX[0]] > positionXX[orderX[2]]) { tmp = orderX[0]; orderX[0] = orderX[2]; orderX[2] = tmp; }
    if (positionXX[orderX[1]] > positionXX[orderX[3]]) { tmp = orderX[1]; orderX[1] = orderX[3]; orderX[3] = tmp; }
    if (positionXX[orderX[1]] > positionXX[orderX[2]]) { tmp = orderX[1]; orderX[1] = orderX[2]; orderX[2] = tmp; }
}

// Ordinamento degli indici in base alla coordinata Y
{
    int tmp;
    if (positionYY[orderY[0]] > positionYY[orderY[1]]) { tmp = orderY[0]; orderY[0] = orderY[1]; orderY[1] = tmp; }
    if (positionYY[orderY[2]] > positionYY[orderY[3]]) { tmp = orderY[2]; orderY[2] = orderY[3]; orderY[3] = tmp; }
    if (positionYY[orderY[0]] > positionYY[orderY[2]]) { tmp = orderY[0]; orderY[0] = orderY[2]; orderY[2] = tmp; }
    if (positionYY[orderY[1]] > positionYY[orderY[3]]) { tmp = orderY[1]; orderY[1] = orderY[3]; orderY[3] = tmp; }
    if (positionYY[orderY[1]] > positionYY[orderY[2]]) { tmp = orderY[1]; orderY[1] = orderY[2]; orderY[2] = tmp; }
}


// --- FASE 2: Assegnazione dei Vertici A,B,C,D tramite Euristiche ---

// orderX[0] è l'indice del punto più a SINISTRA.
// orderY[0] è l'indice del punto più in ALTO.
// orderY[3] è l'indice del punto più in BASSO.

// Calcolo di due distanze chiave per determinare l'orientamento della rotazione.
int32_t dx;
int32_t dy;
int32_t dist_sq1;
int32_t dist_sq2;

// Distanza tra il punto più in alto e il punto più a sinistra
dx = positionXX[orderY[0]] - positionXX[orderX[0]];
dy = positionYY[orderY[0]] - positionYY[orderX[0]];
dist_sq1 = (dx * dx) + (dy * dy);

// Distanza tra il punto più in basso e il punto più a sinistra
dx = positionXX[orderY[3]] - positionXX[orderX[0]];
dy = positionYY[orderY[3]] - positionYY[orderX[0]];
dist_sq2 = (dx * dx) + (dy * dy);

// Definisce una soglia per rilevare quando il rettangolo è quasi verticale.
// Se la differenza di altezza tra i due punti più alti è troppo grande,
// siamo in una rotazione "normale", altrimenti entriamo nella "zona critica".
const int CRITICAL_ZONE = (30 * CamToMouseMult);

if ((positionYY[orderY[1]] - positionYY[orderY[0]]) > CRITICAL_ZONE)
{
    // CASO NORMALE: Il rettangolo non è quasi verticale.
    // L'euristica si basa su quale angolo (in alto a sx o in basso a sx)
    // è più "stretto", per dedurre la direzione della rotazione.
    if (dist_sq1 < dist_sq2)
    {
        // Il vertice A è il più vicino all'angolo in alto a sinistra dello schermo.
        // Tipico di una rotazione ANTI-ORARIA.
        a = orderX[0];
        d = orderX[3];
        if (orderX[1] == orderY[3]) {
            c = orderX[1];
            b = orderX[2];
        } else {
            b = orderX[1];
            c = orderX[2];
        }
    }
    else
    {
        // Il vertice C è il più vicino all'angolo in alto a sinistra dello schermo.
        // Tipico di una rotazione ORARIA.
        c = orderX[0];
        b = orderX[3];
        if (orderX[1] == orderY[3]) {
            d = orderX[1];
            a = orderX[2];
        } else {
            a = orderX[1];
            d = orderX[2];
        }
    }
}
else
{
    // CASO ZONA CRITICA: Il rettangolo è quasi verticale.
    // L'euristica precedente fallisce, quindi si usa un'assegnazione
    // più semplice basata solo sull'ordinamento verticale.
    a = orderY[0];
    b = orderY[1];
    c = orderY[2];
    d = orderY[3];
}

// --- FASE 3: Correzione Finale dell'Ordinamento ---
// Questo blocco di swap garantisce che, indipendentemente dall'euristica usata,
// l'assegnazione finale rispetti la convenzione desiderata
// (es. A e B sopra a C e D; A e C a sinistra di B e D).

// Assicura che A sia sopra C e B sia sopra D
if (positionYY[a] > positionYY[c]) { int aux_swap = a; a = c; c = aux_swap; }
if (positionYY[b] > positionYY[d]) { int aux_swap = b; b = d; d = aux_swap; }

// Assicura che A sia a sinistra di B e C sia a sinistra di D
if (positionXX[a] > positionXX[b]) { int aux_swap = a; a = b; b = aux_swap; }
if (positionXX[c] > positionXX[d]) { int aux_swap = c; c = d; d = aux_swap; }


// --- FASE 4: Assegnazione Finale e Calcoli ---

// Copia le coordinate ordinate negli array di stato finali, usando gli indici
// a, b, c, d appena determinati per mappare la posizione corretta.
FinalX[0] = positionXX[a]; // Assegna il punto A all'indice 0
FinalY[0] = positionYY[a];
FinalX[1] = positionXX[b]; // Assegna il punto B all'indice 1
FinalY[1] = positionYY[b];
FinalX[2] = positionXX[c]; // Assegna il punto C all'indice 2
FinalY[2] = positionYY[c];
FinalX[3] = positionXX[d]; // Assegna il punto D all'indice 3
FinalY[3] = positionYY[d];

// Calcolo del centroide (o punto mediano) del quadrilatero finale.
// L'aggiunta di 2 prima di dividere per 4 è un trucco per ottenere
// l'arrotondamento matematico invece del troncamento con interi.
medianX = (FinalX[0] + FinalX[1] + FinalX[2] + FinalX[3] + 2) / 4;
medianY = (FinalY[0] + FinalY[1] + FinalY[2] + FinalY[3] + 2) / 4;

// Calcolo di height, width, e angle, mantenuti come richiesto per altre parti del codice.
// NOTA: La funzione hypotf calcola la radice quadrata di (x^2 + y^2), richiede <math.h>
float yDistLeft = hypotf((float)FinalY[0] - FinalY[2], (float)FinalX[0] - FinalX[2]);
float yDistRight = hypotf((float)FinalY[3] - FinalY[1], (float)FinalX[3] - FinalX[1]);
float xDistTop = hypotf((float)FinalY[0] - FinalY[1], (float)FinalX[0] - FinalX[1]);
float xDistBottom = hypotf((float)FinalY[2] - FinalY[3], (float)FinalX[2] - FinalX[3]);
height = (yDistLeft + yDistRight) / 2.0f;
width = (xDistTop + xDistBottom) / 2.0f;

// Calcola l'angolo medio dei due lati orizzontali.
angle = (atan2f((float)FinalY[0] - FinalY[1], (float)FinalX[1] - FinalX[0]) + atan2f((float)FinalY[2] - FinalY[3], (float)FinalX[3] - FinalX[2])) / 2.0f;


    }
}

////////////////////////////////////////////////////////////////////////////////////
/////////// roba mia funzionante finoa a 2 punti ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


#endif //USE_SQUARE_ADVANCED