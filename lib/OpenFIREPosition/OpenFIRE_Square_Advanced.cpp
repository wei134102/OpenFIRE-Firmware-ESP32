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
    
    
    /// ===== CODICE MIO PER 2 SENSORI VISTI ==========================================
    #ifdef COMMENTO
    
  // controlla se siamo nel lato corto o nel lato lungo

  // All'interno di if (num_points_seen == 2)

  // 1. Calcola i componenti dei vettori CA e CB (risultati interi)
  int32_t vec_CA_x = positionXX[0] - medianX;
  int32_t vec_CA_y = positionYY[0] - medianY;
  
  int32_t vec_CB_x = positionXX[1] - medianX;
  int32_t vec_CB_y = positionYY[1] - medianY;

  int32_t sum_x = vec_CA_x + vec_CB_x;
  int32_t sum_y = vec_CA_y + vec_CB_y;

  // 2. Calcola il prodotto scalare (dot product). Usiamo 'long' per sicurezza.
  int32_t dotProduct = (vec_CA_x * vec_CB_x) + (vec_CA_y * vec_CB_y);

  //int a,b,c,d = -1;
  int P1; 
  int P2;
  int P1x = positionXX[0]; 
  int P1y = positionYY[0];
  int P2x = positionXX[1]; 
  int P2y = positionYY[1];

  // 3. Controlla il segno del prodotto scalare. Con gli interi il confronto è esatto.
  if (dotProduct == 0) { // angolo retto  ==  90 gradi - non dovrebbe capitare mai
    return;
    // Angolo retto - non dovrebbe capitare mai
    // NON DOVREBBE MAI ACCADERE TRATTANDOSI DI UN RETTANGONO E NON DI UN QUADRATO
  } else if (dotProduct > 0) { // angolo acuto < 90 gradi -- vuol dire che si tratta del lato corto del rettangonlo
    // Angolo acuto - si tratta del lato corto quindi AB o CD
    // LATO CORTO - i punti visti sono AB o CD
    // Confronto verticale
  if (sum_y < 0) { // il lato è sopra al centroide
    // trattasi di AB
    // FARE CONTROLLO SU COORDINATA 'X' PER CAPIRE L'ORDINE DI A B
    P1 = 0, P2 = 1;
    if (P1x < P2x) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
  } else if (sum_y > 0) { // il lato è sotto al centroide
    // trattasi di CD
    // FARE CONTROLLO SU COORDINATA 'X' PER CAPIRE L'ORDINE DI C D
    P1 = 2, P2 = 3;
    if (P1x < P2x) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
  } else { // esattamente centrale rispetto al centroide
    // decidere se impostare AB o CD
// considera AB ??
    return;
  }

  } else { // angolo ottuso  > 90 gradi -- vuol dire che si tratta del lato lungo del rettangolo
    // Angolo ottuso - si tratta del lato lungo - quindi AC o BD
    // LATO LUNGO - i punti visti sono AC o BD
    // Confronto orizzontale
    if (sum_x < 0) { // il lato è a sinistra del centroide
        // trattasi di AC
        // FARE CONTROLLO SU ALTEZZA 'Y' PER CAPIRE L'ORDINE DI A C
        P1 = 0, P2 = 2;
    if (P1y < P2y) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
    } else if (sum_x > 0) { // il lato è a destra del centroide
        // trattasi di BD
        // FARE CONTROLLO SU ALTEZZA 'Y' PER CAPIRE L'ORDINE DI B D
        P1 = 1, P2 = 3;
    if (P1y < P2y) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
    } else { // esattamente centrale al centroide
        //decidere se impostare AC o BD
    // considera AC ???
    return;
    
    } 
  
}

#endif // COMMENTO
//fine codice mio

// codice gemini
#ifndef COMMENTO

// 1. Calcola la distanza al quadrato del segmento VISIBILE ORA.
int32_t dx = (int32_t)positionXX[0] - positionXX[1];
int32_t dy = (int32_t)positionYY[0] - positionYY[1];
int32_t current_dist_sq = dx * dx + dy * dy;

////////////////////////// aggiunta per vertici opposti ////////////////////////////////////////

// 1b. Calcola le dimensioni di riferimento (lati e diagonale) dall'ultimo frame valido (FinalX/Y)
dx = (int32_t)FinalX[A] - FinalX[B]; dy = (int32_t)FinalY[A] - FinalY[B];
int32_t last_avg_width_sq = dx * dx + dy * dy; // Riferimento per il lato "larghezza"

dx = (int32_t)FinalX[A] - FinalX[C]; dy = (int32_t)FinalY[A] - FinalY[C];
int32_t last_avg_height_sq = dx * dx + dy * dy; // Riferimento per il lato "altezza"

// Calcolo la diagonale al quadrato con Pitagora
int32_t last_diagonal_sq = last_avg_width_sq + last_avg_height_sq;

// 1c. Confronta la distanza attuale con i 3 riferimenti per decidere il caso
int32_t diff_as_width = abs(current_dist_sq - last_avg_width_sq);
int32_t diff_as_height = abs(current_dist_sq - last_avg_height_sq);
int32_t diff_as_diag = abs(current_dist_sq - last_diagonal_sq);

// --- Parte 2: Esecuzione della Logica Corretta in base al caso ---

if (diff_as_diag < diff_as_width && diff_as_diag < diff_as_height)
{
    // CASO 1: I PUNTI VISTI SONO UNA DIAGONALE (VERTICI OPPOSTI)
    // Eseguiamo la ricostruzione geometrica (assumendo un quadrato per semplicità e robustezza)
    // Questa logica è stateless e corregge ogni errore di drift accumulato.

    const int32_t p1_x = positionXX[0];
    const int32_t p1_y = positionYY[0];
    const int32_t p2_x = positionXX[1];
    const int32_t p2_y = positionYY[1];

    // 1. Trova il centro della diagonale
    const int32_t center_x = (p1_x + p2_x) / 2;
    const int32_t center_y = (p1_y + p2_y) / 2;

    // 2. Calcola il vettore dal centro a p1
    const int32_t vec_x = p1_x - center_x;
    const int32_t vec_y = p1_y - center_y;

    // 3. Ruota il vettore di 90 gradi
    const int32_t rotated_vec_x = -vec_y;
    const int32_t rotated_vec_y = vec_x;

    // 4. Calcola i due punti mancanti
    const int32_t p3_x = center_x + rotated_vec_x;
    const int32_t p3_y = center_y + rotated_vec_y;
    const int32_t p4_x = center_x - rotated_vec_x;
    const int32_t p4_y = center_y - rotated_vec_y;
    
    // 5. Aggiorna TUTTI e 4 i punti.
    // L'assegnamento corretto agli indici A,B,C,D richiederebbe un ulteriore
    // passaggio di "matching" con le posizioni precedenti (FinalX/Y).
    // Per ora, questo codice popola i 4 slot con una stima geometricamente perfetta
    // e con un ordinamento coerente (es. orario).
    positionXX[0] = p1_x; positionYY[0] = p1_y;
    positionXX[1] = p3_x; positionYY[1] = p3_y;
    positionXX[2] = p2_x; positionYY[2] = p2_y;
    positionXX[3] = p4_x; positionYY[3] = p4_y;

}
else 
{
////////////////////////////////////////////////////////////////////////////////////////////////

// 2. CALCOLA ON-DEMAND le dimensioni medie dei lati al quadrato dall'ultimo frame valido.
dx = (int32_t)FinalX[A] - FinalX[B]; dy = (int32_t)FinalY[A] - FinalY[B];
int32_t last_side_AB_sq = dx * dx + dy * dy;
dx = (int32_t)FinalX[C] - FinalX[D]; dy = (int32_t)FinalY[C] - FinalY[D];
int32_t last_side_CD_sq = dx * dx + dy * dy;
int32_t last_avg_width_sq = (last_side_AB_sq + last_side_CD_sq) / 2; // Lato "corto"

dx = (int32_t)FinalX[A] - FinalX[C]; dy = (int32_t)FinalY[A] - FinalY[C];
int32_t last_side_AC_sq = dx * dx + dy * dy;
dx = (int32_t)FinalX[B] - FinalX[D]; dy = (int32_t)FinalY[B] - FinalY[D];
int32_t last_side_BD_sq = dx * dx + dy * dy;
int32_t last_avg_height_sq = (last_side_AC_sq + last_side_BD_sq) / 2; // Lato "lungo"

// 3. Determina se il segmento attuale è più simile al lato corto o a quello lungo.
//    (Confronto semplificato, senza il caso della diagonale)
int32_t diff_as_short = abs(current_dist_sq - last_avg_width_sq);
int32_t diff_as_long = abs(current_dist_sq - last_avg_height_sq);

///////////////////////////////////
// 1. Calcola i componenti dei vettori CA e CB (risultati interi)
  int32_t vec_CA_x = positionXX[0] - medianX;
  int32_t vec_CA_y = positionYY[0] - medianY;
  
  int32_t vec_CB_x = positionXX[1] - medianX;
  int32_t vec_CB_y = positionYY[1] - medianY;

  int32_t sum_x = vec_CA_x + vec_CB_x;
  int32_t sum_y = vec_CA_y + vec_CB_y;

  // 2. Calcola il prodotto scalare (dot product). Usiamo 'long' per sicurezza.
  //int32_t dotProduct = (vec_CA_x * vec_CB_x) + (vec_CA_y * vec_CB_y);

  //int a,b,c,d = -1;
  int P1; 
  int P2;
  int P1x = positionXX[0]; 
  int P1y = positionYY[0];
  int P2x = positionXX[1]; 
  int P2y = positionYY[1];


///////////////////////////////////
// 4. Prendi la decisione primaria (corto vs lungo) e poi usa il "Metodo del Consenso".
if (diff_as_short < diff_as_long) {
    // CASO LATO CORTO (AB vs CD)

    // --- Metodo A (Confronto con Centroide) ---
    int32_t sum_y = (positionYY[0] - medianY) + (positionYY[1] - medianY);
    bool is_AB_by_centroid = (sum_y < 0);

    // --- Metodo B (Tracciamento Punti Medi) ---
    int32_t mid_now_x = (positionXX[0] + positionXX[1]) / 2;
    int32_t mid_now_y = (positionYY[0] + positionYY[1]) / 2;

    int32_t mid_AB_x = (FinalX[A] + FinalX[B]) / 2;
    int32_t mid_AB_y = (FinalY[A] + FinalY[B]) / 2;
    int32_t mid_CD_x = (FinalX[C] + FinalX[D]) / 2;
    int32_t mid_CD_y = (FinalY[C] + FinalY[D]) / 2;

    dx = mid_now_x - mid_AB_x; dy = mid_now_y - mid_AB_y;
    int32_t dist_to_AB_sq = dx * dx + dy * dy;
    dx = mid_now_x - mid_CD_x; dy = mid_now_y - mid_CD_y;
    int32_t dist_to_CD_sq = dx * dx + dy * dy;

    bool is_AB_by_proximity = (dist_to_AB_sq < dist_to_CD_sq);

    // --- Decisione basata sul CONSENSO ---
    if (is_AB_by_centroid == is_AB_by_proximity) {
        // I metodi sono d'accordo. Procedi con l'assegnazione.
        if (is_AB_by_centroid) {
            // È il lato AB.
            P1 = A; P2 = B;
            // trattasi di AB
    // FARE CONTROLLO SU COORDINATA 'X' PER CAPIRE L'ORDINE DI A B
    P1 = 0, P2 = 1;
    if (P1x < P2x) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }

        } else {
            // È il lato CD.
            P1 = C; P2 = D;
            // trattasi di CD
    // FARE CONTROLLO SU COORDINATA 'X' PER CAPIRE L'ORDINE DI C D
    P1 = 2, P2 = 3;
    if (P1x < P2x) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
            

        }
    } else {
        // DISACCORDO! Situazione ambigua, usciamo per sicurezza.
        return;
    }

} else {
    // CASO LATO LUNGO (AC vs BD)

    // --- Metodo A (Confronto con Centroide) ---
    int32_t sum_x = (positionXX[0] - medianX) + (positionXX[1] - medianX);
    bool is_AC_by_centroid = (sum_x < 0);

    // --- Metodo B (Tracciamento Punti Medi) ---
    int32_t mid_now_x = (positionXX[0] + positionXX[1]) / 2;
    int32_t mid_now_y = (positionYY[0] + positionYY[1]) / 2;

    int32_t mid_AC_x = (FinalX[A] + FinalX[C]) / 2;
    int32_t mid_AC_y = (FinalY[A] + FinalY[C]) / 2;
    int32_t mid_BD_x = (FinalX[B] + FinalX[D]) / 2;
    int32_t mid_BD_y = (FinalY[B] + FinalY[D]) / 2;

    dx = mid_now_x - mid_AC_x; dy = mid_now_y - mid_AC_y;
    int32_t dist_to_AC_sq = dx * dx + dy * dy;
    dx = mid_now_x - mid_BD_x; dy = mid_now_y - mid_BD_y;
    int32_t dist_to_BD_sq = dx * dx + dy * dy;

    bool is_AC_by_proximity = (dist_to_AC_sq < dist_to_BD_sq);

    // --- Decisione basata sul CONSENSO ---
    if (is_AC_by_centroid == is_AC_by_proximity) {
        // I metodi sono d'accordo. Procedi con l'assegnazione.
        if (is_AC_by_centroid) {
            // È il lato AC.
            P1 = A; P2 = C;
        // trattasi di AC
        // FARE CONTROLLO SU ALTEZZA 'Y' PER CAPIRE L'ORDINE DI A C
 
        P1 = 0, P2 = 2;
    if (P1y < P2y) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }                    

        } else {
            // È il lato BD.
            P1 = B; P2 = D;
        // trattasi di BD
        // FARE CONTROLLO SU ALTEZZA 'Y' PER CAPIRE L'ORDINE DI B D
        P1 = 1, P2 = 3;
    if (P1y < P2y) {
        positionXX[P1] = P1x;
        positionYY[P1] = P1y;
        positionXX[P2] = P2x;
        positionYY[P2] = P2y;
    } else {
        positionXX[P1] = P2x;
        positionYY[P1] = P2y;
        positionXX[P2] = P1x;
        positionYY[P2] = P1y;
    }
           
        }
    } else {
        // DISACCORDO! Situazione ambigua, usciamo per sicurezza.
        return;
    }
}

// Qui inizia il resto della logica di stima dei punti mancanti,
// usando P1 e P2 appena determinati...

//} // messa per blocco if ipotesi vertici opposti
#endif // COMMENTO

////////////////////////////////////////////////////////////////////////////

// ... Il resto della logica di stima dei punti mancanti prosegue da qui ...

    // stima i due vertici mancanti

    // 1. Identifica i punti noti originali e nuovi usando gli indici P1 e P2
    int old_p1x = FinalX[P1];
    int old_p1y = FinalY[P1];
    int old_p2x = FinalX[P2];
    int old_p2y = FinalY[P2];

    int new_p1x = positionXX[P1];
    int new_p1y = positionYY[P1];
    int new_p2x = positionXX[P2];
    int new_p2y = positionYY[P2];

    // 2. Trova un terzo punto originale (P3) adiacente a P1, che non sia P2.
    // Questa logica definisce quale lato adiacente usare per la ricostruzione.
    // Assumiamo la geometria A(0)-B(1), A(0)-C(2), B(1)-D(3), C(2)-D(3).
    int P3 = -1;
    if (P1 == 0)      { P3 = (P2 == 1) ? 2 : 1; } // Adiacenti di A sono B e C
    else if (P1 == 1) { P3 = (P2 == 0) ? 3 : 0; } // Adiacenti di B sono A e D
    else if (P1 == 2) { P3 = (P2 == 0) ? 3 : 0; } // Adiacenti di C sono A e D
    else if (P1 == 3) { P3 = (P2 == 1) ? 2 : 1; } // Adiacenti di D sono B e C

    int old_p3x = FinalX[P3];
    int old_p3y = FinalY[P3];

    // 3. Calcola i vettori originali: il lato noto (P1->P2) e il lato adiacente (P1->P3)
    long vec_side_orig_x = old_p2x - old_p1x;
    long vec_side_orig_y = old_p2y - old_p1y;
    long vec_adj_orig_x = old_p3x - old_p1x;
    long vec_adj_orig_y = old_p3y - old_p1y;

    // 4. Calcola il nuovo vettore del lato noto
    long vec_side_new_x = new_p2x - new_p1x;
    long vec_side_new_y = new_p2y - new_p1y;

    // 5. Calcola la trasformazione (scala e rotazione) usando 'float'
    float len_side_orig = sqrtf((float)vec_side_orig_x * vec_side_orig_x + (float)vec_side_orig_y * vec_side_orig_y);
    float len_side_new = sqrtf((float)vec_side_new_x * vec_side_new_x + (float)vec_side_new_y * vec_side_new_y);
    
    float scale = 1.0f;
    if (len_side_orig > 1e-5f) {
        scale = len_side_new / len_side_orig;
    }

    float angle_side_orig = atan2f(vec_side_orig_y, vec_side_orig_x);
    float angle_side_new = atan2f(vec_side_new_y, vec_side_new_x);
    float rotation_delta = angle_side_new - angle_side_orig;

    // 6. Applica la trasformazione al vettore adiacente
    float vec_adj_scaled_x = vec_adj_orig_x * scale;
    float vec_adj_scaled_y = vec_adj_orig_y * scale;

    float cos_rot = cosf(rotation_delta);
    float sin_rot = sinf(rotation_delta);

    float vec_adj_new_x = vec_adj_scaled_x * cos_rot - vec_adj_scaled_y * sin_rot;
    float vec_adj_new_y = vec_adj_scaled_x * sin_rot + vec_adj_scaled_y * cos_rot;

    // 7. Calcola le coordinate stimate dei vertici mancanti
    // Il primo vertice mancante è la nuova posizione di P3
    int estimated_p3x = static_cast<int>(roundf(new_p1x + vec_adj_new_x));
    int estimated_p3y = static_cast<int>(roundf(new_p1y + vec_adj_new_y));

    // Il quarto vertice si trova usando la regola del parallelogramma
    int estimated_p4x = static_cast<int>(roundf(new_p2x + vec_adj_new_x));
    int estimated_p4y = static_cast<int>(roundf(new_p2y + vec_adj_new_y));

    // 8. Trova gli indici dei due vertici mancanti e aggiorna gli array
    for (int i = 0; i < 4; ++i) {
        if (i != P1 && i != P2) { // Questo è un indice mancante
            if (i == P3) { // Se questo indice corrisponde al P3 che abbiamo usato...
                positionXX[i] = estimated_p3x;
                positionYY[i] = estimated_p3y;
            } else { // Altrimenti, è il quarto e ultimo vertice
                positionXX[i] = estimated_p4x;
                positionYY[i] = estimated_p4y;
            }
        }
    }

    // non ci dovrebbe essere bisogno idi ordinare logicamente positionXX e YY perchè dovrebbe essere già a posto
} // messa per blocco if ipotesi vertici opposti
  
    } // FINE SOLO 2 PUNTI VISIBILI
        
    ////////// IN CASO DI SOLI 3 PUNTI ////// CALCOLA IL QUARTO ////////////////////////
    if (num_points_seen == 3)  // 3 PUNTII VISIBILI, BISOGNA STIMARNE 1
    {



#ifdef COMMENTO
    // --- FASE 1: Identificazione della diagonale ---
    // Le differenze possono essere int
    int32_t dx, dy;
    // Usiamo 'long long' per le distanze al quadrato per evitare overflow con coordinate grandi
    int32_t d01_sq, d12_sq, d02_sq;

    // Distanza al quadrato tra P0 e P1
    dx = positionXX[0] - positionXX[1];
    dy = positionYY[0] - positionYY[1];
    d01_sq = dx * dx + dy * dy;

    // Distanza al quadrato tra P1 e P2
    dx = positionXX[1] - positionXX[2];
    dy = positionYY[1] - positionYY[2];
    d12_sq = dx * dx + dy * dy;

    // Distanza al quadrato tra P0 e P2
    dx = positionXX[0] - positionXX[2];
    dy = positionYY[0] - positionYY[2];
    d02_sq = dx * dx + dy * dy;

    // 2. Trova la diagonale e calcola le coordinate mancanti.
    //    Il risultato viene scritto nelle variabili puntate da missingX e missingY.
    if (d12_sq > d01_sq && d12_sq > d02_sq) {
        // Diagonale: 1-2. Altro vertice: 0.
        positionXX[3] = positionXX[1] + positionXX[2] - positionXX[0];
        positionYY[3] = positionYY[1] + positionYY[2] - positionYY[0];
    } else if (d02_sq > d01_sq && d02_sq > d12_sq) {
        // Diagonale: 0-2. Altro vertice: 1.
        positionXX[3] = positionXX[0] + positionXX[2] - positionXX[1];
        positionYY[3] = positionYY[0] + positionYY[2] - positionYY[1];
    } else { 
         // Diagonale: 0-1. Altro vertice: 2.
        positionXX[3] = positionXX[0] + positionXX[1] - positionXX[2];
        positionYY[3] = positionYY[0] + positionYY[1] - positionYY[2];
    }
    #endif //COMMENTO

    #ifndef COMMENTO
// Inizio del blocco di codice.

// --- 1. Calcola le lunghezze al quadrato con interi a 32 bit ---
int32_t dx01 = positionXX[0] - positionXX[1];
int32_t dy01 = positionYY[0] - positionYY[1];
int32_t d01_sq = dx01 * dx01 + dy01 * dy01;

int32_t dx12 = positionXX[1] - positionXX[2];
int32_t dy12 = positionYY[1] - positionYY[2];
int32_t d12_sq = dx12 * dx12 + dy12 * dy12;

int32_t dx02 = positionXX[0] - positionXX[2];
int32_t dy02 = positionYY[0] - positionYY[2];
int32_t d02_sq = dx02 * dx02 + dy02 * dy02;


// --- 2. Identifica gli indici dei punti A, B, C ---
// Il perno B è il vertice opposto al lato più lungo del triangolo.
int32_t a_idx, c_idx, b_idx;

if (d01_sq >= d12_sq && d01_sq >= d02_sq) {
    // Il lato più lungo è tra 0 e 1. Il perno (B) è il punto 2.
    a_idx = 0;
    c_idx = 1;
    b_idx = 2;
} else if (d12_sq >= d02_sq) {
    // Il lato più lungo è tra 1 e 2. Il perno (B) è il punto 0.
    a_idx = 1;
    c_idx = 2;
    b_idx = 0;
} else {
    // Il lato più lungo è tra 0 e 2. Il perno (B) è il punto 1.
    a_idx = 0;
    c_idx = 2;
    b_idx = 1;
}

// --- 3. Applica la stima e salva il risultato direttamente ---
positionXX[3] = positionXX[a_idx] + positionXX[c_idx] - positionXX[b_idx];
positionYY[3] = positionYY[a_idx] + positionYY[c_idx] - positionYY[b_idx];

// Fine del blocco di codice.

    #endif //COMMENTO


}
 
    ///////////// FINE 3 SENSORI VISTI ///////////////////////
    
    ///// DA QUI IN POI SI ESEGUE SEMPRE, UTILIZZA LE COORDINATE NON ORDINATE DI 4 SENSORI VISTI

    //// tutti 4 sensori visti

//// ======== INIZIO BLOCCO LOGICA A 4 PUNTI (IBRIDA E POTENZIATA) ========

// Definiamo gli indici a,b,c,d all'inizio affinché siano disponibili per entrambi i rami della logica
    
    int a,b,c,d;
    
    
    int orderX[4] = {0,1,2,3};
    int orderY[4] = {0,1,2,3};

    
    /*
    // ORDINA VETTORI - CODICE NON OTTIMIZZATO LENTO MA CLASSICO BOUBLE SORT PER ORDINARE 4 ELEMENTI
    // PIU' LEGGIBILE E PIU' BREVE MA PIU' LENTO RISPETTO ALLA SOILUZIONE SUCCESSIVA SEPPUR PIU' VERBOSA
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            if (positionXX[orderX[i]] > positionXX[orderX[j]]) {
                // Scambia i valori di positionXX
                int tempX = orderX[i];
                orderX[i] = orderX[j];
                orderX[j] = tempX;
            }
            if (positionYY[orderY[i]] > positionYY[orderY[j]]) {
                // Scambia i valori di positionXX
                int tempY = orderY[i];
                orderY[i] = orderY[j];
                orderY[j] = tempY;
            }   
        }
    }
    // FINE ORDINA VETTORE
    */


{  //CODICE OTTIMIZZATO PER L'ORINANMENTO PIU' VELOCE POSSIBILE DI SOLI 4 ELEMENTI
    // ordina le X
    int tmp;
    // passo 1
    if (positionXX[orderX[0]] > positionXX[orderX[1]]) {
        tmp         = orderX[0];
        orderX[0]   = orderX[1];
        orderX[1]   = tmp;
    }
    // passo 2
    if (positionXX[orderX[2]] > positionXX[orderX[3]]) {
        tmp         = orderX[2];
        orderX[2]   = orderX[3];
        orderX[3]   = tmp;
    }
    // passo 3
    if (positionXX[orderX[0]] > positionXX[orderX[2]]) {
        tmp         = orderX[0];
        orderX[0]   = orderX[2];
        orderX[2]   = tmp;
    }
    // passo 4
    if (positionXX[orderX[1]] > positionXX[orderX[3]]) {
        tmp         = orderX[1];
        orderX[1]   = orderX[3];
        orderX[3]   = tmp;
    }
    // passo 5
    if (positionXX[orderX[1]] > positionXX[orderX[2]]) {
        tmp         = orderX[1];
        orderX[1]   = orderX[2];
        orderX[2]   = tmp;
    }
}

//   //CODICE OTTIMIZZATO PER L'ORINANMENTO PIU' VELOCE POSSIBILE DI SOLI 4 ELEMENTI
// Ripeti esattamente lo stesso schema per orderY/positionYY
{
    int tmp;
    if (positionYY[orderY[0]] > positionYY[orderY[1]]) {
        tmp         = orderY[0];
        orderY[0]   = orderY[1];
        orderY[1]   = tmp;
    }
    if (positionYY[orderY[2]] > positionYY[orderY[3]]) {
        tmp         = orderY[2];
        orderY[2]   = orderY[3];
        orderY[3]   = tmp;
    }
    if (positionYY[orderY[0]] > positionYY[orderY[2]]) {
        tmp         = orderY[0];
        orderY[0]   = orderY[2];
        orderY[2]   = tmp;
    }
    if (positionYY[orderY[1]] > positionYY[orderY[3]]) {
        tmp         = orderY[1];
        orderY[1]   = orderY[3];
        orderY[3]   = tmp;
    }
    if (positionYY[orderY[1]] > positionYY[orderY[2]]) {
        tmp         = orderY[1];
        orderY[1]   = orderY[2];
        orderY[2]   = tmp;
    }
}

  // Calcola il quadrato della distanza
    
  int32_t dx;
  int32_t dy;
  int32_t dist_sq1;
  int32_t dist_sq2;
  

  dx = positionXX[orderY[0]] - positionXX[orderX[0]]; //- positionX[orderY[0]]; // x1 - x2; tra punto più a sinistra e punto più in alto
  dy = positionYY[orderY[0]] - positionYY[orderX[0]]; //- positionY[orderY[0]]; //y1 - y2;  tra punto più a sinistra e punto più in alto
  dist_sq1 = (dx * dx) + (dy * dy); // lunghezza tra quello più in alto e quello più a sinistra
  //dist1 = sqrt(dx*dx + dy*dy);
  dx = positionXX[orderY[3]] - positionXX[orderX[0]]; //- positionX[orderY[3]]; //x1 - x2; tra puanto più a sinistra e punto più in basso
  dy = positionYY[orderY[3]] - positionYY[orderX[0]]; //- positionY[orderY[3]]; //y1 - y2; tra puanto più a sinistra e punto più in basso
  dist_sq2 = (dx * dx) + (dy * dy); // differenza tra quello più in basso e quello più a sinistra
  //dist2 = sqrt(dx*dx + dy*dy);

  //deve gestire la situazione di errori dovuti ad un non perfetto allineamento dei sesori e rumore dei sensori stessi
  // pertanto quando il rettangolo è quasi verticale deve usare un altro approccio per rilevare i giusti vertici
  // userei una piccola isterisi/buffer per capire quanto siano allienati i sensori e calcolare con l'approccio alternativo fino a un tot di rotazione
  
  const int CRITICAL_ZONE = (30 * CamToMouseMult); // differenza di Y tra i due punti più alti // 50 è molto robusto


  if ((positionYY[orderY[1]] - positionYY[orderY[0]]) > CRITICAL_ZONE) {
  // ======== BUONO FINO 60-70 GRADI ==========================
    //if (positionYY[orderX[0]] < positionYY[orderX[3]])
    if (dist_sq1 < dist_sq2)
    //if (dist1 < dist2)
    { // abbiamo A // rotazione antioraria
        /*
        if ((millis() - testLastStampAux) > 200) {
            //Serial.printf("AAAAAA A-B: %.6f  B-C: %.6f\n", dist1,dist2);
            //Serial.printf("AAAAAA HL: %5d  HR: %5d\n", positionYY[orderX[0]],positionYY[orderX[3]]);
            Serial.println("Quadrante AAAAAAAAAAAAAAAAAAAAAAAAAA");
            testLastStampAux = millis();
        }
        */
        a = orderX[0];
        d = orderX[3];
        if (orderX[1] == orderY[3])
        { // ACBD
            //a = orderX[0];
            c = orderX[1];
            b = orderX[2];
            //d = orderX[3];
        }
        else
        { //ABCD
            //a = orderX[0];
            b = orderX[1];
            c = orderX[2];
            //d = orderX[3];

        }
    }
    else 
    { // abbiamo C // rotazione oraria
        /*
        if ((millis() - testLastStampAux) > 200) {
            //Serial.printf("CCC A-B: %.6f  B-C: %.6f\n", dist1,dist2);
            //Serial.printf("CCC HL: %5d  HR: %5d\n", positionYY[orderX[0]],positionYY[orderX[3]]);
            Serial.println("Quadrante CCCC");
            testLastStampAux = millis();
        }
        */
        c = orderX[0];
        b = orderX[3];
        if (orderX[1] == orderY[3])
        { // CDAB
            //c = orderX[0];
            d = orderX[1];
            a = orderX[2];
            //b = orderX[3];
        }
        else
        { //CADB
            //c = orderX[0];
            a = orderX[1];
            d = orderX[2];
            //b = orderX[3];
        }
    }
}
    else
    { // gestione zona critica
        /*
        if ((millis() - testLastStampAux) > 200) {
            //Serial.printf("CCC A-B: %.6f  B-C: %.6f\n", dist1,dist2);
            //Serial.printf("CCC HL: %5d  HR: %5d\n", positionYY[orderX[0]],positionYY[orderX[3]]);
            Serial.println("DANGER -- ZONA CRITICA");
            testLastStampAux = millis();
        }
        */
        a=orderY[0];
        b=orderY[1];
        c=orderY[2];
        d=orderY[3];
    }
    // ======== BUONO FINO 80-90 GRADI ==========================
   
    // controllo inversione quando la telecamera è esattamente in posizione neuatra e potrebbe confondere
    // il punto più alto o quello più a sinistra e quindi scambiare le coordinate
    //Controllare con la coordinata Y se bisogna scambiare A con C
    //Controllare con la coordinata Y se bisogna scambiare B con D
 
 

    // controllo verticale
    if (positionYY[a] > positionYY[c]) { // scambia a con c;
        int aux_swap = a;
        a = c;
        c = aux_swap;
    }
    if (positionYY[b] > positionYY[d]) {// scambia b con d;
        int aux_swap = b;
        b = d;
        d = aux_swap;
    }

    // ???????????????????? controllo orrizzontale
    if (positionXX[a] > positionXX[b]) { // scambia a con b;
        int aux_swap = a;
        a = b;
        b = aux_swap;
    }
    if (positionXX[c] > positionXX[d]) {// scambia b con d;
        int aux_swap = c;
        c = d;
        d = aux_swap;
    }
 

    
    FinalX[0] = positionXX[a];
    FinalY[0] = positionYY[a];
    FinalX[1] = positionXX[b];
    FinalY[1] = positionYY[b];
    FinalX[2] = positionXX[c];
    FinalY[2] = positionYY[c];
    FinalX[3] = positionXX[d];
    FinalY[3] = positionYY[d];

// Calcolo della medianX e medianY
medianX = (FinalX[0] + FinalX[1] + FinalX[2] + FinalX[3]) / 4;   // +2 ?????????
medianY = (FinalY[0] + FinalY[1] + FinalY[2] + FinalY[3]) / 4;   // +2 ?????????

//////// calcolo di height e width, mai usati, quindi propongo di toglierli
yDistLeft = hypot((FinalY[0] - FinalY[2]), (FinalX[0] - FinalX[2]));
yDistRight = hypot((FinalY[3] - FinalY[1]), (FinalX[3] - FinalX[1]));
xDistTop = hypot((FinalY[0] - FinalY[1]), (FinalX[0] - FinalX[1]));
xDistBottom = hypot((FinalY[2] - FinalY[3]), (FinalX[2] - FinalX[3]));
height = (yDistLeft + yDistRight) / 2.0f;
width = (xDistTop + xDistBottom) / 2.0f;

// Il calcolo dell'angolo che avevi era già corretto per l'orientamento del lato superiore:
// angle = atan2f(static_cast<float>(FinalY[1] - FinalY[0]), static_cast<float>(FinalX[1] - FinalX[0]));
// o, se preferisci basarti sul vettore da A a B (come avevi FinalY[0]-FinalY[1] ma con X invertite):
//angle = atan2f(static_cast<float>(FinalY[1] - FinalY[0]), static_cast<float>(FinalX[1] - FinalX[0]));
// anche questo non viene mai usato, propongo di toglierlo
angle = (atan2(FinalY[0] - FinalY[1], FinalX[1] - FinalX[0]) + atan2(FinalY[2] - FinalY[3], FinalX[3] - FinalX[2])) / 2.0f;

/*
for (int i = 0; i < 4; ++i) {
        see[i] = 1;
    }
*/

    }
}

////////////////////////////////////////////////////////////////////////////////////
/////////// roba mia funzionante finoa a 2 punti ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


#endif //USE_SQUARE_ADVANCED