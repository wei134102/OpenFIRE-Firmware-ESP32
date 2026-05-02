#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_Square_Advanced.cpp
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 *
 * I thank you for producing the first original code:
 * 
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 */

#include <Arduino.h>
#include "OpenFIRE_Square_Advanced.h"

#include <cfloat>



/**
 * @brief Calcola la radice quadrata inversa (1/sqrt(x)) in modo approssimato ma estremamente veloce.
 * @param number Il numero di cui calcolare la radice quadrata inversa.
 * @return Un valore approssimato di 1 / sqrt(number).
 */
inline float fast_inv_sqrtf(float number) {
    union {
        float f;
        uint32_t i;
    } conv = { .f = number };
    conv.i = 0x5f3759df - (conv.i >> 1); // Stima iniziale con "magic number"
    // Iterazione di Newton-Raphson per raffinare la stima e migliorare la precisione
    conv.f *= 1.5f - (number * 0.5f * conv.f * conv.f);
    return conv.f;
}

/**
 * @brief Arrotonda un valore float all'intero più vicino in modo veloce.
 * Funziona correttamente sia con valori positivi che negativi.
 * @param val Il valore float da arrotondare.
 * @return L'intero arrotondato.
 */
inline int fast_roundf(float val) {
    // Aggiunge o sottrae 0.5 e poi tronca (casting a int).
    // Esempio: fast_roundf(-3.7) -> (int)(-3.7 - 0.5) -> (int)(-4.2) -> -4
    // Esempio: fast_roundf(3.7)  -> (int)(3.7 + 0.5)  -> (int)(4.2)  -> 4
    return (int)(val + (val > 0.0f ? 0.5f : -0.5f));
}



// Definizione degli indici per chiarezza
#define A 0
#define B 1
#define C 2
#define D 3

void OpenFIRE_Square::begin(const int* px, const int* py, unsigned int seen) {

    // =========================================================================//
    // ================= FASE 0: INIT E ACQUISIZIONE DATI REALI ================//
    // =========================================================================//
    
    seenFlags = seen;

    if (seenFlags == 0x0F) { 
        start = 0xFF;
    } else if (!start) {
        return; 
    }

    uint8_t num_points_seen = 0;
    int positionXX[4];
    int positionYY[4];

    for (uint8_t i = 0; i < 4; ++i) {
        if ((seenFlags >> i) & 0x01) {
            positionXX[num_points_seen] = (CamMaxX - px[i]) << CamToMouseShift;
            positionYY[num_points_seen] = py[i] << CamToMouseShift;
            num_points_seen++;
        }
    }

    int real_x[4] = {-9999, -9999, -9999, -9999};
    int real_y[4] = {-9999, -9999, -9999, -9999};
    for(uint8_t i=0; i<num_points_seen; i++) {
        real_x[i] = positionXX[i];
        real_y[i] = positionYY[i];
    }

    if (num_points_seen >= 1) {

        // =========================================================================//
        // ======== CANCELLO DI SICUREZZA (COLD START DOPO IL BUIO) ================//
        // =========================================================================//
        if (!is_tracking_stable && num_points_seen < 3) {
            prev_num_points_seen = 0; 
            return; 
        }

        int32_t dx, dy;

        // =========================================================================//
        // ========================= GESTIONE 1 PUNTO ==============================//
        // =========================================================================//
        if (num_points_seen == 1) {
            if (prev_num_points_seen >= 1) {
                const int x1 = positionXX[0]; 
                const int y1 = positionYY[0];

                uint8_t identified_idx = 0;
                int32_t min_dist_sq = INT32_MAX; 

                for (uint8_t i = 0; i < 4; i++) {
                    const int dx_dist = x1 - FinalX[i];
                    const int dy_dist = y1 - FinalY[i];
                    const int32_t dist_sq = (int32_t)dx_dist * dx_dist + (int32_t)dy_dist * dy_dist;

                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        identified_idx = i;
                    }
                }

                const int max_jump_distance = ((int)width * 3) / 4; 
                const int32_t MAX_ALLOWED_DISTANCE_SQ = (int32_t)max_jump_distance * max_jump_distance;

                if (min_dist_sq > MAX_ALLOWED_DISTANCE_SQ) {
                    prev_num_points_seen = 0;
                    return; 
                }

                const int delta_x = x1 - FinalX[identified_idx];
                const int delta_y = y1 - FinalY[identified_idx];

                for (uint8_t i = 0; i < 4; i++) {
                    positionXX[i] = FinalX[i] + delta_x;
                    positionYY[i] = FinalY[i] + delta_y;
                }
            } else {
                prev_num_points_seen = 0;
                return;
            }
        }
        
        // =========================================================================//
        // ========================= GESTIONE 2 PUNTI ==============================//
        // =========================================================================//
        else if (num_points_seen == 2) {
            const int x1 = positionXX[0], y1 = positionYY[0];
            const int x2 = positionXX[1], y2 = positionYY[1];

            uint8_t best_idx1 = 0, best_idx2 = 1;
            int32_t min_total_cost = INT32_MAX;
            const uint8_t pairs[6][2] = { {0,1}, {2,3}, {0,2}, {1,3}, {0,3}, {1,2} };         

            // Distanza fisica tra i due punti per evitare la Cecità Rotazionale
            int32_t read_dx = x2 - x1;
            int32_t read_dy = y2 - y1;
            int32_t read_dist_sq = read_dx * read_dx + read_dy * read_dy;

            for (uint8_t i = 0; i < 6; i++) {
                const uint8_t v1 = pairs[i][0], v2 = pairs[i][1];
                int32_t penalty = 0;
                
                if (prev_num_points_seen >= 1 && prev_point_seen_mask != 0) {
                    bool p1_was_seen = (prev_point_seen_mask & (1 << (3 - v1))) != 0;
                    bool p2_was_seen = (prev_point_seen_mask & (1 << (3 - v2))) != 0;
                    if (!p1_was_seen) penalty += PENALITA_STORICA;
                    if (!p2_was_seen) penalty += PENALITA_STORICA;
                }

                // NUOVO: PENALITÀ INTRINSECA (Impedisce lo scambio Base/Altezza 1->2)
                int32_t expected_dist_sq = 0;
                if (i == 0 || i == 1) expected_dist_sq = (int32_t)(width * width); 
                else if (i == 2 || i == 3) expected_dist_sq = (int32_t)(height * height); 
                else expected_dist_sq = (int32_t)(width * width + height * height);

                penalty += abs(read_dist_sq - expected_dist_sq);

                int32_t dx1a=x1-FinalX[v1], dy1a=y1-FinalY[v1];
                int32_t dx2a=x2-FinalX[v2], dy2a=y2-FinalY[v2];
                int32_t costA = dx1a*dx1a + dy1a*dy1a + dx2a*dx2a + dy2a*dy2a + penalty;
                if (costA < min_total_cost) {
                    min_total_cost = costA;
                    best_idx1 = v1; best_idx2 = v2;
                }

                int32_t dx1b=x1-FinalX[v2], dy1b=y1-FinalY[v2];
                int32_t dx2b=x2-FinalX[v1], dy2b=y2-FinalY[v1];
                int32_t costB = dx1b*dx1b + dy1b*dy1b + dx2b*dx2b + dy2b*dy2b + penalty;
                if (costB < min_total_cost) {
                    min_total_cost = costB;
                    best_idx1 = v2; best_idx2 = v1;
                }
            }

            float Vx[4], Vy[4];
            for (uint8_t i = 0; i < 4; i++) { Vx[i] = (float)FinalX[i]; Vy[i] = (float)FinalY[i]; }
            Vx[best_idx1] = (float)x1; Vy[best_idx1] = (float)y1;
            Vx[best_idx2] = (float)x2; Vy[best_idx2] = (float)y2;
    
            const uint8_t idx1 = best_idx1, idx2 = best_idx2;
            bool top    = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
            bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
            bool left   = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
            bool right  = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

            // NUOVO: FIX DEL BUG CLESSIDRA (Si swappano anche p3 e p4!)
            if (top || bottom) {
                uint8_t p1 = top ? 0 : 2, p2 = top ? 1 : 3;
                uint8_t p3 = top ? 2 : 0, p4 = top ? 3 : 1;
                if (Vx[p1] > Vx[p2]) { 
                    uint8_t t=p1; p1=p2; p2=t; 
                    t=p3; p3=p4; p4=t; 
                }
                float dx_v = Vx[p2]-Vx[p1], dy_v = Vy[p2]-Vy[p1];
                float base = sqrtf(dx_v*dx_v + dy_v*dy_v);
                if (base > 2.0f) {
                    float cos_angle = fabsf(dx_v) / base;
                    base = base / fmaxf(cos_angle, 0.3f);
                    float newH = fmaxf(base / ideal_aspect_ratio, height * 0.5f);
                    float inv_base = 1.0f / base;
                    float ndx = -dy_v*inv_base, ndy = dx_v*inv_base;
                    float sx = ndx*newH, sy = ndy*newH;
                    if (top) {
                        Vx[p3]=Vx[p1]+sx; Vy[p3]=Vy[p1]+sy; Vx[p4]=Vx[p2]+sx; Vy[p4]=Vy[p2]+sy;
                    } else {
                        Vx[p3]=Vx[p1]-sx; Vy[p3]=Vy[p1]-sy; Vx[p4]=Vx[p2]-sx; Vy[p4]=Vy[p2]-sy;
                    }
                }
            } else if (left || right) {
                uint8_t p1 = left ? 0 : 1, p2 = left ? 2 : 3;
                uint8_t p3 = left ? 1 : 0, p4 = left ? 3 : 2;
                if (Vy[p1] > Vy[p2]) { 
                    uint8_t t=p1; p1=p2; p2=t; 
                    t=p3; p3=p4; p4=t; 
                }
                float dx_v = Vx[p2]-Vx[p1], dy_v = Vy[p2]-Vy[p1];
                float hgt = sqrtf(dx_v*dx_v+dy_v*dy_v);
                if (hgt > 2.0f) {
                    float newW = hgt * ideal_aspect_ratio;
                    float inv_hgt = 1.0f/hgt;
                    float ndx = -dy_v*inv_hgt, ndy = dx_v*inv_hgt;
                    float sx = ndx*newW, sy = ndy*newW;
                    if (left) {
                        Vx[p3]=Vx[p1]-sx; Vy[p3]=Vy[p1]-sy; Vx[p4]=Vx[p2]-sx; Vy[p4]=Vy[p2]-sy;
                    } else {
                        Vx[p3]=Vx[p1]+sx; Vy[p3]=Vy[p1]+sy; Vx[p4]=Vx[p2]+sx; Vy[p4]=Vy[p2]+sy;
                    }
                }
            } else { 
                bool diagAD = (idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0);
                float px1 = diagAD ? Vx[0] : Vx[1], py1 = diagAD ? Vy[0] : Vy[1];
                float px2 = diagAD ? Vx[3] : Vx[2], py2 = diagAD ? Vy[3] : Vy[2];
                float dx_v = px2 - px1, dy_v = py2 - py1;
                float L = sqrtf(dx_v*dx_v + dy_v*dy_v);
                if (L > 2.0f) {
                    float H = L / sqrtf(ideal_aspect_ratio*ideal_aspect_ratio + 1.0f);
                    float W = H * ideal_aspect_ratio;
                    float phi_m = atan2f(dy_v, dx_v);
                    float phi_i = diagAD ? atan2f(H, W) : atan2f(H, -W);
                    float theta = phi_m - phi_i;
                    float c_a=cosf(theta), s_a=sinf(theta);
                    float wx=c_a*W, wy=s_a*W;
                    float hx=-s_a*H, hy=c_a*H;
                    if (diagAD) {
                        Vx[1]=Vx[0]+wx; Vy[1]=Vy[0]+wy; Vx[2]=Vx[0]+hx; Vy[2]=Vy[0]+hy;
                    } else {
                        Vx[0]=Vx[1]-wx; Vy[0]=Vy[1]-wy; Vx[3]=Vx[1]+hx; Vy[3]=Vy[1]+hy;
                    }
                }
            }

            for (uint8_t i = 0; i < 4; i++) {
                positionXX[i] = fast_roundf(Vx[i]);
                positionYY[i] = fast_roundf(Vy[i]);
            }
        }
        
        // =========================================================================//
        // ========================= GESTIONE 3 PUNTI ==============================//
        // =========================================================================//
        else if (num_points_seen == 3) {
            int32_t d01_sq, d12_sq, d02_sq;
            
            dx = positionXX[0] - positionXX[1]; dy = positionYY[0] - positionYY[1];
            d01_sq = dx * dx + dy * dy;

            dx = positionXX[1] - positionXX[2]; dy = positionYY[1] - positionYY[2];
            d12_sq = dx * dx + dy * dy;

            dx = positionXX[0] - positionXX[2]; dy = positionYY[0] - positionYY[2];
            d02_sq = dx * dx + dy * dy;
            
            uint8_t a_idx, b_idx, c_idx;  

            if (d01_sq >= d12_sq && d01_sq >= d02_sq) {
                a_idx = 0; c_idx = 1; b_idx = 2;
            } else if (d12_sq >= d02_sq) {
                a_idx = 1; c_idx = 2; b_idx = 0;
            } else {
                a_idx = 0; c_idx = 2; b_idx = 1;
            }
            
            positionXX[3] = positionXX[a_idx] + positionXX[c_idx] - positionXX[b_idx];
            positionYY[3] = positionYY[a_idx] + positionYY[c_idx] - positionYY[b_idx];
        }

        // =========================================================================//
        // ======== ORDINAMENTO PER TUTTI I PUNTI E IDENTIFICAZIONE A,B,C,D ========//
        // =========================================================================//
        
        uint8_t orderX[4] = {0, 1, 2, 3};
        uint8_t orderY[4] = {0, 1, 2, 3};
        uint8_t a, b, c, d;

        {
            uint8_t tmp;
            if (positionXX[orderX[0]] > positionXX[orderX[1]]) { tmp = orderX[0]; orderX[0] = orderX[1]; orderX[1] = tmp; }
            if (positionXX[orderX[2]] > positionXX[orderX[3]]) { tmp = orderX[2]; orderX[2] = orderX[3]; orderX[3] = tmp; }
            if (positionXX[orderX[0]] > positionXX[orderX[2]]) { tmp = orderX[0]; orderX[0] = orderX[2]; orderX[2] = tmp; }
            if (positionXX[orderX[1]] > positionXX[orderX[3]]) { tmp = orderX[1]; orderX[1] = orderX[3]; orderX[3] = tmp; }
            if (positionXX[orderX[1]] > positionXX[orderX[2]]) { tmp = orderX[1]; orderX[1] = orderX[2]; orderX[2] = tmp; }
        }

        {
            uint8_t tmp;
            if (positionYY[orderY[0]] > positionYY[orderY[1]]) { tmp = orderY[0]; orderY[0] = orderY[1]; orderY[1] = tmp; }
            if (positionYY[orderY[2]] > positionYY[orderY[3]]) { tmp = orderY[2]; orderY[2] = orderY[3]; orderY[3] = tmp; }
            if (positionYY[orderY[0]] > positionYY[orderY[2]]) { tmp = orderY[0]; orderY[0] = orderY[2]; orderY[2] = tmp; }
            if (positionYY[orderY[1]] > positionYY[orderY[3]]) { tmp = orderY[1]; orderY[1] = orderY[3]; orderY[3] = tmp; }
            if (positionYY[orderY[1]] > positionYY[orderY[2]]) { tmp = orderY[1]; orderY[1] = orderY[2]; orderY[2] = tmp; }
        }

        int32_t dist_sq1, dist_sq2;
        dx = positionXX[orderY[0]] - positionXX[orderX[0]];
        dy = positionYY[orderY[0]] - positionYY[orderX[0]];
        dist_sq1 = (dx * dx) + (dy * dy);

        dx = positionXX[orderY[3]] - positionXX[orderX[0]];
        dy = positionYY[orderY[3]] - positionYY[orderX[0]];
        dist_sq2 = (dx * dx) + (dy * dy);
        
        const int CRITICAL_ZONE = (30 * CamToMouseMult); 

        if ((positionYY[orderY[1]] - positionYY[orderY[0]]) > CRITICAL_ZONE) {
            if (dist_sq1 < dist_sq2) {
                a = orderX[0]; d = orderX[3];
                if (orderX[1] == orderY[3]) { c = orderX[1]; b = orderX[2]; } else { b = orderX[1]; c = orderX[2]; }
            } else {
                c = orderX[0]; b = orderX[3];
                if (orderX[1] == orderY[3]) { d = orderX[1]; a = orderX[2]; } else { a = orderX[1]; d = orderX[2]; }
            }
        } else {
            a = orderY[0]; b = orderY[1]; c = orderY[2]; d = orderY[3];
        }

        {
            uint8_t aux_swap;
            if (positionYY[a] > positionYY[c]) { aux_swap = a; a = c; c = aux_swap; }
            if (positionYY[b] > positionYY[d]) { aux_swap = b; b = d; d = aux_swap; }
            if (positionXX[a] > positionXX[b]) { aux_swap = a; a = b; b = aux_swap; }
            if (positionXX[c] > positionXX[d]) { aux_swap = c; c = d; d = aux_swap; }
        }

        // =========================================================================//
        // ======== COSTRUZIONE MASCHERA: QUALI PUNTI ORDINATI SONO VERI? ==========//
        // =========================================================================//
        
        current_point_seen_mask = 0;
        
        auto is_real_point = [&](int px, int py) {
            for(uint8_t i=0; i<num_points_seen; i++) {
                if (abs(px - real_x[i]) <= 1 && abs(py - real_y[i]) <= 1) return true;
            }
            return false;
        };

        if (is_real_point(positionXX[a], positionYY[a])) current_point_seen_mask |= 0b1000;
        if (is_real_point(positionXX[b], positionYY[b])) current_point_seen_mask |= 0b0100;
        if (is_real_point(positionXX[c], positionYY[c])) current_point_seen_mask |= 0b0010;
        if (is_real_point(positionXX[d], positionYY[d])) current_point_seen_mask |= 0b0001;

        // =========================================================================//
        // ======== MOTORE UNIVERSALE DELLA MOLLA KINEMATICA =======================//
        // =========================================================================//

        int GeomX[4], GeomY[4];
        GeomX[A] = positionXX[a]; GeomY[A] = positionYY[a];
        GeomX[B] = positionXX[b]; GeomY[B] = positionYY[b];
        GeomX[C] = positionXX[c]; GeomY[C] = positionYY[c];
        GeomX[D] = positionXX[d]; GeomY[D] = positionYY[d];

        if (num_points_seen >= 2 && prev_num_points_seen >= 1) {
            
            int move_x = 0, move_y = 0;
            uint8_t stable_count = 0;
            for (uint8_t i = 0; i < 4; i++) {
                if ((prev_point_seen_mask & (1 << (3 - i))) && (current_point_seen_mask & (1 << (3 - i)))) {
                    move_x += (GeomX[i] - prev_GeomX[i]); 
                    move_y += (GeomY[i] - prev_GeomY[i]);
                    stable_count++;
                }
            }
            
            int avg_move_x = 0, avg_move_y = 0;
            if (stable_count > 0) {
                avg_move_x = move_x / stable_count; 
                avg_move_y = move_y / stable_count;
            } else if (prev_num_points_seen > 0) {
                avg_move_x = medianX - prev2_medianX;
                avg_move_y = medianY - prev2_medianY;
            }

            bool model_changed = (num_points_seen != prev_num_points_seen);
            for (uint8_t i = 0; i < 4; i++) {
                bool was_seen = (prev_point_seen_mask & (1 << (3 - i))) != 0;
                bool is_seen = (current_point_seen_mask & (1 << (3 - i))) != 0;

                if (is_seen != was_seen || (!is_seen && model_changed)) {
                    int predicted_x = FinalX[i] + avg_move_x;
                    int predicted_y = FinalY[i] + avg_move_y;
                    
                    offset_X[i] = (float)(predicted_x - GeomX[i]);
                    offset_Y[i] = (float)(predicted_y - GeomY[i]);
                }
            }

            int spostamento = abs(avg_move_x) + abs(avg_move_y);
            float consumo = (float)spostamento * COSTANTE_MOLLA; 
            for (uint8_t i = 0; i < 4; i++) {
                if (offset_X[i] > 0.0f) { offset_X[i] -= consumo; if (offset_X[i] < 0.0f) offset_X[i] = 0.0f; }
                else if (offset_X[i] < 0.0f) { offset_X[i] += consumo; if (offset_X[i] > 0.0f) offset_X[i] = 0.0f; }

                if (offset_Y[i] > 0.0f) { offset_Y[i] -= consumo; if (offset_Y[i] < 0.0f) offset_Y[i] = 0.0f; }
                else if (offset_Y[i] < 0.0f) { offset_Y[i] += consumo; if (offset_Y[i] > 0.0f) offset_Y[i] = 0.0f; }
            }
        } else {
            for (uint8_t i = 0; i < 4; i++) { offset_X[i] = 0.0f; offset_Y[i] = 0.0f; }
        }

        // =========================================================================//
        // ======== APPLICAZIONE FINALE E SALVATAGGIO STATO ========================//
        // =========================================================================//
        
        for (uint8_t i = 0; i < 4; i++) {
            FinalX[i] = GeomX[i] + fast_roundf(offset_X[i]);
            FinalY[i] = GeomY[i] + fast_roundf(offset_Y[i]);
            
            positionXX[(i==0)?a:(i==1)?b:(i==2)?c:d] = FinalX[i];
            positionYY[(i==0)?a:(i==1)?b:(i==2)?c:d] = FinalY[i];
            
            prev_GeomX[i] = GeomX[i];
            prev_GeomY[i] = GeomY[i];
        }

        prev2_medianX = medianX;
        prev2_medianY = medianY;

        medianX = (FinalX[A] + FinalX[B] + FinalX[C] + FinalX[D] + 2) / 4;
        medianY = (FinalY[A] + FinalY[B] + FinalY[C] + FinalY[D] + 2) / 4;

        height_left = hypotf((float)FinalY[A] - FinalY[C], (float)FinalX[A] - FinalX[C]); 
        height_right = hypotf((float)FinalY[B] - FinalY[D], (float)FinalX[B] - FinalX[D]); 
        width_top = hypotf((float)FinalY[A] - FinalY[B], (float)FinalX[A] - FinalX[B]); 
        width_bottom = hypotf((float)FinalY[C] - FinalY[D], (float)FinalX[C] - FinalX[D]); 
        height = (height_left + height_right) / 2.0f;
        width = (width_top + width_bottom) / 2.0f;
        
        if (num_points_seen == 4) {
            if (height > 1e-6f) { ideal_aspect_ratio = width / height; }

            shape_vec_AB_x = FinalX[1] - FinalX[0];
            shape_vec_AB_y = FinalY[1] - FinalY[0];
            shape_vec_AD_x = FinalX[3] - FinalX[0];
            shape_vec_AD_y = FinalY[3] - FinalY[0];
        }

        angle = (atan2f((float)FinalY[A] - FinalY[B], (float)FinalX[B] - FinalX[A]) + atan2f((float)FinalY[C] - FinalY[D], (float)FinalX[D] - FinalX[C])) / 2.0f;      

        is_tracking_stable = true;
        prev_point_seen_mask = current_point_seen_mask; 
    }
    else {
        is_tracking_stable = false;
        prev_point_seen_mask = 0;
    }
    prev_num_points_seen = num_points_seen;
}


#endif //USE_SQUARE_ADVANCED