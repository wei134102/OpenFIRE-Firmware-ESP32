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
 * da un progetto di:
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 */

#include <Arduino.h>
#include "OpenFIRE_Square_Advanced.h"


// === Implementazione del Costruttore ===
OpenFIRE_Square::OpenFIRE_Square() :
    medianY(MouseMaxY / 2),
    medianX(MouseMaxX / 2),
    xDistTop(0.0f), xDistBottom(0.0f), yDistLeft(0.0f), yDistRight(0.0f),
    angleTop(0.0f), angleBottom(0.0f), angleLeft(0.0f), angleRight(0.0f),
    angle(0.0f), height(0.0f), width(0.0f),
    initialization_complete_flag_(false), // Se questo flag serve per altre logiche, lascialo.
    seenFlags_cam(0),
    current_tracking_state_(TrackingQuality::AWAITING_INITIALIZATION)
{
    for (int i = 0; i < 4; ++i) {
        angleOffset[i] = 0.0f;
        see[i] = 0; 
        // FinalX/Y ora usano x_filt/y_filt che sono inizializzati nel .h
        // FinalX[i] = static_cast<int>(roundf(OpenFIRE_Square::x_filt[i])); // Lascia queste rimosse se non sono usate dal tuo altro Kalman
        // FinalY[i] = static_cast<int>(roundf(OpenFIRE_Square::y_filt[i])); // Lascia queste rimosse se non sono usate dal tuo altro Kalman
        current_input_for_kf_x_[i] = static_cast<float>(FinalX[i]); // Queste inizializzazioni potrebbero causare warning
        current_input_for_kf_y_[i] = static_cast<float>(FinalY[i]); // se FinalX/Y non sono inizializzati prima.
                                                                  // Valuta se sono effettivamente necessarie qui.
    }
    // Rimuovi completamente il blocco commentato di inizializzazioni qui sotto
    // che duplicava quelle statiche.
}

////////////////////////////////////////////////////////////////////////////////
#ifndef COMMENTO
// Definizione degli indici per chiarezza
#define A 0
#define B 1
#define C 2
#define D 3

void OpenFIRE_Square::begin(const int* px, const int* py, unsigned int seen) {

    // --- FASE 0: Inizializzazione e Controllo di Visibilità ---
    
    seenFlags = seen;

    // Il sistema richiede di vedere tutti e 4 i sensori almeno una volta per inizializzarsi.
    if (seenFlags == 0x0F) { // 0x0F in binario è 1111, tutti i sensori visti.
        start = 0xFF;
    } else if (!start) {
        return; // Esci se non siamo ancora stati inizializzati.
    }

    // --- FASE 1: Estrazione e Pre-elaborazione dei Punti Visti ---

    uint8_t num_points_seen = 0;
    int positionXX[4];
    int positionYY[4];

    // Estrae i punti visibili, li mette negli array di lavoro e applica la trasformazione.
    for (uint8_t i = 0; i < 4; ++i) {
        if ((seenFlags >> i) & 0x01) {
            positionXX[num_points_seen] = (CamMaxX - px[i]) << CamToMouseShift;
            positionYY[num_points_seen] = py[i] << CamToMouseShift;
            num_points_seen++;
        }
    }

    // Procede solo se abbiamo abbastanza dati per lavorare (almeno 2 punti).
    if (num_points_seen > 1) {
    
        // Dichiarazione di tutte le variabili locali usate nei vari blocchi logici.
        // Questo risolve gli errori di "scope" (variabile non dichiarata).
        int32_t dx, dy;
        uint8_t a, b, c, d;
        uint8_t P1, P2;

        if (num_points_seen == 2) {
            
            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // ==================== logica per 2 punti disordinati ============================================== //
            ////////////////////////////////////////////////////////////////////////////////////////////////////////       

            // 0) leggi i due punti
            int x1 = positionXX[0], y1 = positionYY[0];
            int x2 = positionXX[1], y2 = positionYY[1];

            // 1) nearest‐neighbor su FinalX/FinalY
            uint8_t idx1 = 0, idx2 = 1;
            unsigned best1 = UINT_MAX, best2 = UINT_MAX;
            for (int i = 0; i < 4; i++) {
                int dx = x1 - FinalX[i], dy = y1 - FinalY[i];
                unsigned d2 = dx*dx + dy*dy;
                if (d2 < best1) { best1 = d2; idx1 = i; }
            }
            for (int i = 0; i < 4; i++) {
                if (i == idx1) continue;
                int dx = x2 - FinalX[i], dy = y2 - FinalY[i];
                unsigned d2 = dx*dx + dy*dy;
                if (d2 < best2) { best2 = d2; idx2 = i; }
            }

            // 2) buffer per i vertici
            float Vx[4], Vy[4];
            Vx[idx1] = (float)x1;  Vy[idx1] = (float)y1;
            Vx[idx2] = (float)x2;  Vy[idx2] = (float)y2;

            // 3) riconosci lato puro
            bool top    = (idx1==0&&idx2==1)||(idx1==1&&idx2==0);
            bool bottom = (idx1==2&&idx2==3)||(idx1==3&&idx2==2);
            bool left   = (idx1==0&&idx2==2)||(idx1==2&&idx2==0);
            bool right  = (idx1==1&&idx2==3)||(idx1==3&&idx2==1);

            // 4) stima mancanti
            if (top || bottom) {
                // *** Inserisci qui: assicuro idx1 sia sempre il vertice più a sinistra
                if (Vx[idx1] > Vx[idx2]) {
                    // scambia gli indici
                    int tmp = idx1; idx1 = idx2; idx2 = tmp;
                }
                // lato orizzontale
                float dx = Vx[idx2] - Vx[idx1];
                float dy = Vy[idx2] - Vy[idx1];
                float base    = hypotf(dx, dy);
                float scale   = base / width;
                //width = base;
                float hScaled = height * scale;
                // normale al lato
                float ndx = -dy/base, ndy = dx/base;
                if (top) {
                    Vx[2] = Vx[0] + ndx*hScaled; Vy[2] = Vy[0] + ndy*hScaled;
                    Vx[3] = Vx[1] + ndx*hScaled; Vy[3] = Vy[1] + ndy*hScaled;
                } else {
                    Vx[0] = Vx[2] - ndx*hScaled; Vy[0] = Vy[2] - ndy*hScaled;
                    Vx[1] = Vx[3] - ndx*hScaled; Vy[1] = Vy[3] - ndy*hScaled;
                }
                //height = hScaled;
            }
            else if (left || right) {
                // *** Inserisci qui: assicuro idx1 sia sempre il vertice più in alto
                if (Vy[idx1] < Vy[idx2]) {
                    // scambia gli indici
                    int tmp = idx1; idx1 = idx2; idx2 = tmp;
                }
                // lato verticale
                float dx = Vx[idx2] - Vx[idx1];
                float dy = Vy[idx2] - Vy[idx1];
                float hgt     = hypotf(dx, dy);
                float scale   = hgt / height;
                //height = hgt;
                float wScaled = width * scale;
                // normale al lato
                float ndx = -dy/hgt, ndy = dx/hgt;
                if (left) {
                    Vx[1] = Vx[0] + ndx*wScaled; Vy[1] = Vy[0] + ndy*wScaled;
                    Vx[3] = Vx[2] + ndx*wScaled; Vy[3] = Vy[2] + ndy*wScaled;
                } else {
                    Vx[0] = Vx[1] - ndx*wScaled; Vy[0] = Vy[1] - ndy*wScaled;
                    Vx[2] = Vx[3] - ndx*wScaled; Vy[2] = Vy[3] - ndy*wScaled;
                }
                //width = wScaled;
            }
            else {
                // → caso: visti solo 2 sensori opposti (diagonale A–D o B–C)
                // A(0)──B(1)
                // │     │
                // C(2)──D(3)
                // → soli 2 sensori opposti: AD (0–3) o BC (1–2)
                bool diagAD = ((idx1==0 && idx2==3) || (idx1==3 && idx2==0));
                bool diagBC = ((idx1==1 && idx2==2) || (idx1==2 && idx2==1));
                if (!diagAD && !diagBC) {
                    // fallback parallelogramma
                    return;
                }

                // 1) P1 = punto più in alto, P2 = punto più in basso
                if (Vy[idx1] > Vy[idx2]) std::swap(idx1, idx2);
                float P1x = Vx[idx1], P1y = Vy[idx1];
                float P2x = Vx[idx2], P2y = Vy[idx2];

                // 2) lunghezza della diagonale misurata
                float dx = P2x - P1x;
                float dy = P2y - P1y;
                float L  = hypotf(dx, dy);

                // 3) lunghezza diagonale ideale e fattore di scala
                float L0 = hypotf((float)width, (float)height);
                float s  = L / L0;
                float W  = width  * s;
                float H  = height * s;

                // 4) angolo della diagonale misurata
                float phi = atan2f(dy, dx);

                // 5) angolo della diagonale ideale
                float phi0 = diagAD
                   ? atan2f((float)height,  (float)width)   // (+w,+h)
                   : atan2f((float)height, -(float)width);  // (-w,+h)

                // 6) rotazione da applicare
                float theta = phi - phi0;
                float c     = cosf(theta);
                float si    = sinf(theta);

                // 7) vettori lato lungo e lato corto
                float wvx =  c * W;
                float wvy =  si * W;
                float hvx = -si * H;
                float hvy =  c * H;

                // 8) ricostruzione esplicita dei vertici
                if (diagAD) {
                    // P1 = A, P2 = D
                    Vx[idx1] = P1x;             Vy[idx1] = P1y;           // A
                    Vx[idx2] = P2x;             Vy[idx2] = P2y;           // D
                    Vx[(idx1+1)&3] = P1x + wvx; Vy[(idx1+1)&3] = P1y + wvy; // B
                    Vx[(idx1+2)&3] = P2x - wvx; Vy[(idx1+2)&3] = P2y - wvy; // C
                } else {
                    // P1 = B, P2 = C
                    Vx[idx1] = P1x;             Vy[idx1] = P1y;           // B
                    Vx[idx2] = P2x;             Vy[idx2] = P2y;           // C
                    Vx[(idx1+3)&3] = P1x - wvx; Vy[(idx1+3)&3] = P1y - wvy; // A
                    Vx[(idx2+1)&3] = P2x + wvx; Vy[(idx2+1)&3] = P2y + wvy; // D
                }
            }

            // 5) copio in positionXX/YY
            for (int i = 0; i < 4; i++) {
                positionXX[i] = (int)roundf(Vx[i]);
                positionYY[i] = (int)roundf(Vy[i]);
            }

            // 6) correzione orientamento: assicuro A->B->D ordine giusto
            //    cross di AB x AD
            {
                float ax = Vx[0], ay = Vy[0];
                float bx = Vx[1], by = Vy[1];
                float dx = Vx[3], dy = Vy[3];
                float cross = (bx-ax)*(dy-ay) - (by-ay)*(dx-ax);
                // se cross < 0, significa che B e D sono “inversi”:
                if (cross < 0) {
                    // scambio C e D (slot 2 e 3)
                    int tx = positionXX[2], ty = positionYY[2];
                    positionXX[2] = positionXX[3];
                    positionYY[2] = positionYY[3];
                    positionXX[3] = tx;
                    positionYY[3] = ty;
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////  fine nuova versione di quando vengono visti soli due punti //////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////
        
        else if (num_points_seen == 3)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // ==================== logica per 3 punti disordinati ============================================== //
            ////////////////////////////////////////////////////////////////////////////////////////////////////////            
            
            // Se vediamo 3 punti, eseguiamo la stima semplice e veloce del quarto punto.
            int32_t d01_sq, d12_sq, d02_sq;
            
            dx = positionXX[0] - positionXX[1]; dy = positionYY[0] - positionYY[1];
            d01_sq = dx * dx + dy * dy;

            dx = positionXX[1] - positionXX[2]; dy = positionYY[1] - positionYY[2];
            d12_sq = dx * dx + dy * dy;

            dx = positionXX[0] - positionXX[2]; dy = positionYY[0] - positionYY[2];
            d02_sq = dx * dx + dy * dy;
            
            uint8_t a_idx;  // uno dei due punti del lato più lungo del triangolo che corrisponde ad una diagonale del rettangolo
            uint8_t b_idx;  // vertice tra lato lungo e lato corto del triangolo 
            uint8_t c_idx;  // uno dei due punti del lato più lungo del triangolo che corrisponde ad una diagonale del rettangolo

            if (d01_sq >= d12_sq && d01_sq >= d02_sq) {
                // d01 è la diagonale del rettangolo
                // b, il punto tra lato lungo e lato corto è per forza il punto 2 
                a_idx = 0; c_idx = 1; b_idx = 2;
            }
            else if (d12_sq >= d02_sq) {
                //d12 è la diagonale  del rettangolo
                // b, il punto tra lato lungo e lato corto è per forza il punto 0
                a_idx = 1; c_idx = 2; b_idx = 0;
            }
            else {
                //d02 è la diagonale  del rettangolo
                // b, il punto tra lato lungo e lato corto è per forza il punto 2 
                a_idx = 0; c_idx = 2; b_idx = 1;
            }
            
            // utilizzando le proprietà del Parallelogramma calcolo il 4 vertice mancante, 
            // senza comunque conoscerne l'esatta posizione logica, che verrà individuata successiva dalla gestione dei 4 punti
            positionXX[3] = positionXX[a_idx] + positionXX[c_idx] - positionXX[b_idx];
            positionYY[3] = positionYY[a_idx] + positionYY[c_idx] - positionYY[b_idx];
        }

        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ==================== logica per 4 punti disordinati ============================================== //
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        // --- FASE 3: Ordinamento Finale dei 4 Punti ---
        // Questa sezione viene eseguita sempre, per garantire che FinalX/Y
        // abbiano sempre un ordine coerente A,B,C,D.
        
        uint8_t orderX[4] = {0, 1, 2, 3};
        uint8_t orderY[4] = {0, 1, 2, 3};

        // ordinanmento più verboso, ma più performante
        // Ordinamento degli indici in base alla coordinata X (sorting network ottimizzato)
        {
            uint8_t tmp;
            if (positionXX[orderX[0]] > positionXX[orderX[1]]) { tmp = orderX[0]; orderX[0] = orderX[1]; orderX[1] = tmp; }
            if (positionXX[orderX[2]] > positionXX[orderX[3]]) { tmp = orderX[2]; orderX[2] = orderX[3]; orderX[3] = tmp; }
            if (positionXX[orderX[0]] > positionXX[orderX[2]]) { tmp = orderX[0]; orderX[0] = orderX[2]; orderX[2] = tmp; }
            if (positionXX[orderX[1]] > positionXX[orderX[3]]) { tmp = orderX[1]; orderX[1] = orderX[3]; orderX[3] = tmp; }
            if (positionXX[orderX[1]] > positionXX[orderX[2]]) { tmp = orderX[1]; orderX[1] = orderX[2]; orderX[2] = tmp; }
        }

        // ordinanmento più verboso, ma più performante
        // Ordinamento degli indici in base alla coordinata Y (sorting network ottimizzato)
        {
            uint8_t tmp;
            if (positionYY[orderY[0]] > positionYY[orderY[1]]) { tmp = orderY[0]; orderY[0] = orderY[1]; orderY[1] = tmp; }
            if (positionYY[orderY[2]] > positionYY[orderY[3]]) { tmp = orderY[2]; orderY[2] = orderY[3]; orderY[3] = tmp; }
            if (positionYY[orderY[0]] > positionYY[orderY[2]]) { tmp = orderY[0]; orderY[0] = orderY[2]; orderY[2] = tmp; }
            if (positionYY[orderY[1]] > positionYY[orderY[3]]) { tmp = orderY[1]; orderY[1] = orderY[3]; orderY[3] = tmp; }
            if (positionYY[orderY[1]] > positionYY[orderY[2]]) { tmp = orderY[1]; orderY[1] = orderY[2]; orderY[2] = tmp; }
        }

        // Assegnazione dei vertici A,B,C,D tramite euristiche
        int32_t dist_sq1, dist_sq2;

        dx = positionXX[orderY[0]] - positionXX[orderX[0]];
        dy = positionYY[orderY[0]] - positionYY[orderX[0]];
        dist_sq1 = (dx * dx) + (dy * dy);

        dx = positionXX[orderY[3]] - positionXX[orderX[0]];
        dy = positionYY[orderY[3]] - positionYY[orderX[0]];
        dist_sq2 = (dx * dx) + (dy * dy);
        
        const int CRITICAL_ZONE = (30 * CamToMouseMult); // 30 è un valore provato in tante situazioni e pare andare bene

        if ((positionYY[orderY[1]] - positionYY[orderY[0]]) > CRITICAL_ZONE)
        {
            // Caso Normale
            if (dist_sq1 < dist_sq2) {
                a = orderX[0]; d = orderX[3];
                if (orderX[1] == orderY[3]) { c = orderX[1]; b = orderX[2]; }
                else { b = orderX[1]; c = orderX[2]; }
            } else {
                c = orderX[0]; b = orderX[3];
                if (orderX[1] == orderY[3]) { d = orderX[1]; a = orderX[2]; }
                else { a = orderX[1]; d = orderX[2]; }
            }
        }
        else
        {
            // Caso Zona Critica (rettangolo quasi verticale)
            a = orderY[0]; b = orderY[1];
            c = orderY[2]; d = orderY[3];
        }

        // sarebbe sufficiente per la zona critica questo ulteriore controllo, ma poichè ha
        // un piccolo peso computazionale, lo facciamo sempre per casi limiti e come maggiore robustezza 
        // Correzione finale per garantire la convenzione (A=TL, B=TR, C=BL, D=BR)
        {
            uint8_t aux_swap;
            if (positionYY[a] > positionYY[c]) { aux_swap = a; a = c; c = aux_swap; }
            if (positionYY[b] > positionYY[d]) { aux_swap = b; b = d; d = aux_swap; }
            if (positionXX[a] > positionXX[b]) { aux_swap = a; a = b; b = aux_swap; }
            if (positionXX[c] > positionXX[d]) { aux_swap = c; c = d; d = aux_swap; }
        }

        // --- FASE 4: Assegnazione Finale e Calcoli Derivati ---
        FinalX[A] = positionXX[a]; FinalY[A] = positionYY[a];
        FinalX[B] = positionXX[b]; FinalY[B] = positionYY[b];
        FinalX[C] = positionXX[c]; FinalY[C] = positionYY[c];
        FinalX[D] = positionXX[d]; FinalY[D] = positionYY[d];
        
        //////////////////////////////////////////////////////////////////////////////////////////
        // ==== calcolo medianX, medianY, height, width, angle ... per compatibilità ========== //
        //////////////////////////////////////////////////////////////////////////////////////////
        
        // Calcolo del centroide con arrotondamento (il +2 è un trucco per l'arrotondamento all'intero più vicino)
        medianX = (FinalX[A] + FinalX[B] + FinalX[C] + FinalX[D] + 2) / 4;
        medianY = (FinalY[A] + FinalY[B] + FinalY[C] + FinalY[D] + 2) / 4;

        // Calcoli finali mantenuti per compatibilità
        float yDistLeft = hypotf((float)FinalY[A] - FinalY[C], (float)FinalX[A] - FinalX[C]); // lunghezza lato AC
        float yDistRight = hypotf((float)FinalY[B] - FinalY[D], (float)FinalX[B] - FinalX[D]); // lunghezza lato BD
        float xDistTop = hypotf((float)FinalY[A] - FinalY[B], (float)FinalX[A] - FinalX[B]); // lunghezza lato AB
        float xDistBottom = hypotf((float)FinalY[C] - FinalY[D], (float)FinalX[C] - FinalX[D]); // lunghezza lato CD
        height = (yDistLeft + yDistRight) / 2.0f;
        width = (xDistTop + xDistBottom) / 2.0f;
        
        angle = (atan2f((float)FinalY[A] - FinalY[B], (float)FinalX[B] - FinalX[A]) + atan2f((float)FinalY[C] - FinalY[D], (float)FinalX[D] - FinalX[C])) / 2.0f;
        
        //////////////////////////////////////////////////////////////////////////////////////////
        // ============================ filtro di Kalman se atttivo  ========================== //
        //////////////////////////////////////////////////////////////////////////////////////////

        #ifdef USE_KALMAN_FILTER
            for (int i = 0; i < 4; i++)
            {
                FinalX[i] = FinalX[i] + MouseMaxX;
                FinalY[i] = FinalY[i] + MouseMaxY;
            }

            //Kalman_filter_base();

            for (int i = 0; i < 4; i++)
            {
                FinalX[i] = FinalX[i] - MouseMaxX;
                FinalY[i] = FinalY[i] - MouseMaxY;
            }

        #endif //USE_KALMAN_FILTER
    }
}
#endif // COMMENTO
//////////////////////////////////////////////////////////////////////////////////////////


#ifdef COMMENTO
//////////////////////////////////////////////////////// nuovo non funzionante bene
void OpenFIRE_Square::Kalman_filter_base() {
    // --- Funzioni Helper integrate (non più esterne, inlined) ---
    auto local_constrain = [](float val, float min_val, float max_val) -> float {
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    };

    auto local_smoothstep = [&](float edge0, float edge1, float x) -> float {
        float t = (x - edge0) / (edge1 - edge0);
        if (t < 0.0f) t = 0.0f; 
        if (t > 1.0f) t = 1.0f;
        return t * t * (3.0f - 2.0f * t);
    };

    auto local_lerp = [](float a, float b, float t) -> float {
        return a + (b - a) * t;
    };

    // --- Parametri di tempo (dt è implicitamente 1.0f) ---
    const float dt = 1.0f;
    const float dt_squared = dt * dt;


    // Iteriamo su ciascuno dei 4 punti
    for (int point_idx = 0; point_idx < 4; ++point_idx) {
        // Le misurazioni grezze per il punto attuale sono lette direttamente da FinalX/FinalY.
        // Applicazione dello shift immediato per portare la misurazione nel sistema di coordinate del filtro [0, 3*MouseMaxX].
        float measured_x = static_cast<float>(FinalX[point_idx]) + base_kf_SHIFT_X;
        float measured_y = static_cast<float>(FinalY[point_idx]) + base_kf_SHIFT_Y;


        // --- 0. Inizializzazione completa del filtro (una volta per tutti i punti) ---
        if (!base_kf_is_initialized_all_points) {
            // Inizializza gli stati di posizione, velocità per questo punto, nello spazio operativo unico.
            base_kf_x_state[point_idx][0] = measured_x; // Posizione X (già shiftata)
            base_kf_x_state[point_idx][1] = 0.0f;       // Velocità X

            base_kf_y_state[point_idx][0] = measured_y; // Posizione Y (già shiftata)
            base_kf_y_state[point_idx][1] = 0.0f;       // Velocità Y

            // Inizializza la matrice di covarianza dell'errore (P) per questo punto.
            base_kf_p_x_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_x_01[point_idx] = 0.0f;
            base_kf_p_x_10[point_idx] = 0.0f; base_kf_p_x_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;

            base_kf_p_y_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_y_01[point_idx] = 0.0f;
            base_kf_p_y_10[point_idx] = 0.0f; base_kf_p_y_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;
            
            // Inizializza l'ultima posizione misurata (shifted) per il calcolo della velocità grezza
            base_kf_last_measured_x[point_idx] = measured_x; // Misura già shiftata
            base_kf_last_measured_y[point_idx] = measured_y; // Misura già shiftata
            base_kf_last_vx_raw[point_idx] = 0.0f; 
            base_kf_last_vy_raw[point_idx] = 0.0f; 

            // L'output iniziale è la misura grezza non filtrata (shiftata indietro per visualizzazione)
            FinalX[point_idx] = static_cast<int>(measured_x - base_kf_SHIFT_X); // Sottrai lo shift per l'output originale
            FinalY[point_idx] = static_cast<int>(measured_y - base_kf_SHIFT_Y); // Sottrai lo shift per l'output originale

            if (point_idx == 3) { 
                base_kf_is_initialized_all_points = true;
            }
            continue; 
        }

        // --- 1. Calcolo di Velocità e Accelerazione (esterne al filtro - da misurazioni grezze SHIFTED) ---
        float current_vx_raw = measured_x - base_kf_last_measured_x[point_idx];
        float current_vy_raw = measured_y - base_kf_last_measured_y[point_idx];
        
        float current_ax_raw = current_vx_raw - base_kf_last_vx_raw[point_idx];
        float current_ay_raw = current_vy_raw - base_kf_last_vy_raw[point_idx];

        float current_motion_magnitude = fabsf(current_vx_raw) + fabsf(current_vy_raw) + fabsf(current_ax_raw) + fabsf(current_ay_raw);


        // --- 2. Modulazione Dinamica di Q e R ---

        // Modulazione di Q (rumore del processo sul modello di velocità costante)
        float q_accel_factor = local_smoothstep(base_kf_ACCEL_Q_THRESH_START, base_kf_ACCEL_Q_THRESH_END, current_motion_magnitude);
        q_accel_factor = powf(q_accel_factor, base_kf_Q_ACCEL_EXPONENT);
        float Q_val_process = local_lerp(base_kf_Q_MIN_PROCESS, base_kf_Q_MAX_PROCESS, q_accel_factor);
        
        // Modulazione di R (rumore della misura)
        float r_accel_factor = local_smoothstep(base_kf_ACCEL_R_THRESH_START, base_kf_ACCEL_R_THRESH_END, current_motion_magnitude);
        r_accel_factor = powf(r_accel_factor, base_kf_R_ACCEL_EXPONENT);
        
        float R_val_base = local_lerp(base_kf_R_MAX, base_kf_R_MIN, r_accel_factor); 

        // GESTIONE ASIMMETRICA DI R AI BORDI (nello spazio operativo UNICO del filtro)
        // Calcolo della distanza normalizzata dal centro dell'UNICO spazio operativo ai bordi.
        float dist_from_center_x_norm = fabsf(base_kf_x_state[point_idx][0] - base_kf_X_CENTER) / base_kf_HALF_WIDTH; // NOME CAMBIATO
        float dist_from_center_y_norm = fabsf(base_kf_y_state[point_idx][0] - base_kf_Y_CENTER) / base_kf_HALF_HEIGHT; // NOME CAMBIATO
        
        // proximity_to_edges_factor aumenta quando il puntatore si avvicina ai bordi estremi dello spazio operativo.
        float proximity_to_x_edges_factor = local_smoothstep(base_kf_R_X_EDGE_SMOOTH_START, base_kf_R_X_EDGE_SMOOTH_END, dist_from_center_x_norm);
        float proximity_to_y_edges_factor = local_smoothstep(base_kf_R_Y_EDGE_SMOOTH_START, base_kf_R_Y_EDGE_SMOOTH_END, dist_from_center_y_norm);

        // Calcolo di R_x: lerp tra R_val_base e R_AT_X_EDGE, influenzato anche dall'asse Y
        float base_kf_R_val_x = local_lerp(R_val_base, base_kf_R_AT_X_EDGE, proximity_to_x_edges_factor);
        float effective_y_edge_influence_for_R_x = powf(proximity_to_y_edges_factor, base_kf_R_X_EDGE_EXPONENT);
        base_kf_R_val_x = local_lerp(base_kf_R_val_x, base_kf_R_AT_Y_EDGE_FOR_X, local_constrain(effective_y_edge_influence_for_R_x * base_kf_R_CROSS_AXIS_INFLUENCE_X, 0.0f, base_kf_MAX_EDGE_R_INFLUENCE_FACTOR));

        // Calcolo di R_y: lerp tra R_val_base e R_AT_Y_EDGE, influenzato anche dall'asse X
        float base_kf_R_val_y = local_lerp(R_val_base, base_kf_R_AT_Y_EDGE, proximity_to_y_edges_factor);
        float effective_x_edge_influence_for_R_y = powf(proximity_to_x_edges_factor, base_kf_R_Y_EDGE_EXPONENT);
        base_kf_R_val_y = local_lerp(base_kf_R_val_y, base_kf_R_AT_X_EDGE_FOR_Y, local_constrain(effective_x_edge_influence_for_R_y * base_kf_R_CROSS_AXIS_INFLUENCE_Y, 0.0f, base_kf_MAX_EDGE_R_INFLUENCE_FACTOR));
        
        // Assicurati che R_val_x e R_val_y rimangano all'interno di limiti ragionevoli.
        base_kf_R_val_x = local_constrain(base_kf_R_val_x, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f); 
        base_kf_R_val_y = local_constrain(base_kf_R_val_y, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f);


        // --- 3. Filtro di Kalman per l'asse X del punto attuale ---
        // (Il resto della funzione segue la logica standard del filtro di Kalman)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + base_kf_x_state[point_idx][1] * dt; 

        float q_term_00_p = Q_val_process * dt_squared / 4.0f; 
        float q_term_01_p = Q_val_process * dt / 2.0f;       
        float q_term_11_p = Q_val_process;                 

        float p_x_00_tmp = base_kf_p_x_00[point_idx] + dt * (base_kf_p_x_01[point_idx] + base_kf_p_x_10[point_idx]) + dt_squared * base_kf_p_x_11[point_idx];
        float p_x_01_tmp = base_kf_p_x_01[point_idx] + dt * base_kf_p_x_11[point_idx];
        float p_x_11_tmp = base_kf_p_x_11[point_idx];

        float base_kf_p_x_00_pred = p_x_00_tmp + q_term_00_p;
        float base_kf_p_x_01_pred = p_x_01_tmp + q_term_01_p;
        float base_kf_p_x_11_pred = p_x_11_tmp + q_term_11_p;
        
        float base_kf_p_x_10_pred = base_kf_p_x_01_pred;

        base_kf_p_x_00_pred = local_constrain(base_kf_p_x_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11_pred = local_constrain(base_kf_p_x_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        float S_x = base_kf_p_x_00_pred + base_kf_R_val_x; 
        float K_x_0 = base_kf_p_x_00_pred / S_x; 
        float K_x_1 = base_kf_p_x_10_pred / S_x; 


        float innovation_x = measured_x - base_kf_x_state[point_idx][0]; // Usa measured_x (già shiftato)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + K_x_0 * innovation_x;
        base_kf_x_state[point_idx][1] = base_kf_x_state[point_idx][1] + K_x_1 * innovation_x;

        base_kf_p_x_00[point_idx] = base_kf_p_x_00_pred - K_x_0 * base_kf_p_x_00_pred;
        base_kf_p_x_01[point_idx] = base_kf_p_x_01_pred - K_x_0 * base_kf_p_x_01_pred;

        base_kf_p_x_10[point_idx] = base_kf_p_x_10_pred - K_x_1 * base_kf_p_x_00_pred;
        base_kf_p_x_11[point_idx] = base_kf_p_x_11_pred - K_x_1 * base_kf_p_x_01_pred;
        
        base_kf_p_x_00[point_idx] = local_constrain(base_kf_p_x_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11[point_idx] = local_constrain(base_kf_p_x_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        // --- 4. Filtro di Kalman per l'asse Y del punto attuale (logica speculare all'asse X) ---
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + base_kf_y_state[point_idx][1] * dt;

        float p_y_00_tmp = base_kf_p_y_00[point_idx] + dt * (base_kf_p_y_10[point_idx] + base_kf_p_y_01[point_idx]) + dt_squared * base_kf_p_y_11[point_idx];
        float p_y_01_tmp = base_kf_p_y_01[point_idx] + dt * base_kf_p_y_11[point_idx];
        float p_y_11_tmp = base_kf_p_y_11[point_idx];

        float base_kf_p_y_00_pred = p_y_00_tmp + q_term_00_p;
        float base_kf_p_y_01_pred = p_y_01_tmp + q_term_01_p;
        float base_kf_p_y_11_pred = p_y_11_tmp + q_term_11_p;
        
        float base_kf_p_y_10_pred = base_kf_p_y_01_pred;

        base_kf_p_y_00_pred = local_constrain(base_kf_p_y_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11_pred = local_constrain(base_kf_p_y_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        float S_y = base_kf_p_y_00_pred + base_kf_R_val_y;
        float K_y_0 = base_kf_p_y_00_pred / S_y;
        float K_y_1 = base_kf_p_y_10_pred / S_y;

        float innovation_y = measured_y - base_kf_y_state[point_idx][0]; // Usa measured_y (già shiftato)
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + K_y_0 * innovation_y;
        base_kf_y_state[point_idx][1] = base_kf_y_state[point_idx][1] + K_y_1 * innovation_y;

        base_kf_p_y_00[point_idx] = local_constrain(base_kf_p_y_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_01[point_idx] = local_constrain(base_kf_p_y_01[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        base_kf_p_y_10[point_idx] = local_constrain(base_kf_p_y_10[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11[point_idx] = local_constrain(base_kf_p_y_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        
        // --- 5. Output del punto filtrato e Salvataggio Stati Precedenti ---
        // Applicare lo shift inverso per ottenere le coordinate originali per l'output.
        FinalX[point_idx] = static_cast<int>(base_kf_x_state[point_idx][0] - base_kf_SHIFT_X);
        FinalY[point_idx] = static_cast<int>(base_kf_y_state[point_idx][0] - base_kf_SHIFT_Y);

        // Salva le misurazioni SHIFTED attuali per il prossimo calcolo della velocità grezza.
        base_kf_last_measured_x[point_idx] = measured_x; 
        base_kf_last_measured_y[point_idx] = measured_y; 
        base_kf_last_vx_raw[point_idx] = current_vx_raw; 
        base_kf_last_vy_raw[point_idx] = current_vy_raw; 

        
        #define DEBUG_KF_POINTS
        #ifdef DEBUG_KF_POINTS
        if (point_idx == 0) { // Stampa solo per il primo punto
        Serial.print("Pt0 Meas=("); Serial.print(measured_x); Serial.print(", "); Serial.print(measured_y); Serial.print(")");
        Serial.print(" Filt=("); Serial.print(FinalX[point_idx]); Serial.print(", "); Serial.print(FinalY[point_idx]); Serial.print(")");
        Serial.print(" | AccelRaw=("); Serial.print(current_ax_raw); Serial.print(", "); Serial.print(current_ay_raw); Serial.print(")");
        Serial.print(" | MotionMag: "); Serial.print(current_motion_magnitude);
        Serial.print(" | Q_val: "); Serial.print(Q_val_process);
        Serial.print(" | R_base: "); Serial.print(R_val_base);
        Serial.print(" | R_x_final: "); Serial.print(base_kf_R_val_x);
        Serial.print(", R_y_final: "); Serial.print(base_kf_R_val_y);
        Serial.print(" | P_x_pos: "); Serial.print(base_kf_p_x_00[point_idx]);
        Serial.print(", P_y_pos: "); Serial.print(base_kf_p_y_00[point_idx]);
        Serial.println(); // Nuova riga
        }
        #endif


    } // Fine del loop for (point_idx)
}


//////////////////////////////////////////////////////////////////////////////////////
#endif // kalman  nuova implementazione base

////////////////////// inizio 17 ter /////////////////////////////////////////////////////////
#ifdef COMMENTO
////////////////////////////////////////////////////////////////////////////////////////
// --- Inizializzazioni delle variabili statiche della classe OpenFIRE_Square ---
// Queste righe DEVONO ESSERE PRESENTI UNA SOLA VOLTA in un singolo file .cpp.
// I valori iniziali qui non sono critici perché verranno sovrascritti nella Kalman_filter_base()
// alla prima chiamata per ogni punto.
float OpenFIRE_Square::base_kf_x_state[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
float OpenFIRE_Square::base_kf_y_state[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};

float OpenFIRE_Square::base_kf_p_x_00[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_x_01[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_p_x_10[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_x_11[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float OpenFIRE_Square::base_kf_p_y_00[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_y_01[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_p_y_10[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_y_11[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float OpenFIRE_Square::base_kf_last_measured_x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_measured_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_vx_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_vy_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool OpenFIRE_Square::base_kf_is_initialized_all_points = false;

// Le funzioni Helper constrain_custom, smoothstep_custom, lerp_custom NON DEVONO PIÙ ESSERE DEFINITE QUI O IN HEADER,
// poiché il loro codice è stato inlined direttamente nella funzione Kalman_filter_base().

/**
 * @brief Applica un filtro di Kalman di primo ordine (Posizione, Velocità)
 * a ciascuno dei 4 punti. Il dt è implicitamente 1.0f (pixel/frame).
 * La reattività è modulata da velocità/accelerazione calcolate esternamente.
 *
 * Questa funzione opera direttamente sui membri FinalX e FinalY (input e output).
 * È progettata per essere leggera, filtrare pesantemente a bassa velocità/fermo
 * e reagire rapidamente a movimenti bruschi della lightgun.
 * Si assume che FinalX/Y contengano sempre dati validi per tutti i 4 punti quando chiamata.
 */
void OpenFIRE_Square::Kalman_filter_base() {
    // dt è implicitamente 1.0f (un frame unitario)
    // float dt = 1.0f; // Rimosso perché non necessario per moltiplicazioni per 1.0f
    // float dt_squared = 1.0f; // Rimosso per la stessa ragione

    // DEFINIZIONE INLINE DI HELPER FUNCTIONS PER MASSIME PRESTAZIONI
    // Notare che queste versioni usano direttamente std::max/min per constrain.
    // Il compilatore le ottimizzerà in modo aggressivo.

    // inline_constrain: Clampa un valore tra min_val e max_val.
    auto inline_constrain = [](float val, float min_val, float max_val) -> float {
        return std::max(min_val, std::min(val, max_val));
    };

    // inline_smoothstep: Funzione smoothstep (curva sigmoide) inlined.
    auto inline_smoothstep = [&](float edge0, float edge1, float x) -> float {
        // x = constrain_custom((x - edge0) / (edge1 - edge0), 0.0f, 1.0f); // Originale
        x = inline_constrain((x - edge0) / (edge1 - edge0), 0.0f, 1.0f); // Inlined
        return x * x * (3.0f - 2.0f * x);
    };

    // inline_lerp: Interpolazione lineare inlined.
    auto inline_lerp = [](float a, float b, float t) -> float {
        return a + (b - a) * t;
    };


    // Iteriamo su ciascuno dei 4 punti
    for (int point_idx = 0; point_idx < 4; ++point_idx) {
        float measured_x = static_cast<float>(FinalX[point_idx]);
        float measured_y = static_cast<float>(FinalY[point_idx]);

        // --- 0. Inizializzazione completa del filtro (una volta per tutti i punti) ---
        if (!base_kf_is_initialized_all_points) {
            // Inizializza gli stati di posizione, velocità per questo punto.
            base_kf_x_state[point_idx][0] = measured_x; // Posizione X
            base_kf_x_state[point_idx][1] = 0.0f;       // Velocità X

            base_kf_y_state[point_idx][0] = measured_y; // Posizione Y
            base_kf_y_state[point_idx][1] = 0.0f;       // Velocità Y

            // Inizializza la matrice di covarianza dell'errore (P) per questo punto.
            base_kf_p_x_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_x_01[point_idx] = 0.0f;
            base_kf_p_x_10[point_idx] = 0.0f; base_kf_p_x_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;

            base_kf_p_y_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_y_01[point_idx] = 0.0f;
            base_kf_p_y_10[point_idx] = 0.0f; base_kf_p_y_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;
            
            // Inizializza l'ultima posizione misurata per il calcolo della velocità grezza
            base_kf_last_measured_x[point_idx] = measured_x;
            base_kf_last_measured_y[point_idx] = measured_y;
            base_kf_last_vx_raw[point_idx] = 0.0f; // Inizializza velocità calcolata
            base_kf_last_vy_raw[point_idx] = 0.0f; // Inizializza velocità calcolata

            // All'inizializzazione, l'output per questo punto è la sua misura iniziale.
            FinalX[point_idx] = static_cast<int>(measured_x);
            FinalY[point_idx] = static_cast<int>(measured_y);

            // Se questo è l'ultimo punto da inizializzare (solo una volta)
            if (point_idx == 3) { 
                base_kf_is_initialized_all_points = true;
            }
            continue; // Passa al prossimo punto per l'inizializzazione
        }

        // --- 1. Calcolo di Velocità e Accelerazione (esterne al filtro - da misurazioni grezze) ---
        // Queste stime sono usate per l'adattività di Q e R, non come stati del filtro.
        // Poiché dt = 1.0f, la velocità è semplicemente la differenza di posizione.
        float current_vx_raw = measured_x - base_kf_last_measured_x[point_idx];
        float current_vy_raw = measured_y - base_kf_last_measured_y[point_idx];
        
        // L'accelerazione è la differenza di velocità (anch'essa in pixel/frame^2)
        float current_ax_raw = current_vx_raw - base_kf_last_vx_raw[point_idx];
        float current_ay_raw = current_vy_raw - base_kf_last_vy_raw[point_idx];

        // Magnitudine del movimento per l'adattività di Q e R
        float current_motion_magnitude = fabsf(current_vx_raw) + fabsf(current_vy_raw) + fabsf(current_ax_raw) + fabsf(current_ay_raw);


        // --- 2. Modulazione Dinamica di Q e R ---

        // Modulazione di Q (rumore del processo sul modello di velocità costante)
        // Q aumenta con la magnitudine del movimento: meno fiducia nel modello (per movimenti bruschi)
        float q_accel_factor = inline_smoothstep(base_kf_ACCEL_Q_THRESH_START, base_kf_ACCEL_Q_THRESH_END, current_motion_magnitude);
        q_accel_factor = q_accel_factor * q_accel_factor; // powf(q_accel_factor, 2.0f) -> q_accel_factor * q_accel_factor
        float Q_val_process = inline_lerp(base_kf_Q_MIN_PROCESS, base_kf_Q_MAX_PROCESS, q_accel_factor);
        
        // Modulazione di R (rumore della misura)
        // R diminuisce con la magnitudine del movimento: più fiducia nella misura (per reattività)
        float r_accel_factor = inline_smoothstep(base_kf_ACCEL_R_THRESH_START, base_kf_ACCEL_R_THRESH_END, current_motion_magnitude);
        r_accel_factor = r_accel_factor * r_accel_factor * r_accel_factor; // powf(r_accel_factor, 3.0f) -> r_accel_factor * r_accel_factor * r_accel_factor
        
        // R base (influenzato solo dal movimento)
        float R_val_base = inline_lerp(base_kf_R_MAX, base_kf_R_MIN, r_accel_factor); 

        // Gestione asimmetrica di R ai bordi (per il traballamento)
        // Calcolo delle distanze normalizzate dalla posizione FILTRATA del punto attuale
        float dist_from_center_x_norm = fabsf(base_kf_x_state[point_idx][0] - base_kf_X_CENTER) / base_kf_HALF_WIDTH;
        float dist_from_center_y_norm = fabsf(base_kf_y_state[point_idx][0] - base_kf_Y_CENTER) / base_kf_HALF_HEIGHT;
        
        float proximity_to_x_edges_factor = inline_smoothstep(base_kf_R_X_EDGE_SMOOTH_START, base_kf_R_X_EDGE_SMOOTH_END, dist_from_center_x_norm);
        float proximity_to_y_edges_factor = inline_smoothstep(base_kf_R_Y_EDGE_SMOOTH_START, base_kf_R_Y_EDGE_SMOOTH_END, dist_from_center_y_norm);

        // R_x (influenzato da bordi X e Y)
        float base_kf_R_val_x = inline_lerp(R_val_base, base_kf_R_AT_X_EDGE, proximity_to_x_edges_factor);
        // powf(proximity_to_y_edges_factor, 4.0f) -> direct multiplication
        float effective_y_edge_influence_for_R_x = proximity_to_y_edges_factor * proximity_to_y_edges_factor * proximity_to_y_edges_factor * proximity_to_y_edges_factor;
        base_kf_R_val_x = inline_lerp(base_kf_R_val_x, base_kf_R_AT_Y_EDGE_FOR_X, effective_y_edge_influence_for_R_x * base_kf_R_CROSS_AXIS_INFLUENCE_X);

        // R_y (influenzato da bordi Y e X)
        float base_kf_R_val_y = inline_lerp(R_val_base, base_kf_R_AT_Y_EDGE, proximity_to_y_edges_factor);
        // powf(proximity_to_x_edges_factor, 4.0f) -> direct multiplication
        float effective_x_edge_influence_for_R_y = proximity_to_x_edges_factor * proximity_to_x_edges_factor * proximity_to_x_edges_factor * proximity_to_x_edges_factor;
        base_kf_R_val_y = inline_lerp(base_kf_R_val_y, base_kf_R_AT_X_EDGE_FOR_Y, effective_x_edge_influence_for_R_y * base_kf_R_CROSS_AXIS_INFLUENCE_Y);
        
        // Clamp finale di R per garantire valori validi e prevenire problemi numerici.
        base_kf_R_val_x = inline_constrain(base_kf_R_val_x, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f); 
        base_kf_R_val_y = inline_constrain(base_kf_R_val_y, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f);


        // --- 3. Filtro di Kalman per l'asse X del punto attuale ---
        // Predizione dello stato
        // x_pos = x_pos + x_vel * dt (dove dt=1)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + base_kf_x_state[point_idx][1]; // dt = 1.0f
        // x_vel = x_vel (modello di velocità costante, senza accelerazione esplicita nel modello A)

        // Predizione della covarianza P_x (P_pred = A * P * A_T + Q)
        // Q è un singolo valore scalare (Q_val_process) che modella il rumore sul processo (accelerazione non modellata/jerk implicito)
        // Matrice Q per modello di velocità costante: [dt^2/4 dt/2; dt/2 1] * Q_val_process
        float q_term_00_p = Q_val_process / 4.0f; // dt_squared = 1.0f
        float q_term_01_p = Q_val_process / 2.0f; // dt = 1.0f
        float q_term_11_p = Q_val_process;

        float p_x_00_tmp = base_kf_p_x_00[point_idx] + (base_kf_p_x_01[point_idx] + base_kf_p_x_10[point_idx]) + base_kf_p_x_11[point_idx]; // dt e dt_squared = 1.0f
        float p_x_01_tmp = base_kf_p_x_01[point_idx] + base_kf_p_x_11[point_idx]; // dt = 1.0f
        float p_x_11_tmp = base_kf_p_x_11[point_idx];

        // Aggiungiamo Q a P_tmp per ottenere P_pred
        float base_kf_p_x_00_pred = p_x_00_tmp + q_term_00_p;
        float base_kf_p_x_01_pred = p_x_01_tmp + q_term_01_p;
        float base_kf_p_x_11_pred = p_x_11_tmp + q_term_11_p;
        
        // Poiché P è simmetrica
        float base_kf_p_x_10_pred = base_kf_p_x_01_pred;

        // Clamp della covarianza predetta
        base_kf_p_x_00_pred = inline_constrain(base_kf_p_x_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11_pred = inline_constrain(base_kf_p_x_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        // Calcolo del Guadagno di Kalman (K_x)
        // H = [1 0] (misuriamo solo la posizione)
        float S_x = base_kf_p_x_00_pred + base_kf_R_val_x; // Covarianza del residuo
        float K_x_0 = base_kf_p_x_00_pred / S_x; // Guadagno per la posizione
        float K_x_1 = base_kf_p_x_10_pred / S_x; // Guadagno per la velocità


        // Aggiornamento dello stato (Correzione)
        float innovation_x = measured_x - base_kf_x_state[point_idx][0]; // Residuo (differenza tra misura e previsione)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + K_x_0 * innovation_x;
        base_kf_x_state[point_idx][1] = base_kf_x_state[point_idx][1] + K_x_1 * innovation_x;

        // Aggiornamento della covarianza P_x (P_k = (I - K * H) * P_pred)
        // H = [1 0], K = [K_x_0; K_x_1]
        base_kf_p_x_00[point_idx] = base_kf_p_x_00_pred - K_x_0 * base_kf_p_x_00_pred;
        base_kf_p_x_01[point_idx] = base_kf_p_x_01_pred - K_x_0 * base_kf_p_x_01_pred;

        base_kf_p_x_10[point_idx] = base_kf_p_x_10_pred - K_x_1 * base_kf_p_x_00_pred;
        base_kf_p_x_11[point_idx] = base_kf_p_x_11_pred - K_x_1 * base_kf_p_x_01_pred;
        
        // Clamp della covarianza post-aggiornamento
        base_kf_p_x_00[point_idx] = inline_constrain(base_kf_p_x_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11[point_idx] = inline_constrain(base_kf_p_x_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        // --- 4. Filtro di Kalman per l'asse Y del punto attuale (logica speculare all'asse X) ---
        // Predizione dello stato
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + base_kf_y_state[point_idx][1]; // dt = 1.0f

        // Predizione della covarianza P_y
        float p_y_00_tmp = base_kf_p_y_00[point_idx] + (base_kf_p_y_10[point_idx] + base_kf_p_y_01[point_idx]) + base_kf_p_y_11[point_idx]; // dt e dt_squared = 1.0f
        float p_y_01_tmp = base_kf_p_y_01[point_idx] + base_kf_p_y_11[point_idx]; // dt = 1.0f
        float p_y_11_tmp = base_kf_p_y_11[point_idx];

        float base_kf_p_y_00_pred = p_y_00_tmp + q_term_00_p;
        float base_kf_p_y_01_pred = p_y_01_tmp + q_term_01_p;
        float base_kf_p_y_11_pred = p_y_11_tmp + q_term_11_p;
        
        float base_kf_p_y_10_pred = base_kf_p_y_01_pred;

        base_kf_p_y_00_pred = inline_constrain(base_kf_p_y_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11_pred = inline_constrain(base_kf_p_y_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        // Calcolo del Guadagno di Kalman (K_y)
        float S_y = base_kf_p_y_00_pred + base_kf_R_val_y;
        float K_y_0 = base_kf_p_y_00_pred / S_y;
        float K_y_1 = base_kf_p_y_10_pred / S_y;

        // Aggiornamento dello stato (Correzione)
        float innovation_y = measured_y - base_kf_y_state[point_idx][0];
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + K_y_0 * innovation_y;
        base_kf_y_state[point_idx][1] = base_kf_y_state[point_idx][1] + K_y_1 * innovation_y;

        // Aggiornamento della covarianza P_y
        base_kf_p_y_00[point_idx] = base_kf_p_y_00_pred - K_y_0 * base_kf_p_y_00_pred;
        base_kf_p_y_01[point_idx] = base_kf_p_y_01_pred - K_y_0 * base_kf_p_y_01_pred;

        base_kf_p_y_10[point_idx] = base_kf_p_y_10_pred - K_y_1 * base_kf_p_y_00_pred;
        base_kf_p_y_11[point_idx] = base_kf_p_y_11_pred - K_y_1 * base_kf_p_y_01_pred;
        
        base_kf_p_y_00[point_idx] = inline_constrain(base_kf_p_y_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11[point_idx] = inline_constrain(base_kf_p_y_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        // --- 5. Output del punto filtrato e Salvataggio Stati Precedenti ---
        FinalX[point_idx] = static_cast<int>(base_kf_x_state[point_idx][0]);
        FinalY[point_idx] = static_cast<int>(base_kf_y_state[point_idx][0]);

        // Salva le misurazioni grezze attuali per il prossimo calcolo della velocità grezza.
        base_kf_last_measured_x[point_idx] = measured_x;
        base_kf_last_measured_y[point_idx] = measured_y;
        base_kf_last_vx_raw[point_idx] = current_vx_raw; // Salva la velocità grezza
        base_kf_last_vy_raw[point_idx] = current_vy_raw; // Salva la velocità grezza

// Il blocco DEBUG_KF_POINTS è stato mantenuto per riferimento, ma è consigliabile disabilitarlo in produzione
// commentando la riga #define DEBUG_KF_POINTS o rimuovendola.
#ifdef DEBUG_KF_POINTS
    if (point_idx == 0) { // Stampa solo per il primo punto
Serial.print("Pt0 Meas=("); Serial.print(measured_x); Serial.print(", "); Serial.print(measured_y); Serial.print(")");
        Serial.print(" Filt=("); Serial.print(FinalX[point_idx]); Serial.print(", "); Serial.print(FinalY[point_idx]); Serial.print(")");
        Serial.print(" | AccelRaw=("); Serial.print(current_ax_raw); Serial.print(", "); Serial.print(current_ay_raw); Serial.print(")");
        Serial.print(" | MotionMag: "); Serial.print(current_motion_magnitude);
        Serial.print(" | Q_val: "); Serial.print(Q_val_process);
        Serial.print(" | R_base: "); Serial.print(R_val_base);
        Serial.print(" | R_x_final: "); Serial.print(base_kf_R_val_x);
        Serial.print(", R_y_final: "); Serial.print(base_kf_R_val_y);
        Serial.print(" | P_x_pos: "); Serial.print(base_kf_p_x_00[point_idx]);
        Serial.print(", P_y_pos: "); Serial.print(base_kf_p_y_00[point_idx]);
        Serial.println(); // Nuova riga
    }
#endif


    } // Fine del loop for (point_idx)
}


#endif
////////////// fine 17 ter //////////////////////////////////////////////////////////



////////////////////// inizio 17bis ////////////////////////////////////////////////////////////////
#ifndef COMMENTO // versione debug 17 la migliore

////////////////////////////////////////////////////////////////////////////////////////
// --- Inizializzazioni delle variabili statiche della classe OpenFIRE_Square ---
// Queste righe DEVONO ESSERE PRESENTI UNA SOLA VOLTA in un singolo file .cpp.
// I valori iniziali qui non sono critici perché verranno sovrascritti nella Kalman_filter_base()
// alla prima chiamata per ogni punto.
float OpenFIRE_Square::base_kf_x_state[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
float OpenFIRE_Square::base_kf_y_state[4][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};

float OpenFIRE_Square::base_kf_p_x_00[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_x_01[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_p_x_10[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_x_11[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float OpenFIRE_Square::base_kf_p_y_00[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_y_01[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_p_y_10[4] = {0.0f, 0.0f, 0.0f, 0.0f}; float OpenFIRE_Square::base_kf_p_y_11[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float OpenFIRE_Square::base_kf_last_measured_x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_measured_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_vx_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float OpenFIRE_Square::base_kf_last_vy_raw[4] = {0.0f, 0.0f, 0.0f, 0.0f};

bool OpenFIRE_Square::base_kf_is_initialized_all_points = false;



// Funzioni Helper (assicurati che siano definite qui o in un header incluso)
float constrain_custom(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(val, max_val));
}

float smoothstep_custom(float edge0, float edge1, float x) {
    x = constrain_custom((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

float lerp_custom(float a, float b, float t) {
    return a + (b - a) * t;
}

/**
 * @brief Applica un filtro di Kalman di primo ordine (Posizione, Velocità)
 * a ciascuno dei 4 punti. Il dt è implicitamente 1.0f (pixel/frame).
 * La reattività è modulata da velocità/accelerazione calcolate esternamente.
 *
 * Questa funzione opera direttamente sui membri FinalX e FinalY (input e output).
 * È progettata per essere leggera, filtrare pesantemente a bassa velocità/fermo
 * e reagire rapidamente a movimenti bruschi della lightgun.
 * Si assume che FinalX/Y contengano sempre dati validi per tutti i 4 punti quando chiamata.
 */
void OpenFIRE_Square::Kalman_filter_base() {
    // dt è implicitamente 1.0f (un frame unitario)
    float dt = 1.0f;
    float dt_squared = dt * dt;


    // Iteriamo su ciascuno dei 4 punti
    for (int point_idx = 0; point_idx < 4; ++point_idx) {
        float measured_x = static_cast<float>(FinalX[point_idx]);
        float measured_y = static_cast<float>(FinalY[point_idx]);

        // --- 0. Inizializzazione completa del filtro (una volta per tutti i punti) ---
        if (!base_kf_is_initialized_all_points) {
            // Inizializza gli stati di posizione, velocità per questo punto.
            base_kf_x_state[point_idx][0] = measured_x; // Posizione X
            base_kf_x_state[point_idx][1] = 0.0f;       // Velocità X

            base_kf_y_state[point_idx][0] = measured_y; // Posizione Y
            base_kf_y_state[point_idx][1] = 0.0f;       // Velocità Y

            // Inizializza la matrice di covarianza dell'errore (P) per questo punto.
            base_kf_p_x_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_x_01[point_idx] = 0.0f;
            base_kf_p_x_10[point_idx] = 0.0f; base_kf_p_x_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;

            base_kf_p_y_00[point_idx] = base_kf_INITIAL_P_POS_VALUE; base_kf_p_y_01[point_idx] = 0.0f;
            base_kf_p_y_10[point_idx] = 0.0f; base_kf_p_y_11[point_idx] = base_kf_INITIAL_P_VEL_VALUE;
            
            // Inizializza l'ultima posizione misurata per il calcolo della velocità grezza
            base_kf_last_measured_x[point_idx] = measured_x;
            base_kf_last_measured_y[point_idx] = measured_y;
            base_kf_last_vx_raw[point_idx] = 0.0f; // Inizializza velocità calcolata
            base_kf_last_vy_raw[point_idx] = 0.0f; // Inizializza velocità calcolata

            // All'inizializzazione, l'output per questo punto è la sua misura iniziale.
            FinalX[point_idx] = static_cast<int>(measured_x);
            FinalY[point_idx] = static_cast<int>(measured_y);

            // Se questo è l'ultimo punto da inizializzare (solo una volta)
            if (point_idx == 3) { 
                base_kf_is_initialized_all_points = true;
            }
            continue; // Passa al prossimo punto per l'inizializzazione
        }

        // --- 1. Calcolo di Velocità e Accelerazione (esterne al filtro - da misurazioni grezze) ---
        // Queste stime sono usate per l'adattività di Q e R, non come stati del filtro.
        // Poiché dt = 1.0f, la velocità è semplicemente la differenza di posizione.
        float current_vx_raw = measured_x - base_kf_last_measured_x[point_idx];
        float current_vy_raw = measured_y - base_kf_last_measured_y[point_idx];
        
        // L'accelerazione è la differenza di velocità (anch'essa in pixel/frame^2)
        float current_ax_raw = current_vx_raw - base_kf_last_vx_raw[point_idx];
        float current_ay_raw = current_vy_raw - base_kf_last_vy_raw[point_idx];

        // Magnitudine del movimento per l'adattività di Q e R
        float current_motion_magnitude = fabsf(current_vx_raw) + fabsf(current_vy_raw) + fabsf(current_ax_raw) + fabsf(current_ay_raw);


        // --- 2. Modulazione Dinamica di Q e R ---

        // Modulazione di Q (rumore del processo sul modello di velocità costante)
        // Q aumenta con la magnitudine del movimento: meno fiducia nel modello (per movimenti bruschi)
        float q_accel_factor = smoothstep_custom(base_kf_ACCEL_Q_THRESH_START, base_kf_ACCEL_Q_THRESH_END, current_motion_magnitude);
        q_accel_factor = powf(q_accel_factor, base_kf_Q_ACCEL_EXPONENT);
        float Q_val_process = lerp_custom(base_kf_Q_MIN_PROCESS, base_kf_Q_MAX_PROCESS, q_accel_factor);
        
        // Modulazione di R (rumore della misura)
        // R diminuisce con la magnitudine del movimento: più fiducia nella misura (per reattività)
        float r_accel_factor = smoothstep_custom(base_kf_ACCEL_R_THRESH_START, base_kf_ACCEL_R_THRESH_END, current_motion_magnitude);
        r_accel_factor = powf(r_accel_factor, base_kf_R_ACCEL_EXPONENT);
        
        // R base (influenzato solo dal movimento)
        // CORREZIONE CRUCIALE: Assicurarsi che lerp vada da R_MAX (ALTO per LENTO) a R_MIN (BASSO per VELOCE)
        float R_val_base = lerp_custom(base_kf_R_MAX, base_kf_R_MIN, r_accel_factor); // <<<< CORREZIONE QUI
                                                                                    // r_accel_factor va da 0 (lento) a 1 (veloce)

        // Gestione asimmetrica di R ai bordi (per il traballamento)
        // Calcolo delle distanze normalizzate dalla posizione FILTRATA del punto attuale
        float dist_from_center_x_norm = fabsf(base_kf_x_state[point_idx][0] - base_kf_X_CENTER) / base_kf_HALF_WIDTH;
        float dist_from_center_y_norm = fabsf(base_kf_y_state[point_idx][0] - base_kf_Y_CENTER) / base_kf_HALF_HEIGHT;
        
        float proximity_to_x_edges_factor = smoothstep_custom(base_kf_R_X_EDGE_SMOOTH_START, base_kf_R_X_EDGE_SMOOTH_END, dist_from_center_x_norm);
        float proximity_to_y_edges_factor = smoothstep_custom(base_kf_R_Y_EDGE_SMOOTH_START, base_kf_R_Y_EDGE_SMOOTH_END, dist_from_center_y_norm);

        // R_x (influenzato da bordi X e Y)
        float base_kf_R_val_x = lerp_custom(R_val_base, base_kf_R_AT_X_EDGE, proximity_to_x_edges_factor);
        float effective_y_edge_influence_for_R_x = powf(proximity_to_y_edges_factor, base_kf_R_X_EDGE_EXPONENT);
        base_kf_R_val_x = lerp_custom(base_kf_R_val_x, base_kf_R_AT_Y_EDGE_FOR_X, effective_y_edge_influence_for_R_x * base_kf_R_CROSS_AXIS_INFLUENCE_X);

        // R_y (influenzato da bordi Y e X)
        float base_kf_R_val_y = lerp_custom(R_val_base, base_kf_R_AT_Y_EDGE, proximity_to_y_edges_factor);
        float effective_x_edge_influence_for_R_y = powf(proximity_to_x_edges_factor, base_kf_R_Y_EDGE_EXPONENT);
        base_kf_R_val_y = lerp_custom(base_kf_R_val_y, base_kf_R_AT_X_EDGE_FOR_Y, effective_x_edge_influence_for_R_y * base_kf_R_CROSS_AXIS_INFLUENCE_Y);
        
        // Clamp finale di R per garantire valori validi e prevenire problemi numerici.
        base_kf_R_val_x = constrain_custom(base_kf_R_val_x, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f); 
        base_kf_R_val_y = constrain_custom(base_kf_R_val_y, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f);


        // --- 3. Filtro di Kalman per l'asse X del punto attuale ---
        // Predizione dello stato
        // x_pos = x_pos + x_vel * dt (dove dt=1)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + base_kf_x_state[point_idx][1] * dt; 
        // x_vel = x_vel (modello di velocità costante, senza accelerazione esplicita nel modello A)

        // Predizione della covarianza P_x (P_pred = A * P * A_T + Q)
        // Q è un singolo valore scalare (Q_val_process) che modella il rumore sul processo (accelerazione non modellata/jerk implicito)
        // Matrice Q per modello di velocità costante: [dt^2/4 dt/2; dt/2 1] * Q_val_process
        float q_term_00_p = Q_val_process * dt_squared / 4.0f; // Q_val_process * dt*dt / 4
        float q_term_01_p = Q_val_process * dt / 2.0f;         // Q_val_process * dt / 2
        float q_term_11_p = Q_val_process;                     // Q_val_process * 1

        float p_x_00_tmp = base_kf_p_x_00[point_idx] + dt * (base_kf_p_x_01[point_idx] + base_kf_p_x_10[point_idx]) + dt_squared * base_kf_p_x_11[point_idx];
        float p_x_01_tmp = base_kf_p_x_01[point_idx] + dt * base_kf_p_x_11[point_idx];
        float p_x_11_tmp = base_kf_p_x_11[point_idx];

        // Aggiungiamo Q a P_tmp per ottenere P_pred
        float base_kf_p_x_00_pred = p_x_00_tmp + q_term_00_p;
        float base_kf_p_x_01_pred = p_x_01_tmp + q_term_01_p;
        float base_kf_p_x_11_pred = p_x_11_tmp + q_term_11_p;
        
        // Poiché P è simmetrica
        float base_kf_p_x_10_pred = base_kf_p_x_01_pred;

        // Clamp della covarianza predetta
        base_kf_p_x_00_pred = constrain_custom(base_kf_p_x_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11_pred = constrain_custom(base_kf_p_x_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        // Calcolo del Guadagno di Kalman (K_x)
        // H = [1 0] (misuriamo solo la posizione)
        float S_x = base_kf_p_x_00_pred + base_kf_R_val_x; // Covarianza del residuo
        float K_x_0 = base_kf_p_x_00_pred / S_x; // Guadagno per la posizione
        float K_x_1 = base_kf_p_x_10_pred / S_x; // Guadagno per la velocità


        // Aggiornamento dello stato (Correzione)
        float innovation_x = measured_x - base_kf_x_state[point_idx][0]; // Residuo (differenza tra misura e previsione)
        base_kf_x_state[point_idx][0] = base_kf_x_state[point_idx][0] + K_x_0 * innovation_x;
        base_kf_x_state[point_idx][1] = base_kf_x_state[point_idx][1] + K_x_1 * innovation_x;

        // Aggiornamento della covarianza P_x (P_k = (I - K * H) * P_pred)
        // H = [1 0], K = [K_x_0; K_x_1]
        base_kf_p_x_00[point_idx] = base_kf_p_x_00_pred - K_x_0 * base_kf_p_x_00_pred;
        base_kf_p_x_01[point_idx] = base_kf_p_x_01_pred - K_x_0 * base_kf_p_x_01_pred;

        base_kf_p_x_10[point_idx] = base_kf_p_x_10_pred - K_x_1 * base_kf_p_x_00_pred;
        base_kf_p_x_11[point_idx] = base_kf_p_x_11_pred - K_x_1 * base_kf_p_x_01_pred;
        
        // Clamp della covarianza post-aggiornamento
        base_kf_p_x_00[point_idx] = constrain_custom(base_kf_p_x_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_x_11[point_idx] = constrain_custom(base_kf_p_x_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);


        // --- 4. Filtro di Kalman per l'asse Y del punto attuale (logica speculare all'asse X) ---
        // Predizione dello stato
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + base_kf_y_state[point_idx][1] * dt;

        // Predizione della covarianza P_y
        float p_y_00_tmp = base_kf_p_y_00[point_idx] + dt * (base_kf_p_y_01[point_idx] + base_kf_p_y_10[point_idx]) + dt_squared * base_kf_p_y_11[point_idx];
        float p_y_01_tmp = base_kf_p_y_01[point_idx] + dt * base_kf_p_y_11[point_idx];
        float p_y_11_tmp = base_kf_p_y_11[point_idx];

        float base_kf_p_y_00_pred = p_y_00_tmp + q_term_00_p;
        float base_kf_p_y_01_pred = p_y_01_tmp + q_term_01_p;
        float base_kf_p_y_11_pred = p_y_11_tmp + q_term_11_p;
        
        float base_kf_p_y_10_pred = base_kf_p_y_01_pred;

        base_kf_p_y_00_pred = constrain_custom(base_kf_p_y_00_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11_pred = constrain_custom(base_kf_p_y_11_pred, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        // Calcolo del Guadagno di Kalman (K_y)
        float S_y = base_kf_p_y_00_pred + base_kf_R_val_y;
        float K_y_0 = base_kf_p_y_00_pred / S_y;
        float K_y_1 = base_kf_p_y_10_pred / S_y;

        // Aggiornamento dello stato (Correzione)
        float innovation_y = measured_y - base_kf_y_state[point_idx][0];
        base_kf_y_state[point_idx][0] = base_kf_y_state[point_idx][0] + K_y_0 * innovation_y;
        base_kf_y_state[point_idx][1] = base_kf_y_state[point_idx][1] + K_y_1 * innovation_y;

        // Aggiornamento della covarianza P_y
        base_kf_p_y_00[point_idx] = base_kf_p_y_00_pred - K_y_0 * base_kf_p_y_00_pred;
        base_kf_p_y_01[point_idx] = base_kf_p_y_01_pred - K_y_0 * base_kf_p_y_01_pred;

        base_kf_p_y_10[point_idx] = base_kf_p_y_10_pred - K_y_1 * base_kf_p_y_00_pred;
        base_kf_p_y_11[point_idx] = base_kf_p_y_11_pred - K_y_1 * base_kf_p_y_01_pred;
        
        base_kf_p_y_00[point_idx] = constrain_custom(base_kf_p_y_00[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        base_kf_p_y_11[point_idx] = constrain_custom(base_kf_p_y_11[point_idx], base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);

        // --- 5. Output del punto filtrato e Salvataggio Stati Precedenti ---
        FinalX[point_idx] = static_cast<int>(base_kf_x_state[point_idx][0]);
        FinalY[point_idx] = static_cast<int>(base_kf_y_state[point_idx][0]);

        // Salva le misurazioni grezze attuali per il prossimo calcolo della velocità grezza.
        base_kf_last_measured_x[point_idx] = measured_x;
        base_kf_last_measured_y[point_idx] = measured_y;
        base_kf_last_vx_raw[point_idx] = current_vx_raw; // Salva la velocità grezza
        base_kf_last_vy_raw[point_idx] = current_vy_raw; // Salva la velocità grezza

//#define DEBUG_KF_POINTS
#ifdef DEBUG_KF_POINTS
    if (point_idx == 0) { // Stampa solo per il primo punto
Serial.print("Pt0 Meas=("); Serial.print(measured_x); Serial.print(", "); Serial.print(measured_y); Serial.print(")");
        Serial.print(" Filt=("); Serial.print(FinalX[point_idx]); Serial.print(", "); Serial.print(FinalY[point_idx]); Serial.print(")");
        Serial.print(" | AccelRaw=("); Serial.print(current_ax_raw); Serial.print(", "); Serial.print(current_ay_raw); Serial.print(")");
        Serial.print(" | MotionMag: "); Serial.print(current_motion_magnitude);
        Serial.print(" | Q_val: "); Serial.print(Q_val_process);
        Serial.print(" | R_base: "); Serial.print(R_val_base);
        Serial.print(" | R_x_final: "); Serial.print(base_kf_R_val_x);
        Serial.print(", R_y_final: "); Serial.print(base_kf_R_val_y);
        Serial.print(" | P_x_pos: "); Serial.print(base_kf_p_x_00[point_idx]);
        Serial.print(", P_y_pos: "); Serial.print(base_kf_p_y_00[point_idx]);
        Serial.println(); // Nuova riga
    }
    #endif




    } // Fine del loop for (point_idx)
}
#endif // COMMENTO - version 17 la migliore
////////////////////////////////////// fine 17 bis ////////////////////////////////////////////



#endif //USE_SQUARE_ADVANCED