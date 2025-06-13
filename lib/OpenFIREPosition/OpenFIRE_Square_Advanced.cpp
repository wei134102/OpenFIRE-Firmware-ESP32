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
                float L  = hypotf(dx, dy); // float L = sqrtf(dx*dx + dy*dy);

                if (L < 1e-5f) {
                    //check di sicurezza
                    // se si verifica, abbandono l’algoritmo o rimango col frame precedente
                    return;
                }

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

            Kalman_filter_base();

            for (int i = 0; i < 4; i++)
            {
                FinalX[i] = FinalX[i] - MouseMaxX;
                FinalY[i] = FinalY[i] - MouseMaxY;
            }

        #endif //USE_KALMAN_FILTER
        

    }
}

//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////// Versione 18 – Inizializzazioni ////////////////////////////////////////

// Stato filtro (posizione e velocità) per ciascun punto
float OpenFIRE_Square::base_kf_x_state[4][2] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};
float OpenFIRE_Square::base_kf_y_state[4][2] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};

// Covarianze P per X
float OpenFIRE_Square::base_kf_p_x_00[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_x_01[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_x_10[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_x_11[4] = {0,0,0,0};

// Covarianze P per Y
float OpenFIRE_Square::base_kf_p_y_00[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_y_01[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_y_10[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_p_y_11[4] = {0,0,0,0};

// Ultime misure grezze per calcolare velocità e accelerazione
float OpenFIRE_Square::base_kf_last_measured_x[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_last_measured_y[4] = {0,0,0,0};
float OpenFIRE_Square::base_kf_last_vx_raw[4]       = {0,0,0,0};
float OpenFIRE_Square::base_kf_last_vy_raw[4]       = {0,0,0,0};

// Flag di inizializzazione (una tantum)
bool OpenFIRE_Square::base_kf_is_initialized_all_points = false;

//////////////////////////////////////// Versione 18 – Funzione ////////////////////////////////////////

static inline float cns(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline float ss(float e0, float e1, float x) {
    x = cns((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
static inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

void OpenFIRE_Square::Kalman_filter_base() {
    const float inv_halfW = 1.0f / base_kf_HALF_WIDTH;
    const float inv_halfH = 1.0f / base_kf_HALF_HEIGHT;
    const float inv_dQ    = 1.0f / (base_kf_ACCEL_Q_THRESH_END   - base_kf_ACCEL_Q_THRESH_START);
    const float inv_dR    = 1.0f / (base_kf_ACCEL_R_THRESH_END   - base_kf_ACCEL_R_THRESH_START);

    for (int i = 0; i < 4; ++i) {
        float mx = (float)FinalX[i];
        float my = (float)FinalY[i];

        // 0) Inizializzazione
        if (!base_kf_is_initialized_all_points) {
            base_kf_x_state[i][0] = mx;  base_kf_x_state[i][1] = 0.0f;
            base_kf_y_state[i][0] = my;  base_kf_y_state[i][1] = 0.0f;
            base_kf_p_x_00[i]     = base_kf_INITIAL_P_POS_VALUE;
            base_kf_p_x_11[i]     = base_kf_INITIAL_P_VEL_VALUE;
            base_kf_p_y_00[i]     = base_kf_INITIAL_P_POS_VALUE;
            base_kf_p_y_11[i]     = base_kf_INITIAL_P_VEL_VALUE;
            base_kf_last_measured_x[i] = mx;
            base_kf_last_measured_y[i] = my;
            base_kf_last_vx_raw[i]     = 0.0f;
            base_kf_last_vy_raw[i]     = 0.0f;
            FinalX[i] = (int)mx;
            FinalY[i] = (int)my;
            if (i == 3) base_kf_is_initialized_all_points = true;
            continue;
        }

        // 1) Velocità & accelerazione grezze
        float vx = mx - base_kf_last_measured_x[i];
        float vy = my - base_kf_last_measured_y[i];
        float ax = vx - base_kf_last_vx_raw[i];
        float ay = vy - base_kf_last_vy_raw[i];
        float mag = fabsf(vx) + fabsf(vy) + fabsf(ax) + fabsf(ay);

        // 2) Modulazione Q
        float tQ = cns((mag - base_kf_ACCEL_Q_THRESH_START) * inv_dQ, 0.0f, 1.0f);
        float Q  = lerp(base_kf_Q_MIN_PROCESS, base_kf_Q_MAX_PROCESS, tQ * tQ);

        // 3) Modulazione R
        float tR = cns((mag - base_kf_ACCEL_R_THRESH_START) * inv_dR, 0.0f, 1.0f);
        float Rb = lerp(base_kf_R_MAX, base_kf_R_MIN, tR * tR * tR);

        // 4) Influenza bordi
        float dx = (base_kf_x_state[i][0] - base_kf_X_CENTER) * inv_halfW;
        float dy = (base_kf_y_state[i][0] - base_kf_Y_CENTER) * inv_halfH;
        float fx = ss(base_kf_R_X_EDGE_SMOOTH_START, base_kf_R_X_EDGE_SMOOTH_END, fabsf(dx));
        float fy = ss(base_kf_R_Y_EDGE_SMOOTH_START, base_kf_R_Y_EDGE_SMOOTH_END, fabsf(dy));

        float Rx = lerp(Rb, base_kf_R_AT_X_EDGE, fx);
        Rx += lerp(0.0f, base_kf_R_AT_Y_EDGE_FOR_X, fy * base_kf_R_CROSS_AXIS_INFLUENCE_X);
        Rx = cns(Rx, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f);

        float Ry = lerp(Rb, base_kf_R_AT_Y_EDGE, fy);
        Ry += lerp(0.0f, base_kf_R_AT_X_EDGE_FOR_Y, fx * base_kf_R_CROSS_AXIS_INFLUENCE_Y);
        Ry = cns(Ry, base_kf_MIN_COVARIANCE_VALUE, base_kf_R_MAX * 5.0f);

        // 5) Predizione & aggiornamento Kalman X
        base_kf_x_state[i][0] += base_kf_x_state[i][1];
        float q00 = Q * 0.25f, q01 = Q * 0.5f, q11 = Q;
        float p00 = base_kf_p_x_00[i] + base_kf_p_x_01[i] + base_kf_p_x_10[i] + base_kf_p_x_11[i] + q00;
        float p01 = base_kf_p_x_01[i] + base_kf_p_x_11[i] + q01;
        float p11 = base_kf_p_x_11[i] + q11;
        p00 = cns(p00, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        p11 = cns(p11, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        float denomX = p00 + Rx;
        float K0 = p00 / denomX, K1 = p01 / denomX;
        float inx = mx - base_kf_x_state[i][0];
        base_kf_x_state[i][0] += K0 * inx;
        base_kf_x_state[i][1] += K1 * inx;
        base_kf_p_x_00[i] = p00 - K0 * p00;
        base_kf_p_x_01[i] = p01 - K0 * p01;
        base_kf_p_x_10[i] = base_kf_p_x_01[i];
        base_kf_p_x_11[i] = p11 - K1 * p01;

        // 6) Predizione & aggiornamento Kalman Y
        base_kf_y_state[i][0] += base_kf_y_state[i][1];
        float py00 = base_kf_p_y_00[i] + base_kf_p_y_01[i] + base_kf_p_y_10[i] + base_kf_p_y_11[i] + q00;
        float py01 = base_kf_p_y_01[i] + base_kf_p_y_11[i] + q01;
        float py11 = base_kf_p_y_11[i] + q11;
        py00 = cns(py00, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        py11 = cns(py11, base_kf_MIN_COVARIANCE_VALUE, base_kf_MAX_P_VALUE);
        float denomY = py00 + Ry;
        float L0 = py00 / denomY, L1 = py01 / denomY;
        float iny = my - base_kf_y_state[i][0];
        base_kf_y_state[i][0] += L0 * iny;
        base_kf_y_state[i][1] += L1 * iny;
        base_kf_p_y_00[i] = py00 - L0 * py00;
        base_kf_p_y_01[i] = py01 - L0 * py01;
        base_kf_p_y_10[i] = base_kf_p_y_01[i];
        base_kf_p_y_11[i] = py11 - L1 * py01;

        // 7) Salvataggio per il prossimo frame
        base_kf_last_measured_x[i] = mx;
        base_kf_last_vx_raw[i]     = vx;
        base_kf_last_measured_y[i] = my;
        base_kf_last_vy_raw[i]     = vy;

        // 8) Output filtrato
        FinalX[i] = (int)base_kf_x_state[i][0];
        FinalY[i] = (int)base_kf_y_state[i][0];
    }
}

//////////////////////////////////////// Versione 18 – fine  ////////////////////////////////////////


#endif //USE_SQUARE_ADVANCED