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
    //////if (num_points_seen > 1) {
    if (num_points_seen >= 1) {

        //just_reacquired_tracking = true; // Abbiamo appena ritrovato i punti
    
        // Dichiarazione di tutte le variabili locali usate nei vari blocchi logici.
        // Questo risolve gli errori di "scope" (variabile non dichiarata).
        int32_t dx, dy;
        uint8_t a, b, c, d;
        uint8_t P1, P2;
#ifdef COMMENTO
if (num_points_seen == 1)
{
    // ============================================================================
    // GESTIONE 1 SENSORE (Versione con Correzione Finale)
    // ============================================================================

    const int x1 = positionXX[0];
    const int y1 = positionYY[0];
    uint8_t identified_idx = 0;
    bool identified_successfully = false;

    // --- FASE 1: IDENTIFICAZIONE ---
    const float shortest_side = fminf(width, height);
    const float dynamic_threshold_sq = (shortest_side / 2.0f) * (shortest_side / 2.0f);

    if (prev_num_points_seen >= 2) // Transizione da stabile
    {
        float min_dist_sq = -1.0f;
        uint8_t best_guess_idx = 0;
        for (uint8_t i = 0; i < 4; i++) {
            if (prev_point_seen_mask & (1 << (3 - i))) {
                const float dx = (float)x1 - FinalX[i], dy = (float)y1 - FinalY[i];
                const float dist_sq = dx * dx + dy * dy;
                if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_guess_idx = i;
                }
            }
        }
        if (min_dist_sq >= 0 && min_dist_sq <= dynamic_threshold_sq) {
            identified_idx = best_guess_idx;
            identified_successfully = true;
        }
    }
    else if (prev_num_points_seen == 1) // Tracking continuato
    {
        const uint8_t candidate_idx = last_identified_idx;
        const float dx = (float)x1 - FinalX[candidate_idx], dy = (float)y1 - FinalY[candidate_idx];
        const float dist_sq = dx * dx + dy * dy;
        if (dist_sq <= dynamic_threshold_sq) {
            identified_idx = candidate_idx;
            identified_successfully = true;
        }
    }

    // --- FASE 2: RICOSTRUZIONE E PREPARAZIONE DATI ---
    if (identified_successfully)
    {
        // 1. Calcola il rettangolo ricostruito in buffer temporanei
        const float delta_x = (float)x1 - (float)FinalX[identified_idx];
        const float delta_y = (float)y1 - (float)FinalY[identified_idx];
        
        float Vx[4], Vy[4];
        for (int i = 0; i < 4; i++) {
            Vx[i] = (float)FinalX[i] + delta_x;
            Vy[i] = (float)FinalY[i] + delta_y;
        }

        // 2. AGGIORNAMENTO CRUCIALE (LA CORREZIONE)
        // Popoliamo positionXX e positionYY con i dati ricostruiti.
        // In questo modo, il tuo codice comune successivo troverà i dati corretti
        // e potrà riorganizzare FinalX/Y come previsto.
        for (int i = 0; i < 4; i++) {
            positionXX[i] = roundf(Vx[i]);
            positionYY[i] = roundf(Vy[i]);
        }
        
        // 3. Aggiorna le altre variabili di stato
        is_tracking_stable = true;
        last_identified_idx = identified_idx;
        // La maschera ora indica che TUTTI i punti sono "noti" (anche se ricostruiti)
        current_point_seen_mask = 0b00001111;
    }
    else
    {
        is_tracking_stable = false;
        current_point_seen_mask = 0;
    }
}
#endif

// Questo blocco va dopo la logica per i 4 e 2 sensori
if (num_points_seen == 1)
{
    // --- CONDIZIONE DI INGRESSO (CORRETTA) ---
    // Procediamo se il frame precedente non era completamente buio.
    if (prev_num_points_seen >= 1) 
    {

        /*
 * OBIETTIVO:
 * Trovare l'indice del vertice più vicino, usando la tecnica
 * di inizializzare la distanza minima a un valore molto alto.
 */

// --- DATI DI INPUT ---
const int x1 = positionXX[0]; 
const int y1 = positionYY[0];

// --- VARIABILE DI OUTPUT ---
uint8_t identified_idx;

// --- LOGICA DI RICERCA ---

// Inizializziamo la distanza minima al valore float più alto possibile.
float min_dist_sq = FLT_MAX; 
uint8_t best_found_idx = 0;

// Itera su tutti e 4 i vertici
for (uint8_t i = 0; i < 4; i++) 
{
    const float dx = (float)x1 - FinalX[i];
    const float dy = (float)y1 - FinalY[i];
    const float dist_sq = dx * dx + dy * dy;

    // L'if ora è più semplice. Funziona sia per il primo che per i successivi.
    // - Al primo ciclo: dist_sq sarà SEMPRE < FLT_MAX.
    // - Ai cicli successivi: funziona come un normale confronto.
    if (dist_sq < min_dist_sq) 
    {
        min_dist_sq = dist_sq;
        best_found_idx = i;
    }
}

identified_idx = best_found_idx;

        // --- CONTROLLO DI SICUREZZA (ANCORA VALIDO E IMPORTANTE) ---
        const float MAX_ALLOWED_DISTANCE_SQ = 200.0f * 200.0f; 
        if (min_dist_sq > MAX_ALLOWED_DISTANCE_SQ) {
            // Punto troppo lontano, probabile rumore. Non facciamo nulla per evitare salti.
            return; 
        }

        // --- FASE 2: RICOSTRUZIONE TRAMITE TRASCINAMENTO RIGIDO ---
        
        // Calcola lo spostamento (delta) del solo punto identificato
        const float delta_x = (float)x1 - FinalX[identified_idx];
        const float delta_y = (float)y1 - FinalY[identified_idx];

        // Applica lo stesso delta a tutti e 4 i vertici per trascinare il rettangolo
        for (uint8_t i = 0; i < 4; i++) {
            FinalX[i] += delta_x;
            FinalY[i] += delta_y;
            positionXX[i] = FinalX[i];
            positionYY[i] = FinalY[i];
        }
    }
    else
    {
        return;

    }
    // else: se prev_num_points_seen == 0, non facciamo nulla.
}


#ifdef COMMENTO
    if (num_points_seen == 2)
{
    // ==================================================================================
    // FASE 1 - Versione Finale con Costi Normalizzati e Angolo Dinamico
    // ==================================================================================

    const int x1 = positionXX[0], y1 = positionYY[0];
    const int x2 = positionXX[1], y2 = positionYY[1];
    
    uint8_t best_idx1 = 0, best_idx2 = 1;
    float min_total_cost = -1.0f;

    // --- 1. SETUP E CALCOLO PRELIMINARE COSTO DI POSIZIONE ---
    const uint8_t pairs[6][2] = { {0, 1}, {2, 3}, {0, 2}, {1, 3}, {0, 3}, {1, 2} };
    float pos_costs[12];
    for (int i = 0; i < 6; i++) {
        const uint8_t v1_idx = pairs[i][0], v2_idx = pairs[i][1];
        const float dx1_opt1 = (float)x1 - FinalX[v1_idx], dy1_opt1 = (float)y1 - FinalY[v1_idx];
        const float dx2_opt1 = (float)x2 - FinalX[v2_idx], dy2_opt1 = (float)y2 - FinalY[v2_idx];
        pos_costs[i*2] = (dx1_opt1*dx1_opt1 + dy1_opt1*dy1_opt1) + (dx2_opt1*dx2_opt1 + dy2_opt1*dy2_opt1);
        const float dx1_opt2 = (float)x1 - FinalX[v2_idx], dy1_opt2 = (float)y1 - FinalY[v2_idx];
        const float dx2_opt2 = (float)x2 - FinalX[v1_idx], dy2_opt2 = (float)y2 - FinalY[v1_idx];
        pos_costs[i*2 + 1] = (dx1_opt2*dx1_opt2 + dy1_opt2*dy1_opt2) + (dx2_opt2*dx2_opt2 + dy2_opt2*dy2_opt2);
    }
    
    // --- 2. SETUP PER LA NORMALIZZAZIONE ---
    // Template di forma
    const float d2_expected_side_h = height * height;
    const float expected_width = height * ideal_aspect_ratio;
    const float d2_expected_side_w = expected_width * expected_width;
    const float d2_expected_diag = d2_expected_side_w + d2_expected_side_h;
    const float expected_d2s[6] = {
        d2_expected_side_w, d2_expected_side_w, d2_expected_side_h,
        d2_expected_side_h, d2_expected_diag,   d2_expected_diag
    };

    // Fattore di normalizzazione per la posizione
    const float pos_normalization_factor = (width * width) + (height * height) + 1e-6f;

    // Pesi (valori di partenza da calibrare)
    const float W_POS   = 1.0f; // Peso per il costo di posizione
    const float W_SHAPE = 1.5f; // Diamo un po' più di importanza alla forma
    const float W_ANGLE = 0.8f; // Diamo un po' meno importanza all'angolo

    // --- 3. CALCOLO DEI VALORI MISURATI ---
    const float dx_measured = (float)x1 - (float)x2;
    const float dy_measured = (float)y1 - (float)y2;
    const float d2_measured = dx_measured * dx_measured + dy_measured * dy_measured;
    const float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
    const float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
    const float dot_product = cdx1 * cdx2 + cdy1 * cdy2;
    const float mag_prod = sqrtf((cdx1*cdx1 + cdy1*cdy1) * (cdx2*cdx2 + cdy2*cdy2));
    const float cos_theta = dot_product / (mag_prod + 1e-6f);

    // --- 4. CICLO DI DECISIONE FINALE CON COSTI NORMALIZZATI E ANGOLO DINAMICO ---
    
    // Calcolo dinamico dei coseni attesi in base all'aspect ratio
    const float r = ideal_aspect_ratio;
    const float r2 = r * r;
    // Coseno dell'angolo per i lati "lunghi" (associati alla larghezza)
    const float cos_theta_expected_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
    // Coseno dell'angolo per i lati "corti" (associati all'altezza)
    const float cos_theta_expected_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);

    for (int i = 0; i < 6; i++) {
        // A) Costo Angolare Normalizzato con valore atteso DINAMICO
        float cos_theta_expected;
        // La convenzione usata in 'expected_d2s' associa i primi 2 indici alla larghezza (W)
        // e i successivi 2 all'altezza (H). Usiamo la stessa convenzione qui.
        if (i < 2)      { cos_theta_expected = cos_theta_expected_W; } 
        else if (i < 4) { cos_theta_expected = cos_theta_expected_H; } 
        else            { cos_theta_expected = -1.0f; } // Diagonali

        const float diff_cos = cos_theta - cos_theta_expected;
        const float angle_cost_norm = (diff_cos * diff_cos) / 4.0f; // Normalizzato tra [0, 1]

        // B) Costo di Forma Normalizzato
        const float shape_cost_norm = fabsf(d2_measured - expected_d2s[i]) / (expected_d2s[i] + 1e-6f);

        // C) Calcolo del Costo Totale per le due possibili assegnazioni
        const float cost1_norm = (pos_costs[i*2]      / pos_normalization_factor) * W_POS + shape_cost_norm * W_SHAPE + angle_cost_norm * W_ANGLE;
        const float cost2_norm = (pos_costs[i*2 + 1]  / pos_normalization_factor) * W_POS + shape_cost_norm * W_SHAPE + angle_cost_norm * W_ANGLE;

        // D) Aggiornamento del minimo
        if (min_total_cost < 0 || cost1_norm < min_total_cost) {
            min_total_cost = cost1_norm;
            best_idx1 = pairs[i][0];
            best_idx2 = pairs[i][1];
        }
        if (min_total_cost < 0 || cost2_norm < min_total_cost) {
            min_total_cost = cost2_norm;
            best_idx1 = pairs[i][1];
            best_idx2 = pairs[i][0];
        }
    }
    
    // --- 5. ASSEGNAZIONE E PREPARAZIONE PER LA FASE 2 ---
    const uint8_t idx1 = best_idx1;
    const uint8_t idx2 = best_idx2;
    float Vx[4], Vy[4];
    Vx[idx1] = (float)x1; Vy[idx1] = (float)y1;
    Vx[idx2] = (float)x2; Vy[idx2] = (float)y2;
    
    // ... qui prosegue la FASE 2 ...


    // ====================================================================================================
    // FASE 2: RICOSTRUZIONE GEOMETRICA
    // ====================================================================================================

    bool top    = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
    bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
    bool left   = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
    bool right  = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);


if (top || bottom) {
        uint8_t p1_idx, p2_idx, p3_idx, p4_idx;
        if (top) { p1_idx = 0; p2_idx = 1; p3_idx = 2; p4_idx = 3; } 
        else   { p1_idx = 2; p2_idx = 3; p3_idx = 0; p4_idx = 1; }

        if (Vx[p1_idx] > Vx[p2_idx]) { uint8_t tmp = p1_idx; p1_idx = p2_idx; p2_idx = tmp; }

        float dx = Vx[p2_idx] - Vx[p1_idx], dy = Vy[p2_idx] - Vy[p1_idx];
        float base = hypotf(dx, dy); // Larghezza misurata

        // La guardia 'width < 1e-6f' non serve più, ci fidiamo del rapporto ideale
        if (!(base < 1e-6f)) {
            // --- LOGICA CORRETTA ---
            // Rimosso scaleFactor. Calcoliamo l'altezza direttamente dalla larghezza misurata
            // e dal nostro rapporto di forma ideale e stabile.
            // Formula: Altezza = Larghezza / (Larghezza/Altezza)
            float newHeight = base / ideal_aspect_ratio;
            
            float inv_base = 1.0f / base;
            float ndx = -dy * inv_base, ndy = dx * inv_base;

            if (top) {
                Vx[p3_idx] = Vx[p1_idx] + ndx * newHeight; Vy[p3_idx] = Vy[p1_idx] + ndy * newHeight;
                Vx[p4_idx] = Vx[p2_idx] + ndx * newHeight; Vy[p4_idx] = Vy[p2_idx] + ndy * newHeight;
            } else {
                Vx[p3_idx] = Vx[p1_idx] - ndx * newHeight; Vy[p3_idx] = Vy[p1_idx] - ndy * newHeight;
                Vx[p4_idx] = Vx[p2_idx] - ndx * newHeight; Vy[p4_idx] = Vy[p2_idx] - ndy * newHeight;
            }
        }
    } else if (left || right) {
        uint8_t p1_idx, p2_idx, p3_idx, p4_idx;
        if (left) { p1_idx = 0; p2_idx = 2; p3_idx = 1; p4_idx = 3; }
        else    { p1_idx = 1; p2_idx = 3; p3_idx = 0; p4_idx = 2; }

        if (Vy[p1_idx] > Vy[p2_idx]) { uint8_t tmp = p1_idx; p1_idx = p2_idx; p2_idx = tmp; }

        float dx = Vx[p2_idx] - Vx[p1_idx], dy = Vy[p2_idx] - Vy[p1_idx];
        float hgt = hypotf(dx, dy); // Altezza misurata

        // La guardia 'height < 1e-6f' non serve più
        if (!(hgt < 1e-6f)) {
            // --- LOGICA CORRETTA ---
            // Rimosso scaleFactor. Calcoliamo la larghezza direttamente dall'altezza misurata
            // e dal nostro rapporto di forma ideale e stabile.
            // Formula: Larghezza = Altezza * (Larghezza/Altezza)
            float newWidth = hgt * ideal_aspect_ratio;
            
            float inv_hgt = 1.0f / hgt;
            float ndx = -dy * inv_hgt, ndy = dx * inv_hgt;

            if (left) {
                Vx[p3_idx] = Vx[p1_idx] - ndx * newWidth; Vy[p3_idx] = Vy[p1_idx] - ndy * newWidth;
                Vx[p4_idx] = Vx[p2_idx] - ndx * newWidth; Vy[p4_idx] = Vy[p2_idx] - ndy * newWidth;
            } else {
                Vx[p3_idx] = Vx[p1_idx] + ndx * newWidth; Vy[p3_idx] = Vy[p1_idx] + ndy * newWidth;
                Vx[p4_idx] = Vx[p2_idx] + ndx * newWidth; Vy[p4_idx] = Vy[p2_idx] + ndy * newWidth;
            }
        }
    }    
else {
    // --- BLOCCO DIAGONALI: LOGICA CORRETTA E SICURA ---
    bool diagAD = ((idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0));
    
    // Identificazione punti di inizio e fine
    float p_start_x = diagAD ? Vx[0] : Vx[1];
    float p_start_y = diagAD ? Vy[0] : Vy[1];
    float p_end_x   = diagAD ? Vx[3] : Vx[2];
    float p_end_y   = diagAD ? Vy[3] : Vy[2];

    // 1. Misura della diagonale attuale
    float dx = p_end_x - p_start_x;
    float dy = p_end_y - p_start_y;
    float L = hypotf(dx, dy);

    if (!(L < 1e-6f)) {
        // --- NUOVA LOGICA: Authoritative Aspect Ratio ---
        // 2. Calcoliamo W e H direttamente dalla diagonale misurata (L) e dal rapporto ideale
        float ratio_sq_plus_one = ideal_aspect_ratio * ideal_aspect_ratio + 1.0f;
        float H = L / sqrtf(ratio_sq_plus_one);
        float W = H * ideal_aspect_ratio;

        // 3. Calcolo dell'angolo di rotazione con il metodo standard
        // Angolo della diagonale che abbiamo misurato
        float phi_measured = atan2f(dy, dx);
        
        // Angolo che la diagonale DOVREBBE avere, usando i nuovi W e H
        float phi_ideal = diagAD ? atan2f(H, W) : atan2f(H, -W);
        
        // La rotazione da applicare è la differenza tra i due angoli
        float theta = phi_measured - phi_ideal;
        float c = cosf(theta);
        float si = sinf(theta);

        // 4. Ricostruzione dei vertici (identica a prima, ma con valori corretti)
        // Vettori dei lati, ruotati e scalati
        float wvx = c * W, wvy = si * W;
        float hvx = -si * H, hvy = c * H;

        if (diagAD) {
            Vx[1] = Vx[0] + wvx; Vy[1] = Vy[0] + wvy;
            Vx[2] = Vx[0] + hvx; Vy[2] = Vy[0] + hvy;
        } else { // diagBC
            Vx[0] = Vx[1] - wvx; Vy[0] = Vy[1] - wvy;
            Vx[3] = Vx[1] + hvx; Vy[3] = Vy[1] + hvy;
        }
    }
}
    // ====================================================================================================
    // FASE 3: SMOOTHING E AGGIORNAMENTO FINALE
    // ====================================================================================================

    const float smoothFactor = 0.7f;
    for (int i = 0; i < 4; i++) {
        FinalX[i] = FinalX[i] * smoothFactor + Vx[i] * (1.0f - smoothFactor);
        FinalY[i] = FinalY[i] * smoothFactor + Vy[i] * (1.0f - smoothFactor);
        positionXX[i] = (int)roundf(FinalX[i]);
        positionYY[i] = (int)roundf(FinalY[i]);
    }

current_point_seen_mask = (1 << (3-idx1)) | (1 << (3-idx2));    

}
#endif // COMMENTO
#ifdef COMMENTO
// ultimo chatgpt buono
if (num_points_seen == 2)
{
    const int x1 = positionXX[0], y1 = positionYY[0];
    const int x2 = positionXX[1], y2 = positionYY[1];

    uint8_t best_idx1 = 0, best_idx2 = 1;
    float min_total_cost = -1.0f;

    const uint8_t pairs[6][2] = { {0,1}, {2,3}, {0,2}, {1,3}, {0,3}, {1,2} };
    float pos_costs[12];

    for (int i = 0; i < 6; i++) {
        const uint8_t v1 = pairs[i][0], v2 = pairs[i][1];

        float dx1a = (float)x1 - (float)FinalX[v1];
        float dy1a = (float)y1 - (float)FinalY[v1];
        float dx2a = (float)x2 - (float)FinalX[v2];
        float dy2a = (float)y2 - (float)FinalY[v2];
        pos_costs[i*2] = dx1a*dx1a + dy1a*dy1a + dx2a*dx2a + dy2a*dy2a;

        float dx1b = (float)x1 - (float)FinalX[v2];
        float dy1b = (float)y1 - (float)FinalY[v2];
        float dx2b = (float)x2 - (float)FinalX[v1];
        float dy2b = (float)y2 - (float)FinalY[v1];
        pos_costs[i*2 + 1] = dx1b*dx1b + dy1b*dy1b + dx2b*dx2b + dy2b*dy2b;
    }

    const float d2H = height * height;
    const float estW = height * ideal_aspect_ratio;
    const float d2W = estW * estW;
    const float d2D = d2W + d2H;
    const float expected_d2s[6] = { d2W, d2W, d2H, d2H, d2D, d2D };

    const float pos_norm = (width*width + height*height + 1e-6f);

    const float W_POS = 1.0f, W_SHAPE = 1.5f, W_ANGLE = 0.8f;

    float dxm = (float)x1 - (float)x2;
    float dym = (float)y1 - (float)y2;
    float d2m = dxm*dxm + dym*dym;

    float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
    float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
    float dot = cdx1 * cdx2 + cdy1 * cdy2;
    float mag = sqrtf((cdx1*cdx1 + cdy1*cdy1) * (cdx2*cdx2 + cdy2*cdy2));
    mag = fmaxf(mag, 1e-6f);
    float cos_theta = dot / mag;
    cos_theta = fminf(1.0f, fmaxf(-1.0f, cos_theta));

    const float r = ideal_aspect_ratio;
    const float r2 = r * r;
    const float cos_theta_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
    const float cos_theta_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);

    for (int i = 0; i < 6; i++) {
        float expected_cos = (i < 2) ? cos_theta_W : (i < 4) ? cos_theta_H : -1.0f;
        float diff_cos = cos_theta - expected_cos;
        float angle_cost = (diff_cos * diff_cos) / 4.0f;

        float safe_d2 = fmaxf(expected_d2s[i], 25.0f);
        float shape_cost = fabsf(d2m - expected_d2s[i]) / safe_d2;

        float cost1 = (pos_costs[i*2]     / pos_norm) * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;
        float cost2 = (pos_costs[i*2 + 1] / pos_norm) * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;

        uint8_t i1 = pairs[i][0], i2 = pairs[i][1];
        float best_cost = cost1;

        if (cost2 < best_cost) {
            best_cost = cost2;
            i1 = pairs[i][1];
            i2 = pairs[i][0];
        }

        if (min_total_cost < 0 || best_cost < min_total_cost) {
            min_total_cost = best_cost;
            best_idx1 = i1;
            best_idx2 = i2;
        }
    }

    // ==================================================================================
    // FASE 2 - Ricostruzione geometrica con correzione prospettica top/bottom
    // ==================================================================================

    float Vx[4], Vy[4];
    for (int i = 0; i < 4; i++) {
        Vx[i] = (float)FinalX[i];
        Vy[i] = (float)FinalY[i];
    }
    Vx[best_idx1] = (float)x1;
    Vy[best_idx1] = (float)y1;
    Vx[best_idx2] = (float)x2;
    Vy[best_idx2] = (float)y2;

    const uint8_t idx1 = best_idx1, idx2 = best_idx2;
    bool top    = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
    bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
    bool left   = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
    bool right  = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

    if (top || bottom) {
        uint8_t p1 = top ? 0 : 2;
        uint8_t p2 = top ? 1 : 3;
        uint8_t p3 = top ? 2 : 0;
        uint8_t p4 = top ? 3 : 1;

        if (Vx[p1] > Vx[p2]) { uint8_t t = p1; p1 = p2; p2 = t; }

        float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
        float base = sqrtf(dx*dx + dy*dy);
        float cos_angle = fabsf(dx) / fmaxf(base, 1e-6f);
        base = base / fmaxf(cos_angle, 0.3f);

        if (base > 2.0f) {
            float newH = base / ideal_aspect_ratio;
            newH = fmaxf(newH, height * 0.5f);

            float inv_base = 1.0f / base;
            float ndx = -dy * inv_base, ndy = dx * inv_base;

            float sx = ndx * newH, sy = ndy * newH;

            if (top) {
                Vx[p3] = Vx[p1] + sx; Vy[p3] = Vy[p1] + sy;
                Vx[p4] = Vx[p2] + sx; Vy[p4] = Vy[p2] + sy;
            } else {
                Vx[p3] = Vx[p1] - sx; Vy[p3] = Vy[p1] - sy;
                Vx[p4] = Vx[p2] - sx; Vy[p4] = Vy[p2] - sy;
            }
        }
    } else if (left || right) {
        uint8_t p1 = left ? 0 : 1;
        uint8_t p2 = left ? 2 : 3;
        uint8_t p3 = left ? 1 : 0;
        uint8_t p4 = left ? 3 : 2;

        if (Vy[p1] > Vy[p2]) { uint8_t t = p1; p1 = p2; p2 = t; }

        float dx = Vx[p2] - Vx[p1], dy = Vy[p2] - Vy[p1];
        float hgt = sqrtf(dx*dx + dy*dy);

        if (hgt > 2.0f) {
            float newW = hgt * ideal_aspect_ratio;
            float inv_hgt = 1.0f / hgt;
            float ndx = -dy * inv_hgt, ndy = dx * inv_hgt;

            float sx = ndx * newW, sy = ndy * newW;

            if (left) {
                Vx[p3] = Vx[p1] - sx; Vy[p3] = Vy[p1] - sy;
                Vx[p4] = Vx[p2] - sx; Vy[p4] = Vy[p2] - sy;
            } else {
                Vx[p3] = Vx[p1] + sx; Vy[p3] = Vy[p1] + sy;
                Vx[p4] = Vx[p2] + sx; Vy[p4] = Vy[p2] + sy;
            }
        }
    } else {
        bool diagAD = (idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0);

        float px1 = diagAD ? Vx[0] : Vx[1];
        float py1 = diagAD ? Vy[0] : Vy[1];
        float px2 = diagAD ? Vx[3] : Vx[2];
        float py2 = diagAD ? Vy[3] : Vy[2];

        float dx = px2 - px1, dy = py2 - py1;
        float L = sqrtf(dx*dx + dy*dy);

        if (L > 2.0f) {
            float ratio_sq = ideal_aspect_ratio * ideal_aspect_ratio + 1.0f;
            float H = L / sqrtf(ratio_sq);
            float W = H * ideal_aspect_ratio;

            float phi_m = atan2f(dy, dx);
            float phi_i = diagAD ? atan2f(H, W) : atan2f(H, -W);
            float theta = phi_m - phi_i;

            float c = cosf(theta), s = sinf(theta);
            float wx = c * W, wy = s * W;
            float hx = -s * H, hy = c * H;

            if (diagAD) {
                Vx[1] = Vx[0] + wx; Vy[1] = Vy[0] + wy;
                Vx[2] = Vx[0] + hx; Vy[2] = Vy[0] + hy;
            } else {
                Vx[0] = Vx[1] - wx; Vy[0] = Vy[1] - wy;
                Vx[3] = Vx[1] + hx; Vy[3] = Vy[1] + hy;
            }
        }
    }

    // ==================================================================================
    // FASE 3 - Assegnazione finale senza smoothing
    // ==================================================================================

    for (int i = 0; i < 4; i++) {
        FinalX[i] = (int)roundf(Vx[i]);
        FinalY[i] = (int)roundf(Vy[i]);
        positionXX[i] = FinalX[i];
        positionYY[i] = FinalY[i];
    }

    current_point_seen_mask = (1 << (3 - best_idx1)) | (1 << (3 - best_idx2));
}
#endif

// =================================================================================================
// GESTIONE 2 PUNTI VISTI (VERSIONE FINALE UNIFICATA CON PESI A 3 LIVELLI)
//
// Strategia:
// La struttura di calcolo è unica. L'intelligenza è demandata alla selezione
// dei pesi (W_POS, W_SHAPE, W_ANGLE) che vengono impostati a zero per i costi
// che si vogliono escludere in un determinato contesto.
// =================================================================================================

if (num_points_seen == 2)
{
    // --- FASE 1: IDENTIFICAZIONE ---

    const int x1 = positionXX[0], y1 = positionYY[0];
    const int x2 = positionXX[1], y2 = positionYY[1];

    uint8_t best_idx1 = 0, best_idx2 = 1;
    float min_total_cost = -1.0f;

    // --- Calcolo di TUTTI i possibili componenti di costo ---

    // 1. Costo di Posizione
    const uint8_t pairs[6][2] = { {0,1}, {2,3}, {0,2}, {1,3}, {0,3}, {1,2} };
    float pos_costs[12];
    for (int i = 0; i < 6; i++) {
        const uint8_t v1 = pairs[i][0], v2 = pairs[i][1];
        float dx1a=(float)x1-FinalX[v1], dy1a=(float)y1-FinalY[v1];
        float dx2a=(float)x2-FinalX[v2], dy2a=(float)y2-FinalY[v2];
        pos_costs[i*2] = dx1a*dx1a + dy1a*dy1a + dx2a*dx2a + dy2a*dy2a;
        float dx1b=(float)x1-FinalX[v2], dy1b=(float)y1-FinalY[v2];
        float dx2b=(float)x2-FinalX[v1], dy2b=(float)y2-FinalY[v1];
        pos_costs[i*2 + 1] = dx1b*dx1b + dy1b*dy1b + dx2b*dx2b + dy2b*dy2b;
    }
    const float inv_pos_norm = 1.0f / (width*width + height*height + 1e-6f);

    // 2. Costo di Forma e Angolo (calcolati anche se poi non usati)
    const float d2H = height * height;
    const float estW = height * ideal_aspect_ratio;
    const float d2W = estW * estW;
    const float d2D = d2W + d2H;
    const float expected_d2s[6] = { d2W, d2W, d2H, d2H, d2D, d2D };
    const float dxm = (float)x1 - (float)x2;
    const float dym = (float)y1 - (float)y2;
    const float d2m = dxm*dxm + dym*dym;
    const float r = ideal_aspect_ratio;
    const float r2 = r * r;
    const float cos_theta_W = (1.0f - r2) / (1.0f + r2 + 1e-6f);
    const float cos_theta_H = (r2 - 1.0f) / (1.0f + r2 + 1e-6f);
    const float cdx1 = (float)x1 - medianX, cdy1 = (float)y1 - medianY;
    const float cdx2 = (float)x2 - medianX, cdy2 = (float)y2 - medianY;
    const float dot = cdx1 * cdx2 + cdy1 * cdy2;
    float mag = sqrtf((cdx1*cdx1 + cdy1*cdy1) * (cdx2*cdx2 + cdy2*cdy2));
    mag = fmaxf(mag, 1e-6f);
    float cos_theta = dot / mag;
    cos_theta = fminf(1.0f, fmaxf(-1.0f, cos_theta));

    #ifdef COMMENTO
    // --- Selezione dei pesi a 3 livelli (il "cervello" della logica) ---
    float W_POS, W_SHAPE, W_ANGLE;
    if (prev_num_points_seen <= 1) {
        // LIVELLO 3 (Riacquisizione): Usa SOLO la posizione.
        W_POS   = 0.4f;
        W_SHAPE = 1.0f;
        W_ANGLE = 0.2f;
    } else if (prev_num_points_seen == 3) {
        // LIVELLO 2 (Transizione): Posizione debole, forma e angolo forti.
        W_POS   = 0.15f;
        W_SHAPE = 1.0f; //1.5f;
        W_ANGLE = 0.8f; //1.0f;
    } else { // prev_num_points_seen == 2 || 4
        // LIVELLO 1 (Stabile): Pesi bilanciati.
        W_POS   = 1.0f;
        W_SHAPE = 0.3f; //1.5f;
        W_ANGLE = 0.15f; //0.8f;
    }
    #endif

// =========================================================================================
// MODELLO DEFINITIVO v2 - Bilanciamento Finale
// Sintesi tra stabilità temporale e robustezza geometrica.
// =========================================================================================

float W_POS, W_SHAPE, W_ANGLE;

if (prev_num_points_seen == 4) {
    // LIVELLO 1 (4/4): GOLD. Posizione e Forma sono ugualmente importanti.
    W_POS   = 1.2f;
    W_SHAPE = 1.2f;  // <- Alzato per eguagliare la posizione.
    W_ANGLE = 1.0f;

} else if (prev_num_points_seen == 3) {
    // LIVELLO 2 (3/4): SILVER. Leggermente meno fiducia nella posizione,
    // ma fiducia massima nella forma "ideale" della memoria.
    W_POS   = 1.0f;
    W_SHAPE = 1.4f;  // <- Leggermente più alto di W_POS per la perfezione della forma.
    W_ANGLE = 1.1f;

} else if (prev_num_points_seen == 2) {
    // LIVELLO 3 (2/4): BRONZE. Meno fiducia nella posizione (basata su 2pt),
    // più fiducia nella geometria misurata ORA.
    W_POS   = 0.8f;
    W_SHAPE = 1.3f;  // <- Alto per compensare la fiducia ridotta in W_POS.
    W_ANGLE = 1.2f;

} else { // prev_num_points_seen <= 1
    // LIVELLO 4 (1/4 o 0/4): INAFFIDABILE. La posizione non conta quasi nulla.
    // Ci si affida solo alla geometria dell'input corrente.
    W_POS   = 0.1f;
    W_SHAPE = 1.8f;
    W_ANGLE = 0.1f; //05f; //0.1f; //1.5f;
}


    // --- Calcolo del costo totale con un'unica logica ---
    for (int i = 0; i < 6; i++) {
        //const float shape_cost = fabsf(d2m - expected_d2s[i]) / fmaxf(expected_d2s[i], 25.0f);
        const float shape_cost = fabsf(d2m - expected_d2s[i]) / fmaxf(d2m, fmaxf(expected_d2s[i], 1.0f));

        
        const float expected_cos = (i < 2) ? cos_theta_W : (i < 4) ? cos_theta_H : -1.0f;
        const float diff_cos = cos_theta - expected_cos;
        const float angle_cost = (diff_cos * diff_cos) / 4.0f;
        const float normalized_pos_cost1 = pos_costs[i*2] * inv_pos_norm;
        const float normalized_pos_cost2 = pos_costs[i*2 + 1] * inv_pos_norm;

        const float cost1 = normalized_pos_cost1 * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;
        const float cost2 = normalized_pos_cost2 * W_POS + shape_cost * W_SHAPE + angle_cost * W_ANGLE;

        if (min_total_cost < 0 || cost1 < min_total_cost) {
            min_total_cost = cost1;
            best_idx1 = pairs[i][0]; best_idx2 = pairs[i][1];
        }
        if (min_total_cost < 0 || cost2 < min_total_cost) {
            min_total_cost = cost2;
            best_idx1 = pairs[i][1]; best_idx2 = pairs[i][0];
        }
    }

    // --- FASE 2: RICOSTRUZIONE GEOMETRICA ---

    float Vx[4], Vy[4];
    for (int i = 0; i < 4; i++) { Vx[i] = FinalX[i]; Vy[i] = FinalY[i]; }
    Vx[best_idx1] = (float)x1; Vy[best_idx1] = (float)y1;
    Vx[best_idx2] = (float)x2; Vy[best_idx2] = (float)y2;
    
    const uint8_t idx1 = best_idx1, idx2 = best_idx2;
    const bool top    = (idx1 == 0 && idx2 == 1) || (idx1 == 1 && idx2 == 0);
    const bool bottom = (idx1 == 2 && idx2 == 3) || (idx1 == 3 && idx2 == 2);
    const bool left   = (idx1 == 0 && idx2 == 2) || (idx1 == 2 && idx2 == 0);
    const bool right  = (idx1 == 1 && idx2 == 3) || (idx1 == 3 && idx2 == 1);

    if (top || bottom) {
        uint8_t p1 = top ? 0 : 2, p2 = top ? 1 : 3;
        uint8_t p3 = top ? 2 : 0, p4 = top ? 3 : 1;
        if (Vx[p1] > Vx[p2]) { uint8_t t=p1; p1=p2; p2=t; }
        float dx = Vx[p2]-Vx[p1], dy = Vy[p2]-Vy[p1];
        float base = sqrtf(dx*dx + dy*dy);
        if (base > 2.0f) {
            float cos_angle = fabsf(dx) / base;
            base = base / fmaxf(cos_angle, 0.3f);
            float newH = fmaxf(base / ideal_aspect_ratio, height * 0.5f);
            float inv_base = 1.0f / base;
            float ndx = -dy*inv_base, ndy = dx*inv_base;
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
        if (Vy[p1] > Vy[p2]) { uint8_t t=p1; p1=p2; p2=t; }
        float dx = Vx[p2]-Vx[p1], dy = Vy[p2]-Vy[p1];
        float hgt = sqrtf(dx*dx+dy*dy);
        if (hgt > 2.0f) {
            float newW = hgt * ideal_aspect_ratio;
            float inv_hgt = 1.0f/hgt;
            float ndx = -dy*inv_hgt, ndy = dx*inv_hgt;
            float sx = ndx*newW, sy = ndy*newW;
            if (left) {
                Vx[p3]=Vx[p1]-sx; Vy[p3]=Vy[p1]-sy; Vx[p4]=Vx[p2]-sx; Vy[p4]=Vy[p2]-sy;
            } else {
                Vx[p3]=Vx[p1]+sx; Vy[p3]=Vy[p1]+sy; Vx[p4]=Vx[p2]+sx; Vy[p4]=Vy[p2]+sy;
            }
        }
    } else { // Diagonale
        bool diagAD = (idx1 == 0 && idx2 == 3) || (idx1 == 3 && idx2 == 0);
        float px1 = diagAD ? Vx[0] : Vx[1], py1 = diagAD ? Vy[0] : Vy[1];
        float px2 = diagAD ? Vx[3] : Vx[2], py2 = diagAD ? Vy[3] : Vy[2];
        float dx = px2 - px1, dy = py2 - py1;
        float L = sqrtf(dx*dx + dy*dy);
        if (L > 2.0f) {
            float H = L / sqrtf(ideal_aspect_ratio*ideal_aspect_ratio + 1.0f);
            float W = H * ideal_aspect_ratio;
            float phi_m = atan2f(dy, dx);
            float phi_i = diagAD ? atan2f(H, W) : atan2f(H, -W);
            float theta = phi_m - phi_i;
            float c=cosf(theta), s=sinf(theta);
            float wx=c*W, wy=s*W;
            float hx=-s*H, hy=c*H;
            if (diagAD) {
                Vx[1]=Vx[0]+wx; Vy[1]=Vy[0]+wy; Vx[2]=Vx[0]+hx; Vy[2]=Vy[0]+hy;
            } else {
                Vx[0]=Vx[1]-wx; Vy[0]=Vy[1]-wy; Vx[3]=Vx[1]+hx; Vy[3]=Vy[1]+hy;
            }
        }
    }

    // --- FASE 3: ASSEGNAZIONE FINALE ---

    for (int i = 0; i < 4; i++) {
        FinalX[i] = roundf(Vx[i]);
        FinalY[i] = roundf(Vy[i]);
        positionXX[i] = FinalX[i];
        positionYY[i] = FinalY[i];
    }
    current_point_seen_mask = (1 << (3 - best_idx1)) | (1 << (3 - best_idx2));
}

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
                //current_point_seen_mask = 0b00001111;
            }
            else if (d12_sq >= d02_sq) {
                //d12 è la diagonale  del rettangolo
                // b, il punto tra lato lungo e lato corto è per forza il punto 0
                a_idx = 1; c_idx = 2; b_idx = 0;
                //current_point_seen_mask = 0b00001111;
            }
            else {
                //d02 è la diagonale  del rettangolo
                // b, il punto tra lato lungo e lato corto è per forza il punto 2 
                a_idx = 0; c_idx = 2; b_idx = 1;
                //current_point_seen_mask = 0b00001111;
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
        
        if (num_points_seen == 4) {
            // NUOVO: Aggiorna la nostra "verità assoluta" sulla forma
            if (height > 1e-6f) { // Evita divisione per zero
                ideal_aspect_ratio = width / height;
            }
        }

        angle = (atan2f((float)FinalY[A] - FinalY[B], (float)FinalX[B] - FinalX[A]) + atan2f((float)FinalY[C] - FinalY[D], (float)FinalX[D] - FinalX[C])) / 2.0f;      

        is_tracking_stable = true;
        prev_point_seen_mask = current_point_seen_mask; 
        if (num_points_seen >= 3) current_point_seen_mask = 0b00001111;
    }
    else {
    is_tracking_stable = false;
    prev_point_seen_mask = 0;
}
    prev_num_points_seen = num_points_seen;
}

#endif //USE_SQUARE_ADVANCED