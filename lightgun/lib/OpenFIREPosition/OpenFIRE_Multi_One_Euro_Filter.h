#ifdef USE_MULTI_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_Multi_One_Euro_Filter.h
 * @brief Library for One Euro Filter for 4 LED
 * @n CPP Library for One Euro Filter for 4 LED
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V2.0
 * @date 2026
 */

#ifndef OpenFIRE_Multi_One_Euro_Filter_h
#define OpenFIRE_Multi_One_Euro_Filter_h

#include <Arduino.h>
#include "OpenFIREConst.h"

class OpenFIRE_One_Euro_Multi {
private:
    // Struttura fusa: mantiene sia lo storico posizionale (prev/hat) che vettoriale (vel).
    // Usare una singola struct garantisce che la cache L1 della CPU legga tutto lo stato
    // di un singolo asse in una sola operazione (Locality of Reference).
    struct FilterState {
        float x_prev, x_hat;
        float y_prev, y_hat;
        float vel_x_hat, vel_y_hat;
    };

    FilterState states[4];
    unsigned long lastMicros;
    bool initialized = false;

    // Variabili per il centro dello schermo (calcolate una sola volta al boot)
    // Conserviamo i reciproci (1/x) per commutare le pesanti divisioni geometriche 
    // in moltiplicazioni veloci durante il runtime.
    float inv_center_x;
    float inv_center_y;
    
    // ==========================================
    // --- PARAMETRI DI TUNING E-SPORTS (BILANCIAMENTO DEFINITIVO) ---

    // min_cutoff: La "lentezza" del mirino quando ti muovi pochissimo o sei fermo.
    // Impostato a 0.1f: il punto di equilibrio perfetto. 0.2f era leggermente 
    // scivoloso, 0.05f era troppo rigido. 0.1f garantisce mira da cecchino solida.
    const float min_cutoff = 0.1f; 
    
    // --- GESTIONE ASIMMETRICA DELLA VELOCITÀ ---
    // d_cutoff_base: Reattività per i movimenti di precisione.
    // IMPOSTATO a 1.0f. A 200Hz, un salto di 1 singolo pixel raw della telecamera
    // equivale a 800 pixel/sec! Serve un cutoff basso (1.0f) per spalmare questi salti 
    // quantizzati (0, 800, 0, 800) in una velocità costante e fluida (es. 400).
    const float d_cutoff_base = 1.0f; 
    
    // d_cutoff_snap: Reattività per scatti violenti e frenate brusche.
    const float d_cutoff_snap = 25.0f; 
    
    // snap_base: Il "Punto di Rottura" al CENTRO dello schermo (in pixel/sec).
    // IMPOSTATO a 1000.0f. Questo è FONDAMENTALE. Essendo il rumore di quantizzazione
    // pari a ~800 px/s, se abbassiamo la soglia a 400 il filtro "scatta" ad ogni singolo
    // aggiornamento dei pixel della telecamera, causando micro-vibrazioni continue.
    // 1000.0f ignora i salti di 1 pixel (800) ma interviene sui flick-shot.
    const float snap_base = 1000.0f;

    // snap_edge_multiplier: Quanta "resistenza" aggiungere quando si mira ai BORDI.
    const float snap_edge_multiplier = 2000.0f;

    // max_cutoff: Il limite di banda passante superiore. 
    const float max_cutoff = 30.0f; 
    
    // beta_multiplier: Moltiplicatore di reattività spaziale.
    // Portato a 2.8f (dal 2.5f originale). Un boost chirurgico: apre il filtro un 
    // pelo più velocemente per abbattere la latenza, senza i tremori visti a 3.5f.
    const float beta_multiplier = 2.8f; 
    const float beta_base = ((0.011f * (float)CamResX) / (float)MouseResX) * beta_multiplier;

    // ==========================================


    const float OEF_TWO_PI = 6.28318530718f;

    // Sostituisce l'implementazione classica dell'Exponential Moving Average.
    // Espande la formula matematica per evitare chiamate di funzione ricorsive.
    inline float fast_alpha(float cutoff, float dt_two_pi) {
        float te = cutoff * dt_two_pi;
        return te / (te + 1.0f);
    }

public:
    OpenFIRE_One_Euro_Multi();
    
    // Il passaggio tramite puntatori permette di leggere i raw (int) e restituire le coordinate
    // sub-pixel perfette (float) senza copie di array.
    void process(int* x_in, int* y_in, float* x_out, float* y_out);
};

#endif // OpenFIRE_Multi_One_Euro_Filter_h

#endif // USE_MULTI_ONE_EURO_FILTER