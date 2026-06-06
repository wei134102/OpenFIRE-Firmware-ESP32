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


    #ifdef COMMENTO
    // --- PARAMETRI DI TUNING ---
    // min_cutoff: La "lentezza" del mirino quando l'utente si muove pochissimo (jitter/tremore). 
    // Valori bassi (1.0f) aumentano la stabilità ma inducono latenza "viscosa".
    ///////////////////////const float min_cutoff = 1.0f; 
    // La "lentezza" del mirino quando l'utente si muove pochissimo.
    // Abbassalo per distruggere il jitter da fermo.
    const float min_cutoff = 0.2f; // 0.02f; //0.03f; //0.05f; //0.01f; //0.05f; // 0.1f; // Parti da 0.1f (originale era 1.0f)
    
    // d_cutoff: Reattività del derivato (velocità). 
    // Filtra il "rumore" dal calcolo della velocità stessa prima di usarla per la correzione.
    ///////////////////////////const float d_cutoff = 10.0f;   
    ////// Alzare questo valore rende la percezione dello scatto più immediata.
    ///////const float d_cutoff = 15.0f; // Originale era 10.0f
    // Reattività del derivato (velocità).
    // Abbassalo se il tremore della mano "inganna" il filtro facendogli credere che ti stai muovendo.
    const float d_cutoff = 2.5f; // 10.0f; //8.0f; //5.0f; //3.0f; // Parti da 5.0f (originale era 10.0f)
    
    // max_cutoff: Il limite di banda passante superiore. 
    // Previene reazioni esagerate quando l'arma si sposta violentemente.
    const float max_cutoff = 30.0f; 
    
    // beta_base: Il coefficiente dinamico. Regola quanto aggressivamente l'algoritmo 
    // disattiva il filtro "lento" (min_cutoff) quando percepisce movimenti veloci.
    // L'equazione relaziona la risoluzione della telecamera a quella del mouse.
    /////////////////////////const float beta_base = (0.011f * (float)CamResX) / (float)MouseResX;
    // Aggiungiamo un moltiplicatore di reattività (Tuning Parameter).
    // Valori consigliati: da 2.0f a 5.0f. Più è alto, meno lag c'è nei movimenti veloci.
    const float beta_multiplier = 2.5f; //3.5f; //4.0f; //2.0f; //3.0f; // alzare a 4 o 5 se necessario
    const float beta_base = ((0.011f * (float)CamResX) / (float)MouseResX) * beta_multiplier;
    #endif // COMMENTO
    
    // ==========================================
    // --- PARAMETRI DI TUNING E-SPORTS (BILANCIAMENTO DEFINITIVO) ---

    // min_cutoff: La "lentezza" del mirino quando ti muovi pochissimo o sei fermo.
    // Impostato a 0.1f: il punto di equilibrio perfetto. 0.2f era leggermente 
    // scivoloso, 0.05f era troppo rigido. 0.1f garantisce mira da cecchino solida.
    const float min_cutoff = 0.1f; 
    
    // --- GESTIONE ASIMMETRICA DELLA VELOCITÀ ---
    // d_cutoff_base: Reattività per i movimenti di precisione.
    // RIPORTATO A 1.0f. A 200Hz, un salto di 1 singolo pixel raw della telecamera
    // equivale a 800 pixel/sec! Serve un cutoff basso (1.0f) per spalmare questi salti 
    // quantizzati (0, 800, 0, 800) in una velocità costante e fluida (es. 400).
    const float d_cutoff_base = 1.0f; 
    
    // d_cutoff_snap: Reattività per scatti violenti e frenate brusche.
    const float d_cutoff_snap = 25.0f; 
    
    // snap_base: Il "Punto di Rottura" al CENTRO dello schermo (in pixel/sec).
    // RIPORTATO A 1000.0f. Questo è FONDAMENTALE. Essendo il rumore di quantizzazione
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
    
    // Il passaggio tramite puntatori int* previene costose copie di array per valore,
    // manipolando direttamente la memoria del layer chiamante (Zero Copy).
    void process(int* x, int* y);
};

#endif // OpenFIRE_Multi_One_Euro_Filter_h

#endif // USE_MULTI_ONE_EURO_FILTER