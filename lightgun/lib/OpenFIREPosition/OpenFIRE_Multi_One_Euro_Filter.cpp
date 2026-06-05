#ifdef USE_MULTI_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_Multi_One_Euro_Filter.cpp
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

#include "OpenFIRE_Multi_One_Euro_Filter.h"

OpenFIRE_One_Euro_Multi::OpenFIRE_One_Euro_Multi() {
    lastMicros = 0;
    initialized = false;
    for(int i = 0; i < 4; i++) {
        states[i] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    // PRE-CALCOLO ASSOLUTO: Eseguito una sola volta all'avvio.
    // Usiamo un Raggio d'Azione Virtuale per non castrare l'Adaptive Beta 
    // quando riceviamo punti stimati matematicamente molto fuori dallo schermo.
    const float VirtualScopeMultiplier = 3.0f; 
    const float VirtualScopeX = (float)MouseMaxX * VirtualScopeMultiplier;
    const float VirtualScopeY = (float)MouseMaxY * VirtualScopeMultiplier;
    
    // Troviamo i reciproci basati sullo spazio virtuale esteso
    inv_center_x = 1.0f / (VirtualScopeX * 0.5f);
    inv_center_y = 1.0f / (VirtualScopeY * 0.5f);
}

void OpenFIRE_One_Euro_Multi::process(int* x, int* y) {
    // Calcolo del Delta-Time (dt) tramite il micro-clock dell'ESP32.
    // L'aritmetica unsigned assorbe nativamente l'overflow dopo i 71 minuti.
    unsigned long currentMicros = micros();
    float dt = ((float)(currentMicros - lastMicros)) * 0.000001f; 
    lastMicros = currentMicros;

    // Protezione base per divisioni per zero o blocchi anomali
    if (dt <= 0.0f) dt = 0.005f; 

    // Cold start dinamico: se passa troppo tempo (es. perdita temporanea del tracking),
    // forziamo il reset per evitare di calcolare velocità inerziali spaziali.
    if (dt > 0.1f) {
        initialized = false;
    }

    // Inizializzazione al primissimo frame valido
    if (!initialized) {
        for (int i = 0; i < 4; i++) {
            states[i].x_prev = states[i].x_hat = (float)x[i];
            states[i].y_prev = states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = states[i].vel_y_hat = 0.0f;
            x[i] = (int)roundf(states[i].x_hat);
            y[i] = (int)roundf(states[i].y_hat);
        }
        initialized = true;
        return;
    }

    const float dt_two_pi = dt * OEF_TWO_PI;
    
    // --- GESTIONE ASIMMETRICA DELLA DERIVATA ---
    // Calcoliamo i due estremi di reazione usando i parametri definiti nell'header.
    // a_d_base: Fluido. Assorbe i microscatti hardware durante il panning lento.
    // a_d_snap: Brutale. Allinea la velocità calcolata in pochi millisecondi.
    const float a_d_base = fast_alpha(d_cutoff_base, dt_two_pi);  
    const float a_d_snap = fast_alpha(d_cutoff_snap, dt_two_pi); 
    
    const float inv_dt = 1.0f / dt; 

    // Core Loop: processiamo i 4 LED in parallelo mantenendo separate le inerzie
    for (int i = 0; i < 4; i++) {
        
        // --- 1. LOGICA SPAZIALE (CURVA QUADRATICA E-SPORTS) ---
        // Utilizziamo i reciproci del centro schermo pre-calcolati al boot per 
        // trasformare pesanti divisioni in moltiplicazioni ultra-veloci.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        
        // Calcolo distanza basato sul quadrato (evita costosi cicli di sqrtf())
        float maxDistSq = (distH * distH) + (distV * distV);
        float edge_attenuation = 1.0f - (maxDistSq * 1.5f);
        
        // Sicurezza in caso di coordinate negative o fortemente fuori schermo
        if (edge_attenuation < 0.20f) edge_attenuation = 0.20f; 
        float adaptiveBeta = beta_base * edge_attenuation;

        // --- 2. FILTRAGGIO ASSE X ---
        float dx = ((float)x[i] - states[i].x_prev) * inv_dt;
        
        // Switch dinamico della derivata (Asse X)
        // Innesca la reattività fulminea se la variazione istantanea supera il Punto di Rottura
        float a_d_current_x = a_d_base;
        if (fabsf(dx - states[i].vel_x_hat) > snap_threshold) {
            a_d_current_x = a_d_snap; 
        }
        
        states[i].vel_x_hat += a_d_current_x * (dx - states[i].vel_x_hat);
        
        float cutoff_x = min_cutoff + adaptiveBeta * fabsf(states[i].vel_x_hat);
        if (cutoff_x > max_cutoff) cutoff_x = max_cutoff; 
        
        float a_x = fast_alpha(cutoff_x, dt_two_pi);
        
        states[i].x_hat += a_x * ((float)x[i] - states[i].x_hat);
        states[i].x_prev = (float)x[i];

        // --- 3. FILTRAGGIO ASSE Y ---
        float dy = ((float)y[i] - states[i].y_prev) * inv_dt;
        
        // Switch dinamico della derivata (Asse Y)
        // L'indipendenza dall'asse X garantisce stabilità verticale nei flick orizzontali
        float a_d_current_y = a_d_base;
        if (fabsf(dy - states[i].vel_y_hat) > snap_threshold) {
            a_d_current_y = a_d_snap; 
        }
        
        states[i].vel_y_hat += a_d_current_y * (dy - states[i].vel_y_hat);
        
        float cutoff_y = min_cutoff + adaptiveBeta * fabsf(states[i].vel_y_hat);
        if (cutoff_y > max_cutoff) cutoff_y = max_cutoff;
        
        float a_y = fast_alpha(cutoff_y, dt_two_pi);
        
        states[i].y_hat += a_y * ((float)y[i] - states[i].y_hat);
        states[i].y_prev = (float)y[i];

        // --- 4. MICRO-SNAP (TAGLIO DELLA CODA ASINTOTICA BLINDATO) ---
        // Interviene esclusivamente al Dead Stop assoluto: la telecamera non rileva 
        // alcuno spostamento fisico dal frame precedente (dx/dy == 0) e il target 
        // è ormai a meno di mezzo pixel di distanza.
        if (dx == 0.0f && fabsf((float)x[i] - states[i].x_hat) < 0.5f) {
            states[i].x_hat = (float)x[i];
        }
        if (dy == 0.0f && fabsf((float)y[i] - states[i].y_hat) < 0.5f) {
            states[i].y_hat = (float)y[i];
        }

        // --- 5. SAFETY NET HARDWARE ---
        // Protezione contro Not-a-Number (NaN) derivanti da divisioni corrotte.
        // Forza le coordinate attuali per prevenire il blocco completo del tracciamento.
        if (isnan(states[i].x_hat) || isnan(states[i].y_hat)) {
            states[i].x_hat = (float)x[i];
            states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = 0.0f;
            states[i].vel_y_hat = 0.0f;
        }

        // Conversione finale nel dominio intero in-place per il protocollo HID del mouse
        x[i] = (int)roundf(states[i].x_hat);
        y[i] = (int)roundf(states[i].y_hat);
    }
}

#ifdef COMMENTO
void OpenFIRE_One_Euro_Multi::process(int* x, int* y) {
    unsigned long currentMicros = micros();
    float dt = ((float)(currentMicros - lastMicros)) * 0.000001f; 
    lastMicros = currentMicros;

    if (dt <= 0.0f) dt = 0.005f; 

    if (dt > 0.1f) {
        initialized = false;
    }

    if (!initialized) {
        for (int i = 0; i < 4; i++) {
            states[i].x_prev = states[i].x_hat = (float)x[i];
            states[i].y_prev = states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = states[i].vel_y_hat = 0.0f;
            x[i] = (int)roundf(states[i].x_hat);
            y[i] = (int)roundf(states[i].y_hat);
        }
        initialized = true;
        return;
    }

    const float dt_two_pi = dt * OEF_TWO_PI;
    const float a_d = fast_alpha(d_cutoff, dt_two_pi);
    const float inv_dt = 1.0f / dt; 

    for (int i = 0; i < 4; i++) {
        
        // --- 1. LOGICA SPAZIALE (CURVA QUADRATICA E-SPORTS) ---
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        
        float maxDistSq = (distH * distH) + (distV * distV);
        float edge_attenuation = 1.0f - (maxDistSq * 1.5f);
        
        if (edge_attenuation < 0.20f) edge_attenuation = 0.20f; 
        float adaptiveBeta = beta_base * edge_attenuation;

        // --- 2. FILTRAGGIO ASSE X ---
        float dx = ((float)x[i] - states[i].x_prev) * inv_dt;
        states[i].vel_x_hat += a_d * (dx - states[i].vel_x_hat);
        
        float cutoff_x = min_cutoff + adaptiveBeta * fabsf(states[i].vel_x_hat);
        if (cutoff_x > max_cutoff) cutoff_x = max_cutoff; 
        
        float a_x = fast_alpha(cutoff_x, dt_two_pi);
        
        states[i].x_hat += a_x * ((float)x[i] - states[i].x_hat);
        states[i].x_prev = (float)x[i];

        // --- 3. FILTRAGGIO ASSE Y ---
        float dy = ((float)y[i] - states[i].y_prev) * inv_dt;
        states[i].vel_y_hat += a_d * (dy - states[i].vel_y_hat);
        
        float cutoff_y = min_cutoff + adaptiveBeta * fabsf(states[i].vel_y_hat);
        if (cutoff_y > max_cutoff) cutoff_y = max_cutoff;
        
        float a_y = fast_alpha(cutoff_y, dt_two_pi);
        
        states[i].y_hat += a_y * ((float)y[i] - states[i].y_hat);
        states[i].y_prev = (float)y[i];

        #ifdef COMMENTO
        // --- 4. MICRO-SNAP (TAGLIO DELLA CODA ASINTOTICA) ---
        // Elimina l'asintoto infinito causato dal min_cutoff estremo (0.02f).
        // Se la distanza residua è < 0.5 pixel e la velocità è quasi nulla,
        // forziamo l'allineamento perfetto. Questo ferma l'invio continuo 
        // di report USB ridondanti, liberando il mouse fisico del PC.
        if (fabsf((float)x[i] - states[i].x_hat) < 0.5f && fabsf(states[i].vel_x_hat) < 5.0f) {
            states[i].x_hat = (float)x[i];
        }
        if (fabsf((float)y[i] - states[i].y_hat) < 0.5f && fabsf(states[i].vel_y_hat) < 5.0f) {
            states[i].y_hat = (float)y[i];
        }
        #endif // COMMENTO

        // --- 4. MICRO-SNAP (TAGLIO DELLA CODA ASINTOTICA) ---
        // Condizione blindata: interveniamo SOLO se la telecamera ha smesso fisicamente
        // di registrare variazioni (dx e dy grezzi sono zero) e siamo a meno di 0.5 pixel dal target.
        // Questo impedisce categoricamente al Micro-Snap di "scattare" durante i movimenti lenti.
        if (dx == 0.0f && fabsf((float)x[i] - states[i].x_hat) < 0.5f) {
            states[i].x_hat = (float)x[i];
        }
        if (dy == 0.0f && fabsf((float)y[i] - states[i].y_hat) < 0.5f) {
            states[i].y_hat = (float)y[i];
        }

        // --- 5. SAFETY NET HARDWARE ---
        if (isnan(states[i].x_hat) || isnan(states[i].y_hat)) {
            states[i].x_hat = (float)x[i];
            states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = 0.0f;
            states[i].vel_y_hat = 0.0f;
        }

        x[i] = (int)roundf(states[i].x_hat);
        y[i] = (int)roundf(states[i].y_hat);
    }
}
#endif // COMMENTO

#ifdef COMMENTO
void OpenFIRE_One_Euro_Multi::process(int* x, int* y) {
    // Il calcolo del Delta-Time (dt) si basa sul micro-clock dell'ESP32. 
    // Questa precisione temporale rende il filtro intrinsecamente immune 
    // alle variazioni di framerate causate da eventuali picchi di carico sulla CPU.
    unsigned long currentMicros = micros();
    
    float dt = ((float)(currentMicros - lastMicros)) * 0.000001f; 
    lastMicros = currentMicros;

    // Se il timer impazzisce (o al primissimo ciclo), resettiamo il dt 
    // a un valore nominale per evitare divisioni per zero.
    if (dt <= 0.0f) dt = 0.005f; 

    // COLD START DINAMICO: Se passa troppo tempo (>100ms), ad esempio per la perdita 
    // di tracciamento, reset per evitare velocità inerziali fittizie.
    if (dt > 0.1f) {
        initialized = false;
    }

    if (!initialized) {
        // Cold start (Aggancio Immediato): evita che l'interpolatore inizi a calcolare
        // da zero, spingendo la matematica istantaneamente dove si trova l'arma.
        for (int i = 0; i < 4; i++) {
            states[i].x_prev = states[i].x_hat = (float)x[i];
            states[i].y_prev = states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = states[i].vel_y_hat = 0.0f;
            // I puntatori x[i] e y[i] contengono già il dato grezzo corretto,
            // non sprechiamo cicli di clock per ri-arrotondarlo e ri-assegnarlo.
        }
        initialized = true;
        return; // Usciamo subito: per questo primo frame il dato grezzo è l'output ottimale.
    }

    const float dt_two_pi = dt * OEF_TWO_PI;
    const float a_d = fast_alpha(d_cutoff, dt_two_pi);
    const float inv_dt = 1.0f / dt; 

    // Il core loop. Trattiamo indipendentemente i 4 angoli del quadrilatero.
    for (int i = 0; i < 4; i++) {
        
        #ifdef COMMENTO
        // --- 1. LOGICA SPAZIALE ---
        // L'uso di inv_center_x/y basato sul VirtualScope permette di scalare dolcemente 
        // l'Adaptive Beta per smorzare il rumore introdotto dai punti predetti matematicamente.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        
        float maxDist = (distH > distV) ? distH : distV;    
        if (maxDist > 1.0f) maxDist = 1.0f; 
        float adaptiveBeta = beta_base * (1.0f - (maxDist * 0.10f));
        #endif // COMMENTO

        // --- 1. LOGICA SPAZIALE (CURVA QUADRATICA E-SPORTS) ---
        // distH e distV usano fabsf() e sono sempre positivi, anche per coordinate come -8000 o +12000.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        
        // OTTIMIZZAZIONE ESTREMA: Usiamo la distanza al quadrato (Pitagora senza radice).
        // 1. Risparmiamo i cicli hardware della funzione sqrtf(), rendendo il codice fulmineo a 200Hz.
        // 2. Creiamo un "Plateau" perfetto: al centro la reattività resta intatta al 100%, 
        //    per poi crollare vertiginosamente solo negli angoli e nel fuori-schermo.
        float maxDistSq = (distH * distH) + (distV * distV);
        
        // Al centro (maxDistSq ~ 0.0) -> attenuazione = 1.0 (Reattività esplosiva al 100%)
        // Ai bordi laterali (maxDistSq ~ 0.11) -> attenuazione = 0.83 (Reattività 83%)
        // Agli angoli estremi TV (maxDistSq ~ 0.22) -> attenuazione = 0.67 (Reattività 67% - addio tremore)
        // Fuori schermo folle (maxDistSq > 0.6) -> l'attenuazione precipita verso lo zero (bloccata dal clamp).
        float edge_attenuation = 1.0f - (maxDistSq * 1.5f);
        
        // CLAMP DI SICUREZZA PER PUNTI STIMATI/FUORI SCHERMO: 
        // Quando i LED escono dal sensore e la matematica inventa coordinate estreme (es. X = -4095), 
        // l'attenuazione diventa negativa. Noi la inchiodiamo al 20%. 
        // In questo modo, i punti stimati diventano "viscosi", ignorando i glitch matematici, 
        // ma permettendo al fucile di rientrare fluidamente nello schermo.
        if (edge_attenuation < 0.20f) edge_attenuation = 0.20f; 
        
        float adaptiveBeta = beta_base * edge_attenuation;

        // --- 2. FILTRAGGIO ASSE X ---
        float dx = ((float)x[i] - states[i].x_prev) * inv_dt;
        
        // OTTIMIZZAZIONE EMA: Forma matematica ridotta
        states[i].vel_x_hat += a_d * (dx - states[i].vel_x_hat);
        
        float cutoff_x = min_cutoff + adaptiveBeta * fabsf(states[i].vel_x_hat);
        if (cutoff_x > max_cutoff) cutoff_x = max_cutoff; 
        
        float a_x = fast_alpha(cutoff_x, dt_two_pi);
        
        states[i].x_hat += a_x * ((float)x[i] - states[i].x_hat);
        states[i].x_prev = (float)x[i];

        // --- 3. FILTRAGGIO ASSE Y ---
        float dy = ((float)y[i] - states[i].y_prev) * inv_dt;
        
        states[i].vel_y_hat += a_d * (dy - states[i].vel_y_hat);
        
        float cutoff_y = min_cutoff + adaptiveBeta * fabsf(states[i].vel_y_hat);
        if (cutoff_y > max_cutoff) cutoff_y = max_cutoff;
        
        float a_y = fast_alpha(cutoff_y, dt_two_pi);
        
        states[i].y_hat += a_y * ((float)y[i] - states[i].y_hat);
        states[i].y_prev = (float)y[i];

        // --- 4. SAFETY NET HARDWARE ---
        // Se un glitch cosmico genera un Not-A-Number (NaN), lo resettiamo 
        // istantaneamente forzando le coordinate grezze attuali per non bloccare l'asse.
        if (isnan(states[i].x_hat) || isnan(states[i].y_hat)) {
            states[i].x_hat = (float)x[i];
            states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = 0.0f;
            states[i].vel_y_hat = 0.0f;
        }

        // Il casting esplicito a intero post-arrotondamento riporta il dato al dominio
        // del protocollo HID del mouse in-place, pronto per la comunicazione USB.
        x[i] = (int)roundf(states[i].x_hat);
        y[i] = (int)roundf(states[i].y_hat);
    }
}
#endif // COMMENTO
#endif // USE_MULTI_ONE_EURO_FILTER

#ifdef COMMENTO
#ifdef USE_MULTI_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_Multi_One_Euro_Filter.cpp
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

#include "OpenFIRE_Multi_One_Euro_Filter.h"

OpenFIRE_One_Euro_Multi::OpenFIRE_One_Euro_Multi() {
    lastMicros = 0;
    initialized = false;
    for(int i = 0; i < 4; i++) {
        states[i] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    // PRE-CALCOLO ASSOLUTO: Eseguito una sola volta all'avvio.
    // Usiamo un Raggio d'Azione Virtuale per non castrare l'Adaptive Beta 
    // quando riceviamo punti stimati matematicamente molto fuori dallo schermo.
    const float VirtualScopeMultiplier = 3.0f; 
    const float VirtualScopeX = (float)MouseMaxX * VirtualScopeMultiplier;
    const float VirtualScopeY = (float)MouseMaxY * VirtualScopeMultiplier;
    
    // Troviamo i reciproci basati sullo spazio virtuale esteso
    inv_center_x = 1.0f / (VirtualScopeX * 0.5f);
    inv_center_y = 1.0f / (VirtualScopeY * 0.5f);
}

void OpenFIRE_One_Euro_Multi::process(int* x, int* y) {
    // Il calcolo del Delta-Time (dt) si basa sul micro-clock dell'ESP32. 
    // Questa precisione temporale rende il filtro intrinsecamente immune 
    // alle variazioni di framerate causate da eventuali picchi di carico sulla CPU.
    unsigned long currentMicros = micros();
    
    float dt = ((float)(currentMicros - lastMicros)) * 0.000001f; 
    lastMicros = currentMicros;

    // Se il timer impazzisce resettiamo il dt a un valore nominale
    // per evitare divisioni per zero.
    if (dt <= 0.0f) dt = 0.005f; 

    // COLD START DINAMICO: Se passa troppo tempo (>100ms), ad esempio per la perdita 
    // di tracciamento o pausa, forziamo il reset per evitare di inquinare 
    // le derivate spaziali e causare micro-jitter al rientro.
    if (dt > 0.1f) {
        initialized = false;
    }

    if (!initialized) {
        // Cold start (Aggancio Immediato): evita che l'interpolatore inizi a calcolare
        // da zero (origine asse), spingendo le coordinate istantaneamente dove si trova l'arma.
        for (int i = 0; i < 4; i++) {
            states[i].x_prev = states[i].x_hat = (float)x[i];
            states[i].y_prev = states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = states[i].vel_y_hat = 0.0f;
            
            // Applichiamo immediatamente i valori arrotondati agli array d'uscita 
            // per evitare di restituire dati sporchi nel primo frame di rientro.
            x[i] = (int)roundf(states[i].x_hat);
            y[i] = (int)roundf(states[i].y_hat);
        }
        initialized = true;
        return;
    }

    const float dt_two_pi = dt * OEF_TWO_PI;
    const float a_d = fast_alpha(d_cutoff, dt_two_pi);
    const float inv_dt = 1.0f / dt; 

    // Il core loop. Trattiamo indipendentemente i 4 angoli del quadrilatero per non 
    // mescolare le singole inerzie geometriche.
    for (int i = 0; i < 4; i++) {
        
        // --- 1. LOGICA SPAZIALE ---
        // Adattamento periferico: man mano che ci allontaniamo dal centro,
        // la deformazione ottica (o l'errore dei punti stimati) amplifica il rumore. 
        // L'uso di inv_center_x/y basato sul VirtualScope permette di scalare
        // dolcemente questa sensibilità anche oltre i confini fisici del monitor.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        float maxDist = (distH > distV) ? distH : distV;
        
        if (maxDist > 1.0f) maxDist = 1.0f; 
        float adaptiveBeta = beta_base * (1.0f - (maxDist * 0.10f));

        // --- 2. FILTRAGGIO ASSE X ---
        float dx = ((float)x[i] - states[i].x_prev) * inv_dt;
        
        // OTTIMIZZAZIONE EMA: Forma matematica ridotta (risparmia CPU)
        states[i].vel_x_hat += a_d * (dx - states[i].vel_x_hat);
        
        float cutoff_x = min_cutoff + adaptiveBeta * fabsf(states[i].vel_x_hat);
        if (cutoff_x > max_cutoff) cutoff_x = max_cutoff; 
        
        float a_x = fast_alpha(cutoff_x, dt_two_pi);
        
        states[i].x_hat += a_x * ((float)x[i] - states[i].x_hat);
        states[i].x_prev = (float)x[i];

        // --- 3. FILTRAGGIO ASSE Y ---
        // Mirroring speculare della logica dell'asse X. 
        float dy = ((float)y[i] - states[i].y_prev) * inv_dt;
        
        states[i].vel_y_hat += a_d * (dy - states[i].vel_y_hat);
        
        float cutoff_y = min_cutoff + adaptiveBeta * fabsf(states[i].vel_y_hat);
        if (cutoff_y > max_cutoff) cutoff_y = max_cutoff;
        
        float a_y = fast_alpha(cutoff_y, dt_two_pi);
        
        states[i].y_hat += a_y * ((float)y[i] - states[i].y_hat);
        states[i].y_prev = (float)y[i];

        // --- 4. SAFETY NET HARDWARE ---
        // Se un glitch cosmico genera un Not-A-Number, lo resettiamo 
        // istantaneamente forzando le coordinate attuali grezze. 
        if (isnan(states[i].x_hat) || isnan(states[i].y_hat)) {
            states[i].x_hat = (float)x[i];
            states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = 0.0f;
            states[i].vel_y_hat = 0.0f;
        }

        // Il casting esplicito a intero post-arrotondamento riporta il dato al dominio
        // del protocollo HID del mouse, pronto per la comunicazione USB/Bluetooth.
        x[i] = (int)roundf(states[i].x_hat);
        y[i] = (int)roundf(states[i].y_hat);
    }
}

#endif // USE_MULTI_ONE_EURO_FILTER
#endif

#ifdef COMMENTO
#ifdef USE_MULTI_ONE_EURO_FILTER
/*!
 * @file OpenFIRE_Multi_One_Euro_Filter.cpp
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

#include "OpenFIRE_Multi_One_Euro_Filter.h"

OpenFIRE_One_Euro_Multi::OpenFIRE_One_Euro_Multi() {
    lastMicros = 0;
    initialized = false;
    for(int i = 0; i < 4; i++) {
        states[i] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    // PRE-CALCOLO ASSOLUTO: Eseguito una sola volta all'avvio.
    // Troviamo i reciproci del centro schermo per usare moltiplicazioni iper-veloci.
    inv_center_x = 1.0f / ((float)MouseMaxX * 0.5f);
    inv_center_y = 1.0f / ((float)MouseMaxY * 0.5f);
}

void OpenFIRE_One_Euro_Multi::process(int* x, int* y) {
    // Il calcolo del Delta-Time (dt) si basa sul micro-clock dell'ESP32. 
    // Questa precisione temporale rende il filtro intrinsecamente immune 
    // alle variazioni di framerate causate da eventuali picchi di carico sulla CPU.
    unsigned long currentMicros = micros();
    
    float dt = ((float)(currentMicros - lastMicros)) * 0.000001f; 
    
    // Se il timer impazzisce o il ciclo si blocca per il Wi-Fi, resettiamo il dt
    // a un valore nominale per evitare divisioni per zero o salti catastrofici del filtro.
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f; 
    lastMicros = currentMicros;

    if (!initialized) {
        // Cold start (Aggancio Immediato): evita che l'interpolatore inizi a calcolare
        // da zero (origine asse), spingendo le coordinate istantaneamente dove si trova l'arma.
        for (int i = 0; i < 4; i++) {
            states[i].x_prev = states[i].x_hat = (float)x[i];
            states[i].y_prev = states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = states[i].vel_y_hat = 0.0f;
        }
        initialized = true;
        return;
    }

    const float dt_two_pi = dt * OEF_TWO_PI;
    const float a_d = fast_alpha(d_cutoff, dt_two_pi);
    const float inv_dt = 1.0f / dt; 

    // Il core loop. Trattiamo indipendentemente i 4 angoli del quadrilatero per non 
    // mescolare le singole inerzie geometriche.
    for (int i = 0; i < 4; i++) {
        
        // --- 1. LOGICA SPAZIALE ---
        // Sfruttiamo i centri calcolati nel costruttore
        // Adattamento periferico: man mano che ci allontaniamo dal centro della TV,
        // la deformazione ottica amplifica il rumore. Questa logica riduce la sensibilità 
        // del filtro (Beta) dinamicamente ai bordi per contrastare l'instabilità fisiologica.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        float maxDist = (distH > distV) ? distH : distV;
        
        if (maxDist > 1.0f) maxDist = 1.0f; 
        float adaptiveBeta = beta_base * (1.0f - (maxDist * 0.10f));

        // --- 2. FILTRAGGIO ASSE X ---
        float dx = ((float)x[i] - states[i].x_prev) * inv_dt;
        
        // OTTIMIZZAZIONE EMA: Forma matematica ridotta (risparmia CPU)
        // Matematica equivalente: y_hat = a_d * dx + (1 - a_d) * vel_x_hat, 
        // ma evita una moltiplicazione per ciclo raggruppando i fattori.
        states[i].vel_x_hat += a_d * (dx - states[i].vel_x_hat);
        
        float cutoff_x = min_cutoff + adaptiveBeta * fabsf(states[i].vel_x_hat);
        if (cutoff_x > max_cutoff) cutoff_x = max_cutoff; 
        
        float a_x = fast_alpha(cutoff_x, dt_two_pi);
        
        // OTTIMIZZAZIONE EMA: Forma matematica ridotta
        states[i].x_hat += a_x * ((float)x[i] - states[i].x_hat);
        states[i].x_prev = (float)x[i];

        // --- 3. FILTRAGGIO ASSE Y ---
        // Mirroring speculare della logica dell'asse X. Separazione necessaria per
        // permettere movimenti rapidi in orizzontale preservando la stabilità verticale.
        float dy = ((float)y[i] - states[i].y_prev) * inv_dt;
        
        states[i].vel_y_hat += a_d * (dy - states[i].vel_y_hat);
        
        float cutoff_y = min_cutoff + adaptiveBeta * fabsf(states[i].vel_y_hat);
        if (cutoff_y > max_cutoff) cutoff_y = max_cutoff;
        
        float a_y = fast_alpha(cutoff_y, dt_two_pi);
        
        states[i].y_hat += a_y * ((float)y[i] - states[i].y_hat);
        states[i].y_prev = (float)y[i];

        // --- 4. SAFETY NET HARDWARE ---
        // Se un glitch cosmico (es. divisione fallita) genera un Not-A-Number, lo resettiamo 
        // istantaneamente forzando le coordinate attuali grezze. Evita il blocco totale dell'asse.
        if (isnan(states[i].x_hat) || isnan(states[i].y_hat)) {
            states[i].x_hat = (float)x[i];
            states[i].y_hat = (float)y[i];
            states[i].vel_x_hat = 0.0f;
            states[i].vel_y_hat = 0.0f;
        }

        // Il casting esplicito a intero post-arrotondamento riporta il dato al dominio
        // del protocollo HID del mouse, pronto per la comunicazione USB/Bluetooth.
        x[i] = (int)roundf(states[i].x_hat);
        y[i] = (int)roundf(states[i].y_hat);
    }
}

#endif // USE_MULTI_ONE_EURO_FILTER
#endif
