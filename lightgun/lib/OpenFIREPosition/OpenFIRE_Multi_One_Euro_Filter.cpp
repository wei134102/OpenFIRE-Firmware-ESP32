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
        
        // --- 1. LOGICA SPAZIALE ---
        // L'uso di inv_center_x/y basato sul VirtualScope permette di scalare dolcemente 
        // l'Adaptive Beta per smorzare il rumore introdotto dai punti predetti matematicamente.
        float distH = fabsf((float)x[i] - ((float)MouseMaxX * 0.5f)) * inv_center_x;
        float distV = fabsf((float)y[i] - ((float)MouseMaxY * 0.5f)) * inv_center_y;
        float maxDist = (distH > distV) ? distH : distV;
        
        if (maxDist > 1.0f) maxDist = 1.0f; 
        float adaptiveBeta = beta_base * (1.0f - (maxDist * 0.10f));

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
