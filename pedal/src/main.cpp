/*!
 * @file main.cpp
 * @brief main PEDAL
 * @n CPP main PEDAL
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2026
 */


#include <Arduino.h>

#include "TinyUSB_Devices.h"

#include "OpenFIRE-PEDAL-version.h"

// ===================================================================================
// HARDWARE STATE & CONFIGURATION
// ===================================================================================

bool display_init = false;

const int8_t leds[4] = {PIN_LED1, 
                        PIN_LED2, 
                        PIN_LED3, 
                        PIN_LED4}; // I GPIO ove sono collegati i 4 LED

// Manteniamo lo stato di tutti i pedali in un singolo byte (maschera di bit). 
// Questo permette di trasmettere l'intero stato del modulo con un solo byte di payload radio, 
// minimizzando l'overhead di rete e i tempi di volo (Air Time) su ESP-NOW.
uint8_t buttons_state = 0;
volatile uint8_t buttons_last_state = 0;

// ===================================================================================
// GESTIONE KEEP-ALIVE E DEBOUNCING
// ===================================================================================

volatile bool send_packet_pedal = false; // true se bisogna spedire un pacchetto
#define DEBOUNCE_DELAY 15 // tempo di deboincing per pulsanti in ms (forse 20ms è più sicuro)

// Quando un pedale viene tenuto premuto a lungo (es. per ripararsi in Time Crisis), 
// il dongle potrebbe perdere la connessione o interpretare il silenzio radio come disconnessione.
// Questo timer garantisce un reinvio periodico (keep-alive) dello stato attivo.
// NOTA: il valore è trasmesso ogni TIME_REPEAT_SEND solo se qualche buttone è premuto 
// per ridurre al minimo le collisioni. Il valore va specificato in microsecondi quindi ms x 1000
#define TIME_REPEAT_SEND (uint64_t)69000 // tempo in microsendi (impostato a 69ms, forse meglio a 50ms)

#define NUM_BUTTONS 2

// La struttura Button incapsula la Macchina a Stati per il debouncing software.
// L'uso di questa struct permette di scalare il numero di pedali senza dover 
// duplicare la logica nel loop principale.
struct Button {
    int8_t pin;
    bool currentState;
    bool lastState;
    uint8_t debounceTime; // 0 vuole dire che non abbisogna di deboucing
    unsigned long lastDebounceTime;
};

Button buttons[NUM_BUTTONS] = {
    {PIN_PEDAL, LOW, LOW, 0, 0}, // Pedale 1
    {PIN_PEDAL2, LOW, LOW, 0, 0}  // Pedale 2
};


// ===================================================================================
// FEEDBACK VISIVO CON I 4 LED
// ===================================================================================
// L'inizializzazione del layer Wireless è bloccante e può richiedere secondi.
// Sfruttiamo il processore dual-core dell'ESP32 delegando un task indipendente a 
// un FreeRTOS thread. Questo fornisce un feedback visivo immediato (animazione Supercar)
// all'utente, confermando che il pedale è vivo e sta cercando la rete.

void animTaskLink(void *pvParameters) {
   
  int currentLed = 0;
  int direction = 1;

  for (;;) {
    digitalWrite(leds[currentLed], HIGH);
    vTaskDelay(pdMS_TO_TICKS(150)); 
    digitalWrite(leds[currentLed], LOW);
    currentLed += direction;
    if (currentLed >= 3) {
      direction = -1;
    } else if (currentLed <= 0) {
      direction = 1;
    }
  } 
}

// ===============================================================================
// TIMER REINVIO AUTOMATICO (KEEP-ALIVE)
// ===============================================================================
// CALLBACK
// Questa callback viene eseguita in un contesto Hardware Timer interrupt (o task ad 
// alta priorità). Deve essere estremamente breve e non bloccante. 
// Alziamo semplicemente la flag atomica `send_packet_pedal`.
void timer_callback_send_repeat(void* arg) {
  
  if (buttons_last_state) send_packet_pedal = true;
 
}

esp_timer_handle_t timer_handle_send_repeat;

// ===============================================================================
// MAIN SETUP
// ===============================================================================

// The main show!
void setup() {

  // Configurazione Pedali (Ingressi con Pull-Up)
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW); 
  }

  // Creazione del task visivo asincrono.
  TaskHandle_t animTaskHandleLink = NULL;  
  xTaskCreatePinnedToCore(
        animTaskLink,          // funzione del task
        "AnimTaskLink",        // nome
        4096,                  // stack size
        NULL,                  // parametri
        1,                     // priorità
        &animTaskHandleLink,   // handle
        APP_CPU_NUM            // core (puoi usare 0 o 1)
  );  
  
  // ====== gestione connessione wireless ====================
  // Fase bloccante: Il pedale attende qui finché non stabilisce l'handshake ESP-NOW.
  SerialWireless.init_wireless();
  SerialWireless.begin();
  SerialWireless.connection_pedal();
  // ====== fine gestione wireless .. va avanti solo dopo che si è accoppiato il dispositivo =======
    
  // Connessione stabilita. Uccidiamo il task visivo poiché ora il controllo dei LED passa alla logica di sistema.
  if (animTaskHandleLink != NULL) {
      vTaskDelete(animTaskHandleLink);
      animTaskHandleLink = NULL;
  }

  vTaskDelay(pdMS_TO_TICKS(150));

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(leds[i], LOW); 
  }

  // Riscontro visivo di accoppiamento riuscito.
  if (usb_data_wireless.devicePlayer >= 1 && usb_data_wireless.devicePlayer <= 4) {
    digitalWrite(leds[usb_data_wireless.devicePlayer - 1], HIGH); // accende fisso il led del player corrispondente 1,2,3,4
  } else {
    // Gestione errore: accende il primo e l'ultimo led
    digitalWrite(leds[0], HIGH);
    digitalWrite(leds[3], HIGH);
  }
  
  // Inizializzazione Timer Hardware per il keep-alive.
  esp_timer_create_args_t timer_args = {
    .callback = &timer_callback_send_repeat,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "timer_send_repeat"
  };
    
  esp_timer_create(&timer_args, &timer_handle_send_repeat);
  esp_timer_start_periodic(timer_handle_send_repeat, TIME_REPEAT_SEND);
 
}

// ===============================================================================
// MAIN LOOP
// ===============================================================================

void loop()
{
  unsigned long millis_current = millis(); 
  
  uint8_t bitMask = 1;
  buttons_state = 0;

  // SCANSIONE PEDALI E DEBOUNCING NON-BLOCCANTE
  for (uint8_t i = 0; i < NUM_BUTTONS; i++, bitMask <<= 1) {
    if (!buttons[i].debounceTime) {
      // Pedale a riposo (o stato stabile raggiunto). Leggiamo l'hardware.
      buttons[i].currentState = buttons[i].pin >= 0 ? !(bool)digitalRead(buttons[i].pin) : false;
      
      if (buttons[i].currentState) {
        buttons_state |= bitMask; 
      }
      
      // Se c'è un cambiamento fisico, inneschiamo la finestra di debounce.
      // In questo periodo di "cecità" volontaria ignoreremo i rimbalzi meccanici dello switch.
      if (buttons[i].currentState != buttons[i].lastState) {
        buttons[i].debounceTime = DEBOUNCE_DELAY;
        buttons[i].lastDebounceTime = millis_current; 
        buttons[i].lastState = buttons[i].currentState;
      }
    }
    else {
      // Finestra di debounce attiva. Riduciamo il timer scalando il delta temporale trascorso (`aux`).
      unsigned long aux = millis_current - buttons[i].lastDebounceTime;
      
      if (aux >= buttons[i].debounceTime) {
          buttons[i].debounceTime = 0;
      } else {
          buttons[i].debounceTime -= aux; 
          buttons[i].lastDebounceTime = millis_current;
      }
      
      // Manteniamo lo stato consolidato in attesa che il rimbalzo hardware finisca.
      if (buttons[i].lastState) {
        buttons_state |= bitMask; 
      }  
    }
  }

  // LOGICA EVENT-DRIVEN DI TRASMISSIONE RADIO
  // Trasmettiamo sulla rete ESCLUSIVAMENTE in due scenari:
  // A) È avvenuto un cambio di stato fisico (salita/discesa di un pedale).
  // B) Il timer hardware ha alzato la flag per il keep-alive (pedale mantenuto premuto).
  if ((buttons_state != buttons_last_state) || (send_packet_pedal == true)) {
    // Invia pacchetto con posizione pedali alla lightgun - invia il byte buttons_state
 
    SerialWireless.SendPacket((const uint8_t *)&buttons_state, 1, PACKET_TX::PEDAL_TX);

    buttons_last_state = buttons_state;
     
    esp_timer_restart(timer_handle_send_repeat, TIME_REPEAT_SEND);
     
    send_packet_pedal = false;

  }
      
  // Rate limiting volontario per cedere CPU al layer WiFi/ESP-NOW sottostante.
  vTaskDelay(pdMS_TO_TICKS(5));  // fai polling ogni 5ms
}