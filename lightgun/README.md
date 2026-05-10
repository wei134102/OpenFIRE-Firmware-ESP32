> [!WARNING]
> **🚧 Work in Progress 🚧**
> 
> This documentation is currently being written. The information provided may be partial, incomplete, or subject to change. Please check back soon!


---

> [!WARNING]
> **🚧 Documentazione in Lavorazione 🚧**
> 
> Questo file è attualmente in fase di stesura. Le informazioni contenute potrebbero essere parziali, incomplete o soggette a modifiche. Torna a visitarlo presto per la versione definitiva!

---
---
---

> [!WARNING]
> **🚧 Documentazione in Lavorazione 🚧**
> 
> Questo file è attualmente in fase di stesura.

# 🔫 Lightgun Firmware (ESP32-S3)

Questa sezione contiene la documentazione specifica per la costruzione e la programmazione del modulo principale della Lightgun basato su ESP32-S3.

---

## 🛠️ Requisiti Hardware

La flessibilità di OpenFIRE ti permette di costruire una lightgun che va dalla configurazione più basilare a un sistema arcade completo di ogni feedback.

### Componenti Obbligatori (Essenziali)
Per il funzionamento di base del sistema di puntamento e sparo, sono indispensabili:

* **Microcontrollore:** Un **ESP32-S3** per eseguire il firmware. Le schede consigliate e testate sono:
  * *ESP32-S3-WROOM1-DevKitC-1*
  * *Waveshare ESP32-S3-PICO*
* **Sensore Ottico:** Telecamera di posizionamento IR **DFRobot SEN0158**. *(In alternativa, è possibile utilizzare una **Wii CAM** originale, configurando un pin specifico del microcontrollore per generare il segnale di clock necessario al suo funzionamento).*
* **Emettitori IR:** 4x LED Infrarossi. Sebbene le normali barre sensore della Wii possano funzionare a distanze molto ravvicinate, è **ALTAMENTE consigliato** l'utilizzo di LED OSRAM SFH 4547 con resistenze da 5.6Ω.
* **Input Primario:** Almeno 1 interruttore da utilizzare come grilletto (Trigger).

### Componenti Altamente Consigliati
Sebbene la lightgun possa funzionare con il solo grilletto, per poter fruire appieno della quasi totalità dei giochi retrogame per lightgun si consiglia caldamente di aggiungere:
* **Pulsanti A e B:** 2 interruttori aggiuntivi per gestire le ricariche o le azioni secondarie.

### Moduli Opzionali (Controlli Avanzati e Feedback)
Il firmware può gestire nativamente fino a 13 interruttori aggiuntivi (oltre al grilletto) e input analogici, tutti completamente rimappabili tramite la OpenFIRE Desktop App. Sentiti libero di integrare i componenti che preferisci per la tua build personalizzata:

* **Input Aggiuntivi:**
  * Un ulteriore pulsante C.
  * Modulo di navigazione direzionale (D-Pad a 5 vie) + 2 pulsanti per Start e Select.
  * Switch per la ricarica a pompa (Pump action).
  * Joystick analogico a 2 assi.
  * Interruttori SPDT a 2 vie: per regolare via hardware l'attivazione di rumble, solenoide o fuoco rapido *(tali funzioni rimangono comunque attivabili via software).*
* **Feedback di Forza (Rinculo):** Qualsiasi solenoide a 12V-24V con relativa scheda driver MOSFET.
  * *Nota per build Wireless:* Richiede un'alimentazione separata (es. trasformatore 12V-24V). Per le build 100% senza fili con solenoide a 12V, è possibile utilizzare una batteria Li-ion da 3.7V (sia formato 18650 che 21700, ma la **21700** è altamente consigliata per garantire una durata accettabile visti i consumi elevati) ad alta scarica (minimo 10A) abbinata a un modulo step-up/booster potente come l'**MP3429** (3.7V -> 12V).
  * *Controllo Termico:* Si consiglia un sensore di temperatura **TMP36** da posizionare sul solenoide per monitorare ed evitare surriscaldamenti durante le lunghe sessioni.
* **Feedback Aptico:** Qualsiasi motore rumble per gamepad a 5V con relativa scheda driver di base.
* **Feedback Visivo:** * Display OLED 128x64 I2C (SSD1306) per interfaccia utente, menu e contatori (vita/munizioni).
  * LED RGB (NeoPixel WS2812B o moduli a 4 pin) per l'illuminazione dinamica e le reazioni in-game.

### Risorse e Guide all'Assemblaggio
Puoi trovare link per l'acquisto dei componenti e istruzioni di montaggio dettagliate all'interno del mio progetto hardware **[PICON-AS](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/)**.

Allego inoltre alcune guide grafiche per l'autocostruzione e la connessione dei componenti, originariamente fornite dal progetto OpenFIRE. *Nota bene: sebbene le immagini mostrino i collegamenti su un Raspberry Pi Pico (RP2040), la logica di cablaggio è identica e applicabile a qualsiasi pin configurato dell'ESP32-S3.*
* [Guida Componenti OpenFIRE](docs/NOME_FILE_GUIDA.pdf)
* [Guida Cablaggio Wii CAM](docs/NOME_FILE_WII.jpg)

---

## 📌 Pinout di Default delle Board

Fai riferimento alle seguenti immagini per i pinout predefiniti delle due schede consigliate. 
> **Attenzione:** Ogni scheda supporta layout completamente personalizzati. Se decidi di saldare i componenti su pin differenti per comodità di cablaggio, potrai riassegnarli liberamente tramite la **OpenFIRE Desktop App**.

| ESP32-S3-DevKitC-1 | Waveshare ESP32-S3-PICO |
| :---: | :---: |
| <img src="docs/NOME_FILE_DEVKIT.png" width="100%" alt="Pinout DevKitC-1"> | <img src="docs/NOME_FILE_PICO.png" width="100%" alt="Pinout S3-PICO"> |

---

## 💻 Installazione e Flashing del Firmware

A differenza dei microcontrollori RP2040 (che utilizzano il drag-and-drop di file `.UF2`), l'ESP32-S3 richiede il caricamento di file binari (`.bin`) tramite comunicazione seriale. 

Per rendere questa operazione il più semplice possibile, nella pagina delle **[Releases](https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/releases)** troverai pacchetti già pronti per ogni scheda supportata.

### Metodo 1: Procedura Semplificata con Script (Consigliato)
Questo è il metodo più veloce e non richiede l'installazione di software aggiuntivi. I pacchetti sono disponibili per **Windows, Linux e MacOS**.

1. Vai alla pagina delle **[Releases](https://github.com/alessandro-satanassi/OpenFIRE-Firmware-ESP32/releases)** e scarica lo ZIP "Procedura Semplificata" relativo alla tua board (es. `DevKitC-1` o `PICO`).
2. Estrai l'intero contenuto dell'archivio ZIP in una cartella sul tuo PC.
3. Collega la scheda ESP32-S3 al computer tramite cavo USB *(assicurati che sia un cavo dati, non solo per la ricarica)*.
4. Esegui lo script `flash_firmware` (su Windows sarà il file `.bat`).
5. Lo script cercherà automaticamente la porta seriale e ti guiderà nell'installazione.

> **ℹ️ Aggiornamento vs Installazione Pulita (NoFS vs Full)**
> Durante la procedura guidata per la Lightgun, ti verrà chiesto quale versione installare:
> * **Versione Base (NoFS):** Ideale per gli aggiornamenti. Aggiorna solo il codice applicativo, **mantenendo intatte** le tue calibrazioni e mappature dei pulsanti.
> * **Versione Completa (Full):** Da usare per la prima installazione. Formatta il microcontrollore e installa il filesystem di fabbrica, **cancellando** ogni impostazione precedente.

### Metodo 2: Installazione Manuale
Per gli utenti avanzati che preferiscono utilizzare tool grafici come **NodeMCU PyFlasher** o l'utility a riga di comando **esptool**, nella pagina delle Release sono forniti anche i singoli file `.bin` "merged" pronti per essere flashati direttamente all'indirizzo di base `0x0`.

---

### ⚠️ Risoluzione dei Problemi (Troubleshooting)

* **Il Flashing non parte (Connecting...):** Alcune schede ESP32-S3 possono essere "capricciose" nell'entrare automaticamente in modalità download. Se durante lo script vedi la scritta `Connecting...` che si ripete senza avanzare, tieni premuto il piccolo pulsante fisico **BOOT** (o `B`) sulla tua scheda ESP32 finché l'installazione non inizia.
* **Falso Positivo Antivirus (Windows):** Lo script utilizza `esptool.exe` originale di Espressif. Alcuni antivirus potrebbero bloccarlo o segnalarlo come falso positivo. Il file è sicuro al 100%, potresti doverlo aggiungere alle eccezioni temporanee.
* **Configurazione Post-Installazione:** Ricorda che questi script servono *solo* a installare il firmware. Per configurare la lightgun (mappare i pin, i pulsanti, effettuare la calibrazione IR) dovrai utilizzare la **OpenFIRE Desktop App** originale. Controlla le note di release per scaricare la versione dell'App compatibile con il firmware.

---
---
---

## Requisiti
INDISPENSABILI:
* Un microcontrollore **ESP32-S3** per eseguire il firmware. Le schede consigliate sono: ESP32-S3-WROOM1-DevKitC-1, Waveshare ESP32-S3-PICO;

* Telecamera di posizionamento IR DFRobot SEN0158 (alternativamente si può utilizzare una Wii CAM ed utilizzare un pin del micro per generale il Wii CAM clock necessario per il funzionamento della CAM);

* 4 Emettitori LED IR: le normali barre sensore della Wii potrebbero funzionare per brevi distanze, ma è ALTAMENTE consigliato l'utilizzo di LED OSRAM SFH 4547 con resistenze da 5.6Ω (ohm);

* 1 interruttore con funzioni di grilletto;

Anche se non sono obbligatori, sono altamente consigliati almeno altri 2 interruttori (A e B), che permettono un utilizzo completo di quasi tutti i giochi esistenti retrogame per lightgun;

OPZIONALI che possono essere gestiti dalla lightgun:

Innterruttori (pulsanti): oltre al grilletto, il firmware può gestire altri 13 interruttori/pulsanti completamente configurabili tramite la OpenFIRE Desktop App (vedi immagini seguenti dei micro ove si mostra la mappatura dei pulsanti);
  Opzionale: oltre ai pulsanti A e B (altamente consigliati) anche un ulteriore pulsante C;
  Opzionale: Modulo di navigazione direzionale 5D D-Pad + ulteriori 2 pulsanti (start e select);
  Opzionale: switch per pompa;

Opzionale: Qualsiasi solenoide a 12-24V, con relativa scheda driver *(Richiede un alimentatore separato regolabile da 12-24V, oppure, solo per solenoidi 12V una batteria li-ion 3,7V (18650 o 21700) con corrente di scarica di almeno 10A e scheda booster potente tipo una MP3429 (3,7V -> 12V))*;

Opzionale: Un sensore ti temperatura TMP36 da posizionare sul solenoide per evitare surriscaldamenti dopo lunghe sessioni;

Opzionale: Qualsiasi motore rumble per gamepad a 5V, con relativa scheda driver;

Opzionale: Joystick analogico a 2 assi;

Opzionale: Qualsiasi interruttore SPDT a 2 vie, per regolare via hardware lo stato di rumble/solenoide/fuoco rapido (può essere regolato via software se non disponibile!).

Opzionale: Qualsiasi NeoPixel WS2812B GRB, o qualsiasi LED RGB a quattro pin per illuminazione e reazioni in tempo reale; 

Opzionale: Display OLED 128x64 basato su I2C (2 fili/4 pin) SSD1306 per un'interfaccia visiva e per il supporto al feedback dei contatori di vita/munizioni;

Poi trovare link per acquisto e istruzioni sul mio progetto [PICON-AS](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/) nella sezione hardware e istruzioni, ma sentiti libero di utilizzare qualsiasi tipo di compoente che vorrai provare, per la tua lightgun personalizzata.

Allego inoltre una guida di auto-costruuzione/connessione di alcuni componenti, reperibile sul sito del progetto originale OpenFIRE (tale guida grafica mostra come micro un Raspberry P Pico RP2040, ma è applicabile allo stesso modo a qualsiasi pin dell ESP32-S3, configurato) Guida componenti __ guida wii cam.



Fai riferimento alle seguenti board per i pinout predefiniti delle varie schede; tieni presente, tuttavia, che ogni scheda supporta layout di pin completamente personalizzati, configurabili tramite la OpenFIRE Desktop App. Questi layout possono anche essere visualizzati dalla nuova Desktop App.

foto delle board due board, fare tabella con due celle ove all'interno si mette foto delle board

## Installazione:

Scarica l'ultimo binario .UF2 per la tua rispettiva scheda dalla pagina delle release e trascina il file sul tuo microcontrollore mentre è avviato in modalità Bootloader; l'RP2040 viene montato automaticamente in questo modo quando non c'è alcun programma caricato, ma può essere forzato in questa modalità tenendo premuto il tasto BOOTSEL mentre lo si collega al computer: apparirà come un dispositivo di archiviazione rimovibile chiamato RPI-RP2.

## Informazioni aggiuntive

Dai un'occhiata al manuale di istruzioni allegato! Per gli sviluppatori, consultate la documentazione sulla configurazione e la compilazione del firmware OpenFIRE.

## Problemi noti:



## Da fare (TODO):

Inventare battute migliori.

## Nota
I solenoidi possono causare disconnessioni dovute a interferenze elettromagnetiche (EMI) se il cablaggio è troppo sottile. I cavi per questo cablaggio dovrebbero essere di almeno 24AWG, altrimenti si trasformeranno in antenne in caso di uso prolungato, facendo scattare le soglie di sicurezza USB del PC per proteggere le porte.



===================================================================================================
===================================================================================================
===================================================================================================

# OpenFIRE-Firmware-ESP32

> 🛠️ **Hardware sponsored by [PCBWay](https://www.pcbway.com)**

---

## PICON-AS: Wireless Lightgun Hardware
Se vuoi costruire una lightgun wireless funzionante completamente a batteria basata su questo firmware, dai un'occhiata al progetto **PICON-AS**. Trovi tutte le istruzioni per l'hardware e il montaggio a questo indirizzo: [Istruzioni PICON-AS](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/)
> [!NOTE]
> Nota: Il sito è già fruibile e i contenuti tecnici sono corretti; stiamo solo ultimando la stesura dettagliata di alcune sezioni delle istruzioni. Al momento il sito è solo in lingua italiana, sarà tradotto in inglese quando sarà completato.

---
If you want to build a fully battery-powered wireless lightgun based on this firmware, check out the **PICON-AS** project. You can find all the hardware and assembly instructions at this link: [PICON-AS Instructions](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/)
> [!NOTE]
> Note: The website is already functional and all technical data is correct; we are currently finalizing the detailed step-by-step instructions. "Currently, the website is only available in Italian; it will be translated into English once it is completed
---

## ... porting OpenFIRE-firmware for EPS32

Questo repository è un porting del progetto originale 'OpenFIRE-firmware' del TeamOpenFIRE, adattato per funzionare sul microcontrollore ESP32S3.
Il progetto è stato sviluppato utilizzando PlatformIO e, salvo alcuni adattamenti e piccole modifiche, il codice rimane sostanzialmente fedele all'originale del TeamOpenFIRE.
Ho inoltre implementato la connessione wireless utilizzando il protocollo ESP-NOW per ESP32. Per farlo, ho sviluppato un dongle ESP32S3 da collegare al PC, il cui codice è disponibile in questo stesso repository.
Grazie a questa soluzione, il PC non rileva alcuna differenza tra la connessione diretta via USB e quella wireless tramite dongle.
Il codice di questo repository funziona anche sul microcontrollore RP2040, ma in questo caso supporta solo il collegamento diretto tramite USB, senza connessione wireless.
Ogni volta che verranno apportate modifiche al progetto originale 'OpenFIRE-firmware' del TeamOpenFIRE, aggiornerò di conseguenza anche il codice di questo repository.
Ringrazio di cuore il TeamOpenFIRE per la creazione del progetto 'OpenFIRE-firmware': a loro vanno tutti i meriti e la mia piena gratitudine.
Questo è semplicemente un adattamento per il funzionamento su ESP32S3, con l'aggiunta della connessione wireless tramite ESP-NOW.

## Sequenza di avvio del firmware

1. **Calibrazione automatica del joystick analogico**
   > **IMPORTANTE:** La procedura dura circa due secondi. Durante questa fase **non toccare il joystick** per garantire una calibrazione corretta.

2. **Gestione della connettività**
   * **Connessione via Cavo:** Se il cavo USB è collegato al PC, la lightgun si connette direttamente in modalità cablata.
   * **Connessione Wireless (USB scollegato):**
     * La lightgun scansiona l'ambiente e seleziona il canale Wi-Fi con minori interferenze per una trasmissione ottimale.
     * Rimane in ascolto di un **dongle**. Se durante l'attesa viene collegato un cavo USB, la connessione passa immediatamente su cavo.
     * Una volta rilevato un dongle libero, esegue l'associazione. Il PC gestirà la periferica esattamente come se fosse collegata via cavo (nessuna differenza di funzionamento).
     * **Riconnessione automatica:** In caso di spegnimento, al riavvio cercherà prioritariamente l'ultimo dongle associato per una connessione istantanea. Se non lo trova, farà una nuova scansione dei canali.

3. **Stato della connessione**
   Una volta stabilito il collegamento con il PC, l'interfaccia mostrerà lo stato attuale:
   * **Icona Wi-Fi**: Connessione tramite dongle wireless.
   * **Icona USB**: Connessione tramite cavo fisico.

---
---

This repository is a porting of the original 'OpenFIRE-firmware' project by TeamOpenFIRE, adapted to work on the ESP32S3 microcontroller.
The project was developed using PlatformIO and, apart from some adaptations and small adjustments, the code remains essentially faithful to the original by TeamOpenFIRE.
I have also implemented wireless connectivity using the ESP-NOW protocol for ESP32. To achieve this, I developed an ESP32S3 dongle to connect to the PC, and its code is available in this same repository.
Thanks to this solution, the PC does not detect any difference between a direct USB connection and a wireless connection via dongle.
The code in this repository also works on the RP2040 microcontroller, but in this case, it only supports direct USB connection without wireless connectivity.
Whenever there are changes to the original 'OpenFIRE-firmware' project by TeamOpenFIRE, I will adapt the code in this repository accordingly.
I sincerely thank TeamOpenFIRE for creating the 'OpenFIRE-firmware' project; all credit and gratitude go to them for their work.
This is simply an adaptation to make it work on ESP32S3, with the addition of wireless connectivity via ESP-NOW.

## Firmware boot sequence

1. **Automatic analog joystick calibration**
   > **IMPORTANT:** This process takes approximately two seconds. During this phase, **do not touch the joystick** to ensure correct calibration.

2. **Connectivity management**
   * **Wired Connection:** If the USB cable is connected to the PC, the lightgun will connect directly in wired mode.
   * **Wireless Connection (USB disconnected):**
     * The lightgun scans the environment and selects the Wi-Fi channel with the least interference for optimal transmission.
     * It remains in listening mode for a **dongle**. If a USB cable is connected during this wait, the connection immediately switches to wired mode.
     * Once a free dongle is detected, it performs the pairing. The PC will handle the peripheral exactly as if it were connected via cable (no difference in functionality).
     * **Automatic reconnection:** In case of power-off, upon restart, it will prioritize searching for the last paired dongle for an instantaneous connection. If not found, it will perform a new channel scan.

3. **Connection status**
   Once the connection with the PC is established, the interface will show the current status:
   * **Wi-Fi Icon**: Connection via wireless dongle.
   * **USB Icon**: Connection via physical cable.

---

## 🤝 Sponsorship & Support

A special thanks to **[PCBWay](https://www.pcbway.com)** for sponsoring the hardware development of this project. Their professional PCB manufacturing service has been fundamental in transforming our schematics into reliable, high-quality physical boards.

We chose PCBWay for their:
* **Manufacturing Precision:** Excellent solder mask and silkscreen quality even on dense designs.
* **Reliability:** Consistent build quality across different batches.
* **Fast Prototyping:** Quick turnaround times that significantly accelerated our testing phase.

<p align="left">
  <a href="https://www.pcbway.com">
    <img src="docs/img/pcbway-logo.png" alt="PCBWay - PCB Prototype & Fabrication" width="200">
  </a>
</p>

---

![Waveshare-esp32-s3-pico](https://github.com/user-attachments/assets/5f7bf9ae-6ab5-4240-b930-d8cf20cb1c75)

![YD-esp32-s3-wroom1-DevKitC-1-N16R8](https://github.com/user-attachments/assets/6e865f2a-d90f-4dd0-9b57-c992bfd7377f)


## ... segue il link della la pagina originale del progetto
## ... follow the link to the original project page

### OpenFIRE - The Open *Four Infa-Red Emitter* Light Gun System [https://github.com/TeamOpenFIRE/OpenFIRE-Firmware](https://github.com/TeamOpenFIRE/OpenFIRE-Firmware)

![BannerDark](docs/of_bannerLoD.png#gh-dark-mode-only)![BannerLight](docs/of_bannerDoL.png#gh-light-mode-only)

