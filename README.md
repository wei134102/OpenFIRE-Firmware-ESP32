# OpenFIRE Firmware for ESP32

<p align="center">
  <img src="docs/img/immagine_di_copertina.png" alt="OpenFIRE Firmware ESP32 Cover" width="100%">
</p>

---
> 🛠️ **Hardware sponsored by [PCBWay](https://www.pcbway.com)**
---

This repository contains the port of the original [OpenFIRE-firmware](https://github.com/TeamOpenFIRE/OpenFIRE-Firmware) project, developed by TeamOpenFIRE, adapted and optimized for the ESP32-S3 microcontroller. The main goal of this work is to extend the exceptional tracking capabilities of the OpenFIRE system by introducing **high-performance native wireless connectivity**.

## Main Features and Capabilities
The firmware transforms the microcontroller into a highly advanced lightgun controller, offering the following core features (inherited from the original project):

* **Advanced IR Tracking:** Utilizes a four-point infrared system with real-time perspective correction. Supports multiple emitter configurations, including double lightbar (recommended) or diamond layouts, ensuring absolute precision regardless of the player's angle to the screen.
* **Complete Peripheral Support:** Native management of tactile and force feedback (Solenoid and Rumble motor), temperature monitoring via TMP36 sensor, and dynamic lighting via WS2812B NeoPixel LEDs.
* **Flexible Inputs and Mapping:** The system provides simultaneous outputs such as Keyboard, 5-button Absolute Positioning Mouse (ABS), and dual-stick Gamepad (with D-pad support). It offers a robust button mapping system configurable for any need.
* **Dedicated App and Internal Memory:** Full integration with the **[OpenFIRE App](https://github.com/TeamOpenFIRE/OpenFIRE-App)** for cross-platform, on-the-fly configuration. Calibration profiles and user settings are saved directly to the lightgun's internal memory, making it portable across different PCs without needing to run the setup again.
* **OLED Visual Feedback:** Support for I2C SSD1306 displays, used for menu navigation and providing visual indicators for in-game elements (e.g., life count, ammo).
* **Advanced Compatibility:** Fully compatible with PC Force Feedback handlers (such as Mame Hooker, The Hook Of The Reaper, and QMamehook) and the MiSTer FPGA ecosystem.
* **Dual-Core Optimization:** Leverages the microcontroller's dual-core capabilities to simultaneously manage input polling, camera processing, and peripheral management without any slowdowns.

## Project Philosophy and Porting
The OpenFIRE-Firmware-ESP32 firmware stems from the need to bring the features of a professional open-source lightgun to an architecture that allows maximum freedom of movement. The code was developed using the PlatformIO ecosystem and, while introducing structural innovations for wireless management, maintains extremely high logical and functional fidelity to the original TeamOpenFIRE code.

The distinctive features of this port include:

* **Transparent Wireless Integration**: The implementation of the ESP-NOW protocol allows direct communication between the peripheral and a dedicated dongle connected to the PC. This solution is designed to be totally transparent: the operating system detects the lightgun as a standard USB peripheral, with no perceivable latency and no need for third-party drivers or software.

* **Enhanced Tracking Algorithms:** This port introduces deep refinements to the spatial calculation algorithms. The system now offers exceptional cursor stability even at very close distances and correctly calculates tracking even during wide rotations of the lightgun on its axis (tilt). At the end of the testing phase, if these implementations prove solid and superior, it is my intention to propose a Pull Request (PR) to the original project so that the entire OpenFIRE community can benefit from them.

* **Fidelity to the Original Code**: Excluding the necessary adaptations for the ESP32 architecture and radio transmission management, the core control logic remains consistent with the official version. This ensures that improvements and fixes made by TeamOpenFIRE can be cyclically integrated into this repository.

* **Hardware Versatility**: Although the project is focused on the wireless capabilities of the ESP32-S3, the code maintains compatibility with the RP2040 microcontroller. In the latter case, operation is limited to a wired USB connection, while still maintaining firmware uniformity within the OpenFIRE ecosystem.

Special thanks to TeamOpenFIRE for their excellent work in creating the original firmware; they deserve credit for the core architecture and our gratitude for making such an advanced system available to the community.

## Supported Microcontrollers
The firmware has been tested and optimized for the **ESP32-S3** architecture, which is the reference microcontroller for this project. The system relies entirely on the dual-core power and connectivity of the **ESP32-S3**, which supports all features, including wireless communication via ESP-NOW and HID management via USB OTG. *(e.g., ESP32-S3-WROOM1-DevKitC-1, Waveshare ESP32-S3-PICO, Waveshare ESP32-S3-ZERO, LILYGO T-Dongle-S3, ESP32-S3 Pocket Dongle S3)*.

For best results and maximum ease of assembly, the following form factors are recommended:

| Device | Recommended Boards | Usage |
| :--- | :---: | :--- |
| **Lightgun** | <img src="docs/board_scheme/ESP32S3-Devkit-C.svg" width="44%"> <img src="docs/board_scheme/esp32-s3-pico.svg" width="40%"> | **ESP32-S3-DevKitC-1 / Waveshare S3-PICO**<br>Ideal for integration within the gun shell due to the high number of GPIO pins, necessary to easily manage all buttons, the sensor, and actuators. |
| **Dongle** | <img src="docs/board_scheme/LILYGO-T-Dongle-S3-ESP32-S3.svg" width="45%"> <img src="docs/board_scheme/esp32-s3-pocket-dongle-s3.svg" width="45%"> | **LILYGO T-Dongle-S3 / Pocket Dongle S3**<br>"Turnkey" solutions with integrated USB connector. Perfect as compact receivers to connect directly to the PC or console, without the need for cables or soldering. |
| **Pedal** | <img src="docs/board_scheme/esp32-s3-zero.svg" width="45%"> | **Waveshare ESP32-S3-ZERO**<br>Thanks to its ultra-compact dimensions, it is the perfect choice to be placed inside a wireless pedal structure, where space is limited and very few pins are needed. |

**Hardware Versatility and Interchangeability**
Except for the "USB Sticks" (like the LILYGO T-Dongle or Pocket Dongle), which by their physical nature are designed exclusively for PC use as receivers, **all other standard boards are universal**. 
A board like the DevKitC-1, PICO, or ZERO can be programmed and used interchangeably for the Lightgun, the Pedal, or even to build a "DIY" Dongle by wiring a USB plug. Specific hardware configuration details are illustrated in their respective folders.

> *Compatibility Note: Although the code maintains basic compatibility with the original RP2040 architecture, its use remains strictly limited to wired USB connections. Active development, optimization, and all advanced wireless features are focused exclusively on the ESP32-S3 platform.*

## System Architecture
The project is divided into three modular components, each with specific technical documentation within their respective folders:

1. ***Lightgun Firmware***
   The main firmware that manages the IR sensor, button logic, and feedback peripherals (solenoid, rumble, LEDs). It can operate in both wireless and wired modes.
   Technical documentation: **[Lightgun Folder](./lightgun)**

2. ***Receiver Dongle***
   The Dongle acts as an invisible bridge between the lightgun and the PC. It handles the reception of ESP-NOW packets and translates the data into standard HID protocols.
   Technical documentation: **[Dongle Folder](./dongle)**

3. ***Wireless Pedal***
   An optional but essential accessory for certain cover shooter titles (e.g., Time Crisis). It communicates directly with the lightgun to send ultra-low latency input signals, eliminating the need for bulky wiring on the floor.
   Technical documentation: **[Pedal Folder](./pedal)**

## Connectivity Management
The firmware intelligently manages connection priorities:

* **Wired Connection**: If the lightgun is connected to the PC via a USB cable, the system disables wireless scanning and operates as a direct HID peripheral.

* **Wireless Connection**: In the absence of a USB connection, the lightgun activates ESP-NOW mode. The pairing process is completely automatic and requires no user intervention: the dongle connected to the PC handles scanning the environment to select the radio channel with the least interference. The lightgun then searches for an available dongle and pairs with it. Immediately after, if a wired pedal is not already configured, the lightgun starts a 10-second search to locate and pair an available wireless pedal (the use of which is entirely optional; the system works perfectly without it). Once the connection is established, the PC will manage the peripheral exactly as if it were connected via cable, with no operational difference.
In the event of a power-off and subsequent restart of the lightgun, the system will prioritize searching for the last paired dongle and pedal to ensure instant reconnection; if it does not detect them, it will automatically start a new search. *(Note: this instant reconnection is only possible if the dongle and pedal have remained continuously powered on since the first pairing; if they are rebooted, they will reset and listen for new connections, requiring a new scan from the lightgun)*.

* **Visual Feedback**: If the system is equipped with a display, the interface will show dedicated icons to distinguish the connection state (USB or Wireless) and monitor the link status (on the lightgun and dongle displays, or via 4 dedicated LEDs on the pedal).

## PICON-AS Hardware Project
For those who wish to build a fully battery-operated wireless lightgun based on this firmware, the **PICON-AS** reference hardware project is available. This is a lightgun derived from the PICON-OG project, optimized to integrate a rechargeable 21700 Li-ion battery and support the cable-free wireless ecosystem.
This project provides detailed assembly instructions, complete with STL files for 3D printing and wiring diagrams.

Hardware manual: **[PICON-AS Documentation Site](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/)**

> [!NOTE]
> The site is already usable and the technical content is correct; we are just finalizing the drafting of some instruction sections. Currently, the site is only available in Italian, but it will be translated into English once completed.

## 🤝 Sponsorship & Support

A special thanks to **[PCBWay](https://www.pcbway.com)** for sponsoring the hardware development of this project. Their professional PCB manufacturing service has been fundamental in transforming our schematics into reliable, high-quality physical boards.

We chose PCBWay for their:
* **Manufacturing Precision:** Excellent solder mask and silkscreen quality even on dense designs.
* **Reliability:** Consistent build quality across different batches.
* **Fast Prototyping:** Quick turnaround times that significantly accelerated our testing phase.

<p align="left">
  <a href="https://www.pcbway.com">
    <img src="lightgun/docs/img/pcbway-logo.png" alt="PCBWay - PCB Prototype & Fabrication" width="200">
  </a>
</p>

---
---

# OpenFIRE Firmware per ESP32

<p align="center">
  <img src="docs/img/immagine_di_copertina.png" alt="OpenFIRE Firmware ESP32 Cover" width="100%">
</p>

---
> 🛠️ **Hardware sponsored by [PCBWay](https://www.pcbway.com)**
---

Questo repository contiene il porting del progetto originale [OpenFIRE-firmware](https://github.com/TeamOpenFIRE/OpenFIRE-Firmware), sviluppato dal TeamOpenFIRE, adattato e ottimizzato per il microcontrollore ESP32-S3. L'obiettivo principale di questo lavoro è estendere le eccezionali capacità di tracciamento del sistema OpenFIRE introducendo la **connettività wireless nativa ad alte prestazioni**.

## Caratteristiche e Funzionalità Principali
Il firmware trasforma il microcontrollore in un controller per lightgun estremamente avanzato, offrendo le seguenti funzionalità di base (ereditate dal progetto originale):

* **Tracciamento IR Avanzato:** Utilizza un sistema a quattro punti a infrarossi con correzione prospettica in tempo reale. Supporta configurazioni multiple degli emettitori, inclusi layout a doppia barra luminosa (consigliato) o a diamante, garantendo una precisione assoluta indipendentemente dall'angolazione del giocatore rispetto allo schermo.
* **Supporto Periferiche Completo:** Gestione nativa del feedback tattile e di forza (Solenoide e motore Rumble), monitoraggio della temperatura tramite sensore TMP36 e illuminazione dinamica tramite LED NeoPixel WS2812B.
* **Input Flessibili e Mappatura:** Il sistema fornisce output simultanei come Tastiera, Mouse a posizionamento assoluto (ABS) a 5 pulsanti e Gamepad dual-stick (con supporto D-pad). Offre un robusto sistema di mappatura dei pulsanti configurabile per ogni esigenza.
* **App Dedicata e Memoria Interna:** Piena integrazione con la **[OpenFIRE App](https://github.com/TeamOpenFIRE/OpenFIRE-App)** per una configurazione multipiattaforma e "al volo". I profili di calibrazione e le impostazioni dell'utente vengono salvati direttamente nella memoria interna della lightgun, rendendola portabile tra diversi PC senza dover rifare il setup.
* **Feedback Visivo OLED:** Supporto per display I2C SSD1306, utilizzati per la navigazione dei menu e per fornire indicazioni visive degli elementi in-game (es. conteggio vite, munizioni).
* **Compatibilità Avanzata:** Pienamente compatibile con i gestori di Force Feedback per PC (come Mame Hooker, The Hook Of The Reaper e QMamehook) e con l'ecosistema MiSTer FPGA.
* **Ottimizzazione Dual-Core:** Sfrutta le capacità dual-core del microcontrollore per gestire simultaneamente e senza rallentamenti il polling degli input, l'elaborazione della videocamera e la gestione delle periferiche.

## Filosofia del Progetto e Porting
Il firmware OpenFIRE-Firmware-ESP32 nasce dalla necessità di portare le funzionalità di una lightgun open-source professionale su un'architettura che consenta la massima libertà di movimento. Il codice è stato sviluppato utilizzando l'ecosistema PlatformIO e, pur introducendo innovazioni strutturali per la gestione del wireless, mantiene un'altissima fedeltà logica e funzionale rispetto al codice originale del TeamOpenFIRE.

Le caratteristiche distintive di questo porting includono:

* **Integrazione Wireless Trasparente**: L'implementazione del protocollo ESP-NOW permette una comunicazione diretta tra la periferica e un dongle dedicato collegato al PC. Questa soluzione è progettata per essere totalmente trasparente: il sistema operativo rileva la lightgun come una periferica USB standard, senza latenze percepibili e senza la necessità di driver o software di terze parti.

* **Algoritmi di Tracciamento Potenziati:** Questo porting introduce profondi affinamenti agli algoritmi di calcolo spaziale. Il sistema offre ora una stabilità del cursore eccezionale anche a distanze ravvicinate e calcola correttamente il tracciamento anche durante ampie rotazioni della lightgun sull'asse (tilt). Al termine della fase di testing, se queste implementazioni si confermeranno solide e superiori, è mia intenzione proporre una Pull Request (PR) al progetto originale affinché tutta la community di OpenFIRE possa beneficiarne.

* **Fedeltà al Codice Originale**: Ad esclusione degli adattamenti necessari per l'architettura ESP32 e per la gestione della trasmissione radio, il cuore della logica di controllo rimane coerente con la versione ufficiale. Questo garantisce che i miglioramenti e le correzioni apportate dal TeamOpenFIRE possano essere integrati ciclicamente in questo repository.

* **Versatilità Hardware**: Sebbene il progetto sia focalizzato sulle capacità wireless dell'ESP32-S3, il codice mantiene la compatibilità con il microcontrollore RP2040. In quest'ultimo caso, il funzionamento rimane limitato alla connessione cablata USB, mantenendo però l'uniformità del firmware all'interno dell'ecosistema OpenFIRE.

Si ringrazia il TeamOpenFIRE per l'eccellente lavoro svolto nella creazione del firmware originale; a loro vanno i meriti dell'architettura di base e la gratitudine per aver reso disponibile un sistema così avanzato alla comunità.

## Microcontrollori Supportati
Il firmware è stato testato e ottimizzato per l'architettura **ESP32-S3**, che è il microcontrollore di riferimento per questo progetto. Il sistema si basa interamente sulla potenza e sulla connettività dell'**ESP32-S3**, che supporta tutte le funzionalità, inclusa la comunicazione wireless via ESP-NOW e la gestione HID via USB OTG. *(ESP32-S3-WROOM1-DevKitC-1, Waveshare ESP32-S3-PICO, Waveshare ESP32-S3-ZERO, LILYGO T-Dongle-S3, ESP32-S3 Pocket Dongle S3)*.

Per ottenere i migliori risultati e la massima facilità di montaggio, si consigliano i seguenti form factor:

| Dispositivo | Board Consigliate | Utilizzo |
| :--- | :---: | :--- |
| **Lightgun** | <img src="docs/board_scheme/ESP32S3-Devkit-C.svg" width="44%"> <img src="docs/board_scheme/esp32-s3-pico.svg" width="40%"> | **ESP32-S3-DevKitC-1 / Waveshare S3-PICO**<br>Ideali per essere integrate all'interno della scocca della pistola grazie all'elevato numero di pin GPIO, necessari per gestire agevolmente tutti i pulsanti, il sensore e gli attuatori. |
| **Dongle** | <img src="docs/board_scheme/LILYGO-T-Dongle-S3-ESP32-S3.svg" width="45%"> <img src="docs/board_scheme/esp32-s3-pocket-dongle-s3.svg" width="45%"> | **LILYGO T-Dongle-S3 / Pocket Dongle S3**<br>Soluzioni "chiave in mano" con connettore USB integrato. Perfette come ricevitori compatti da collegare direttamente al PC o alla console, senza necessità di cavi o saldature. |
| **Pedale** | <img src="docs/board_scheme/esp32-s3-zero.svg" width="45%"> | **Waveshare ESP32-S3-ZERO**<br>Grazie alle sue dimensioni ultracompatte, è la scelta perfetta per essere inserita all'interno della struttura di un pedale wireless, dove lo spazio è ridotto e servono pochissimi pin. |

**Versatilità e Intercambiabilità dell'Hardware**
Fatta eccezione per le "USB Stick" (come il LILYGO T-Dongle o il Pocket Dongle) che per la loro natura fisica sono pensate esclusivamente per l'uso su PC come ricevitori, **tutte le altre schede standard sono universali**. 
Una board come la DevKitC-1, la PICO o la ZERO può essere programmata e utilizzata indistintamente per la Lightgun, per il Pedale o persino per costruire un Dongle "fai-da-te" cablando una presa USB. I dettagli specifici sulle configurazioni hardware sono illustrati nelle rispettive cartelle.

> *Nota di compatibilità: Sebbene il codice mantenga la compatibilità di base con l'architettura originale RP2040, il suo utilizzo rimane strettamente limitato alla connessione cablata USB. Lo sviluppo attivo, l'ottimizzazione e tutte le funzionalità wireless avanzate sono focalizzate esclusivamente sulla piattaforma ESP32-S3.*

## Architettura del Sistema
Il progetto si articola in tre componenti modulari, ognuno dei quali dispone di documentazione tecnica specifica all'interno delle relative cartelle:

1. ***Lightgun Firmware***
   Il firmware principale che gestisce il sensore IR, la logica dei pulsanti e le periferiche di feedback (solenoide, rumble, LED). Può operare sia in modalità wireless che via cavo.
   Documentazione tecnica: **[Cartella Lightgun](./lightgun)**

2. ***Dongle Ricevitore***
   Il Dongle agisce come ponte invisibile tra la lightgun e il PC. Gestisce la ricezione dei pacchetti ESP-NOW e traduce i dati in protocolli HID standard.
   Documentazione tecnica: **[Cartella Dongle](./dongle)**

3. ***Pedale Wireless***
   Un accessorio opzionale ma fondamentale per alcuni titoli cover shooter (es. Time Crisis). Comunica direttamente con la lightgun per inviare segnali di input a bassissima latenza, eliminando la necessità di cablaggi ingombranti sul pavimento.
   Documentazione tecnica: **[Cartella Pedal](./pedal)**

## Gestione della Connettività
Il firmware gestisce in modo intelligente la priorità delle connessioni:

* **Connessione Cablata**: Se la lightgun viene collegata al PC tramite cavo USB, il sistema disabilita la scansione wireless e opera come una periferica HID diretta.

* **Connessione Wireless**: In assenza di collegamento USB, la lightgun attiva la modalità ESP-NOW. Il processo di accoppiamento è completamente automatico e non richiede alcun intervento da parte dell'utente: è il dongle collegato al PC a occuparsi della scansione dell'ambiente per selezionare il canale radio con minori interferenze. La lightgun cerca quindi un dongle libero e vi si associa. Subito dopo, se non è già configurato un pedale cablato, la lightgun avvia una ricerca di 10 secondi per individuare e accoppiare un pedale wireless libero (il cui utilizzo rimane comunque facoltativo, il sistema funziona perfettamente anche senza). Una volta stabilita la connessione, il PC gestirà la periferica esattamente come se fosse collegata via cavo, senza alcuna differenza di funzionamento.
In caso di spegnimento e successivo riavvio della lightgun, il sistema cercherà in via prioritaria l'ultimo dongle e l'ultimo pedale associati per garantire una riconnessione istantanea; se non li rileva, avvierà automaticamente una nuova ricerca. *(Nota bene: questa riconnessione istantanea è possibile solo se il dongle e il pedale sono rimasti ininterrottamente accesi dopo il primo accoppiamento; se vengono riavviati, si resetteranno mettendosi in ascolto di nuove connessioni, richiedendo una nuova scansione da parte della lightgun)*.

* **Feedback Visivo**: Se il sistema è dotato di display, l'interfaccia mostrerà icone dedicate per distinguere lo stato della connessione (USB o Wireless) e monitorare lo stato del collegamento (sui display di lightgun e dongle, o tramite 4 LED dedicati sul pedale).

## Progetto Hardware PICON-AS
Per chi desidera realizzare una lightgun wireless funzionante completamente a batteria basata su questo firmware, è disponibile il progetto hardware di riferimento **PICON-AS**. Si tratta di una lightgun derivata dal progetto PICON-OG, ottimizzata per integrare una batteria Li-ion 21700 ricaricabile e supportare l'ecosistema wireless senza cavi.
Tale progetto fornisce istruzioni dettagliate per l'assemblaggio, completo di file STL per la stampa 3D e schemi elettrici.

Manuale hardware: **[Sito Documentazione PICON-AS](https://alessandro-satanassi.github.io/OpenFIRE-PICON-AS-ESP32/)**

> [!NOTE]
> Il sito è già fruibile e i contenuti tecnici sono corretti; stiamo solo ultimando la stesura di alcune sezioni delle istruzioni. Al momento il sito è solo in lingua italiana, sarà tradotto in inglese quando sarà completato.

## 🤝 Sponsorizzazione e Supporto

Un ringraziamento speciale a **[PCBWay](https://www.pcbway.com)** per aver sponsorizzato lo sviluppo hardware di questo progetto. Il loro servizio professionale di produzione di PCB è stato fondamentale per trasformare i nostri schemi elettrici in schede fisiche affidabili e di alta qualità.

Abbiamo scelto PCBWay per:
* **Precisione di Produzione:** Eccellente qualità del solder mask e della serigrafia anche su design ad alta densità.
* **Affidabilità:** Qualità costruttiva uniforme e costante su diversi lotti di produzione.
* **Prototipazione Rapida:** Tempi di realizzazione brevi che hanno accelerato significativamente la nostra fase di test.

<p align="left">
  <a href="https://www.pcbway.com">
    <img src="lightgun/docs/img/pcbway-logo.png" alt="PCBWay - Prototipazione e Produzione PCB" width="200">
  </a>
</p>

---
![BannerDark](lightgun/docs/of_bannerLoD.png#gh-dark-mode-only)![BannerLight](lightgun/docs/of_bannerDoL.png#gh-light-mode-only)

