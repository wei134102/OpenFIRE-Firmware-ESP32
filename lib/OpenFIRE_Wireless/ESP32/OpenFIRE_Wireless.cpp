#if defined(OPENFIRE_WIRELESS_ENABLE) && defined(ARDUINO_ARCH_ESP32)

#include "OpenFIRE_Wireless.h"

#define ESPNOW_WIFI_CHANNEL 4

// la potenza di trasmissione può andare da 8 a 84, dove 84 è il valore massimo che corrisponde a 20 db
#define ESPNOW_WIFI_POWER 69 
  
SerialWireless_ SerialWireless;
extern Adafruit_USBD_HID usbHid;
SemaphoreHandle_t tx_sem = NULL;
uint8_t buffer_espnow[ESP_NOW_MAX_DATA_LEN];
esp_now_peer_info_t peerInfo; // deve stare fuori funzioni da funzioni -- globale --variabile di utilità per configurazione

static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len); // callback esp_now
static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status); // callback esp_now

void packet_callback_read_dongle(); // callback packet 
void packet_callback_read_gun(); // callback packet

///////////////////////////////////////////////////////////////////
uint8_t aux_buffer[13];
//uint8_t mac_esp_another_card[6];
//uint8_t stato_conneddione_wireless = 0;
uint8_t tipo_connessione = 0;

// stato della connessione 

// in caso di DONGLE
// 0 = non stabilita
// 1 = inviata richiesta blroad cast
// 2 = qualcuno ha risposto alla connessione ed ha chiesto di essere conesso
// 3 = inviato conferma connessione;

// in caso di GUN
// 0 = non stabilita
// 1 = arrivata richiesta broadcast di connessione
// 2 = accettata ed inviata richiesta di connessione
// 3 = arrivara accettazione di connessione

const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#if defined(DONGLE)
  ///////////////////////uint8_t peerAddress[6] = {0x24, 0x58, 0x7C, 0xDA, 0x38, 0xA0}; // quello montato con piedini su breackboard
  uint8_t peerAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast

  const functionPtr callbackArr[] = { packet_callback_read_dongle };
#elif defined(GUN)
  //const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // espe32s3 con piedini (riceve ma non trasmette)
  ///////////////////////////uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE7, 0x65, 0xD4}; // espe32s3 senz apiedini
  uint8_t peerAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast
  //const uint8_t peerAddress[6] = {0xF0, 0x9E, 0x9E, 0x28, 0x9E, 0xB8}; // DONGLE LILYGO

  const functionPtr callbackArr[] = { packet_callback_read_gun };
#endif
///////////////////////////////////////////////////////////////////

/*****************************
 *   SERIAL WIRELESS SECTION
 *****************************/

SerialWireless_::operator bool() {
  return true;
}

int SerialWireless_::peek() {
  if (lenBufferSerialRead) {
    return bufferSerialRead[_readerSerialRead];
  }
  return -1;
}

int SerialWireless_::peekBin() {
  if (_readLen) {
    return _queue[_reader];
  }
  return -1;
}

int SerialWireless_::read() {
  if (lenBufferSerialRead) {
    uint8_t ret = bufferSerialRead[_readerSerialRead];
    lenBufferSerialRead --;
    _readerSerialRead ++;
    if (_readerSerialRead >= FIFO_SIZE_READ_SERIAL) {
      _readerSerialRead -= FIFO_SIZE_READ_SERIAL;
    }
    return ret;
  }
  return -1;  
}

bool SerialWireless_::checkForRxPacket() {
  uint16_t numAvailableBin = availableBin();
  uint8_t dato;
  for (uint16_t i = 0; i<numAvailableBin; i++) {
    dato = (uint8_t) readBin();
    packet.parse(dato, true);
  }
  return true;
}

int SerialWireless_::readBin() {
  if (_readLen) {
    uint8_t ret = _queue[_reader];
    _readLen --;
    _reader ++;
    if (_reader >= FIFO_SIZE_READ) {
      _reader -= FIFO_SIZE_READ;
    }
    return ret;
  }
  return -1;  
}

int SerialWireless_::available() {
  return lenBufferSerialRead;
}

int SerialWireless_::availableBin() {
  return _readLen;
}

int SerialWireless_::availableForWrite() {
  return FIFO_SIZE_WRITE_SERIAL - lenBufferSerialWrite;
}

int SerialWireless_::availableForWriteBin() {
  return BUFFER_SIZE - _writeLen;
}

void SerialWireless_::flush() {
  if (availableForWriteBin() > (lenBufferSerialWrite + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
    memcpy(&packet.txBuff[PREAMBLE_SIZE], bufferSerialWrite, lenBufferSerialWrite);
    packet.constructPacket(lenBufferSerialWrite, PACKET_TX::SERIAL_TX);
    writeBin(packet.txBuff, lenBufferSerialWrite + PREAMBLE_SIZE+POSTAMBLE_SIZE);
    lenBufferSerialWrite = 0;
  }
  SendData(); // try a send
}

void SerialWireless_::flushBin() {
  SendData();
}

void SerialWireless_::SendData() {
  uint16_t dataAvailable = _writeLen;
  if (dataAvailable > 0) {
    uint16_t len_tx = dataAvailable > ESP_NOW_MAX_DATA_LEN ? ESP_NOW_MAX_DATA_LEN : dataAvailable;
    uint16_t bytesToSendEnd = len_tx > (BUFFER_SIZE - readIndex) ? BUFFER_SIZE - readIndex : len_tx;
    memcpy(buffer_espnow, &buffer[readIndex], bytesToSendEnd);
    if (len_tx > bytesToSendEnd) {
      memcpy(&buffer_espnow[bytesToSendEnd], &buffer[0], len_tx - bytesToSendEnd);
    }
    if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
      esp_err_t result = esp_now_send(peerAddress, buffer_espnow, len_tx);
      if (result == ESP_OK) {
        readIndex += len_tx; 
        if (readIndex >= BUFFER_SIZE) { readIndex -= BUFFER_SIZE; }
        _writeLen -= len_tx;  
      } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("ESPNOW not init.");
      } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid argument");
      } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        Serial.println("Internal Error");
      } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Our of memory");
      } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Peer not found.");
      } else if (result == ESP_ERR_ESPNOW_IF) {
        Serial.println("Interface does not match.");
      } else Serial.println("Errore sconosciuto");
    }
  }
}

size_t SerialWireless_::write(uint8_t c) {
  return write(&c, 1);
}

size_t SerialWireless_::writeBin(uint8_t c) {
  return writeBin(&c, 1);
}

size_t SerialWireless_::write(const uint8_t *data, size_t len) { 
  memcpy(&bufferSerialWrite[lenBufferSerialWrite], data, len);
  lenBufferSerialWrite += len;
  flush();
  return len;
}

size_t SerialWireless_::writeBin(const uint8_t *data, size_t len) {
  if ((BUFFER_SIZE - _writeLen) >= len) {
    size_t firstChunk = BUFFER_SIZE - writeIndex;
    if (firstChunk < len) {
      memcpy(&buffer[writeIndex], data, firstChunk);
      writeIndex = len - firstChunk;  
      memcpy(&buffer[0], data + firstChunk, writeIndex);
    }
    else {
      memcpy(&buffer[writeIndex], data, len);
      writeIndex += len;
      if (writeIndex == BUFFER_SIZE) writeIndex = 0;
    }
    _writeLen += len;
    return len;
  }
  else {
    _overflow_write = true;
    Serial.println("Overflow nello scrivere nel BUFFER scrittura");
    return 0;
  }
}

void SerialWireless_::SendPacket(const uint8_t *data, const uint8_t &len,const uint8_t &packetID) { 
  if (availableForWriteBin() > (len + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
    memcpy(&packet.txBuff[PREAMBLE_SIZE], data, len);
    packet.constructPacket(len, packetID);
    writeBin(packet.txBuff, len + PREAMBLE_SIZE + POSTAMBLE_SIZE);
    SendData(); // try_Send
  }
}

void SerialWireless_::write_on_rx_serialBuffer(const uint8_t *data, int len) {
  if ((FIFO_SIZE_READ_SERIAL - lenBufferSerialRead) >= len) {
    size_t firstChunk = FIFO_SIZE_READ_SERIAL - _writerSerialRead;
    if (firstChunk < len) {
      memcpy(&bufferSerialRead[_writerSerialRead], data, firstChunk);
      _writerSerialRead = len - firstChunk;  
      memcpy(&bufferSerialRead[0], data + firstChunk, _writerSerialRead);
    }
    else {
      memcpy(&bufferSerialRead[_writerSerialRead], data, len);
      _writerSerialRead += len;
      if (_writerSerialRead == FIFO_SIZE_READ_SERIAL) _writerSerialRead = 0;
    }
    lenBufferSerialRead += len;
  }
  else {
    _overflow_bufferSerialRead = true;
    Serial.println("Overflow nello scrivere nel BUFFER lettura del SERIAL BUFFER");
  }
}

int SerialWireless_::availablePacket() {
  return numAvailablePacket;
}

void SerialWireless_::begin() {
  configST myConfig; // variabile di utilità per configurazione
  tx_sem = xSemaphoreCreateBinary();

  xSemaphoreGive(tx_sem);
  
  WiFi.mode(WIFI_STA); // ??????????????????????
  
  /*
  esp_err_t err = esp_wifi_start();
  if (err != ESP_OK) {
    log_e("WiFi not started! 0x%x)", err);
    return false;
  }
  */

  //WiFi.macAddress(mac_esp_inteface); // registra il mac addres della scheda

  esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac_esp_inteface);
  if (err != ESP_OK) {
    Serial.println("Failed to read MAC address");
  }
  
  err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed! 0x%x", err);
  }
  
  err = esp_wifi_set_max_tx_power(ESPNOW_WIFI_POWER); // tra 8 e 84 corrispondenti a 2dbm a 20 dbm);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power failed! 0x%x", err);
  }

  WiFi.disconnect();   // ???????????????????

  delay (1000); // ??????????????????????????
  
  err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("esp_now_init failed! 0x%x", err);
  }

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Errore nell'aggiunta del peer");
  }
  
  /*
  //esp_err_t result = esp_now_del_peer(mac_addr);
  //esp_err_t result = esp_now_mod_peer(&peer);
  
  */

  /*
  esp_err_t err = esp_now_deinit();
  if (err != ESP_OK) {
    log_e("esp_now_deinit failed! 0x%x", err);
    return false;
  }
  */


  err = esp_now_register_recv_cb(_esp_now_rx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_recv_cb failed! 0x%x", err);
  }

  err = esp_now_register_send_cb(_esp_now_tx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_send_cb failed! 0x%x", err);
  }

  myConfig.port         = &Serial; // questo andrà tolta - rimasta solo per contabilità =========================================
  myConfig.debug        = false; //true; //false; //true;
  myConfig.debugPort    = &Serial;
  myConfig.timeout      = DEFAULT_TIMEOUT; // 50ms
  myConfig.callbacks    = callbackArr;
  //myConfig.callbacks    = NULL;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  //myConfig.callbacksLen = 0;
  
  packet.begin(myConfig);
  //TinyUSBDevices.onBattery = true;
  stato_connessione_wireless = 0;
  TinyUSBDevices.wireless_mode = WIRELESS_MODE::ENABLE_ESP_NOW_TO_DONGLE;
}

bool SerialWireless_::end() {

  esp_now_del_peer(peerAddress);
  esp_err_t err = esp_now_deinit();
  if (err != ESP_OK) {
    Serial.printf("esp_now_deinit failed! 0x%x", err);
    return false;
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF); // Disattiva il WiFi
  return true;
}

bool SerialWireless_::connection_dongle() {

  uint8_t channel = ESPNOW_WIFI_CHANNEL;   // tra e 1 e 13 (il 14 dovrebbe ssere riservato)
  #define TIMEOUT_TX_PACKET 500 // in millisecondi
  #define TIMEOUT_CHANGE_CHANNEL 3000 // in millisecondi
  #define TIMEOUT_DIALOGUE 6000 // in millisecondi
  unsigned long lastMillis_tx_packet = millis ();
  unsigned long lastMillis_change_channel = millis ();
  unsigned long lastMillis_start_dialogue = millis ();
  uint8_t aux_buffer_tx[13];

  
  // fare il begin qui o nel main setup ?
  
  
  
  aux_buffer_tx[0] = 1;
  memcpy(&aux_buffer_tx[1], SerialWireless.mac_esp_inteface, 6);
  memcpy(&aux_buffer_tx[7], peerAddress, 6);
  //stato_connessione_wireless = 0;

  while (stato_connessione_wireless != 5) {
    if (stato_connessione_wireless == 0) {
      if ((millis() - lastMillis_change_channel) > TIMEOUT_CHANGE_CHANNEL) { 
        channel++;
        if (channel >13) channel = 1;
        if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
          Serial.printf("DONGLE - esp_wifi_set_channel failed!");
        }
        peerInfo.channel = channel;
        if (esp_now_mod_peer(&peerInfo) != ESP_OK) {  // modifica il canale del peer
          Serial.println("DONGLE - Errore nella modifica del canale");
        }             
        //lastMillis_tx_packet = millis (); 
        lastMillis_change_channel = millis ();
      }
      if ((millis() - lastMillis_tx_packet) > TIMEOUT_TX_PACKET) {
        SerialWireless.SendPacket((const uint8_t *)&aux_buffer_tx, sizeof(aux_buffer_tx), PACKET_TX::CONNECTION);
        Serial.print("DONGLE - inviato pacchetto broadcast sul canale: ");
        Serial.println(channel);
        lastMillis_tx_packet = millis (); 
      }
      lastMillis_start_dialogue = millis();
    }
    else {
      if (((millis() - lastMillis_start_dialogue) > TIMEOUT_DIALOGUE) && stato_connessione_wireless != 5) {
        stato_connessione_wireless = 0;
        //lastMillis_tx_packet = millis ();
        Serial.println("DONGLE - Non si è conclusa la negoziazione tra DONGLE/GUN e si riparte da capo");
        lastMillis_change_channel = millis ();
      }  
    }
    yield(); // in attesa dello stabilimento di una connessione
  }
  
  Serial.println("DONGLE - Negosazione completata - associazione dei dispositivi GUN/DONGLE");
  if (esp_now_del_peer(peerAddress) != ESP_OK) {  // cancella il broadcast dai peer
    Serial.println("DONGLE - Errore nella cancellazione del peer broadcast");
  }
  memcpy(peerAddress, mac_esp_another_card, 6);
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  //peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  //peerInfo.encrypt = false;              
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // inserisce il dongle nei peer
    Serial.println("DONGLE - Errore nell'aggiunta del nuovo peer della GUN");
  }
  TinyUSBDevices.onBattery = true;
  return true;
}


bool SerialWireless_::connection_gun() {

  // fare il begin qui o nel main setup ?

  #define TIMEOUT_GUN_DIALOGUE 1000 // in millisecondi
  unsigned long lastMillis_start_dialogue = millis ();

  // fare il begin qui o nel main setup ?

  //stato_connessione_wireless = 0;

  
  lastMillis_start_dialogue = millis ();

  while(!TinyUSBDevice.mounted() && stato_connessione_wireless != 4) { 
    if (stato_connessione_wireless == 0) {
      lastMillis_start_dialogue = millis ();
    }
    if (((millis() - lastMillis_start_dialogue) > TIMEOUT_GUN_DIALOGUE) && stato_connessione_wireless != 4) {
      stato_connessione_wireless = 0;
    }
  }

  if (stato_connessione_wireless == 4) {
    if (esp_now_del_peer(peerAddress) != ESP_OK) {  // cancella il broadcast dai peer
      Serial.println("Errore nella cancellazione del peer");
    }
    memcpy(peerAddress, mac_esp_another_card, 6);
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    //peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    //peerInfo.encrypt = false;              
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // inserisce il dongle nei peer
      Serial.println("Errore nell'aggiunta del nuovo peer");
    }                       
    TinyUSBDevices.onBattery = true;
    return true;
  }
  return false;
}


void packet_callback_read_dongle() {
  switch (SerialWireless.packet.currentPacketID()) {
    case PACKET_TX::SERIAL_TX:
      Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
      break;
    case PACKET_TX::MOUSE_TX :
      usbHid.sendReport(HID_RID_e::HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);  
      break;
    case PACKET_TX::GAMEPADE_TX:
      usbHid.sendReport(HID_RID_e::HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
      break;
    case PACKET_TX::KEYBOARD_TX:
      usbHid.sendReport(HID_RID_e::HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
      break; 
    case PACKET_TX::CONNECTION:
      // code
      //tipo_connessione = (uint8_t)SerialWireless.packet.rxBuff[PREAMBLE_SIZE];
      memcpy(aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], 13);
      Serial.println("DONGLE - arrivato richiesta di connessione");
      switch (aux_buffer[0]) {
        case 2:
          // code
          if ((memcmp(&aux_buffer[7],SerialWireless.mac_esp_inteface,6) == 0) && SerialWireless.stato_connessione_wireless == 0) {
            memcpy(SerialWireless.mac_esp_another_card, &aux_buffer[1], 6);
            aux_buffer[0] = 3;
            memcpy(&aux_buffer[1], SerialWireless.mac_esp_inteface, 6);
            memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
            SerialWireless.SendPacket((const uint8_t *)&aux_buffer, sizeof(aux_buffer), PACKET_TX::CONNECTION);
            Serial.println("DONGLE - inviato pacchetto di conferma di connessione");

            // assicurati che i dati siano stati spediti
 
            SerialWireless.stato_connessione_wireless = 3;
          }
          break;
        case 4:  
          // code
          if ((memcmp(&aux_buffer[1],SerialWireless.mac_esp_another_card,6) == 0) && 
             (memcmp(&aux_buffer[7],SerialWireless.mac_esp_inteface,6) == 0) &&
             SerialWireless.stato_connessione_wireless == 3) {           
            #ifdef COMMENTO
            if (esp_now_del_peer(peerAddress) != ESP_OK) {  // cancella il broadcast dai peer
              Serial.println("Errore nella cancellazione del peer");
            }
            memcpy(peerAddress, SerialWireless.mac_esp_another_card, 6);
            memcpy(peerInfo.peer_addr, peerAddress, 6);
            //peerInfo.channel = ESPNOW_WIFI_CHANNEL;
            //peerInfo.encrypt = false;              
            if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // inserisce il dongle nei peer
              Serial.println("Errore nell'aggiunta del nuovo peer");
            }
            #endif // COMMENTO
            SerialWireless.stato_connessione_wireless = 5;
            Serial.println("DONGLE - ricevuto pacchetto di avvenuta connessione");
            //TinyUSBDevices.onBattery = true;
        }
          break;
        default:
          break;
      }
      break;  
    default:
      break;
  }
}

void packet_callback_read_gun() {
  switch (SerialWireless.packet.currentPacketID()) {
    case PACKET_TX::SERIAL_TX:
      SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
      break;
    case PACKET_TX::MOUSE_TX :
      /* code */
      break;
    case PACKET_TX::GAMEPADE_TX:
      /* code */
      break;
    case PACKET_TX::KEYBOARD_TX:
      /* code */  
      break; 
    case PACKET_TX::CONNECTION:
      // code
      //tipo_connessione = (uint8_t)SerialWireless.packet.rxBuff[PREAMBLE_SIZE];
      memcpy(aux_buffer, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], 13);
      switch (aux_buffer[0]) {
        case 1:
          if (SerialWireless.stato_connessione_wireless == 0) { // prende la prima dongle disposnibile
            /* code */
            
            memcpy(SerialWireless.mac_esp_another_card, &aux_buffer[1], 6);
            // memcpy(peerAddress, &aux_buffer[1], 6);
            // invia richiesta connessione
            aux_buffer[0] = 2;
            memcpy(&aux_buffer[1], SerialWireless.mac_esp_inteface, 6);
            memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
            SerialWireless.SendPacket((const uint8_t *)&aux_buffer, sizeof(aux_buffer), PACKET_TX::CONNECTION);
            SerialWireless.stato_connessione_wireless = 2;
            // assicurati che i dati siano stati spediti

          }
          break;
        case 2:
          /* code */
          break;
        case 3:
          /* code */
          if ((memcmp(&aux_buffer[1],SerialWireless.mac_esp_another_card,6) == 0) && 
             (memcmp(&aux_buffer[7],SerialWireless.mac_esp_inteface,6) == 0) &&
              SerialWireless.stato_connessione_wireless == 2) {
              
              aux_buffer[0] = 4;
              memcpy(&aux_buffer[1], SerialWireless.mac_esp_inteface, 6);
              memcpy(&aux_buffer[7], SerialWireless.mac_esp_another_card, 6);
              SerialWireless.SendPacket((const uint8_t *)&aux_buffer, sizeof(aux_buffer), PACKET_TX::CONNECTION);
              
              // assicurati che i dati siano stati spediti

              // aspetta fino a quando ha spedito il pacchetto
              #ifdef COMMENTO
              if (esp_now_del_peer(peerAddress) != ESP_OK) {  // cancella il broadcast dai peer
                  Serial.println("Errore nella cancellazione del peer");
                }
                memcpy(peerAddress, SerialWireless.mac_esp_another_card, 6);
                memcpy(peerInfo.peer_addr, peerAddress, 6);
                //peerInfo.channel = ESPNOW_WIFI_CHANNEL;
                //peerInfo.encrypt = false;              
                if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // inserisce il dongle nei peer
                  Serial.println("Errore nell'aggiunta del nuovo peer");
                }            
               #endif // COMMENTO               
                //TinyUSBDevices.onBattery = true;

                SerialWireless.stato_connessione_wireless = 4;
          }
          break;        
        default:
          break;
      }
      break;
    default:
      break;
  }
}

static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if ((FIFO_SIZE_READ - SerialWireless._readLen) >= len) {
    size_t firstChunk = FIFO_SIZE_READ - SerialWireless._writer;
    if (firstChunk < len) {
      memcpy(&SerialWireless._queue[SerialWireless._writer], data, firstChunk);
      SerialWireless._writer = len - firstChunk;  
      memcpy(&SerialWireless._queue[0], data + firstChunk, SerialWireless._writer);
    }
    else {
      memcpy(&SerialWireless._queue[SerialWireless._writer], data, len);
      SerialWireless._writer += len;
      if (SerialWireless._writer == FIFO_SIZE_READ) SerialWireless._writer = 0;
    }
    SerialWireless._readLen += len;
  }
  else {
    SerialWireless._overflow_read = true;
    Serial.println("Overflow nello scrivere nel BUFFER lettura");
  }
  SerialWireless.checkForRxPacket();
}

static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  xSemaphoreGive(/*SerialWireless.*/tx_sem);
  SerialWireless.SendData();
}

#endif //OPENFIRE_WIRELESS_ENABLE