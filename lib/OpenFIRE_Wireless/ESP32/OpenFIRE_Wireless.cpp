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


// ====================================================================================================
// ====================================================================================================
// ====================================================================================================

/////////////////////////////////////////////////////////////////// Callbacks
static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len); // callback esp_now
static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status); // callback esp_now

void packet_callback_read_dongle(); // callback packet 
void packet_callback_read_gun(); // callback packet

#if defined(DONGLE)
  const uint8_t peerAddress[6] = {0x24, 0x58, 0x7C, 0xDA, 0x38, 0xA0}; // quello montato con piedini su breackboard

  const functionPtr callbackArr[] = { packet_callback_read_dongle };
#elif defined(GUN)
  //const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // espe32s3 con piedini (riceve ma non trasmette)
  const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE7, 0x65, 0xD4}; // espe32s3 senz apiedini
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
  /*
  if (OpenFIRE_serialTransfer.bytesRead) {
    return OpenFIRE_serialTransfer.packet.rxBuff[0];
    //return OpenFIRE_serialTransfer.packet.rxBuff[OpenFIRE_serialTransfer.bytesRead - 1];
  }
  return -1;
  */
 if (lenBufferSerialRead) {
  return bufferSerialRead[_readerSerialRead];
}
return -1;

}

int SerialWireless_::peekBin() {
  /*
  if (OpenFIRE_serialTransfer.bytesRead) {
    return OpenFIRE_serialTransfer.packet.rxBuff[0];
    //return OpenFIRE_serialTransfer.packet.rxBuff[OpenFIRE_serialTransfer.bytesRead - 1];
  }
  return -1;
  */
 if (_readLen) {
  return _queue[_reader];
}
return -1;

}

int SerialWireless_::read() {
  //if (_readLen_atomic.load()) {
    if (lenBufferSerialRead) {
      uint8_t ret = bufferSerialRead[_readerSerialRead];
      //uint16_t aux_reader = _reader;
      //uint8_t ret = _queue[aux_reader];
      lenBufferSerialRead --;

      //asm volatile("" ::: "memory"); // Ensure the value is read before advancing
      // Aggiorna _reader senza l'uso dell'operatore %
      //uint16_t next_reader = (_reader + 1);
      _readerSerialRead ++;
      if (_readerSerialRead >= FIFO_SIZE_READ_SERIAL) {
        _readerSerialRead -= FIFO_SIZE_READ_SERIAL;
      }
      //asm volatile("" ::: "memory"); // Ensure the reader value is only written once, correctly
      //_reader = next_reader;
      
      //_reader = aux_reader;
      //_readLen --;
      //_readLen_atomic.fetch_sub(1);
      return ret;
  }
return -1;  

}


// ===============================================================================================================
#ifdef COMMENDO
  #define PACKET_CONTINUE             3
  #define PACKET_NEW_DATA             2
  #define PACKET_NO_DATA              1 
  #define PACKET_CRC_ERROR            0
  #define PACKET_PAYLOAD_ERROR       -1
  #define PACKET_STOP_BYTE_ERROR     -2
  #define PACKET_STALE_PACKET_ERROR  -3
#endif // COMMENTO

bool SerialWireless_::checkForRxPacket() {

// AGGIUNGERE UN TIMEOUT e USCIRE QUANDO LETTO TUTTO UN PACCHETTO  - decidere altrimenti se usare la callback
//bool packetComplete = false;
uint16_t numAvailableBin = availableBin();
uint8_t dato;
for (uint16_t i = 0; i<numAvailableBin; i++)
//for (uint16_t i = 0; i<numAvailableBin && !packetComplete; i++)
//while (availableBin() && !packetComplete) 
{
  //uint8_t dato;
  /*
  dato = ESP32_NOW_Serial_OpenFIRE.read();
  ESP32_NOW_Serial_OpenFIRE.read(&dato, 1);
  */
  dato = (uint8_t) readBin();
  packet.parse(dato, true);
  //if (dato==START_BYTE) Serial.println(" === START_BYTE ===");

#ifdef COMMENTO

switch (packet.status)
{
case PACKET_CONTINUE:
  /* code */
  //Serial.println("PACKET_CONTINUE");
  break;
case PACKET_NEW_DATA :
  // =======================================================================
  switch (packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    write_on_rx_serialBuffer(&packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    usbHid.sendReport(HID_RID_MOUSE, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    usbHid.sendReport(HID_RID_GAMEPAD, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    usbHid.sendReport(HID_RID_KEYBOARD, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break; 
  default:
    break;
  }
  // ============================================================
  
  /* code */
  //Serial.println("PACKET_NEW_DATA");
  ///////////////////////////////////////////Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],SerialWireless.packet.bytesRead);
  //Serial.println();
  //Serial.print("PACCHHETTO IN NUMERI: ");
  /*
  for (uint8_t i =0 ; i<SerialWireless.packet.bytesRead;i++)
  {
     Serial.print(SerialWireless.packet.rxBuff[PREAMBLE_SIZE+i]);
     Serial.print(" ");
  }
  Serial.println();
  */
  break;
case PACKET_NO_DATA :
  /* code */
  //Serial.println("PACKET_NO_DATA");
  break;
case PACKET_CRC_ERROR:
  /* code */
  //Serial.println("PACKET_CRC_ERROR");
  break;
case PACKET_PAYLOAD_ERROR :
  /* code */
  //Serial.println("PACKET_PAYLOAD_ERROR");
  break;
case PACKET_STOP_BYTE_ERROR:
  /* code */
  //Serial.println("PACKET_STOP_BYTE_ERROR");
  break;
case PACKET_STALE_PACKET_ERROR:
  /* code */
  //Serial.println("PACKET_STALE_PACKET_ERROR");
  break;
default:
  //Serial.println("PACKET_ALTRO");
  break;
}

/*
  if (SerialWireless.packet.status==NEW_DATA) Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],SerialWireless.packet.bytesRead);
  //Serial.write(&dato,1);
*/
#endif // COMMENTO

}
  return true;
}

// =====================================================================================================

int SerialWireless_::readBin() {
  //if (_readLen_atomic.load()) {
    if (_readLen) {
        uint8_t ret = _queue[_reader];
        //uint16_t aux_reader = _reader;
        //uint8_t ret = _queue[aux_reader];
        _readLen --;

        //asm volatile("" ::: "memory"); // Ensure the value is read before advancing
        // Aggiorna _reader senza l'uso dell'operatore %
        //uint16_t next_reader = (_reader + 1);
        _reader ++;
        if (_reader >= FIFO_SIZE_READ) {
            _reader -= FIFO_SIZE_READ;
        }
        //asm volatile("" ::: "memory"); // Ensure the reader value is only written once, correctly
        //_reader = next_reader;
        
        //_reader = aux_reader;
        //_readLen --;
        //_readLen_atomic.fetch_sub(1);
        return ret;
    }
  return -1;  

  /*
  uint8_t _queue[FIFO_SIZE_READ]; //poi cambiare


  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;

    recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);
  
    //OpenFIRE_serialTransfer.bytesRead;
    //OpenFIRE_serialTransfer.packet.rxBuff[i];
  }
  // dai un carattere
  return -1;
  */
}

int SerialWireless_::available() {
  return lenBufferSerialRead;
}

int SerialWireless_::availableBin() {
  return _readLen;
  /*
  return OpenFIRE_serialTransfer.available();
  //return _readLen;
*/
#ifdef COMMENTO  
if (OpenFIRE_serialTransfer.available())
  {
    if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
      uint16_t recSize = 0;
      uint16_t size;
      size = OpenFIRE_serialTransfer.bytesRead;
      //recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);

      // =====================================================================================================
      // =====================================================================================================
      //if ((FIFO_SIZE_READ - _readLen_atomic.load()) >= size) {
        if ((FIFO_SIZE_READ - _readLen) >= size) {
            size_t firstChunk = FIFO_SIZE_READ - _writer;
            //size_t firstChunk = FIFO_SIZE_READ - _writer_atomic.load();
            if (firstChunk > size) {
                firstChunk = size;
            }
            // Copia il primo blocco di dati nel buffer circolare
            memcpy(&_queue[_writer], OpenFIRE_serialTransfer.packet.rxBuff, firstChunk);
            //memcpy(&_queue[_writer_atomic.load()], packet, firstChunk);
            //asm volatile("" ::: "memory");
            //_writer += firstChunk;
            _writer += firstChunk;
            //asm volatile("" ::: "memory");
            if (_writer == FIFO_SIZE_READ) {
                //asm volatile("" ::: "memory");
                _writer = 0;
                //asm volatile("" ::: "memory");
            }
            uint16_t remaining = size - firstChunk;
            if (remaining > 0) {
                memcpy(&_queue[_writer], OpenFIRE_serialTransfer.packet.rxBuff + firstChunk, remaining);
                //asm volatile("" ::: "memory");
                _writer += remaining;
                //asm volatile("" ::: "memory");
                /*
                if (_writer == _fifoSize) {
                    _writer = 0;
                }
                */
            }
            //asm volatile("" ::: "memory");
            _readLen += size;
            //_readLen_atomic.fetch_add(size);
            //asm volatile("" ::: "memory"); // Assicura che la coda venga scritta prima di avanzare il conteggio scritt
        } else {
            _overflow = true;       
        }
        //mutex_exit(&_mutex_read);       
    //}


      // =====================================================================================================
      // =====================================================================================================





    }
  }
  return _readLen;
  #endif //COMMENTO
}

int SerialWireless_::availableForWrite() {
  return FIFO_SIZE_WRITE_SERIAL - lenBufferSerialWrite;
  
  //return (ESP32_NOW_Serial_OpenFIRE.availableForWrite() - PREAMBLE_SIZE - POSTAMBLE_SIZE);
}

int SerialWireless_::availableForWriteBin() {
  return BUFFER_SIZE - _writeLen;
  
  //return (ESP32_NOW_Serial_OpenFIRE.availableForWrite() - PREAMBLE_SIZE - POSTAMBLE_SIZE);
}

void SerialWireless_::flush() {
  uint8_t sendSize = 0;
  if (availableForWriteBin() > (lenBufferSerialWrite + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
    sendSize = packet.txObj(bufferSerialWrite, sendSize, lenBufferSerialWrite);
    packet.constructPacket(lenBufferSerialWrite, PACKET_TX::SERIAL_TX);
    //SerialWireless.packet.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
    writeBin(SerialWireless.packet.txBuff, lenBufferSerialWrite + PREAMBLE_SIZE+POSTAMBLE_SIZE);
    lenBufferSerialWrite = 0;
  }
  
  
  SendData(); // try a send
  /*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
  */
}

void SerialWireless_::flushBin() {
  SendData();
  /*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
  */
}


void SerialWireless_::SendPacket() {

}

void SerialWireless_::SendData() {
  //uint8_t buffer_espnow[ESP_NOW_MAX_DATA_LEN];
  
  //Serial.println("Entrato deltro SendaData");
  uint16_t dataAvailable = _writeLen;
  if (dataAvailable > 0)
  {
    uint16_t len_tx = dataAvailable > ESP_NOW_MAX_DATA_LEN ? ESP_NOW_MAX_DATA_LEN : dataAvailable;
  
    //Serial.println("Entrato dentroa dataAvailable");
    // Copia i dati nel buffer di uscita
    //uint16_t bytesToSendEnd = min(len_tx, BUFFER_SIZE - readIndex);
    uint16_t bytesToSendEnd = len_tx > (BUFFER_SIZE - readIndex) ? BUFFER_SIZE - readIndex : len_tx;
    memcpy(buffer_espnow, &buffer[readIndex], bytesToSendEnd);
    if (len_tx > bytesToSendEnd) {
      memcpy(&buffer_espnow[bytesToSendEnd], &buffer[0], len_tx - bytesToSendEnd);
    }

    if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    esp_err_t result = esp_now_send(peerAddress, buffer_espnow, len_tx);
    //Serial.println("Entrato esp_now_send");
    if (result == ESP_OK) {
    
      readIndex += len_tx; 
      if (readIndex >= BUFFER_SIZE) { readIndex -= BUFFER_SIZE; }
      _writeLen -= len_tx;  
    
      /////////////Serial.println("INVIATO CORRETTAMENTE da esp_now_send");
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

/*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
*/

}

size_t SerialWireless_::write(uint8_t c) {
  //sendSize = OpenFIRE_serialTransfer.txObj(c, sendSize);
  return write(&c, 1);
}


size_t SerialWireless_::writeBin(uint8_t c) {
  //sendSize = OpenFIRE_serialTransfer.txObj(c, sendSize);
  return writeBin(&c, 1);
}


size_t SerialWireless_::write(const uint8_t *data, size_t len) { 
  /*
  #define FIFO_SIZE_WRITE_SERIAL 200
  #define TIME_OUT_SERIAL_WRITE 3
  uint8_t bufferSerialWrite[FIFO_SIZE_WRITE_SERIAL];
  unsigned long startTimeSerialWrite = 0; // = millis();
  volatile uint16_t lenBufferSerialWrite = 0;
  */
  
  /////////////////////////uint8_t *_queue;

    //bool inizializzato = false;
    //uint16_t sendSize = 0;
    //int rx_avalaible = 0;
    //unsigned long startTimeSerialWrite = 0; // = millis();
    //volatile uint16_t lenBufferSerialWrite = 0;
  
  //#define TIME_OUT_SERIAL_WRITE 3

  #ifdef COMMENTO
  // =====================
  #define TIME_OUT_AVALAIBLE 3
    
  if (Serial.available() > rx_avalaible)
  {
     startTime = millis();
     rx_avalaible = Serial.available();
    } 
  
  if (rx_avalaible && (millis() > startTime + TIME_OUT_AVALAIBLE)) {
    /////////////rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
    //rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
    //rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
    
    /////////////////Serial.println("Pacchetto SERIALE in arrivo da PC");
    
    if (rx_avalaible > FIFO_SIZE_READ_SER) rx_avalaible= FIFO_SIZE_READ_SER;

    Serial.readBytes(SerialWireless.bufferSerialWrite, rx_avalaible);
    //Serial.write(SerialWireless.bufferSerialWrite, rx_avalaible);
    SerialWireless.lenBufferSerialWrite = rx_avalaible;
    //SerialWireless.printf("Topo gigio come va - millis: %d \n", millis());
    SerialWireless.flush();
    //_queue[rx_avalaible]='\0';
    //Serial.println((char *)_queue);
    sendSize = 0;

    //sendSize = OpenFIRE_serialTransfer.txObj(_queue, sendSize, rx_avalaible);
    //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
    rx_avalaible = 0;
  } 



  // =====================
#endif // COMMENTO

memcpy(&bufferSerialWrite[lenBufferSerialWrite], data, len);
lenBufferSerialWrite += len;
flush();

#ifdef COMMENTO
// =======================
  if (lenBufferSerialWrite == 0)
  {
     startTimeSerialWrite = millis();
  } 
  
  if ((lenBufferSerialWrite + len) > FIFO_SIZE_WRITE_SERIAL) {
    // invia tutto
    flush();
    return 0;
  }
  else
  {
    memcpy(&bufferSerialWrite[lenBufferSerialWrite], data, len);
    lenBufferSerialWrite += len;
    if ((data[len-1] == '\n') || (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE))) {
      // invia tutto
      flush();
      //return len;
    }
    //startTimeSerialWrite = millis();  ========== VALUTARE ==========
  }
// ==========================
#endif //COMMENTO
  
return len;

  /*  
  if (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE))
  {
    //invia tutto
    flush();
    return len;

  }
  */

 /* 
  if (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE)) {
    rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
    //rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
    //rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
    
    Serial.println("Pacchetto SERIALE in arrivo da PC");
    
    if (rx_avalaible > FIFO_SIZE_READ_SERIAL) rx_avalaible= FIFO_SIZE_READ_SERIAL;

    Serial.readBytes(bufferSerial, rx_avalaible);
    bufferSerial[rx_avalaible]='\0';
    Serial.println((char *)bufferSerial);
    sendSize = 0;

    //sendSize = OpenFIRE_serialTransfer.txObj(_queue, sendSize, rx_avalaible);
    //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
    rx_avalaible = 0;
  } 
 */

  //return len;
}

size_t SerialWireless_::writeBin(const uint8_t *data, size_t len) {
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
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
  
  // ==========================================================================================
  /*  
  sendSize = OpenFIRE_serialTransfer.txObj(p, sendSize, len);
  if (p[len-1] == '\n') flush();
  return len; 
  */
}

// ===== INIZIO SCRITTURA ================

void SerialWireless_::SendPacketKeyboard() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketMouse() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketGamepad() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketSerial() {
  
  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
// ====== FINE SCRITTURA ==============

// ====== LETTURA ==============

void SerialWireless_::ReadPacketKeyboard() {

}
void SerialWireless_::ReadPacketMouse() {

}
void SerialWireless_::ReadPacketGamepad() {

}
void SerialWireless_::ReadPacketSerial() {

}
// ===== FINE LETTURA =============

// ===== INIZIO UTILITA' ===========

bool SerialWireless_::checkForTxData() {
/*
  //do we have something that failed the last time?
  resend_count = 0;
  if (queued_buff == NULL) {
    queued_buff = (uint8_t *)xRingbufferReceiveUpTo(tx_ring_buf, &queued_size, 0, ESP_NOW_MAX_DATA_LEN);
  } else {
    log_d(MACSTR " : PREVIOUS", MAC2STR(addr()));
  }
  if (queued_buff != NULL) {
    return tryToSend() > 0;
  }
  //log_d(MACSTR ": EMPTY", MAC2STR(addr()));
  xSemaphoreGive(tx_sem);
  return false;
*/
return tryToSend() > 0;
}

size_t SerialWireless_::tryToSend() {
  /*
  //log_d(MACSTR ": %u", MAC2STR(addr()), queued_size);
  size_t sent = send(queued_buff, queued_size);
  if (!sent) {
    //_onSent will not be called anymore
    //the data is lost in this case
    vRingbufferReturnItem(tx_ring_buf, queued_buff);
    queued_buff = NULL;
    xSemaphoreGive(tx_sem);
    end();
  }
  return sent;
*/
return 0;
}

// ===== FINE UTILITA' =============

void SerialWireless_::write_on_rx_serialBuffer(const uint8_t *data, int len) {
    
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
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
  // ==========================================================================================
  
  // ORA BISOGNA ESAMINARE I DATI PRESENTI NEL BUFFER 
  
  }

int SerialWireless_::availablePacket() {

  return numAvailablePacket;

}

void SerialWireless_::begin() {

  //#define ESPNOW_WIFI_CHANNEL 4
  // la potenza di trasmissione può andare da 8 a 84, dove 84 è il valore massimo che corrisponde a 20 db
  //#define ESPNOW_WIFI_POWER 84 
  
  //esp_now_peer_info_t peerInfo; // variabile di utilità per configurazione
  configST myConfig; // variabile di utilità per configurazione
  

  tx_sem = xSemaphoreCreateBinary();
  //xSemaphoreTake(tx_sem, 0);
  xSemaphoreGive(tx_sem);
  
  WiFi.mode(WIFI_STA);
  
  //WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  
  esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }
  
  err = esp_wifi_set_max_tx_power(ESPNOW_WIFI_POWER); // tra 8 e 84 corrispondenti a 2dbm a 20 dbm);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }
  WiFi.disconnect();
  
  ///WiFi.begin();

  /* // quello buono
  while (!WiFi.STA.started()) {
    delay(100);
  }
  */
  
  //Serial.println("Wi-Fi parameters:");
  //Serial.println("  Mode: STA");
  //Serial.println("  MAC Address: " + WiFi.macAddress());
  //Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);



  #ifndef COMMENTO

  delay (1000);
  /*
  esp_err_t err = esp_wifi_start();
  if (err != ESP_OK) {
    Serial.printf("WiFi not started! 0x%x)", err);
  }
  */

  
  err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("esp_now_init failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

    //esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Errore nell'aggiunta del peer");
    }
  /* // IMPOSTARE SOLO PER CIFRATURA DATI
  if (pmk) {
    err = esp_now_set_pmk(pmk);
    if (err != ESP_OK) {
      log_e("esp_now_set_pmk failed! 0x%x", err);
      _esp_now_has_begun = false;
      return false;
    }
  }
  */

  err = esp_now_register_recv_cb(_esp_now_rx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_recv_cb failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

  err = esp_now_register_send_cb(_esp_now_tx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_send_cb failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

#endif // COMMENTO

/*
if (!ESP32_NOW_Serial_OpenFIRE.begin(0)) {
    //Serial.println("FAILED a inizializzare ESP NOW SERIAL");
    delay (1000);
  }
  else {
    //Serial.println("ESP NOW Serial inizializzato regolarmente");
    delay (1000);
  }
*/
  ////////////////////////////////////////////////////OpenFIRE_SerialWireless = &ESP32_NOW_Serial_OpenFIRE;
  //OpenFIRE_serialTransfer.begin(OpenFIRE_SerialWireless);
  ////////////////////////OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE);

  ///////////////////////////////////////////////////////////////// Config Parameters
  myConfig.port         = &Serial; // questo andrà tolta - rimasta solo per contabilità =========================================
  myConfig.debug        = true; //false; //true;
  myConfig.debugPort    = &Serial;
  myConfig.timeout      = DEFAULT_TIMEOUT; // 50ms
  myConfig.callbacks    = callbackArr;
  /////myConfig.callbacks    = NULL;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////myConfig.callbacksLen = 0;
  /////////////////////////////////////////////////////////////////
  /*
  OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE, myConfig);
  delay(500);
  */

  packet.begin(myConfig);
  TinyUSBDevices.onBattery = true;
  TinyUSBDevices.wireless_mode = WIRELESS_MODE::ENABLE_ESP_NOW_TO_DONGLE;
}

// ============================================================
// ============================================================


#ifdef COMMENTO

void serialTransfer_callback_read() {

  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;
    uint16_t size;
    size = OpenFIRE_serialTransfer.bytesRead;
    //recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);

    // =====================================================================================================
    // =====================================================================================================
    //if ((FIFO_SIZE_READ - _readLen_atomic.load()) >= size) {
      if ((FIFO_SIZE_READ - SerialWireless._readLen) >= size) {
          size_t firstChunk = FIFO_SIZE_READ - SerialWireless._writer;
          //size_t firstChunk = FIFO_SIZE_READ - _writer_atomic.load();
          if (firstChunk > size) {
              firstChunk = size;
          }
          // Copia il primo blocco di dati nel buffer circolare
          memcpy(&SerialWireless._queue[SerialWireless._writer], OpenFIRE_serialTransfer.packet.rxBuff, firstChunk);
          //memcpy(&_queue[_writer_atomic.load()], packet, firstChunk);
          //asm volatile("" ::: "memory");
          //_writer += firstChunk;
          SerialWireless._writer += firstChunk;
          //asm volatile("" ::: "memory");
          if (SerialWireless._writer == FIFO_SIZE_READ) {
              //asm volatile("" ::: "memory");
              SerialWireless._writer = 0;
              //asm volatile("" ::: "memory");
          }
          uint16_t remaining = size - firstChunk;
          if (remaining > 0) {
              memcpy(&SerialWireless._queue[SerialWireless._writer], OpenFIRE_serialTransfer.packet.rxBuff + firstChunk, remaining);
              //asm volatile("" ::: "memory");
              SerialWireless._writer += remaining;
              //asm volatile("" ::: "memory");
              /*
              if (_writer == _fifoSize) {
                  _writer = 0;
              }
              */
          }
          //asm volatile("" ::: "memory");
          SerialWireless._readLen += size;
          //_readLen_atomic.fetch_add(size);
          //asm volatile("" ::: "memory"); // Assicura che la coda venga scritta prima di avanzare il conteggio scritt
      } else {
        SerialWireless._overflow_read = true;       
      }
      //mutex_exit(&_mutex_read);       
  //}


    // =====================================================================================================
    // =====================================================================================================





  }

  /*
  uint8_t _queue[FIFO_SIZE_READ]; //poi cambiare
  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;

    recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);
    //OpenFIRE_serialTransfer.bytesRead;
    //OpenFIRE_serialTransfer.packet.rxBuff[i];
  }
  
  //Serial.println("hi");

  */
}


#endif // COMMENTO


void packet_callback_read_dongle() {
  ///Serial.println("DONGLE packet callback chiamata");
  switch (SerialWireless.packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    
    //VALUTARE SE USARE IL BUFFER O MENO E CONTROLLARE SERIAL.AVALIABLEFORWRITE
    //SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    // inviare dati seriale al PC .. il buffer forse neppure serve
    //controllare quandi dati puoi inviare al PC 'aviablele write' ed inviare solo quielli che si possono inviare, magari fare una ruotine
    // chiama routine per inviare pacchetti al PC

    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    usbHid.sendReport(HID_RID_e::HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    usbHid.sendReport(HID_RID_e::HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    usbHid.sendReport(HID_RID_e::HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break; 
  default:
    break;
  }
}

void packet_callback_read_gun() {
  ///Serial.println("GUN packet callback chiamata");
  switch (SerialWireless.packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    ///Serial.println("packet callback Serial_TX");
    SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    //usbHid.sendReport(HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    //usbHid.sendReport(HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    //usbHid.sendReport(HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break; 
  default:
    break;
  }
}



static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  ///////////////////Serial.printf("%s from " MACSTR ", data length : %u", "RICEVUTO", MAC2STR(info->src_addr), len);
  ///Serial.println("esp now rx callback chiamata");
  
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
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
    // SerialWireless.checkForRxPacket(); // ??????????????????????????????????????????????????

  }
  else {
    SerialWireless._overflow_read = true;
    Serial.println("Overflow nello scrivere nel BUFFER lettura");
  }
  // ==========================================================================================
  SerialWireless.checkForRxPacket();
  // ORA BISOGNA ESAMINARE I DATI PRESENTI NEL BUFFER 


  /*
  bool broadcast = memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
  log_v("%s from " MACSTR ", data length : %u", broadcast ? "Broadcast" : "Unicast", MAC2STR(info->src_addr), len);
  log_buf_v(data, len);
  if (!esp_now_is_peer_exist(info->src_addr) && new_cb != NULL) {
    log_v("Calling new_cb, peer not found.");
    new_cb(info, data, len, new_arg);
    return;
  }
  //find the peer and call it's callback
  for (uint8_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++) {
    if (_esp_now_peers[i] != NULL) {
      log_v("Checking peer " MACSTR, MAC2STR(_esp_now_peers[i]->addr()));
    }
    if (_esp_now_peers[i] != NULL && memcmp(info->src_addr, _esp_now_peers[i]->addr(), ESP_NOW_ETH_ALEN) == 0) {
      log_v("Calling onReceive");
      _esp_now_peers[i]->onReceive(data, len, broadcast);
      return;
    }
  }
*/
  }

static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //////////////Serial.printf(MACSTR " : STATUS : %s", MAC2STR(mac_addr), (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS\n" : "FAILED\n");
  // PRONTO PER TRASMETTERE ANCORA - GESTIRE TRAMITE UN SEMAFORO BINARIO (LIBERARE IL FILE BINARIO)
  xSemaphoreGive(/*SerialWireless.*/tx_sem);
  SerialWireless.SendData();
  
  /*
  log_v(MACSTR " : %s", MAC2STR(mac_addr), (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAILED");
  //find the peer and call it's callback
  for (uint8_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++) {
    if (_esp_now_peers[i] != NULL && memcmp(mac_addr, _esp_now_peers[i]->addr(), ESP_NOW_ETH_ALEN) == 0) {
      _esp_now_peers[i]->onSent(status == ESP_NOW_SEND_SUCCESS);
      return;
    }
  }
  */
  }


  #endif //OPENFIRE_WIRELESS_ENABLE