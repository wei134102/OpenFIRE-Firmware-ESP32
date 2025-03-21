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
  
  WiFi.mode(WIFI_STA);
  esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed! 0x%x", err);
  }
  
  err = esp_wifi_set_max_tx_power(ESPNOW_WIFI_POWER); // tra 8 e 84 corrispondenti a 2dbm a 20 dbm);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power failed! 0x%x", err);
  }
  WiFi.disconnect();

  delay (1000);
  
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
  TinyUSBDevices.onBattery = true;
  TinyUSBDevices.wireless_mode = WIRELESS_MODE::ENABLE_ESP_NOW_TO_DONGLE;
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