#ifdef OPENFIRE_WIRELESS_ENABLE

#include "OpenFIRE_Packet.h"


//PacketCRC crc;  0x9B = 10011011 = 155
//#define POLY_CRC 0x9B
uint8_t poly = 0x9B;
void Packet::generateTable_crc()
	{
		for (uint16_t i = 0; i < 256; ++i)
		{
			int curr = i;

			for (int j = 0; j < 8; ++j)
			{
				if ((curr & 0x80) != 0)
					curr = (curr << 1) ^ (int)poly;
				else
					curr <<= 1;
			}

			csTable_crc[i] = (byte)curr;
		}
	}
	
	uint8_t Packet::calculate_crc(const uint8_t& val)
	{
		if (val < 256)
			return csTable_crc[val];
		return 0;
	}
	
	uint8_t Packet::calculate_crc(uint8_t arr[], uint8_t len)
	{
		uint8_t crc = 0;
		for (uint16_t i = 0; i < len; i++)
			crc = csTable_crc[crc ^ arr[i]];

		return crc;
	}



/*
 void Packet::begin(const configST& configs)
 Description:
 ------------
  * Advanced initializer for the Packet Class
 Inputs:
 -------
  * const configST& configs - Struct that holds config
  values for all possible initialization parameters
 Return:
 -------
  * void
*/
void Packet::begin(const configST& configs)
{
	port         = configs.port; 
	debugPort    = configs.debugPort; 
	debug        = configs.debug;
	callbacks    = configs.callbacks;
	callbacksLen = configs.callbacksLen;
	timeout 	 = configs.timeout;
	
	txBuff[0] = START_BYTE;
	generateTable_crc();
}


/*
 void Packet::begin(const bool& _debug, Stream& _debugPort, const uint32_t& _timeout)
 Description:
 ------------
  * Simple initializer for the Packet Class
 Inputs:
 -------
  * const bool& _debug - Whether or not to print error messages
  * Stream &_debugPort - Serial port to print error messages
  * const uint32_t& _timeout - Number of ms to wait before
  declaring packet parsing timeout
 Return:
 -------
  * void
*/
void Packet::begin(Stream& _port, const bool& _debug, Stream& _debugPort, const uint32_t& _timeout)
{
	port      = &_port;
	debugPort = &_debugPort;
	debug     = _debug;
	timeout   = _timeout;

	txBuff[0] = START_BYTE;
	generateTable_crc();
}


/*
 uint8_t Packet::constructPacket(const uint16_t& messageLen, const uint8_t& packetID)
 Description:
 ------------
  * Calculate, format, and insert the packet protocol metadata into the packet transmit
  buffer
 Inputs:
 -------
  * const uint16_t& messageLen - Number of values in txBuff
  to send as the payload in the next packet
  * const uint8_t& packetID - The packet 8-bit identifier
 Return:
 -------
  * uint8_t - Number of payload bytes included in packet
*/
uint8_t Packet::constructPacket(const uint8_t& messageLen, const uint8_t& packetID)
{
		//txBuff[0] = START_BYTE; //
		//txBuff[1] = overheadByte;
		txBuff[2] = packetID;
		txBuff[3] = messageLen;

		uint8_t crcVal = calculate_crc(&txBuff[2], messageLen+2);
		txBuff[messageLen + PREAMBLE_SIZE] = crcVal;
	
		calcOverhead(&txBuff[PREAMBLE_SIZE], (uint8_t)messageLen+1);
		stuffPacket(&txBuff[PREAMBLE_SIZE], (uint8_t)messageLen+1);
		//uint8_t crcVal = calculate_crc(txBuff, (uint8_t)messageLen);

		//txBuff[1] = packetID;
		//txBuff[2] = overheadByte;
		//txBuff[3] = messageLen;
		txBuff[1] = overheadByte;

		//txBuff[messageLen + PREAMBLE_SIZE] = crcVal;
		txBuff[messageLen + PREAMBLE_SIZE + 1] = STOP_BYTE;

		return (uint8_t)messageLen;
	
}


/*
 uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
 Description:
 ------------
  * Parses incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception. Executes
  callback functions for parsed packets whos ID has a
  corresponding callback function set via
  "void Packet::begin(const configST configs)"
 Inputs:
 -------
  * const uint8_t& recChar - Next char to parse in the stream
  * const bool& valid - Set if stream is "available()" and clear if not
 Return:
 -------
  * uint8_t - Num bytes in RX buffer
*/

uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
{
	bool packet_fresh = (packetStart == 0) || ((millis() - packetStart) < timeout);

	if (!packet_fresh) //packet is stale, start over.
	{
		
		//Serial.println("!packet_fresh");
		
		if (debug)
			debugPort->println("ERROR: STALE PACKET");


		bytesRead   = 0;
		state       = find_start_byte;
		status      = STALE_PACKET_ERROR;
		packetStart = 0;

		return bytesRead;
	}

	if (valid)
	{
		switch (state)
		{
		case find_start_byte: /////////////////////////////////////////
		{
			//Serial.print("find_start_byte: ");
			//Serial.println(recChar);

			if (recChar == START_BYTE)
			{
				state       = find_overhead_byte; //find_id_byte;
				packetStart = millis();	//start the timer
				rxBuff[0] = recChar;
				//Serial.print("Start: ");
			}

			break;
		}

		case find_overhead_byte: //////////////////////////////////////
		{
			//Serial.print("find_overhead_byte: ");
			//Serial.println(recChar);
			
			recOverheadByte = recChar;
			state           = find_id_byte; //find_payload_len;
			rxBuff[1] = recChar;
			break;
		}

		case find_id_byte: ////////////////////////////////////////////
		{
						
			//Serial.print("find_id_byte: ");
			//Serial.println(recChar);

			idByte = recChar;
			state  = find_payload_len; // find_overhead_byte;
			rxBuff[2] = recChar;
			break;
		}

		/*
		case find_overhead_byte: //////////////////////////////////////
		{
			recOverheadByte = recChar;
			state           = find_payload_len;
			break;
		}
		*/

		case find_payload_len: ////////////////////////////////////////
		{
			
			//Serial.print("find_payload_len: ");
			//Serial.println(recChar);
			//Serial.print("find_payload:");
			
			if ((recChar > 0) && (recChar <= MAX_PACKET_SIZE))
			{
				bytesToRec = recChar;
				payIndex   = 0;
				state      = find_payload;
				rxBuff[3] = recChar;
			}
			else
			{
				bytesRead = 0;
				state     = find_start_byte;
				status    = PAYLOAD_ERROR;

				if (debug)
					debugPort->println("ERROR: PAYLOAD_ERROR");

				reset();
				return bytesRead;
			}
			break;
		}

		case find_payload: ////////////////////////////////////////////
		{
			
			//Serial.print(" ");
			//Serial.print(recChar);
			
			if (payIndex < bytesToRec)
			{
				rxBuff[payIndex + PREAMBLE_SIZE] = recChar;
				payIndex++;

				if (payIndex == bytesToRec)
					state    = find_crc;
			}
			break;
		}

		case find_crc: ///////////////////////////////////////////
		{
			//Serial.println();
			//Serial.println("find_crc");
			//Serial.print("CRC letto: ");
			//Serial.println(recChar);
			
			rxBuff[bytesToRec + PREAMBLE_SIZE] = recChar;						
			// SISTEMARE PRIMA I COBS
			unpackPacket(&rxBuff[PREAMBLE_SIZE], bytesToRec + 1);
			// FINITO DI SISTEMARE I COBS
			
			//Serial.print("CRC letto dopo unpackPacket: ");
			//Serial.println(rxBuff[bytesToRec + PREAMBLE_SIZE]);

			uint8_t calcCrc = calculate_crc(&rxBuff[2], bytesToRec+2);

			//Serial.print("CRC calcolato: ");
			//Serial.println(calcCrc);

			//Serial.println("CRC");
			
			if (calcCrc == rxBuff[bytesToRec + PREAMBLE_SIZE])
				state = find_end_byte;
			else
			{
				bytesRead = 0;
				state     = find_start_byte;
				status    = CRC_ERROR;

				if (debug)
					debugPort->println("ERROR: CRC_ERROR");

				reset();
				return bytesRead;
			}

			break;
		}

		case find_end_byte: ///////////////////////////////////////////
		{
			
			//Serial.print("find_end_byte: ");
			//Serial.println(recChar);
			
			state = find_start_byte;

			if (recChar == STOP_BYTE)
			{
				//debugPort->println(" Pacchetto ricevuto CORRETTO");
				//Serial.println(" Pacchetto ricevuto CORRETTO");
				//unpackPacket(rxBuff, __); //// ????????
				bytesRead = bytesToRec;
				status    = NEW_DATA;

				if (callbacks) callbacks[0]();
				/*
				{				
					if (idByte < callbacksLen)
						callbacks[idByte]();
					else if (debug)
					{
						debugPort->print(F("ERROR: No callback available for packet ID "));
						debugPort->println(idByte);
					}
				}
				*/
				packetStart = 0;	// reset the timer
				return bytesToRec;
			}

			bytesRead = 0;
			status    = STOP_BYTE_ERROR;

			if (debug)
				debugPort->println("ERROR: STOP_BYTE_ERROR");

			reset();
			return bytesRead;
			break;
		}

		default:
		{
			
			//Serial.println("default");
			
			if (debug)
			{
				debugPort->print("ERROR: Undefined state ");
				debugPort->println(state);
			}

			reset();
			bytesRead = 0;
			state     = find_start_byte;
			break;
		}
		}
	}
	else
	{
		bytesRead = 0;
		status    = NO_DATA;
		return bytesRead;
	}

	bytesRead = 0;
	status    = CONTINUE;
	return bytesRead;
}


/*
 uint8_t Packet::currentPacketID()
 Description:
 ------------
  * Returns the ID of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - ID of the last parsed packet
*/
uint8_t Packet::currentPacketID()
{
	return idByte;
}


/*
 void Packet::calcOverhead(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Calculates the COBS (Consistent Overhead Stuffing) Overhead
  byte and stores it in the class's overheadByte variable. This
  variable holds the byte position (within the payload) of the
  first payload byte equal to that of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values the overhead is to be calculated
  over
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::calcOverhead(uint8_t arr[], const uint8_t& len)
{
	overheadByte = MAX_COBS;  // 254 ===== //0xFF;

	for (uint8_t i = 0; i < len; i++)
	{
		if (arr[i] == START_BYTE)
		{
			overheadByte = i;
			break;
		}
	}
}


/*
 int16_t Packet::findLast(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Finds last instance of the value START_BYTE within the given
  packet array
 Inputs:
 -------
  * uint8_t arr[] - Packet array
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * int16_t - Index of last instance of the value START_BYTE within the given
  packet array
*/
int16_t Packet::findLast(uint8_t arr[], const uint8_t& len)
{
	for (uint8_t i = (len - 1); i != 0xFF; i--)
		if (arr[i] == START_BYTE)
			return i;

	return -1;
}


/*
 void Packet::stuffPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Enforces the COBS (Consistent Overhead Stuffing) ruleset across
  all bytes in the packet against the value of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values to stuff
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::stuffPacket(uint8_t arr[], const uint8_t& len)
{
	int16_t refByte = findLast(arr, len);

	if (refByte != -1)
	{
		for (uint8_t i = (len - 1); i != 0xFF; i--)
		{
			if (arr[i] == START_BYTE)
			{
				arr[i]  = refByte - i;
				refByte = i;
			}
		}
	}
}


/*
 void Packet::unpackPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Unpacks all COBS-stuffed bytes within the array
 Inputs:
 -------
  * uint8_t arr[] - Array of values to unpack
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::unpackPacket(uint8_t arr[], const uint8_t &len)
{
	uint8_t testIndex = recOverheadByte;
	uint8_t delta     = 0;

	//if (testIndex <= MAX_PACKET_SIZE + 1)
	if (testIndex != MAX_COBS)
	{
		while (arr[testIndex])
		{
			delta          = arr[testIndex];
			arr[testIndex] = START_BYTE;
			testIndex += delta;
		}
		arr[testIndex] = START_BYTE;
	}
}


/*
 void Packet::reset()
 Description:
 ------------
  * Clears out the tx, and rx buffers, plus resets
  the "bytes read" variable, finite state machine, etc
 Inputs:
 -------
  * void
 Return:
 -------
  * void
*/
void Packet::reset()
{
	//memset(txBuff, 0, sizeof(txBuff));
	//memset(rxBuff, 0, sizeof(rxBuff));

	bytesRead   = 0;
	packetStart = 0;
}

/*
void Packet::reset_fifo()
{
	while (port->available())
		port->read();

	reset();
	//status = packet.status;
}
*/

/*
uint8_t Packet::sendData(const uint16_t& messageLen, const uint8_t packetID)
{
	
	
	uint8_t numBytesIncl;

	numBytesIncl = constructPacket(messageLen, packetID);
	//port->write(preamble, sizeof(preamble));
	port->write(txBuff, numBytesIncl+PREAMBLE_SIZE+POSTAMBLE_SIZE);
	//port->write(postamble, sizeof(postamble));

	return numBytesIncl;

	
}
*/

/*
uint8_t Packet::available()
{
	bool    valid   = false;
	uint8_t recChar = 0xFF;

	if (port->available())
	{
		valid = true;

		while (port->available())
		{
			recChar = port->read();

			bytesRead = parse(recChar, valid);
			//status    = packet.status;

			if (status != CONTINUE)
			{
				if (status < 0)
					reset_fifo();

				break;
			}
		}
	}
	else
	{
		bytesRead = parse(recChar, valid);
		//status    = packet.status;

		if (status < 0)
			reset_fifo();
	}

	return bytesRead;
}
*/
/*
bool Packet::tick()
{
	if (available())
		return true;

	return false;
}

*/


#endif //OPENFIRE_WIRELESS_ENABLE