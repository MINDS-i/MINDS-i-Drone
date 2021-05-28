#include "MPU6000_HW.h"





// INTERRUPT FROM MPU-6000 DETECTION ROUTINE
volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile uint16_t gyroIrqCount=0;

void dmpDataReady()
{
  mpuInterrupt = true;
  gyroIrqCount++;
}

uint16_t MPU6000_DMP::irqCount()
{
	return gyroIrqCount;
}

uint16_t MPU6000_DMP::irqCountClear()
{
	gyroIrqCount=0;
}


void MPU6000_DMP::update()
{
	//if we have data ready (interrupt) or if there is data in fifo
	if (1) //( mpuInterrupt == true || m_fifoCount >= m_packetSize )
	{
		//todo: *fix?* it is possible that interrupt maybe fire between above and now.
		

		mpuInterrupt = false;

		// by reading INT_STATUS register, all interrupts are cleared
		byte mpuIntStatus = SPIread(0x3A, m_chipSelect); 

		// get current FIFO count
		m_fifoCount = getFIFOCount(m_chipSelect);
  
  		// check for FIFO overflow 
  		// mpuIntStatus & 0x10 checks register 0x3A for FIFO_OFLOW_INT
    	// the size of the FIFO buffer is 1024 bytes, but max. set to 1008 so after 24 packets of 42 bytes
    	// the FIFO is reset, otherwise the last FIFO reading before reaching 1024 contains only 16 bytes
    	// and can/will produces output value miscalculations 
  		if ((mpuIntStatus & 0x10) || m_fifoCount == 1008)
  		{
  			// FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
  			SPIwriteBit(0x6A, 2, true, m_chipSelect); 
  		}
  		else if (mpuIntStatus & 0x02)
  		{
  			byte trycount=0;
  			//check for DMP data ready interrupt (this should happen frequently)

			// wait for correct available data length, should be a VERY short wait
    		while (m_fifoCount < m_packetSize || trycount++ < 3)
    		{
    			m_fifoCount = getFIFOCount(m_chipSelect);
    		}


    		if (m_fifoCount >= m_packetSize)
    		{
				// read a packet from FIFO
    			SPIreadBytes(0x74, m_packetSize, m_fifoBuffer, m_chipSelect);

    			// track FIFO count here in case there is more than one packet (each of 42 bytes) available
    			// (this lets us immediately read more without waiting for an interrupt)
    			//todo: Might be easier to just read packet size directly at beginning of function
    			m_fifoCount = m_fifoCount - m_packetSize;

				m_raw_q_w = ((m_fifoBuffer[0] << 8)  + m_fifoBuffer[1]);  // W
				m_raw_q_x = ((m_fifoBuffer[4] << 8)  + m_fifoBuffer[5]);  // X
				m_raw_q_y = ((m_fifoBuffer[8] << 8)  + m_fifoBuffer[9]);  // Y
				m_raw_q_z = ((m_fifoBuffer[12] << 8) + m_fifoBuffer[13]); // Z
				m_q_w = m_raw_q_w / 16384.0f;
				m_q_x = m_raw_q_x / 16384.0f;
				m_q_y = m_raw_q_y / 16384.0f;
				m_q_z = m_raw_q_z / 16384.0f;
				m_AcceX = ((m_fifoBuffer[28] << 8) + m_fifoBuffer[29]);
				m_AcceY = ((m_fifoBuffer[32] << 8) + m_fifoBuffer[33]);
				m_AcceZ = ((m_fifoBuffer[36] << 8) + m_fifoBuffer[37]);
				m_Ax = m_AcceX / 8192.0f; // calculate g-value
				m_Ay = m_AcceY / 8192.0f; // calculate g-value
				m_Az = m_AcceZ / 8192.0f; // calculate g-value
				m_GyroX = ((m_fifoBuffer[16] << 8) + m_fifoBuffer[17]);
				m_GyroY = ((m_fifoBuffer[20] << 8) + m_fifoBuffer[21]);
				m_GyroZ = ((m_fifoBuffer[24] << 8) + m_fifoBuffer[25]);


 				m_euler_x = atan2((2 * m_q_y * m_q_z) - (2 * m_q_w * m_q_x), (2 * m_q_w * m_q_w) + (2 * m_q_z * m_q_z) - 1); // phi
			    m_euler_y = -asin((2 * m_q_x * m_q_z) + (2 * m_q_w * m_q_y));                                        // theta
			    m_euler_z = atan2((2 * m_q_x * m_q_y) - (2 * m_q_w * m_q_z), (2 * m_q_w * m_q_w) + (2 * m_q_x * m_q_x) - 1); // psi
			    // m_euler_x = m_euler_x * 180/M_PI; // angle in degrees -180 to +180
			    // m_euler_y = m_euler_y * 180/M_PI; // angle in degrees
			    // m_euler_z = m_euler_z * 180/M_PI; // angle in degrees -180 to +180

				m_updateReady=true;

    		}
    		else
    		{
    			//took to long wait until next interrupt
    			//want to avoid lockup condition with infinite loop


    		}

    
  		}


	}
}



void MPU6000_DMP::begin()
{
	SPI.begin();  // start the SPI library
	SPI.setClockDivider(SPI_CLOCK_DIV16); // ArduIMU+ V3 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
	                                    // 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
	SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
	SPI.setDataMode(SPI_MODE0); // latched on rising edge, transitioned on falling edge, active low

	pinMode(m_chipSelect, OUTPUT);
	
	Serial.println("dmpInitialize...");
	m_devStatus = dmpInitialize();
	Serial.println("done");

	if (m_devStatus == 0)
    {
	    // now that it's ready, turn on the DMP 
	    //Serial.print("Enabling DMP... ");
	    SPIwriteBit(0x6A, 7, true, m_chipSelect); // USER_CTRL_DMP_EN
	    //Serial.println("done.");

	    //Serial.print("Enabling interrupt detection... ");
	    // ArduIMU+ V3 has ATMEGA328 INT0 / D2 pin 32 (input) connected to MPU-6000 INT pin 12 (output)
	    attachInterrupt(6, dmpDataReady, RISING); 
	    // by reading INT_STATUS register, all interrupts are cleared
	    byte mpuIntStatus = SPIread(0x3A, m_chipSelect); 
		//Serial.println("done");

	    // set our DMP Ready flag so the main loop() function knows it's okay to use it
	    m_dmpReady = true;
	    
    }
	else
	{
		//todo error?
	}

}




// ############################################################################################## //
// ################################ SPI read/write functions #################################### //
// ############################################################################################## //

// --- Function for SPI reading one byte from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > register contents
byte MPU6000_DMP::SPIread(byte reg, int ChipSelPin)
{
	digitalWrite(ChipSelPin, LOW);     
	SPI.transfer(reg | 0x80);          
	byte read_value = SPI.transfer(0x00); 
	digitalWrite(ChipSelPin, HIGH);    
	return read_value;
}

// --- Function for SPI writing one byte to sensor
// reg        : MPU-6000 register number to write to
// data       : data to be written into reg
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > nothing
void MPU6000_DMP::SPIwrite(byte reg, byte data, int ChipSelPin)
{
	digitalWrite(ChipSelPin, LOW);
	SPI.transfer(reg);
	SPI.transfer(data);
	digitalWrite(ChipSelPin, HIGH);
}

// --- Function for SPI reading one bit from sensor
// reg        : MPU-6000 register number to read from
// bitNum     : bit number in the register to read - 7 (MSB) to 0 (LSB)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > byte 0x00 if bit is 0, otherwise byte with a 1 at bitNum (rest 0's)
byte MPU6000_DMP::SPIreadBit(byte reg, byte bitNum, int ChipSelPin)
{
	byte byte_value = SPIread(reg, ChipSelPin);
	byte bit_value  = byte_value & (1 << bitNum); 
	return bit_value;
}

//--- Function for SPI writing one bit to sensor
// reg        : MPU-6000 register number to write to
// bitNum     : bit number in the register to write to - 7 (MSB) to 0 (LSB)
// databit    : bit value to be written into reg - false or 0 | true or non-zero (1 will be logical)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > nothing
//
// first read byte, then insert bit value, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void MPU6000_DMP::SPIwriteBit(byte reg, byte bitNum, byte databit, int ChipSelPin)
{
	byte byte_value = SPIread(reg, ChipSelPin);
	if (databit == 0)
	{
	 	byte_value = byte_value & ~(1 << bitNum); 
	}
	else // databit is intended to be a "1"
	{
		byte_value = byte_value |  (1 << bitNum); 
	}
	SPIwrite(reg, byte_value, ChipSelPin);
}

//--- Function for SPI reading multiple bytes to sensor
// read multiple bytes from the same device register, most of the times this
// is the FIFO transfer register (which after each read, is automatically
// loaded with new data for the next read)
// reg        : MPU-6000 register number to write to
// length     : number of bytes to be read
// data       : buffer array (starting with [0]) to store the read data in
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > array of data[0 - length]
void MPU6000_DMP::SPIreadBytes(byte reg, unsigned int length, byte *data, int ChipSelPin) 
{
	digitalWrite(ChipSelPin, LOW);
	// wait 10 ms for MPU-6000 to react on chipselect (if this is 4 ms or less, SPI.transfer fails)
	delay(10); 
	SPI.transfer(reg | 0x80); 
	unsigned int count = 0;
	byte data_bytes_printed = 0;
	for (count = 0; count < length; count ++)
	{
		data[count] = SPI.transfer(0x00);
	}
	digitalWrite(ChipSelPin, HIGH);
	
}

//--- Function for SPI reading multiple bits from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
// return     > databits
//
// 01101001 read byte
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
//    010   masked
//   -> 010 shifted
byte MPU6000_DMP::SPIreadBits(byte reg, byte bitStart, byte length, int ChipSelPin)
{
	byte b = SPIread(reg, ChipSelPin);
	byte mask = ((1 << length) - 1) << (bitStart - length + 1);
	b = b & mask;
	b = b >> (bitStart - length + 1);
	return b;
}

//--- Function for SPI writing multiple bits to sensor
// reg        : MPU-6000 register number to write to
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by m_chipSelect)
//
// bbbbb010 -> data (bits to write - leading 0's)
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
// 00011100 mask byte
// 10101111 original reg value (read)
// 10100011 original reg value & ~mask
// 10101011 masked | original reg value
//
// first read byte, then insert bit values, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void MPU6000_DMP::SPIwriteBits(byte reg, byte bitStart, byte length, byte data, int ChipSelPin)
{
	byte byte_value = SPIread(reg, m_chipSelect);
	byte mask = ((1 << length) - 1) << (bitStart - length + 1); // create mask
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask;                     // zero all non-important bits in data (just to make sure)
	byte_value &= ~(mask);            // zero all important bits in existing byte, maintain the rest
	byte_value |= data;               // combine data with existing byte
	SPIwrite(reg, byte_value, ChipSelPin);

	#ifdef DEBUG
	//Serial.print(" bits set: "); 
	//for (byte i = 0; i < (7 - bitStart); i ++) Serial.print("x");
	//for (byte j = 0; j < length; j ++) Serial.print(bitRead(data, bitStart - j));
	//for (byte k = 0; k < (bitStart - length + 1); k ++) Serial.print("x");
	//Serial.println();
	#endif

}


// ############################################################################################## //
// ################################ DMP functions used in dmpInitialize() ####################### //
// ############################################################################################## //
// If you like to know how it works, please read on. Otherwise, just FIRE AND FORGET ;-)

void MPU6000_DMP::setMemoryBank(byte bank, boolean prefetchEnabled, boolean userBank, int ChipSelPin)
{
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)
  bank = bank & 0x1F; // 0x1F = 00011111
  // bank 0: 0 & 0x1F = 00000000 $ 00011111 = 00000000
  // bank 1: 1 & 0x1F = 00000001 $ 00011111 = 00000001
  // bank 2: 2 & 0x1F = 00000010 $ 00011111 = 00000010
  // bank 3: 3 & 0x1F = 00000011 $ 00011111 = 00000011
  // bank 4: 4 & 0x1F = 00000100 $ 00011111 = 00000100
  // bank 5: 5 & 0x1F = 00000101 $ 00011111 = 00000101
  // bank 6: 6 & 0x1F = 00000110 $ 00011111 = 00000110
  // bank 7: 7 & 0x1F = 00000111 $ 00011111 = 00000111
  // is this to maximize the number of banks to 00011111 is 0x1F = 31 ?
  if (userBank) 
  	bank |= 0x20;
  if (prefetchEnabled) 
  	bank |= 0x40;
  SPIwrite(0x6D, bank, ChipSelPin);
}

//***********************************************************//
void MPU6000_DMP::setMemoryStartAddress(byte startaddress, int ChipSelPin)
{
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)
  SPIwrite(0x6E, startaddress, ChipSelPin);
}


//***********************************************************//
boolean MPU6000_DMP::writeDMPMemory()
{
	// - the value in 0x6D activates a specific bank in the DMP
	// - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
	// - register 0x6F is the register from which to read or to which to write the data
	//   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)

	//Serial.print("\tWriting   DMP memory.......... ");

	unsigned int i, j;
	byte dmp_byte;

	// ### there are 8 DMP banks (numbers 0 to 7)

	// DMP banks 0 - 6 are completely filled with 256 bytes:
	for (i = 0; i < 7; i ++)
	{
		//DEBUG_PRINT("@@@ write bank "); DEBUG_PRINTLN(i);
		setMemoryBank(i, false, false, m_chipSelect); // bank number  = i
		setMemoryStartAddress(0, m_chipSelect);       // startaddress = 0 so start writing every DMP bank from the beginning
		digitalWrite(m_chipSelect,LOW);
		SPI.transfer(0x6F);

		for (j = 0; j < 256; j ++) // max. 256 bytes of data fit into one DMP bank
		{
			dmp_byte = pgm_read_byte(dmpMemory + (i * 256) + j);
			SPI.transfer(dmp_byte);
			// #ifdef DEBUG
			// if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
			// Serial.println(dmp_byte, HEX);
			// #endif
		}
		digitalWrite(m_chipSelect,HIGH);
		//DEBUG_PRINTLN();
    }

	// DMP bank 7 gets only 137 bytes:
	//DEBUG_PRINTLN("@@@ write bank 7");
	setMemoryBank(7, false, false, m_chipSelect); // bank number  = 7
	setMemoryStartAddress(0, m_chipSelect);       // startaddress = 0 so start writing also this DMP bank from the beginning
	digitalWrite(m_chipSelect,LOW);
	SPI.transfer(0x6F);

	for (j = 0; j < 137; j ++) // only 137 bytes of data into DMP bank 7
	{
		dmp_byte = pgm_read_byte(dmpMemory + (7 * 256) + j);
		SPI.transfer(dmp_byte);
		// #ifdef DEBUG
		//   if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
		//   Serial.println(dmp_byte, HEX);
		// #endif
	}
	digitalWrite(m_chipSelect,HIGH);
	//DEBUG_PRINTLN();

	//Serial.println("done.");

	return true; // end of writeDMPMemory reached
}

//***********************************************************//
boolean MPU6000_DMP::verifyDMPMemory()
{
	// - the value in 0x6D activates a specific bank in the DMP
	// - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
	// - register 0x6F is the register from which to read or to which to write the data
	//   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)

	//Serial.print("\tVerifying DMP memory.......... ");

	unsigned int i, j;
	byte dmp_byte, check_byte;
	boolean verification = true;

	// ### there are 8 DMP banks (numbers 0 to 7)

	// DMP banks 0 - 6 are completely read, all 256 bytes:
	for (i = 0; i < 7; i ++)
	{
		//DEBUG_PRINT(">>> read bank "); DEBUG_PRINTLN(i);
		setMemoryBank(i, false, false, m_chipSelect); // bank number  = i
		setMemoryStartAddress(0, m_chipSelect);       // startaddress = 0 so start reading every DMP bank from the beginning
		digitalWrite(m_chipSelect,LOW);
		SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

		for (j = 0; j < 256; j ++) // max. 256 bytes of data fit into one DMP bank
		{
			check_byte = pgm_read_byte(dmpMemory + (i * 256) + j);
			dmp_byte = SPI.transfer(0x00);
			if (dmp_byte != check_byte)
			{
				//Serial.println("$$$ dmpMemory: byte verification error");
				verification = false;
			}
			// #ifdef DEBUG
			// if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
			// Serial.println(dmp_byte, HEX);
			// #endif
		}
		digitalWrite(m_chipSelect,HIGH);
		//DEBUG_PRINTLN();
	}

	// DMP bank 7 only read first 137 bytes:
	//DEBUG_PRINTLN(">>> read bank 7");
	setMemoryBank(7, false, false, m_chipSelect); // bank number  = 7
	setMemoryStartAddress(0, m_chipSelect);       // startaddress = 0 so start reading also this DMP bank from the beginning
	digitalWrite(m_chipSelect,LOW);
	SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

	for (j = 0; j < 137; j ++) // only 137 bytes of data into DMP bank 7
	{
		check_byte = pgm_read_byte(dmpMemory + (7 * 256) + j);
		dmp_byte = SPI.transfer(0x00);
		if (dmp_byte != check_byte)
		{
			//Serial.println("$$$ dmpMemory: byte verification error");
			verification = false;
		}
		// #ifdef DEBUG
		// if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
		//   Serial.println(dmp_byte, HEX);
		// #endif
	}
	digitalWrite(m_chipSelect,HIGH);
	//DEBUG_PRINTLN();

	// if (verification == true)  Serial.println("success!");
	// if (verification == false) Serial.println("FAILED!");

	return verification; // true if DMP correctly written, false if not
}

//***********************************************************//
boolean MPU6000_DMP::writeDMPConfig()
{
	byte progBuffer, success, special;
	unsigned int i, j;
	// config set dmpConfig is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	byte bank, offset, length;

	//Serial.print("\tWriting   DMP configuration... ");

	for (i = 0; i < MPU6050_DMP_CONFIG_SIZE;)
	{
		bank   = pgm_read_byte(dmpConfig + i++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
		offset = pgm_read_byte(dmpConfig + i++);
		length = pgm_read_byte(dmpConfig + i++);

		if (length > 0) // regular block of data to write
		{
			//DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
			setMemoryBank(bank, false, false, m_chipSelect); // bank number  = bank
			//DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
			setMemoryStartAddress(offset, m_chipSelect);     // startaddress = offset from the beginning (0) of the bank
			//DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

			digitalWrite(m_chipSelect,LOW);
			SPI.transfer(0x6F);

			for (j = 0; j < length; j++)
			{
				progBuffer = pgm_read_byte(dmpConfig + i + j);
				SPI.transfer(progBuffer);
				//DEBUG_PRINTLNF(progBuffer, HEX);
			}

			digitalWrite(m_chipSelect,HIGH);
			i = i + length;
		}	    
		else // length = 0; special instruction to write
	  	{
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = pgm_read_byte(dmpConfig + i++);
			//DEBUG_PRINTLN("!! Special command code ");
			//DEBUG_PRINTF(special, HEX);
			//DEBUG_PRINTLN(" found...");
			if (special == 0x01)
			{
				// enable DMP-related interrupts (ZeroMotion, FIFOBufferOverflow, DMP)
				SPIwrite(0x38, 0x32, m_chipSelect);  // write 00110010: ZMOT_EN, FIFO_OFLOW_EN, DMP_INT_EN true
				                                    // by the way: this sets all other interrupt enables to false
				success = true;
			}
			else
			{
				// unknown other special command if this may be needed in the future, but for now this should not happen
				success = false;
			}
		}
	}

	//Serial.println("done.");

	return true;
}

//***********************************************************//
boolean MPU6000_DMP::verifyDMPConfig()
{
	byte check_byte, progBuffer, success, special;
	unsigned int i, j;
	// config set dmpConfig is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	byte bank, offset, length;
	boolean verification = true;

	//Serial.print("\tVerifying DMP configuration... ");

	for (i = 0; i < MPU6050_DMP_CONFIG_SIZE;)
	{
		bank   = pgm_read_byte(dmpConfig + i++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
		offset = pgm_read_byte(dmpConfig + i++);
		length = pgm_read_byte(dmpConfig + i++);
	  
		if (length > 0) // regular block of data to read
		{
			//DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
			setMemoryBank(bank, false, false, m_chipSelect); // bank number  = bank
			//DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
			setMemoryStartAddress(offset, m_chipSelect);     // startaddress = offset from the beginning (0) of the bank
			//DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

			digitalWrite(m_chipSelect,LOW);
			SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

			for (j = 0; j < length; j++)
			{
				progBuffer = pgm_read_byte(dmpConfig + i + j);
				check_byte = SPI.transfer(0x00);
				if (progBuffer != check_byte)
				{
					//DEBUG_PRINTLN("$$$ dmpConfig: byte verification error");
					verification = false;
				}
				// #ifdef DEBUG
				//   if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
				//   Serial.println(check_byte, HEX);
				// #endif
			}

			digitalWrite(m_chipSelect,HIGH);
			i = i + length;
		}
		else // length = 0; special instruction to write
		{
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = pgm_read_byte(dmpConfig + i++);
			//DEBUG_PRINT("!! Special command code ");
			//DEBUG_PRINTF(special, HEX);
			//DEBUG_PRINTLN(" found...");
			if (special == 0x01)
			{
				// enable DMP-related interrupts (ZeroMotion, FIFOBufferOverflow, DMP)
				check_byte = SPIread(0x38, m_chipSelect);  // shoudl read 00110010: ZMOT_EN, FIFO_OFLOW_EN, DMP_INT_EN true
				                                          
				if (check_byte != 0x32)
				{
					//DEBUG_PRINTLN("$$$ dmpConfig: byte verification error");
					verification = false;
				}
				// #ifdef DEBUG
				//   if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
				//   Serial.println(check_byte, HEX);
				// #endif

				success = true;
			}
			else
			{
				// unknown special command
				success = false;
			}
		}
	}

	//if (verification == true)  Serial.println("success!");
	//if (verification == false) Serial.println("FAILED!");

	return verification; // true if DMP correctly written, false if not
}

//***********************************************************//
// process only one line from dmpUpdates each time writeDMPUpdates() is called  
unsigned int MPU6000_DMP::writeDMPUpdates(unsigned int pos, byte update_number)
{
	// pos is the current reading position within dmpUpdates
	byte progBuffer, success;
	unsigned int j;
	// config set dmpUpdates is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	byte bank, offset, length;

	//Serial.print("\tWriting   DMP update "); Serial.print(update_number); Serial.print("/7 ..... ");


	bank   = pgm_read_byte(dmpUpdates + pos++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
	offset = pgm_read_byte(dmpUpdates + pos++);
	length = pgm_read_byte(dmpUpdates + pos++);
	  
	//DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
	setMemoryBank(bank, false, false, m_chipSelect); // bank number  = bank
	//DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
	setMemoryStartAddress(offset, m_chipSelect);     // startaddress = offset from the beginning (0) of the bank
	//DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

	digitalWrite(m_chipSelect,LOW);
	SPI.transfer(0x6F);

	for (j = 0; j < length; j++)
	{
		progBuffer = pgm_read_byte(dmpUpdates + pos + j);
		SPI.transfer(progBuffer);
		//DEBUG_PRINTLNF(progBuffer, HEX);
	}
	   
	digitalWrite(m_chipSelect,HIGH);
	pos = pos + length;
	//DEBUG_PRINT("!! last position written: "); DEBUG_PRINTLN(pos);

	//Serial.println("done.");

	return pos; // return last used position in dmpUpdates: will be starting point for next call!
}

//***********************************************************//
// process only one line from dmpUpdates each time writeDMPUpdates() is called
unsigned int MPU6000_DMP::verifyDMPUpdates(unsigned int pos_verify, byte update_number)
{
	// pos_verify is the current verifying position within dmpUpdates
	byte check_byte, progBuffer, success;
	unsigned int j;
	// config set dmpUpdates is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	byte bank, offset, length;
	boolean verification = true;

	//Serial.print("\tVerifying DMP update "); Serial.print(update_number); Serial.print("/7 ..... ");

	bank   = pgm_read_byte(dmpUpdates + pos_verify++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
	offset = pgm_read_byte(dmpUpdates + pos_verify++);
	length = pgm_read_byte(dmpUpdates + pos_verify++);
	  
	//DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
	setMemoryBank(bank, false, false, m_chipSelect); // bank number  = bank
	//DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
	setMemoryStartAddress(offset, m_chipSelect);     // startaddress = offset from the beginning (0) of the bank
	//DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

	digitalWrite(m_chipSelect,LOW);
	SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

	for (j = 0; j < length; j++)
	{
		progBuffer = pgm_read_byte(dmpUpdates + pos_verify + j);
		check_byte = SPI.transfer(0x00);
		if (progBuffer != check_byte)
		{
			//DEBUG_PRINTLN("$$$ dmpUpdates: byte verification error");
			verification = false;
		}
		// #ifdef DEBUG
		//   if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
		//   Serial.println(check_byte, HEX);
		// #endif
	}
	   
	digitalWrite(m_chipSelect,HIGH);
	pos_verify = pos_verify + length;
	//DEBUG_PRINT("!! last position verified: "); DEBUG_PRINTLN(pos_verify);

	//if (verification == true)  Serial.println("success!");
	//if (verification == false) Serial.println("FAILED!");
	//return verification; // true if DMP correctly written, false if not

	return pos_verify; // return last used position in dmpUpdates: will be starting point for next call!
}

//***********************************************************//
/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (defined by register 35 and 36).
 * - return: Current FIFO buffer size
 */
unsigned int MPU6000_DMP::getFIFOCount(int ChipSelPin)
{
	// FIFO_COUNT should always be read in high-low order (0x72-0x73) in order to
	// guarantee that the most current FIFO Count value is read
	byte fifo_H = SPIread(0x72, m_chipSelect);
	byte fifo_L = SPIread(0x73, m_chipSelect);
	unsigned int two_bytes = (fifo_H << 8) | fifo_L;
	return two_bytes;
}

byte MPU6000_DMP::dmpInitialize()
{
	// Trigger a full device reset.
	// A small delay of ~50ms may be desirable after triggering a reset.
	
	//Serial.println("Resetting MPU6000...");
	SPIwriteBit(0x6B, 7, true, m_chipSelect); // DEVICE_RESET
	//digitalWrite(BLUE_LED_PIN, HIGH); // shows start of DMP inititialize
	delay(30 + 170); // wait after reset + time to see the LED blink

	// Setting the SLEEP bit in the register puts the device into very low power
	// sleep mode. In this mode, only the serial interface and internal registers
	// remain active, allowing for a very low standby current. Clearing this bit
	// puts the device back into normal mode. To save power, the individual standby
	// selections for each of the gyros should be used if any gyro axis is not used
	// by the application.
	// disable sleep mode
	//Serial.println("Disabling sleep mode...");
	SPIwriteBit(0x6B, 6, false, m_chipSelect); // SLEEP

	// get MPU hardware revision
	//Serial.println("Selecting user bank 16...");
	setMemoryBank(0x10, true, true, m_chipSelect);

	//Serial.println("Selecting memory byte 6...");
	setMemoryStartAddress(0x06, m_chipSelect);

	//Serial.println("Checking hardware revision...");
	digitalWrite(m_chipSelect,LOW);	
	SPI.transfer(0x6F | 0x80); 
	byte hwRevision = SPI.transfer(0x00);
	digitalWrite(m_chipSelect,HIGH);
	//Serial.println("Revision @ user[16][6] = ");
	//Serial.println(hwRevision, HEX);
	//Serial.println("Resetting memory bank selection to 0...");
	setMemoryBank(0, false, false, m_chipSelect);

	// check OTP bank valid
	//DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
	byte otpValid = SPIreadBit(0x00, 0, m_chipSelect);
	//DEBUG_PRINT(F("OTP bank is "));
	//DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

	// get X/Y/Z gyro offsets
	//DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
	byte xgOffsetTC = SPIreadBits(0x01, 6, 6, m_chipSelect); // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
	byte ygOffsetTC = SPIreadBits(0x02, 6, 6, m_chipSelect); // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
	byte zgOffsetTC = SPIreadBits(0x03, 6, 6, m_chipSelect); // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
	//DEBUG_PRINT("X gyro offset = ");
	//DEBUG_PRINTLN(xgOffsetTC);
	//DEBUG_PRINT("Y gyro offset = ");
	//DEBUG_PRINTLN(ygOffsetTC);
	//DEBUG_PRINT("Z gyro offset = ");
	//DEBUG_PRINTLN(zgOffsetTC);

	// load DMP code into memory banks
	//DEBUG_PRINT(F("########################### Writing DMP code to MPU memory banks ("));
	//DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
	//DEBUG_PRINTLN(F(" bytes)"));
	if (writeDMPMemory())
	{
		//DEBUG_PRINTLN(F("########################### Success! DMP code written but not verified."));

		//DEBUG_PRINTLN(F("########################### Verify DMP code..."));
		//Serial.println("verifyDMPMemory");
		verifyDMPMemory();
		    
		//digitalWrite(BLUE_LED_PIN, LOW); // shows end of write DMP memory
		//delay(200); // time to see the LED blink

		// write DMP configuration
		//DEBUG_PRINT(F("########################### Writing DMP configuration to MPU memory banks ("));
		//DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
		//DEBUG_PRINTLN(F(" bytes in config def)"));
		if (writeDMPConfig())
		{
			//DEBUG_PRINTLN(F("########################### Success! DMP configuration written but not verified."));

			//DEBUG_PRINTLN(F("########################### Verify DMP configuration..."));
			//Serial.println("Verify DMP configuration");
			verifyDMPConfig();
			    
			//digitalWrite(BLUE_LED_PIN, HIGH); // shows start of write DMP configuration
			//delay(200); // time to see the LED blink

			//DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
			SPIwriteBits(0x6B, 2, 3, 0x03, m_chipSelect); // CLKSEL[2:0] = 011 = PLL with Z axis gyroscope reference

			//DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
			SPIwrite(0x38, 0x12, m_chipSelect); // INT_ENABLE = 00010010 = FIFO_OFLOW_EN & DMP_INT_EN

			// register INT_ENABLE 0x38:
			// bit 0 DATA_RDY_EN      0x01
			// bit 1 DMP_INT_EN       0x02 (undocumented) - enabling this bit also enables DATA_RDY_EN it seems
			// bit 2 UNKNOWN_INT_EN   0x04 (undocumented)
			// bit 3 I2C_MST_INT_EN   0x08
			// bit 4 FIFO_OFLOW_EN    0x10
			// bit 5 ZMOT_EN          0x20 (undocumented)
			// bit 6 MOT_EN           0x40
			// bit 7 FF_EN            0x80 (undocumented)

			// register INT_STATUS 0x3A:
			// bit 0 DATA_RDY_INT     0x01
			// bit 1 DMP_INT          0x02 (undocumented)
			// bit 2 UNKNOWN_INT      0x04 (undocumented)
			// bit 3 I2C_MST_INT      0x08
			// bit 4 FIFO_OFLOW_INT   0x10
			// bit 5 ZMOT_INT         0x20 (undocumented)
			// bit 6 MOT_INT          0x40
			// bit 7 FF_INT           0x80 (undocumented)

			//DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
			//setRate(4); // 1kHz / (1 + 4) = 200 Hz (when DLPF_CFG enabled [1 to 6] - true, see below)
			SPIwrite(0x19, 4, m_chipSelect); // SMPLRT_DIV[7:0] = 4 (ok)

			// FSYNC input not connnected on ArduIMU+ V3
			//DEBUG_PRINTLN(F("Disable external frame synchronization..."));
			SPIwriteBits(0x1A, 5, 3, 0x00, m_chipSelect); // EXT_SYNC_SET[2:0] = 000 = input disabled

			//DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
			SPIwriteBits(0x1A, 2, 3, 0x03, m_chipSelect); // DLPF_CFG[2:0] = 011 = accel 44 Hz gyro 42 Hz

			//DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
			SPIwriteBits(0x1B, 4, 2, 0x03, m_chipSelect); // FS_SEL[1:0] = 11 = +/- 2000 deg/s

			//DEBUG_PRINTLN(F("Setting accelerometer full scale range to +/- 2 g..."));
			SPIwriteBits(0x1C, 4, 2, 0x00, m_chipSelect);

			//DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
			SPIwrite(0x70, 0x03, m_chipSelect); // DMP related register
			SPIwrite(0x71, 0x00, m_chipSelect); // DMP related register

			//DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
			SPIwriteBit(0x00, 0, false, m_chipSelect); // [0] OTP_BNK_VLD

			// enabling this part causes misalignment and drift of x, y and z axis
			// relative to ArduIMU+ V3/MPU-6000 x, y and z axis
			/*
			//DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
			SPIwriteBits(0x00, 6, 6, xgOffsetTC, m_chipSelect);
			SPIwriteBits(0x01, 6, 6, ygOffsetTC, m_chipSelect);
			SPIwriteBits(0x02, 6, 6, zgOffsetTC, m_chipSelect);
			*/

			/*
			//DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
			setXGyroOffset(0);
			setYGyroOffset(0);
			setZGyroOffset(0);
			*/

			//DEBUG_PRINTLN(F("###################### Writing final memory update 1/7 (function unknown)..."));
			byte update_number = 1;      // holds update number for user information
			unsigned int pos = 0;        // pos        is the current reading position within dmpUpdates; this is the first call; set pos        = 0 only once!
			pos = writeDMPUpdates(pos, update_number); 
			unsigned int pos_verify = 0; // pos_verify is the current reading position within dmpUpdates; this is the first call; set pos_verify = 0 only once!
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("Resetting FIFO..."));
			//SPIwriteBit(0x6A, 6, false, m_chipSelect); // FIFO_EN = 0 = disable
			SPIwriteBit(0x6A, 2, true, m_chipSelect); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
			//SPIwriteBit(0x6A, 6, true, m_chipSelect); // FIFO_EN = 1 = enable

			// Get current FIFO buffer size.
			// This value indicates the number of bytes stored in the FIFO buffer. This
			// number is in turn the number of bytes that can be read from the FIFO buffer
			// and it is directly proportional to the number of samples available given the
			// set of sensor data bound to be stored in the FIFO (register 35 and 36).
			//DEBUG_PRINTLN(F("Reading FIFO count..."));
			unsigned int fifoCount = getFIFOCount(m_chipSelect);

			// just after FIFO reset so count probably 0
			//DEBUG_PRINT(F("Current FIFO count = "));
			//DEBUG_PRINTLN(fifoCount);
			SPIreadBytes(0x74, fifoCount, m_fifoBuffer, m_chipSelect);

			//DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
			SPIwrite(0x1F, 2, m_chipSelect); // MOT_THR[7:0] = 2

			//DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
			SPIwrite(0x21, 156, m_chipSelect); // detection threshold for Zero Motion interrupt generation

			//DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
			SPIwrite(0x20, 80, m_chipSelect); // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms

			//DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
			SPIwrite(0x22, 0, m_chipSelect); // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms

			//DEBUG_PRINTLN(F("Resetting FIFO..."));
			//SPIwriteBit(0x6A, 6, false, m_chipSelect); // FIFO_EN = 0 = disable
			SPIwriteBit(0x6A, 2, true, m_chipSelect); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
			//SPIwriteBit(0x6A, 6, true, m_chipSelect); // FIFO_EN = 1 = enable

			//DEBUG_PRINTLN(F("Enabling FIFO..."));
			SPIwriteBit(0x6A, 6, true, m_chipSelect); // FIFO_EN = 1 = enable

			//DEBUG_PRINTLN(F("Enabling DMP..."));
			SPIwriteBit(0x6A, 7, true, m_chipSelect); // USER_CTRL_DMP_EN

			//DEBUG_PRINTLN(F("Resetting DMP..."));
			SPIwriteBit(0x6A, 3, true, m_chipSelect); // Reset DMP

			//DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			delay(50); // may be used as test just to see if number of FIFO bytes changes


			//DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
			//Serial.println("Waiting for FIFO count to increment");
			while ((fifoCount = getFIFOCount(m_chipSelect)) < 3);
			//Serial.println("Done");
			/* switched off, may crash the sketch (FIFO contents not used here anyway)
			// 1st read FIFO
			//DEBUG_PRINT(F("Current FIFO count = "));
			//DEBUG_PRINTLN(fifoCount);
			//DEBUG_PRINTLN(F("Reading FIFO data..."));
			//Serial.println("Reading FIFO data 1st time...");
			SPIreadBytes(0x74, fifoCount, fifoBuffer, m_chipSelect);
			*/

			//DEBUG_PRINTLN(F("Reading interrupt status..."));
			byte mpuIntStatus = SPIread(0x3A, m_chipSelect);

			//DEBUG_PRINT(F("Current interrupt status = "));
			//DEBUG_PRINTLNF(mpuIntStatus, HEX);

			// Jeff Rowberg's code had a read statement here... I suppose that must be a write statement!
			//DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
			//for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
			//readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
			//DEBUG_PRINTLN(F("Writing final memory update 6/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
			//Serial.println("Waiting for FIFO count to increment");
			while ((fifoCount = getFIFOCount(m_chipSelect)) < 3);
			//Serial.println("Done");

			//DEBUG_PRINT(F("Current FIFO count="));
			//DEBUG_PRINTLN(fifoCount);

			/* switched off, may crash the sketch (FIFO contents not used here anyway)
			// 2nd read FIFO
			//DEBUG_PRINTLN(F("Reading FIFO data..."));
			Serial.println("Reading FIFO data 2nd time...");
			SPIreadBytes(0x74, fifoCount, fifoBuffer, m_chipSelect);
			*/

			//DEBUG_PRINTLN(F("Reading interrupt status..."));
			mpuIntStatus = SPIread(0x3A, m_chipSelect);

			//DEBUG_PRINT(F("Current interrupt status="));
			//DEBUG_PRINTLNF(mpuIntStatus, HEX);

			//DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
			update_number ++;
			pos = writeDMPUpdates(pos, update_number); 
			pos_verify = verifyDMPUpdates(pos_verify, update_number);

			//DEBUG_PRINTLN(F("DMP is good to go! Finally."));

			//DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
			SPIwriteBit(0x6A, 7, false, m_chipSelect); // USER_CTRL_DMP_EN

			//DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
			//SPIwriteBit(0x6A, 6, false, m_chipSelect); // FIFO_EN = 0 = disable
			SPIwriteBit(0x6A, 2, true, m_chipSelect); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
			//SPIwriteBit(0x6A, 6, true, m_chipSelect); // FIFO_EN = 1 = enable
			SPIread(0x3A, m_chipSelect); // reading the register will clear all INT bits

			//Serial.println("Done dmp verified");

		}
		else
		{
			//DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
			return 2; // configuration block loading failed
		}

	}
	else
	{
		//DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
		return 1; // main binary block loading failed
	}

	//digitalWrite(BLUE_LED_PIN, LOW); // shows end of write DMP configuration
	//delay(200); // time to see the LED blink

	Serial.println("... Digital Motion Processor (DMP) initializing done.");
	Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	return 0; // success
}
