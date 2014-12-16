#ifndef EEPROMSUBSYSTEM
#define EEPROMSUBSYSTEM
#include "Arduino.h"
#include "avr/io.h"

#include "storage/queue.h"
#include "util/byteConv.h"

typedef uint16_t EEaddr;

struct eepromWrite{
	EEaddr  addr;
	uint8_t data;
	eepromWrite(EEaddr address, uint8_t toBeWrit):
		addr(address), data(toBeWrit) {}
	eepromWrite(): addr(0), data(0) {}
};

const  uint8_t QUEUE_SIZE = 32;
static eepromWrite QUEUE_DATA[QUEUE_SIZE];
static SimpleQueue<eepromWrite> _eepromWriteQueue(QUEUE_DATA, QUEUE_SIZE);

class eeprom{
public:
	static const EEaddr EENULL = 0;
	static const EEaddr EE_MAX = 1024;
	static void setup();
	static void write(EEaddr addr, uint8_t val); //attempts to add to buffer
	static void safeWrite(EEaddr addr, uint8_t val); //blocks till buff has room
	static void writeLong(EEaddr addr, uint32_t val); //these are safe writes
	static void writeFloat(EEaddr addr, float val);
	static uint8_t  read(EEaddr addr); 	  //attemps to read, skipping if writing
	static uint8_t  safeRead(EEaddr addr);//reads eeprom, blocking if necessary
	static uint32_t readLong(EEaddr addr);
	static float    readFloat(EEaddr addr);
	static void disableInterrupt();
private:
	static void enableInterrupt();
	static boolean safeToRead();
	eeprom(){}
};
void
eeprom::setup(){
	bitClear(EECR, EEPM0);
	bitClear(EECR, EEPM1);
}
void
eeprom::disableInterrupt(){
	bitClear(EECR, EERIE);
}
void
eeprom::enableInterrupt(){
	bitSet(EECR, EERIE);
}
void
eeprom::write(EEaddr addr, uint8_t val){
	if(addr == eeprom::EENULL || addr >= eeprom::EE_MAX) return;
	_eepromWriteQueue.push(eepromWrite(addr,val));
	eeprom::enableInterrupt();
}
void
eeprom::safeWrite(EEaddr addr, uint8_t val){
	if(addr == eeprom::EENULL || addr >= eeprom::EE_MAX) return;
	while( !_eepromWriteQueue.push(eepromWrite(addr,val)) );
	eeprom::enableInterrupt();
}
void
eeprom::writeLong(EEaddr addr, uint32_t val){
	byteConv data;
	data.l = val;
	for(int i=0; i<4; i++) safeWrite(addr+i, data.bytes[i]);
}
void
eeprom::writeFloat(EEaddr addr, float val){
	byteConv data;
	data.f = val;
	for(int i=0; i<4; i++) safeWrite(addr+i, data.bytes[i]);
}
boolean
eeprom::safeToRead(){
	return ( _eepromWriteQueue.isEmpty() && !bitRead(EECR, EEPE) );
}
uint8_t
eeprom::read(EEaddr addr){
	if(addr == eeprom::EENULL || addr >= eeprom::EE_MAX) return 0;
	if( !eeprom::safeToRead() ) return 0; //don't block, just exit
	EEAR = addr;
	bitSet(EECR, EERE); //start read; takes 4 cycles
	return EEDR;
}
uint8_t
eeprom::safeRead(EEaddr addr){
	if(addr == eeprom::EENULL || addr >= eeprom::EE_MAX) return 0;
	while( !eeprom::safeToRead() ); //wait for chance to Read
	EEAR = addr;
	bitSet(EECR, EERE); //start read; takes 4 cycles
	return EEDR;
}
uint32_t
eeprom::readLong(EEaddr addr){
	byteConv data;
	for(int i=0; i<4; i++) data.bytes[i] = safeRead(addr+i);
	return data.l;
}
float
eeprom::readFloat(EEaddr addr){
	byteConv data;
	for(int i=0; i<4; i++) data.bytes[i] = safeRead(addr+i);
	return data.f;
}
ISR(EE_READY_vect){
	//cycle queue through writing process and write
	if(_eepromWriteQueue.isEmpty()){
		eeprom::disableInterrupt();
		return;
	}
	eepromWrite write = _eepromWriteQueue.pop();
	uint8_t oldSREG = SREG;
	cli();
	EEAR = write.addr;
	EEDR = write.data;
	EECR = (EECR & 0xf8)|0x04; //write 1 to EEMPE and 0 to EEPE
	bitSet(EECR, EEPE);
	SREG = oldSREG;
}

#endif
