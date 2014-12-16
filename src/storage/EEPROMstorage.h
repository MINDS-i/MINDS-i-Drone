#ifndef EEPROMSTORAGE_H
#define EEPROMSTORAGE_H
#include "storage/Storage.h"
#include "storage/EEPROMsubsystem.h"
/*
	The EEPROM storage singleton
	singleton instead of static to implement Storage
	also singleton given a guarenteed initialization
	keeps track of callbacks, stores reconds in EEPROM,
		should call callback immediatly upon attachment
*/

const uint8_t NUM_STORED_RECORDS = 32;
const EEaddr EEaddrStart = 1;
const EEaddr EEsanityLoc = eeprom::EE_MAX - 1;

class eeStorage : public Storage<float> {
private:
	static eeStorage* m_instance;
	eeStorage();
	void (*callback[NUM_STORED_RECORDS])(float);
public:
	static eeStorage* getInstance(){
		if(m_instance == NULL) m_instance = new eeStorage();
		return m_instance;
	}
	boolean checkEEPROMsanity();
	void attachCallback(uint8_t dataNum, void (*call)(float));
	void updateRecord(uint8_t dataNum, float value);
	float getRecord(uint8_t dataNum);
};
eeStorage* eeStorage::m_instance = NULL;

eeStorage::eeStorage(){
	eeprom::setup();
	for(int i=0; i<NUM_STORED_RECORDS; i++) callback[i] = NULL;
}
boolean
eeStorage::checkEEPROMsanity(){
	return (eeprom::safeRead(EEsanityLoc)==1);
}
void
eeStorage::attachCallback(uint8_t dataNum, void (*call)(float)){
	if(dataNum >= NUM_STORED_RECORDS) return;
	callback[dataNum] = call;
	callback[dataNum](getRecord(dataNum));
}
void
eeStorage::updateRecord(uint8_t dataNum, float value){
	if(dataNum >= NUM_STORED_RECORDS) return;
	eeprom::writeFloat(EEaddrStart+4*dataNum, value);
	if(callback[dataNum] != NULL) callback[dataNum](value);
}
float
eeStorage::getRecord(uint8_t dataNum){
	if(dataNum >= NUM_STORED_RECORDS) return 0.f;
	return eeprom::readFloat(EEaddrStart+4*dataNum);
}

#endif
