#ifndef EEPROMSTORAGE_H
#define EEPROMSTORAGE_H
#include "storage/EEPROMconfig.h"
#include "storage/EEPROMsubsystem.h"
#include "storage/Storage.h"
/*
    The EEPROM storage singleton
    singleton instead of static to implement Storage
    also singleton given a guarenteed initialization
    keeps track of callbacks, stores reconds in EEPROM,
        should call callback immediatly upon attachment
*/

class eeStorage : public Storage<EE_STORAGE_TYPE> {
  private:
    static eeStorage* m_instance;
    void (*callback[NUM_STORED_RECORDS])(EE_STORAGE_TYPE);
    eeStorage();

  public:
    static eeStorage* getInstance() {
        if (m_instance == NULL)
            m_instance = new eeStorage();
        return m_instance;
    }
    void attachCallback(uint8_t dataNum, void (*call)(EE_STORAGE_TYPE));
    void updateRecord(uint8_t dataNum, EE_STORAGE_TYPE value);
    EE_STORAGE_TYPE getRecord(uint8_t dataNum);
};
eeStorage* eeStorage::m_instance = NULL;

eeStorage::eeStorage() {
    eeprom::setup();
    for (int i = 0; i < NUM_STORED_RECORDS; i++)
        callback[i] = NULL;
}

void eeStorage::attachCallback(uint8_t dataNum, void (*call)(EE_STORAGE_TYPE)) {
    if (dataNum >= NUM_STORED_RECORDS)
        return;
    callback[dataNum] = call;
    if (call != NULL)
        call(getRecord(dataNum));
}

void eeStorage::updateRecord(uint8_t dataNum, EE_STORAGE_TYPE value) {
    if (dataNum >= NUM_STORED_RECORDS)
        return;
    eeprom::writeFloat(EEaddrStart + 4 * dataNum, value);
    if (callback[dataNum] != NULL)
        callback[dataNum](value);
}

EE_STORAGE_TYPE eeStorage::getRecord(uint8_t dataNum) {
    if (dataNum >= NUM_STORED_RECORDS)
        return 0.f;
    return eeprom::readFloat(EEaddrStart + 4 * dataNum);
}

#endif
