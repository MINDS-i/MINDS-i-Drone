#ifndef AIR_SETTIGS_H
#define AIR_SETTIGS_H

#include "storage/EEPROMconfig.h"
#include "storage/Storage.h"
#include "util/LTATune.h"

namespace commonSettings{
	static const uint16_t VERSION =	 __TIME__[7]
									+__TIME__[6]*10
									+__TIME__[4]*100
									+uint16_t(__TIME__[3])*1000;
	static const uint16_t CALIBRATION_VERSON = 8;
	enum Common{
		ACCL_X_SHFT	= 50,
		ACCL_Y_SHFT	= 51,
		ACCL_Z_SHFT	= 52,
		ACCL_X_SCLR	= 53,
		ACCL_Y_SCLR	= 54,
		ACCL_Z_SCLR	= 55,
		MAG_X_SHFT	= 56,
		MAG_Y_SHFT	= 57,
		MAG_Z_SHFT	= 58,
		MAG_X_SCLR	= 59,
		MAG_Y_SCLR	= 60,
		MAG_Z_SCLR	= 61,
		CALIB_VER	= 62,
		STORAGE_VER	= 63
	};
}
using namespace commonSettings;
class Settings{
	// makes the usage of Storage for settings easier to top level sketches
	// attach is a single action
	// keeps track of weather or not storage was initialized
	// structured and safe retreival of 2 LTATunes from memory
private:
	Storage<EE_STORAGE_TYPE> *storage = NULL;
	bool formatChecked = false;
	bool validFormat = false;
	bool validCalib  = false;
public:
	Settings(Storage<EE_STORAGE_TYPE> *str) : storage(str) {
	}
	bool foundIMUTune(){
		if (!formatChecked) checkStorageFormat();
		return validCalib;
	}
	bool foundSettings(){
		if (!formatChecked) checkStorageFormat();
		return validFormat;
	}
	void checkStorageFormat(){
		if(storage == NULL) return;
		validCalib  = (storage->getRecord(CALIB_VER  ) == CALIBRATION_VERSON);
		validFormat = (storage->getRecord(STORAGE_VER) == VERSION);
		if (!validFormat) storage->updateRecord(STORAGE_VER, VERSION);
		formatChecked = true;
	}
	void writeCalibrationVersion(){
		if(storage == NULL) return;
		storage->updateRecord(CALIB_VER, CALIBRATION_VERSON);
	}
	void writeTuningValues(LTATune accel, LTATune mag){
		if(storage == NULL) return;
		for(int i=0; i<6; i++){
			storage->updateRecord(ACCL_X_SHFT+i, accel.raw[i]);
			storage->updateRecord( MAG_X_SHFT+i,   mag.raw[i]);
		}
		writeCalibrationVersion();
	}
	bool attach(int type, EE_STORAGE_TYPE defaul, void (*call)(EE_STORAGE_TYPE)){
		if(!formatChecked) checkStorageFormat();
		if(storage == NULL) return false;
		uint8_t index = (int)type;

		if (!validFormat) {
			storage->updateRecord(index, defaul);
		}
		storage->attachCallback(index, call);
		return true;
	}
	bool attach(int type, EE_STORAGE_TYPE min, EE_STORAGE_TYPE max, EE_STORAGE_TYPE defaul, void (*call)(EE_STORAGE_TYPE)){
		if(storage == NULL) return false;
		uint8_t index = (int)type;

		EE_STORAGE_TYPE value  = storage->getRecord(index);

		if (value < min || value > max) {
			storage->updateRecord(index, defaul);
		}
		storage->attachCallback(index, call);
		return true;
	}
	float get(int type){
		uint8_t index = (int)type;
		return storage->getRecord(index);
	}
	LTATune getTuneAt(int startIndex){
		if(!formatChecked) checkStorageFormat();
		if(storage == NULL) return LTATune();
		LTATune output;
		if(!validCalib) return output;
		for(int i=0; i<6; i++){
			output.raw[i] = storage->getRecord(startIndex+i);
		}
		return output;
	}
	LTATune getAccelTune(){
		if(!formatChecked) checkStorageFormat();
		return getTuneAt(ACCL_X_SHFT);
	}
	LTATune getMagTune(){
		if(!formatChecked) checkStorageFormat();
		return getTuneAt( MAG_X_SHFT);
	}
};

#endif
