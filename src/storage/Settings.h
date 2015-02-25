#ifndef AIR_SETTIGS_H
#define AIR_SETTIGS_H

#include "storage/Storage.h"
#include "util/LTATune.h"

/*
These enumerations fill up all 32 settings slots (some are named unused_[alpha])
To make more settings, one would need to stretch the default number of storage
records in the passed in storage object
*/
typedef float setting_t;
namespace AirSettings{	
	enum Air{
		INT_PERIOD	=  0,
		OUT_FEEDBK	=  1,
		GYRO_MSE	=  2,
		ACCL_MSE	=  3,
		RATE_SYSMSE	=  4,
		ATT_SYSMSE	=  5,
		ATT_P_TERM	=  6,
		ATT_I_TERM	=  7,
		ATT_D_TERM	=  8,
		UNUSED_I	=  9,
		UNUSED_H	= 10,
		UNUSED_G	= 11,
		UNUSED_F	= 12,
		UNUSED_E	= 13,
		UNUSED_D	= 14,
		UNUSED_C	= 15,
		UNUSED_B	= 16,
		UNUSED_A	= 17
	};
}
namespace groundSettings{	
	enum Ground{
		LINE_GRAV	=  0,
		STEER_THROW	=  1,
		STEER_STYLE	=  2,
		STEER_FAC	=  3,
		MIN_FWD_SPD	=  4,
		MAX_FWD_SPD	=  5,
		REV_STR_THR	=  6,
		MAX_REV_SPD	=  7,
		PING_WEIGHT	=  8,
		COAST_TIME	=  9,
		MIN_REV_T	= 10,
		CRUISE_P	= 11,
		CRUISE_I	= 12,
		CRUISE_D	= 13,
		TIRE_DIAM	= 14,
		STR_CENTER	= 15,
		UNUSED_B	= 16,
		UNUSED_A	= 17
	};
}
namespace commonSettings{
	static const uint16_t VERSION =	 __TIME__[6]
									+__TIME__[7]*10
									+__TIME__[3]*100
									+__TIME__[4]*1000;
	static const uint16_t CALIBRATION_VERSON = 7;
	enum Common{
		ACCL_X_SHFT	= 18,
		ACCL_Y_SHFT	= 19,
		ACCL_Z_SHFT	= 20,
		ACCL_X_SCLR	= 21,
		ACCL_Y_SCLR	= 22,
		ACCL_Z_SCLR	= 23,
		MAG_X_SHFT	= 24,
		MAG_Y_SHFT	= 25,
		MAG_Z_SHFT	= 26,
		MAG_X_SCLR	= 27,
		MAG_Y_SCLR	= 28,
		MAG_Z_SCLR	= 29,
		CALIB_VER	= 30,
		STORAGE_VER	= 31
	};
}
using namespace commonSettings;
class Settings{ 
	// makes the usage of Storage for settings easier to top level sketches
	// attach is a single action
	// keeps track of weather or not storage was initialized
	// structured and safe retreival of 2 LTATunes from memory
private:
	Storage<setting_t> *storage = NULL;
	bool formatChecked = false;
	bool validFormat = false;
	bool validCalib  = false;
public:
	Settings(Storage<setting_t> *str) : storage(str) {
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
	bool attach(int type, setting_t defaul, void (*call)(setting_t)){
		if(!formatChecked) checkStorageFormat();
		if(storage == NULL) return false;
		uint8_t index = (int)type;
		
		if (!validFormat) {
			storage->updateRecord(index, defaul);
		}
		storage->attachCallback(index, call);
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
