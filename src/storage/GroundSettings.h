#ifndef GROUND_SETTINGS_H
#define GROUND_SETTINGS_H

#include "storage/Storage.h"
typedef float setting_t;

namespace AirSettings{
	//uses minutes and seconds of build time to generate a different
	//version (for the most part) every time the code is compiled
	static uint16_t VERSION =	 __TIME__[6]
								+__TIME__[7]*10
								+__TIME__[3]*100
								+__TIME__[4]*1000;
	//enum of established settings
	enum Setting{
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
		UNUSED_A	= 16,
		UNUSED_B	= 17,
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
		CALIB_CHECK = 30,
		STORAGE_VER	= 31
	};

	static uint8_t OTHER_INDEX = 0;
	static uint8_t getUniqueIndex(){
		return ((int)OTHER)+OTHER_INDEX++;
	}
	static Storage<setting_t> *storage = NULL;
	static bool validFormat = false;
	static void checkStorageFormat(){
		if(storage == NULL) return;
		validFormat = (storage->getRecord(STORAGE_VER) == VERSION);
		if (!validFormat) storage->updateRecord(STORAGE_VER, VERSION);
	}
	void useStorage(Storage<setting_t> *str){
		storage = str;
		checkStorageFormat();
	}
	bool attach(Setting type, setting_t def, void (*call)(setting_t)){
		if(storage == NULL) return false;
		uint8_t index = (type==OTHER) ? getUniqueIndex() : (int)type;
		
		if (!validFormat) {
			storage->updateRecord(index, def);
		}
		storage->attachCallback(index, call);
	}
}

#endif
