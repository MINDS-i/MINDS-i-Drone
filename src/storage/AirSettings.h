#ifndef AIR_SETTIGS_H
#define AIR_SETTIGS_H

#include "storage/Storage.h"
typedef float setting_t;

namespace AirSettings{
	//uses minutes and seconds of build time to generate a different
	//version (for the most part) every time the code is compiled
	static uint16_t VERSION =	 __TIME__[6]
								+__TIME__[7]*10
								+__TIME__[3]*100
								+__TIME__[4]*1000;
	//enum of established settings -- NUM_SETTINGS is 32
	enum Setting{
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
		UNUSED_A	= 17,
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
