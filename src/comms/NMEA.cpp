#include "NMEA.h"

namespace{
	float inline toDecDegrees(float degreeDecimalMinutes){
	    float minutes = fmod(degreeDecimalMinutes,100.0);
	    float degrees = (degreeDecimalMinutes - minutes) / 100.0;
	    float decimalDegrees = degrees + (minutes/60.0);
	    return decimalDegrees;
	}

	float inline toMilesPerHours(float knots){
		return knots * 1.15077945;
	}
}

void NMEA::update(){
	while(inStream.available()){
		char n = inStream.read();

		// $ only appears at the beggining of a nmea string
		if(n == '$') {
			seqPos = 0;
			sectionBufPos = 0;
		} else if (seqPos != -1) {
			if(n != ','){
				sectionBuf[sectionBufPos] = n;
				sectionBufPos++;
				if(sectionBufPos >= sizeSectionBuf){
					seqPos = -1; //overflow, reset parser
				}
			} else {
				bool parseSuccess = handleSections();
				sectionBufPos = 0;
				seqPos = (parseSuccess)? seqPos+1 : -1; //reset parser on fail
				if(seqPos >= NumSections) {
					isNew = true;
					seqPos = -1; //done reading
				}
			}
		}
	}
}

bool NMEA::readFloat(float& store){
	char* endptr;
	float tmp = strtod(sectionBuf, &endptr);
	if(endptr == sectionBuf) return false;
	store = tmp;
	return true;
}

/*const SectionHandler NMEA::sectionHandlers[] {
	//GPRMC
	[](NMEA& nmea) -> bool { return strcmp("GPRMC", nmea.sectionBuf) == 0; },
	//TimeOfFix
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.timeOfFix); },
	//Status
	[](NMEA& nmea) -> bool {
		if(nmea.sectionBufPos != 1) return false;
		else if(nmea.sectionBuf[0] == 'A') nmea.warning = false;
		else if(nmea.sectionBuf[0] == 'V') nmea.warning = true;
		else return false;
		return true;
	},
	//Latitude
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.tmpLatLon); },
	//Latitude Hemisphere
	[](NMEA& nmea) -> bool {
		if(nmea.sectionBufPos != 1) return false;
		else if(nmea.sectionBuf[0] == 'N')
			nmea.latitude =  toDecDegrees(nmea.tmpLatLon);
		else if(nmea.sectionBuf[0] == 'S')
			nmea.latitude = -toDecDegrees(nmea.tmpLatLon);
		else return false;
		return true;
	},
	//Longitude
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.tmpLatLon); },
	//Longitude Hemisphere
	[](NMEA& nmea) -> bool {
		if(nmea.sectionBufPos != 1) return false;
		else if(nmea.sectionBuf[0] == 'E')
			nmea.longitude =  toDecDegrees(nmea.tmpLatLon);
		else if(nmea.sectionBuf[0] == 'W')
			nmea.longitude = -toDecDegrees(nmea.tmpLatLon);
		else return false;
		return true;
	},
	//Ground Speed
	[](NMEA& nmea) -> bool {
		bool success = nmea.readFloat(nmea.groundSpeed);
		if(success) nmea.groundSpeed = toMilesPerHours(nmea.groundSpeed);
		return success;
	},
	//Track Angle
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.course); },
	//Date
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.dateOfFix); },
	//Magnetic Variation
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.magVar); },
};
const int NMEA::NumSections =sizeof(sectionHandlers)/sizeof(sectionHandlers[0]);
*/
//const SectionHandler NMEA::sectionHandlers[] = {};


/*

switch:
Loaded 26942 .text at address 0x0
Loaded 2896 .data
benchmark 0 --------------------------> passed (33316 cycles)
benchmark 1 --------------------------> passed (32984 cycles)
benchmark 2 --------------------------> passed (31875 cycles)
benchmark 3 --------------------------> passed (35619 cycles)
benchmark 4 --------------------------> passed (33112 cycles)
benchmark 5 --------------------------> passed (663518 cycles)



lambdas:
Loaded 27076 .text at address 0x0
Loaded 2896 .data
benchmark 0 --------------------------> passed (33176 cycles)
benchmark 1 --------------------------> passed (32842 cycles)
benchmark 2 --------------------------> passed (31733 cycles)
benchmark 3 --------------------------> passed (35477 cycles)
benchmark 4 --------------------------> passed (32968 cycles)
benchmark 5 --------------------------> passed (663191 cycles)

switch - lambda
-134 (switch)
0 (tie)
140 (lambda)
142 (l)
142 (l)
142 (l)
144 (l)
327 (l)
*/
const int NMEA::NumSections = 11;

bool NMEA::handleSections(){
	sectionBuf[sectionBufPos] = '\0';
	//return sectionHandlers[seqPos](*this);
	switch(seqPos){
	case 0: //GPRMC
		return strcmp("GPRMC", sectionBuf) == 0;
	case 1: //TimeOfFix
		return readFloat(timeOfFix);
	case 2: //Status
		if(sectionBufPos != 1) return false;
		else if(sectionBuf[0] == 'A') warning = false;
		else if(sectionBuf[0] == 'V') warning = true;
		else return false;
		return true;
	case 3: //Latitude
		return readFloat(tmpLatLon);
	case 4: //Latitude Hemisphere
		if(sectionBufPos != 1) return false;
		else if(sectionBuf[0] == 'N') latitude =  toDecDegrees(tmpLatLon);
		else if(sectionBuf[0] == 'S') latitude = -toDecDegrees(tmpLatLon);
		else return false;
		return true;
	case 5: //Longitude
		return readFloat(tmpLatLon);
	case 6: //Longitude Hemisphere
		if(sectionBufPos != 1) return false;
		else if(sectionBuf[0] == 'E') longitude =  toDecDegrees(tmpLatLon);
		else if(sectionBuf[0] == 'W') longitude = -toDecDegrees(tmpLatLon);
		else return false;
		return true;
	case 7: //Ground Speed
		{
			bool success = readFloat(groundSpeed);
			if(success) groundSpeed = toMilesPerHours(groundSpeed);
			return success;
		}
	case 8: //Track Angle
		return readFloat(course);
	case 9: //Date
		return readFloat(dateOfFix);
	case 10: //Magnetic Variation
		return readFloat(magVar);
	}
	return false;
}

