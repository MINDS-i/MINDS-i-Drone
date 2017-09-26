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

		// NMEA strings always begin with a '$', followed by comma sep. values
		// Attempt to parse any NMEA string, parsing each value as it comes
		// until the end of the string is reached or a value fails to parse

		if(n == '$') {
			dataFrameIndex++;
			seqPos = 0;
			clearBuffer();
		} else if (seqPos != -1) {
			if((n != ',') && (n != '*')){
				bool success = pushToBuffer(n);
				if(!success) seqPos = -1; //buffer full
			} else {
				// two consective commas is not an error, just skip it
				bool parseSuccess = (sectionBufPos==0)? true : handleSections();
				seqPos = (parseSuccess)? seqPos+1 : -1; //reset parser on fail
				clearBuffer();
				if(seqPos >= NumSections) {
					// dataFrameIndex++; could be here to only advance the
					// index when a full packed is completed, but the
					// position hold algorithm was tested with it marking
					// the beginnings of the packets so it should remain where
					// it is until enough testing of position hold can be done
					// to validate its placement here
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


bool NMEA::handleSections(){
	sectionBuf[sectionBufPos] = '\0';
	return sectionHandlers[seqPos](*this);
}

/* using an array of lambdas:
	forces a consecutive ordering
	automatically determines number of sections
	is slightly faster than a switch statement
*/
const SectionHandler NMEA::sectionHandlers[] {
	//GPRMC
	[](NMEA& nmea) -> bool {
		return strcmp("GPRMC", nmea.sectionBuf) == 0
				|| strcmp("GNRMC", nmea.sectionBuf) == 0;
	},
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
	//magnetic Variation direction
	[](NMEA& nmea) -> bool {
		if(nmea.sectionBuf[0] == 'E') nmea.magVar *= -1;
		return true;
	},
};
const int NMEA::NumSections =sizeof(sectionHandlers)/sizeof(sectionHandlers[0]);
