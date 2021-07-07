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


enum NMEA_MSG_TYPE_ENUM 
{
	NMEA_MSG_TYPE_UNKNOWN = 0,
	NMEA_MSG_TYPE_GPRMC,
	NMEA_MSG_TYPE_GPGNS,
};

void NMEA::update(){
	while(inStream.available())
	{
		char n = inStream.read();

		// NMEA strings always begin with a '$', followed by comma sep. values
		// Attempt to parse any NMEA string, parsing each value as it comes
		// until the end of the string is reached or a value fails to parse

		//Serial.print(n);

		if(n == '$') 
		{
			//Serial.println(n);
			dataFrameIndex++;
			seqPos = 0;
			clearBuffer();
		} 
		else if (seqPos != -1) 
		{
			if((n != ',') && (n != '*'))
			{
				bool success = pushToBuffer(n);
				if(!success) seqPos = -1; //buffer full
			} 
			else 
			{
				bool parseSuccess=false;

				// two consective commas is not an error, just skip it
				if (sectionBufPos==0)
				{
					parseSuccess=true;
				}
				else
				{				
					parseSuccess = handleSections();
				}

				if (!parseSuccess)
					seqPos=-1;
				else
					seqPos+=1;


				clearBuffer();

				if(seqPos >= numSections) 
				{
					// dataFrameIndex++; could be here to only advance the
					// index when a full packed is completed, but the
					// position hold algorithm was tested with it marking
					// the beginnings of the packets so it should remain where
					// it is until enough testing of position hold can be done
					// to validate its placement here
					seqPos = -1; //done reading
					updatedRMC = true;
					nmeaMsgType = NMEA_MSG_TYPE_UNKNOWN;
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

bool NMEA::readUInt(unsigned int& store){
	char* endptr;
	unsigned int tmp = (unsigned int)strtoul(sectionBuf, &endptr,10);
	if(endptr == sectionBuf) return false;
	store = tmp;
	return true;
}

bool NMEA::handleSections()
{
	bool retval = 0;
	sectionBuf[sectionBufPos] = '\0';

	//The first seqPos is used to determine the nmeaMsgType
	if (seqPos == 0)
	{
		//first section handler should not modify anything in class just return true/false
		if (sectionHandlersGPRMC[0](*this))
		{
			nmeaMsgType = NMEA_MSG_TYPE_GPRMC;
			numSections = numSectionsGPRMC;
			retval = 1;
			//debug
			//Serial.println("Found gprmc");
		}
		else if (sectionHandlersGPGNS[0](*this))
		{
			nmeaMsgType = NMEA_MSG_TYPE_GPGNS;
			numSections = numSectionsGPGNS;
			retval = 1;
			//Serial.println("Found gpgns");
		}
		else
		{
			//invalid message
			nmeaMsgType = NMEA_MSG_TYPE_UNKNOWN;
			numSections = 0;
			retval = 0;
			//Serial.println("Found unhandled message");
		}
	}
	else
	{
		//Serial.println(seqPos);
		switch (nmeaMsgType)
		{
			case NMEA_MSG_TYPE_GPRMC:
				retval = sectionHandlersGPRMC[seqPos](*this);
				//if (!retval)
				//	Serial.println("===Error parsinng gprmc==");
			break;
			case NMEA_MSG_TYPE_GPGNS:
				retval = sectionHandlersGPGNS[seqPos](*this);
				//if (!retval)
				//	Serial.println("===Error parsinng gprmc==");				
			break;
			default:
				//Do nothing or return error?
				retval = 0;
			break;	
			//Serial.println("");		
		}

	}

	return retval;

}

/* using an array of lambdas:
	forces a consecutive ordering
	automatically determines number of sections
	is slightly faster than a switch statement
*/
const sectionHandler NMEA::sectionHandlersGPRMC[] {
	//GPRMC
	[](NMEA& nmea) -> bool {
		//Serial.println(nmea.sectionBuf);
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
	}
};


const sectionHandler NMEA::sectionHandlersGPGNS[] {
	//GPRMC
	[](NMEA& nmea) -> bool {
		return strcmp("GPGNS", nmea.sectionBuf) == 0
				|| strcmp("GNGNS", nmea.sectionBuf) == 0;
	},
	//utc time
	[](NMEA& nmea) -> bool { return 1; },
	//lat
	[](NMEA& nmea) -> bool { return 1; },
	//lat n/s
	[](NMEA& nmea) -> bool { return 1; },
	//long
	[](NMEA& nmea) -> bool { return 1; },	
	//long e/w
	[](NMEA& nmea) -> bool { return 1; },	
	//posMode (status for gps,glonass,galileo,Beidou)
	[](NMEA& nmea) -> bool { return 1; },
	//NumSV
	[](NMEA& nmea) -> bool {
		bool success = nmea.readUInt(nmea.numSV);		
		return success;
	},
	//HDOP
	[](NMEA& nmea) -> bool { return nmea.readFloat(nmea.hdop); },
	//alt
	[](NMEA& nmea) -> bool { return 1; },
	//sep
	[](NMEA& nmea) -> bool { return 1; },
	//diffAge
	[](NMEA& nmea) -> bool { return 1; },
	//difstation
	[](NMEA& nmea) -> bool { return 1; },
	//nav status
	[](NMEA& nmea) -> bool { return 1; },


};

const int NMEA::numSectionsGPRMC =sizeof(sectionHandlersGPRMC)/sizeof(sectionHandlersGPRMC[0]);
const int NMEA::numSectionsGPGNS =sizeof(sectionHandlersGPGNS)/sizeof(sectionHandlersGPGNS[0]);
