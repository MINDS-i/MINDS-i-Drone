#ifndef NMEA_H
#define NMEA_H
#include "Arduino.h"
#include "math/SpatialMath.h"
#include "math/Waypoint.h"
#include "math/floatgps.h"

class NMEA;
/**
 * Section buffer will be terminated with a '/0' and contain
 *   'sectionBufPos' characters, with said value pointing to the '/0'
 * Attempt to parse the buffer appropriatly, setting the appropriate field
 *   on success
 * return weather or not the parse succeeded
 */
typedef bool (*sectionHandler)(NMEA&);

struct rmc_data{
	float latitude;
	float longitude;
	float timeOfFix;
	float dateOfFix;
	bool warning;
	float groundSpeed;
	float course;
	float magVar;
};

struct gns_data{
	unsigned int numSV;
	float hdop;
};

class NMEA{
public:
	explicit NMEA(Stream& stream): inStream(stream) { stream.setTimeout(0); memset(&curGPSCoord,0,sizeof(GPS_COORD)); }
	/** Read more data from the input stream and parse whats available */
	void update();

	/** Verify that the checksum is correct */
	bool verifyChecksum(char msg[],uint8_t msgLen,uint8_t msgCheckSum);

	/** Start reading from a different input stream */
	void newStream(Stream& stream){
		inStream = stream;
		stream.setTimeout(0);
	}
	/** true if data has been updated since the last time anything was read */
	uint16_t dataIndex(){ return dataFrameIndex;	}

	//
	//NOTE: this are function that will return less percision
	//
	/** Latitude in decimal degrees, north is positive */
	float getLatitude(){ return gps_angle_to_float(&curGPSCoord.latitude);	}
	/** Longitude in decimal degrees, east is positive */
	float getLongitude(){ return gps_angle_to_float(&curGPSCoord.longitude);	}
	/** Time of GPS fix in HHMMSS format */

	/** Lat/long in GPS_COORD */
	GPS_COORD getGPS_COORD() { return curGPSCoord; }		

	float getTimeOfFix(){ return timeOfFix;	}
	/** Date of fix in DDMMYY format */
	float getDateOfFix(){ return dateOfFix;	}
	/** True if the location data may be missing or incorrect */
	bool getWarning(){ return warning;	}
	/** Get ground speed in miles per hours */
	float getGroundSpeed(){ return groundSpeed;	}
	/** Get course in degrees true (relative true north, clockwise positive) */
	float getCourse(){ return course;	}
	/** Angle between magnetic north and true north */
	float getMagVar(){ return magVar;	}

	/** Latitude/Longitude location as a Waypoint, CCW positive */
	Waypoint getLocation(){ return Waypoint(curGPSCoord);	}

	uint16_t getNumSat(){ return numSV; }
	float getHDOP(){ return hdop; }


	bool getUpdatedRMC()
	{
		return updatedRMC;
	}

	void clearUpdatedRMC()
	{
		updatedRMC=false;
	}
private:
	Stream& inStream;

	GPS_COORD curGPSCoord;

	float latitude = 0.0;
	float longitude = 0.0;
	float timeOfFix = 0, dateOfFix = 0;
	bool warning = true;
	float groundSpeed = 0;
	float course = 0;
	float magVar = 0;
	unsigned int numSV=0;
	float hdop = 0.0;
	uint16_t dataFrameIndex = 0;

	rmc_data rmc_msg;
	gns_data gns_msg;
	char msg[256];
	uint8_t msgLen;

	bool updatedRMC = false;

	//holds a section between commas in a GPRMC string
	char sectionBuf[16];
	int sectionBufPos = 0;
	int nmeaMsgType = -1;
	const int sizeSectionBuf = sizeof(sectionBuf)/sizeof(sectionBuf[0]);
	void clearBuffer(){ //here for inlining
		sectionBufPos = 0;
	}
	bool pushToBuffer(char c){ //here for inlining
		if(sectionBufPos >= sizeSectionBuf) return false;
		sectionBuf[sectionBufPos] = c;
		sectionBufPos++;
		return true;
	}
	/**
	 * Try and read a float off the sectionBuf, wich will be terminated by '\0'
     * on success, return true and update `store`
     * on failure, return false and leave `store` alone
	 */
	bool readFloat(float& store);
	//bool readGPSCoordFloat(GPS_COORD coord, uint8_t type );
	//same as readFloat but with unsigned int type
	bool readUInt(unsigned int& store);
	//holds the parsers sequence position in the $GPRMC string, -1 otherwise
	int seqPos = -1;
	//holds a temporary latitude or longitude value that has not been fully read
	float tmpLatLon;
	//an array of section handlers, coresponding to sections of a GPRMC string
	static const sectionHandler sectionHandlersGPRMC[];
	static const sectionHandler sectionHandlersGPGNS[];
	static const int numSectionsGPRMC;
	static const int numSectionsGPGNS;
	int numSections=0;

	bool handleSections();
};

#endif
