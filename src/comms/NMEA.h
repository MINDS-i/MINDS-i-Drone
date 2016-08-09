#ifndef NMEA_H
#define NMEA_H
#include "Arduino.h"
#include "math/SpatialMath.h"
#include "math/Waypoint.h"

class NMEA;
/**
 * Section buffer will be terminated with a '/0' and contain
 *   'sectionBufPos' characters, with said value pointing to the '/0'
 * Attempt to parse the buffer appropriatly, setting the appropriate field
 *   on success
 * return weather or not the parse succeeded
 */
typedef bool (*SectionHandler)(NMEA&);

class NMEA{
public:
	explicit NMEA(Stream& stream): inStream(stream) { stream.setTimeout(0); }
	/** Read more data from the input stream and parse whats available */
	void update();
	/** Start reading from a different input stream */
	void newStream(Stream& stream){
		inStream = stream;
		stream.setTimeout(0);
	}
	/** true if data has been updated since the last time anything was read */
	uint16_t dataIndex(){
		return dataFrameIndex;
	}
	/** Latitude in decimal degrees, north is positive */
	float getLatitude(){
		return latitude;
	}
	/** Longitude in decimal degrees, east is positive */
	float getLongitude(){
		return longitude;
	}
	/** Time of GPS fix in HHMMSS format */
	float getTimeOfFix(){
		return timeOfFix;
	}
	/** Date of fix in DDMMYY format */
	float getDateOfFix(){
		return dateOfFix;
	}
	/** True if the location data may be missing or incorrect */
	bool getWarning(){
		return warning;
	}
	/** Get ground speed in miles per hours */
	float getGroundSpeed(){
		return groundSpeed;
	}
	/** Get course in degrees true (relative true north, clockwise positive) */
	float getCourse(){
		return course;
	}
	/** Angle between magnetic north and true north */
	float getMagVar(){
		return magVar;
	}
	/** Latitude/Longitude location as a Waypoint, CCW positive */
	Waypoint getLocation(){
		return Waypoint(latitude,longitude);
	}
private:
	Stream& inStream;
	bool isNew = false;
	float latitude = 0.0;
	float longitude = 0.0;
	float timeOfFix = 0, dateOfFix = 0;
	bool warning = true;
	float groundSpeed = 0;
	float course = 0;
	float magVar = 0;
	uint16_t dataFrameIndex = 0;
	//holds a section between commas in a GPRMC string
	char sectionBuf[16];
	int sectionBufPos = 0;
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
	//holds the parsers sequence position in the $GPRMC string, -1 otherwise
	int seqPos = -1;
	//holds a temporary latitude or longitude value that has not been fully read
	float tmpLatLon;
	//an array of section handlers, coresponding to sections of a GPRMC string
	static const SectionHandler sectionHandlers[];
	static const int NumSections;

	bool handleSections();
};

#endif
