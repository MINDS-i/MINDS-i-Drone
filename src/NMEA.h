#ifndef NMEA_H
#define NMEA_H
#include "Arduino.h"
#include "GreatCircle.h"

class NMEA{
public:
	NMEA(Stream&);
	void newStream(Stream&);
	void update();
	bool newData();
	#ifdef GREATCIRCLE_H
	Point getLocation();
	#endif
	float getLatitude();
	float getLongitude();
	float getTimeOfFix();
	float getDateOfFix();
	bool getWarning();
	float getGroundSpeed(); //mph
	float getCourse();
	float getMagVar();
private:
	float latitude;
	float longitude;
	float timeOfFix, dateOfFix;
	bool warning;
	float groundSpeed;
	float course;
	float magVar;
	void parseLine(char*, int);
	void takeData(char*&, char*, int);
	float parseFloat(char*&, char*);
	bool overlap(char*, String);
	const int bufferSize;
	char buffer[200];
	int bufferPos;
	bool isNew;
	Stream& inStream;
};

#endif
