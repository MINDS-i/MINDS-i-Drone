#include "NMEA.h"
NMEA::NMEA(Stream& stream):
	inStream(stream),
	bufferPos(0),
	bufferSize(100),
	latitude(0),
	longitude(0),
	timeOfFix(0), dateOfFix(0),
	warning(0),
	groundSpeed(0),
	course(0),
	magVar(0),
	isNew(false) {
}

void NMEA::newStream(Stream& stream){
	inStream = stream;
}

void NMEA::update(){
	while(inStream.available()){
		char tmp = inStream.read();
		if(tmp != '$') {
			buffer[bufferPos] = tmp;
			bufferPos++;
			if(bufferPos >= bufferSize) bufferPos = 0;
		} else {
			parseLine(buffer, bufferPos);
			isNew = true;
			bufferPos = 0;
		}
	}
}

bool NMEA::newData(){
	return isNew;
}

float NMEA::getLatitude(){
	isNew = false;
	return latitude;
}

float NMEA::getLongitude(){
	isNew = false;
	return longitude;
}

float NMEA::getTimeOfFix(){
	isNew = false;
	return timeOfFix;
}

float NMEA::getDateOfFix(){
	isNew = false;
	return dateOfFix;
}

bool NMEA::getWarning(){
	isNew = false;
	return warning;
}

float NMEA::getGroundSpeed(){
	isNew = false;
	return groundSpeed;
}

float NMEA::getCourse(){
	isNew = false;
	return course;
}

float NMEA::getMagVar(){
	isNew = false;
	return magVar;
}

void NMEA::parseLine(char* buffer, int endPos){
	char* endPtr = buffer+endPos;
	if( overlap(buffer,"GPRMC") ){
		buffer += 5;
		for(int i=0; i<11; i++){
			if(buffer!=endPtr && *buffer == ','){
				buffer++;
				takeData(buffer, endPtr, i);
			} else {
				return;
			}
		}
	}
}

void NMEA::takeData(char*& buffer, char* endPtr, int type){
	switch(type){
		case 0: //system time
			timeOfFix = parseFloat(buffer, endPtr);
			break;
		case 1: //warning
			if(*buffer == 'A') warning = false;
			else if (*buffer == 'V') warning = true;
			buffer++;
			break;
		case 2: //latitude digits
			{
				float tmp = parseFloat(buffer, endPtr);
				latitude  = fmod(tmp, 100);
				latitude  = ((tmp-latitude)/100) + (latitude/60.f);
			} break;
		case 3: //latitude direction
			if(*buffer == 'N') buffer++;
			else if(*buffer == 'S') {
				buffer++;
				latitude *= -1;
			} else {
				//Error
			}
			break;
		case 4: //longitude digits
			{
				float tmp = parseFloat(buffer, endPtr);
				longitude  = fmod(tmp, 100);
				longitude  = ((tmp-longitude)/100) + (longitude/60.f);
			} break;
		case 5: //longitude direction
			if(*buffer == 'E') buffer++;
			else if(*buffer == 'W') {
				buffer++;
				longitude *= -1;
			} else{
				//Error
			}
			break;
		case 6: //speed over ground
			groundSpeed = parseFloat(buffer, endPtr);
			groundSpeed *= 1.15077945l; //convert to MPH
			break;
		case 7: //Course made true
			course = parseFloat(buffer, endPtr);
			break;
		case 8: //date of fix
			dateOfFix = parseFloat(buffer, endPtr);
			break;
		case 9: //magnetic variation
			magVar = parseFloat(buffer, endPtr);
			break;
		case 10: //checksum
			break;
		default: //Error
			break;
	}
}

float NMEA::parseFloat(char*& buffer, char* endPtr){
	float out=0;
	float decPlcs=0;
	bool isNegative = false;
	if(*buffer == '-'){
		isNegative = true;
		buffer++;
	}

	while(buffer != endPtr && *buffer != ','){
		switch(*buffer){
			case '.':
				decPlcs = 1;
				break;
			case '0':
				if(decPlcs != 0) decPlcs*=10;
				else out*=10;
				break;
			case '1':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (1./decPlcs);
				} else {
					out *= 10;
					out += 1;
				}
				break;
			case '2':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (2./decPlcs);
				} else {
					out *= 10;
					out += 2;
				}
				break;
			case '3':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (3./decPlcs);
				} else {
					out *= 10;
					out += 3;
				}
				break;
			case '4':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (4./decPlcs);
				} else {
					out *= 10;
					out += 4;
				}
				break;
			case '5':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (5./decPlcs);
				} else {
					out *= 10;
					out += 5;
				}
				break;
			case '6':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (6./decPlcs);
				} else {
					out *= 10;
					out += 6;
				}
				break;
			case '7':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (7./decPlcs);
				} else {
					out *= 10;
					out += 7;
				}
				break;
			case '8':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (8./decPlcs);
				} else {
					out *= 10;
					out += 8;
				}
				break;
			case '9':
				if(decPlcs != 0){
					decPlcs*=10;
					out += (9./decPlcs);
				} else {
					out *= 10;
					out += 9;
				}
				break;
		}
		buffer++;
	}

	if(isNegative) out*=-1;
	return out;
}

bool NMEA::overlap(char* buffer, String input){
	bool tmp = true;
	for (int i = 0; i < input.length(); ++i){
		tmp &= (buffer[i] == input[i]);
	}
	return tmp;
}

Point NMEA::getLocation(){
	isNew = false;
	return Point(latitude,longitude);
}

