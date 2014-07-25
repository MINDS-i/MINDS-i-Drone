#include <Wire.h>
#include "Compass.h"
#include "Arduino.h"

const int address = 0x1E;

void beginCompass(){
	Wire.begin();

	Wire.beginTransmission(address);
	Wire.write((uint8_t) 0x00);
	Wire.write((uint8_t) 0x70);
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.write((uint8_t) 0x01);
	Wire.write((uint8_t) 0x00);
	Wire.endTransmission();
}

void rawCompass(int*x, int*y, int*z){
	Wire.beginTransmission(address);
	Wire.write((uint8_t)0x02);
	Wire.write((uint8_t)0x01);
	Wire.endTransmission();


	Wire.requestFrom(address, 0x06);
	if(Wire.available()>=6){
		*x = ((long) Wire.read() )<<8; //X msb
		*x |= ((long) Wire.read() ); //X lsb
		*z = ((long) Wire.read() )<<8; //Z msb
		*z |= ((long) Wire.read() ); //Z lsb
		*y = ((long) Wire.read() )<<8; //Y msb
		*y |= ((long) Wire.read() );
	}
}

float getHeading(){
	return getHeading(0,1,0,1);
}

float getHeading(int xshift, int xscalar, int yshift, int yscalar){
	float adjX, adjY;
	float azimuth;
	int x,y,z;
	rawCompass(&x,&y,&z);
	adjX = float(x+xshift)/float(xscalar);
	adjY = float(y+yshift)/float(yscalar);

	azimuth  = atan2(adjY, adjX);
	azimuth *= 57.3578; // (180/pi) - converting to degrees

	return azimuth;
}

float getHeadingTiltComp(int xshift, int xscalar,
						 int yshift, int yscalar,
						 double pitch, double roll) {
	static float maxVec=1; //used to approximate Z axis
	float adjX, adjY, adjZ;
	float finalX, finalY;
	float azimuth;
	int x,y,z;
	rawCompass(&x,&y,&z);
	adjX = float(x+xshift)/float(xscalar);
	adjY = float(y+yshift)/float(yscalar);

	//when the vector lies 100% in X and Y we will find the magnitude we need
	if(maxVec < adjX*adjX+adjY*adjY) maxVec = adjX*adjX+adjY*adjY;
	adjZ =  sqrt( maxVec-(adjX*adjX)-(adjY*adjY) );

	finalX = adjX*cos(-pitch)							 + adjZ*sin(-pitch);
	finalY = adjX*sin(roll)*sin(-pitch)	+ adjY*cos(roll) - adjZ*sin(roll)*cos(-pitch);

	azimuth  = atan2(finalY, finalX);
	azimuth *= 57.3578;

	return azimuth;
}
