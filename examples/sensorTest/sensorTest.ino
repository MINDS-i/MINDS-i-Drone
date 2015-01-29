#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const uint32_t UPDATE_INTERVAL = 5;
MpuSensor  mpu;
APMCompass cmp;
InertialSensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
Settings set(eeStorage::getInstance());
float accl[4];
float gyro[4];
float magn[4];
		
void setup(){
	Serial.begin(115200);
	mpu.tuneAccl(set.getAccelTune());
	cmp.tune(set.getMagTune());
	
	sensors.start();
	delay(500);
	sensors.calibrate();
}

void display(long input){
	Serial.print(input);
	Serial.print("\t");
}

void loop(){
	static uint32_t time = millis();
	if (millis() > time) {
		time += UPDATE_INTERVAL;
		
		sensors.update();		
		sensors.getLinAccel(accl[0], accl[1], accl[2], accl[3]);
		sensors.getRotRates(gyro[0], gyro[1], gyro[2], gyro[3]);
		sensors.getMagField(magn[0], magn[1], magn[2], magn[3]);
		
		//Serial.print( ((long)time) - ((long)millis()) );
		Serial.print(time);
		Serial.print("\t");		
		for(int i=0; i<3; i++) display(accl[i]*10000.0f);
		for(int i=0; i<3; i++) display(gyro[i]*1000000.0f);
		for(int i=0; i<3; i++) display(magn[i]*10000.0f);
		Serial.print("\n");
	}
}
