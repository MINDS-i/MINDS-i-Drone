#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

const uint32_t UPDATE_INTERVAL = 5;
MPU6000   mpu;
HMC5883L  cmp;
InertialVec* sens[2] = {&mpu, &cmp};
Translator   conv[2] = {Translators::identity, Translators::identity};
InertialManager sensors(sens, conv, 2);

Settings set(eeStorage::getInstance());
float accl[3];
float gyro[3];
float magn[3];

void setup(){
	Serial.begin(115200);
	mpu.tuneAccl(set.getAccelTune());
	cmp.tune(set.getMagTune());

	sensors.start();
	delay(500);
	sensors.calibrate();
}

void display(float input){
	Serial.print(input);
	Serial.print("\t");
}

void loop(){
	static uint32_t time = millis();
	if (millis() > time) {
		time += UPDATE_INTERVAL;

		sensors.update();
		sensors.getLinAccel(accl[0], accl[1], accl[2]);
		sensors.getRotRates(gyro[0], gyro[1], gyro[2]);
		sensors.getMagField(magn[0], magn[1], magn[2]);

		display(time);
		for(int i=0; i<3; i++) display(accl[i]);
		for(int i=0; i<3; i++) display(gyro[i]*1000);
		for(int i=0; i<3; i++) display(magn[i]);

		Serial.print("\n");
	}
}
