#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const char *names[] = {"+X","-X","+Y","-Y","+Z","-Z"};
const uint16_t AVSIZE		= 25;
const uint32_t UPDATE_DELAY	= 75;
const int Z_VAL = 550;
const int ZPVAL =  50;
const char startMessage[] = "\
Hello! \n\
To calibrate your APM2, you will need to hold the device \n\
very steadily on each axis. The progress indicator will \n\
show if the sensor is ligned up with an axis; just \n\
hold it on any axis long enough to get a good reading, \n\
and then move to the rest \n\n\
Ready to to start the calibration?";
int facingDir;
int accelMax[3][2];
int magVert[3][2];
int state = 0;

MpuSensor  mpu;
APMCompass cmp;
InertialSensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);

class datastream{
private:
	int average[AVSIZE];
	int* avloc;
	int der;
	int value;
	int goodSamples;
	void pushAverage(int v){
		*avloc = v;
		avloc++;
		if(avloc >= &average[AVSIZE]) avloc = &average[0];
	}
public:
	datastream(): avloc(&average[0]) {}
	void update(int v){
		der = v-value;
		value = v;
		pushAverage(value);
		if(abs(der)<ZPVAL) {
			goodSamples++;
		} else {
			goodSamples=0;
		}
	}
	boolean zero(){
		return (abs(value)<Z_VAL);
	}
	boolean stable(){
		return goodSamples >= AVSIZE;
	}
	int getAverage(){
		float sum;
		for(int i=0; i<AVSIZE; i++){
			sum += average[i];
		}
		int av = sum/((float)AVSIZE);
		return av;
	}
	int getValue(){
		return value;
	}
};
datastream data[3];
datastream  mag[3];

void setup(){
	Serial.begin(57600);
	sensors.start();
	delay(1000);
	sensors.calibrate(); //ummmmmmm
	//beginCompass();
}

void loop(){
	static uint32_t time = millis();
	if( millis() > time){
		time += UPDATE_DELAY;
		
		switch(state){
			case 0:
				Serial.println(startMessage);
				if(getTrueFalseResponse()){
					state++;
				}
				time = millis();
				break;
			case 1:
				updateSensorData();
				tuneAccelerometer();
				printAccelerometerStatus();
				if(accelTuned()) state++;
				break;
			case 2:
				printTuningData();
				Serial.println("would you like to store these values?");
				if (getTrueFalseResponse()) {
					//write to eeprom
				}
				time = millis();
				state = -1;
				break;
			case 3:
				updateSensorData();
				printTunedMag();
				//show magnetometer values
				break;
			default:
				for(;;);
		}		
		
		Serial.flush();
	}
}

void burnInput(){
	delay(10);
	while(Serial.available()){
		Serial.read();
		delay(1);
	}
}

boolean getTrueFalseResponse(){
	Serial.print("\nEnter response [y]es, [n]o: ");
	while(true){	
		if(Serial.available()){
			char input = Serial.read();
			burnInput(); //extra should be ignored
			Serial.println();
			switch(input){
				case 'y':
				case 'Y':
					return true;
				case 'n':
				case 'N':
					return false;
				default:
					Serial.print("\nInvalid input; try again [y]es, [n]o: ");
					break;
			}
		}
	}
}

void updateSensorData(){	
	sensors.update();
	
	float val[4];
	sensors.getLinAccel(val[0], val[1], val[2], val[3]);
	for(int i=0; i<3; i++){
		data[i].update(val[i]);
	}
	
	sensors.getMagField(val[0], val[1], val[2], val[3]);
	for(int i=0; i<3; i++){
		mag[i].update(val[i]);
	}
}

void tuneAccelerometer(){
	//grab good values		
	facingDir = -1;
	for(int i=0; i<3; i++){
		if (data[i].zero() && data[(i+1)%3].zero()) {
			int idx = (i+2)%3;
			facingDir = idx*2 + (data[idx].getValue() > 0);
			if(data[idx].stable()){
				int prospect = data[idx].getAverage();
				accelMax[idx][(prospect>0)] = prospect;
				magVert[idx][(prospect>0)] = mag[idx].getAverage();
			}
		}
	}
}

boolean accelTuned(){
	boolean done = true;
	for(int i=0; i<6; i++) done = done && (accelMax[i/2][i%2] != 0);
	return done;
}

void printAccelerometerStatus(){
	for(int i=0; i<6; i++){
		boolean facing = (facingDir==i);
		if(facing){
			Serial.print('*');
		} else {
			Serial.print(' ');
		}
		
		Serial.print(names[i]);
		Serial.print(" [");
		if( accelMax[i/2][i%2] !=0){
			Serial.print( (int) accelMax[i/2][i%2] );
		} else {
			Serial.print("nan");
		}
		Serial.print("]");
		
		if(facing){
			Serial.print('*');
		} else {
			Serial.print(' ');
		}
	}
	Serial.print("\n");
}

void printTunedMag(){
	float m[3];
	float length = 0;
	for(int i=0; i<3; i++){
		m[i]  = mag[i].getValue();
		m[i] -= (magVert[i][1]+magVert[i][0])/2.f;
		m[i] /= (magVert[i][1]-magVert[i][0])/2.f;
		length += m[i]*m[i];
	}
	length = sqrt(length);
	
	Serial.print("|");
	Serial.print(length);
	Serial.print("|\t");
	for(int i=0; i<3; i++){
		Serial.print(m[i]);
		Serial.print("\t");
	}

	Serial.print(atan2(m[0], m[1]));
	Serial.print("\t");

	Serial.print("\n");
}

void printTuningData(){
	Serial.print("Accelerometer Data:\n");
	for(int i=0; i<3; i++){
		Serial.print("\t");
		Serial.print(names[i*2]);
		Serial.print("\t");
		Serial.print(-(accelMax[i][1]+accelMax[i][0])/2.f);
		Serial.print("\t");
		Serial.print( (2.f)/((float)(accelMax[i][1]-accelMax[i][0])) ,DEC);
		Serial.print("\n");
	}
	Serial.print("Magnetometer Data:\n");
	for(int i=0; i<3; i++){
		Serial.print("\t");
		Serial.print(names[i*2]);
		Serial.print("\t");
		Serial.print(-(magVert[i][1]+magVert[i][0])/2.f);
		Serial.print("\t");
		Serial.print( (2.f)/((float)(magVert[i][1]-magVert[i][0])) ,DEC);
		Serial.print("\n");
	}
	Serial.print("\n");
}
