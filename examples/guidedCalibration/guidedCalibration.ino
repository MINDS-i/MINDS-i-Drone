#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const char *names[] = {"-X","+X","-Y","+Y","-Z","+Z"};
const uint16_t AVSIZE		= 25;
const uint32_t UPDATE_DELAY	= 75;
const int Z_VAL = 550; //maximum accelerometer reading on "empty" axiz
const int ZPVAL =  75; //maximum derivative of value to be "stable"
const char startMessage[] = "\
Hello! \n\
To calibrate your APM2, you will need to hold the device \n\
very steadily on each axis. The progress indicator will \n\
show if the sensor is ligned up with an axis; just \n\
hold it on any axis long enough to get a good reading, \n\
and then move to the rest \n\n\
Tune now (yes) or skip straight to streaming sensor data (no)?";
int facingDir, isShaking;
float accelMax[3][2];
float magVert[3][2];
int state = 0;

/*
use Settings and EEStorage w/ LTATune to store the cooked values in EEPROM
*/

MpuSensor  mpu;
APMCompass cmp;
InertialSensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
Settings set(eeStorage::getInstance());

class datastream{
private:
	float average[AVSIZE];
	float* avloc;
	float der;
	float value;
	float goodSamples;
	void pushAverage(float v){
		*avloc = v;
		avloc++;
		if(avloc >= &average[AVSIZE]) avloc = &average[0];
	}
public:
	datastream(): avloc(&average[0]) {}
	void update(float v){
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
	float getAverage(){
		float sum;
		for(int i=0; i<AVSIZE; i++){
			sum += average[i];
		}
		float av = sum/((float)AVSIZE);
		return av;
	}
	float getValue(){
		return value;
	}
	float isShaking(){
		return (abs(der)<ZPVAL);
	}
};
datastream accl[3];
datastream  mag[3];

void setup(){
	Serial.begin(57600);
	sensors.start();
	delay(1000);
	sensors.calibrate();
}

void loop(){
	static uint32_t time = millis();
	if( millis() > time){
		time += UPDATE_DELAY;

		switch(state){
			case 0:
				Serial.println(startMessage);
				if(getTrueFalseResponse()){
					state = 1;
				} else {
					state = 3;
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
					writeTuningData();
					Serial.println("Done Writing");
				}
				time = millis();
				state++;
				break;
			case 3:
				Serial.println("The sensors tuned by EEPROM will now be streamed:");
				tuneAcclandMag();
				state++;
				break;
			case 4:
				updateSensorData();
				printRawVectors();
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

	float val[3];
	sensors.getLinAccel(val[0], val[1], val[2]);
	for(int i=0; i<3; i++){
		accl[i].update(val[i]);
	}

	sensors.getMagField(val[0], val[1], val[2]);
	for(int i=0; i<3; i++){
		mag[i].update(val[i]);
	}
}

void tuneAccelerometer(){
	//grab good values
	facingDir = -1;
	isShaking = -1;
	for(int i=0; i<3; i++){
		if (accl[i].zero() && accl[(i+1)%3].zero()) {
			int idx = (i+2)%3;
			facingDir = idx*2 + (accl[idx].getValue() > 0);
			isShaking = !(accl[idx].isShaking());
			if(accl[idx].stable()){
				float prospect = accl[idx].getAverage();
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
		char delim = ' ';
		if(facing && isShaking){
			delim = '!';
		} else if (facing){
			delim = '*';
		}
		Serial.print(delim);

		Serial.print(names[i]);
		Serial.print(" [");
		if( accelMax[i/2][i%2] !=0){
			Serial.print( accelMax[i/2][i%2] );
		} else {
			Serial.print( "nan");
		}
		Serial.print("]");

		Serial.print(delim);
	}
	Serial.print("\n");
}

void printRawVectors(){
	for(int i=0; i<3; i++){
		Serial.print(accl[i].getValue());
		Serial.print("\t");
	}
	for(int i=0; i<3; i++){
		Serial.print(mag[i].getValue());
		Serial.print("\t");
	}
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

void writeTuningData(){
	LTATune accl, mag;
	for(int i=0; i<3; i++){
		accl.values.shift[i]  = -(accelMax[i][1]+accelMax[i][0])/2.f;
		mag.values.shift[i]   = -( magVert[i][1]+ magVert[i][0])/2.f;
		accl.values.scalar[i] = (2.f)/((float)(accelMax[i][1]-accelMax[i][0]));
		mag.values.scalar[i]  = (2.f)/((float)( magVert[i][1]- magVert[i][0]));
	}
	set.writeTuningValues(accl, mag);
}

void tuneAcclandMag(){
	mpu.tuneAccl(set.getAccelTune());
	cmp.tune(set.getMagTune());
}
