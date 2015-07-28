#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

enum ProgramState {
    COLLECT_STATES,
    CALC_RESULTS,
    TUNE_AND_PRINT,
    STREAM_DATA
} state;

const char *names[] = {"-X","+X","-Y","+Y","-Z","+Z"};
const uint16_t AVSIZE       = 25;
const uint32_t UPDATE_DELAY = 75;
const int Z_VAL = 2500; //maximum accelerometer reading on "empty" axiz
const int ZPVAL =  100; //maximum derivative of value to be "stable"
const char startMessage[] = "\
Hello! \n\
To calibrate your APM2, you will need to hold the device \n\
very steadily on each axis. The progress indicator will \n\
show if the sensor is ligned up with an axis; just \n\
hold it on any axis long enough to get a good reading, \n\
and then move to the rest \n\n\
Tune now (yes) or skip straight to streaming sensor data (no)?";
int facingDir, isShaking;
uint8_t axisLogCount = 0;
bool  axisLogged[6];
float acclLog[6][3];
float magnLog[6][3];

MPU6000  mpu;
HMC5883L cmp;
Sensor* sens[2] = {&mpu, &cmp};
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
        float sum = 0.0f;
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
datastream magn[3];

void setup(){
    Serial.begin(9600);
    sensors.start();
    delay(1000);
    sensors.calibrate();
    delay(250);
    if(mpu.status() == STATUS_BAD){
        Serial.println("Bad MPU6000 status");
        while(true);
    }
    if(cmp.status() == STATUS_BAD){
        Serial.println("Bad HMC5883L status");
        while(true);
    }

    Serial.println(startMessage);
    state = (getTrueFalseResponse())? COLLECT_STATES : TUNE_AND_PRINT;
}

void loop(){
    static uint32_t time = millis();
    if( millis() > time){
        time += UPDATE_DELAY;
        updateSensorData();

        switch(state){
            case COLLECT_STATES:
                //returns true when all necessary states have been visited
                if(collectStates()) state = CALC_RESULTS;
                printCollectionStatus();
                break;
            case CALC_RESULTS:
                calculateResults();
                axisLogCount = 0;
                state = TUNE_AND_PRINT;
                break;
            case TUNE_AND_PRINT:
                applyTuneFromEEPROM();
                state = STREAM_DATA;
                break;
            case STREAM_DATA:
                streamData();
                break;
            default:
                while(1);//stop
        }

        Serial.flush();
    }
}

bool collectStates(){
    facingDir = -1;
    isShaking = -1;
    for(int i=0; i<3; i++){
        //if two axis are zero'd
        if (accl[i].zero() && accl[(i+1)%3].zero()) {
            int idx   = (i+2)%3; //pick remaining axis
            facingDir = idx*2 + (accl[idx].getValue() > 0);
            isShaking = !(accl[idx].isShaking());
            if(accl[idx].stable()) logData(facingDir);
        }
    }
    return axisLogCount == 0b00111111; //all axis have been counted
}

void printCollectionStatus(){
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
        Serial.print(": ");
        Serial.print((axisLogged[i])?"ok":"NA");
        Serial.print(delim);
    }
    Serial.print("\n");
}

void logData(int dir){
    axisLogCount |= (1<<dir);
    axisLogged[dir] = true;
    for(int i=0; i<3; i++){
        acclLog[dir][i] = accl[i].getAverage();
        magnLog[dir][i] = magn[i].getAverage();
    }
}

void printTune(LTATune tune){
    Serial.println("Shift x,y,z: ");
    for(int i=0; i<3; i++){
        Serial.print(tune.shift[i],0);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("Scalar x,y,z: ");
    for(int i=0; i<3; i++){
        Serial.print(tune.scalar[i],7);
        Serial.print("\t");
    }
    Serial.println();
}

void calculateResults(){
    LTATune newAccl = LTATune::FitEllipsoid(acclLog);
    LTATune newMagn = LTATune::FitEllipsoid(magnLog);

//{"-X","+X","-Y","+Y","-Z","+Z"}


    Serial.println();
    for(int i=0; i<3; i++){
        float a = magnLog[i*2+0][i];
        float b = magnLog[i*2+1][i];
        newMagn.calibrate(a, i);
        newMagn.calibrate(b, i);
        a *= -1;
        if(a < 0 && b < 0){
            Serial.print("Mag axis ");
            Serial.print(i);
            Serial.println(" is inverted");
            newMagn.scalar[i] *= -1;
        }
    }
/*
    Serial.println();
    for(int j=0; j<6; j++){
        float data[3];
        for(int i=0; i<3; i++) data[i] = magnLog[j][i];
        newMagn.calibrate(data);
        float val = data[j/2] * ((j%2)? 1.0 : -1.0);
        Serial.print(j/2);
        Serial.print("\t");
        Serial.print(val);
        Serial.print("\t");
        Serial.println();
    }
*/

    Serial.println("res = (raw+shift)*scalar");
    Serial.println("New accelerometer tune:");
    printTune(newAccl);
    Serial.println("New magnetometer tune:");
    printTune(newMagn);
    Serial.println("Would you like to save these values? (y/n)");
    if(getTrueFalseResponse()){
        set.writeTuningValues(newAccl, newMagn);
    }
}

void applyTuneFromEEPROM(){
    mpu.tuneAccl(set.getAccelTune());
    cmp.tune(set.getMagTune());
}

void resetTuneParameters(){
    mpu.tuneAccl(LTATune());
    cmp.tune(LTATune());
}

void streamData(){
    Serial.print("Accel x,y,z: ");
    for(int i=0; i<3; i++){
        Serial.print(accl[i].getValue());
        Serial.print("\t");
    }
    Serial.print("Mag x,y,z: ");
    for(int i=0; i<3; i++){
        Serial.print(magn[i].getValue());
        Serial.print("\t");
    }
    Serial.println();
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
        magn[i].update(val[i]);
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
