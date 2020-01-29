#include "HMC5883L.h"
#include "Wire.h"

void
HMC5883L::begin(){

    Wire.begin();
    Wire.setClock(800000L);
    delay(10);
    
    Wire.beginTransmission(address);
    Wire.write((uint8_t) 0x00);
    Wire.write((uint8_t) 0x70);
    Wire.endTransmission();
    
    Wire.beginTransmission(address);
    Wire.write((uint8_t) 0x01);
    Wire.write((uint8_t) 0x00);
    Wire.endTransmission();
}
void
HMC5883L::tune(LTATune t){
    LTA = t;
}
void
HMC5883L::end(){

}
bool
HMC5883L::checkGoodValues() {
    /*perform the original check that looks for a good status bit */
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x09);
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)0x01);
    if (Wire.available() >= 1) {
        uint8_t status = Wire.read();
        if ((status & 0x3) == 1) return true;
    }

    /*bad return value: either compass failed or is clone. */
    /*if not reporting as a good HMC5883L then check the values returned by the compass to determine if it is ok*/
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float mag = (float)(m[0] * m[0] + m[1] * m[1]);
    mag = sqrt(mag);
    bool isGood = true;
    if (mag >= HMC5883L_MAX_EXPECTED_VALUE_MAG || mag <= HMC5883L_MIN_EXPECTED_VALUE_MAG)
        isGood = false;
    int zmag = abs(m[2]);
    isGood=( isGood && (zmag <= HMC5883L_MAX_EXPECTED_VALUE_MAG));
    if (isGood) return true; //check if cloned compass is returning reasonable values, if so go on.
    /*#HMCFAIL HMC5883L Compass sensor failed contact or reported bad status*/
    return false;
}
Sensor::Status 
HMC5883L::status()
{
    /*check if compass is the original HMC5883L or a clone*/
    isTrueHMC5883L = true;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x09);
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)0x01);
    if (Wire.available() >= 1) {
        uint8_t status = Wire.read();
        if ((status & 0x3) == 1) return Sensor::OK; //always return ok here
    }

    isTrueHMC5883L = false;//bad return value either compass failed or is clone. Set compass as clone.
    /*set the compass to continous mode, this is the only mode the clone works in*/
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x02);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();

    return Sensor::OK; //always return sensor ok here, this function is called when board is powered on, good values will be checked at arming time.
}
void
HMC5883L::calibrate(){

}
void
HMC5883L::update(InertialManager& man, Translator axis){
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float M[3];
    LTA.calibrate<int>(m,M);
    man.mag = axis(M);
}
void
HMC5883L::rawValues(int& x, int& y, int& z) {
    Wire.beginTransmission(address);
    if (isTrueHMC5883L) {
        Wire.write((uint8_t)0x02);
        Wire.write((uint8_t)0x01);
    }
    else {
        Wire.write((uint8_t)0x03);
    }
    Wire.endTransmission();

    Wire.requestFrom(address, (uint8_t)0x06);
    if(Wire.available()>=6){
        uint8_t* d = (uint8_t*) &x;
        d[1] = Wire.read();
        d[0] = Wire.read();
        d = (uint8_t*) &z;
        d[1] = Wire.read();
        d[0] = Wire.read();
        d = (uint8_t*) &y;
        d[1] = Wire.read();
        d[0] = Wire.read();
    }
}
float
HMC5883L::getAzimuth(){
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float M[3];
    for(int i=0; i<3; i++){
        M[i]  = m[i];
        M[i] += LTA.shift[i];
        M[i] *= LTA.scalar[i];
    }
    return atan2(M[0], M[1]);
}
