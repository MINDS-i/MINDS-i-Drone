#include "input/InertialManager.h"
#include "input/Sensor.h"
#include "util/LTATune.h"
constexpr auto HMC5883L_MAX_EXPECTED_VALUE_MAG = 1500;
constexpr auto HMC5883L_MIN_EXPECTED_VALUE_MAG = 50;

//HMC5883L Compass
class HMC5883L : public InertialVec {
protected:
    static const uint8_t HMC_I2C_ADDR   = 0x1E;
    static const uint8_t HMC_STATUS_REG = 0x09;
    LTATune LTA;
    uint8_t address;
    bool isTrueHMC5883L;
public:
    HMC5883L(): address(HMC_I2C_ADDR) {}
    HMC5883L(uint8_t addr): address(addr) {}
    void  begin();
    void  end();
    bool  checkGoodValues();
    Sensor::Status  status();
    void  calibrate();
    void  update(InertialManager& man, Translator axis);
    void  tune(LTATune t);
    void  rawValues(int& x, int& y, int& z);
    float getAzimuth();
};
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
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float mag = (float)(m[0] * m[0] + m[1] * m[1]);
    mag = sqrt(mag);
    if (mag >= HMC5883L_MAX_EXPECTED_VALUE_MAG || mag <= HMC5883L_MIN_EXPECTED_VALUE_MAG)
        return false;
    int zmag = abs(m[2]);
    return (zmag <= HMC5883L_MAX_EXPECTED_VALUE_MAG);
}
Sensor::Status
HMC5883L::status(){
    isTrueHMC5883L = true;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x09);
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)0x01);
    if(Wire.available()>=1){
        uint8_t status = Wire.read();
        if((status&0x3) == 1) return Sensor::OK;
    }
    isTrueHMC5883L = false;//bad return value either compass failed or is clone. Set compass as clone.
    if (checkGoodValues()) return Sensor::OK; //check if cloned compass is returning reasonable values, if so go on.
    /*#HMCFAIL HMC5883L Compass sensor failed contact or reported bad status*/
    return Sensor::BAD("HMCFAIL");
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
