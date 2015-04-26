#include "input/InertialManager.h"
#include "input/Sensor.h"

//HMC5883L Compass
class HMC5883L : public Sensor{
protected:
    static const uint8_t HMC_I2C_ADDR   = 0x1E;
    static const uint8_t HMC_STATUS_REG = 0x09;
    LTATune LTA;
    uint8_t address;
public:
    HMC5883L(): address(HMC_I2C_ADDR) {}
    HMC5883L(uint8_t addr): address(addr) {}
    void  init();
    void  stop();
    bool  status();
    void  calibrate();
    void  update(InertialManager& man);
    void  tune(LTATune t);
    void  rawValues(int& x, int& y, int& z);
    float getAzimuth();
};
void
HMC5883L::init(){
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
HMC5883L::stop(){

}
bool
HMC5883L::status(){
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x09);
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)0x01);
    if(Wire.available()>=1){
        uint8_t status = Wire.read();
        if((status&0x3) == 1) return STATUS_OK;
    }
    return STATUS_BAD;
}
void
HMC5883L::calibrate(){

}
void
HMC5883L::update(InertialManager& man){
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float M[3];
    LTA.calibrate<int>(m,M);
    man.updateMagField(M[0],M[1],M[2]);
}
void
HMC5883L::rawValues(int& x, int& y, int& z){
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x02);
    Wire.write((uint8_t)0x01);
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
