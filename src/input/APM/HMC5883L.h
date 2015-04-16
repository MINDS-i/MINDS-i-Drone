#include "input/InertialManager.h"
#include "input/Sensor.h"

//HMC5883L Compass
class HMC5883L : public Sensor{
protected:
    static const uint8_t HMC_I2C_ADDR = 0x1E;
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
    return STATUS_OK;
}
void
HMC5883L::calibrate(){

}
void
HMC5883L::update(InertialManager& man){
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float M[3];
    for(int i=0; i<3; i++){
        M[i]  = m[i];
        M[i] += LTA.values.shift[i];
        M[i] *= LTA.values.scalar[i];
    }
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
        d = (uint8_t*) &y;
        d[1] = Wire.read();
        d[0] = Wire.read();
        d = (uint8_t*) &z;
        d[1] = Wire.read();
        d[0] = Wire.read();

/*        x = ((long) Wire.read() )<<8; //X msb
        x |= ((long) Wire.read() ); //X lsb
        z = ((long) Wire.read() )<<8; //Z msb
        z |= ((long) Wire.read() ); //Z lsb
        y = ((long) Wire.read() )<<8; //Y msb
        y |= ((long) Wire.read() );*/
    }
}
float
HMC5883L::getAzimuth(){
    int m[3];
    rawValues(m[0], m[1], m[2]);
    float M[3];
    for(int i=0; i<3; i++){
        M[i]  = m[i];
        M[i] += LTA.values.shift[i];
        M[i] *= LTA.values.scalar[i];
    }
    return atan2(M[0], M[1]);
}
