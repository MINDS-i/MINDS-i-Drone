#include "input/SPIcontroller.h"
//MPU6000 Accelerometer and Gyroscope on SPI

namespace{
    //this holds raw mpu data (accl x,y,z; temp; gyro x,y,z);
    union rawData{
        struct { uint8_t bytes[14]; };
        struct { int16_t accl[3], temp, gyro[3]; };
    };
}
class MPU6000 : public Sensor {
protected:
    const static uint8_t APM26_CS_PIN = 53;
    const static uint8_t DATA_ADDR    = 0x3B;
    SPIcontroller spiControl;
    bool    writeTo(uint8_t addr, uint8_t len, uint8_t* msg);
    bool    readFrom(uint8_t addr, uint8_t len, uint8_t* data);
    rawData readSensors(); //optimized for just sensor data
    LTATune acclLTA;
    float   gCal[3];
public:
    //clock speed 8E6 instead of default(4E6) makes readSensors about 50% faster
    MPU6000()
        : spiControl(APM26_CS_PIN, SPISettings(8E6, MSBFIRST, SPI_MODE0)) {}
    MPU6000(uint8_t chip_select)
        : spiControl(chip_select , SPISettings(8E6, MSBFIRST, SPI_MODE0)) {}
    void init();
    void stop();
    bool status();
    void calibrate();
    void update(InertialManager& man);
    //end of sensor interface
    void readTest();
};
rawData
MPU6000::readSensors(){
    //Note: its faster to read and ignore temp than make two transfers
    //Note: this is unrolled for efficiency
    rawData data;
    spiControl.capture();
    SPI.transfer(DATA_ADDR | 0x80); //last bit set to specify a read
    //the order in memory is accl x,y,z; temp; gyro x,y,z
    data.bytes[1]  = SPI.transfer(0); data.bytes[0]  = SPI.transfer(0);
    data.bytes[3]  = SPI.transfer(0); data.bytes[2]  = SPI.transfer(0);
    data.bytes[5]  = SPI.transfer(0); data.bytes[4]  = SPI.transfer(0);
    data.bytes[7]  = SPI.transfer(0); data.bytes[6]  = SPI.transfer(0);
    data.bytes[9]  = SPI.transfer(0); data.bytes[8]  = SPI.transfer(0);
    data.bytes[11] = SPI.transfer(0); data.bytes[10] = SPI.transfer(0);
    data.bytes[13] = SPI.transfer(0); data.bytes[12] = SPI.transfer(0);
    spiControl.release();
    return data;
}
bool
MPU6000::readFrom(uint8_t addr, uint8_t len, uint8_t* data){
    spiControl.capture();
    SPI.transfer(addr | 0x80); //last bit set to specify a read
    for(int i=0; i<len; i++) data[i] = SPI.transfer(0);
    return spiControl.release();
}
bool
MPU6000::writeTo(uint8_t addr, uint8_t len, uint8_t* msg){
    spiControl.capture();
    SPI.transfer(addr & ~0x80); //clear last bit to specify a write
    for(int i=0; i<len; i++) SPI.transfer(msg[i]);
    return spiControl.release();
}
// ---- public functions below -------------------------------------------------
void
MPU6000::init(){
}
void
MPU6000::stop(){
}
bool
MPU6000::status(){
}
void
MPU6000::calibrate(){
}
void
MPU6000::update(InertialManager& man){
}
void
MPU6000::readTest(){
/*    uint8_t address = 0x3B | 0x80;
    byte returns[24];
    for (int i = 0; i < 24; i++) returns[i] = 0;

    tic(0);
    spiControl.capture();
    for(int i=0; i<24; i++) returns[i] = SPI.transfer(address+i);
    spiControl.release();
    toc(0);

    Serial.println("\nBatch read:");
    for(int i=0; i<24; i++){
        if(i%6==0) Serial.print("\n");
        Serial.print(returns[i],HEX);
        Serial.print("\t");
    }

    delay(500);

    Serial.println("\nOld style read:");
    for(int i=0; i<24; i++){
        if(i%6==0) Serial.print("\n");
        Serial.print(MPU_SPI_read(address+i),HEX);
        Serial.print("\t");
    }*/
    tic(0);
    rawData data = readSensors();
    //rawData gyro = read6(0x43);
    toc(0);
    for (int i = 0; i < 3; i++){
        Serial.print(data.accl[i]);
        Serial.print("\t");
    }
}
