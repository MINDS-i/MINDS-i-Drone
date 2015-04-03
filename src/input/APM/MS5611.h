#ifndef MS5611_H
#define MS5611_H

#include "input/InertialManager.h"
#include "input/Sensor.h"
#include "util/byteConv.h"
#include <SPI.h>

//equations and spec pulled from datasheet at
//http://www.daedalus.ei.tum.de/attachments/article/61/MS5611-01BA01.pdf

class MS5611 : public Sensor{
protected:
    const static uint8_t ADDRESS         = 0x76;
    const static uint8_t ADDR_SENS_T1    = 0xA2;
    const static uint8_t ADDR_OFF_T1     = 0xA4;
    const static uint8_t ADDR_TCS        = 0xA6;
    const static uint8_t ADDR_TCO        = 0xA8;
    const static uint8_t ADDR_T_REF      = 0xAA;
    const static uint8_t ADDR_TEMPSENS   = 0xAC;
    const static uint8_t CMD_Pressure    = 0x40;
    const static uint8_t CMD_Temp        = 0x50;
    const static uint8_t RESET           = 0x1E;
    const static uint8_t ADC_READ_ADDR   = 0;
    const static uint8_t TEMP_DUTY_CYCLE = 100;

    // OSR (Over Sampling Ratio) constants and calculation milliseconds
    // 0x00, 0x02, 0x04, 0x06, 0x08
    //    1,    2,    3,    5,   10
    const static uint8_t OSR_RATIO = 0x08;
    const static uint8_t OSR_DELAY =   10;

    //calibration terms stored in ms5611 prom
    uint16_t SENS_T1;
    uint16_t OFF_T1;
    uint16_t TCS;
    uint16_t TCO;
    uint16_t T_REF;
    uint16_t TEMPSENS;

    //variables for use by program
    uint32_t readyTime;
    int32_t  dT, P;
    uint8_t  tempCycle;
    uint8_t  select;
    bool     newData;

    void     sendCommand(uint8_t command);
    uint32_t get24from(uint8_t prom_addr);
    uint16_t get16from(uint8_t prom_addr);
    void     calculateP (uint32_t D1);
    void     calculateDT(uint32_t D2);
public:
    MS5611(): select(40), newData(true) {/*Can't put IO code here on embed platforms*/}
    MS5611(uint8_t cs_pin): select(cs_pin), newData(true) {}
    void init();
    void stop();
    bool status();
    void calibrate();
    void update(InertialManager& man);
    void update();
    float getPascals();
    float getMilliBar();
    float getCelsius(); // returns celsius
    float getAltitude(); // return feet
};
void
MS5611::sendCommand(uint8_t cmd){
    digitalWrite(select, LOW);
    SPI.transfer(cmd);
    digitalWrite(select, HIGH);
}
uint32_t
MS5611::get24from(uint8_t prom_addr){
    digitalWrite(select, LOW);
    uint8_t tmp[4];
    tmp[0] = prom_addr;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    SPI.transfer(tmp, 4);
    byteConv value;
    value.bytes[0] = tmp[3];
    value.bytes[1] = tmp[2];
    value.bytes[2] = tmp[1];
    //tmp[0] is a crc checksum
    digitalWrite(select, HIGH);
    return value.l;
}
uint16_t
MS5611::get16from(uint8_t prom_addr){
    digitalWrite(select, LOW);
    uint8_t tmp[3];
    tmp[0] = prom_addr;
    tmp[1] = 0;
    tmp[2] = 0;
    SPI.transfer(tmp, 3);
    digitalWrite(select, HIGH);
    //tmp[0] is a crc checksum
    return ((uint16_t) tmp[1] << 8 ) | ( tmp[2] );
}
void
MS5611::init(){
    pinMode(select, OUTPUT);
    digitalWrite(select, HIGH);
    SPI.begin();
    sendCommand(RESET);
    readyTime = millis();
    tempCycle = TEMP_DUTY_CYCLE; //get temperature first
}
void
MS5611::stop(){
}
bool
MS5611::status(){
    return STATUS_OK;
}
void
MS5611::calibrate(){
    SENS_T1  = get16from(ADDR_SENS_T1 );
    OFF_T1   = get16from(ADDR_OFF_T1  );
    TCS      = get16from(ADDR_TCS     );
    TCO      = get16from(ADDR_TCO     );
    T_REF    = get16from(ADDR_T_REF   );
    TEMPSENS = get16from(ADDR_TEMPSENS);
}
void
MS5611::update(){
    if(millis() > readyTime){
        //get data
        uint32_t tmp = get24from(ADC_READ_ADDR);
        if(tempCycle == 0){
            calculateDT(tmp);
        } else {
            calculateP(tmp);
        }
        newData = true;

        //send new request
        if(tempCycle >= TEMP_DUTY_CYCLE){
            sendCommand(CMD_Temp + OSR_RATIO);
            tempCycle = 0;
        } else {
            sendCommand(CMD_Pressure + OSR_RATIO);
            tempCycle++;
        }

        readyTime = millis()+OSR_DELAY;
    }
}
void
MS5611::calculateP (uint32_t D1){
    if(SENS_T1 == 0){ //not calibrated successfully
        P = D1;
        return;
    }
    int64_t OFF  = ((int64_t)OFF_T1 ) * 65536 + ((int64_t)TCO) * dT / 128;
    int64_t SENS = ((int64_t)SENS_T1) * 32768 + ((int64_t)TCS) * dT / 256;
    P = (D1 * SENS / 2097152 - OFF) / 32768;
}
void
MS5611::calculateDT(uint32_t D2){
    //D2: 8289716 T_REF: 31345    dT: 8260788 297.29
    dT = D2 - (((int32_t)T_REF) * 256);
}
float
MS5611::getPascals(){
    update();
    return ((float) P);
}
float
MS5611::getMilliBar(){
    return getPascals()/100.f;
}
float
MS5611::getCelsius(){
    update();
    int32_t temp = 2000 + ((int64_t) dT * TEMPSENS) / 8388608;
    return ((float) temp)/100.f;
}
float
MS5611::getAltitude(){ // feet
    float tmp = getPascals()/(101325);
    float alt = (1-pow(tmp, 0.190284))*145366.45;
    return alt;
}
void
MS5611::update(InertialManager& man){
    update();
    if(newData) {
        man.updatePressure(P);
        newData = false;
    }
}

#endif