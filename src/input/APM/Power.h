#ifndef POWER_H
#define POWER_H

#include "Arduino.h"

class Power{
private:
    float batteryInternalOhms = 0.038;
    float lowPassConst = 0.05;
    float voltage, amperage;
    void update(){
        amperage = float((analogRead(66)/1024.l)*5.l*17.0f);
        float rawVolt = float((analogRead(67)/1024.l)*5.l*9.7f);
        float adjVolt = rawVolt + amperage*batteryInternalOhms;
        voltage = adjVolt * lowPassConst + voltage * (1.0-lowPassConst);
    }
public:
    void setLowPassConstant(float lpc){
        lowPassConst = lpc;
    }
    void setBatteryInternalResistance(float ohms){
        batteryInternalOhms = ohms;
    }
    float getVoltage(){
        update();
        return voltage;
    }
    float getAmperage(){
        return amperage;
    }
};

#endif
