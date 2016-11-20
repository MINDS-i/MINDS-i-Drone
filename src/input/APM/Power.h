#ifndef POWER_H
#define POWER_H

#include "Arduino.h"
#include "comms/CommManager.h"

class Power{
private:
    const int INITIAL_WARN_INTERVAL = 30000; // milliseconds
    const int WARN_INTERVAL = 1000; // milliseconds
    float batteryInternalOhms = 0.038;
    float lowPassConst = 0.05;
    float voltage, amperage;
    float lowWarningVoltage = -1;
    Interval::SingleInterval warnTimer = Interval::elapsed(0);
    void update(){
        amperage = float((analogRead(66)/1024.l)*5.l*17.0f);
        float rawVolt = float((analogRead(67)/1024.l)*5.l*9.7f);
        float adjVolt = rawVolt + amperage*batteryInternalOhms;
        voltage = adjVolt * lowPassConst + voltage * (1.0-lowPassConst);
    }
public:
    Power(){
        warnTimer = Interval::elapsed(INITIAL_WARN_INTERVAL);
    }
    void setLowPassConstant(float lpc){
        lowPassConst = lpc;
    }
    void setBatteryInternalResistance(float ohms){
        batteryInternalOhms = ohms;
    }
    void setLowVolt(float low){
        lowWarningVoltage = low;
    }
    float getVoltage(){
        update();
        return voltage;
    }
    float getAmperage(){
        return amperage;
    }
    void checkCapacity(CommManager& comms){
        if(voltage < lowWarningVoltage && warnTimer()){
            warnTimer = Interval::elapsed(WARN_INTERVAL);
            /*#LOWVOLT Quadcopter battery critically low
             * quadcopter battery level fell below the low battery warning
             * voltage
            **/
            comms.sendString("LOWVOLT");
        }
    }
};

#endif
