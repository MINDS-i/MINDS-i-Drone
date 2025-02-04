#ifndef POWER_H
#define POWER_H

#include "Arduino.h"
#include "comms/CommManager.h"

class Power {
  private:
    const int INITIAL_WARN_INTERVAL = 30000; // milliseconds
    const int WARN_INTERVAL = 1000;          // milliseconds
    float batteryInternalOhms = 0.038;
    float lowPassConst = 0.05;
    float voltage, amperage;
    float lowWarningVoltage = -1;
    Interval::SingleInterval warnTimer = Interval::elapsed(0);
    bool batteryLow;
    void update() {
        amperage = float((analogRead(66) / 1024.l) * 5.l * 17.0f);
        float rawVolt = float((analogRead(67) / 1024.l) * 5.l * 9.7f);
        float adjVolt = rawVolt + amperage * batteryInternalOhms;
        voltage = adjVolt * lowPassConst + voltage * (1.0 - lowPassConst);
    }

  public:
    Power() { warnTimer = Interval::elapsed(INITIAL_WARN_INTERVAL); }
    void setLowPassConstant(float lpc) { lowPassConst = lpc; }
    void setBatteryInternalResistance(float ohms) { batteryInternalOhms = ohms; }
    void setLowVolt(float low) { lowWarningVoltage = low; }
    float getVoltage() {
        update();
        return voltage;
    }
    float getAmperage() { return amperage; }
    void checkCapacity(CommManager& comms) {
        update();
        if (voltage < lowWarningVoltage && warnTimer()) {
            batteryLow = true;
            warnTimer = Interval::elapsed(WARN_INTERVAL);
            /*#LOWVOLT Quadcopter battery critically low
             * quadcopter battery level fell below the low battery warning
             * voltage
             **/
            comms.sendString("LOWVOLT");
        }
    }
    /**
     * Checks the battery capacity
     * Return true if the battery is considered low, false otherwise
     */
    bool isBatteryLow() { return batteryLow; }
    /**
     * When the battery is low, returns a scalar in the range (0,1) as a
     *   suggested cap on the maximum output to prevent damage to the
     *   battery
     */
    float suggestedPowerCap() {
        if (voltage > lowWarningVoltage)
            return 1.0;
        /*
        11 = .00
        12 = .33
        13 = .66
        14 = 1.0
        */
        return max(1.0 - (lowWarningVoltage - voltage) / 3.0, 0.0);
    }
};

#endif
