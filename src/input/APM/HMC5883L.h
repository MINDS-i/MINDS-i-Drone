#include "input/InertialManager.h"
#include "input/Sensor.h"

//HMC5883L Compass
class HMC5883L : public Sensor{
protected:
public:
    void init();
    void stop();
    bool status();
    void calibrate();
    void update(InertialManager& man);
};
