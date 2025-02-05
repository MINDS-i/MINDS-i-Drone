#define Output_t EMaxESC

#include "MINDS-i-Drone.h"
#include "SPI.h"
#include "Wire.h"
#include "platforms/Quadcopter.h"
using namespace Platform;

void setup() {
    calibrateESCs();
    output.disable();
}
void loop() {}
