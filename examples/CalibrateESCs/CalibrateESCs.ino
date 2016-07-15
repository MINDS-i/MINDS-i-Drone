#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
#include "platforms/Quadcopter.h"
using namespace Platform;

void setup(){
    calibrateESCs();
    output.disable();
}
void loop(){
}
